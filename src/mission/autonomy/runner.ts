import { initPhysics } from '../../sim/world';
import { initOutdoorPhysics, OutdoorPhysics } from '../../world/outdoor/physics';
import { sampleHeight } from '../../world/outdoor/terrain';
import type { Mission } from '../types';
import type {
  LiveDetectionSource,
  MissionRunner,
  MissionWorld,
  RunnerOptions,
  RunnerSnapshot,
  RunOutcome,
  RunRecord,
  SafetyAction,
  V3,
} from './contracts';
import { requirementsMet } from './planner/confirm';
import { columnClear, cruiseZ, dist, segmentClear } from './planner/geometry';
import { landedAtHome, landingCommand, planReturn } from './planner/home';
import { floorOf } from './planner/indoor';
import type { VehicleCommand } from './vehicle';
import { IndoorPhases } from './phases/indoor';
import { DETECT_PERIOD, fmt, RANK, TRACK_PERIOD, type Waypoint } from './phases/shared';
import { DEFAULT_BATTERY } from './battery';

const USABLE_WH = DEFAULT_BATTERY.capacityWh * DEFAULT_BATTERY.usableFraction;
/**
 * The wind estimate is the true wind low-passed with this time constant, s: a flight controller
 * infers wind from the tilt it holds against it, which settles over a second or two.
 */
const WIND_ESTIMATE_TAU = 2;
/** Landing position integral gain, 1/s^2, and its cap, m/s. */
const LAND_TRIM_KI = 0.6;
const LAND_TRIM_MAX = 3;
/** Emergency landings: open-column radius, m, search radius, m, and how close over it before descending, m. */
const LAND_COLUMN_M = 0.8;
const LAND_SEARCH_M = 15;
const LAND_ALIGN_M = 0.5;
/** Height a weak airframe may lose on the way to its landing column, m. */
const LAND_SINK_M = 2;
/** Height over the range-finder ground at which a landing cuts the motors, m. */
const LAND_CUT_AGL = 0.4;

/**
 * Hierarchical mission runner. A fixed-tick phase machine over the planner
 * stages: coverage discovers buildings, each building is approached,
 * inspected, entered through the safest observed opening, explored, and
 * exited; contacts of a mission class are confirmed from several bearings;
 * the drone returns home and lands. The safety monitor is checked every
 * tick and its actions run through the `recover` phase. Same inputs give
 * the same RunRecord.
 *
 * The phases live in layers under ./phases, each extending the one before:
 * base (state and motion primitives) -> search (coverage, detection,
 * confirmation) -> entry (approach, inspection, fly-through) -> indoor
 * (exploration and the way out). This class adds the public API, the tick,
 * return and landing, and the safety ladder.
 */
class Runner extends IndoorPhases {
  // ----------------------------------------------------------- public

  get done(): boolean {
    return this.finished;
  }

  approve(): void {
    this.operatorAnswer = 'approve';
  }

  deny(): void {
    this.operatorAnswer = 'deny';
  }

  abort(): void {
    if (this.finished || this.aborted) return;
    this.aborted = true;
    this.log('warn', 'Operator abort. Returning home.');
    this.startReturn('operator abort');
  }

  step(dtSec?: number): void {
    const n = dtSec === undefined ? 1 : Math.max(1, Math.round(dtSec / this.dt));
    for (let i = 0; i < n && !this.finished; i++) this.tick();
  }

  snapshot(): RunnerSnapshot {
    const track: V3[] = this.trackRec.slice(-400).map((s) => s.pos);
    return {
      tSec: this.t,
      phase: this.phase,
      pose: this.vehicle.pose(),
      batteryPct: this.battery.remainingPct(),
      inside: this.inside,
      plan: this.job.plan(),
      track,
      contacts: this.recent.slice(-20),
      confirmed: this.confirmed.slice(),
      pendingEntry: this.pendingEntry,
      log: this.logs.slice(-50),
      done: this.finished,
    };
  }

  record(): RunRecord {
    const phases = this.phases.map((p) => ({ ...p }));
    const last = phases[phases.length - 1];
    if (last && !this.finished) last.tEnd = this.t;
    return {
      mission: this.mission,
      seed: this.world.seed,
      outcome: this.finished ? this.outcome : this.computeOutcome(),
      tSec: this.t,
      track: this.trackRec.slice(),
      phases,
      confirmed: this.confirmed.slice(),
      declaredClear: this.declaredClear.slice(),
      collisions: this.collisions.slice(),
      entries: this.entries.slice(),
      safetyEvents: [...this.monitor.events(), ...this.failsafe.events()].sort((a, b) => a.tSec - b.tSec),
      batteryStartPct: this.batteryStart,
      batteryEndPct: this.battery.remainingPct(),
      landedHome: this.landedHome,
      indoorExplorer: this.policyClean ? 'policy' : this.frontierRan ? (this.oracle ? 'oracle' : 'frontier') : null,
      log: this.logs.slice(),
      faults: this.faults ? this.faults.transitions().slice() : [],
      ...(this.sar ? { search: { pos: this.sar.history.slice(), reachable: 1 - this.sar.prior.rest, replans: this.sar.replans } } : {}),
    };
  }

  protected tick(): void {
    this.ticks += 1;
    this.t = this.ticks * this.dt;
    this.advanceFaults();

    // Navigation and the monitors work on the position the drone believes; with a GNSS fix
    // that is the truth. The camera and the airframe always act at the true position.
    this.believed(() => {
      this.checkSafety(this.vehicle.pos);
      if (!this.finished) this.checkHealth();
      if (!this.finished && this.t >= this.nextBudgetCheck) {
        this.nextBudgetCheck = this.t + 1;
        this.checkBudgets();
      }
    });
    if (this.finished) return;
    if (this.t + 1e-9 >= this.nextDetect) {
      this.nextDetect = this.t + DETECT_PERIOD;
      // No frames, no detections and no seen-empty cells (SAFETY_CASE.md H4).
      if (this.faults?.cameraUp() ?? true) this.detect();
    }
    if (this.finished) return;

    const cmd = this.believed(() => this.climbClear(this.job.tick()));
    if (this.finished) return;
    this.lastCmd = cmd;
    const wind: V3 = this.faults && !this.inside ? this.faults.wind() : (this.options.wind?.(this.t) ?? [0, 0, 0]);
    const step = this.vehicle.step(cmd, this.dt, this.limits(), { wind });
    this.lastPowerW = step.powerW;
    const k = 1 - Math.exp(-this.dt / WIND_ESTIMATE_TAU);
    for (let i = 0; i < 3; i++) this.windEstimate[i] = this.windEstimate[i]! + (wind[i]! - this.windEstimate[i]!) * k;
    this.lastCollided = step.collided;
    if (step.collided) {
      const pos = this.vehicle.pos;
      this.collisions.push({
        tSec: this.t,
        pos: [pos[0], pos[1], pos[2]],
        inside: this.inside,
      });
      this.log('warn', `Contact at ${this.where(pos)}${this.inside ? ' inside the building' : ''} during ${this.phase.replace('_', ' ')}.`);
      if (this.visit && this.phase === 'indoor_explore') this.visit.policyCollisions += 1;
    }
    this.breadcrumb();
    if (this.t + 1e-9 >= this.nextTrack) {
      this.nextTrack = this.t + TRACK_PERIOD;
      this.recordTrack();
    }
    if (!this.finished && this.t >= this.maxSim - 1e-9) {
      this.hardStopped = true;
      this.log('warn', `Time limit ${fmt(this.maxSim, 0)} s reached before landing.`);
      this.finish();
    }
  }

  protected startReturn(reason: string): void {
    if (this.finished) return;
    if (!this.returning) this.log('action', `Returning home: ${reason}.`);
    this.returning = true;
    this.suspended = null;
    this.recoverRank = 0;
    this.pendingEntry = null;
    if (this.inside) {
      this.startExit('rth');
      return;
    }
    this.startReturnOutside();
  }

  protected startReturnOutside(): void {
    this.returning = true;
    const climb = this.climbRoute();
    const from = climb.length ? climb[climb.length - 1]!.pos : this.vehicle.pos;
    const plan = planReturn(this.physics, this.air, from, this.world.home, null, this.agl);
    const wps: Waypoint[] = [...climb.map((w) => ({ ...w, tol: 0.3 })), ...plan.waypoints.map((p) => ({ pos: p, tol: 1.5 }))];
    this.visit = null;
    this.setJob(this.followJob('return_home', wps, () => this.startLand('home'), 1.5));
  }

  protected startLand(reason: string): void {
    let target: V3 = reason === 'home' ? this.world.home : [this.vehicle.pos[0], this.vehicle.pos[1], this.vehicle.pos[2]];
    if (reason !== 'home') {
      this.landingEmergency = true;
      const site = this.inside ? null : this.landingSite();
      if (site && dist(site, this.vehicle.pos) > 0.3) {
        target = site;
        this.log('warn', `Landing in place: ${reason}. Moving ${fmt(dist(site, this.vehicle.pos), 1)} m to clear ground first.`);
      } else this.log('warn', `Landing in place: ${reason}.`);
    } else {
      this.log('action', 'Over the pad. Landing.');
    }
    const v = this.visit;
    const indoorGround = this.inside && v ? v.box.baseZ + floorOf(v.grid, this.vehicle.pos[2] - v.box.baseZ) * v.grid.H + 0.1 : null;
    // Outdoors the range finder gives the ground under the drone, wherever the position estimate puts it.
    const groundNow = (): number => {
      if (indoorGround !== null) return indoorGround;
      const p = this.truePos ?? this.vehicle.pos;
      return sampleHeight(this.world.outdoor.terrain, p[0], p[1]);
    };
    let settle = 0;
    let stuck = 0;
    // Integral on the horizontal error, so a steady wind doesn't leave the drone off the pad.
    const trim: [number, number] = [0, 0];
    let touchedAt: V3 | null = null;
    this.setJob({
      phase: 'land',
      plan: () => [target],
      tick: (): VehicleCommand => {
        const pos = this.vehicle.pos;
        const g = groundNow();
        const c = landingCommand(pos, target, g);
        // Over the chosen column before descending, so the drone never sinks onto a roof or into canopy.
        const aligned = Math.hypot(target[0] - pos[0], target[1] - pos[1]) <= LAND_ALIGN_M;
        if (!aligned && !c.touched) c.vel[2] = Math.max(c.vel[2]!, 0);
        for (let i = 0; i < 2; i++) {
          trim[i] = Math.max(-LAND_TRIM_MAX, Math.min(LAND_TRIM_MAX, trim[i]! + (target[i]! - pos[i]!) * LAND_TRIM_KI * this.dt));
          c.vel[i] = c.vel[i]! + trim[i]!;
        }
        // Like a flight controller's land detector: in the last few tens of centimetres over measured ground,
        // cut the motors and drop, rather than hover in ground effect where the wind can slide the drone.
        if (aligned && pos[2] - g <= LAND_CUT_AGL) {
          this.touchdown([pos[0], pos[1], pos[2]]);
          return { vel: [0, 0, 0] };
        }
        // Resting on something above the expected ground also counts.
        stuck = aligned && this.vehicle.vel[2] > -0.03 && !c.touched ? stuck + this.dt : 0;
        if (c.touched || stuck > 1.5) {
          // Where it first touched: on the ground the motors idle and the wind no longer moves it.
          touchedAt ??= [pos[0], pos[1], pos[2]];
          settle += this.dt;
          if (settle > 0.5) this.touchdown(touchedAt);
        }
        return { vel: c.vel };
      },
    });
  }

  /**
   * Nearest point at the current height, within LAND_SEARCH_M, with an open column down to the
   * ground and a clear straight line to it. Null when there is none (land straight down then).
   */
  protected landingSite(): V3 | null {
    const pos = this.vehicle.pos;
    const terrain = this.world.outdoor.terrain;
    const open = (x: number, y: number): boolean => columnClear(this.physics, x, y, sampleHeight(terrain, x, y) + 0.2, pos[2], LAND_COLUMN_M);
    if (open(pos[0], pos[1])) return [pos[0], pos[1], pos[2]];
    for (let r = 1; r <= LAND_SEARCH_M; r += 1) {
      const n = Math.max(8, Math.round(r * 4));
      for (let k = 0; k < n; k++) {
        const a = (k / n) * Math.PI * 2;
        const p: V3 = [pos[0] + Math.cos(a) * r, pos[1] + Math.sin(a) * r, pos[2]];
        // A drone that can't hold height sinks on the way over, so the leg must also be clear lower down.
        const low = (q: V3): V3 => [q[0], q[1], q[2] - LAND_SINK_M];
        if (open(p[0], p[1]) && segmentClear(this.physics, pos, p) && segmentClear(this.physics, low(pos), low(p))) return p;
      }
    }
    return null;
  }

  protected touchdown(at: V3 = this.vehicle.pos): void {
    if (this.finished) return;
    this.landedHome = !this.inside && landedAtHome(at, this.world.home);
    this.log('action', this.landedHome ? 'Landed at home.' : `Landed away from home at ${this.where(at)}.`);
    this.finish();
  }

  protected finish(): void {
    this.policyRun?.release();
    if (this.finished) return;
    this.outcome = this.computeOutcome();
    this.setPhase('done');
    const last = this.phases[this.phases.length - 1];
    if (last) last.tEnd = this.t;
    this.recordTrack();
    this.finished = true;
    this.log(
      'info',
      `Mission ${this.outcome}. ${fmt(this.t, 0)} s, battery ${fmt(this.battery.remainingPct(), 0)}% left, ${this.collisions.length} collision(s).`,
    );
    this.job = this.holdJob('done', 1e9, () => undefined);
  }

  protected computeOutcome(): RunOutcome {
    if (this.aborted || (this.landingEmergency && !this.landedHome)) return 'aborted';
    const searchedAll = this.coverageDone && this.pending.length === 0;
    let out: RunOutcome;
    if (this.findMission()) {
      out = requirementsMet(this.mission, this.confirmed) ? 'found' : searchedAll ? 'not_found' : 'incomplete';
    } else if (this.mission.missionType === 'safety_sweep') {
      if (this.confirmed.some((c) => c.classId === 'person')) out = 'found';
      else {
        const scopes = new Set(this.declaredClear.map((d) => d.scope));
        // Building scopes decide a sweep. The outdoor scope rarely completes near the fence,
        // so it is declared when the source says clear but is not required.
        const all = this.discovered.size > 0 && [...this.discovered].every((id) => scopes.has(id));
        out = searchedAll && all ? 'clear' : 'incomplete';
      }
    } else {
      out = 'landed';
    }
    if (this.hardStopped && out !== 'found' && out !== 'clear') return 'incomplete';
    return out;
  }

  // ----------------------------------------------------------- safety

  protected distanceHome(pos: V3): number {
    const home = this.world.home;
    if (this.inside && this.visit?.opening) {
      const c = this.visit.opening.center as V3;
      return dist(pos, c) + Math.hypot(c[0] - home[0], c[1] - home[1]);
    }
    return Math.hypot(pos[0] - home[0], pos[1] - home[1]);
  }

  protected checkBudgets(): void {
    if (this.returning || this.finished || this.phase === 'preflight') return;
    const pos = this.vehicle.pos;
    const d = this.distanceHome(pos);
    const climb = Math.max(0, cruiseZ(this.air, pos[0], pos[1], this.agl) - pos[2]);
    const timeHome = d / 4 + climb / 1.2 + (this.inside ? 60 : 0) + 30;
    if (this.t + timeHome * 1.2 + 10 >= this.maxSim) {
      this.startReturn(`time budget (${fmt(this.maxSim - this.t, 0)} s left)`);
      return;
    }
    const need = this.battery.costPct(d, climb) * 1.3 * (this.health?.costScale ?? 1) + (this.inside ? 3 : 0) + 3;
    if (this.battery.usedPct() + need >= this.mission.budgets.batteryPct) {
      this.startReturn(`battery budget (${fmt(this.battery.usedPct(), 0)}% used of ${fmt(this.mission.budgets.batteryPct, 0)}%)`);
    }
  }

  // ----------------------------------------------------------- faults

  /** Advance injected faults: motor thrust, the operator link. */
  protected advanceFaults(): void {
    const f = this.faults;
    if (!f) return;
    f.advance(this.t, this.dt, this.phase);
    this.vehicle.setThrustAuthority(f.motorEfficiency());
    const down = !f.linkUp();
    if (down !== this.linkDown) this.log(down ? 'warn' : 'info', down ? 'Operator link down.' : 'Operator link back.');
    this.linkDown = down;
  }

  /** Run `fn` with the vehicle at the position the drone believes it is at. */
  /** The true position while believed() runs on a GNSS estimate; the range finder measures from here. */
  private truePos: V3 | null = null;

  protected believed<T>(fn: () => T): T {
    const gnss = this.faults && !this.inside ? this.faults.gnss() : null;
    if (!gnss || gnss.fix) return fn();
    const truth = this.vehicle.pos;
    const e = gnss.errorM;
    this.vehicle.pos = [truth[0] + e[0], truth[1] + e[1], truth[2] + e[2]];
    this.truePos = truth;
    try {
      return fn();
    } finally {
      this.vehicle.pos = truth;
      this.truePos = null;
    }
  }

  /**
   * GNSS lost outdoors, before the failsafe sends the drone home: the horizontal position is only a
   * dead-reckoning guess, so climb to cruise height (height comes from the barometer and range
   * finder, which still work) and back away from the nearest building, instead of flying the plan.
   */
  protected climbClear(cmd: VehicleCommand): VehicleCommand {
    const flags = this.health?.flags;
    // Without a fix, hold position by climbing rather than hovering where drift can carry the drone into a wall.
    // In strong wind, get above the roofline before the trip home, since the gusts push the drone around near walls.
    const gnss = flags?.has('gnss_lost') && !this.returning;
    if (!(gnss || flags?.has('wind_severe')) || this.inside || this.phase === 'land' || this.phase === 'done') return cmd;
    const pos = this.vehicle.pos;
    const vz = Math.max(0, Math.min(1.2, cruiseZ(this.air, pos[0], pos[1], this.agl) - pos[2]));
    let away: [number, number] = [0, 0];
    let nearest = Infinity;
    for (const b of this.world.buildings) {
      const ring = b.footprint;
      const cx = ring.reduce((a, p) => a + p.x, 0) / ring.length;
      const cy = ring.reduce((a, p) => a + p.y, 0) / ring.length;
      const d = Math.hypot(pos[0] - cx, pos[1] - cy);
      if (d < nearest && d > 1e-6) {
        nearest = d;
        away = [(pos[0] - cx) / d, (pos[1] - cy) / d];
      }
    }
    // Only near a building, and only until the drone is above the roofline.
    if (!gnss && vz <= 0.05) return cmd;
    const near = nearest < 15 && vz > 0.05;
    return { vel: [near ? away[0] * 0.5 : 0, near ? away[1] * 0.5 : 0, vz], yaw: null };
  }

  /** ESC telemetry: achieved over commanded thrust per motor, with a little measurement noise. */
  protected motorTelemetry(): number[] {
    const eff = this.faults?.motorEfficiency() ?? [1, 1, 1, 1];
    return eff.map((e) => Math.min(1.05, e + this.escNoise.gaussian() * 0.01));
  }

  protected checkHealth(): void {
    const pos = this.vehicle.pos;
    const home = this.world.home;
    const dh = Math.hypot(home[0] - pos[0], home[1] - pos[1]);
    const climb = Math.max(0, cruiseZ(this.air, pos[0], pos[1], this.agl) - pos[2]);
    const seen = this.failsafe.events().length;
    const verdict = this.failsafe.check({
      tSec: this.t,
      dt: this.dt,
      phase: this.phase,
      inside: this.inside,
      linkUp: !this.linkDown,
      cameraFrame: this.faults?.cameraUp() ?? true,
      lidar: this.lastRing,
      lidarExpected: this.lastExpected,
      gnssFix: this.faults ? this.faults.gnss().fix : true,
      poseHealth: null,
      motorRatio: this.motorTelemetry(),
      batteryPct: this.battery.remainingPct(),
      modelPowerW: this.lastPowerW,
      usableWh: USABLE_WH,
      costHomePct: this.battery.costPct(this.distanceHome(pos), climb),
      budgetPct: this.mission.budgets.batteryPct,
      windEstimate: this.inside ? null : this.windEstimate,
      homeDistM: dh,
      toHome: dh > 1e-6 ? [(home[0] - pos[0]) / dh, (home[1] - pos[1]) / dh] : [1, 0],
      cruiseMps: DEFAULT_BATTERY.cruiseMps,
    });
    this.lastRing = null;
    this.lastExpected = null;
    this.health = verdict;
    for (const e of this.failsafe.events().slice(seen)) this.log(e.action === 'none' ? 'info' : 'warn', e.message);
    if (verdict.action.kind !== 'none') this.handleSafety(verdict.action);
  }

  protected checkSafety(pos: V3): void {
    const look = 0.5;
    const predicted: V3 = [pos[0] + this.lastCmd.vel[0] * look, pos[1] + this.lastCmd.vel[1] * look, pos[2] + this.lastCmd.vel[2] * look];
    const action = this.monitor.check({
      tSec: this.t,
      pose: this.vehicle.pose(),
      predicted,
      battery: this.battery,
      distanceHomeM: this.distanceHome(pos),
      collided: this.lastCollided,
      inside: this.inside,
      phase: this.phase,
    });
    this.lastCollided = false;
    if (action.kind !== 'none') this.handleSafety(action);
  }

  protected handleSafety(action: SafetyAction): void {
    const rank = RANK[action.kind];
    if (action.kind === 'land') {
      if (this.phase !== 'land') {
        this.log('warn', `Safety: ${action.reason}.`);
        this.suspended = null;
        this.startLand(action.reason);
      }
      return;
    }
    if (action.kind === 'exit_building' && this.inside && this.phase === 'exit') {
      if (!this.returning) this.log('warn', `Safety: ${action.reason}. Returning home after the exit.`);
      this.returning = true;
      return;
    }
    if (action.kind === 'return_home' || action.kind === 'exit_building') {
      if (this.returning) return;
      this.log('warn', `Safety: ${action.reason}.`);
      this.startReturn(`safety (${action.reason})`);
      return;
    }
    // hold and backtrack are latched in the recover phase.
    if (this.phase === 'recover' && rank <= this.recoverRank) return;
    if (this.phase === 'land' || this.phase === 'done') return;
    const resume = this.suspended ?? this.job;
    this.suspended = resume;
    this.recoverRank = rank;
    this.log('warn', `Safety ${action.kind}: ${action.reason}.`);
    const back = () => {
      const job = this.suspended;
      this.suspended = null;
      this.recoverRank = 0;
      if (job) this.setJob(job);
    };
    if (action.kind === 'hold') {
      this.setJob(this.holdJob('recover', action.seconds ?? 1, back));
      return;
    }
    // Backtrack: retrace the breadcrumb trail.
    const meters = action.meters ?? 1.5;
    const pos = this.vehicle.pos;
    const trail: Waypoint[] = [];
    let acc = 0;
    let prev: V3 = pos;
    for (let i = this.crumbs.length - 1; i >= 0 && acc < meters; i--) {
      const c = this.crumbs[i]!;
      const d = dist(prev, c);
      if (d < 0.05) continue;
      acc += d;
      trail.push({ pos: c, yaw: this.vehicle.yaw });
      prev = c;
    }
    this.setJob(this.followJob('recover', trail, back, 0.15));
  }
}

/** Build a runner. Rapier is initialised here, so it is async. */
export async function createMissionRunner(
  mission: Mission,
  world: MissionWorld,
  source: LiveDetectionSource,
  options: RunnerOptions = {},
): Promise<MissionRunner> {
  await initOutdoorPhysics();
  if (options.policy && options.indoor !== 'frontier') await initPhysics();
  const physics = new OutdoorPhysics(world.outdoor);
  return new Runner(mission, world, source, options, physics);
}

/** Run a mission headless to completion. */
export async function runMission(
  mission: Mission,
  world: MissionWorld,
  source: LiveDetectionSource,
  options: RunnerOptions = {},
): Promise<RunRecord> {
  const runner = await createMissionRunner(mission, world, source, options);
  const dt = options.dt ?? 0.05;
  const limit = Math.ceil(((options.maxSimSec ?? mission.budgets.timeSec) + 5) / dt);
  for (let i = 0; i < limit && !runner.done; i++) runner.step();
  return runner.record();
}

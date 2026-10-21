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
import { cruiseZ, dist } from './planner/geometry';
import { landedAtHome, landingCommand, planReturn } from './planner/home';
import { floorOf } from './planner/indoor';
import type { VehicleCommand } from './vehicle';
import { IndoorPhases } from './phases/indoor';
import { DETECT_PERIOD, fmt, RANK, TRACK_PERIOD, type Waypoint } from './phases/shared';

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
      safetyEvents: this.monitor.events().slice(),
      batteryStartPct: this.batteryStart,
      batteryEndPct: this.battery.remainingPct(),
      landedHome: this.landedHome,
      indoorExplorer: this.policyClean ? 'policy' : this.frontierRan ? (this.oracle ? 'oracle' : 'frontier') : null,
      log: this.logs.slice(),
    };
  }

  protected tick(): void {
    this.ticks += 1;
    this.t = this.ticks * this.dt;
    const pose = this.vehicle.pose();

    this.checkSafety(pose.pos);
    if (this.finished) return;
    if (this.t >= this.nextBudgetCheck) {
      this.nextBudgetCheck = this.t + 1;
      this.checkBudgets();
    }
    if (this.t + 1e-9 >= this.nextDetect) {
      this.nextDetect = this.t + DETECT_PERIOD;
      this.detect();
    }
    if (this.finished) return;

    const cmd = this.job.tick();
    if (this.finished) return;
    this.lastCmd = cmd;
    const step = this.vehicle.step(cmd, this.dt, this.limits(), { wind: this.options.wind?.(this.t) });
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
    const target: V3 = reason === 'home' ? this.world.home : [this.vehicle.pos[0], this.vehicle.pos[1], this.vehicle.pos[2]];
    if (reason !== 'home') {
      this.landingEmergency = true;
      this.log('warn', `Landing in place: ${reason}.`);
    } else {
      this.log('action', 'Over the pad. Landing.');
    }
    const v = this.visit;
    const ground =
      this.inside && v
        ? v.box.baseZ + floorOf(v.grid, this.vehicle.pos[2] - v.box.baseZ) * v.grid.H + 0.1
        : sampleHeight(this.world.outdoor.terrain, target[0], target[1]);
    let settle = 0;
    let stuck = 0;
    this.setJob({
      phase: 'land',
      plan: () => [target],
      tick: (): VehicleCommand => {
        const pos = this.vehicle.pos;
        const c = landingCommand(pos, target, ground);
        // Resting on something above the expected ground also counts.
        stuck = this.vehicle.vel[2] > -0.03 && !c.touched ? stuck + this.dt : 0;
        if (c.touched || stuck > 1.5) {
          settle += this.dt;
          if (settle > 0.5) this.touchdown();
        }
        return { vel: c.vel };
      },
    });
  }

  protected touchdown(): void {
    if (this.finished) return;
    this.landedHome = !this.inside && landedAtHome(this.vehicle.pos, this.world.home);
    this.log('action', this.landedHome ? 'Landed at home.' : `Landed away from home at ${this.where(this.vehicle.pos)}.`);
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
    const need = this.battery.costPct(d, climb) * 1.3 + (this.inside ? 3 : 0) + 3;
    if (this.battery.usedPct() + need >= this.mission.budgets.batteryPct) {
      this.startReturn(`battery budget (${fmt(this.battery.usedPct(), 0)}% used of ${fmt(this.mission.budgets.batteryPct, 0)}%)`);
    }
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

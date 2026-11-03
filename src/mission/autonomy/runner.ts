import { initPhysics } from '../../sim/world';
import { initOutdoorPhysics, OutdoorPhysics } from '../../world/outdoor/physics';
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
  ClearanceRecord,
  MissionBuilding,
} from './contracts';
import { requirementsMet } from './planner/confirm';
import { cruiseZ, dist, segmentClear } from './planner/geometry';
import type { VehicleCommand } from './vehicle';
import { MissionCore } from './phases/core';
import { EntryPhase } from './phases/entry';
import { HomePhase } from './phases/home';
import { IndoorPhase } from './phases/indoor';
import { Pilot } from './phases/pilot';
import { SearchPhase } from './phases/search';
import { centerDist, DETECT_PERIOD, fmt, RANK, TRACK_PERIOD, type Waypoint } from './phases/shared';
import { DEFAULT_BATTERY } from './battery';
import type { HealthInput } from './failsafe';
import { CRASH_SPEED } from '../../shared/constants';

const USABLE_WH = DEFAULT_BATTERY.capacityWh * DEFAULT_BATTERY.usableFraction;
/**
 * The wind estimate is the true wind low-passed with this time constant, s: a flight controller
 * infers wind from the tilt it holds against it, which settles over a second or two.
 */
const WIND_ESTIMATE_TAU = 2;
/**
 * Battery swaps. The airframe (shared/airframe.ts) flies about 17 minutes on a pack, so a long sweep
 * lands at home on its battery budget, takes a fresh pack and flies on: SPARE_PACKS spares, SWAP_SEC
 * on the pad for each, and only when at least SWAP_MIN_FLIGHT_SEC of the time budget is left after it.
 */
const SPARE_PACKS = 2;
const SWAP_SEC = 60;
const SWAP_MIN_FLIGHT_SEC = 120;
/**
 * Usable % of a pack one building search is assumed to take before any has been flown on this
 * mission; after that, the most any visit took. A building search that would not fit in what is
 * left of the pack starts on a fresh one instead, so it is not cut short.
 */
const VISIT_PCT_DEFAULT = 40;
/** Indoor exit pricing for the battery budget (exitPct): average speed out, m/s, and the crossing, s. */
const EXIT_MPS = 0.5;
const EXIT_EXTRA_SEC = 60;
/** Climbing clear in severe wind ends this close under cruise height, m. */
const CLIMB_CLEAR_TOL_M = 0.5;
/** ...and does not apply this close to home, m. */
const CLIMB_CLEAR_HOME_M = 8;
/** A drone that cannot re-acquire its pose for this long lands where it is. */
const LOC_LOST_LAND_SEC = 20;

/**
 * Hierarchical mission runner. A fixed-tick phase machine over the planner
 * stages: coverage discovers buildings, each building is approached,
 * inspected, entered through the safest observed opening, explored, and
 * exited; contacts of a mission class are confirmed from several bearings;
 * the drone returns home and lands. The safety monitor is checked every
 * tick and its actions run through the `recover` phase. Same inputs give
 * the same RunRecord.
 *
 * The runner is the MissionRunner facade over a few parts in ./phases: the
 * shared state (core.ts MissionCore), the motion primitives (pilot.ts), and
 * one object per phase that owns its own state: search (coverage, detection,
 * confirmation, confirm-clear), entry (approach, inspection, fly-through),
 * indoor (exploration and the way out) and home (return and landing). A phase
 * hands over only through the exits its constructor is given, which this
 * class wires below. The runner itself keeps the tick, the safety ladder and
 * the record.
 */
class Runner implements MissionRunner {
  private readonly c: MissionCore;
  private readonly pilot: Pilot;
  private readonly search: SearchPhase;
  private readonly entry: EntryPhase;
  private readonly indoor: IndoorPhase;
  private readonly home: HomePhase;
  private locLostSince = -1;
  private closest: ClearanceRecord | null = null;
  private gnssPadHoldLogged = false;
  /** Pack energy used, usable %, when the current building visit started; and what each visit took. */
  private visitFromPct: number | null = null;
  private readonly visitPcts: number[] = [];

  constructor(mission: Mission, world: MissionWorld, source: LiveDetectionSource, options: RunnerOptions, physics: OutdoorPhysics) {
    const c = new MissionCore(mission, world, source, options, physics);
    this.c = c;
    const pilot = new Pilot(c);
    this.pilot = pilot;
    this.search = new SearchPhase(c, pilot, {
      startBuilding: (b) => this.beginBuilding(b),
      startReturn: (reason) => this.home.startReturn(reason),
      resumeIndoor: () => this.indoor.resumeIndoor(),
    });
    this.entry = new EntryPhase(c, pilot, {
      continueMission: () => this.search.continueMission(),
      leaveBuilding: () => this.indoor.leaveBuilding(),
      startExplore: () => this.indoor.startExplore(),
    });
    this.indoor = new IndoorPhase(c, pilot, {
      lastLook: (id) => this.search.lastLook(id),
      declareIfClear: (scope) => this.search.declareIfClear(scope),
      continueMission: () => this.search.continueMission(),
      startReturnOutside: () => this.home.startReturnOutside(),
      startLand: (reason) => this.home.startLand(reason),
    });
    this.home = new HomePhase(c, pilot, {
      startExit: (after) => this.indoor.startExit(after),
      finish: () => (this.c.swapping && this.c.landedHome ? this.swapBattery() : this.finish()),
    });
    c.job = pilot.holdJob('preflight', 1, () => this.search.startCoverage());
    c.recordTrack();
  }

  // ----------------------------------------------------------- public

  get done(): boolean {
    return this.c.finished;
  }

  approve(): void {
    this.entry.answer('approve');
  }

  deny(): void {
    this.entry.answer('deny');
  }

  abort(): void {
    if (this.c.finished || this.c.aborted) return;
    this.c.aborted = true;
    this.c.log('warn', 'Operator abort. Returning home.');
    this.home.startReturn('operator abort');
  }

  step(dtSec?: number): void {
    const n = dtSec === undefined ? 1 : Math.max(1, Math.round(dtSec / this.c.dt));
    for (let i = 0; i < n && !this.c.finished; i++) this.tick();
  }

  snapshot(): RunnerSnapshot {
    const track: V3[] = this.c.trackRec.slice(-400).map((s) => s.pos);
    return {
      tSec: this.c.t,
      phase: this.c.phase,
      pose: this.c.vehicle.pose(),
      batteryPct: this.c.battery.remainingPct(),
      inside: this.c.inside,
      plan: this.c.job.plan(),
      track,
      contacts: this.c.recent.slice(-20),
      confirmed: this.c.confirmed.slice(),
      pendingEntry: this.c.pendingEntry,
      log: this.c.logs.slice(-50),
      done: this.c.finished,
    };
  }

  record(): RunRecord {
    const phases = this.c.phases.map((p) => ({ ...p }));
    const last = phases[phases.length - 1];
    if (last && !this.c.finished) last.tEnd = this.c.t;
    return {
      mission: this.c.mission,
      seed: this.c.world.seed,
      outcome: this.c.finished ? this.c.outcome : this.computeOutcome(),
      tSec: this.c.t,
      track: this.c.trackRec.slice(),
      phases,
      confirmed: this.c.confirmed.slice(),
      unresolved: this.c.tracker.tracks.filter((t) => t.status === 'unresolved').map((t) => this.c.tracker.toConfirmed(t, t.lastSec)),
      declaredClear: this.c.declaredClear.slice(),
      collisions: this.c.collisions.slice(),
      entries: this.c.entries.slice(),
      safetyEvents: [...this.c.monitor.events(), ...this.c.failsafe.events()].sort((a, b) => a.tSec - b.tSec),
      batteryStartPct: this.c.batteryStart,
      batteryEndPct: this.c.battery.remainingPct(),
      ...(this.c.batterySwaps ? { batterySwaps: this.c.batterySwaps, swappedUsedPct: this.c.swappedUsedPct } : {}),
      landedHome: this.c.landedHome,
      indoorExplorer: this.indoor.policyClean ? 'policy' : this.indoor.frontierRan ? (this.c.oracle ? 'oracle' : 'frontier') : null,
      localization: [...this.c.locVisits, ...(this.c.loc && this.c.loc.flownTicks > 0 ? [this.c.loc.record(this.c.visit?.b.id ?? '')] : [])],
      log: this.c.logs.slice(),
      faults: this.c.faults ? this.c.faults.transitions().slice() : [],
      ...(this.closest ? { clearance: { ...this.closest } } : {}),
      ...(this.c.sar ? { search: { pos: this.c.sar.history.slice(), reachable: 1 - this.c.sar.prior.rest, replans: this.c.sar.replans } } : {}),
    };
  }

  private tick(): void {
    if (this.visitFromPct !== null && !this.c.visit) {
      this.visitPcts.push(this.energyUsedPct() - this.visitFromPct);
      this.visitFromPct = null;
    }
    this.c.ticks += 1;
    this.c.t = this.c.ticks * this.c.dt;
    this.advanceFaults();

    this.c.stepGnss();
    // Navigation and the monitors work on the position the drone believes: outdoors the GNSS
    // reading, indoors the pose estimate (or the true pose in the reference arms). The camera and
    // the airframe always act at the true position.
    this.c.believed(() => {
      this.checkSafety(this.c.vehicle.pos);
      if (!this.c.finished) this.checkHealth();
      if (!this.c.finished && this.c.t >= this.c.nextBudgetCheck) {
        this.c.nextBudgetCheck = this.c.t + 1;
        this.checkBudgets();
      }
    });
    if (this.c.finished) return;
    if (this.c.t + 1e-9 >= this.c.nextDetect) {
      this.c.nextDetect = this.c.t + DETECT_PERIOD;
      // No frames, no detections and no seen-empty cells (SAFETY_CASE.md H4).
      if (this.c.faults?.cameraUp() ?? true) {
        // The camera renders from where the drone really is; what it saw is placed where the drone believes it is.
        const sensed = this.c.vehicle.pose();
        this.c.believed(() => this.search.detect(sensed));
      }
    }
    if (this.c.finished) return;

    let cmd = this.c.believed(() => this.climbClear(this.c.job.tick()));
    if (this.c.finished) return;
    if (this.c.loc) cmd = this.flyOnEstimate(cmd);
    this.c.lastCmd = cmd;
    const wind: V3 = this.c.faults && !this.c.inside ? this.c.faults.wind() : (this.c.options.wind?.(this.c.t) ?? [0, 0, 0]);
    const step = this.c.vehicle.step(cmd, this.c.dt, this.c.limits(), { wind });
    this.c.lastPowerW = step.powerW;
    const k = 1 - Math.exp(-this.c.dt / WIND_ESTIMATE_TAU);
    for (let i = 0; i < 3; i++) this.c.windEstimate[i] = this.c.windEstimate[i]! + (wind[i]! - this.c.windEstimate[i]!) * k;
    this.c.lastCollided = step.collided;
    // Sensor models read the airframe's true state once it has moved; the planners read their output next tick.
    if (this.c.loc) this.c.loc.step(this.c.localTruth(), this.c.t);
    this.c.aperture?.sense(this.c.vehicle.pos, this.c.vehicle.vel, this.c.vehicle.yaw, this.c.dt);
    if (step.collided) {
      const pos = this.c.vehicle.pos;
      const crash = step.impactSpeed > CRASH_SPEED;
      this.c.collisions.push({
        tSec: this.c.t,
        pos: [pos[0], pos[1], pos[2]],
        inside: this.c.inside,
        speed: Math.round(step.impactSpeed * 100) / 100,
        ...(crash ? { crash: true } : {}),
      });
      const where = `${this.c.where(pos)}${this.c.inside ? ' inside the building' : ''} during ${this.c.phase.replace('_', ' ')}`;
      if (crash) {
        // The crash rule: the airframe is down where it hit, so the flight ends here.
        this.c.log('warn', `Crash at ${where}: hit at ${fmt(step.impactSpeed, 1)} m/s, over the ${fmt(CRASH_SPEED, 1)} m/s crash limit.`);
        this.c.crashed = true;
        this.finish();
        return;
      }
      this.c.log('warn', `Contact at ${where}, ${fmt(step.impactSpeed, 1)} m/s.`);
      if (this.c.visit && this.c.phase === 'indoor_explore') this.c.visit.policyCollisions += 1;
    }
    if (this.c.options.measureClearance) this.measureClearance();
    this.pilot.breadcrumb();
    if (this.c.t + 1e-9 >= this.c.nextTrack) {
      this.c.nextTrack = this.c.t + TRACK_PERIOD;
      this.c.recordTrack();
    }
    if (!this.c.finished && this.c.t >= this.c.maxSim - 1e-9) {
      this.c.hardStopped = true;
      this.c.log('warn', `Time limit ${fmt(this.c.maxSim, 0)} s reached before landing.`);
      this.finish();
    }
  }

  private measureClearance(): void {
    const pos = this.c.vehicle.pos;
    const c = this.c.physics.clearance([pos[0], pos[1], pos[2]]);
    if (!c || (this.closest && c.gapM >= this.closest.minM)) return;
    this.closest = { minM: c.gapM, tSec: this.c.t, pos: [pos[0], pos[1], pos[2]], phase: this.c.phase, inside: this.c.inside, tag: c.tag };
  }

  /**
   * A command planned in the estimate's frame, as the flight controller flies
   * it: velocities and headings are body-relative, so they turn by the
   * heading error on the way to the real world. While the estimate is lost
   * the drone hovers so the matcher can re-acquire.
   */
  /**
   * The estimator's health as the failsafe reads it. A lost estimate counts
   * only once the re-acquire hover has run LOC_LOST_LAND_SEC: then the
   * failsafe lands in place (estimator_fault).
   */
  private poseHealth(): HealthInput['poseHealth'] {
    if (!this.c.loc || !this.c.inside) return null;
    const h = this.c.loc.estimate.health;
    if (h.state !== 'lost') return h;
    return this.locLostSince >= 0 && this.c.t - this.locLostSince > LOC_LOST_LAND_SEC ? h : null;
  }

  private flyOnEstimate(cmd: VehicleCommand): VehicleCommand {
    const l = this.c.loc!;
    if (this.c.phase === 'land') return cmd;
    if (l.lost) {
      if (this.locLostSince < 0) {
        this.locLostSince = this.c.t;
        this.c.log('warn', 'Pose estimate lost. Hovering while the scan matcher re-acquires.');
      }
      return { vel: [0, 0, 0] };
    }
    if (this.locLostSince >= 0) this.c.log('info', `Pose estimate re-acquired after ${fmt(this.c.t - this.locLostSince)} s.`);
    this.locLostSince = -1;
    const d = l.yawError;
    const c = Math.cos(d);
    const s = Math.sin(d);
    const [vx, vy, vz] = cmd.vel;
    return {
      ...cmd,
      vel: [c * vx - s * vy, s * vx + c * vy, vz],
      yaw: cmd.yaw === undefined || cmd.yaw === null ? cmd.yaw : cmd.yaw + d,
    };
  }

  private finish(): void {
    this.indoor.release();
    if (this.c.finished) return;
    this.c.outcome = this.computeOutcome();
    this.c.setPhase('done');
    const last = this.c.phases[this.c.phases.length - 1];
    if (last) last.tEnd = this.c.t;
    this.c.recordTrack();
    this.c.finished = true;
    this.c.log(
      'info',
      `Mission ${this.c.outcome}. ${fmt(this.c.t, 0)} s, battery ${fmt(this.c.battery.remainingPct(), 0)}% left, ${this.c.collisions.length} collision(s).`,
    );
    this.c.job = this.pilot.holdJob('done', 1e9, () => undefined);
  }

  private computeOutcome(): RunOutcome {
    if (this.c.crashed) return 'crashed';
    if (this.c.aborted || (this.c.landingEmergency && !this.c.landedHome)) return 'aborted';
    const searchedAll = this.search.coverageDone && this.search.pending.length === 0;
    let out: RunOutcome;
    if (this.search.findMission()) {
      out = requirementsMet(this.c.mission, this.c.confirmed) ? 'found' : searchedAll ? 'not_found' : 'incomplete';
    } else if (this.c.mission.missionType === 'safety_sweep') {
      if (this.c.confirmed.some((c) => c.classId === 'person')) out = 'found';
      else {
        const scopes = new Set(this.c.declaredClear.map((d) => d.scope));
        // Building scopes decide a sweep. The outdoor scope rarely completes near the fence,
        // so it is declared when the source says clear but is not required.
        const all = this.search.discovered.size > 0 && [...this.search.discovered].every((id) => scopes.has(id));
        out = searchedAll && all ? 'clear' : 'incomplete';
      }
    } else {
      out = 'landed';
    }
    if (this.c.hardStopped && out !== 'found' && out !== 'clear') return 'incomplete';
    return out;
  }

  // ----------------------------------------------------------- safety

  private distanceHome(pos: V3): number {
    const home = this.c.world.home;
    if (this.c.inside && this.c.visit?.opening) {
      const c = this.c.visit.opening.center as V3;
      return dist(pos, c) + Math.hypot(c[0] - home[0], c[1] - home[1]);
    }
    return Math.hypot(pos[0] - home[0], pos[1] - home[1]);
  }

  /** Rough flight time home from here, s: out of the building, up to cruise and across. */
  private timeHome(): number {
    const pos = this.c.vehicle.pos;
    const climb = Math.max(0, cruiseZ(this.c.air, pos[0], pos[1], this.c.agl) - pos[2]);
    return this.distanceHome(pos) / 4 + climb / 1.2 + (this.c.inside ? 60 : 0) + 30;
  }

  /**
   * Energy to get out of the building the drone is in, usable %: the way back to the opening at
   * EXIT_MPS, both ways round obstacles, plus EXIT_EXTRA_SEC for the crossing, all near hover power.
   * costPct prices cruise flight, which indoors is far too cheap; on the small pack an exit from deep
   * inside a building can take a tenth of it.
   */
  private exitPct(): number {
    const opening = this.c.visit?.opening?.center as V3 | undefined;
    if (!this.c.inside || !opening) return 0;
    const sec = (dist(this.c.vehicle.pos, opening) * 2) / EXIT_MPS + EXIT_EXTRA_SEC;
    const hoverW = this.c.battery.powerFor([0, 0, 0], [0, 0, 0]);
    return Math.max(3, ((hoverW * sec) / 3600 / USABLE_WH) * 100);
  }

  private checkBudgets(): void {
    if (this.c.returning || this.c.finished || this.c.phase === 'preflight') return;
    const pos = this.c.vehicle.pos;
    const d = this.distanceHome(pos);
    const climb = Math.max(0, cruiseZ(this.c.air, pos[0], pos[1], this.c.agl) - pos[2]);
    const timeHome = this.timeHome();
    if (this.c.t + timeHome * 1.2 + 10 >= this.c.maxSim) {
      this.home.startReturn(`time budget (${fmt(this.c.maxSim - this.c.t, 0)} s left)`);
      return;
    }
    const need = this.c.battery.costPct(d, climb) * 1.3 * (this.c.health?.costScale ?? 1) + this.exitPct() + 3;
    if (this.c.battery.usedPct() + need >= this.c.mission.budgets.batteryPct) {
      const budget = `battery budget (${fmt(this.c.battery.usedPct(), 0)}% used of ${fmt(this.c.mission.budgets.batteryPct, 0)}%)`;
      if (this.canSwap(timeHome)) this.startSwapReturn(budget);
      else this.home.startReturn(budget);
    }
  }

  /**
   * Whether a battery-budget return should land for a fresh pack and fly on: there is search left,
   * a spare pack, time for the swap and a useful flight after it, and no fault active (a faulted
   * flight comes home and stays home, so the fault bench measures the failsafe, not the spares).
   */
  private canSwap(timeHome: number, next: MissionBuilding | null = null): boolean {
    if (this.c.batterySwaps >= SPARE_PACKS || this.c.health?.flags.size) return false;
    if (this.c.t + timeHome * 1.2 + SWAP_SEC + SWAP_MIN_FLIGHT_SEC >= this.c.maxSim) return false;
    if (this.search.findMission() && requirementsMet(this.c.mission, this.c.confirmed)) return false;
    return next !== null || !this.search.coverageDone || this.search.pending.length > 0 || this.unfinishedVisit() !== null;
  }

  /** Energy drawn so far over every pack, usable %. */
  private energyUsedPct(): number {
    return this.c.swappedUsedPct + this.c.battery.usedPct();
  }

  /**
   * Start a building search, or first swap the battery when the search would not fit in what is
   * left of the pack (the probability search picks its next building itself, so it is left alone).
   */
  private beginBuilding(b: MissionBuilding): void {
    const used = this.c.battery.usedPct();
    const est = this.visitPcts.length ? Math.max(...this.visitPcts) : VISIT_PCT_DEFAULT;
    const home = this.c.battery.costPct(centerDist(b, this.c.world.home), 0) * 1.3 + 6;
    const budget = this.c.mission.budgets.batteryPct;
    // Only when a fresh pack would cover it, or the drone would shuttle between pad and building.
    if (!this.c.sar && used + est + home >= budget && est + home < budget - used && this.canSwap(this.timeHome(), b)) {
      this.search.pending.unshift(b);
      this.c.swapping = true;
      this.home.startReturn(`battery: ${fmt(used, 0)}% used and a building search takes about ${fmt(est, 0)}%, so swapping the battery before ${b.label}`);
      return;
    }
    this.visitFromPct = this.energyUsedPct();
    this.entry.startBuilding(b);
  }

  /** The building being searched, while its search is not finished. */
  private unfinishedVisit(): MissionBuilding | null {
    const v = this.c.visit;
    return v && !v.searchComplete && this.c.phase !== 'exit' ? v.b : null;
  }

  private startSwapReturn(budget: string): void {
    // A building cut short goes back in the queue: the next flight searches it from the start.
    const b = this.unfinishedVisit();
    if (b && !this.search.pending.some((p) => p.id === b.id)) this.search.pending.unshift(b);
    this.c.swapping = true;
    this.search.dropConfirm();
    this.home.startReturn(`${budget}, to swap the battery${b ? ` and come back for ${b.label}` : ''}`);
  }

  /** On the pad after a swap return: fit a fresh pack, take off and carry on where the search left off. */
  private swapBattery(): void {
    this.c.swapping = false;
    this.c.batterySwaps += 1;
    this.c.swappedUsedPct += this.c.battery.usedPct();
    this.c.battery.swap();
    this.c.monitor.batterySwapped();
    this.c.log('action', `Battery swap ${this.c.batterySwaps} of ${SPARE_PACKS}: fresh pack fitted, ${fmt(SWAP_SEC, 0)} s on the pad.`);
    this.c.setJob(
      this.pilot.holdJob('preflight', SWAP_SEC, () => {
        this.c.returning = false;
        this.c.landedHome = false;
        this.c.log('action', 'Taking off to resume the search.');
        const climb = this.pilot.climbToCruise();
        if (climb.length) this.c.setJob(this.pilot.followJob('coverage', climb, () => this.search.continueMission(), 0.6));
        else this.search.continueMission();
      }),
    );
  }

  // ----------------------------------------------------------- faults

  /** Advance injected faults: motor thrust, the operator link. */
  private advanceFaults(): void {
    const f = this.c.faults;
    if (!f) return;
    f.advance(this.c.t, this.c.dt, this.c.phase);
    this.c.vehicle.setThrustAuthority(f.motorEfficiency());
    const down = !f.linkUp();
    if (down !== this.c.linkDown) this.c.log(down ? 'warn' : 'info', down ? 'Operator link down.' : 'Operator link back.');
    this.c.linkDown = down;
  }

  /**
   * GNSS lost outdoors, before the failsafe sends the drone home: the horizontal position is only a
   * dead-reckoning guess, so climb to cruise height (height comes from the barometer and range
   * finder, which still work) and back away from the nearest building, instead of flying the plan.
   */
  private climbClear(cmd: VehicleCommand): VehicleCommand {
    const flags = this.c.health?.flags;
    // Without a fix, hold position by climbing rather than hovering where drift can carry the drone into a wall.
    // In strong wind, get above the roofline before the trip home, since the gusts push the drone around near walls.
    const gnss = flags?.has('gnss_lost') && !this.c.returning;
    const pos = this.c.vehicle.pos;
    // Not on the final descent over the pad: the clearing around home is open, and climbing back up there
    // turns a landing into a climb-descend cycle that never ends.
    const atHome = Math.hypot(pos[0] - this.c.world.home[0], pos[1] - this.c.world.home[1]) < CLIMB_CLEAR_HOME_M;
    const severe = flags?.has('wind_severe') && !atHome;
    // Home on dead reckoning: the descent over the pad would put the drone down metres from it, among
    // trees. Hold at cruise height over the believed pad until the fix is back.
    if (flags?.has('gnss_lost') && this.c.returning && this.c.phase === 'return_home' && atHome && !this.c.inside) {
      if (!this.gnssPadHoldLogged) {
        this.gnssPadHoldLogged = true;
        this.c.log('warn', 'Over the pad on dead reckoning. Holding at cruise height until the GNSS fix is back.');
      }
      return { ...cmd, vel: [cmd.vel[0]!, cmd.vel[1]!, Math.max(cmd.vel[2]!, 0)] };
    }
    if (!(gnss || severe) || this.c.inside || this.c.phase === 'land' || this.c.phase === 'done') return cmd;
    const vz = Math.max(0, Math.min(1.2, cruiseZ(this.c.air, pos[0], pos[1], this.c.agl) - pos[2]));
    let away: [number, number] = [0, 0];
    let nearest = Infinity;
    for (const b of this.c.world.buildings) {
      const ring = b.footprint;
      const cx = ring.reduce((a, p) => a + p.x, 0) / ring.length;
      const cy = ring.reduce((a, p) => a + p.y, 0) / ring.length;
      const d = Math.hypot(pos[0] - cx, pos[1] - cy);
      if (d < nearest && d > 1e-6) {
        nearest = d;
        away = [(pos[0] - cx) / d, (pos[1] - cy) / d];
      }
    }
    // Only near a building, and only until the drone is up at cruise height. Within half a metre counts:
    // the altitude loop holds a few centimetres low in wind, and a stricter test never hands back.
    const below = cruiseZ(this.c.air, pos[0], pos[1], this.c.agl) - pos[2] > CLIMB_CLEAR_TOL_M;
    if (!gnss && !below) return cmd;
    // Back away and climb only where the way is open: away from the building's middle can be into a tree,
    // and straight up next to a wall can be under the eaves.
    const step = (d: V3): boolean => segmentClear(this.c.physics, pos, [pos[0] + d[0], pos[1] + d[1], pos[2] + d[2]]);
    const near = nearest < 15 && below && step([away[0] * 1.5, away[1] * 1.5, 0]);
    const up = vz > 0 && step([0, 0, 1.5]) ? vz : 0;
    // Boxed in, under eaves or branches with no open way back: leave the flight to the job, whose route
    // was planned around them, rather than hover where the wind or the drift carries the drone.
    if (!near && up === 0) return cmd;
    return { vel: [near ? away[0] * 0.5 : 0, near ? away[1] * 0.5 : 0, up], yaw: null };
  }

  /** ESC telemetry: achieved over commanded thrust per motor, with a little measurement noise. */
  private motorTelemetry(): number[] {
    const eff = this.c.faults?.motorEfficiency() ?? [1, 1, 1, 1];
    return eff.map((e) => Math.min(1.05, e + this.c.escNoise.gaussian() * 0.01));
  }

  private checkHealth(): void {
    const pos = this.c.vehicle.pos;
    const home = this.c.world.home;
    const dh = Math.hypot(home[0] - pos[0], home[1] - pos[1]);
    const climb = Math.max(0, cruiseZ(this.c.air, pos[0], pos[1], this.c.agl) - pos[2]);
    const seen = this.c.failsafe.events().length;
    const verdict = this.c.failsafe.check({
      tSec: this.c.t,
      dt: this.c.dt,
      phase: this.c.phase,
      inside: this.c.inside,
      linkUp: !this.c.linkDown,
      cameraFrame: this.c.faults?.cameraUp() ?? true,
      lidar: this.c.lastRing,
      lidarExpected: this.c.lastExpected,
      gnssFix: this.c.faults ? this.c.faults.gnss().fix : true,
      poseHealth: this.poseHealth(),
      motorRatio: this.motorTelemetry(),
      batteryPct: this.c.battery.remainingPct(),
      modelPowerW: this.c.lastPowerW,
      usableWh: USABLE_WH,
      costHomePct: this.c.battery.costPct(this.distanceHome(pos), climb),
      budgetPct: this.c.mission.budgets.batteryPct,
      windEstimate: this.c.inside ? null : this.c.windEstimate,
      homeDistM: dh,
      toHome: dh > 1e-6 ? [(home[0] - pos[0]) / dh, (home[1] - pos[1]) / dh] : [1, 0],
      cruiseMps: DEFAULT_BATTERY.cruiseMps,
    });
    this.c.lastRing = null;
    this.c.lastExpected = null;
    this.c.health = verdict;
    for (const e of this.c.failsafe.events().slice(seen)) this.c.log(e.action === 'none' ? 'info' : 'warn', e.message);
    if (verdict.action.kind !== 'none') this.handleSafety(verdict.action);
  }

  private checkSafety(pos: V3): void {
    const look = 0.5;
    const predicted: V3 = [pos[0] + this.c.lastCmd.vel[0] * look, pos[1] + this.c.lastCmd.vel[1] * look, pos[2] + this.c.lastCmd.vel[2] * look];
    const action = this.c.monitor.check({
      tSec: this.c.t,
      pose: this.c.vehicle.pose(),
      predicted,
      battery: this.c.battery,
      distanceHomeM: this.distanceHome(pos),
      collided: this.c.lastCollided,
      inside: this.c.inside,
      phase: this.c.phase,
    });
    this.c.lastCollided = false;
    if (action.kind !== 'none') this.handleSafety(action);
  }

  private handleSafety(action: SafetyAction): void {
    const rank = RANK[action.kind];
    if (action.kind === 'land') {
      if (this.c.phase !== 'land') {
        this.c.log('warn', `Safety: ${action.reason}.`);
        this.c.suspended = null;
        this.home.startLand(action.reason);
      }
      return;
    }
    if (action.kind === 'exit_building' && this.c.inside && this.c.phase === 'exit') {
      if (!this.c.returning) this.c.log('warn', `Safety: ${action.reason}. Returning home after the exit.`);
      this.c.returning = true;
      return;
    }
    if (action.kind === 'return_home' || action.kind === 'exit_building') {
      if (this.c.returning) return;
      this.c.log('warn', `Safety: ${action.reason}.`);
      // The battery reserve is the end of a pack, not of the mission, while there are spares.
      if (action.reason.startsWith('Battery reserve') && this.canSwap(this.timeHome())) this.startSwapReturn(`safety (${action.reason})`);
      else this.home.startReturn(`safety (${action.reason})`);
      return;
    }
    // hold and backtrack are latched in the recover phase.
    if (this.c.phase === 'recover' && rank <= this.c.recoverRank) return;
    if (this.c.phase === 'land' || this.c.phase === 'done') return;
    const resume = this.c.suspended ?? this.c.job;
    this.c.suspended = resume;
    this.c.recoverRank = rank;
    this.c.log('warn', `Safety ${action.kind}: ${action.reason}.`);
    const back = () => {
      const job = this.c.suspended;
      this.c.suspended = null;
      this.c.recoverRank = 0;
      if (job) this.c.setJob(job);
    };
    if (action.kind === 'hold') {
      this.c.setJob(this.pilot.holdJob('recover', action.seconds ?? 1, back));
      return;
    }
    // Backtrack: retrace the breadcrumb trail.
    const meters = action.meters ?? 1.5;
    const pos = this.c.navPos;
    const trail: Waypoint[] = [];
    let acc = 0;
    let prev: V3 = pos;
    for (let i = this.c.crumbs.length - 1; i >= 0 && acc < meters; i--) {
      const c = this.c.crumbs[i]!;
      const d = dist(prev, c);
      if (d < 0.05) continue;
      acc += d;
      trail.push({ pos: c, yaw: this.c.navYaw });
      prev = c;
    }
    this.c.setJob(this.pilot.followJob('recover', trail, back, 0.15));
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

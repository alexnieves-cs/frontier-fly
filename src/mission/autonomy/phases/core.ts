import { RNG } from '../../../shared/rng';
import { OutdoorPhysics } from '../../../world/outdoor/physics';
import type { Mission } from '../../types';
import { createBattery } from '../battery';
import type {
  BatteryModel,
  ConfirmedContact,
  Contact,
  DronePose,
  EntryCandidate,
  EntryRecord,
  LiveDetectionSource,
  LogEntry,
  MissionWorld,
  Phase,
  RunnerOptions,
  RunOutcome,
  RunRecord,
  SafetyMonitor,
  V3,
} from '../contracts';
import { ConfirmTracker } from '../planner/confirm';
import { cameraFootprint, cruiseAglFor, planCoverage, type CoveragePlan } from '../planner/coverage';
import {
  type Airspace,
  airspaceFor,
  cruiseZ,
} from '../planner/geometry';
import { GIMBAL_RANGE_SCALE, OutdoorLook } from '../perception';
import { type ApertureSensor, OpeningSensor } from '../openingSensor';
import { type ActingPolicy, toLocal, toWorld } from '../planner/indoor';
import type { LocalTruth, Localizer, LocVisitRecord } from '../localizer';
import { QuadVehicle } from '../quadVehicle';
import { createFailsafe, type Failsafe, type HealthVerdict } from '../failsafe';
import { FaultInjector } from '../faults';
import { createSafetyMonitor } from '../safety';
import { SarSearch } from '../sar/search';
import {
  INDOOR_LIMITS,
  OUTDOOR_LIMITS,
  Vehicle,
  type VehicleCommand,
  type VehicleLimits,
} from '../vehicle';
import {
  type Job,
  NEAR_LIMITS,
  type Visit,
  fmt,
} from './shared';

/**
 * Outdoor tilt limit, rad (55°). The 1.2 kg quad has a thrust-to-weight of 2.5, so 55° leaves 30% of thrust
 * for altitude and attitude. Indoors keeps the default 23° the close-in planners are tuned around.
 */
const OUTDOOR_TILT_RAD = 0.96;
/** Tilt limit flying through an opening in calm air, rad (40°): the fly-through servo was tuned below it. */
const FLY_THROUGH_TILT_RAD = 0.7;

/** RTK GNSS error with a fix (see stepGnss): 1-sigma horizontal and vertical, m, and correlation time, s. */
const GNSS_SIGMA_H_M = 0.015;
const GNSS_SIGMA_V_M = 0.025;
const GNSS_TAU_S = 30;

/**
 * What every phase of a run shares: the airframe, the sensors and planners built for the mission,
 * the current phase and job, the visit in progress, and the run record. The phase objects
 * (search.ts, entry.ts, indoor.ts, home.ts) each own their own state on top of this and act through
 * it; the motion primitives they fly with are the Pilot (pilot.ts). Nothing here decides what
 * happens next.
 */
export class MissionCore {
  readonly dt: number;
  readonly maxSim: number;
  readonly air: Airspace;
  readonly agl: number;
  readonly vehicle: Vehicle;
  readonly battery: BatteryModel;
  readonly monitor: SafetyMonitor;
  readonly tracker: ConfirmTracker;
  readonly coverage: CoveragePlan;
  /** Probability-of-area search, when the mission flies one instead of lanes. */
  readonly sar: SarSearch | null;
  readonly policy: ActingPolicy | null;
  /** Reference mode: indoor exploration and paths read the true floor plan. */
  readonly oracle: boolean;
  readonly batteryStart: number;
  /** Injected faults, when the run has any. Ground truth: only hardware and sensor models read it. */
  readonly faults: FaultInjector | null;
  readonly failsafe: Failsafe;
  /** The failsafe's latest verdict. */
  health: HealthVerdict | null = null;
  /** No operator heartbeat this tick. */
  linkDown = false;
  /** The last lidar ring as measured this tick (indoors), and what the drone's map predicted for it. */
  lastRing: Float64Array | null = null;
  lastExpected: Float64Array | null = null;
  /** Electrical power of the last vehicle step, W. */
  lastPowerW = 0;
  readonly escNoise: RNG;
  /** The flight controller's wind estimate, world frame, m/s: the wind it has to lean into, low-passed. */
  windEstimate: V3 = [0, 0, 0];

  ticks = 0;
  t = 0;
  phase: Phase = 'preflight';
  /** What the drone is flying now. The runner sets the first one (preflight) once the phases exist. */
  job!: Job;
  /** The job a safety hold or backtrack interrupted, and that action's rank (the recover latch). */
  suspended: Job | null = null;
  recoverRank = 0;
  finished = false;
  hardStopped = false;
  aborted = false;
  /** The crash rule ended the flight (shared/constants.ts CRASH_SPEED). */
  crashed = false;
  returning = false;
  landingEmergency = false;
  inside: string | null = null;
  lastCollided = false;
  lastCmd: VehicleCommand = { vel: [0, 0, 0] };
  nextTrack = 0;
  nextDetect = 0;
  nextBudgetCheck = 0;

  visit: Visit | null = null;
  /** The entry waiting on the operator, shown in the snapshot. */
  pendingEntry: EntryCandidate | null = null;
  readonly crumbs: V3[] = [];

  readonly logs: LogEntry[] = [];
  readonly trackRec: RunRecord['track'] = [];
  readonly phases: RunRecord['phases'] = [];
  readonly confirmed: ConfirmedContact[] = [];
  readonly declaredClear: RunRecord['declaredClear'] = [];
  readonly collisions: RunRecord['collisions'] = [];
  readonly entries: EntryRecord[] = [];
  readonly recent: Contact[] = [];
  outcome: RunOutcome = 'incomplete';
  landedHome = false;
  /** The visit's pose estimator while the drone flies a building on it (localization 'estimated'). */
  loc: Localizer | null = null;
  /** Opening measurements during inspection, and the aperture tracker during a fly-through. */
  readonly openingSensor: OpeningSensor;
  aperture: ApertureSensor | null = null;
  /** Where the camera has looked outdoors, by the drone's own pose: outdoor confirm-clear evidence. */
  readonly outdoorLook: OutdoorLook;
  readonly locVisits: LocVisitRecord[] = [];

  constructor(
    readonly mission: Mission,
    readonly world: MissionWorld,
    readonly source: LiveDetectionSource,
    readonly options: RunnerOptions,
    readonly physics: OutdoorPhysics,
  ) {
    this.dt = options.dt ?? 0.05;
    this.maxSim = options.maxSimSec ?? mission.budgets.timeSec;
    this.air = airspaceFor(mission, world);
    this.agl = cruiseAglFor(mission);
    this.faults = options.faults ? new FaultInjector(options.faults.plan, options.faults.seed ?? world.seed) : null;
    this.failsafe = createFailsafe();
    this.escNoise = new RNG(world.seed >>> 0, 'esc-telemetry');
    this.openingSensor = new OpeningSensor(world.seed);
    this.outdoorLook = new OutdoorLook(world);
    this.battery = this.faults ? this.faults.wrapBattery(createBattery()) : createBattery();
    this.batteryStart = this.battery.remainingPct();
    this.vehicle =
      options.vehicle === 'quad'
        ? new QuadVehicle(physics, this.battery, world.home, world.homeYaw, world.outdoor)
        : new Vehicle(physics, this.battery, world.home, world.homeYaw, world.outdoor);
    this.monitor = createSafetyMonitor(mission, world);
    // Outdoor views come from the gimbal lens, which resolves GIMBAL_RANGE_SCALE times farther.
    this.tracker = new ConfirmTracker(mission, { outdoorRangeScale: GIMBAL_RANGE_SCALE });
    const search = options.search;
    const spacing = search?.mode === 'lanes' ? search.laneSpacingM : undefined;
    // planCoverage spaces lanes by camera footprint and overlap; a requested spacing maps onto the overlap.
    const overlap = spacing ? 1 - spacing / (2 * cameraFootprint(this.agl)) : undefined;
    this.coverage = planCoverage(mission, world, { agl: this.agl, overlap });
    this.sar =
      search?.mode === 'probability' && mission.missionType === 'find_person' ? new SarSearch(world, this.air, this.agl, search) : null;
    const p = options.policy;
    this.oracle = options.indoor === 'oracle';
    this.policy = p && options.indoor !== 'frontier' && !this.oracle ? p : null;
    if (options.indoor === 'policy' && !this.policy)
      this.log('warn', 'Policy explorer requested without a policy. Using frontier exploration.');
    this.phases.push({ phase: 'preflight', tStart: 0, tEnd: 0 });
    if (this.sar) {
      const [lx, ly] = this.sar.prior.ipp;
      this.log(
        'info',
        `${mission.name}: find person, entry policy ${mission.entryPolicy}, probability search at ${fmt(this.agl, 0)} m AGL from the last known point (${fmt(lx)}, ${fmt(ly)}). ${fmt((1 - this.sar.prior.rest) * 100, 0)}% of the prior is within aerial reach.`,
      );
    } else {
      this.log(
        'info',
        `${mission.name}: ${mission.missionType.replace('_', ' ')}, entry policy ${mission.entryPolicy}, ${this.coverage.lanes.length} coverage lanes at ${fmt(this.agl, 0)} m AGL, ${fmt(this.coverage.laneSpacing)} m apart.`,
      );
    }
    if (this.agl < 11) this.log('warn', `Altitude band caps cruise at ${fmt(this.agl)} m AGL, close to the canopy.`);
  }

  limits(): VehicleLimits {
    let base = this.baseLimits();
    // Outdoors the airframe may lean to 55° (PX4's MPC_TILTMAX_AIR allows up to 89°). The default 23° holds
    // about 5 m/s of wind on the 1.2 kg quad and 40° about 8 m/s, so at 40° an 8 m/s wind left no headway home.
    // The limit is fixed rather than scheduled on the wind estimate: the estimate lags a wind front by a second
    // or two, and that is when the drone is blown into walls. Fly-throughs keep 40° unless it is windy.
    const windy = Math.hypot(this.windEstimate[0], this.windEstimate[1]) >= 3;
    if (!this.inside)
      base = { ...base, maxTilt: this.phase === 'fly_through' && !windy ? FLY_THROUGH_TILT_RAD : OUTDOOR_TILT_RAD, gentleStops: this.gentleStops(windy) };
    const k = this.health?.speedScale ?? 1;
    return k < 1 ? { ...base, speedH: base.speedH * k, speedV: base.speedV * Math.max(0.5, k) } : base;
  }

  /**
   * Building approach and inspection in calm air, with no fault flagged: slow moves there keep to 40°
   * (quadVehicle.ts). Wind, failsafe responses (which may need every degree of lean) and other phases keep 55°.
   */
  private gentleStops(windy: boolean): boolean {
    return !windy && !this.health?.flags.size && (this.phase === 'approach' || this.phase === 'inspect');
  }

  private baseLimits(): VehicleLimits {
    if (this.inside || this.phase === 'fly_through') return INDOOR_LIMITS;
    const pos = this.vehicle.pos;
    if (pos[2] < cruiseZ(this.air, pos[0], pos[1], this.agl) - 3) return NEAR_LIMITS;
    return OUTDOOR_LIMITS;
  }

  /**
   * The pose the planners and controllers fly on: the estimate while a
   * localizer runs, else the vehicle's true pose. Truth stays in use for the
   * record, the safety referee and outdoor flight.
   */
  get navPos(): V3 {
    const l = this.loc;
    if (!l) return this.vehicle.pos;
    return toWorld(l.grid, l.pos);
  }

  /**
   * The pose the drone believes it has: indoors the estimate (true pose in the reference arms), during a
   * fly-through the aperture tracker's reading, outdoors the vehicle position as believed() presents it.
   */
  ownPose(): DronePose {
    if (this.aperture?.hasReading && (this.phase === 'fly_through' || this.phase === 'exit') && !this.loc)
      return { pos: this.aperture.pos, yaw: this.aperture.yaw, vel: this.aperture.vel };
    return { pos: this.navPos, yaw: this.navYaw, vel: this.navVel };
  }

  get navVel(): V3 {
    return this.loc ? this.loc.vel : this.vehicle.vel;
  }

  get navYaw(): number {
    return this.loc ? this.loc.yaw : this.vehicle.yaw;
  }

  /** The vehicle's true motion in the visit's building frame: what the localization sensors measure. */
  localTruth(): LocalTruth {
    const grid = this.loc?.grid ?? this.visit!.grid;
    const a = this.vehicle.accel;
    const v = this.vehicle.vel;
    return {
      pos: toLocal(grid, this.vehicle.pos),
      yaw: this.vehicle.yaw,
      vel: [v[0], v[1], v[2]],
      accel: [a[0], a[1], a[2]],
      yawRate: this.vehicle.yawRate,
      dt: this.dt,
    };
  }

  /** Stop flying on the estimate and keep the visit's localization record. */
  endLocalization(): void {
    const l = this.loc;
    if (!l) return;
    this.loc = null;
    if (l.flownTicks > 0) this.locVisits.push(l.record(this.visit?.b.id ?? ''));
  }

  setPhase(phase: Phase): void {
    if (phase === this.phase) return;
    const last = this.phases[this.phases.length - 1];
    if (last) last.tEnd = this.t;
    this.phases.push({ phase, tStart: this.t, tEnd: this.t });
    this.phase = phase;
  }

  setJob(job: Job): void {
    this.job = job;
    this.setPhase(job.phase);
  }

  log(tone: LogEntry['tone'], message: string): void {
    this.logs.push({ tSec: Math.round(this.t * 100) / 100, tone, message });
  }

  where(p: V3): string {
    return `(${fmt(p[0])}, ${fmt(p[1])}, ${fmt(p[2])})`;
  }

  recordTrack(): void {
    const p = this.vehicle.pos;
    this.trackRec.push({
      t: Math.round(this.t * 100) / 100,
      pos: [p[0], p[1], p[2]],
      phase: this.phase,
    });
  }

  // ----------------------------------------------------------- GNSS

  /**
   * Nominal GNSS error with a fix: a first-order Gauss-Markov process, GNSS_SIGMA_H_M horizontally and
   * GNSS_SIGMA_V_M vertically with a GNSS_TAU_S correlation time, as an RTK receiver's fixed solution
   * wanders. Drawn every tick from its own stream, so it never shifts another random draw.
   */
  private gnssNoise: V3 = [0, 0, 0];
  private gnssRng: RNG | null = null;

  stepGnss(): void {
    this.gnssRng ??= new RNG(this.world.seed >>> 0, 'gnss-rtk');
    const a = Math.exp(-this.dt / GNSS_TAU_S);
    const b = Math.sqrt(1 - a * a);
    const sig = [GNSS_SIGMA_H_M, GNSS_SIGMA_H_M, GNSS_SIGMA_V_M];
    for (let i = 0; i < 3; i++) this.gnssNoise[i] = this.gnssNoise[i]! * a + sig[i]! * b * this.gnssRng.gaussian();
  }

  /** The true position while believed() runs on a GNSS estimate; the range finder measures from here. */
  truePos: V3 | null = null;

  /** Run `fn` with the vehicle at the position the drone believes it is at (outdoors: the GNSS reading). */
  believed<T>(fn: () => T): T {
    if (this.inside) return fn();
    // With a fix, an RTK-grade receiver's few centimetres; without one, the dead-reckoning drift on top.
    const lost = this.faults?.gnss();
    const n = this.gnssNoise;
    const d = lost && !lost.fix ? lost.errorM : null;
    const e: V3 = d ? [n[0] + d[0], n[1] + d[1], n[2] + d[2]] : n;
    const truth = this.vehicle.pos;
    this.vehicle.pos = [truth[0] + e[0], truth[1] + e[1], truth[2] + e[2]];
    this.truePos = truth;
    try {
      return fn();
    } finally {
      this.vehicle.pos = truth;
      this.truePos = null;
    }
  }
}

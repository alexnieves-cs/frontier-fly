import { descentClear } from '../../../sim/sensorPlanner';
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
  MissionBuilding,
  MissionRunner,
  MissionWorld,
  Phase,
  RunnerOptions,
  RunnerSnapshot,
  RunOutcome,
  RunRecord,
  SafetyMonitor,
  V3,
} from '../contracts';
import { ConfirmTracker, type ConfirmTrack } from '../planner/confirm';
import { cameraFootprint, cruiseAglFor, planCoverage, type CoveragePlan } from '../planner/coverage';
import {
  type Airspace,
  airspaceFor,
  cruiseZ,
  dist,
  len,
  segmentClear,
  sub,
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
  PLAN_ACCEL,
  SHIELD_STALL_SEC,
  type Visit,
  type Waypoint,
  fmt,
  hold,
  scaleTo,
} from './shared';

/**
 * Outdoor tilt limit, rad (55°). The 1.2 kg quad has a thrust-to-weight of 2.5, so 55° leaves 30% of thrust
 * for altitude and attitude. Indoors keeps the default 23° the close-in planners are tuned around.
 */
const OUTDOOR_TILT_RAD = 0.96;
/** Tilt limit flying through an opening in calm air, rad (40°): the fly-through servo was tuned below it. */
const FLY_THROUGH_TILT_RAD = 0.7;

/**
 * Runner state, the constructor and the motion primitives every phase builds on:
 * waypoint following, holds, the indoor shield and breadcrumb retracing. The
 * phase layers stack on top (search, entry, indoor, then runner.ts), and the
 * transitions a phase hands to a later layer are declared abstract here.
 */
export abstract class RunnerBase implements MissionRunner {
  protected readonly dt: number;
  protected readonly maxSim: number;
  protected readonly air: Airspace;
  protected readonly agl: number;
  protected readonly vehicle: Vehicle;
  protected readonly battery: BatteryModel;
  protected readonly monitor: SafetyMonitor;
  protected readonly tracker: ConfirmTracker;
  protected readonly coverage: CoveragePlan;
  /** Probability-of-area search, when the mission flies one instead of lanes. */
  protected readonly sar: SarSearch | null;
  protected readonly policy: ActingPolicy | null;
  /** Reference mode: indoor exploration and paths read the true floor plan. */
  protected readonly oracle: boolean;
  protected readonly batteryStart: number;
  /** Injected faults, when the run has any. Ground truth: only hardware and sensor models read it. */
  protected readonly faults: FaultInjector | null;
  protected readonly failsafe: Failsafe;
  /** The failsafe's latest verdict. */
  protected health: HealthVerdict | null = null;
  /** No operator heartbeat this tick. */
  protected linkDown = false;
  /** The last lidar ring as measured this tick (indoors), and what the drone's map predicted for it. */
  protected lastRing: Float64Array | null = null;
  protected lastExpected: Float64Array | null = null;
  /** Electrical power of the last vehicle step, W. */
  protected lastPowerW = 0;
  protected readonly escNoise: RNG;
  /** The flight controller's wind estimate, world frame, m/s: the wind it has to lean into, low-passed. */
  protected windEstimate: V3 = [0, 0, 0];

  protected ticks = 0;
  protected t = 0;
  protected phase: Phase = 'preflight';
  protected job: Job;
  protected suspended: Job | null = null;
  protected recoverRank = 0;
  protected finished = false;
  protected hardStopped = false;
  protected aborted = false;
  protected returning = false;
  protected landingEmergency = false;
  protected inside: string | null = null;
  protected lastCollided = false;
  protected lastCmd: VehicleCommand = { vel: [0, 0, 0] };
  protected nextTrack = 0;
  protected nextDetect = 0;
  protected nextBudgetCheck = 0;

  protected covIndex = 0;
  protected coverageDone = false;
  protected readonly discovered = new Set<string>();
  protected readonly pending: MissionBuilding[] = [];
  protected readonly visitedBuildings = new Set<string>();
  protected visit: Visit | null = null;
  protected operatorAnswer: 'approve' | 'deny' | null = null;
  protected pendingEntry: EntryCandidate | null = null;
  protected confirmTrack: ConfirmTrack | null = null;
  /** Earliest time each track may get another confirmation attempt. */
  protected readonly attempted = new Map<string, number>();
  /** Per track: indoor re-view bearings (camera heading, rad) that added no view. */
  protected readonly triedBearings = new Map<string, number[]>();
  protected readonly confirmRounds = new Map<string, number>();
  /** Tracks already given a last look when exploration of their building ended. */
  protected readonly finalLook = new Set<string>();
  protected outdoorClearChecked = false;
  protected policyClean = false;
  protected policyFell = false;
  protected frontierRan = false;
  protected readonly crumbs: V3[] = [];

  protected readonly logs: LogEntry[] = [];
  protected readonly trackRec: RunRecord['track'] = [];
  protected readonly phases: RunRecord['phases'] = [];
  protected readonly confirmed: ConfirmedContact[] = [];
  protected readonly declaredClear: RunRecord['declaredClear'] = [];
  protected readonly collisions: RunRecord['collisions'] = [];
  protected readonly entries: EntryRecord[] = [];
  protected readonly recent: Contact[] = [];
  /** What to run once a confirmation ends: back to the sweep, or back to indoor exploration. */
  protected resumeAfterConfirm: (() => void) | null = null;
  /** The running policy explorer, so a confirmation can hand control back and an exit can free it. */
  protected policyRun: { resume: () => void; release: () => void } | null = null;
  protected outcome: RunOutcome = 'incomplete';
  protected landedHome = false;
  /** The visit's pose estimator while the drone flies a building on it (localization 'estimated'). */
  protected loc: Localizer | null = null;
  /** Opening measurements during inspection, and the aperture tracker during a fly-through. */
  protected readonly openingSensor: OpeningSensor;
  protected aperture: ApertureSensor | null = null;
  /** Where the camera has looked outdoors, by the drone's own pose: outdoor confirm-clear evidence. */
  protected readonly outdoorLook: OutdoorLook;
  protected readonly locVisits: LocVisitRecord[] = [];

  constructor(
    readonly mission: Mission,
    readonly world: MissionWorld,
    readonly source: LiveDetectionSource,
    readonly options: RunnerOptions,
    protected readonly physics: OutdoorPhysics,
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
    const p = options.policy as ActingPolicy | undefined;
    this.oracle = options.indoor === 'oracle';
    this.policy = p && typeof p.actWith === 'function' && options.indoor !== 'frontier' && !this.oracle ? p : null;
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
    this.job = this.holdJob('preflight', 1, () => this.startCoverage());
    this.recordTrack();
  }

  protected limits(): VehicleLimits {
    let base = this.baseLimits();
    // Outdoors the airframe may lean to 55° (PX4's MPC_TILTMAX_AIR allows up to 89°). The default 23° holds
    // about 5 m/s of wind on the 1.2 kg quad and 40° about 8 m/s, so at 40° an 8 m/s wind left no headway home.
    // The limit is fixed rather than scheduled on the wind estimate: the estimate lags a wind front by a second
    // or two, and that is when the drone is blown into walls. Fly-throughs keep 40° unless it is windy.
    const windy = Math.hypot(this.windEstimate[0], this.windEstimate[1]) >= 3;
    if (!this.inside) base = { ...base, maxTilt: this.phase === 'fly_through' && !windy ? FLY_THROUGH_TILT_RAD : OUTDOOR_TILT_RAD };
    const k = this.health?.speedScale ?? 1;
    return k < 1 ? { ...base, speedH: base.speedH * k, speedV: base.speedV * Math.max(0.5, k) } : base;
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
  protected get navPos(): V3 {
    const l = this.loc;
    if (!l) return this.vehicle.pos;
    return toWorld(l.grid, l.pos);
  }

  /**
   * The pose the drone believes it has: indoors the estimate (true pose in the reference arms), during a
   * fly-through the aperture tracker's reading, outdoors the vehicle position as believed() presents it.
   */
  protected ownPose(): DronePose {
    if (this.aperture?.hasReading && (this.phase === 'fly_through' || this.phase === 'exit') && !this.loc)
      return { pos: this.aperture.pos, yaw: this.aperture.yaw, vel: this.aperture.vel };
    return { pos: this.navPos, yaw: this.navYaw, vel: this.navVel };
  }

  protected get navVel(): V3 {
    return this.loc ? this.loc.vel : this.vehicle.vel;
  }

  protected get navYaw(): number {
    return this.loc ? this.loc.yaw : this.vehicle.yaw;
  }

  /** The vehicle's true motion in the visit's building frame: what the localization sensors measure. */
  protected localTruth(): LocalTruth {
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
  protected endLocalization(): void {
    const l = this.loc;
    if (!l) return;
    this.loc = null;
    if (l.flownTicks > 0) this.locVisits.push(l.record(this.visit?.b.id ?? ''));
  }

  protected setPhase(phase: Phase): void {
    if (phase === this.phase) return;
    const last = this.phases[this.phases.length - 1];
    if (last) last.tEnd = this.t;
    this.phases.push({ phase, tStart: this.t, tEnd: this.t });
    this.phase = phase;
  }

  protected setJob(job: Job): void {
    this.job = job;
    this.setPhase(job.phase);
  }

  protected log(tone: LogEntry['tone'], message: string): void {
    this.logs.push({ tSec: Math.round(this.t * 100) / 100, tone, message });
  }

  protected where(p: V3): string {
    return `(${fmt(p[0])}, ${fmt(p[1])}, ${fmt(p[2])})`;
  }

  protected recordTrack(): void {
    const p = this.vehicle.pos;
    this.trackRec.push({
      t: Math.round(this.t * 100) / 100,
      pos: [p[0], p[1], p[2]],
      phase: this.phase,
    });
  }

  protected breadcrumb(): void {
    const p = this.navPos;
    const last = this.crumbs[this.crumbs.length - 1];
    if (!last || dist(last, p) >= 0.3) {
      this.crumbs.push([p[0], p[1], p[2]]);
      if (this.crumbs.length > 400) this.crumbs.shift();
    }
    const v = this.visit;
    if (v && !this.inside && v.standoff) {
      const tl = v.trail[v.trail.length - 1];
      if (!tl || dist(tl, p) >= 0.3) v.trail.push([p[0], p[1], p[2]]);
    }
  }

  /** Retrace flown positions (known clear) backwards until `enough` holds. String-pulled with casts. */
  protected retrace(trail: readonly V3[], enough: (p: V3) => boolean): Waypoint[] {
    const out: Waypoint[] = [];
    let cur: V3 = this.navPos;
    let i = trail.length - 1;
    for (let guard = 0; i >= 0 && guard < 400 && !enough(cur); guard++) {
      let j = i;
      for (let k = Math.max(0, i - 40); k < i; k++) {
        if (segmentClear(this.physics, cur, trail[k]!, 0.25)) {
          j = k;
          break;
        }
      }
      cur = trail[j]!;
      out.push({ pos: cur, tol: 0.2 });
      i = j - 1;
    }
    return out;
  }

  // ----------------------------------------------------------- jobs


  protected followJob(phase: Phase, route: Waypoint[], onDone: () => void, passTol?: number): Job {
    // Close to obstacles, brake to a stop before turning onto a new path.
    const v0 = this.navVel;
    const p0 = this.navPos;
    const wps =
      passTol !== undefined && passTol <= 0.3 && Math.hypot(v0[0], v0[1], v0[2]) > 0.15 && route.length
        ? [
            {
              pos: [p0[0], p0[1], p0[2]] as V3,
              tol: 0.12,
            },
            ...route,
          ]
        : route;
    let idx = 0;
    let dwell = -1;
    let spin = -1;
    let called = false;
    // Progress watch for shielded indoor moves: a waypoint the lidar shield will not let the drone
    // reach (structure closer than the map knew) is skipped rather than flown at forever.
    let bestD = Infinity;
    let sinceBest = 0;
    const tolFor = (i: number): number => {
      const wp = wps[i]!;
      if (wp.tol !== undefined) return wp.tol;
      return passTol ?? 0.25;
    };
    const isStop = (i: number): boolean => {
      const wp = wps[i]!;
      if (i === wps.length - 1 || wp.dwell || wp.spin) return true;
      return false;
    };
    const cornerSpeed = (i: number): number => {
      if (isStop(i)) return 0;
      const prev = i === 0 ? this.navPos : wps[i - 1]!.pos;
      const a = sub(wps[i]!.pos, prev);
      const b = sub(wps[i + 1]!.pos, wps[i]!.pos);
      const la = len(a);
      const lb = len(b);
      if (la < 1e-6 || lb < 1e-6) return 0.5;
      const cos = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb);
      if (cos > 0.9) return 99;
      if (cos > 0.7) return 1.2;
      // Sharp corners are full stops so the drone never swings wide near obstacles.
      return 0;
    };
    return {
      phase,
      plan: () => wps.slice(idx).map((w) => w.pos),
      tick: (): VehicleCommand => {
        const pos = this.navPos;
        while (idx < wps.length) {
          const wp = wps[idx]!;
          const d = dist(pos, wp.pos);
          const stop = isStop(idx) || cornerSpeed(idx) === 0;
          const tol = stop ? 0.12 : tolFor(idx);
          const slow = len(this.navVel) < 0.3;
          if (d < tol && (!stop || slow)) {
            if (wp.spin && spin < 0 && dwell < 0) spin = Math.PI * 2;
            if (spin > 0) {
              const rate = INDOOR_LIMITS.yawRate;
              spin -= rate * this.dt;
              if (spin <= 0) spin = 0;
              return { vel: hold(pos, wp.pos), yawRate: rate };
            }
            if (dwell === -1) dwell = wp.dwell ?? 0;
            if (dwell > 0) {
              dwell -= this.dt;
              return { vel: hold(pos, wp.pos), yaw: wp.yaw ?? null };
            }
            idx += 1;
            dwell = -1;
            spin = -1;
            bestD = Infinity;
            sinceBest = 0;
            continue;
          }
          // Speed from the distance to the next stop or corner.
          let ahead = d;
          let vEnd = 0;
          for (let k = idx; k < wps.length; k++) {
            const cs = cornerSpeed(k);
            if (cs < 99) {
              vEnd = cs;
              break;
            }
            ahead += dist(wps[k]!.pos, wps[k + 1]!.pos);
          }
          const speed = Math.min(99, Math.sqrt(vEnd * vEnd + 2 * PLAN_ACCEL * ahead), stop ? 1.6 * d + 0.03 : 99);
          if (d < bestD - 0.05) {
            bestD = d;
            sinceBest = 0;
          } else if ((sinceBest += this.dt) > SHIELD_STALL_SEC && this.inside && this.visit?.sensing && phase !== 'fly_through') {
            idx += 1;
            bestD = Infinity;
            sinceBest = 0;
            continue;
          }
          const dir = sub(wp.pos, pos);
          const want = scaleTo(dir, speed, this.limits());
          const vel = this.shielded(phase, want);
          const moving = Math.hypot(vel[0], vel[1]) > 0.3;
          // With a failed lidar arc, turn the middle of the healthy arc toward the direction of travel. The
          // wanted direction, not the shielded one: the shield treats the blind arc as blocked, so facing
          // it the drone barely moves and would never turn to see where it is going.
          const blind = this.blindYawOffset();
          const yaw =
            wp.yaw !== undefined && wp.yaw !== null && d < 4
              ? wp.yaw
              : blind !== 0 && Math.hypot(want[0], want[1]) > 0.1
                ? Math.atan2(want[1], want[0]) - blind
                : moving
                  ? Math.atan2(vel[1], vel[0])
                  : null;
          return { vel, yaw };
        }
        if (!called) {
          called = true;
          onDone();
        }
        return { vel: [0, 0, 0] };
      },
    };
  }

  /**
   * Indoors on the sensor-only map, routed moves (re-views, the way out) pass through the same lidar
   * shield as the explorers: the map is young and a route can graze structure the lidar sees now.
   */
  protected shielded(phase: Phase, vel: V3): V3 {
    const v = this.visit;
    if (!this.inside || !v?.sensing || phase === 'fly_through') return vel;
    const local = toLocal(v.grid, this.navPos);
    v.sensing.sense(local, this.navVel, this.navYaw, this.t);
    const [vx, vy] = v.sensing.filter(vel[0], vel[1], this.navYaw);
    const vz = vel[2] < 0 && !descentClear(v.sensing.lidar) ? 0 : vel[2];
    return [vx, vy, vz];
  }

  /**
   * IndoorSensing range hook, called on every indoor lidar reading before anything uses it: inject
   * lidar faults, keep the ring as measured (and what the drone's map predicts for it) for the
   * failsafe, and blank the beams the failsafe distrusts. NaN reads as blocked to every filter.
   */
  protected lidarHook(lidar: Float64Array): void {
    this.faults?.lidarRing(lidar);
    const v = this.visit;
    if (v?.sensing) {
      this.lastExpected = v.sensing.expectedRanges(toLocal(v.grid, this.vehicle.pos), this.vehicle.yaw, new Float64Array(lidar.length));
      this.lastRing = Float64Array.from(lidar);
    }
    for (const i of this.health?.unknownBeams ?? []) lidar[i] = NaN;
  }

  /**
   * Body angle (rad, counter-clockwise from the nose) of the middle of the widest healthy lidar arc
   * while the failsafe distrusts some beams; 0 when the lidar is healthy.
   */
  protected blindYawOffset(): number {
    const bad = this.health?.unknownBeams;
    if (!bad || bad.length === 0) return 0;
    const n = this.visit?.sensing?.lidar.length ?? 32;
    const blocked = new Set(bad);
    if (blocked.size >= n) return 0;
    let best = 0;
    let bestGap = -1;
    for (let i = 0; i < n; i++) {
      if (blocked.has(i)) continue;
      let gap = n;
      for (const j of blocked) gap = Math.min(gap, Math.abs(i - j), n - Math.abs(i - j));
      if (gap > bestGap) {
        bestGap = gap;
        best = i;
      }
    }
    return (best / n) * Math.PI * 2;
  }

  protected holdJob(phase: Phase, seconds: number, onDone: () => void): Job {
    const p = this.navPos;
    const at: V3 = [p[0], p[1], p[2]];
    const end = this.t + seconds;
    let called = false;
    return {
      phase,
      plan: () => [],
      tick: (): VehicleCommand => {
        if (this.t >= end - 1e-9 && !called) {
          called = true;
          onDone();
        }
        return { vel: hold(this.navPos, at) };
      },
    };
  }

  // ----------------------------------------------------------- transitions owned by later phases

  protected abstract startCoverage(): void;
  protected abstract startBuilding(b: MissionBuilding): void;
  protected abstract startReturn(reason: string): void;
  protected abstract exploreNext(): void;
  protected abstract leaveBuilding(): void;
  protected abstract startExplore(): void;
  protected abstract startReturnOutside(): void;
  protected abstract startLand(reason: string): void;

  abstract get done(): boolean;
  abstract approve(): void;
  abstract deny(): void;
  abstract abort(): void;
  abstract step(dtSec?: number): void;
  abstract snapshot(): RunnerSnapshot;
  abstract record(): RunRecord;
}

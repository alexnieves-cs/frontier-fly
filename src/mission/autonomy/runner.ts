import type { LightingState } from '../../shared/types';
import { initPhysics, PhysicsWorld } from '../../sim/world';
import { initOutdoorPhysics, OutdoorPhysics } from '../../world/outdoor/physics';
import { sampleHeight } from '../../world/outdoor/terrain';
import type { Opening } from '../../world/outdoor/types';
import type { Mission } from '../types';
import { createBattery } from './battery';
import type {
  BatteryModel,
  ClearScope,
  ConfirmedContact,
  Contact,
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
  SafetyAction,
  SafetyActionKind,
  SafetyMonitor,
  V3,
} from './contracts';
import { planApproach, type Standoff } from './planner/approach';
import { ConfirmTracker, confirmViewpoints, requirementsMet, type ConfirmTrack } from './planner/confirm';
import { cruiseAglFor, planCoverage, senseBuildings, type CoveragePlan } from './planner/coverage';
import { passageAxis, rankCandidates, scoreOpening, type PassageAxis } from './planner/entry';
import { FlyThrough } from './planner/flythrough';
import {
  type Airspace,
  airspaceFor,
  buildingBox,
  type BuildingBox,
  columnClear,
  cruisePath,
  cruiseZ,
  dist,
  headingTo,
  legBetween,
  len,
  operable,
  segmentClear,
  sub,
} from './planner/geometry';
import { landedAtHome, landingCommand, planReturn } from './planner/home';
import {
  FrontierExplorer,
  gridLos,
  IndoorVisibility,
  indoorGrid,
  type IndoorGrid,
  nearestNav,
  PATH_CLEARANCE,
  planIndoorPath,
  PolicyExplorer,
  type ActingPolicy,
  toLocal,
  toWorld,
  floorOf,
  wallClearance,
  INDOOR_FLY_Z,
} from './planner/indoor';
import { observeOpenings, planInspection } from './planner/inspect';
import { createSafetyMonitor } from './safety';
import { INDOOR_LIMITS, OUTDOOR_LIMITS, Vehicle, type VehicleCommand, type VehicleLimits } from './vehicle';

/**
 * Hierarchical mission this. A fixed-tick phase machine over the planner
 * stages: coverage discovers buildings, each building is approached,
 * inspected, entered through the safest observed opening, explored, and
 * exited; contacts of a mission class are confirmed from several bearings;
 * the drone returns home and lands. The safety monitor is checked every
 * tick and its actions run through the `recover` phase. Same inputs give
 * the same RunRecord.
 */

const NEAR_LIMITS: VehicleLimits = { ...OUTDOOR_LIMITS, speedH: 2.5, speedV: 1.2, accel: 2 };
const DETECT_PERIOD = 0.2;
const TRACK_PERIOD = 0.5;
const PLAN_ACCEL = 2;
/** No new policy voxel for this long counts as a stall, s. */
const POLICY_STALL_SEC = 8;
/** One contact hands the visit to frontier exploration before the safety ladder escalates. */
const POLICY_MAX_COLLISIONS = 1;
const POLICY_MAX_SEC = 150;
const MAX_EXPLORE_SEC = 260;
const CONFIRM_DWELL = 3.5;
/** Confirmation rounds a track gets before it is rejected. */
const CONFIRM_ROUNDS = 2;

const RANK: Record<SafetyActionKind, number> = { none: 0, hold: 1, backtrack: 2, exit_building: 3, return_home: 4, land: 5 };

interface Waypoint {
  pos: V3;
  /** Heading to hold. Null or undefined faces the travel direction. */
  yaw?: number | null;
  dwell?: number;
  spin?: boolean;
  /** Pass-through tolerance override. */
  tol?: number;
}

interface Job {
  phase: Phase;
  tick(): VehicleCommand;
  plan(): V3[];
}

interface Visit {
  b: MissionBuilding;
  box: BuildingBox;
  grid: IndoorGrid;
  standoff: Standoff | null;
  observed: Map<number, Opening>;
  ranked: EntryCandidate[];
  candIdx: number;
  denied: number;
  opening: Opening | null;
  axis: PassageAxis | null;
  aborts: number;
  vis: IndoorVisibility | null;
  explorer: FrontierExplorer | null;
  exploreStart: number;
  pass: number;
  policyCollisions: number;
  entered: boolean;
  /** Positions flown outside near this building, known to be clear. */
  trail: V3[];
}

function fmt(n: number, d = 1): string {
  return n.toFixed(d);
}

class Runner implements MissionRunner {
  private readonly dt: number;
  private readonly maxSim: number;
  private readonly air: Airspace;
  private readonly agl: number;
  private readonly vehicle: Vehicle;
  private readonly battery: BatteryModel;
  private readonly monitor: SafetyMonitor;
  private readonly tracker: ConfirmTracker;
  private readonly coverage: CoveragePlan;
  private readonly policy: ActingPolicy | null;
  private readonly batteryStart: number;

  private ticks = 0;
  private t = 0;
  private phase: Phase = 'preflight';
    private job: Job;
  private suspended: Job | null = null;
  private recoverRank = 0;
  private finished = false;
  private hardStopped = false;
  private aborted = false;
  private returning = false;
  private landingEmergency = false;
  private inside: string | null = null;
  private lastCollided = false;
  private lastCmd: VehicleCommand = { vel: [0, 0, 0] };
  private nextTrack = 0;
  private nextDetect = 0;
  private nextBudgetCheck = 0;

  private covIndex = 0;
  private coverageDone = false;
  private readonly discovered = new Set<string>();
  private readonly pending: MissionBuilding[] = [];
  private readonly visitedBuildings = new Set<string>();
  private visit: Visit | null = null;
  private operatorAnswer: 'approve' | 'deny' | null = null;
  private pendingEntry: EntryCandidate | null = null;
  private confirmTrack: ConfirmTrack | null = null;
  /** Earliest time each track may get another confirmation attempt. */
  private readonly attempted = new Map<string, number>();
  private readonly confirmRounds = new Map<string, number>();
  /** Tracks already given a last look when exploration of their building ended. */
  private readonly finalLook = new Set<string>();
  private outdoorClearChecked = false;
  private policyClean = false;
  private policyFell = false;
  private frontierRan = false;
  private readonly crumbs: V3[] = [];

  private readonly logs: LogEntry[] = [];
  private readonly trackRec: RunRecord['track'] = [];
  private readonly phases: RunRecord['phases'] = [];
  private readonly confirmed: ConfirmedContact[] = [];
  private readonly declaredClear: RunRecord['declaredClear'] = [];
  private readonly collisions: RunRecord['collisions'] = [];
  private readonly entries: EntryRecord[] = [];
  private readonly recent: Contact[] = [];
  private outcome: RunOutcome = 'incomplete';
  private landedHome = false;

  constructor(
    readonly mission: Mission,
    readonly world: MissionWorld,
    readonly source: LiveDetectionSource,
    readonly options: RunnerOptions,
    private readonly physics: OutdoorPhysics,
  ) {
    this.dt = options.dt ?? 0.05;
    this.maxSim = options.maxSimSec ?? mission.budgets.timeSec;
    this.air = airspaceFor(mission, world);
    this.agl = cruiseAglFor(mission);
    this.battery = createBattery();
    this.batteryStart = this.battery.remainingPct();
    this.vehicle = new Vehicle(physics, this.battery, world.home, world.homeYaw, world.outdoor);
    this.monitor = createSafetyMonitor(mission, world);
    this.tracker = new ConfirmTracker(mission);
    this.coverage = planCoverage(mission, world, { agl: this.agl });
    const p = options.policy as ActingPolicy | undefined;
    this.policy = p && typeof p.actWith === 'function' && options.indoor !== 'frontier' ? p : null;
    if (options.indoor === 'policy' && !this.policy) this.log('warn', 'Policy explorer requested without a policy. Using frontier exploration.');
    this.phases.push({ phase: 'preflight', tStart: 0, tEnd: 0 });
    this.log(
      'info',
      `${mission.name}: ${mission.missionType.replace('_', ' ')}, entry policy ${mission.entryPolicy}, ${this.coverage.lanes.length} coverage lanes at ${fmt(this.agl, 0)} m AGL, ${fmt(this.coverage.laneSpacing)} m apart.`,
    );
    if (this.agl < 11) this.log('warn', `Altitude band caps cruise at ${fmt(this.agl)} m AGL, close to the canopy.`);
    this.job = this.holdJob('preflight', 1, () => this.startCoverage());
    this.recordTrack();
  }

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
      indoorExplorer: this.policyClean ? 'policy' : this.frontierRan ? 'frontier' : null,
      log: this.logs.slice(),
    };
  }

  // ----------------------------------------------------------- tick

  private tick(): void {
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
    const step = this.vehicle.step(cmd, this.dt, this.limits());
    this.lastCollided = step.collided;
    if (step.collided) {
      const pos = this.vehicle.pos;
      this.collisions.push({ tSec: this.t, pos: [pos[0], pos[1], pos[2]], inside: this.inside });
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

  private limits(): VehicleLimits {
    if (this.inside || this.phase === 'fly_through') return INDOOR_LIMITS;
    const pos = this.vehicle.pos;
    if (pos[2] < cruiseZ(this.air, pos[0], pos[1], this.agl) - 3) return NEAR_LIMITS;
    return OUTDOOR_LIMITS;
  }

  private setPhase(phase: Phase): void {
    if (phase === this.phase) return;
    const last = this.phases[this.phases.length - 1];
    if (last) last.tEnd = this.t;
    this.phases.push({ phase, tStart: this.t, tEnd: this.t });
    this.phase = phase;
  }

  private setJob(job: Job): void {
    this.job = job;
    this.setPhase(job.phase);
  }

  private log(tone: LogEntry['tone'], message: string): void {
    this.logs.push({ tSec: Math.round(this.t * 100) / 100, tone, message });
  }

  private where(p: V3): string {
    return `(${fmt(p[0])}, ${fmt(p[1])}, ${fmt(p[2])})`;
  }

  private recordTrack(): void {
    const p = this.vehicle.pos;
    this.trackRec.push({ t: Math.round(this.t * 100) / 100, pos: [p[0], p[1], p[2]], phase: this.phase });
  }

  private breadcrumb(): void {
    const p = this.vehicle.pos;
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
  private retrace(trail: readonly V3[], enough: (p: V3) => boolean): Waypoint[] {
    const out: Waypoint[] = [];
    let cur: V3 = this.vehicle.pos;
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

  private followJob(phase: Phase, route: Waypoint[], onDone: () => void, passTol?: number): Job {
    // Close to obstacles, brake to a stop before turning onto a new path.
    const v0 = this.vehicle.vel;
    const wps =
      passTol !== undefined && passTol <= 0.3 && Math.hypot(v0[0], v0[1], v0[2]) > 0.15 && route.length
        ? [{ pos: [this.vehicle.pos[0], this.vehicle.pos[1], this.vehicle.pos[2]] as V3, tol: 0.12 }, ...route]
        : route;
    let idx = 0;
    let dwell = -1;
    let spin = -1;
    let called = false;
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
      const prev = i === 0 ? this.vehicle.pos : wps[i - 1]!.pos;
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
        const pos = this.vehicle.pos;
        while (idx < wps.length) {
          const wp = wps[idx]!;
          const d = dist(pos, wp.pos);
          const stop = isStop(idx) || cornerSpeed(idx) === 0;
          const tol = stop ? 0.12 : tolFor(idx);
          const slow = len(this.vehicle.vel) < 0.3;
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
          const dir = sub(wp.pos, pos);
          const vel = scaleTo(dir, speed, this.limits());
          const moving = Math.hypot(vel[0], vel[1]) > 0.3;
          const yaw = wp.yaw !== undefined && wp.yaw !== null && d < 4 ? wp.yaw : moving ? Math.atan2(vel[1], vel[0]) : null;
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

  private holdJob(phase: Phase, seconds: number, onDone: () => void): Job {
    const at: V3 = [this.vehicle.pos[0], this.vehicle.pos[1], this.vehicle.pos[2]];
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
        return { vel: hold(this.vehicle.pos, at) };
      },
    };
  }

  // ----------------------------------------------------------- coverage

  private startCoverage(): void {
    const wps = this.coverage.waypoints;
    if (!wps.length) {
      this.coverageDone = true;
      this.log('warn', 'No operable coverage lanes inside the geofence.');
      this.continueMission();
      return;
    }
    const pos = this.vehicle.pos;
    const climb: Waypoint[] = [];
    if (this.covIndex === 0 && pos[2] < cruiseZ(this.air, pos[0], pos[1], this.agl) - 1) {
      climb.push({ pos: [pos[0], pos[1], cruiseZ(this.air, pos[0], pos[1], this.agl)], tol: 0.6 });
      this.log('action', `Takeoff. Climbing to ${fmt(this.agl, 0)} m AGL.`);
    }
    const from = climb.length ? climb[0]!.pos : pos;
    const target = wps[this.covIndex]!;
    const link = cruisePath(this.air, from, { x: target[0], y: target[1] }, this.agl).slice(0, -1);
    const lead: Waypoint[] = [...climb, ...link.map((p) => ({ pos: p }))];
    const lane: Waypoint[] = wps.slice(this.covIndex).map((p) => ({ pos: p }));
    const base = this.covIndex;
    const inner = this.followJob(
      'coverage',
      [...lead, ...lane],
      () => {
        this.covIndex = wps.length;
        this.coverageDone = true;
        this.log('info', `Coverage complete: ${fmt(this.coverage.lengthM, 0)} m of lanes, ${this.discovered.size} building(s) found.`);
        this.continueMission();
      },
      1.5,
    );
    const job: Job = {
      phase: 'coverage',
      plan: () => inner.plan(),
      tick: (): VehicleCommand => {
        const cmd = inner.tick();
        // Keep the resume index in step with the lane waypoints already passed.
        const left = inner.plan().length;
        const passed = lead.length + lane.length - left - lead.length;
        if (passed > 0) this.covIndex = Math.max(this.covIndex, Math.min(wps.length, base + passed));
        return cmd;
      },
    };
    if (this.covIndex > 0) this.log('action', `Resuming coverage at waypoint ${this.covIndex + 1}/${wps.length}.`);
    else this.log('action', 'Starting area coverage.');
    this.setJob(job);
  }

  /** Decide what to do next once a building, confirmation, or lane set is finished. */
  private continueMission(): void {
    if (this.finished || this.returning) return;
    if (this.findMission() && requirementsMet(this.mission, this.confirmed)) {
      this.startReturn('all required targets confirmed');
      return;
    }
    if (this.pending.length) {
      const pos = this.vehicle.pos;
      this.pending.sort((a, b) => centerDist(a, pos) - centerDist(b, pos) || a.id.localeCompare(b.id));
      const next = this.pending.shift()!;
      this.startBuilding(next);
      return;
    }
    if (!this.coverageDone) {
      this.startCoverage();
      return;
    }
    if (this.mission.missionType === 'safety_sweep' && !this.outdoorClearChecked) {
      this.outdoorClearChecked = true;
      this.declareIfClear('outdoor');
    }
    this.startReturn(this.findMission() ? 'search complete, targets not all found' : 'sweep complete');
  }

  private findMission(): boolean {
    return this.mission.missionType === 'find_person' || this.mission.missionType === 'find_objects';
  }

  private sweptCategories(): ('person' | 'object')[] {
    const out: ('person' | 'object')[] = ['person'];
    if (this.mission.targets.some((t) => t.kind === 'object')) out.push('object');
    return out;
  }

  private declareIfClear(scope: ClearScope): boolean {
    const reports = this.sweptCategories().map((c) => ({ c, r: this.source.clearStatus(scope, c) }));
    const label = scope === 'outdoor' ? 'Outdoor area' : (this.world.buildings.find((b) => b.id === scope)?.label ?? scope);
    if (reports.every(({ r }) => r.status === 'clear')) {
      this.declaredClear.push({ scope, tSec: this.t });
      const r = reports[0]!.r;
      this.log('action', `${label} confirmed clear (${r.seenEmpty}/${r.total} cells seen empty).`);
      return true;
    }
    const bad = reports.find(({ r }) => r.status !== 'clear')!;
    if (bad.r.status === 'occupied') {
      this.log('warn', `${label} not clear: ${bad.r.occupied} cell(s) occupied for ${bad.c}.`);
    } else {
      this.log('warn', `${label} incomplete: ${bad.r.unseen} of ${bad.r.total} cells unseen for ${bad.c}. Not declaring clear.`);
    }
    return false;
  }

  // ----------------------------------------------------------- detection

  private detect(): void {
    const pose = this.vehicle.pose();
    // Building discovery by camera footprint and line of sight.
    if (!this.inside) {
      for (const b of senseBuildings(pose.pos, this.world, this.physics, this.discovered)) {
        this.discovered.add(b.id);
        this.pending.push(b);
        this.log('info', `${b.label} discovered ${fmt(centerDist(b, pose.pos), 0)} m away.`);
      }
      if (this.phase === 'coverage' && this.pending.length && !this.returning) {
        this.continueMission();
        return;
      }
    }
    // Openings seen while near the building.
    const v = this.visit;
    if (v && (this.phase === 'approach' || this.phase === 'inspect') && !this.inside) {
      for (const o of observeOpenings(pose.pos, pose.yaw, v.b, this.physics)) {
        if (!v.observed.has(o.id)) {
          v.observed.set(o.id, o);
          this.log('info', `Observed opening ${o.id} (${o.kind.replace('_', ' ')}, ${fmt(o.width, 2)} x ${fmt(o.height, 2)} m) on ${v.b.label}.`);
        }
      }
    }
    if (v && this.inside && v.vis) v.vis.observe(toLocal(v.grid, pose.pos), pose.yaw);

    const obs = this.source.observe(pose, this.t);
    for (const c of obs.contacts) {
      this.recent.push(c);
      if (this.recent.length > 60) this.recent.shift();
      this.tracker.ingest(c);
    }
    for (const track of this.tracker.tracks) {
      if (track.status !== 'open') continue;
      if (this.tracker.evaluate(track) === 'confirmed') {
        const cc = this.tracker.toConfirmed(track, this.t);
        this.confirmed.push(cc);
        this.log(
          'action',
          `Confirmed ${track.classId} at ${this.where(cc.estimate)} from ${cc.views} views, mean confidence ${fmt(cc.confidence, 2)}.`,
        );
        if (this.confirmTrack === track) this.endConfirm();
      }
    }
    if (this.finished || this.returning) return;
    // Start a confirmation when a mission-class contact appears.
    if (this.phase === 'coverage' || this.phase === 'indoor_explore') {
      for (const track of this.tracker.tracks) {
        if (track.status !== 'open' || (this.attempted.get(track.id) ?? 0) > this.t) continue;
        const indoor = this.phase === 'indoor_explore';
        if (indoor && v && track.buildingId !== v.b.id) continue;
        if (!indoor && track.buildingId !== null) continue;
        this.startConfirm(track);
        break;
      }
    }
  }

  // ----------------------------------------------------------- confirm

  private resumeAfterConfirm: (() => void) | null = null;

  private startConfirm(track: ConfirmTrack): void {
    this.attempted.set(track.id, Infinity);
    const need = Math.max(2, this.tracker.minViews - track.views.length + 1);
    const pos = this.vehicle.pos;
    const v = this.visit;
    const wps: Waypoint[] = [];
    if (this.inside && v) {
      const grid = v.grid;
      const est = toLocal(grid, track.estimate);
      const f = floorOf(grid, est[2]);
      const z = f * grid.H + INDOOR_FLY_Z;
      let cur = toLocal(grid, pos);
      let views: V3[] = [];
      for (const r of track.classId === 'person' ? [2.2, 2.8, 1.6] : [2.4, 2.0, 2.8]) {
        views = confirmViewpoints(track, est, cur, r, z, need, (p) => {
          const c = nearestNav(grid, p);
          if (!c || Math.hypot((c.x + 0.5) * grid.cs - p[0], (c.z + 0.5) * grid.cs - p[1]) > 0.75) return false;
          if (wallClearance(grid, f, p[0], p[1]) < PATH_CLEARANCE) return false;
          if (!gridLos(grid, f, p[0], p[1], est[0], est[1])) return false;
          return planIndoorPath(grid, cur, p) !== null;
        });
        if (views.length >= need) break;
      }
      for (const vp of views) {
        const path = planIndoorPath(grid, cur, vp);
        if (!path) continue;
        for (const p of path.slice(0, -1)) wps.push({ pos: toWorld(grid, p) });
        const at = toWorld(grid, vp);
        wps.push({ pos: at, yaw: headingTo(at, track.estimate), dwell: CONFIRM_DWELL });
        cur = vp;
      }
      const policy = this.policyRun;
      this.resumeAfterConfirm = () => (policy && this.policyRun === policy ? policy.resume() : this.exploreNext());
    } else {
      const z = cruiseZ(this.air, track.estimate[0], track.estimate[1], this.agl);
      const views = confirmViewpoints(track, track.estimate, pos, 8, z, need, (p) => {
        if (!operable(this.air, p[0], p[1])) return false;
        const d = sub(track.estimate, p);
        const hit = this.physics.cast(p[0], p[1], p[2], d[0], d[1], d[2], len(d));
        return !hit || hit.dist > len(d) - 1;
      });
      let cur: V3 = pos;
      for (const vp of views) {
        for (const p of cruisePath(this.air, cur, { x: vp[0], y: vp[1] }, this.agl).slice(0, -1)) wps.push({ pos: p });
        wps.push({ pos: vp, yaw: headingTo(vp, track.estimate), dwell: CONFIRM_DWELL });
        cur = vp;
      }
      this.resumeAfterConfirm = () => this.startCoverage();
    }
    if (!wps.some((w) => w.dwell)) {
      // Nowhere to re-view from yet. Keep the track open and try again later.
      this.attempted.set(track.id, this.t + 6);
      const resume = this.resumeAfterConfirm;
      this.resumeAfterConfirm = null;
      resume?.();
      return;
    }
    this.confirmTrack = track;
    this.log(
      'action',
      `Contact ${track.classId} (${fmt(track.best.confidence, 2)}) at ${this.where(track.estimate)}. Re-viewing from ${wps.filter((w) => w.dwell).length} more bearing(s).`,
    );
    this.setJob(
      this.followJob(
        'confirm',
        wps,
        () => {
          const rounds = (this.confirmRounds.get(track.id) ?? 0) + 1;
          this.confirmRounds.set(track.id, rounds);
          if (track.status === 'open' && rounds < CONFIRM_ROUNDS) {
            this.attempted.set(track.id, this.t + 4);
            this.log('info', `${track.classId} contact still unconfirmed (${track.views.length} view(s), ${fmt(this.tracker.meanConfidence(track), 2)}). Will look again.`);
          } else if (track.status === 'open') {
            this.tracker.reject(track);
            this.log(
              'warn',
              `Rejected ${track.classId} contact: ${track.views.length} view(s), mean confidence ${fmt(this.tracker.meanConfidence(track), 2)} after re-views.`,
            );
          }
          this.endConfirm();
        },
        this.inside ? 0.2 : 1.5,
      ),
    );
  }

  private endConfirm(): void {
    this.confirmTrack = null;
    const resume = this.resumeAfterConfirm;
    this.resumeAfterConfirm = null;
    if (this.findMission() && requirementsMet(this.mission, this.confirmed)) {
      this.startReturn('all required targets confirmed');
      return;
    }
    if (resume) resume();
  }

  // ----------------------------------------------------------- building

  private startBuilding(b: MissionBuilding): void {
    this.visitedBuildings.add(b.id);
    const box = buildingBox(b);
    const v: Visit = {
      b,
      box,
      grid: indoorGrid(b),
      standoff: null,
      observed: new Map(),
      ranked: [],
      candIdx: 0,
      denied: 0,
      opening: null,
      axis: null,
      aborts: 0,
      vis: null,
      explorer: null,
      exploreStart: 0,
      pass: 0,
      policyCollisions: 0,
      entered: false,
      trail: [],
    };
    this.visit = v;
    const plan = planApproach(b, this.vehicle.pos, this.physics, this.air, this.agl);
    if (!plan.ok || !plan.standoff) {
      this.log('warn', `${b.label}: ${plan.reason}. Skipping.`);
      this.visit = null;
      this.continueMission();
      return;
    }
    v.standoff = plan.standoff;
    this.log('action', `Approaching ${b.label}. Standoff ${this.where(plan.standoff.pos)}, open descent column checked.`);
    const wps: Waypoint[] = [
      ...plan.cruise.map((p) => ({ pos: p, tol: 1.5 })),
      ...plan.descent.map((p, i) => ({ pos: p, tol: 0.3, yaw: i === plan.descent.length - 1 ? plan.standoff!.yaw : null })),
    ];
    this.setJob(this.followJob('approach', wps, () => this.startInspect(), 0.3));
  }

  private startInspect(): void {
    const v = this.visit!;
    const steps = planInspection(v.b, this.vehicle.pos, this.physics, this.air);
    this.log('action', `Inspecting the ${v.b.label} envelope from ${steps.filter((s) => s.dwell > 0).length} viewpoints.`);
    const wps: Waypoint[] = steps.map((s) => ({ pos: s.pos, yaw: s.yaw, dwell: s.dwell || undefined }));
    this.setJob(this.followJob('inspect', wps, () => this.selectEntry(), 0.25));
  }

  private selectEntry(): void {
    const v = this.visit!;
    this.setPhase('entry_select');
    const cands = [...v.observed.values()]
      .sort((a, b) => a.id - b.id)
      .map((o) => scoreOpening(v.b, o, this.physics, this.mission.entryPolicy));
    for (const c of cands) {
      this.log(
        'info',
        `Opening ${c.openingId} ${c.kind.replace('_', ' ')}: clearance ${fmt(c.clearanceM, 2)} m, safety ${fmt(c.safety, 2)}, ${c.decision}.`,
      );
    }
    v.ranked = rankCandidates(cands);
    v.candIdx = 0;
    if (!cands.length) {
      this.log('warn', `No openings observed on ${v.b.label}.`);
    }
    if (this.mission.entryPolicy === 'never') {
      const best = [...cands].sort((a, b) => b.safety - a.safety)[0];
      this.entries.push({ tSec: this.t, buildingId: v.b.id, openingId: best?.openingId ?? -1, safety: best?.safety ?? 0, decision: 'never' });
      this.log('info', `Entry policy is never. Not entering ${v.b.label}.`);
      this.leaveBuilding();
      return;
    }
    this.tryCandidate();
  }

  private tryCandidate(): void {
    const v = this.visit!;
    const cand = v.ranked[v.candIdx];
    if (!cand) {
      if (v.denied === 0) {
        const best = [...v.observed.values()][0];
        this.entries.push({ tSec: this.t, buildingId: v.b.id, openingId: best?.id ?? -1, safety: 0, decision: 'none_viable' });
      }
      this.log('warn', `No viable entry into ${v.b.label}. Skipping the interior.`);
      this.leaveBuilding();
      return;
    }
    if (cand.decision === 'auto') {
      this.entries.push({ tSec: this.t, buildingId: v.b.id, openingId: cand.openingId, safety: cand.safety, decision: 'auto' });
      this.log('action', `Entering ${v.b.label} through opening ${cand.openingId} (${cand.kind.replace('_', ' ')}), safety ${fmt(cand.safety, 2)}.`);
      this.goEnter(cand);
      return;
    }
    this.pendingEntry = cand;
    this.operatorAnswer = null;
    this.log('action', `Operator approval needed: opening ${cand.openingId} (${cand.kind.replace('_', ' ')}) on ${v.b.label}, safety ${fmt(cand.safety, 2)}.`);
    const at: V3 = [this.vehicle.pos[0], this.vehicle.pos[1], this.vehicle.pos[2]];
    this.setJob({
      phase: 'await_operator',
      plan: () => [],
      tick: (): VehicleCommand => {
        let answer = this.operatorAnswer;
        if (!answer && this.options.operator) {
          const r = this.options.operator(cand);
          if (r !== 'pending') answer = r;
        }
        if (answer) {
          this.operatorAnswer = null;
          this.pendingEntry = null;
          if (answer === 'approve') {
            this.entries.push({ tSec: this.t, buildingId: v.b.id, openingId: cand.openingId, safety: cand.safety, decision: 'approved' });
            this.log('action', `Operator approved opening ${cand.openingId}.`);
            this.goEnter(cand);
          } else {
            this.entries.push({ tSec: this.t, buildingId: v.b.id, openingId: cand.openingId, safety: cand.safety, decision: 'denied' });
            this.log('info', `Operator denied opening ${cand.openingId}.`);
            v.denied += 1;
            v.candIdx += 1;
            this.tryCandidate();
          }
        }
        return { vel: hold(this.vehicle.pos, at) };
      },
    });
  }

  private goEnter(cand: EntryCandidate): void {
    const v = this.visit!;
    const opening = v.b.shell.openings.find((o) => o.id === cand.openingId)!;
    v.opening = opening;
    v.axis = passageAxis(opening);
    v.aborts = 0;
    const ft = new FlyThrough(v.axis, 'in');
    const hop = v.box.roofZ + 2.5;
    let leg = legBetween(this.physics, this.vehicle.pos, ft.start, hop);
    const refuge = v.standoff?.pos;
    if (!leg && refuge) {
      // Retrace the inspection path until the alignment point is reachable.
      const back = this.retrace(v.trail, (p) => legBetween(this.physics, p, ft.start, hop) !== null);
      const end = back.length ? back[back.length - 1]!.pos : this.vehicle.pos;
      const tail = legBetween(this.physics, end, ft.start, hop);
      if (tail) leg = [...back.map((w) => w.pos), ...tail];
    }
    if (!leg) {
      this.log('warn', `No clear path to the alignment point of opening ${opening.id}.`);
      v.candIdx += 1;
      this.tryCandidate();
      return;
    }
    this.setJob(this.followJob('fly_through', leg.map((p) => ({ pos: p })), () => this.crossIn(), 0.25));
  }

  private flyThroughJob(ft: FlyThrough, onDone: (ok: boolean) => void): Job {
    let called = false;
    return {
      phase: this.phase === 'exit' ? 'exit' : 'fly_through',
      plan: () => [ft.start, ft.end],
      tick: (): VehicleCommand => {
        const s = ft.step(this.vehicle.pos, this.vehicle.vel, this.lastCollided);
        if ((s.state === 'done' || s.state === 'aborted') && !called) {
          called = true;
          onDone(s.state === 'done');
        }
        return s.cmd;
      },
    };
  }

  private crossIn(): void {
    const v = this.visit!;
    const ft = new FlyThrough(v.axis!, 'in');
    this.setPhase('fly_through');
    this.setJob(
      this.flyThroughJob(ft, (ok) => {
        if (ok) {
          this.inside = v.b.id;
          v.entered = true;
          this.log(
            'action',
            `Inside ${v.b.label}. Centerline error peaked at ${fmt(ft.maxErrU, 3)} / ${fmt(ft.maxErrV, 3)} m (margin ${fmt(v.axis!.marginU, 2)} / ${fmt(v.axis!.marginV, 2)} m).`,
          );
          this.startExplore();
          return;
        }
        v.aborts += 1;
        this.log('warn', `Fly-through aborted: ${ft.abortReason}. Backed out.`);
        if (v.aborts < 3) {
          this.crossIn();
        } else {
          v.candIdx += 1;
          this.tryCandidate();
        }
      }),
    );
  }

  // ----------------------------------------------------------- indoor

  private startExplore(): void {
    const v = this.visit!;
    v.vis = new IndoorVisibility(v.grid);
    v.explorer = new FrontierExplorer(v.grid, v.vis);
    v.exploreStart = this.t;
    v.pass = 0;
    v.policyCollisions = 0;
    v.vis.observe(toLocal(v.grid, this.vehicle.pos), this.vehicle.yaw);
    if (this.policy) {
      this.startPolicy();
      return;
    }
    this.log('info', `Frontier exploration of ${v.b.label}: ${v.vis.cells.length} cells to see.`);
    this.exploreNext();
  }

  /** The running policy explorer, so a confirmation can hand control back and an exit can free it. */
  private policyRun: { resume: () => void; release: () => void } | null = null;

  private startPolicy(): void {
    const v = this.visit!;
    const physicsWorld = new PhysicsWorld(v.grid.spec);
    const lighting: LightingState = {
      timeOfDay: this.world.timeOfDay,
      interiorLightsOn: v.b.interiorLightsOn,
      flashlightOn: this.world.flashlightOn,
      sensorMode: this.world.sensorMode,
    };
    const pe = new PolicyExplorer(v.grid, this.policy!, physicsWorld, lighting, this.world.seed);
    pe.lastNewVoxelSec = this.t;
    const start = this.t;
    let ended = false;
    this.log('info', `Trained policy exploring ${v.b.label}.`);
    // The record credits the policy unless a later fallback hands the building to the frontier planner.
    if (!this.policyFell) this.policyClean = true;
    const release = () => {
      if (ended) return;
      ended = true;
      physicsWorld.free();
      this.policyRun = null;
    };
    const end = (why: string, fallback: boolean) => {
      if (ended) return;
      release();
      if (fallback) {
        this.policyClean = false;
        this.policyFell = true;
        this.log('warn', `Policy explorer ${why}. Falling back to frontier exploration.`);
      } else {
        this.log('info', `Policy explorer ${why}.`);
      }
      this.exploreNext();
    };
    const job: Job = {
      phase: 'indoor_explore',
      plan: () => [],
      tick: (): VehicleCommand => {
        const local = toLocal(v.grid, this.vehicle.pos);
        if (this.t - pe.lastNewVoxelSec > POLICY_STALL_SEC) end(`stalled (no new coverage for ${POLICY_STALL_SEC} s)`, true);
        else if (v.policyCollisions >= POLICY_MAX_COLLISIONS) end(`collided ${v.policyCollisions} times`, true);
        else if (v.vis!.coverage() >= 0.97) end(`saw ${fmt(v.vis!.coverage() * 100, 0)}% of cells`, false);
        else if (this.t - start > POLICY_MAX_SEC) end(`reached its ${POLICY_MAX_SEC} s slot`, false);
        if (ended) return { vel: [0, 0, 0] };
        const cmd = pe.act(local, this.vehicle.vel, this.vehicle.yaw, this.t);
        return { vel: cmd.vel, yawRate: cmd.yawRate };
      },
    };
    this.policyRun = {
      resume: () => {
        // Time spent confirming does not count as a stall.
        pe.lastNewVoxelSec = this.t;
        this.setJob(job);
      },
      release,
    };
    this.setJob(job);
  }

  private exploreNext(): void {
    const v = this.visit;
    if (!v || !v.vis || !v.explorer) return;
    this.frontierRan = true;
    if (this.t - v.exploreStart > MAX_EXPLORE_SEC) {
      this.log('warn', `Exploration time slot for ${v.b.label} used up at ${fmt(v.vis.coverage() * 100, 0)}% seen.`);
      this.explorationDone();
      return;
    }
    const local = toLocal(v.grid, this.vehicle.pos);
    const goal = v.explorer.next(local);
    if (!goal) {
      this.explorationDone();
      return;
    }
    const wps: Waypoint[] = goal.path.map((p, i) => ({
      pos: toWorld(v.grid, p),
      spin: i === goal.path.length - 1 && goal.spin,
    }));
    this.setJob(this.followJob('indoor_explore', wps, () => this.exploreNext(), 0.2));
  }

  private explorationDone(): void {
    const v = this.visit!;
    // An open contact in this building gets one more look before leaving.
    const open = this.tracker.tracks.find(
      (tr) => tr.status === 'open' && tr.buildingId === v.b.id && this.attempted.get(tr.id) !== Infinity && !this.finalLook.has(tr.id),
    );
    if (open && !this.returning) {
      this.finalLook.add(open.id);
      this.startConfirm(open);
      return;
    }
    this.log('info', `${v.b.label} explored: ${fmt(v.vis!.coverage() * 100, 0)}% of cells seen by the planner.`);
    if (this.mission.missionType === 'safety_sweep') {
      const ok = this.declareIfClear(v.b.id);
      const timeLeft = this.maxSim - this.t;
      if (!ok && v.pass === 0 && timeLeft > 240) {
        v.pass = 1;
        v.vis = new IndoorVisibility(v.grid, 3.5);
        v.explorer = new FrontierExplorer(v.grid, v.vis, 2.5);
        v.exploreStart = this.t;
        this.log('action', `Second, closer pass through ${v.b.label}.`);
        this.exploreNext();
        return;
      }
    }
    this.startExit('continue');
  }

  // ----------------------------------------------------------- exit and home

  private startExit(after: 'continue' | 'rth'): void {
    this.policyRun?.release();
    const v = this.visit;
    if (!v || !this.inside || !v.axis) {
      if (after === 'rth') this.startReturnOutside();
      else this.leaveBuilding();
      return;
    }
    const ft = new FlyThrough(v.axis, 'out');
    const local = toLocal(v.grid, this.vehicle.pos);
    const path = planIndoorPath(v.grid, local, toLocal(v.grid, ft.start)) ?? [];
    const wps: Waypoint[] = path.map((p) => ({ pos: toWorld(v.grid, p) }));
    this.log('action', `Leaving ${v.b.label} through opening ${v.opening?.id ?? -1}.`);
    this.setJob(this.followJob('exit', wps, () => this.crossOut(after), 0.2));
  }

  private crossOut(after: 'continue' | 'rth'): void {
    const v = this.visit!;
    const ft = new FlyThrough(v.axis!, 'out');
    this.setPhase('exit');
    this.setJob(
      this.flyThroughJob(ft, (ok) => {
        if (ok) {
          this.inside = null;
          this.log('action', `Outside ${v.b.label}.`);
          if (after === 'rth' || this.returning) this.startReturnOutside();
          else this.leaveBuilding();
          return;
        }
        v.aborts += 1;
        this.log('warn', `Exit fly-through aborted: ${ft.abortReason}. Retrying.`);
        if (v.aborts < 8) this.crossOut(after);
        else this.startLand('cannot exit the building');
      }),
    );
  }

  /** Climb away from the current building to cruise, then continue. */
  private leaveBuilding(): void {
    const route = this.climbRoute();
    this.setJob(
      this.followJob(
        'exit',
        route,
        () => {
          this.visit = null;
          if (this.returning) this.startReturnOutside();
          else this.continueMission();
        },
        0.3,
      ),
    );
  }

  private climbRoute(): Waypoint[] {
    const pos = this.vehicle.pos;
    const top = cruiseZ(this.air, pos[0], pos[1], this.agl);
    if (pos[2] >= top - 0.5) return [];
    if (columnClear(this.physics, pos[0], pos[1], pos[2], top, 0.45)) return [{ pos: [pos[0], pos[1], top] }];
    const open = (p: V3) => columnClear(this.physics, p[0], p[1], p[2], cruiseZ(this.air, p[0], p[1], this.agl), 0.45);
    const back = this.retrace(this.visit?.trail.length ? this.visit.trail : this.crumbs, open);
    const end = back.length ? back[back.length - 1]!.pos : pos;
    if (back.length && !open(end)) this.log('warn', 'No open column found on the way back. Climbing anyway.');
    return [...back, { pos: [end[0], end[1], cruiseZ(this.air, end[0], end[1], this.agl)] }];
  }

  private startReturn(reason: string): void {
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

  private startReturnOutside(): void {
    this.returning = true;
    const climb = this.climbRoute();
    const from = climb.length ? climb[climb.length - 1]!.pos : this.vehicle.pos;
    const plan = planReturn(this.physics, this.air, from, this.world.home, null, this.agl);
    const wps: Waypoint[] = [...climb.map((w) => ({ ...w, tol: 0.3 })), ...plan.waypoints.map((p) => ({ pos: p, tol: 1.5 }))];
    this.visit = null;
    this.setJob(this.followJob('return_home', wps, () => this.startLand('home'), 1.5));
  }

  private startLand(reason: string): void {
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

  private touchdown(): void {
    if (this.finished) return;
    this.landedHome = !this.inside && landedAtHome(this.vehicle.pos, this.world.home);
    this.log('action', this.landedHome ? 'Landed at home.' : `Landed away from home at ${this.where(this.vehicle.pos)}.`);
    this.finish();
  }

  private finish(): void {
    this.policyRun?.release();
    if (this.finished) return;
    this.outcome = this.computeOutcome();
    this.setPhase('done');
    const last = this.phases[this.phases.length - 1];
    if (last) last.tEnd = this.t;
    this.recordTrack();
    this.finished = true;
    this.log('info', `Mission ${this.outcome}. ${fmt(this.t, 0)} s, battery ${fmt(this.battery.remainingPct(), 0)}% left, ${this.collisions.length} collision(s).`);
    this.job = this.holdJob('done', 1e9, () => undefined);
  }

  private computeOutcome(): RunOutcome {
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

  private distanceHome(pos: V3): number {
    const home = this.world.home;
    if (this.inside && this.visit?.opening) {
      const c = this.visit.opening.center as V3;
      return dist(pos, c) + Math.hypot(c[0] - home[0], c[1] - home[1]);
    }
    return Math.hypot(pos[0] - home[0], pos[1] - home[1]);
  }

  private checkBudgets(): void {
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

  private checkSafety(pos: V3): void {
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

  private handleSafety(action: SafetyAction): void {
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

function centerDist(b: MissionBuilding, p: V3): number {
  const box = buildingBox(b);
  return Math.hypot(box.cx - p[0], box.cy - p[1]);
}

function hold(pos: V3, at: V3): V3 {
  const k = 1.5;
  return [(at[0] - pos[0]) * k, (at[1] - pos[1]) * k, (at[2] - pos[2]) * k];
}

/** Velocity along `dir` at `speed`, scaled uniformly so both caps hold and the path stays straight. */
function scaleTo(dir: V3, speed: number, limits: VehicleLimits): V3 {
  const l = len(dir);
  if (l < 1e-9) return [0, 0, 0];
  const s = speed / l;
  const v: V3 = [dir[0] * s, dir[1] * s, dir[2] * s];
  let f = 1;
  const h = Math.hypot(v[0], v[1]);
  if (h > limits.speedH) f = Math.min(f, limits.speedH / h);
  if (Math.abs(v[2]) > limits.speedV) f = Math.min(f, limits.speedV / Math.abs(v[2]));
  return [v[0] * f, v[1] * f, v[2] * f];
}

/** Build a this. Rapier is initialised here, so it is async. */
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

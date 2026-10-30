/**
 * Contracts for autonomous woods-to-building missions.
 *
 * Frame: world x east, y north, z up, meters. This is the OutdoorWorld frame
 * and the Mission frame (Mission points are the x/y of this frame).
 * Building-local frames are world minus `shell.origin`; BuildingSpec,
 * placeIndoor targets, and perception occlusion use the local frame.
 *
 * Ownership:
 *   world.ts, perception.ts            world and detection providers
 *   planner/*, vehicle.ts, runner.ts   hierarchical planner and runner
 *   battery.ts, safety.ts, report.ts   energy, safety monitor, report metrics
 *   scenarios.ts, src/ui/demo/*        demo page and seeded scenarios
 * Change this file only by agreement; every module codes against it.
 */
import type { ClearReport } from '../../perception/seenEmpty';
import type { SensorMode } from '../../shared/types';
import type { ObjectId, PersonState, TargetCategory, TargetClass, TargetScene } from '../../targets/types';
import type { BuildingShell, Opening, OutdoorWorld } from '../../world/outdoor/types';
import type { Mission, MissionType, Point } from '../types';
import type { Fault, FaultTransition } from './faults';
import type { PlannerConfig } from './sar/planner';
import type { LocVisitRecord } from './localizer';
import type { LocSensorHook } from './locSensors';
import type { ActingPolicy } from './planner/indoor';

export type V3 = [number, number, number];

// ---------------------------------------------------------------- world

export interface MissionBuilding {
  /** `bldg-<shell.id>`. Also the WorldFootprint id. */
  id: string;
  label: string;
  shell: BuildingShell;
  /** World-frame outer footprint, counter-clockwise. */
  footprint: Point[];
  /** Roof height above the pad. */
  heightM: number;
  /** Targets in this building's local frame (world minus shell.origin). */
  scene: TargetScene;
  /** One entry per room, indexed like target.roomIndex / indoorCells roomIndex. */
  interiorLightsOn: Uint8Array;
}

export interface MissionWorld {
  seed: number;
  outdoor: OutdoorWorld;
  buildings: MissionBuilding[];
  /** Outdoor targets in the world frame. */
  outdoorScene: TargetScene;
  timeOfDay: number;
  flashlightOn: boolean;
  sensorMode: SensorMode;
  /** Takeoff and landing pad, world frame. */
  home: V3;
  homeYaw: number;
}

export interface TargetWant {
  classId: 'person' | ObjectId;
  state?: PersonState;
  where: 'indoor' | 'outdoor';
  /** Building index for indoor targets. Defaults to 0. */
  building?: number;
}

export interface MissionWorldOptions {
  seed: number;
  /** 1..4. Omitted uses the outdoor generator's seeded count. */
  buildings?: number;
  timeOfDay?: number;
  flashlightOn?: boolean;
  sensorMode?: SensorMode;
  interiorLights?: 'on' | 'off' | 'seeded';
  /** Exact targets to plant. An empty list means the area is truly clear. */
  targets?: TargetWant[];
  /** Look-alike decoys (mannequin, coat, ...) to plant indoors. Default 0. */
  decoys?: number;
  /**
   * Seed for where targets and decoys are planted and which rooms are lit, apart from the layout.
   * Defaults to `seed`. The failure atlas varies the two independently.
   */
  placementSeed?: number;
}

export interface MissionForWorldOptions {
  missionType: MissionType;
  name?: string;
  targets?: Mission['targets'];
  requiredConfidence?: number;
  entryPolicy?: Mission['entryPolicy'];
  budgets?: Mission['budgets'];
}

// ------------------------------------------------------------ detection

export interface DronePose {
  pos: V3;
  yaw: number;
  vel: V3;
}

/** One detector hit from one frame. */
export interface Contact {
  id: string;
  tSec: number;
  /** Reported class. Can be wrong (confusion) or a decoy look-alike. */
  classId: TargetClass;
  confidence: number;
  /** Estimated world position from the bbox ray and range. Planners use this. */
  estimate: V3;
  range: number;
  /** Viewing direction, unit, world frame. Multi-view confirmation uses it. */
  bearing: V3;
  buildingId: string | null;
  /**
   * What the camera measured, in the drone's heading frame (x along the nose, y to the left, z up):
   * the camera's offset from the drone, the unit ray through the box centre, and the range. This is
   * all a real drone gets from a detection; the runner places the contact with its own pose
   * (perception.ts placeContact). `estimate`, `range` and `bearing` above are the sensor's
   * world-frame view of the same measurement, for tools without a pose of their own.
   */
  sensor: { eye: V3; ray: V3; range: number };
  /** Ground truth for scoring only. Planners must not read it. */
  truth: { targetId: string; trueClass: TargetClass; kind: 'tp' | 'fp' | 'confused' };
}

export interface Observation {
  tSec: number;
  contacts: Contact[];
}

export interface TruthTarget {
  id: string;
  classId: TargetClass;
  category: TargetCategory;
  buildingId: string | null;
  world: V3;
}

/** `outdoor` or a building id. */
export type ClearScope = string;

export interface LiveDetectionSource {
  readonly id: string;
  /** Run the camera detector from this pose. Also updates seen-empty maps. */
  observe(pose: DronePose, tSec: number): Observation;
  /**
   * Ground-truth confirm-clear status for a scope, for scoring only: it knows where the targets are.
   * The mission decides clear from its own look estimate (phases/search.ts ownClear) and never calls it.
   */
  truthClearStatus(scope: ClearScope, category: 'person' | 'object'): ClearReport;
  /** Ground truth, for the report only. */
  truthTargets(): TruthTarget[];
}

// -------------------------------------------------------------- vehicle

export interface BatteryModel {
  readonly capacityWh: number;
  /** Electrical power for a velocity and acceleration, watts. */
  powerFor(vel: V3, accel: V3): number;
  draw(powerW: number, dtSec: number): void;
  remainingPct(): number;
  usedPct(): number;
  /** Percent of the pack needed to fly `distanceM` horizontally and climb `climbM`, at cruise. */
  costPct(distanceM: number, climbM: number): number;
}

// --------------------------------------------------------------- safety

export type SafetyActionKind = 'none' | 'hold' | 'backtrack' | 'exit_building' | 'return_home' | 'land';

export interface SafetyAction {
  kind: SafetyActionKind;
  reason: string;
  /** For hold. */
  seconds?: number;
  /** For backtrack. */
  meters?: number;
}

export interface SafetyEvent {
  tSec: number;
  kind:
    | 'geofence'
    | 'no_fly'
    | 'altitude'
    | 'battery_reserve'
    | 'battery_critical'
    | 'collision'
    // Raised by the fault failsafe (failsafe.ts).
    | 'comms_loss'
    | 'sensor_fault'
    | 'motor_fault'
    | 'battery_fault'
    | 'gnss_loss'
    | 'estimator_fault'
    | 'wind';
  message: string;
  action: SafetyActionKind;
}

export interface SafetyInput {
  tSec: number;
  pose: DronePose;
  /** Where the current command would put the drone after `lookaheadSec`. */
  predicted: V3;
  battery: BatteryModel;
  /** Straight-line route length home, through the exit opening when inside. */
  distanceHomeM: number;
  collided: boolean;
  /** Building id while inside a shell. */
  inside: string | null;
  phase: Phase;
}

export interface SafetyMonitor {
  /** The strongest action needed now. `none` when flight is nominal. */
  check(input: SafetyInput): SafetyAction;
  events(): SafetyEvent[];
}

// -------------------------------------------------------------- planner

export type Phase =
  | 'preflight'
  | 'coverage'
  | 'approach'
  | 'inspect'
  | 'entry_select'
  | 'await_operator'
  | 'fly_through'
  | 'indoor_explore'
  | 'confirm'
  | 'exit'
  | 'return_home'
  | 'land'
  | 'recover'
  | 'done';

export interface EntryCandidate {
  buildingId: string;
  openingId: number;
  kind: Opening['kind'];
  clearWidth: number;
  clearHeight: number;
  /** Clearance margin over the airframe, meters (min of width and height margins). */
  clearanceM: number;
  /** 0..1. Combines clearance, approach obstruction, opening type, and leaf hazards. */
  safety: number;
  decision: 'auto' | 'ask' | 'never';
}

export interface ConfirmedContact {
  contactId: string;
  classId: TargetClass;
  confidence: number;
  views: number;
  estimate: V3;
  tSec: number;
  buildingId: string | null;
  truth: Contact['truth'];
}

export interface EntryRecord {
  tSec: number;
  buildingId: string;
  openingId: number;
  safety: number;
  decision: 'auto' | 'approved' | 'denied' | 'never' | 'none_viable';
}

export interface LogEntry {
  tSec: number;
  tone: 'info' | 'warn' | 'action';
  message: string;
}

export type RunOutcome = 'found' | 'clear' | 'not_found' | 'incomplete' | 'aborted' | 'landed';

/** Everything the runner saw and decided. The report is derived from this. */
export interface RunRecord {
  mission: Mission;
  seed: number;
  outcome: RunOutcome;
  tSec: number;
  track: { t: number; pos: V3; phase: Phase }[];
  phases: { phase: Phase; tStart: number; tEnd: number }[];
  confirmed: ConfirmedContact[];
  /** Scopes the planner declared clear, and when. */
  declaredClear: { scope: ClearScope; tSec: number }[];
  collisions: { tSec: number; pos: V3; inside: string | null }[];
  entries: EntryRecord[];
  safetyEvents: SafetyEvent[];
  batteryStartPct: number;
  batteryEndPct: number;
  /** True when the drone landed within 1.5 m of home. */
  landedHome: boolean;
  indoorExplorer: 'policy' | 'frontier' | 'oracle' | null;
  /** Per building visit flown on the estimated pose; empty when the run used the true pose. */
  localization: LocVisitRecord[];
  log: LogEntry[];
  /** Injected fault starts and ends (ground truth). Empty without faults. */
  faults?: FaultTransition[];
  /**
   * Probability search only: the model's cumulative probability of success every 5 s, the share of
   * the prior within aerial reach (inside the fence, off crown-covered ground), and the replans.
   */
  search?: { pos: { tSec: number; pos: number }[]; reachable: number; replans: number };
  /** With RunnerOptions.measureClearance: the closest the airframe came to any solid but the ground. */
  clearance?: ClearanceRecord;
}

/** Closest approach of the drone's sphere to a solid (the ground excluded), and where it happened. */
export interface ClearanceRecord {
  /** Gap, m; zero or below is contact. */
  minM: number;
  tSec: number;
  pos: V3;
  phase: Phase;
  inside: string | null;
  /** Collider tag of the solid: wall, floor, door, trunk, canopy, undergrowth. */
  tag: string;
}

export interface RunnerSnapshot {
  tSec: number;
  phase: Phase;
  pose: DronePose;
  batteryPct: number;
  inside: string | null;
  plan: V3[];
  track: V3[];
  contacts: Contact[];
  confirmed: ConfirmedContact[];
  pendingEntry: EntryCandidate | null;
  log: LogEntry[];
  done: boolean;
}

export interface RunnerOptions {
  /** Control tick, seconds. Default 0.05. */
  dt?: number;
  /** Hard stop on simulated time. Default mission.budgets.timeSec. */
  maxSimSec?: number;
  /**
   * Operator answer for `ask` entries. Returning 'pending' leaves the
   * request open until approve() or deny(). Default: 'pending'.
   */
  operator?: (candidate: EntryCandidate) => 'approve' | 'deny' | 'pending';
  /**
   * Indoor exploration mode. 'auto' uses the trained policy when given, else
   * the sensor-only frontier explorer. 'frontier' forces that explorer.
   * 'oracle' plans on the true floor plan: a reference for benchmarks, not
   * something a real drone could fly.
   */
  indoor?: 'auto' | 'frontier' | 'policy' | 'oracle';
  /** Optional trained policy (policy.ts NormalizedPolicy or an InferPolicy): anything with a mean action. */
  policy?: ActingPolicy;
  /**
   * Airframe model. 'kinematic' (default) follows commands within the
   * acceleration caps; 'quad' flies a 1.2 kg quadrotor on rigid-body dynamics
   * and an angle-mode flight controller, and indoors uses the braking filter.
   */
  vehicle?: 'kinematic' | 'quad';
  /** Wind at sim time t, world frame, m/s. Default calm. */
  wind?: (tSec: number) => V3;
  /**
   * Faults to inject (faults.ts), for the fault bench and the demo. The
   * failsafe always runs; without faults it only watches the healthy drone.
   * `seed` defaults to the world seed.
   */
  faults?: { plan: readonly Fault[]; seed?: number };
  /**
   * Where the drone's indoor pose comes from. 'truth' (default) hands the
   * planners the simulator's pose. 'estimated' flies every non-oracle indoor
   * visit on the src/loc estimator (IMU, optical flow, lidar scan matching):
   * maps, paths, the shield and the controller all run in the estimate's frame.
   * The fly-through in and out stays on the truth, as if the drone servoed on
   * the opening it can see.
   */
  localization?: 'truth' | 'estimated';
  /**
   * Test hook: rewrites the localization sensor readings in place before the estimator reads them.
   * Faults the benches and the demo inject belong in `faults`; this is for perturbations no fault
   * kind models (tests/unit/autonomy.localization.test.ts turns scans against the map).
   */
  locFault?: LocSensorHook;
  /** Outdoor search pattern. Default: lanes. */
  search?: OutdoorSearchOptions;
  /** Track the closest approach to any solid every tick (RunRecord.clearance). Off by default: it costs a query per tick. */
  measureClearance?: boolean;
}

/**
 * How the drone searches outdoors. 'lanes' flies the lawnmower sweep and
 * visits each building as it is found. 'probability' (find-person missions
 * only) plans on a probability-of-area map built from the last known point
 * and lost-person behavior, updated by what the camera saw (src/mission/autonomy/sar).
 */
export interface OutdoorSearchOptions {
  mode: 'lanes' | 'probability';
  /** Where the missing person was last seen, world x and y. Default: home. */
  lastKnown?: [number, number];
  /** Probability search prior. Default 'behavior'. */
  prior?: 'behavior' | 'ring' | 'uniform';
  /** Probability search planner overrides. */
  planner?: Partial<PlannerConfig>;
  /** Lanes: spacing between lanes, meters. Default from the camera footprint. */
  laneSpacingM?: number;
  /** Lanes: fly every lane before visiting any building. Default false. */
  buildingsLast?: boolean;
}

export interface MissionRunner {
  step(dtSec?: number): void;
  approve(): void;
  deny(): void;
  abort(): void;
  snapshot(): RunnerSnapshot;
  record(): RunRecord;
  readonly done: boolean;
}

// --------------------------------------------------------------- report

export interface AutonomyReport {
  missionName: string;
  missionType: MissionType;
  outcome: RunOutcome;
  durationSec: number;
  /** First confirmed true positive of a mission target class. Null when none. */
  timeToFindSec: number | null;
  /** Operable geofence area seen by the camera footprint, 0..1. */
  outdoorCoverage: number;
  /** Indoor cells resolved by the camera, 0..1, over all entered or required buildings. */
  indoorCoverage: number;
  perBuildingCoverage: Record<string, number>;
  /** Scopes declared clear that held a truth target of the swept category. */
  falseClears: number;
  /** Confirmed contacts that are not a truth target of the reported class. */
  falseAlarms: number;
  /** Truth targets of a mission class that were never confirmed. */
  missed: number;
  collisions: number;
  batteryUsedPct: number;
  landedHome: boolean;
  entries: EntryRecord[];
  safetyEvents: number;
  notes: string[];
}

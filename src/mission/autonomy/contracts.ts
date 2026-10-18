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
  /** Confirm-clear status for a scope. Unseen cells keep a scope incomplete. */
  clearStatus(scope: ClearScope, category: 'person' | 'object'): ClearReport;
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
  kind: 'geofence' | 'no_fly' | 'altitude' | 'battery_reserve' | 'battery_critical' | 'collision';
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
  indoorExplorer: 'policy' | 'frontier' | null;
  log: LogEntry[];
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
  /** Indoor exploration mode. 'auto' uses the trained policy when given, else frontier. */
  indoor?: 'auto' | 'frontier' | 'policy';
  /** Optional trained policy weights (InferPolicy). */
  policy?: unknown;
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

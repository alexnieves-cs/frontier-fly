/** Constants, job/visit shapes and small helpers shared by the mission runner phases. */
import type { Opening } from '../../../world/outdoor/types';
import type {
  EntryCandidate,
  MissionBuilding,
  Phase,
  SafetyActionKind,
  V3,
} from '../contracts';
import type { Standoff } from '../planner/approach';
import type { PassageAxis } from '../planner/entry';
import { buildingBox, type BuildingBox, len } from '../planner/geometry';
import type { FrontierExplorer, IndoorGrid, IndoorNav, IndoorSensing, IndoorVisibility } from '../planner/indoor';
import { OUTDOOR_LIMITS, type VehicleCommand, type VehicleLimits } from '../vehicle';

export const NEAR_LIMITS: VehicleLimits = {
  ...OUTDOOR_LIMITS,
  speedH: 2.5,
  speedV: 1.2,
  accel: 2,
};
export const DETECT_PERIOD = 0.2;
export const TRACK_PERIOD = 0.5;
export const PLAN_ACCEL = 2;
/** No new policy voxel for this long counts as a stall, s. */
export const POLICY_STALL_SEC = 8;
/** One contact hands the visit to frontier exploration before the safety ladder escalates. */
export const POLICY_MAX_COLLISIONS = 1;
export const POLICY_MAX_SEC = 150;
export const MAX_EXPLORE_SEC = 260;
/** After the slot: time to get back to the entry floor before the exit starts regardless. */
export const WRAP_UP_SEC = 90;
/** A shielded indoor move that makes no progress for this long skips its waypoint. */
export const SHIELD_STALL_SEC = 4;
export const CONFIRM_DWELL = 3.5;
export const CONFIRM_DWELL_DARK = 6;
/** Confirmation rounds a track gets before it is rejected. */
export const CONFIRM_ROUNDS = 3;

export const RANK: Record<SafetyActionKind, number> = {
  none: 0,
  hold: 1,
  backtrack: 2,
  exit_building: 3,
  return_home: 4,
  land: 5,
};

export interface Waypoint {
  pos: V3;
  /** Heading to hold. Null or undefined faces the travel direction. */
  yaw?: number | null;
  dwell?: number;
  spin?: boolean;
  /** Pass-through tolerance override. */
  tol?: number;
}

export interface Job {
  phase: Phase;
  tick(): VehicleCommand;
  plan(): V3[];
}

export interface Visit {
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
  /** Oracle mode only: camera-visibility estimate and planner on the true floor plan. */
  vis: IndoorVisibility | null;
  explorer: FrontierExplorer | null;
  /** The drone's own maps of this building (every mode but oracle). */
  sensing: IndoorSensing | null;
  /** Path and viewpoint queries: sensor maps, or the floor plan in oracle mode. */
  nav: IndoorNav;
  /** The running sensor-explorer job, so a confirmation can hand control back. */
  sensorJob: Job | null;
  exploreStart: number;
  pass: number;
  policyCollisions: number;
  entered: boolean;
  /** Positions flown outside near this building, known to be clear. */
  trail: V3[];
}

export function fmt(n: number, d = 1): string {
  return n.toFixed(d);
}


export function centerDist(b: MissionBuilding, p: V3): number {
  const box = buildingBox(b);
  return Math.hypot(box.cx - p[0], box.cy - p[1]);
}

export function hold(pos: V3, at: V3): V3 {
  const k = 1.5;
  return [(at[0] - pos[0]) * k, (at[1] - pos[1]) * k, (at[2] - pos[2]) * k];
}

/** Velocity along `dir` at `speed`, scaled uniformly so both caps hold and the path stays straight. */
export function scaleTo(dir: V3, speed: number, limits: VehicleLimits): V3 {
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

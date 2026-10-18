/** Mission schema v1. Local frame: x east, y north, meters. */

export const MISSION_SCHEMA_VERSION = 1;

export const MISSION_TYPES = [
  'find_objects',
  'find_person',
  'safety_sweep',
  'building_inspection',
  'composite',
] as const;

export type MissionType = (typeof MISSION_TYPES)[number];

export const TARGET_KINDS = ['object', 'person', 'hazard', 'structure', 'clearance'] as const;

export type TargetKind = (typeof TARGET_KINDS)[number];

export const ENTRY_POLICIES = ['auto', 'ask', 'never'] as const;

export type EntryPolicy = (typeof ENTRY_POLICIES)[number];

export const MISSION_TYPE_LABELS: Record<MissionType, string> = {
  find_objects: 'Find objects',
  find_person: 'Find simulated person',
  safety_sweep: 'Safety sweep, confirm clear',
  building_inspection: 'Building inspection',
  composite: 'Composite',
};

export const TARGET_KIND_LABELS: Record<TargetKind, string> = {
  object: 'Object',
  person: 'Person',
  hazard: 'Hazard',
  structure: 'Structure',
  clearance: 'Clearance',
};

export interface Point {
  x: number;
  y: number;
}

export interface Ring {
  points: Point[];
}

export interface NoFlyZone {
  id: string;
  name: string;
  polygon: Ring;
  reason: string;
}

export interface TargetSpec {
  id: string;
  kind: TargetKind;
  label: string;
  minCount: number;
  /** Null uses the mission required confidence. */
  minConfidence: number | null;
}

export interface AltitudeBand {
  minM: number;
  maxM: number;
}

export interface MissionBudgets {
  /** Percent of the pack this mission may consume. */
  batteryPct: number;
  timeSec: number;
}

export interface StartPose {
  x: number;
  y: number;
  z: number;
  yawRad: number;
}

export interface Mission {
  schemaVersion: typeof MISSION_SCHEMA_VERSION;
  id: string;
  name: string;
  seed: number;
  notes: string;
  geofence: Ring;
  noFlyZones: NoFlyZone[];
  altitude: AltitudeBand;
  missionType: MissionType;
  targets: TargetSpec[];
  requiredConfidence: number;
  entryPolicy: EntryPolicy;
  budgets: MissionBudgets;
  startPose: StartPose;
}

export interface MissionIssue {
  path: string;
  message: string;
  severity: 'error' | 'warning';
}

export interface MissionValidation {
  ok: boolean;
  mission: Mission | null;
  issues: MissionIssue[];
}

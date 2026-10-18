/** Sim coordinates: x and y are horizontal, z is up. Matches BuildingSpec. */

export type Vec3 = [number, number, number];

export const PERSON_STATES = ['standing', 'sitting', 'lying', 'walking', 'hiding'] as const;
export type PersonState = (typeof PERSON_STATES)[number];

export const OBJECT_IDS = [
  'backpack',
  'phone',
  'toolbox',
  'medkit',
  'bottle',
  'radio',
  'helmet',
  'blanket',
] as const;
export type ObjectId = (typeof OBJECT_IDS)[number];

export const DECOY_IDS = ['mannequin', 'coat', 'decoy_pack', 'decoy_case'] as const;
export type DecoyId = (typeof DECOY_IDS)[number];

export type TargetClass = 'person' | ObjectId | DecoyId;
export type TargetCategory = 'person' | 'object' | 'decoy';

export interface Aabb {
  min: Vec3;
  max: Vec3;
}

/** A rectangular outdoor patch. The outdoor generator can fill this in later. */
export interface OutdoorArea {
  id: string;
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
  groundZ: number;
  obstacles?: Array<{ minX: number; minY: number; maxX: number; maxY: number }>;
  /** Optional walking routes as polylines of [x, y] on the ground. */
  paths?: Array<Array<[number, number]>>;
}

export interface PlacedTarget {
  id: string;
  classId: TargetClass;
  category: TargetCategory;
  state: PersonState | 'static';
  /** Bottom center of the body, on the floor or at a hook. */
  feet: Vec3;
  yaw: number;
  hx: number;
  hy: number;
  height: number;
  /** Feet waypoints. Walking people loop this polyline. */
  path?: Vec3[];
  speed: number;
  floor: number;
  roomIndex: number;
  occluders: Aabb[];
  /** Decoys report detections as this class. */
  looksLike?: TargetClass;
}

export interface ResolvedTarget {
  id: string;
  classId: TargetClass;
  category: TargetCategory;
  state: PersonState | 'static';
  feet: Vec3;
  center: Vec3;
  yaw: number;
  aabb: Aabb;
  speed: number;
  floor: number;
  roomIndex: number;
  occluders: Aabb[];
  looksLike?: TargetClass;
}

export interface TargetScene {
  seed: number;
  space: 'indoor' | 'outdoor';
  targets: PlacedTarget[];
  outdoor?: OutdoorArea;
}

export interface PlaceOptions {
  seed: number;
  people?: number;
  objects?: number;
  decoys?: number;
  /**
   * Line up one of each person state, the object library, and the decoys
   * in the largest room so a camera can see them together.
   */
  showcase?: boolean;
}

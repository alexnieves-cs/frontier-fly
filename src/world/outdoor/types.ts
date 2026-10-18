import type { BuildingSpec } from '../../shared/types';

export type OpeningKind =
  | 'open_window'
  | 'broken_window'
  | 'ajar_door'
  | 'closed_door'
  | 'vent'
  | 'garage_gap'
  | 'roof_hatch';

export interface GridCell {
  x: number;
  z: number;
}

export interface Opening {
  id: number;
  buildingId: number;
  kind: OpeningKind;
  center: [number, number, number];
  /** Points from the interior toward the outside. Roof hatches point up. */
  normal: [number, number, number];
  /** Width axis. Horizontal for walls; +X for hatches. */
  tangent: [number, number, number];
  width: number;
  height: number;
  /** Radians from the closed pose. Zero when the opening has no leaf. */
  angle: number;
  sill: number;
  floor: number;
  passable: boolean;
  decoy: boolean;
  clearWidth: number;
  clearHeight: number;
  apertureCells: GridCell[];
  carvedCells: GridCell[];
}

export interface BuildingShell {
  id: number;
  spec: BuildingSpec;
  /** Min corner of the cell grid, on the pad. */
  origin: [number, number, number];
  openings: Opening[];
  /** Point just outside the entrance the forest corridor runs to. */
  approach: [number, number, number];
}

export interface Terrain {
  size: number;
  resolution: number;
  heights: Float32Array;
}

export interface Clearing {
  x: number;
  y: number;
  inner: number;
  outer: number;
  height: number;
}

export interface Corridor {
  ax: number;
  ay: number;
  bx: number;
  by: number;
  halfWidth: number;
}

export type TreeKind = 'pine' | 'broadleaf' | 'undergrowth';

export interface TreeInstance {
  x: number;
  y: number;
  /** Ground contact. */
  z: number;
  yaw: number;
  kind: TreeKind;
  trunkRadius: number;
  trunkHeight: number;
  canopyRadius: number;
  /** Canopy center, meters above the ground contact. */
  canopyZ: number;
  /** Distance-from-spawn bucket: 0 near, 1 mid, 2 far. The view recomputes this from the camera. */
  lod: 0 | 1 | 2;
  tint: number;
}

export interface Geofence {
  polygon: [number, number][];
}

export interface NoFlyZone {
  id: number;
  name: string;
  polygon: [number, number][];
  minZ: number;
  maxZ: number;
}

export type Solid =
  | { shape: 'trimesh'; tag: string; vertices: Float32Array; indices: Uint32Array }
  | {
      shape: 'cuboid';
      tag: string;
      cx: number;
      cy: number;
      cz: number;
      hx: number;
      hy: number;
      hz: number;
      qx: number;
      qy: number;
      qz: number;
      qw: number;
    }
  | { shape: 'cylinder'; tag: string; cx: number; cy: number; cz: number; radius: number; halfHeight: number }
  | { shape: 'ball'; tag: string; cx: number; cy: number; cz: number; radius: number };

export interface OutdoorWorld {
  seed: number;
  size: number;
  terrain: Terrain;
  clearings: Clearing[];
  corridors: Corridor[];
  trees: TreeInstance[];
  buildings: BuildingShell[];
  geofence: Geofence;
  noFly: NoFlyZone[];
  spawn: [number, number, number];
  spawnYaw: number;
  colliders: Solid[];
}

export interface OutdoorParams {
  seed: number;
  /** Building count. Clamped to 1..4. Omitted values come from the seed. */
  buildings?: number;
}

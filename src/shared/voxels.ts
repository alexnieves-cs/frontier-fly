import type { BuildingSpec } from './types';

/**
 * Layout of the per-building voxel maps (visited, seen-free, occupancy, look, shaft and team maps):
 * 0.5 m voxels (VOXEL), VOXELS_PER_CELL per 1 m floor-plan cell along each axis, all floors in one
 * array indexed `f * vw * vd + vy * vw + vx`. Every map that shares this layout derives it here, so
 * the conversions live in one place. Hot loops keep their own `base` and `vw` locals; this module
 * only defines them.
 *
 * The truth-side seen-empty cells (perception/seenEmpty.ts) and the localization likelihood field
 * (loc/grid.ts) are different grids on purpose and do not use it.
 */
export const VOXELS_PER_CELL = 2;

export interface VoxelLayout {
  /** Voxels along x and y on each floor. */
  readonly vw: number;
  readonly vd: number;
  readonly floors: number;
}

/** The voxel layout of a building, from its ground floor's cell grid. */
export function voxelLayout(spec: Pick<BuildingSpec, 'floors'>): VoxelLayout {
  const floor0 = spec.floors[0]!;
  return { vw: floor0.w * VOXELS_PER_CELL, vd: floor0.d * VOXELS_PER_CELL, floors: spec.floors.length };
}

/** Voxels on every floor together: the length of a map array. */
export function voxelCount(l: VoxelLayout): number {
  return l.floors * l.vw * l.vd;
}

/** Index of floor f's first voxel. */
export function floorBase(l: Pick<VoxelLayout, 'vw' | 'vd'>, f: number): number {
  return f * l.vw * l.vd;
}

/** Index of voxel (vx, vy) on floor f. No bounds check: see inPlane. */
export function voxelIndex(l: Pick<VoxelLayout, 'vw' | 'vd'>, f: number, vx: number, vy: number): number {
  return f * l.vw * l.vd + vy * l.vw + vx;
}

/** (vx, vy) lies on a floor's voxel plane. */
export function inPlane(l: Pick<VoxelLayout, 'vw' | 'vd'>, vx: number, vy: number): boolean {
  return vx >= 0 && vy >= 0 && vx < l.vw && vy < l.vd;
}

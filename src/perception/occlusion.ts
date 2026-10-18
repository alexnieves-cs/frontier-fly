import { SURFACE } from '../shared/constants';
import type { BuildingSpec } from '../shared/types';
import type { Aabb, Vec3 } from '../targets/types';
import type { CameraView } from './camera';

function solidAt(spec: BuildingSpec, x: number, y: number, z: number): number {
  if (z < -0.2) return 0;
  const f = Math.floor(z / spec.floorHeight);
  if (f < 0 || f >= spec.floors.length) return 0;
  const floor = spec.floors[f]!;
  const ix = Math.floor(x / spec.cellSize);
  const iy = Math.floor(y / spec.cellSize);
  if (ix < 0 || iy < 0 || ix >= floor.w || iy >= floor.d) return 0;
  const i = iy * floor.w + ix;
  if (floor.cells[i] !== 1) return 0;
  return (floor.surfaces?.[i] ?? 0) === SURFACE.GLASS ? 0.45 : 1;
}

/** Worst occlusion along a segment. 1 is a solid wall, 0.45 is glass only. */
export function segmentOcclusion(spec: BuildingSpec | null | undefined, origin: Vec3, dest: Vec3): number {
  if (!spec) return 0;
  const dx = dest[0] - origin[0];
  const dy = dest[1] - origin[1];
  const dz = dest[2] - origin[2];
  const length = Math.hypot(dx, dy, dz);
  if (length < 1e-4) return 0;
  const steps = Math.max(2, Math.ceil(length / 0.2));
  let worst = 0;
  for (let i = 1; i < steps; i++) {
    const t = i / steps;
    const hit = solidAt(spec, origin[0] + dx * t, origin[1] + dy * t, origin[2] + dz * t);
    if (hit > worst) worst = hit;
    if (worst >= 1) return 1;
  }
  return worst;
}

function rayHitsAabb(origin: Vec3, dest: Vec3, box: Aabb): boolean {
  const dir = [dest[0] - origin[0], dest[1] - origin[1], dest[2] - origin[2]];
  let tMin = 0;
  let tMax = 1;
  for (let axis = 0; axis < 3; axis++) {
    const o = origin[axis]!;
    const d = dir[axis]!;
    const min = box.min[axis]!;
    const max = box.max[axis]!;
    if (Math.abs(d) < 1e-8) {
      if (o < min || o > max) return false;
      continue;
    }
    let t1 = (min - o) / d;
    let t2 = (max - o) / d;
    if (t1 > t2) {
      const swap = t1;
      t1 = t2;
      t2 = swap;
    }
    if (t1 > tMin) tMin = t1;
    if (t2 < tMax) tMax = t2;
    if (tMin > tMax) return false;
  }
  return tMax > 0.02 && tMin < 0.98;
}

export function samplePoints(aabb: Aabb): Vec3[] {
  const pts: Vec3[] = [];
  for (const u of [0.3, 0.7]) {
    for (const v of [0.3, 0.7]) {
      for (const w of [0.28, 0.85]) {
        pts.push([
          aabb.min[0] + (aabb.max[0] - aabb.min[0]) * u,
          aabb.min[1] + (aabb.max[1] - aabb.min[1]) * v,
          aabb.min[2] + (aabb.max[2] - aabb.min[2]) * w,
        ]);
      }
    }
  }
  pts.push([
    (aabb.min[0] + aabb.max[0]) * 0.5,
    (aabb.min[1] + aabb.max[1]) * 0.5,
    (aabb.min[2] + aabb.max[2]) * 0.5,
  ]);
  return pts;
}

export function occlusionFraction(
  cam: CameraView,
  aabb: Aabb,
  spec: BuildingSpec | null | undefined,
  occluders: Aabb[],
): number {
  const points = samplePoints(aabb);
  let sum = 0;
  for (const point of points) {
    let hit = segmentOcclusion(spec, cam.position, point);
    if (hit < 1) {
      for (const box of occluders) {
        if (rayHitsAabb(cam.position, point, box)) {
          hit = 1;
          break;
        }
      }
    }
    sum += hit;
  }
  return sum / points.length;
}

export function pointOcclusion(spec: BuildingSpec | null | undefined, origin: Vec3, point: Vec3, occluders: Aabb[]): number {
  const hit = segmentOcclusion(spec, origin, point);
  if (hit < 1) {
    for (const box of occluders) {
      if (rayHitsAabb(origin, point, box)) return 1;
    }
  }
  return hit;
}

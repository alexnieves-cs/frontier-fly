import type { BuildingSpec } from '../shared/types';
import type { ResolvedTarget, Vec3 } from './types';

export interface Frame {
  eye: Vec3;
  look: Vec3;
}

/** A south-of-targets camera that stays inside the ground floor. */
export function frameOnTargets(spec: BuildingSpec, targets: ResolvedTarget[], distance: number): Frame {
  const floor = spec.floors[0]!;
  const cs = spec.cellSize;
  const usable = targets.length ? targets : [];
  let sx = 0;
  let sy = 0;
  let sz = 0;
  let minY = Infinity;
  for (const target of usable) {
    sx += target.center[0];
    sy += target.center[1];
    sz += target.center[2];
    if (target.aabb.min[1] < minY) minY = target.aabb.min[1];
  }
  const n = Math.max(1, usable.length);
  const look: Vec3 = [sx / n, sy / n, Math.max(sz / n, spec.floorHeight * 0.35)];
  const south = cs * 1.35;
  const north = (floor.d - 1.35) * cs;
  const west = cs * 1.35;
  const east = (floor.w - 1.35) * cs;
  let eyeY = Math.min(north, Math.max(south, Math.min(minY - 0.8, look[1] - distance)));
  let eyeX = Math.min(east, Math.max(west, look[0]));
  const ix = Math.floor(eyeX / cs);
  const iy = Math.floor(eyeY / cs);
  if (ix < 0 || iy < 0 || ix >= floor.w || iy >= floor.d || floor.cells[iy * floor.w + ix] !== 0) {
    eyeX = Math.min(east, Math.max(west, (Math.floor(look[0] / cs) + 0.5) * cs));
    eyeY = south;
  }
  const eye: Vec3 = [eyeX, eyeY, spec.floorHeight * 0 + 1.45];
  return { eye, look: [look[0], look[1], 1.05] };
}

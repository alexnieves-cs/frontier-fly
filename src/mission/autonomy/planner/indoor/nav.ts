/**
 * IndoorNav, the seam between the truth-side reference and the sensor-only drone: path and viewpoint
 * queries on the floor plan (oracleNav) or on the drone's own maps (sensorNav).
 */
import { VOXEL } from '../../../../shared/constants';
import { sensorClearance, sensorLos, sensorPath } from '../../../../sim/sensorMap';
import { type V3 } from '../../contracts';
import { INDOOR_FLY_Z, type IndoorGrid, PATH_CLEARANCE, floorOf, gridLos, nearestNav, planIndoorPath, wallClearance } from './grid';
import type { IndoorSensing } from './sensing';

/** Path and viewpoint queries the runner makes inside a building. */
export interface IndoorNav {
  /** Local waypoints from one local point to another, or null when unreachable. */
  path(from: V3, to: V3): V3[] | null;
  /** A hover point is safe and sees the target point. */
  canView(p: V3, target: V3): boolean;
}

/** Floor-plan navigation: the reference oracle. It knows every wall before the drone sees it. */
export function oracleNav(grid: IndoorGrid): IndoorNav {
  return {
    path: (from, to) => planIndoorPath(grid, from, to),
    canView: (p, target) => {
      const f = floorOf(grid, p[2]);
      const c = nearestNav(grid, p);
      if (!c || Math.hypot((c.x + 0.5) * grid.cs - p[0], (c.z + 0.5) * grid.cs - p[1]) > 0.75) return false;
      if (wallClearance(grid, f, p[0], p[1]) < PATH_CLEARANCE) return false;
      return gridLos(grid, f, p[0], p[1], target[0], target[1]);
    },
  };
}

/** Navigation on the drone's own maps: only voxels it has seen free or flown. */
export function sensorNav(grid: IndoorGrid, sensing: IndoorSensing): IndoorNav {
  return {
    path: (from, to) => {
      const f = floorOf(grid, from[2]);
      if (floorOf(grid, to[2]) !== f) return null;
      const xy = sensorPath(sensing, f, [from[0], from[1]], [to[0], to[1]]);
      if (!xy) return null;
      const z = f * grid.H + INDOOR_FLY_Z;
      return xy.map(([x, y]) => [x, y, z] as V3);
    },
    canView: (p, target) => {
      const f = floorOf(grid, p[2]);
      if (sensorClearance(sensing, f, p[0], p[1]) < VOXEL) return false;
      // The target itself may sit in a voxel the lidar never painted free (a person on the floor).
      return sensorLos(sensing, f, p[0], p[1], target[0], target[1], VOXEL);
    },
  };
}

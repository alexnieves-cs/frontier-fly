import { CELL, VOXEL } from '../shared/constants';
import type { BuildingSpec } from '../shared/types';

export function isWalkable(cell: number): boolean {
  return cell === CELL.EMPTY || cell === CELL.DOOR || cell === CELL.SHAFT;
}

/** Per-floor masks of cells reachable from the spawn, including shaft climbs. */
export function reachableMasks(spec: BuildingSpec): Uint8Array[] {
  const masks = spec.floors.map((floor) => new Uint8Array(floor.w * floor.d));
  const cs = spec.cellSize;
  const sx = Math.floor(spec.spawn[0] / cs);
  const sy = Math.floor(spec.spawn[1] / cs);
  const sf = Math.max(0, Math.min(spec.floors.length - 1, Math.floor(spec.spawn[2] / spec.floorHeight)));
  const floor0 = spec.floors[sf];
  if (!floor0 || sx < 0 || sy < 0 || sx >= floor0.w || sy >= floor0.d) return masks;
  if (!isWalkable(floor0.cells[sy * floor0.w + sx]!)) return masks;

  const queue: number[] = [sf, sx, sy];
  masks[sf]![sy * floor0.w + sx] = 1;
  const dirs = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ];
  for (let head = 0; head < queue.length; head += 3) {
    const f = queue[head]!;
    const x = queue[head + 1]!;
    const z = queue[head + 2]!;
    const floor = spec.floors[f]!;
    const i = z * floor.w + x;
    for (const [dx, dz] of dirs) {
      const nx = x + dx;
      const nz = z + dz;
      if (nx < 0 || nz < 0 || nx >= floor.w || nz >= floor.d) continue;
      const ni = nz * floor.w + nx;
      if (masks[f]![ni] || !isWalkable(floor.cells[ni]!)) continue;
      masks[f]![ni] = 1;
      queue.push(f, nx, nz);
    }
    if (floor.cells[i] === CELL.SHAFT) {
      if (f + 1 < spec.floors.length) {
        const up = spec.floors[f + 1]!;
        const ui = z * up.w + x;
        if (!masks[f + 1]![ui] && up.cells[ui] === CELL.SHAFT) {
          masks[f + 1]![ui] = 1;
          queue.push(f + 1, x, z);
        }
      }
      if (f - 1 >= 0) {
        const down = spec.floors[f - 1]!;
        const di = z * down.w + x;
        if (!masks[f - 1]![di] && down.cells[di] === CELL.SHAFT) {
          masks[f - 1]![di] = 1;
          queue.push(f - 1, x, z);
        }
      }
    }
  }
  return masks;
}

export function countReachableVoxels(spec: BuildingSpec): number {
  const perCell = Math.round(spec.cellSize / VOXEL) ** 2;
  let cells = 0;
  for (const mask of reachableMasks(spec)) {
    for (const bit of mask) if (bit) cells += 1;
  }
  return cells * perCell;
}

export function isFullyReachable(spec: BuildingSpec): boolean {
  const masks = reachableMasks(spec);
  for (let f = 0; f < spec.floors.length; f++) {
    const floor = spec.floors[f]!;
    const mask = masks[f]!;
    if (floor.rooms.length === 0) return false;
    for (const room of floor.rooms) {
      let ok = false;
      for (let z = room.z; z < room.z + room.d && !ok; z++) {
        for (let x = room.x; x < room.x + room.w; x++) {
          if (x < 0 || z < 0 || x >= floor.w || z >= floor.d) continue;
          if (mask[z * floor.w + x]) {
            ok = true;
            break;
          }
        }
      }
      if (!ok) return false;
    }
  }
  return countReachableVoxels(spec) > 0;
}

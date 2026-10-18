import { VOXEL } from '../shared/constants';
import type { BuildingSpec } from '../shared/types';
import { isWalkable } from '../gen/validate';

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

/** min(best, distance to each unvisited voxel of walkable cell (x, z)). Out-of-grid cells are ignored. */
function cellBest(
  floor: BuildingSpec['floors'][number],
  visited: Uint8Array,
  base: number,
  vw: number,
  x: number,
  z: number,
  px: number,
  py: number,
  best: number,
): number {
  if (x < 0 || z < 0 || x >= floor.w || z >= floor.d) return best;
  if (!isWalkable(floor.cells[z * floor.w + x]!)) return best;
  for (let dz = 0; dz < 2; dz++) {
    for (let dx = 0; dx < 2; dx++) {
      const vx = x * 2 + dx;
      const vy = z * 2 + dz;
      if (visited[base + vy * vw + vx]) continue;
      const dist = Math.hypot((vx + 0.5) * VOXEL - px, (vy + 0.5) * VOXEL - py);
      if (dist < best) best = dist;
    }
  }
  return best;
}

/**
 * Privileged coverage teacher. Walks to the nearest unfinished cell, using a
 * grid BFS so the heading does not point through walls. The observation the
 * policy sees still has the local visited map, which is what imitation learns.
 */
function nearestOpen(spec: BuildingSpec, visited: Uint8Array, pos: [number, number, number], radius: number): number {
  if (spec.cellSize !== VOXEL * 2) return nearestOpenScan(spec, visited, pos, radius);
  const vw = spec.floors[0]!.w * 2;
  const vd = spec.floors[0]!.d * 2;
  const cs = spec.cellSize;
  const px = pos[0];
  const py = pos[1];
  const gx = Math.floor(px / cs);
  const gz = Math.floor(py / cs);
  const floors = spec.floors;
  // Every voxel centre in a cell at Chebyshev ring r >= 1 is more than (r - 0.75) * cs away
  // (centres sit at 0.25 and 0.75 of a cell). The 1e-9 keeps the skip test conservative.
  const ringSkips = (r: number, threshold: number) => r >= 1 && (r - 0.75) * cs - 1e-9 >= threshold;

  // Phase A. The reference scan returns the first voxel, in (floor, z, x, dz, dx) order, that is
  // closer than 0.4 m. Such voxels can only live in rings 0..1, so scanning that 3x3 block in the
  // same order returns the same value.
  for (let f = 0; f < floors.length; f++) {
    const floor = floors[f]!;
    const z0 = Math.max(0, gz - radius, gz - 1);
    const z1 = Math.min(floor.d - 1, gz + radius, gz + 1);
    const x0 = Math.max(0, gx - radius, gx - 1);
    const x1 = Math.min(floor.w - 1, gx + radius, gx + 1);
    const base = f * vw * vd;
    for (let z = z0; z <= z1; z++) {
      for (let x = x0; x <= x1; x++) {
        if (!isWalkable(floor.cells[z * floor.w + x]!)) continue;
        for (let dz = 0; dz < 2; dz++) {
          for (let dx = 0; dx < 2; dx++) {
            const vx = x * 2 + dx;
            const vy = z * 2 + dz;
            if (visited[base + vy * vw + vx]) continue;
            const dist = Math.hypot((vx + 0.5) * VOXEL - px, (vy + 0.5) * VOXEL - py);
            if (dist < 0.4) return dist;
          }
        }
      }
    }
  }

  // Phase B. No voxel is closer than 0.4 m, so the reference returns min(6, nearest distance).
  // A minimum does not depend on visit order, so search outward ring by ring and stop once a
  // ring cannot beat the best distance found so far.
  let best = 6;
  for (let r = 0; r <= radius; r++) {
    if (ringSkips(r, best)) break;
    const zLo = gz - r;
    const zHi = gz + r;
    const xLo = gx - r;
    const xHi = gx + r;
    let enclosesAll = true;
    for (let f = 0; f < floors.length; f++) {
      const floor = floors[f]!;
      const fw = floor.w;
      const fd = floor.d;
      // Once a ring surrounds the whole floor, it and every later ring lie outside it.
      if (xLo < 0 && zLo < 0 && xHi > fw - 1 && zHi > fd - 1) continue;
      enclosesAll = false;
      const base = f * vw * vd;
      if (r === 0) {
        best = cellBest(floor, visited, base, vw, gx, gz, px, py, best);
        continue;
      }
      const xa = Math.max(0, xLo);
      const xb = Math.min(fw - 1, xHi);
      for (let x = xa; x <= xb; x++) {
        best = cellBest(floor, visited, base, vw, x, zLo, px, py, best);
        best = cellBest(floor, visited, base, vw, x, zHi, px, py, best);
      }
      const za = Math.max(0, zLo + 1);
      const zb = Math.min(fd - 1, zHi - 1);
      for (let z = za; z <= zb; z++) {
        best = cellBest(floor, visited, base, vw, xLo, z, px, py, best);
        best = cellBest(floor, visited, base, vw, xHi, z, px, py, best);
      }
    }
    if (enclosesAll) break;
  }
  return best;
}

/** Reference row-major scan. Kept for non-standard cell sizes; nearestOpen must match it exactly. */
function nearestOpenScan(spec: BuildingSpec, visited: Uint8Array, pos: [number, number, number], radius: number): number {
  const vw = spec.floors[0]!.w * 2;
  const vd = spec.floors[0]!.d * 2;
  const cs = spec.cellSize;
  let best = 6;
  for (let f = 0; f < spec.floors.length; f++) {
    const floor = spec.floors[f]!;
    const gx = Math.floor(pos[0] / cs);
    const gz = Math.floor(pos[1] / cs);
    const z0 = Math.max(0, gz - radius);
    const z1 = Math.min(floor.d - 1, gz + radius);
    const x0 = Math.max(0, gx - radius);
    const x1 = Math.min(floor.w - 1, gx + radius);
    for (let z = z0; z <= z1; z++) {
      for (let x = x0; x <= x1; x++) {
        if (!isWalkable(floor.cells[z * floor.w + x]!)) continue;
        for (let dz = 0; dz < 2; dz++) {
          for (let dx = 0; dx < 2; dx++) {
            const vx = x * 2 + dx;
            const vy = z * 2 + dz;
            if (visited[f * vw * vd + vy * vw + vx]) continue;
            const cx = (vx + 0.5) * VOXEL;
            const cy = (vy + 0.5) * VOXEL;
            const dist = Math.hypot(cx - pos[0], cy - pos[1]);
            if (dist < best) best = dist;
            if (best < 0.4) return best;
          }
        }
      }
    }
  }
  return best;
}

/** Distance in meters to the nearest unfinished voxel, capped at 6. */
export function frontierDistance(spec: BuildingSpec, visited: Uint8Array, pos: [number, number, number]): number {
  const local = nearestOpen(spec, visited, pos, 4);
  if (local < 6) return local;
  return nearestOpen(spec, visited, pos, 64);
}

export function frontierAction(
  spec: BuildingSpec,
  visited: Uint8Array,
  pos: [number, number, number],
  yaw: number,
): Float32Array {
  const floorIndex = Math.max(0, Math.min(spec.floors.length - 1, Math.floor(pos[2] / spec.floorHeight)));
  const floor = spec.floors[floorIndex]!;
  const cs = spec.cellSize;
  const vw = spec.floors[0]!.w * 2;
  const vd = spec.floors[0]!.d * 2;
  const gx = clamp(Math.floor(pos[0] / cs), 0, floor.w - 1);
  const gz = clamp(Math.floor(pos[1] / cs), 0, floor.d - 1);

  const open = (x: number, z: number): boolean => {
    if (x < 0 || z < 0 || x >= floor.w || z >= floor.d) return false;
    if (!isWalkable(floor.cells[z * floor.w + x]!)) return false;
    const base = floorIndex * vw * vd;
    for (let dz = 0; dz < 2; dz++) {
      for (let dx = 0; dx < 2; dx++) {
        const vx = x * 2 + dx;
        const vy = z * 2 + dz;
        if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) continue;
        if (!visited[base + vy * vw + vx]) return true;
      }
    }
    return false;
  };

  let goalX = (gx + 0.5) * cs;
  let goalY = (gz + 0.5) * cs;
  if (open(gx, gz)) {
    let best = Infinity;
    for (let dz = 0; dz < 2; dz++) {
      for (let dx = 0; dx < 2; dx++) {
        const vx = gx * 2 + dx;
        const vy = gz * 2 + dz;
        const index = floorIndex * vw * vd + vy * vw + vx;
        if (visited[index]) continue;
        const cx = (vx + 0.5) * VOXEL;
        const cy = (vy + 0.5) * VOXEL;
        const d = (cx - pos[0]) ** 2 + (cy - pos[1]) ** 2;
        if (d < best) {
          best = d;
          goalX = cx;
          goalY = cy;
        }
      }
    }
  } else {
    const prev = new Int32Array(floor.w * floor.d).fill(-1);
    const queue: number[] = [gx, gz];
    prev[gz * floor.w + gx] = gz * floor.w + gx;
    let found = -1;
    for (let head = 0; head < queue.length && found < 0; head += 2) {
      const x = queue[head]!;
      const z = queue[head + 1]!;
      const dirs = [
        [1, 0],
        [-1, 0],
        [0, 1],
        [0, -1],
      ];
      for (const [dx, dz] of dirs) {
        const nx = x + dx;
        const nz = z + dz;
        if (!open(nx, nz) && (nx < 0 || nz < 0 || nx >= floor.w || nz >= floor.d || !isWalkable(floor.cells[nz * floor.w + nx]!))) {
          continue;
        }
        if (nx < 0 || nz < 0 || nx >= floor.w || nz >= floor.d) continue;
        const ni = nz * floor.w + nx;
        if (prev[ni] !== -1) continue;
        if (!isWalkable(floor.cells[ni]!)) continue;
        prev[ni] = z * floor.w + x;
        if (open(nx, nz)) {
          found = ni;
          break;
        }
        queue.push(nx, nz);
      }
    }
    if (found >= 0) {
      let cursor = found;
      let parent = prev[cursor]!;
      while (parent !== cursor && parent !== gz * floor.w + gx) {
        cursor = parent;
        parent = prev[cursor]!;
      }
      const cx = cursor % floor.w;
      const cz = Math.floor(cursor / floor.w);
      goalX = (cx + 0.5) * cs;
      goalY = (cz + 0.5) * cs;
    }
  }

  for (let dz = -1; dz <= 1; dz++) {
    for (let dx = -1; dx <= 1; dx++) {
      const cx = gx + dx;
      const cz = gz + dz;
      if (cx < 0 || cz < 0 || cx >= floor.w || cz >= floor.d || isWalkable(floor.cells[cz * floor.w + cx]!)) continue;
      const nearestX = clamp(pos[0], cx * cs, (cx + 1) * cs);
      const nearestY = clamp(pos[1], cz * cs, (cz + 1) * cs);
      const ox = pos[0] - nearestX;
      const oy = pos[1] - nearestY;
      const dist = Math.hypot(ox, oy) || 1e-4;
      if (dist < 0.4) {
        goalX += (ox / dist) * (0.4 - dist) * 3;
        goalY += (oy / dist) * (0.4 - dist) * 3;
      }
    }
  }

  const dx = goalX - pos[0];
  const dy = goalY - pos[1];
  const bx = Math.cos(yaw) * dx + Math.sin(yaw) * dy;
  const by = -Math.sin(yaw) * dx + Math.cos(yaw) * dy;
  let err = Math.atan2(dy, dx) - yaw;
  const tau = Math.PI * 2;
  err = ((err + Math.PI) % tau + tau) % tau - Math.PI;
  let forward = clamp(bx * 2.2, -0.35, 0.9);
  let strafe = clamp(by * 2.2, -0.75, 0.75);
  const yawCmd = clamp(err / (Math.PI / 3), -0.8, 0.8);
  const wx = Math.cos(yaw) * forward - Math.sin(yaw) * strafe;
  const wy = Math.sin(yaw) * forward + Math.cos(yaw) * strafe;
  const ahead = (ox: number, oy: number) => {
    const cx = Math.floor((pos[0] + ox) / cs);
    const cz = Math.floor((pos[1] + oy) / cs);
    if (cx < 0 || cz < 0 || cx >= floor.w || cz >= floor.d) return true;
    return !isWalkable(floor.cells[cz * floor.w + cx]!);
  };
  if (ahead(wx * 0.45, wy * 0.45)) {
    if (!ahead(wx * 0.45, 0)) strafe *= 0.15;
    else if (!ahead(0, wy * 0.45)) forward *= 0.15;
    else {
      forward *= 0.2;
      strafe *= 0.2;
    }
  }
  const floorZ = floorIndex * spec.floorHeight + 1;
  const vz = clamp((floorZ - pos[2]) * 3, -0.6, 0.6);
  return new Float32Array([forward, strafe, vz, yawCmd]);
}

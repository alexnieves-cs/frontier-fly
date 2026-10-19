import { DRONE_RADIUS, VOXEL } from '../shared/constants';
import type { BuildingSpec } from '../shared/types';
import { furnitureIndex, type FurnitureIndex } from '../gen/furniture';
import { FLIGHT_ALTITUDE, flightMap, isWalkable, wallHits, type FlightMap } from '../gen/validate';

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

/** True when a sphere swept from (x0, y0) to (x1, y1) at height z clears walls and furniture. */
function segmentClear(
  spec: BuildingSpec,
  furniture: FurnitureIndex,
  f: number,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  z: number,
  r: number,
  from = 0,
  margin = 0,
  wallMargin = margin,
): boolean {
  const len = Math.hypot(x1 - x0, y1 - y0);
  const steps = Math.max(1, Math.ceil(len / 0.05));
  for (let i = from; i <= steps; i++) {
    const t = i / steps;
    const x = x0 + (x1 - x0) * t;
    const y = y0 + (y1 - y0) * t;
    if (wallHits(spec, f, x, y, r + wallMargin) || furniture.hits(x, y, z, r, margin)) return false;
  }
  return true;
}

const VOXEL_DIRS = [
  [1, 0],
  [-1, 0],
  [0, 1],
  [0, -1],
] as const;
/** Cells around tall furniture where the teacher plans at voxel level. */
const NEAR_CELLS = 0;
/** Extra clearance demanded of straight hops from the drone toward a goal. */
const VIEW_MARGIN = 0.08;
/** Per flight map: 4 edge states per voxel (0 unknown, 1 clear, 2 blocked). */
const edgeCache = new WeakMap<FlightMap, Uint8Array>();

/**
 * Voxel-level planner for the neighbourhood of furniture: BFS over reachable
 * voxels (edges are clear straight hops between their free target points) from
 * the drone's voxel to the nearest unvisited reachable voxel (or, without
 * `visited`, to the voxel holding the cell-level goal), then the furthest path
 * point in straight clear view.
 */
function voxelDetour(
  spec: BuildingSpec,
  map: FlightMap,
  furniture: FurnitureIndex,
  f: number,
  pos: [number, number, number],
  goalX: number,
  goalY: number,
  visited: Uint8Array | null,
): [number, number] | null {
  const { vw, vd } = map;
  const base = f * vw * vd;
  let edges = edgeCache.get(map);
  if (!edges) {
    edges = new Uint8Array(map.reachable.length * 4);
    edgeCache.set(map, edges);
  }
  const zf = f * spec.floorHeight + FLIGHT_ALTITUDE;
  const voxelOf = (x: number, y: number) => {
    const vx = Math.floor(x / VOXEL);
    const vy = Math.floor(y / VOXEL);
    if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) return -1;
    return vy * vw + vx;
  };
  let start = voxelOf(pos[0], pos[1]);
  if (start < 0 || !map.reachable[base + start]) {
    let best = Infinity;
    const vx0 = Math.floor(pos[0] / VOXEL);
    const vy0 = Math.floor(pos[1] / VOXEL);
    start = -1;
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        const vx = vx0 + dx;
        const vy = vy0 + dy;
        if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) continue;
        const vi = vy * vw + vx;
        if (!map.reachable[base + vi]) continue;
        const dist = (map.tx[base + vi]! - pos[0]) ** 2 + (map.ty[base + vi]! - pos[1]) ** 2;
        if (dist < best) {
          best = dist;
          start = vi;
        }
      }
    }
    if (start < 0) return null;
  }
  // Without `visited`: BFS outward from the goal voxel, so `next` is a distance
  // field toward it and the hop chosen from any start is consistent from step
  // to step. With `visited`: BFS from the start to the nearest unvisited
  // reachable voxel (each hop still lowers the distance to that set).
  let target = visited ? -1 : voxelOf(goalX, goalY);
  if (!visited && (target < 0 || !map.reachable[base + target])) return null;
  const root = visited ? start : target;
  const link = new Int32Array(vw * vd).fill(-1);
  link[root] = root;
  const queue = [root];
  let found = visited ? !visited[base + start] : start === target;
  if (found) target = start;
  for (let head = 0; head < queue.length && !found; head++) {
    const v = queue[head]!;
    const vx = v % vw;
    const vy = (v - vx) / vw;
    for (let k = 0; k < 4; k++) {
      const nx = vx + VOXEL_DIRS[k]![0];
      const ny = vy + VOXEL_DIRS[k]![1];
      if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
      const nv = ny * vw + nx;
      if (link[nv] !== -1 || !map.reachable[base + nv]) continue;
      const e = (base + v) * 4 + k;
      if (edges[e] === 0) {
        const clear = segmentClear(spec, furniture, f, map.tx[base + v]!, map.ty[base + v]!, map.tx[base + nv]!, map.ty[base + nv]!, zf, DRONE_RADIUS);
        edges[e] = clear ? 1 : 2;
        edges[(base + nv) * 4 + (k ^ 1)] = edges[e]!;
      }
      if (edges[e] !== 1) continue;
      link[nv] = v;
      if (visited ? !visited[base + nv] : nv === start) {
        if (visited) target = nv;
        found = true;
        break;
      }
      queue.push(nv);
    }
  }
  if (!found) return null;
  const path: number[] = [];
  if (visited) {
    for (let v = target; ; v = link[v]!) {
      path.push(v);
      if (v === start) break;
    }
    path.reverse();
  } else {
    for (let v = start; ; v = link[v]!) {
      path.push(v);
      if (v === target) break;
    }
  }
  // String-pull: aim at the furthest of the next few path points in clear view.
  // The path starts at the start voxel's free point, which may itself be out of
  // view (the drone is off it, behind a chair back), so fall back to the
  // nearest neighbouring voxel point in view, which rejoins the graph.
  const inView = (v: number, margin = VIEW_MARGIN, wallMargin = margin) =>
    segmentClear(spec, furniture, f, pos[0], pos[1], map.tx[v]!, map.ty[v]!, pos[2], DRONE_RADIUS, 1, margin, wallMargin);
  const last = Math.min(path.length - 1, 6);
  // Prefer hops with clearance from walls and furniture; only the next hop may graze.
  for (const [margin, wallMargin, deepest] of [
    [VIEW_MARGIN, VIEW_MARGIN, last],
    [0, VIEW_MARGIN, last],
    [0, 0, Math.min(1, last)],
  ] as const) {
    for (let i = deepest; i >= 1; i--) {
      const v = base + path[i]!;
      if (inView(v, margin, wallMargin)) return [map.tx[v]!, map.ty[v]!];
    }
  }
  // On the start point the hop to the next path point was checked clear from
  // exactly here, so take it; off it, return to it first.
  const v0 = base + start;
  const off = Math.hypot(map.tx[v0]! - pos[0], map.ty[v0]! - pos[1]);
  if (off <= 0.03 && path.length > 1) return [map.tx[base + path[1]!]!, map.ty[base + path[1]!]!];
  if (off > 0.03 && inView(v0, 0)) return [map.tx[v0]!, map.ty[v0]!];
  const vx0 = Math.floor(pos[0] / VOXEL);
  const vy0 = Math.floor(pos[1] / VOXEL);
  let best = Infinity;
  let pick: [number, number] | null = null;
  for (let dy = -1; dy <= 1; dy++) {
    for (let dx = -1; dx <= 1; dx++) {
      const vx = vx0 + dx;
      const vy = vy0 + dy;
      if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) continue;
      const v = base + vy * vw + vx;
      if (!map.reachable[v]) continue;
      const dist = (map.tx[v]! - pos[0]) ** 2 + (map.ty[v]! - pos[1]) ** 2;
      if (dist > 0.0025 && dist < best && inView(v, 0)) {
        best = dist;
        pick = [map.tx[v]!, map.ty[v]!];
      }
    }
  }
  return pick;
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

  // Furniture awareness: only voxels the drone center can reach at flight
  // altitude are targets (the same rule as the coverage denominator), goals are
  // free points inside voxels and cells, and furniture repels like walls.
  const map = flightMap(spec);
  const furniture = furnitureIndex(spec);
  const cellBase = floorIndex * floor.w * floor.d;
  const passable = (x: number, z: number): boolean =>
    x >= 0 && z >= 0 && x < floor.w && z < floor.d && map.cellPass[cellBase + z * floor.w + x] === 1;

  const open = (x: number, z: number): boolean => {
    if (x < 0 || z < 0 || x >= floor.w || z >= floor.d) return false;
    if (!isWalkable(floor.cells[z * floor.w + x]!)) return false;
    const base = floorIndex * vw * vd;
    for (let dz = 0; dz < 2; dz++) {
      for (let dx = 0; dx < 2; dx++) {
        const vx = x * 2 + dx;
        const vy = z * 2 + dz;
        if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) continue;
        const vi = base + vy * vw + vx;
        if (!visited[vi] && map.reachable[vi]) return true;
      }
    }
    return false;
  };

  let goalX = (gx + 0.5) * cs;
  let goalY = (gz + 0.5) * cs;
  const localGoal = open(gx, gz);
  if (localGoal) {
    let best = Infinity;
    for (let dz = 0; dz < 2; dz++) {
      for (let dx = 0; dx < 2; dx++) {
        const vx = gx * 2 + dx;
        const vy = gz * 2 + dz;
        const index = floorIndex * vw * vd + vy * vw + vx;
        if (visited[index] || !map.reachable[index]) continue;
        const cx = map.tx[index]!;
        const cy = map.ty[index]!;
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
        if (!isWalkable(floor.cells[ni]!) || !passable(nx, nz)) continue;
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
      goalX = map.wx[cellBase + cursor]!;
      goalY = map.wy[cellBase + cursor]!;
    }
  }

  if (furniture.size > 0) {
    // Near tall furniture plan at voxel level (stateless, so the choice must not
    // flicker with small pose changes); elsewhere keep the cell-level plan.
    let near = false;
    for (let dz = -NEAR_CELLS; dz <= NEAR_CELLS && !near; dz++) {
      for (let dx = -NEAR_CELLS; dx <= NEAR_CELLS; dx++) {
        const cx = gx + dx;
        const cz = gz + dz;
        if (cx < 0 || cz < 0 || cx >= floor.w || cz >= floor.d) continue;
        if (map.fine[cellBase + cz * floor.w + cx]) {
          near = true;
          break;
        }
      }
    }
    let detour: [number, number] | null = null;
    if (near) {
      // A voxel of this cell is the goal: route to it. Otherwise head for the
      // nearest unvisited voxel by voxel BFS, which does not flicker when the
      // drone straddles a cell boundary the way the cell-level waypoint does.
      detour = voxelDetour(spec, map, furniture, floorIndex, pos, goalX, goalY, localGoal ? null : visited);
    }
    else if (!segmentClear(spec, furniture, floorIndex, pos[0], pos[1], goalX, goalY, pos[2], DRONE_RADIUS, 1, VIEW_MARGIN, 0)) {
      detour = voxelDetour(spec, map, furniture, floorIndex, pos, goalX, goalY, null);
    }
    if (detour) {
      goalX = detour[0];
      goalY = detour[1];
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

  const [px, py] = furniture.repulsion(pos[0], pos[1], pos[2], DRONE_RADIUS, DRONE_RADIUS + 0.06, 3);
  goalX += px;
  goalY += py;

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
    if (!isWalkable(floor.cells[cz * floor.w + cx]!)) return true;
    return furniture.hits(pos[0] + ox, pos[1] + oy, pos[2], DRONE_RADIUS);
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

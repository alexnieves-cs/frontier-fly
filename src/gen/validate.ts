import { CELL, DRONE_RADIUS, VOXEL } from '../shared/constants';
import type { BuildingSpec } from '../shared/types';
import { boxExtent, FurnitureIndex } from './furniture';

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

/** Height above each floor at which coverage voxels are judged (the teacher holds this altitude). */
export const FLIGHT_ALTITUDE = 1;
/** Sample spacing used inside cells that tall furniture touches. */
const LATTICE = 0.1;
/** Extra clearance preferred for the goal point of a voxel or cell. */
const GOAL_MARGIN = 0.08;

/**
 * Where the drone center can be at flight altitude, at 0.5 m voxel resolution.
 * `reachable` and `tx`/`ty` use the env's visited-grid layout
 * (f * vw * vd + vy * vw + vx). `tx`/`ty` is a free, connected point inside each
 * reachable voxel (the voxel center unless furniture is in the way), and
 * `wx`/`wy` the same for each grid cell (`cellPass` marks cells with any),
 * indexed (f * d + gz) * w + gx.
 */
export interface FlightMap {
  vw: number;
  vd: number;
  reachable: Uint8Array;
  tx: Float32Array;
  ty: Float32Array;
  count: number;
  cellPass: Uint8Array;
  wx: Float32Array;
  wy: Float32Array;
  /** Cells whose interior is sampled finely because tall furniture is near. */
  fine: Uint8Array;
}

/** 2D sphere (circle) vs the grid walls of floor f; out of bounds counts as wall. */
export function wallHits(spec: BuildingSpec, f: number, x: number, y: number, r: number): boolean {
  const floor = spec.floors[f]!;
  const cs = spec.cellSize;
  const x0 = Math.floor((x - r) / cs);
  const x1 = Math.floor((x + r) / cs);
  const y0 = Math.floor((y - r) / cs);
  const y1 = Math.floor((y + r) / cs);
  for (let gz = y0; gz <= y1; gz++) {
    for (let gx = x0; gx <= x1; gx++) {
      const out = gx < 0 || gz < 0 || gx >= floor.w || gz >= floor.d;
      if (!out && floor.cells[gz * floor.w + gx] !== CELL.WALL) continue;
      const nx = Math.max(gx * cs, Math.min((gx + 1) * cs, x));
      const ny = Math.max(gz * cs, Math.min((gz + 1) * cs, y));
      if ((x - nx) ** 2 + (y - ny) ** 2 < r * r) return true;
    }
  }
  return false;
}

/**
 * Reachability rule for the coverage denominator.
 *
 * Grid cells follow `reachableMasks`: 4-connected walkable cells from the spawn
 * plus shaft climbs, and every voxel of a reached cell counts. Cells touched by
 * a furniture box that reaches the flight band (box within DRONE_RADIUS of
 * z = floor + FLIGHT_ALTITUDE; footprint inflated by DRONE_RADIUS) are instead
 * sampled on a 0.1 m lattice: a sample is free when a DRONE_RADIUS sphere at
 * flight altitude clears both the grid walls and every furniture box (the same
 * tests PhysicsWorld.penetrates uses). Free samples connect to their
 * 4-neighbours and to untouched neighbouring cells, and one BFS from the spawn
 * runs over the mixed graph. A voxel in a touched cell counts only when one of
 * its 25 samples is reached. So a shelf or a chair back removes the voxels the
 * drone center cannot occupy (or cannot get to) at flight altitude, while
 * tables, sofas and plants, which top out below 0.85 m, stay flyable-over.
 */
export function computeFlightMap(spec: BuildingSpec, index = new FurnitureIndex(spec)): FlightMap {
  const floor0 = spec.floors[0]!;
  const cs = spec.cellSize;
  const H = spec.floorHeight;
  const w = floor0.w;
  const d = floor0.d;
  const nF = spec.floors.length;
  const perCell = Math.round(cs / VOXEL);
  const vw = w * perCell;
  const vd = d * perCell;
  const n = Math.round(cs / LATTICE);
  const pv = n / perCell;
  const lw = w * n;
  const ld = d * n;
  const r = DRONE_RADIUS;
  const map: FlightMap = {
    vw,
    vd,
    reachable: new Uint8Array(nF * vw * vd),
    tx: new Float32Array(nF * vw * vd),
    ty: new Float32Array(nF * vw * vd),
    count: 0,
    cellPass: new Uint8Array(nF * w * d),
    wx: new Float32Array(nF * w * d),
    wy: new Float32Array(nF * w * d),
    fine: new Uint8Array(nF * w * d),
  };

  for (const box of index.boxes) {
    const zf = box.floor * H + FLIGHT_ALTITUDE;
    if (Math.abs(zf - box.cz) - box.hz >= r) continue;
    const [ex, ey] = boxExtent(box);
    const x0 = Math.max(0, Math.floor((box.cx - ex - r) / cs));
    const x1 = Math.min(w - 1, Math.floor((box.cx + ex + r) / cs));
    const y0 = Math.max(0, Math.floor((box.cy - ey - r) / cs));
    const y1 = Math.min(d - 1, Math.floor((box.cy + ey + r) / cs));
    for (let gz = y0; gz <= y1; gz++) {
      for (let gx = x0; gx <= x1; gx++) map.fine[(box.floor * d + gz) * w + gx] = 1;
    }
  }

  // Lattice state per floor: 0 unknown, 1 free, 2 blocked, 3 reached. Only floors with fine cells get one.
  const lattice: (Uint8Array | null)[] = [];
  for (let f = 0; f < nF; f++) {
    let any = false;
    for (let i = 0; i < w * d && !any; i++) any = map.fine[f * w * d + i] === 1;
    lattice.push(any ? new Uint8Array(lw * ld) : null);
  }
  const visitPoint = (f: number, X: number, Y: number) => {
    const grid = lattice[f]!;
    const i = Y * lw + X;
    let v = grid[i]!;
    if (v === 0) {
      const px = (X + 0.5) * LATTICE;
      const py = (Y + 0.5) * LATTICE;
      v = wallHits(spec, f, px, py, r) || index.hits(px, py, f * H + FLIGHT_ALTITUDE, r) ? 2 : 1;
    }
    if (v !== 1) {
      grid[i] = v;
      return;
    }
    grid[i] = 3;
    queue.push(f * lw * ld + i);
  };

  const cellSeen = new Uint8Array(nF * w * d);
  // Node encoding: coarse cell -> -(cellIndex + 1); lattice point -> f * lw * ld + Y * lw + X.
  const queue: number[] = [];
  const visitCell = (f: number, gx: number, gz: number) => {
    const ci = (f * d + gz) * w + gx;
    if (cellSeen[ci]) return;
    cellSeen[ci] = 1;
    queue.push(-(ci + 1));
  };
  // Step from a coarse cell into its neighbour (gx, gz), moving by (dx, dz).
  const enter = (f: number, gx: number, gz: number, dx: number, dz: number) => {
    if (gx < 0 || gz < 0 || gx >= w || gz >= d) return;
    if (!isWalkable(spec.floors[f]!.cells[gz * w + gx]!)) return;
    if (!map.fine[(f * d + gz) * w + gx]) {
      visitCell(f, gx, gz);
      return;
    }
    for (let k = 0; k < n; k++) {
      const X = dx === 0 ? gx * n + k : dx > 0 ? gx * n : gx * n + n - 1;
      const Y = dz === 0 ? gz * n + k : dz > 0 ? gz * n : gz * n + n - 1;
      visitPoint(f, X, Y);
    }
  };

  const sx = Math.floor(spec.spawn[0] / cs);
  const sy = Math.floor(spec.spawn[1] / cs);
  const sf = Math.max(0, Math.min(nF - 1, Math.floor(spec.spawn[2] / H)));
  if (sx >= 0 && sy >= 0 && sx < w && sy < d && isWalkable(spec.floors[sf]!.cells[sy * w + sx]!)) {
    if (!map.fine[(sf * d + sy) * w + sx]) visitCell(sf, sx, sy);
    else {
      visitPoint(
        sf,
        Math.max(0, Math.min(lw - 1, Math.floor(spec.spawn[0] / LATTICE))),
        Math.max(0, Math.min(ld - 1, Math.floor(spec.spawn[1] / LATTICE))),
      );
    }
  }

  const dirs = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ] as const;
  for (let head = 0; head < queue.length; head++) {
    const node = queue[head]!;
    if (node < 0) {
      const ci = -node - 1;
      const f = Math.floor(ci / (w * d));
      const rest = ci - f * w * d;
      const gz = Math.floor(rest / w);
      const gx = rest - gz * w;
      for (const [dx, dz] of dirs) enter(f, gx + dx, gz + dz, dx, dz);
      if (spec.floors[f]!.cells[gz * w + gx] === CELL.SHAFT) {
        for (const nf of [f + 1, f - 1]) {
          if (nf < 0 || nf >= nF) continue;
          if (spec.floors[nf]!.cells[gz * w + gx] !== CELL.SHAFT) continue;
          if (!map.fine[(nf * d + gz) * w + gx]) visitCell(nf, gx, gz);
        }
      }
      continue;
    }
    const f = Math.floor(node / (lw * ld));
    const rest = node - f * lw * ld;
    const Y = Math.floor(rest / lw);
    const X = rest - Y * lw;
    for (const [dx, dy] of dirs) {
      const nx = X + dx;
      const ny = Y + dy;
      if (nx < 0 || ny < 0 || nx >= lw || ny >= ld) continue;
      const gx = Math.floor(nx / n);
      const gz = Math.floor(ny / n);
      if (!isWalkable(spec.floors[f]!.cells[gz * w + gx]!)) continue;
      if (!map.fine[(f * d + gz) * w + gx]) visitCell(f, gx, gz);
      else visitPoint(f, nx, ny);
    }
  }

  for (let f = 0; f < nF; f++) {
    const grid = lattice[f];
    for (let gz = 0; gz < d; gz++) {
      for (let gx = 0; gx < w; gx++) {
        const ci = (f * d + gz) * w + gx;
        if (!map.fine[ci]) {
          if (!cellSeen[ci]) continue;
          map.cellPass[ci] = 1;
          map.wx[ci] = (gx + 0.5) * cs;
          map.wy[ci] = (gz + 0.5) * cs;
          for (let dz = 0; dz < perCell; dz++) {
            for (let dx = 0; dx < perCell; dx++) {
              const vx = gx * perCell + dx;
              const vy = gz * perCell + dz;
              const vi = f * vw * vd + vy * vw + vx;
              map.reachable[vi] = 1;
              map.tx[vi] = (vx + 0.5) * VOXEL;
              map.ty[vi] = (vy + 0.5) * VOXEL;
              map.count += 1;
            }
          }
          continue;
        }
        if (!grid) continue;
        let bestCell = Infinity;
        for (let dz = 0; dz < perCell; dz++) {
          for (let dx = 0; dx < perCell; dx++) {
            const vx = gx * perCell + dx;
            const vy = gz * perCell + dz;
            const vcx = (vx + 0.5) * VOXEL;
            const vcy = (vy + 0.5) * VOXEL;
            const vi = f * vw * vd + vy * vw + vx;
            let best = Infinity;
            for (let j = 0; j < pv; j++) {
              for (let i = 0; i < pv; i++) {
                const X = vx * pv + i;
                const Y = vy * pv + j;
                if (grid[Y * lw + X] !== 3) continue;
                const px = (X + 0.5) * LATTICE;
                const py = (Y + 0.5) * LATTICE;
                // Prefer goal points with some margin so a tracking controller does not graze.
                const tight = wallHits(spec, f, px, py, r + GOAL_MARGIN) || index.hits(px, py, f * H + FLIGHT_ALTITUDE, r, GOAL_MARGIN);
                const penalty = tight ? 100 : 0;
                const dv = (px - vcx) ** 2 + (py - vcy) ** 2 + penalty;
                if (dv < best) {
                  best = dv;
                  map.tx[vi] = px;
                  map.ty[vi] = py;
                }
                const dc = (px - (gx + 0.5) * cs) ** 2 + (py - (gz + 0.5) * cs) ** 2 + penalty;
                if (dc < bestCell) {
                  bestCell = dc;
                  map.wx[ci] = px;
                  map.wy[ci] = py;
                }
              }
            }
            if (best < Infinity) {
              map.reachable[vi] = 1;
              map.count += 1;
            }
          }
        }
        if (bestCell < Infinity) map.cellPass[ci] = 1;
      }
    }
  }
  return map;
}

const flightCache = new WeakMap<BuildingSpec, { boxes: unknown; map: FlightMap }>();

/** Cached flight map. Specs are treated as immutable once generated. */
export function flightMap(spec: BuildingSpec): FlightMap {
  const boxes = spec.furniture?.boxes;
  const hit = flightCache.get(spec);
  if (hit && hit.boxes === boxes) return hit.map;
  const map = computeFlightMap(spec);
  flightCache.set(spec, { boxes, map });
  return map;
}

/** Coverage denominator (BuildingSpec.reachableVoxels). See computeFlightMap for the rule. */
export function countReachableVoxels(spec: BuildingSpec): number {
  return flightMap(spec).count;
}

/** Every room on every floor still has a cell the drone can reach at flight altitude. */
export function isFullyReachable(spec: BuildingSpec, map = flightMap(spec)): boolean {
  const floor0 = spec.floors[0]!;
  for (let f = 0; f < spec.floors.length; f++) {
    const floor = spec.floors[f]!;
    if (floor.rooms.length === 0) return false;
    for (const room of floor.rooms) {
      let ok = false;
      for (let z = room.z; z < room.z + room.d && !ok; z++) {
        for (let x = room.x; x < room.x + room.w; x++) {
          if (x < 0 || z < 0 || x >= floor.w || z >= floor.d) continue;
          if (map.cellPass[(f * floor0.d + z) * floor0.w + x]) {
            ok = true;
            break;
          }
        }
      }
      if (!ok) return false;
    }
  }
  return map.count > 0;
}


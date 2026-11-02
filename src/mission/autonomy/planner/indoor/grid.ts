/**
 * The building grid in the building-local frame (world minus shell.origin): frame conversions, the
 * floor-plan cells, and the truth-side path and line-of-sight queries the oracle reference plans on.
 * Phases import only the frame conversions, through ./index.ts.
 */
import { isWalkable } from '../../../../gen/validate';
import { CELL, DRONE_RADIUS } from '../../../../shared/constants';
import { type BuildingSpec } from '../../../../shared/types';
import { type MissionBuilding, type V3 } from '../../contracts';
import { clamp } from '../geometry';

/** Flight height above each floor, meters. */
export const INDOOR_FLY_Z = 1.2;
/** Range the visibility estimate trusts, meters. */
export const SEE_RANGE = 6;
/** Range the sensor-map look estimate trusts, meters (shorter: the map is coarser than the floor plan). */
export const LOOK_RANGE = 5;
/** Half of the horizontal angle the estimate trusts, radians. */
export const SEE_HALF_FOV = (55 * Math.PI) / 180;
/** Free air kept around the airframe on planned paths, meters. */
export const PATH_CLEARANCE = DRONE_RADIUS + 0.17;

export interface IndoorGrid {
  building: MissionBuilding;
  spec: BuildingSpec;
  origin: V3;
  w: number;
  d: number;
  floors: number;
  cs: number;
  H: number;
  /** Per floor: 1 where the planner may fly. The outer wall ring is excluded. */
  nav: Uint8Array[];
}

export interface IndoorCell {
  f: number;
  x: number;
  z: number;
}

export function indoorGrid(building: MissionBuilding): IndoorGrid {
  const spec = building.shell.spec;
  const w = spec.floors[0]!.w;
  const d = spec.floors[0]!.d;
  const nav = spec.floors.map((floor) => {
    const mask = new Uint8Array(w * d);
    for (let z = 1; z < d - 1; z++) {
      for (let x = 1; x < w - 1; x++) if (isWalkable(floor.cells[z * w + x]!)) mask[z * w + x] = 1;
    }
    return mask;
  });
  return {
    building,
    spec,
    origin: [building.shell.origin[0], building.shell.origin[1], building.shell.origin[2]],
    w,
    d,
    floors: spec.floors.length,
    cs: spec.cellSize,
    H: spec.floorHeight,
    nav,
  };
}

export function toLocal(grid: IndoorGrid, p: V3): V3 {
  return [p[0] - grid.origin[0], p[1] - grid.origin[1], p[2] - grid.origin[2]];
}

export function toWorld(grid: IndoorGrid, p: V3): V3 {
  return [p[0] + grid.origin[0], p[1] + grid.origin[1], p[2] + grid.origin[2]];
}

export function floorOf(grid: IndoorGrid, z: number): number {
  return clamp(Math.floor(z / grid.H), 0, grid.floors - 1);
}

export function cellAt(grid: IndoorGrid, local: V3): IndoorCell {
  return {
    f: floorOf(grid, local[2]),
    x: clamp(Math.floor(local[0] / grid.cs), 0, grid.w - 1),
    z: clamp(Math.floor(local[1] / grid.cs), 0, grid.d - 1),
  };
}

export function cellCenter(grid: IndoorGrid, c: IndoorCell): V3 {
  return [(c.x + 0.5) * grid.cs, (c.z + 0.5) * grid.cs, c.f * grid.H + INDOOR_FLY_Z];
}

function isNav(grid: IndoorGrid, f: number, x: number, z: number): boolean {
  if (f < 0 || f >= grid.floors || x < 0 || z < 0 || x >= grid.w || z >= grid.d) return false;
  return grid.nav[f]![z * grid.w + x] === 1;
}

function isOpen(grid: IndoorGrid, f: number, x: number, z: number): boolean {
  if (f < 0 || f >= grid.floors || x < 0 || z < 0 || x >= grid.w || z >= grid.d) return false;
  return isWalkable(grid.spec.floors[f]!.cells[z * grid.w + x]!);
}

function key(grid: IndoorGrid, c: IndoorCell): number {
  return (c.f * grid.d + c.z) * grid.w + c.x;
}

export function unkey(grid: IndoorGrid, k: number): IndoorCell {
  const x = k % grid.w;
  const z = Math.floor(k / grid.w) % grid.d;
  const f = Math.floor(k / (grid.w * grid.d));
  return { f, x, z };
}

/** Nearest navigable cell on the same floor, searching outward. */
export function nearestNav(grid: IndoorGrid, local: V3): IndoorCell | null {
  const c = cellAt(grid, local);
  if (isNav(grid, c.f, c.x, c.z)) return c;
  let best: IndoorCell | null = null;
  let bestD = Infinity;
  for (let z = 0; z < grid.d; z++) {
    for (let x = 0; x < grid.w; x++) {
      if (!isNav(grid, c.f, x, z)) continue;
      const dd = Math.hypot((x + 0.5) * grid.cs - local[0], (z + 0.5) * grid.cs - local[1]);
      if (dd < bestD) {
        bestD = dd;
        best = { f: c.f, x, z };
      }
    }
  }
  return best;
}

function neighbours(grid: IndoorGrid, c: IndoorCell): { c: IndoorCell; cost: number }[] {
  const out: { c: IndoorCell; cost: number }[] = [];
  for (const [dx, dz] of [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ] as const) {
    if (isNav(grid, c.f, c.x + dx, c.z + dz)) out.push({ c: { f: c.f, x: c.x + dx, z: c.z + dz }, cost: 1 });
  }
  const here = grid.spec.floors[c.f]!.cells[c.z * grid.w + c.x];
  if (here === CELL.SHAFT) {
    for (const nf of [c.f + 1, c.f - 1]) {
      if (nf < 0 || nf >= grid.floors) continue;
      if (grid.spec.floors[nf]!.cells[c.z * grid.w + c.x] === CELL.SHAFT && isNav(grid, nf, c.x, c.z)) {
        out.push({ c: { f: nf, x: c.x, z: c.z }, cost: 3 });
      }
    }
  }
  return out;
}

/** Dijkstra over nav cells with shaft links. Returns distances and parents by cell key. */
export function gridDistances(grid: IndoorGrid, from: IndoorCell): { dist: Float64Array; prev: Int32Array } {
  const n = grid.floors * grid.w * grid.d;
  const dist = new Float64Array(n).fill(Infinity);
  const prev = new Int32Array(n).fill(-1);
  const start = key(grid, from);
  dist[start] = 0;
  const open: number[] = [start];
  while (open.length) {
    let bi = 0;
    for (let k = 1; k < open.length; k++) if (dist[open[k]!]! < dist[open[bi]!]!) bi = k;
    const cur = open[bi]!;
    open.splice(bi, 1);
    for (const { c, cost } of neighbours(grid, unkey(grid, cur))) {
      const nk = key(grid, c);
      const nd = dist[cur]! + cost;
      if (nd < dist[nk]!) {
        if (!Number.isFinite(dist[nk]!)) open.push(nk);
        dist[nk] = nd;
        prev[nk] = cur;
      }
    }
  }
  return { dist, prev };
}

/** Distance from a point to the nearest blocked cell edge on a floor, capped at 1 m. */
export function wallClearance(grid: IndoorGrid, f: number, x: number, y: number): number {
  let best = 1;
  const gx = Math.floor(x / grid.cs);
  const gz = Math.floor(y / grid.cs);
  for (let dz = -1; dz <= 1; dz++) {
    for (let dx = -1; dx <= 1; dx++) {
      const cx = gx + dx;
      const cz = gz + dz;
      if (isNav(grid, f, cx, cz)) continue;
      const nx = clamp(x, cx * grid.cs, (cx + 1) * grid.cs);
      const ny = clamp(y, cz * grid.cs, (cz + 1) * grid.cs);
      best = Math.min(best, Math.hypot(x - nx, y - ny));
    }
  }
  return best;
}

function segmentSafe(grid: IndoorGrid, f: number, a: V3, b: V3): boolean {
  const l = Math.hypot(b[0] - a[0], b[1] - a[1]);
  const n = Math.max(1, Math.ceil(l / 0.08));
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    if (wallClearance(grid, f, a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t) < PATH_CLEARANCE) return false;
  }
  return true;
}

/** Cells to local waypoints, string-pulled per floor while keeping PATH_CLEARANCE. */
export function cellsToPath(grid: IndoorGrid, cells: readonly IndoorCell[]): V3[] {
  const pts = cells.map((c) => cellCenter(grid, c));
  if (pts.length <= 2) return pts.slice(1);
  const out: V3[] = [];
  let anchor = 0;
  while (anchor < pts.length - 1) {
    let next = anchor + 1;
    const f = cells[anchor]!.f;
    if (cells[next]!.f === f) {
      for (let k = pts.length - 1; k > anchor + 1; k--) {
        let sameFloor = true;
        for (let j = anchor; j <= k; j++) if (cells[j]!.f !== f) sameFloor = false;
        if (sameFloor && segmentSafe(grid, f, pts[anchor]!, pts[k]!)) {
          next = k;
          break;
        }
      }
    }
    out.push(pts[next]!);
    anchor = next;
  }
  return out;
}

/** Local waypoint path between two local points, or null when unreachable. */
export function planIndoorPath(grid: IndoorGrid, from: V3, to: V3): V3[] | null {
  const a = nearestNav(grid, from);
  const b = nearestNav(grid, to);
  if (!a || !b) return null;
  const { dist, prev } = gridDistances(grid, a);
  const goal = key(grid, b);
  if (!Number.isFinite(dist[goal]!)) return null;
  const cells: IndoorCell[] = [];
  for (let k = goal; k >= 0; k = prev[k]!) cells.push(unkey(grid, k));
  cells.reverse();
  const first = cellCenter(grid, a);
  const path = cellsToPath(grid, cells);
  const out: V3[] = [];
  if (Math.hypot(first[0] - from[0], first[1] - from[1]) > 0.05 || Math.abs(first[2] - from[2]) > 0.2) out.push(first);
  out.push(...path);
  return out;
}

/** Line of sight on a floor grid between two local xy points. Walls block. */
export function gridLos(grid: IndoorGrid, f: number, ax: number, ay: number, bx: number, by: number): boolean {
  const l = Math.hypot(bx - ax, by - ay);
  const n = Math.max(1, Math.ceil(l / 0.1));
  for (let i = 1; i < n; i++) {
    const t = i / n;
    const x = Math.floor((ax + (bx - ax) * t) / grid.cs);
    const z = Math.floor((ay + (by - ay) * t) / grid.cs);
    if (!isOpen(grid, f, x, z)) return false;
  }
  return true;
}

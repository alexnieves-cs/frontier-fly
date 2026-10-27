import * as C from '../shared/constants';

// Module-local copies (imported bindings are getter calls under vite-node).
const LIDAR_H = C.LIDAR_H;
const LIDAR_MAX = C.LIDAR_MAX;
const VOXEL = C.VOXEL;

/**
 * The drone's own map of a building, and path queries over it: lidar occupancy
 * evidence per voxel, line of sight, clearance and shortest paths over the
 * voxels it has seen free or flown. Nothing here reads the floor plan.
 */

const DX = [1, -1, 0, 0, 1, 1, -1, -1];
const DY = [0, 0, 1, -1, 1, -1, 1, -1];
const STEP = [1, 1, 1, 1, Math.SQRT2, Math.SQRT2, Math.SQRT2, Math.SQRT2];

const RAY_COS = new Float64Array(LIDAR_H);
const RAY_SIN = new Float64Array(LIDAR_H);
for (let i = 0; i < LIDAR_H; i++) {
  RAY_COS[i] = Math.cos((i / LIDAR_H) * Math.PI * 2);
  RAY_SIN[i] = Math.sin((i / LIDAR_H) * Math.PI * 2);
}

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

/** The maps a sensor-only path query reads. */
export interface SensorGrid {
  seen: Uint8Array;
  visited: Uint8Array;
  occupied?: Uint8Array;
  vw: number;
  vd: number;
}

function gridOpen(g: SensorGrid, base: number, x: number, y: number): boolean {
  if (x < 0 || y < 0 || x >= g.vw || y >= g.vd) return false;
  const k = base + y * g.vw + x;
  return g.visited[k] === 1 || (g.seen[k] === 1 && !(g.occupied && g.occupied[k] === 1));
}

/**
 * Lidar occupancy evidence per voxel: how many returns landed in it and how
 * many rays passed through it. A voxel is occupied when returns outweigh
 * pass-throughs (hits * 2 > passes). A wall voxel with a hole smaller than the
 * drone (a broken window) takes most of the returns of the rays around it and
 * a few pass-throughs, so it stays blocked; a doorway whose jamb pokes into
 * the voxel takes a few grazing returns and many pass-throughs, so it stays
 * open. The seen-free map alone cannot tell these apart.
 */
export class OccupancyMap {
  readonly occupied: Uint8Array;
  private readonly hits: Uint8Array;
  private readonly passes: Uint8Array;
  private readonly fixed: Uint8Array;
  /** Mean height of the lidar returns in each voxel (NaN before the first), for expected(). */
  private readonly hitZ: Float32Array;

  constructor(
    readonly vw: number,
    readonly vd: number,
    floors: number,
  ) {
    const n = vw * vd * floors;
    this.occupied = new Uint8Array(n);
    this.hits = new Uint8Array(n);
    this.passes = new Uint8Array(n);
    this.fixed = new Uint8Array(n);
    this.hitZ = new Float32Array(n).fill(NaN);
  }

  /** Mark voxel k occupied whatever the lidar says (structure known some other way). */
  block(k: number): void {
    this.fixed[k] = 1;
    this.occupied[k] = 1;
  }

  private update(k: number): void {
    this.occupied[k] = this.fixed[k] || this.hits[k]! * 2 > this.passes[k]! ? 1 : 0;
  }

  /** Add one horizontal lidar ring taken at (x, y) on `floor`, from height `z` when known. */
  mark(floor: number, x: number, y: number, yaw: number, lidar: ArrayLike<number>, z = NaN): void {
    const { vw, vd } = this;
    const base = floor * vw * vd;
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    for (let i = 0; i < LIDAR_H; i++) {
      const r = lidar[i]!;
      // A beam flagged invalid (NaN) says nothing about free or occupied space.
      if (Number.isNaN(r)) continue;
      const ux = cy * RAY_COS[i]! - sy * RAY_SIN[i]!;
      const uy = sy * RAY_COS[i]! + cy * RAY_SIN[i]!;
      const hit = r < LIDAR_MAX - 1e-3;
      // Pass-throughs up to just short of the return (or the whole range on a miss).
      const free = hit ? r - 0.15 : LIDAR_MAX;
      const n = Math.max(1, Math.ceil(free / (VOXEL * 0.5)));
      let last = -1;
      for (let s = 1; s <= n; s++) {
        const d = (s / n) * free;
        const vx = Math.floor((x + ux * d) / VOXEL);
        const vy = Math.floor((y + uy * d) / VOXEL);
        if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) break;
        const k = base + vy * vw + vx;
        if (k === last) continue;
        last = k;
        if (this.passes[k]! < 255) this.passes[k] = this.passes[k]! + 1;
        this.update(k);
      }
      if (!hit) continue;
      // Step just past the surface so the return lands in the voxel behind it.
      const hx = Math.floor((x + ux * (r + 0.05)) / VOXEL);
      const hy = Math.floor((y + uy * (r + 0.05)) / VOXEL);
      if (hx < 0 || hy < 0 || hx >= vw || hy >= vd) continue;
      const k = base + hy * vw + hx;
      if (this.hits[k]! < 255) this.hits[k] = this.hits[k]! + 1;
      if (!Number.isNaN(z)) {
        const h = this.hitZ[k]!;
        this.hitZ[k] = Number.isNaN(h) ? z : h + (z - h) / Math.min(this.hits[k]!, 16);
      }
      this.update(k);
    }
  }

  /**
   * Range to the first occupied voxel along each ring beam from (x, y, z), into `out`; Infinity when
   * none within range. Only voxels with at least `minHits` lidar returns taken within `dz` of `z` count:
   * the map is 2D per floor, and a beam at another height can pass an opening, a sill or a table top.
   */
  expected(floor: number, x: number, y: number, z: number, yaw: number, out: Float64Array, minHits = 3, dz = 0.25): Float64Array {
    const { vw, vd } = this;
    const base = floor * vw * vd;
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const step = VOXEL * 0.5;
    for (let i = 0; i < LIDAR_H; i++) {
      const ux = cy * RAY_COS[i]! - sy * RAY_SIN[i]!;
      const uy = sy * RAY_COS[i]! + cy * RAY_SIN[i]!;
      out[i] = Infinity;
      for (let d = step; d <= LIDAR_MAX; d += step) {
        const vx = Math.floor((x + ux * d) / VOXEL);
        const vy = Math.floor((y + uy * d) / VOXEL);
        if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) break;
        const k = base + vy * vw + vx;
        // Only walls the lidar itself returned from at this height: the outer wall ring is blocked in and can hold window holes.
        if (this.occupied[k] && this.hits[k]! >= minHits && Math.abs(this.hitZ[k]! - z) <= dz) {
          out[i] = d;
          break;
        }
      }
    }
    return out;
  }
}

/** Voxels (0..2) between (x, y) and the nearest voxel that is neither seen free nor flown. */
function gridClearance(g: SensorGrid, base: number, x: number, y: number): number {
  if (!gridOpen(g, base, x, y)) return 0;
  for (let r = 1; r <= 2; r++) {
    for (let dy = -r; dy <= r; dy++) {
      for (let dx = -r; dx <= r; dx++) {
        if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue;
        if (!gridOpen(g, base, x + dx, y + dy)) return r;
      }
    }
  }
  return 3;
}

/** Straight line between two xy points crosses only seen-free or flown voxels. */
export function sensorLos(g: SensorGrid, floor: number, ax: number, ay: number, bx: number, by: number, skipLast = 0): boolean {
  const base = floor * g.vw * g.vd;
  const l = Math.hypot(bx - ax, by - ay);
  const n = Math.max(1, Math.ceil(l / (VOXEL * 0.4)));
  const end = Math.max(0, l - skipLast);
  for (let i = 1; i <= n; i++) {
    if ((i / n) * l > end) break;
    const t = i / n;
    if (!gridOpen(g, base, Math.floor((ax + (bx - ax) * t) / VOXEL), Math.floor((ay + (by - ay) * t) / VOXEL))) return false;
  }
  return true;
}

/** Free air around an xy point in the sensor map, meters (0, 0.5, 1 or 1.5: voxels to unknown space). */
export function sensorClearance(g: SensorGrid, floor: number, x: number, y: number): number {
  return gridClearance(g, floor * g.vw * g.vd, Math.floor(x / VOXEL), Math.floor(y / VOXEL)) * VOXEL;
}

/**
 * Shortest path on one floor over voxels the drone has seen free or flown,
 * kept off voxels next to unknown space where it can be, then string-pulled.
 * Returns xy waypoints after `from`, ending at `to`, or null when the maps
 * hold no route. Reads only the sensor maps.
 */
export function sensorPath(
  g: SensorGrid,
  floor: number,
  from: readonly [number, number],
  to: readonly [number, number],
): [number, number][] | null {
  const { vw, vd } = g;
  const base = floor * vw * vd;
  const sx = clamp(Math.floor(from[0] / VOXEL), 0, vw - 1);
  const sy = clamp(Math.floor(from[1] / VOXEL), 0, vd - 1);
  let tx = clamp(Math.floor(to[0] / VOXEL), 0, vw - 1);
  let ty = clamp(Math.floor(to[1] / VOXEL), 0, vd - 1);
  if (!gridOpen(g, base, tx, ty)) {
    let best = Infinity;
    let bx = -1;
    let by = -1;
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        if (!gridOpen(g, base, tx + dx, ty + dy)) continue;
        const d = dx * dx + dy * dy;
        if (d < best) {
          best = d;
          bx = tx + dx;
          by = ty + dy;
        }
      }
    }
    if (bx < 0) return null;
    tx = bx;
    ty = by;
  }
  const n = vw * vd;
  const dist = new Float64Array(n).fill(Infinity);
  const prev = new Int32Array(n).fill(-1);
  const start = sy * vw + sx;
  const goal = ty * vw + tx;
  dist[start] = 0;
  const open: number[] = [start];
  const settled = new Uint8Array(n);
  while (open.length) {
    let bi = 0;
    for (let i = 1; i < open.length; i++) if (dist[open[i]!]! < dist[open[bi]!]!) bi = i;
    const k = open[bi]!;
    open[bi] = open[open.length - 1]!;
    open.pop();
    if (settled[k]) continue;
    settled[k] = 1;
    if (k === goal) break;
    const x = k % vw;
    const y = (k - x) / vw;
    for (let d = 0; d < 8; d++) {
      const nx = x + DX[d]!;
      const ny = y + DY[d]!;
      if (!gridOpen(g, base, nx, ny)) continue;
      if (d >= 4 && (!gridOpen(g, base, nx, y) || !gridOpen(g, base, x, ny))) continue;
      const nk = ny * vw + nx;
      if (settled[nk]) continue;
      const cl = gridClearance(g, base, nx, ny);
      const nd = dist[k]! + STEP[d]! * (1 + (cl <= 1 ? 2.5 : cl === 2 ? 0.6 : 0));
      if (nd < dist[nk]!) {
        dist[nk] = nd;
        prev[nk] = k;
        open.push(nk);
      }
    }
  }
  if (!Number.isFinite(dist[goal]!)) return null;
  const cells: [number, number][] = [];
  for (let k = goal; k >= 0; k = prev[k]!) cells.push([((k % vw) + 0.5) * VOXEL, (Math.floor(k / vw) + 0.5) * VOXEL]);
  cells.reverse();
  cells[cells.length - 1] = [to[0], to[1]];
  // String-pull: from each anchor, jump to the farthest point still in clear line of sight.
  const out: [number, number][] = [];
  let ax = from[0];
  let ay = from[1];
  let i = 0;
  while (i < cells.length) {
    let j = cells.length - 1;
    while (j > i && !(sensorLos(g, floor, ax, ay, cells[j]![0], cells[j]![1]) && pathClear(g, base, ax, ay, cells[j]![0], cells[j]![1])))
      j--;
    out.push(cells[j]!);
    ax = cells[j]![0];
    ay = cells[j]![1];
    i = j + 1;
  }
  return out;
}

/** Every interior sample of a segment has at least one voxel of known free space around it. */
function pathClear(g: SensorGrid, base: number, ax: number, ay: number, bx: number, by: number): boolean {
  const l = Math.hypot(bx - ax, by - ay);
  const n = Math.max(1, Math.ceil(l / (VOXEL * 0.4)));
  for (let i = 1; i < n; i++) {
    const t = i / n;
    if (gridClearance(g, base, Math.floor((ax + (bx - ax) * t) / VOXEL), Math.floor((ay + (by - ay) * t) / VOXEL)) < 2) return false;
  }
  return true;
}

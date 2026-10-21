/**
 * The estimator's own map of one floor: a log-odds occupancy grid built from
 * its lidar at its estimated poses. Each occupied cell keeps the mean of the
 * returns that landed in it, so walls sit where the lidar saw them rather
 * than at cell centres (a cell-centre field would pull every match half a
 * cell into the wall, and that bias compounds as the map grows). An exact
 * Euclidean distance transform gives, for every cell, the nearest occupied
 * cell; the matcher then measures point-to-line distance to that cell's wall
 * (normal from its neighbours) or point-to-point at corners. It never sees
 * the floor plan.
 */

const HIT = 0.85;
const MISS = -0.4;
const CLAMP = 4;
/** Log-odds above this is occupied. One hit on fresh space is enough. */
const OCC = 0.5;
/** Squared distance of a cell with no occupied cell in reach; finite so the EDT arithmetic stays exact. */
const FAR = 1e10;
const INF = Infinity;
const scratch = new Float64Array(2);

export class LikelihoodGrid {
  readonly w: number;
  readonly h: number;
  readonly logOdds: Float32Array;
  /** Distance between cell centres to the nearest occupied cell, metres, capped at dMax. */
  readonly dist: Float32Array;
  /** Index of the nearest occupied cell, or -1. */
  readonly near: Int32Array;
  /** Sum of return positions and count per cell. */
  private readonly hitX: Float64Array;
  private readonly hitY: Float64Array;
  private readonly hitN: Uint32Array;
  /** Cached wall normal per occupied cell (NaN: not computed since the last refresh; 0,0: a corner). */
  private readonly nx: Float32Array;
  private readonly ny: Float32Array;
  occupiedCount = 0;
  private dirty = false;
  private readonly f: Float64Array;
  private readonly v: Int32Array;
  private readonly z: Float64Array;
  private readonly d1: Float64Array;
  private readonly sq: Float64Array;
  private readonly arg1: Int32Array;
  private readonly colArg: Int32Array;

  /**
   * @param ox world x of the grid's corner, m
   * @param oy world y of the grid's corner, m
   * @param width extent along x, m
   * @param height extent along y, m
   * @param res cell size, m
   * @param dMax distance field cap, m
   */
  constructor(
    readonly ox: number,
    readonly oy: number,
    width: number,
    height: number,
    readonly res = 0.1,
    readonly dMax = 1,
  ) {
    this.w = Math.max(1, Math.ceil(width / res));
    this.h = Math.max(1, Math.ceil(height / res));
    const n = this.w * this.h;
    this.logOdds = new Float32Array(n);
    this.dist = new Float32Array(n).fill(dMax);
    this.near = new Int32Array(n).fill(-1);
    this.hitX = new Float64Array(n);
    this.hitY = new Float64Array(n);
    this.hitN = new Uint32Array(n);
    this.nx = new Float32Array(n).fill(NaN);
    this.ny = new Float32Array(n).fill(NaN);
    this.colArg = new Int32Array(n);
    const m = Math.max(this.w, this.h);
    this.arg1 = new Int32Array(m);
    this.f = new Float64Array(m);
    this.v = new Int32Array(m);
    this.z = new Float64Array(m + 1);
    this.d1 = new Float64Array(m);
    this.sq = new Float64Array(n);
  }

  isOccupied(i: number): boolean {
    return this.logOdds[i]! > OCC;
  }

  /**
   * Seen free with every neighbour seen too. A return landing here contradicts
   * the map; one landing in a free cell beside unseen space is more likely the
   * first sight of a surface the free rays only grazed.
   */
  deepFree(cx: number, cy: number): boolean {
    if (cx < 1 || cy < 1 || cx >= this.w - 1 || cy >= this.h - 1) return false;
    const w = this.w;
    if (!(this.logOdds[cy * w + cx]! < -1.5)) return false;
    for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) if (this.logOdds[(cy + dy) * w + cx + dx] === 0) return false;
    return true;
  }

  /** True when any lidar return or free ray has touched the cell under (x, y). */
  known(x: number, y: number): boolean {
    const cx = Math.floor((x - this.ox) / this.res);
    const cy = Math.floor((y - this.oy) / this.res);
    if (cx < 0 || cy < 0 || cx >= this.w || cy >= this.h) return false;
    return this.logOdds[cy * this.w + cx] !== 0;
  }

  /**
   * Fold one sweep in at a pose: cells along each beam lose occupancy, the
   * cell at the return gains it. Misses (>= maxRange - 1e-3) mark nothing,
   * so a glass dropout never clears the glass.
   */
  integrate(x: number, y: number, yaw: number, ranges: ArrayLike<number>, maxRange: number): void {
    const n = ranges.length;
    const res = this.res;
    const w = this.w;
    const h = this.h;
    const sx = (x - this.ox) / res;
    const sy = (y - this.oy) / res;
    for (let i = 0; i < n; i++) {
      const r = ranges[i]!;
      if (!(r < maxRange - 1e-3) || r <= 0) continue;
      const a = yaw + (i / n) * Math.PI * 2;
      const ex = sx + (Math.cos(a) * r) / res;
      const ey = sy + (Math.sin(a) * r) / res;
      // DDA from the sensor cell to the return cell; every cell before the return is free.
      let cx = Math.floor(sx);
      let cy = Math.floor(sy);
      const tx = Math.floor(ex);
      const ty = Math.floor(ey);
      const dx = ex - sx;
      const dy = ey - sy;
      const stepX = dx > 0 ? 1 : -1;
      const stepY = dy > 0 ? 1 : -1;
      const tdx = dx !== 0 ? Math.abs(1 / dx) : Infinity;
      const tdy = dy !== 0 ? Math.abs(1 / dy) : Infinity;
      let tmx = dx > 0 ? (cx + 1 - sx) * tdx : dx < 0 ? (sx - cx) * tdx : Infinity;
      let tmy = dy > 0 ? (cy + 1 - sy) * tdy : dy < 0 ? (sy - cy) * tdy : Infinity;
      for (let guard = 0; guard < 4 * (w + h); guard++) {
        if (cx === tx && cy === ty) break;
        if (cx >= 0 && cy >= 0 && cx < w && cy < h) this.add(cy * w + cx, MISS);
        if (tmx < tmy) {
          cx += stepX;
          tmx += tdx;
        } else {
          cy += stepY;
          tmy += tdy;
        }
      }
      if (tx >= 0 && ty >= 0 && tx < w && ty < h) {
        const k = ty * w + tx;
        this.add(k, HIT);
        this.hitX[k] += this.ox + ex * res;
        this.hitY[k] += this.oy + ey * res;
        this.hitN[k] += 1;
        this.dirty = true;
      }
    }
  }

  private add(k: number, delta: number): void {
    const before = this.logOdds[k]!;
    let after = before + delta;
    if (after > CLAMP) after = CLAMP;
    else if (after < -CLAMP) after = -CLAMP;
    this.logOdds[k] = after;
    const was = before > OCC;
    const now = after > OCC;
    if (was !== now) {
      this.dirty = true;
      this.occupiedCount += now ? 1 : -1;
    }
  }

  /** Recompute the distance field if the occupied set changed (exact EDT, Felzenszwalb & Huttenlocher 2012). */
  refresh(): void {
    if (!this.dirty) return;
    this.dirty = false;
    const { w, h, sq } = this;
    for (let k = 0; k < w * h; k++) sq[k] = this.logOdds[k]! > OCC ? 0 : FAR;
    const f = this.f;
    const d1 = this.d1;
    const arg = this.arg1;
    for (let x = 0; x < w; x++) {
      for (let y = 0; y < h; y++) f[y] = sq[y * w + x]!;
      this.edt1(h);
      for (let y = 0; y < h; y++) {
        sq[y * w + x] = d1[y]!;
        this.colArg[y * w + x] = arg[y]!;
      }
    }
    const capSq = (this.dMax / this.res) ** 2;
    for (let y = 0; y < h; y++) {
      const row = y * w;
      for (let x = 0; x < w; x++) f[x] = sq[row + x]!;
      this.edt1(w);
      for (let x = 0; x < w; x++) {
        const v = d1[x]!;
        if (v >= capSq) {
          this.dist[row + x] = this.dMax;
          this.near[row + x] = -1;
        } else {
          this.dist[row + x] = Math.sqrt(v) * this.res;
          const bx = arg[x]!;
          this.near[row + x] = this.colArg[row + bx]! * w + bx;
        }
      }
    }
    this.nx.fill(NaN);
    this.ny.fill(NaN);
  }

  /** 1D squared distance transform of f[0..n) into d1, with the minimizing index in arg1. */
  private edt1(n: number): void {
    const { f, v, z, d1, arg1 } = this;
    let k = 0;
    v[0] = 0;
    z[0] = -INF;
    z[1] = INF;
    for (let q = 1; q < n; q++) {
      let s = (f[q]! + q * q - (f[v[k]!]! + v[k]! * v[k]!)) / (2 * q - 2 * v[k]!);
      while (s <= z[k]!) {
        k -= 1;
        s = (f[q]! + q * q - (f[v[k]!]! + v[k]! * v[k]!)) / (2 * q - 2 * v[k]!);
      }
      k += 1;
      v[k] = q;
      z[k] = s;
      z[k + 1] = INF;
    }
    k = 0;
    for (let q = 0; q < n; q++) {
      while (z[k + 1]! < q) k += 1;
      const dq = q - v[k]!;
      d1[q] = dq * dq + f[v[k]!]!;
      arg1[q] = v[k]!;
    }
  }

  /** Mean return position in an occupied cell. */
  hitPoint(k: number, out: Float64Array): void {
    const n = this.hitN[k]!;
    if (n > 0) {
      out[0] = this.hitX[k]! / n;
      out[1] = this.hitY[k]! / n;
    } else {
      out[0] = this.ox + ((k % this.w) + 0.5) * this.res;
      out[1] = this.oy + (Math.floor(k / this.w) + 0.5) * this.res;
    }
  }

  /**
   * Wall normal at an occupied cell from the returns of the occupied cells
   * within two cells (principal axes). [2, 0] at a corner or clutter (match
   * point to point); [0, 0] with fewer than three returns around, where the
   * direction of the wall is unknown and the return is better left out than
   * allowed to pin the pose along the wall.
   */
  normal(k: number, out: Float64Array): void {
    if (Number.isNaN(this.nx[k]!)) {
      const w = this.w;
      const cx = k % w;
      const cy = (k - cx) / w;
      let n = 0;
      let mx = 0;
      let my = 0;
      let sxx = 0;
      let sxy = 0;
      let syy = 0;
      for (let dy = -2; dy <= 2; dy++) {
        const yy = cy + dy;
        if (yy < 0 || yy >= this.h) continue;
        for (let dx = -2; dx <= 2; dx++) {
          const xx = cx + dx;
          if (xx < 0 || xx >= w) continue;
          const j = yy * w + xx;
          if (!(this.logOdds[j]! > OCC) || this.hitN[j] === 0) continue;
          const px = this.hitX[j]! / this.hitN[j]!;
          const py = this.hitY[j]! / this.hitN[j]!;
          n += 1;
          mx += px;
          my += py;
          sxx += px * px;
          sxy += px * py;
          syy += py * py;
        }
      }
      let nx = 0;
      let ny = 0;
      if (n >= 3) {
        mx /= n;
        my /= n;
        const a = sxx / n - mx * mx;
        const b = sxy / n - mx * my;
        const c = syy / n - my * my;
        const tr = (a + c) / 2;
        const disc = Math.sqrt(Math.max(0, ((a - c) / 2) ** 2 + b * b));
        const lMin = tr - disc;
        const lMax = tr + disc;
        // A line: one direction much flatter than the other.
        if (lMax > 1e-6 && lMin < 0.08 * lMax) {
          // Eigenvector of the smaller eigenvalue; of the two equivalent forms, the better conditioned.
          const e1x = b;
          const e1y = lMin - a;
          const e2x = lMin - c;
          const e2y = b;
          const use1 = Math.hypot(e1x, e1y) >= Math.hypot(e2x, e2y);
          const ex = use1 ? e1x : e2x;
          const ey = use1 ? e1y : e2y;
          const l = Math.hypot(ex, ey);
          if (l > 1e-12) {
            nx = ex / l;
            ny = ey / l;
          }
        } else if (lMax > 1e-6) {
          // A corner or clutter: point-to-point.
          nx = 2;
        }
      }
      this.nx[k] = nx;
      this.ny[k] = ny;
    }
    out[0] = this.nx[k]!;
    out[1] = this.ny[k]!;
  }


  /**
   * Residual of a point against the map, written to out: [r, dr/dx, dr/dy,
   * gap]. r is the signed distance to the nearest wall's line (or to its
   * return point at a corner); gap is the plain distance to that return,
   * which the matcher uses to reject outliers. Far from any wall: gap = dMax.
   */
  residual(x: number, y: number, out: Float64Array): void {
    const cx = Math.floor((x - this.ox) / this.res);
    const cy = Math.floor((y - this.oy) / this.res);
    const k = cx < 0 || cy < 0 || cx >= this.w || cy >= this.h ? -1 : this.near[cy * this.w + cx]!;
    if (k < 0) {
      out[0] = 0;
      out[1] = 0;
      out[2] = 0;
      out[3] = this.dMax;
      return;
    }
    const q = scratch;
    this.hitPoint(k, q);
    const dx = x - q[0]!;
    const dy = y - q[1]!;
    const gap = Math.hypot(dx, dy);
    this.normal(k, q);
    const nx = q[0]!;
    const ny = q[1]!;
    if (nx === 0 && ny === 0) {
      // Unknown wall direction: no residual.
      out[0] = 0;
      out[1] = 0;
      out[2] = 0;
      out[3] = this.dMax;
      return;
    }
    if (nx !== 2) {
      out[0] = nx * dx + ny * dy;
      out[1] = nx;
      out[2] = ny;
    } else if (gap > 1e-9) {
      out[0] = gap;
      out[1] = dx / gap;
      out[2] = dy / gap;
    } else {
      out[0] = 0;
      out[1] = 0;
      out[2] = 0;
    }
    out[3] = gap;
  }

  /**
   * Bilinear distance at (x, y) and its gradient, written to out[0..3).
   * Outside the grid reads dMax with zero gradient.
   */
  sample(x: number, y: number, out: Float64Array): void {
    const res = this.res;
    const fx = (x - this.ox) / res - 0.5;
    const fy = (y - this.oy) / res - 0.5;
    const ix = Math.floor(fx);
    const iy = Math.floor(fy);
    if (ix < 0 || iy < 0 || ix + 1 >= this.w || iy + 1 >= this.h) {
      out[0] = this.dMax;
      out[1] = 0;
      out[2] = 0;
      return;
    }
    const tx = fx - ix;
    const ty = fy - iy;
    const k = iy * this.w + ix;
    const d00 = this.dist[k]!;
    const d10 = this.dist[k + 1]!;
    const d01 = this.dist[k + this.w]!;
    const d11 = this.dist[k + this.w + 1]!;
    out[0] = (d00 * (1 - tx) + d10 * tx) * (1 - ty) + (d01 * (1 - tx) + d11 * tx) * ty;
    out[1] = ((d10 - d00) * (1 - ty) + (d11 - d01) * ty) / res;
    out[2] = ((d01 - d00) * (1 - tx) + (d11 - d10) * tx) / res;
  }
}

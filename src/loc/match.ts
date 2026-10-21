import type { LikelihoodGrid } from './grid';

/**
 * Scan-to-map matching. Gauss-Newton over (x, y, yaw) minimizes the squared
 * point-to-line distance from each lidar return to the nearest mapped wall
 * (LikelihoodGrid.residual), plus the odometry prior as a Mahalanobis term, so a direction
 * the scan can't pin down (along a bare corridor) stays where odometry put
 * it instead of sliding. Robust: Huber weights, and returns farther than
 * `outlier` from any mapped wall (new rooms, open doors) are left out.
 */

export interface MatchOptions {
  /** Range noise the information is scaled by, m. Calibrated on dev seeds (see loc-study). */
  sigma: number;
  /** Huber knee, m. */
  huber: number;
  /** Returns farther than this from the map are outliers, m. */
  outlier: number;
  /** A return within this of the map counts as an inlier, m. */
  inlier: number;
  iterations: number;
}

/** Tuned on the dev seeds (scripts/study/loc-study.ts --dev); sigma absorbs beam correlation and map error, not just range noise. */
export const DEFAULT_MATCH: MatchOptions = { sigma: 0.25, huber: 0.08, outlier: 0.4, inlier: 0.12, iterations: 8 };

export interface MatchResult {
  x: number;
  y: number;
  yaw: number;
  /** Row-major 3x3 information the scan alone contributes at the result. */
  info: Float64Array;
  /** Returns that landed within `outlier` of the map. */
  used: number;
  /** Returns within `inlier` of the map. */
  inliers: number;
  /** Valid returns (not misses) in the sweep. */
  valid: number;
}

const sample = new Float64Array(4);

/** Inverse of a symmetric 3x3 into out; false when singular. */
export function invert3(m: ArrayLike<number>, out: Float64Array): boolean {
  const a = m[0]!, b = m[1]!, c = m[2]!, d = m[3]!, e = m[4]!, f = m[5]!, g = m[6]!, h = m[7]!, i = m[8]!;
  const A = e * i - f * h;
  const B = -(d * i - f * g);
  const C = d * h - e * g;
  const det = a * A + b * B + c * C;
  if (!(Math.abs(det) > 1e-300)) return false;
  const inv = 1 / det;
  out[0] = A * inv;
  out[1] = -(b * i - c * h) * inv;
  out[2] = (b * f - c * e) * inv;
  out[3] = B * inv;
  out[4] = (a * i - c * g) * inv;
  out[5] = -(a * f - c * d) * inv;
  out[6] = C * inv;
  out[7] = -(a * h - b * g) * inv;
  out[8] = (a * e - b * d) * inv;
  return true;
}

/**
 * Eigen-decomposition of a symmetric 3x3 (cyclic Jacobi). Writes the
 * eigenvalues to vals and the eigenvectors as the columns of vecs (row-major).
 */
export function eigSym3(m: ArrayLike<number>, vals: Float64Array, vecs: Float64Array): void {
  const a = Float64Array.from(m as ArrayLike<number>);
  vecs.fill(0);
  vecs[0] = 1;
  vecs[4] = 1;
  vecs[8] = 1;
  for (let sweep = 0; sweep < 24; sweep++) {
    const off = a[1]! * a[1]! + a[2]! * a[2]! + a[5]! * a[5]!;
    if (off < 1e-24 * (a[0]! * a[0]! + a[4]! * a[4]! + a[8]! * a[8]! + 1e-300)) break;
    for (const [p, q] of [
      [0, 1],
      [0, 2],
      [1, 2],
    ] as const) {
      const apq = a[p * 3 + q]!;
      if (Math.abs(apq) < 1e-300) continue;
      const app = a[p * 3 + p]!;
      const aqq = a[q * 3 + q]!;
      const theta = (aqq - app) / (2 * apq);
      const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1);
      const s = t * c;
      for (let k = 0; k < 3; k++) {
        const akp = a[k * 3 + p]!;
        const akq = a[k * 3 + q]!;
        a[k * 3 + p] = c * akp - s * akq;
        a[k * 3 + q] = s * akp + c * akq;
      }
      for (let k = 0; k < 3; k++) {
        const apk = a[p * 3 + k]!;
        const aqk = a[q * 3 + k]!;
        a[p * 3 + k] = c * apk - s * aqk;
        a[q * 3 + k] = s * apk + c * aqk;
      }
      for (let k = 0; k < 3; k++) {
        const vkp = vecs[k * 3 + p]!;
        const vkq = vecs[k * 3 + q]!;
        vecs[k * 3 + p] = c * vkp - s * vkq;
        vecs[k * 3 + q] = s * vkp + c * vkq;
      }
    }
  }
  vals[0] = a[0]!;
  vals[1] = a[4]!;
  vals[2] = a[8]!;
}

/** Body-frame endpoints of the valid returns of a sweep. */
export function endpoints(ranges: ArrayLike<number>, maxRange: number): { bx: Float64Array; by: Float64Array; n: number } {
  const total = ranges.length;
  const bx = new Float64Array(total);
  const by = new Float64Array(total);
  let n = 0;
  for (let i = 0; i < total; i++) {
    const r = ranges[i]!;
    if (!(r < maxRange - 1e-3) || r <= 0.05) continue;
    const a = (i / total) * Math.PI * 2;
    bx[n] = Math.cos(a) * r;
    by[n] = Math.sin(a) * r;
    n += 1;
  }
  return { bx, by, n };
}

export function matchScan(
  grid: LikelihoodGrid,
  pts: { bx: Float64Array; by: Float64Array; n: number },
  x0: number,
  y0: number,
  yaw0: number,
  priorInfo: Float64Array | null,
  opts: MatchOptions = DEFAULT_MATCH,
): MatchResult {
  let x = x0;
  let y = y0;
  let yaw = yaw0;
  const A = new Float64Array(9);
  const g = new Float64Array(3);
  const inv = new Float64Array(9);
  const info = new Float64Array(9);
  const w2 = 1 / (opts.sigma * opts.sigma);
  let used = 0;
  let inliers = 0;
  for (let it = 0; it <= opts.iterations; it++) {
    A.fill(0);
    g.fill(0);
    info.fill(0);
    used = 0;
    inliers = 0;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    for (let k = 0; k < pts.n; k++) {
      const bx = pts.bx[k]!;
      const by = pts.by[k]!;
      grid.residual(x + c * bx - s * by, y + s * bx + c * by, sample);
      const d = sample[0]!;
      const gap = sample[3]!;
      if (gap >= opts.outlier) continue;
      used += 1;
      const ad = Math.abs(d);
      if (ad < opts.inlier) inliers += 1;
      const w = (ad <= opts.huber ? 1 : opts.huber / ad) * w2;
      const j0 = sample[1]!;
      const j1 = sample[2]!;
      const j2 = j0 * (-s * bx - c * by) + j1 * (c * bx - s * by);
      info[0] += w * j0 * j0;
      info[1] += w * j0 * j1;
      info[2] += w * j0 * j2;
      info[4] += w * j1 * j1;
      info[5] += w * j1 * j2;
      info[8] += w * j2 * j2;
      g[0] += w * j0 * d;
      g[1] += w * j1 * d;
      g[2] += w * j2 * d;
    }
    info[3] = info[1];
    info[6] = info[2];
    info[7] = info[5];
    if (it === opts.iterations) break;
    for (let k = 0; k < 9; k++) A[k] = info[k]!;
    if (priorInfo) {
      const ex = x - x0;
      const ey = y - y0;
      let ew = yaw - yaw0;
      ew = Math.atan2(Math.sin(ew), Math.cos(ew));
      for (let r = 0; r < 3; r++) {
        g[r] += priorInfo[r * 3]! * ex + priorInfo[r * 3 + 1]! * ey + priorInfo[r * 3 + 2]! * ew;
      }
      for (let k = 0; k < 9; k++) A[k] += priorInfo[k]!;
    }
    // Levenberg damping keeps a step on an almost flat field small.
    A[0] += 1e-6 + A[0] * 1e-3;
    A[4] += 1e-6 + A[4] * 1e-3;
    A[8] += 1e-6 + A[8] * 1e-3;
    if (!invert3(A, inv)) break;
    const dx = -(inv[0]! * g[0]! + inv[1]! * g[1]! + inv[2]! * g[2]!);
    const dy = -(inv[3]! * g[0]! + inv[4]! * g[1]! + inv[5]! * g[2]!);
    let dw = -(inv[6]! * g[0]! + inv[7]! * g[1]! + inv[8]! * g[2]!);
    // One step never jumps farther than the field can see.
    const step = Math.hypot(dx, dy);
    const cap = opts.outlier * 0.5;
    const sc = step > cap ? cap / step : 1;
    dw = Math.max(-0.1, Math.min(0.1, dw));
    x += dx * sc;
    y += dy * sc;
    yaw += dw;
    if (step * sc < 1e-4 && Math.abs(dw) < 1e-5) {
      // Converged: one more pass to report the information at the result.
      it = opts.iterations - 1;
    }
  }
  return { x, y, yaw, info, used, inliers, valid: pts.n };
}

/**
 * Brute-force correlative search (Olson 2009) around a guess: every pose on
 * a grid of offsets is scored by how close its returns land to mapped walls.
 * Used to re-acquire after the estimator loses track. Returns the best pose
 * and the fraction of returns it puts within `inlier` of the map.
 */
export function searchScan(
  grid: LikelihoodGrid,
  pts: { bx: Float64Array; by: Float64Array; n: number },
  x0: number,
  y0: number,
  yaw0: number,
  halfXY: number,
  halfYaw: number,
  opts: MatchOptions = DEFAULT_MATCH,
): { x: number; y: number; yaw: number; score: number } {
  const stepXY = grid.res;
  const stepYaw = Math.max(0.01, Math.min(0.03, grid.res / 6));
  const nXY = Math.max(0, Math.round(halfXY / stepXY));
  const nYaw = Math.max(0, Math.round(halfYaw / stepYaw));
  const stride = pts.n > 60 ? 2 : 1;
  const m = Math.ceil(pts.n / stride);
  const rx = new Float64Array(m);
  const ry = new Float64Array(m);
  let best = { x: x0, y: y0, yaw: yaw0, score: -1 };
  const inl = opts.inlier;
  for (let a = -nYaw; a <= nYaw; a++) {
    const yaw = yaw0 + a * stepYaw;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    for (let k = 0, j = 0; k < pts.n; k += stride, j++) {
      rx[j] = c * pts.bx[k]! - s * pts.by[k]!;
      ry[j] = s * pts.bx[k]! + c * pts.by[k]!;
    }
    for (let iy = -nXY; iy <= nXY; iy++) {
      const y = y0 + iy * stepXY;
      for (let ix = -nXY; ix <= nXY; ix++) {
        const x = x0 + ix * stepXY;
        let score = 0;
        for (let j = 0; j < m; j++) {
          const cx = Math.floor((x + rx[j]! - grid.ox) / grid.res);
          const cy = Math.floor((y + ry[j]! - grid.oy) / grid.res);
          if (cx < 0 || cy < 0 || cx >= grid.w || cy >= grid.h) continue;
          const d = grid.dist[cy * grid.w + cx]!;
          if (d < inl) score += 1 - d / (2 * inl);
        }
        // Prefer the smaller offset on a tie, so a symmetric room doesn't pull the pose away.
        score -= 1e-3 * (Math.abs(ix) + Math.abs(iy) + Math.abs(a));
        if (score > best.score) best = { x, y, yaw, score };
      }
    }
  }
  return { ...best, score: m ? Math.max(0, best.score) / m : 0 };
}

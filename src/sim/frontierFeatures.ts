import * as C from '../shared/constants';

// Module-local copies (imported bindings are getter calls under vite-node).
const COARSE_VOXELS = C.COARSE_VOXELS;
const LIDAR_MAX = C.LIDAR_MAX;
const OBS = C.OBS;
const VOXEL = C.VOXEL;

/** BFS pops at most this many voxels per step (a 20x20 m floor is 1600). */
export const BFS_MAX_EXPANSIONS = 2048;
/** Waypoint lookahead along the BFS path, meters. */
export const LOOKAHEAD = 1.25;
/** Path length, cluster distance and stall normalizers. */
export const PATH_NORM = 20;
export const CLUSTER_NORM = 32;
export const CLUSTER_MASS_NORM = 128;
export const FRONTIER_TOTAL_NORM = 512;
export const STALL_NORM = 50;

const DIRS_X = [1, -1, 0, 0, 1, 1, -1, -1];
const DIRS_Y = [0, 0, 1, -1, 1, -1, 1, -1];

/** Write the body-frame unit vector (forward, left) of a world offset; returns its length. */
function bodyUnit(out: Float32Array, at: number, dx: number, dy: number, cy: number, sy: number): number {
  const fx = cy * dx + sy * dy;
  const fy = -sy * dx + cy * dy;
  const len = Math.hypot(fx, fy);
  out[at] = len > 1e-6 ? fx / len : 0;
  out[at + 1] = len > 1e-6 ? fy / len : 0;
  return len;
}

export interface FrontierResult {
  found: boolean;
  truncated: boolean;
  /** World waypoint (meters), valid when found. */
  wx: number;
  wy: number;
  /** World target voxel center, valid when found. */
  tx: number;
  ty: number;
  pathMeters: number;
  expansions: number;
}

/**
 * Frontier navigation features from the sensor map only. Frontier voxels are
 * seen-free and not visited; traversable voxels are seen-free or visited. All
 * scratch is typed arrays reused across steps; nothing allocates per call once
 * the grid size is known.
 */
export class FrontierNav {
  private queue = new Int32Array(0);
  private parent = new Int32Array(0);
  private stamp = new Uint32Array(0);
  private epoch = 0;
  private ccount = new Int32Array(0);
  private csumX = new Float64Array(0);
  private csumY = new Float64Array(0);
  private cstamp = new Uint32Array(0);
  private cstack = new Int32Array(0);
  readonly last: FrontierResult = { found: false, truncated: false, wx: 0, wy: 0, tx: 0, ty: 0, pathMeters: 0, expansions: 0 };

  private ensure(n: number, cn: number): void {
    if (this.queue.length < n) {
      this.queue = new Int32Array(n);
      this.parent = new Int32Array(n);
      this.stamp = new Uint32Array(n);
      this.epoch = 0;
    }
    if (this.ccount.length < cn) {
      this.ccount = new Int32Array(cn);
      this.csumX = new Float64Array(cn);
      this.csumY = new Float64Array(cn);
      this.cstamp = new Uint32Array(cn);
      this.cstack = new Int32Array(cn);
    }
  }

  private nextEpoch(): number {
    this.epoch += 1;
    if (this.epoch >= 0xffffffff) {
      this.stamp.fill(0);
      this.cstamp.fill(0);
      this.epoch = 1;
    }
    return this.epoch;
  }

  /** BFS from the drone voxel to the nearest frontier voxel. Fills `last`. */
  search(visited: Uint8Array, seen: Uint8Array, vw: number, vd: number, floor: number, floors: number, x: number, y: number): FrontierResult {
    const r = this.last;
    r.found = false;
    r.truncated = false;
    r.pathMeters = 0;
    r.expansions = 0;
    const n = vw * vd;
    this.ensure(n, Math.ceil(vw / COARSE_VOXELS) * Math.ceil(vd / COARSE_VOXELS));
    const sx = Math.floor(x / VOXEL);
    const sy = Math.floor(y / VOXEL);
    if (floor < 0 || floor >= floors || sx < 0 || sy < 0 || sx >= vw || sy >= vd) return r;
    const base = floor * vw * vd;
    const epoch = this.nextEpoch();
    const { queue, parent, stamp } = this;
    const start = sy * vw + sx;
    let head = 0;
    let tail = 0;
    queue[tail++] = start;
    stamp[start] = epoch;
    parent[start] = start;
    let found = -1;
    let expansions = 0;
    // Level-synchronous BFS: among the frontier voxels at the smallest hop
    // count, take the one nearest the drone (stable, heading-free tie-break).
    let levelEnd = tail;
    search: while (head < tail) {
      let best = Infinity;
      for (let i = head; i < levelEnd; i++) {
        const k = queue[i]!;
        if (!seen[base + k] || visited[base + k]) continue;
        const kx = k % vw;
        const ex = (kx + 0.5) * VOXEL - x;
        const ey = ((k - kx) / vw + 0.5) * VOXEL - y;
        const d2 = ex * ex + ey * ey;
        if (d2 < best) {
          best = d2;
          found = k;
        }
      }
      if (found >= 0) break;
      for (; head < levelEnd; head++) {
        if (expansions >= BFS_MAX_EXPANSIONS) {
          r.truncated = true;
          break search;
        }
        const k = queue[head]!;
        expansions += 1;
        const kx = k % vw;
        const ky = (k - kx) / vw;
        for (let d = 0; d < 8; d++) {
          const dx = DIRS_X[d]!;
          const dy = DIRS_Y[d]!;
          const nx = kx + dx;
          const ny = ky + dy;
          if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
          const nk = ny * vw + nx;
          if (stamp[nk] === epoch) continue;
          if (!(seen[base + nk] || visited[base + nk])) continue;
          if (dx !== 0 && dy !== 0) {
            const a = base + ky * vw + nx;
            const b = base + ny * vw + kx;
            if (!(seen[a] || visited[a]) || !(seen[b] || visited[b])) continue;
          }
          stamp[nk] = epoch;
          parent[nk] = k;
          queue[tail++] = nk;
        }
      }
      levelEnd = tail;
    }
    r.expansions = expansions;
    if (found < 0) return r;
    // Walk back target -> start into the (now free) queue prefix, reversed.
    let len = 0;
    let cursor = found;
    const path = queue;
    for (;;) {
      path[len++] = cursor;
      if (cursor === start) break;
      cursor = parent[cursor]!;
    }
    // path[len-1] is start, path[0] is the target.
    let acc = 0;
    let px = (sx + 0.5) * VOXEL;
    let py = (sy + 0.5) * VOXEL;
    let wp = found;
    let reached = false;
    for (let i = len - 2; i >= 0; i--) {
      const k = path[i]!;
      const kx = k % vw;
      const cx = (kx + 0.5) * VOXEL;
      const cyy = ((k - kx) / vw + 0.5) * VOXEL;
      acc += Math.hypot(cx - px, cyy - py);
      px = cx;
      py = cyy;
      if (!reached && acc >= LOOKAHEAD) {
        wp = k;
        reached = true;
      }
    }
    const wx = wp % vw;
    r.wx = (wx + 0.5) * VOXEL;
    r.wy = ((wp - wx) / vw + 0.5) * VOXEL;
    const fx = found % vw;
    r.tx = (fx + 0.5) * VOXEL;
    r.ty = ((found - fx) / vw + 0.5) * VOXEL;
    r.pathMeters = acc;
    r.found = true;
    return r;
  }

  /**
   * Write the 12-float navigation block at OBS.nav. `stall` is steps since the
   * last new visited voxel.
   */
  write(
    out: Float32Array,
    visited: Uint8Array,
    seen: Uint8Array,
    vw: number,
    vd: number,
    floor: number,
    floors: number,
    x: number,
    y: number,
    yaw: number,
    stall: number,
  ): void {
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const r = this.search(visited, seen, vw, vd, floor, floors, x, y);
    if (r.found) {
      bodyUnit(out, OBS.wpDir, r.wx - x, r.wy - y, cy, sy);
      out[OBS.pathLen] = Math.min(1, r.pathMeters / PATH_NORM);
      out[OBS.hasFrontier] = 1;
      out[OBS.frontierDist] = Math.min(1, Math.hypot(r.tx - x, r.ty - y) / LIDAR_MAX);
    } else {
      out[OBS.wpDir] = 0;
      out[OBS.wpDir + 1] = 0;
      out[OBS.pathLen] = 1;
      out[OBS.hasFrontier] = 0;
      out[OBS.frontierDist] = 1;
    }
    out[OBS.bfsTruncated] = r.truncated ? 1 : 0;
    out[OBS.stall] = Math.min(1, Math.max(0, stall) / STALL_NORM);

    // Coarse clusters: 2 m cells over the whole floor, 8-connected components.
    out[OBS.clusterDir] = 0;
    out[OBS.clusterDir + 1] = 0;
    out[OBS.clusterDist] = 1;
    out[OBS.clusterMass] = 0;
    out[OBS.frontierTotal] = 0;
    if (floor < 0 || floor >= floors || vw === 0) return;
    const cw = Math.ceil(vw / COARSE_VOXELS);
    const cd = Math.ceil(vd / COARSE_VOXELS);
    const cn = cw * cd;
    const { ccount, csumX, csumY, cstamp, cstack } = this;
    ccount.fill(0, 0, cn);
    csumX.fill(0, 0, cn);
    csumY.fill(0, 0, cn);
    const base = floor * vw * vd;
    let total = 0;
    for (let vy = 0; vy < vd; vy++) {
      const row = base + vy * vw;
      const crow = ((vy / COARSE_VOXELS) | 0) * cw;
      for (let vx = 0; vx < vw; vx++) {
        if (!seen[row + vx] || visited[row + vx]) continue;
        const c = crow + ((vx / COARSE_VOXELS) | 0);
        ccount[c] += 1;
        csumX[c] += vx + 0.5;
        csumY[c] += vy + 0.5;
        total += 1;
      }
    }
    out[OBS.frontierTotal] = Math.min(1, total / FRONTIER_TOTAL_NORM);
    if (total === 0) return;
    const epoch = this.nextEpoch();
    let bestScore = -1;
    let bestX = 0;
    let bestY = 0;
    let bestMass = 0;
    for (let c0 = 0; c0 < cn; c0++) {
      if (ccount[c0] === 0 || cstamp[c0] === epoch) continue;
      let top = 0;
      cstack[top++] = c0;
      cstamp[c0] = epoch;
      let mass = 0;
      let mx = 0;
      let my = 0;
      while (top > 0) {
        const c = cstack[--top]!;
        mass += ccount[c]!;
        mx += csumX[c]!;
        my += csumY[c]!;
        const ccx = c % cw;
        const ccy = (c - ccx) / cw;
        for (let d = 0; d < 8; d++) {
          const nx = ccx + DIRS_X[d]!;
          const ny = ccy + DIRS_Y[d]!;
          if (nx < 0 || ny < 0 || nx >= cw || ny >= cd) continue;
          const nc = ny * cw + nx;
          if (ccount[nc] === 0 || cstamp[nc] === epoch) continue;
          cstamp[nc] = epoch;
          cstack[top++] = nc;
        }
      }
      const gx = (mx / mass) * VOXEL;
      const gy = (my / mass) * VOXEL;
      const dist = Math.hypot(gx - x, gy - y);
      const score = mass / (1 + dist / 8);
      if (score > bestScore) {
        bestScore = score;
        bestX = gx;
        bestY = gy;
        bestMass = mass;
      }
    }
    const dist = bodyUnit(out, OBS.clusterDir, bestX - x, bestY - y, cy, sy);
    out[OBS.clusterDist] = Math.min(1, dist / CLUSTER_NORM);
    out[OBS.clusterMass] = Math.min(1, bestMass / CLUSTER_MASS_NORM);
  }
}

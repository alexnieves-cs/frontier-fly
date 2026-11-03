import { LikelihoodGrid } from './grid';
import { DEFAULT_MATCH, eigSym3, endpoints, invert3, matchScan, searchScan, type MatchOptions } from './match';

/**
 * Loop closure for the pose estimator: a pose-graph back end (graph SLAM,
 * Lu & Milios 1997; Grisetti et al., "A tutorial on graph-based SLAM", 2010).
 *
 * The estimator (or plain odometry) is the front end. Every `keyDist` metres
 * or `keyYaw` radians of its motion, the sweep becomes a keyframe, a node of
 * the graph. Two kinds of edge join them:
 *
 * - sequential: the new keyframe's sweep matched against a submap of the
 *   last few keyframes, starting from the front end's own motion and keeping
 *   it as a prior, so a bare corridor the sweep can't pin stays where the
 *   front end put it;
 * - loop: the new keyframe's sweep found, by a correlative search and then a
 *   refinement, in a submap of keyframes passed minutes earlier near the
 *   same spot. Only well-constrained matches with most returns on the old
 *   walls are kept, and they enter the optimisation under a Cauchy kernel,
 *   so a wrong one the checks let through pulls weakly.
 *
 * The graph is optimised by Gauss-Newton after each closure (so the next
 * search starts from corrected poses) and once more at the end. The linear
 * system is solved by block Gaussian elimination in greedy minimum-degree
 * order, which keeps the fill of a pose graph small. Every sweep's pose is
 * then its keyframe's optimised pose composed with the front end's motion
 * since that keyframe.
 */

export type Pose2 = [number, number, number];

export function wrapAngle(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

/** a ⊕ b: pose b, given in a's frame, in the world frame. */
export function compose(a: ArrayLike<number>, b: ArrayLike<number>): Pose2 {
  const c = Math.cos(a[2]!);
  const s = Math.sin(a[2]!);
  return [a[0]! + c * b[0]! - s * b[1]!, a[1]! + s * b[0]! + c * b[1]!, wrapAngle(a[2]! + b[2]!)];
}

/** a⁻¹ ⊕ b: pose b in a's frame. */
export function between(a: ArrayLike<number>, b: ArrayLike<number>): Pose2 {
  const c = Math.cos(a[2]!);
  const s = Math.sin(a[2]!);
  const dx = b[0]! - a[0]!;
  const dy = b[1]! - a[1]!;
  return [c * dx + s * dy, -s * dx + c * dy, wrapAngle(b[2]! - a[2]!)];
}

export interface GraphEdge {
  i: number;
  j: number;
  /** Measured pose of node j in node i's frame. */
  z: Pose2;
  /** Row-major 3x3 information of the measurement (x, y, yaw). */
  info: Float64Array;
  kind: 'seq' | 'loop';
  /** Cauchy weight after the last optimisation (1 for sequential edges). */
  weight: number;
}

/**
 * SE(2) pose graph. Node 0 is held fixed. Loop edges are robust (Cauchy, scale
 * `cauchy` in units of the chi-square of the edge); sequential edges are not.
 */
export class PoseGraph {
  readonly poses: number[] = [];
  readonly edges: GraphEdge[] = [];

  constructor(readonly cauchy = 1) {}

  get n(): number {
    return this.poses.length / 3;
  }

  addNode(p: ArrayLike<number>): number {
    this.poses.push(p[0]!, p[1]!, p[2]!);
    return this.n - 1;
  }

  pose(i: number): Pose2 {
    return [this.poses[i * 3]!, this.poses[i * 3 + 1]!, this.poses[i * 3 + 2]!];
  }

  addEdge(i: number, j: number, z: Pose2, info: Float64Array, kind: GraphEdge['kind']): GraphEdge {
    const e: GraphEdge = { i, j, z, info: Float64Array.from(info), kind, weight: 1 };
    this.edges.push(e);
    return e;
  }

  /** Error of an edge at the current poses: t2v(z⁻¹ (x_i⁻¹ x_j)), written to out. */
  private residual(e: GraphEdge, out: Float64Array): void {
    const d = between(this.pose(e.i), this.pose(e.j));
    const r = between(e.z, d);
    out[0] = r[0];
    out[1] = r[1];
    out[2] = r[2];
  }

  /** Sum over edges of eᵀ Ω e (unweighted). */
  chi2(): number {
    const r = new Float64Array(3);
    let total = 0;
    for (const e of this.edges) {
      this.residual(e, r);
      total += quad(e.info, r);
    }
    return total;
  }

  /**
   * Gauss-Newton with Cauchy reweighting of loop edges. Stops early when a
   * step moves no pose more than 1 mm and 0.01°. Returns the iterations run.
   */
  optimize(iterations = 5): number {
    const n = this.n;
    if (n < 2) return 0;
    const r = new Float64Array(3);
    const A = new Float64Array(9);
    const B = new Float64Array(9);
    const OA = new Float64Array(9);
    const OB = new Float64Array(9);
    const tmp = new Float64Array(9);
    let it = 0;
    for (; it < iterations; it++) {
      const sys = new BlockSystem(n);
      for (const e of this.edges) {
        const xi = this.poses[e.i * 3]!;
        const yi = this.poses[e.i * 3 + 1]!;
        const ti = this.poses[e.i * 3 + 2]!;
        const xj = this.poses[e.j * 3]!;
        const yj = this.poses[e.j * 3 + 1]!;
        this.residual(e, r);
        const chi = quad(e.info, r);
        const w = e.kind === 'loop' ? 1 / (1 + chi / (this.cauchy * this.cauchy)) : 1;
        e.weight = w;
        // Jacobians of e = [Rzᵀ(Riᵀ(tj - ti) - tz); θj - θi - θz] (Grisetti et al. 2010, eq. 31-33).
        const ci = Math.cos(ti);
        const si = Math.sin(ti);
        const cz = Math.cos(e.z[2]);
        const sz = Math.sin(e.z[2]);
        const dx = xj - xi;
        const dy = yj - yi;
        // RzᵀRiᵀ = R(-(θi + θz)).
        const c = Math.cos(ti + e.z[2]);
        const s = Math.sin(ti + e.z[2]);
        // d(Riᵀ)/dθ (tj - ti) = [-si dx + ci dy; -ci dx - si dy], then Rzᵀ.
        const gx = -si * dx + ci * dy;
        const gy = -ci * dx - si * dy;
        const a02 = cz * gx + sz * gy;
        const a12 = -sz * gx + cz * gy;
        A.set([-c, -s, a02, s, -c, a12, 0, 0, -1]);
        B.set([c, s, 0, -s, c, 0, 0, 0, 1]);
        // OA = Ω A, OB = Ω B (weighted).
        mul3(e.info, A, OA);
        mul3(e.info, B, OB);
        for (let k = 0; k < 9; k++) {
          OA[k]! *= w;
          OB[k]! *= w;
        }
        // H_ii += Aᵀ Ω A, H_ij += Aᵀ Ω B, H_jj += Bᵀ Ω B; b_i += Aᵀ Ω e, b_j += Bᵀ Ω e.
        mulT3(A, OA, tmp);
        sys.addDiag(e.i, tmp);
        mulT3(B, OB, tmp);
        sys.addDiag(e.j, tmp);
        mulT3(A, OB, tmp);
        sys.addOff(e.i, e.j, tmp);
        sys.addRhs(e.i, OA, r);
        sys.addRhs(e.j, OB, r);
      }
      // Hold node 0.
      sys.diag[0] += 1e12;
      sys.diag[4] += 1e12;
      sys.diag[8] += 1e12;
      const dxs = sys.solve();
      let maxStep = 0;
      let maxRot = 0;
      for (let k = 0; k < n; k++) {
        // H Δ = -b
        const ddx = -dxs[k * 3]!;
        const ddy = -dxs[k * 3 + 1]!;
        const ddt = -dxs[k * 3 + 2]!;
        if (!Number.isFinite(ddx + ddy + ddt)) continue;
        this.poses[k * 3] += ddx;
        this.poses[k * 3 + 1] += ddy;
        this.poses[k * 3 + 2] = wrapAngle(this.poses[k * 3 + 2]! + ddt);
        maxStep = Math.max(maxStep, Math.hypot(ddx, ddy));
        maxRot = Math.max(maxRot, Math.abs(ddt));
      }
      if (maxStep < 1e-3 && maxRot < 1.7e-4) {
        it += 1;
        break;
      }
    }
    return it;
  }
}

function quad(m: ArrayLike<number>, r: ArrayLike<number>): number {
  let s = 0;
  for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) s += r[a]! * m[a * 3 + b]! * r[b]!;
  return s;
}

/** out = a b (3x3 row-major). */
function mul3(a: ArrayLike<number>, b: ArrayLike<number>, out: Float64Array): void {
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) out[i * 3 + j] = a[i * 3]! * b[j]! + a[i * 3 + 1]! * b[3 + j]! + a[i * 3 + 2]! * b[6 + j]!;
}

/** out = aᵀ b. */
function mulT3(a: ArrayLike<number>, b: ArrayLike<number>, out: Float64Array): void {
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) out[i * 3 + j] = a[i]! * b[j]! + a[3 + i]! * b[3 + j]! + a[6 + i]! * b[6 + j]!;
}

/**
 * Symmetric block-sparse system H x = b over n nodes of 3 unknowns each.
 * Solved by block Gaussian elimination in greedy minimum-degree order.
 */
class BlockSystem {
  readonly diag: Float64Array;
  readonly rhs: Float64Array;
  /** off[v].get(a) = H_va (rows v, columns a). Both directions are stored. */
  private readonly off: Map<number, Float64Array>[];

  constructor(readonly n: number) {
    this.diag = new Float64Array(n * 9);
    this.rhs = new Float64Array(n * 3);
    this.off = Array.from({ length: n }, () => new Map());
  }

  addDiag(i: number, m: ArrayLike<number>): void {
    for (let k = 0; k < 9; k++) this.diag[i * 9 + k] += m[k]!;
  }

  /** H_ij += m, H_ji += mᵀ. */
  addOff(i: number, j: number, m: ArrayLike<number>): void {
    if (i === j) {
      for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) this.diag[i * 9 + a * 3 + b] += m[a * 3 + b]! + m[b * 3 + a]!;
      return;
    }
    let ij = this.off[i]!.get(j);
    if (!ij) {
      ij = new Float64Array(9);
      this.off[i]!.set(j, ij);
      this.off[j]!.set(i, new Float64Array(9));
    }
    const ji = this.off[j]!.get(i)!;
    for (let a = 0; a < 3; a++)
      for (let b = 0; b < 3; b++) {
        ij[a * 3 + b] += m[a * 3 + b]!;
        ji[b * 3 + a] += m[a * 3 + b]!;
      }
  }

  /** b_i += Mᵀ r (M = Ω J, so this is Jᵀ Ω r). */
  addRhs(i: number, M: ArrayLike<number>, r: ArrayLike<number>): void {
    for (let a = 0; a < 3; a++) this.rhs[i * 3 + a] += M[a]! * r[0]! + M[3 + a]! * r[1]! + M[6 + a]! * r[2]!;
  }

  solve(): Float64Array {
    const n = this.n;
    const done = new Uint8Array(n);
    const order: number[] = [];
    const saved: { dinv: Float64Array; rhs: Float64Array; nb: [number, Float64Array][] }[] = new Array(n);
    const dinv = new Float64Array(9);
    const M = new Map<number, Float64Array>();
    const tmp = new Float64Array(9);
    // Greedy minimum degree, from a heap with lazy deletion (an entry is stale when its degree no longer matches).
    const heap = new MinHeap();
    for (let k = 0; k < n; k++) heap.push(this.off[k]!.size, k);
    for (let step = 0; step < n; step++) {
      let v = -1;
      while (heap.size) {
        const [d, k] = heap.pop();
        if (!done[k] && d === this.off[k]!.size) {
          v = k;
          break;
        }
      }
      done[v] = 1;
      order.push(v);
      const D = this.diag.subarray(v * 9, v * 9 + 9);
      if (!invert3(D, dinv)) dinv.fill(0);
      const bv = this.rhs.subarray(v * 3, v * 3 + 3);
      const nb = [...this.off[v]!.entries()];
      saved[v] = { dinv: Float64Array.from(dinv), rhs: Float64Array.from(bv), nb };
      M.clear();
      for (const [a, Hva] of nb) {
        const m = new Float64Array(9);
        mul3(dinv, Hva, m);
        M.set(a, m);
        this.off[a]!.delete(v);
      }
      for (const [a, Hva] of nb) {
        const Ma = M.get(a)!;
        // b_a -= Maᵀ b_v
        for (let r = 0; r < 3; r++) this.rhs[a * 3 + r] -= Ma[r]! * bv[0]! + Ma[3 + r]! * bv[1]! + Ma[6 + r]! * bv[2]!;
        for (const [b, Hvb] of nb) {
          if (b < a) continue;
          // H_ab -= H_vaᵀ Dinv H_vb = H_vaᵀ M_b
          mulT3(Hva, M.get(b)!, tmp);
          if (a === b) {
            for (let k = 0; k < 9; k++) this.diag[a * 9 + k] -= tmp[k]!;
          } else {
            for (let k = 0; k < 9; k++) tmp[k] = -tmp[k]!;
            this.addOff(a, b, tmp);
          }
          void Hvb;
        }
      }
      for (const [a] of nb) heap.push(this.off[a]!.size, a);
    }
    const x = new Float64Array(n * 3);
    const acc = new Float64Array(3);
    for (let s = n - 1; s >= 0; s--) {
      const v = order[s]!;
      const { dinv: Di, rhs, nb } = saved[v]!;
      acc[0] = rhs[0]!;
      acc[1] = rhs[1]!;
      acc[2] = rhs[2]!;
      for (const [a, Hva] of nb) {
        const xa = a * 3;
        for (let r = 0; r < 3; r++) acc[r] -= Hva[r * 3]! * x[xa]! + Hva[r * 3 + 1]! * x[xa + 1]! + Hva[r * 3 + 2]! * x[xa + 2]!;
      }
      for (let r = 0; r < 3; r++) x[v * 3 + r] = Di[r * 3]! * acc[0]! + Di[r * 3 + 1]! * acc[1]! + Di[r * 3 + 2]! * acc[2]!;
    }
    return x;
  }
}

/** Binary min-heap of (key, value) pairs. */
class MinHeap {
  private readonly keys: number[] = [];
  private readonly vals: number[] = [];

  get size(): number {
    return this.keys.length;
  }

  push(key: number, val: number): void {
    const { keys, vals } = this;
    let i = keys.length;
    keys.push(key);
    vals.push(val);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (keys[p]! <= key) break;
      keys[i] = keys[p]!;
      vals[i] = vals[p]!;
      i = p;
    }
    keys[i] = key;
    vals[i] = val;
  }

  pop(): [number, number] {
    const { keys, vals } = this;
    const top: [number, number] = [keys[0]!, vals[0]!];
    const lk = keys.pop()!;
    const lv = vals.pop()!;
    const n = keys.length;
    if (n) {
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        if (l >= n) break;
        const r = l + 1;
        const c = r < n && keys[r]! < keys[l]! ? r : l;
        if (keys[c]! >= lk) break;
        keys[i] = keys[c]!;
        vals[i] = vals[c]!;
        i = c;
      }
      keys[i] = lk;
      vals[i] = lv;
    }
    return top;
  }
}

export interface LoopClosureOptions {
  /** A sweep becomes a keyframe after this much front-end motion since the last one, m / rad. */
  keyDist: number;
  keyYaw: number;
  /** Lidar range used for submaps and matching, m (returns at or beyond are misses). */
  maxRange: number;
  /** Submap cell size, m. */
  res: number;
  /** Sub-cell shift of the submap lattice, in cells (0..1 each). */
  gridShift: [number, number];
  /** Keyframes in the sequential submap (the last ones before the new keyframe). */
  seqWindow: number;
  /** Correlative search half-width before the sequential match, m and rad (0: Gauss-Newton from the front end's guess only). */
  seqSearchXY: number;
  seqSearchYaw: number;
  /** Keyframes on each side of a loop candidate in its submap. */
  loopHalfWindow: number;
  /** A loop candidate is at least this many keyframes older than the new one. */
  loopMinGap: number;
  /** Search half-width around the graph's guess, m: base, plus growth per metre flown since the last closure, capped. */
  loopSearchBase: number;
  loopSearchGrowth: number;
  loopSearchMax: number;
  /** Search half-width in heading, rad. */
  loopSearchYaw: number;
  /** Try loop closure every this many keyframes, at most this many candidates each time. */
  loopEvery: number;
  loopCandidates: number;
  /** Accept a loop when this share of the sweep's returns land on the old walls after refinement. */
  loopMinInliers: number;
  /** ...and the match pins translation at least this well (smallest information eigenvalue of x, y, in 1/m²). */
  loopMinInfo: number;
  /** Scan noise the match information is scaled by, m (see MatchOptions.sigma). */
  sigma: number;
  /** Front-end motion noise between keyframes: sigma = a + b * distance (m), and yaw a + b * turn (rad). */
  odomSigmaXY: [number, number];
  odomSigmaYaw: [number, number];
  /**
   * Floor on a loop edge's standard deviation, m and rad: its information is scaled down (keeping its shape)
   * until neither the translation nor the heading claims better. 0: the match's own information.
   */
  loopMinSigmaXY: number;
  loopMinSigmaYaw: number;
  /** Cauchy kernel scale for loop edges, in chi-square units. */
  cauchy: number;
  /** Gauss-Newton iterations after each closure and at the end. */
  itersOnline: number;
  itersFinal: number;
}

export function defaultLoopClosureOptions(): LoopClosureOptions {
  return {
    keyDist: 0.25,
    keyYaw: 0.15,
    maxRange: 8,
    res: 0.05,
    gridShift: [0, 0],
    seqWindow: 30,
    seqSearchXY: 0.2,
    seqSearchYaw: 0.1,
    loopHalfWindow: 10,
    loopMinGap: 40,
    loopSearchBase: 0.8,
    loopSearchGrowth: 0.02,
    loopSearchMax: 2.5,
    loopSearchYaw: 0.3,
    loopEvery: 2,
    loopCandidates: 2,
    loopMinInliers: 0.65,
    loopMinInfo: 400,
    sigma: 0.1,
    odomSigmaXY: [0.02, 0.05],
    odomSigmaYaw: [0.01, 0.05],
    loopMinSigmaXY: 0,
    loopMinSigmaYaw: 0,
    cauchy: 3,
    itersOnline: 4,
    itersFinal: 15,
  };
}

export interface LoopClosureStats {
  keyframes: number;
  seqMatched: number;
  seqFallback: number;
  loopTried: number;
  loopAccepted: number;
  /** Loop edges left with a Cauchy weight under 0.1 after the final optimisation (judged wrong). */
  loopDownweighted: number;
  optimizations: number;
}

interface Keyframe {
  sweep: number;
  /** Front-end pose at the keyframe. */
  front: Pose2;
  ranges: Float64Array;
  pts: { bx: Float64Array; by: Float64Array; n: number };
}

/**
 * Online loop closure over a front end's trajectory. Feed every sweep in order
 * with `add`, then call `finish` for the optimised pose of every sweep.
 */
export class LoopCloser {
  readonly opts: LoopClosureOptions;
  readonly graph: PoseGraph;
  readonly stats: LoopClosureStats = { keyframes: 0, seqMatched: 0, seqFallback: 0, loopTried: 0, loopAccepted: 0, loopDownweighted: 0, optimizations: 0 };
  private readonly keys: Keyframe[] = [];
  /** Per sweep: keyframe index at or before it, and the front-end pose. */
  private readonly sweepKey: number[] = [];
  private readonly sweepFront: number[] = [];
  private pathSinceLoop = 0;
  private pendingLoops = 0;
  private readonly match: MatchOptions;

  constructor(opts: Partial<LoopClosureOptions> = {}) {
    this.opts = { ...defaultLoopClosureOptions(), ...opts };
    this.graph = new PoseGraph(this.opts.cauchy);
    this.match = { ...DEFAULT_MATCH, sigma: this.opts.sigma };
  }

  /** One sweep: the front end's pose and the lidar ring (beam i at yaw + i 2π / n). */
  add(front: ArrayLike<number>, ranges: ArrayLike<number>): void {
    const f: Pose2 = [front[0]!, front[1]!, front[2]!];
    const sweep = this.sweepKey.length;
    this.sweepFront.push(f[0], f[1], f[2]);
    const last = this.keys[this.keys.length - 1];
    if (last) {
      const d = between(last.front, f);
      if (Math.hypot(d[0], d[1]) < this.opts.keyDist && Math.abs(d[2]) < this.opts.keyYaw) {
        this.sweepKey.push(this.keys.length - 1);
        return;
      }
    }
    const r = Float64Array.from(ranges);
    const key: Keyframe = { sweep, front: f, ranges: r, pts: endpoints(r, this.opts.maxRange) };
    this.keys.push(key);
    this.sweepKey.push(this.keys.length - 1);
    this.addKeyframe(key);
  }

  private addKeyframe(key: Keyframe): void {
    const o = this.opts;
    const g = this.graph;
    const k = this.keys.length - 1;
    this.stats.keyframes += 1;
    if (k === 0) {
      g.addNode(key.front);
      return;
    }
    const prev = this.keys[k - 1]!;
    const odo = between(prev.front, key.front);
    const dist = Math.hypot(odo[0], odo[1]);
    this.pathSinceLoop += dist;
    const sxy = o.odomSigmaXY[0] + o.odomSigmaXY[1] * dist;
    const syaw = o.odomSigmaYaw[0] + o.odomSigmaYaw[1] * Math.abs(odo[2]);
    const odoInfo = new Float64Array([1 / (sxy * sxy), 0, 0, 0, 1 / (sxy * sxy), 0, 0, 0, 1 / (syaw * syaw)]);
    const guess = compose(g.pose(k - 1), odo);
    g.addNode(guess);
    // Sequential: match against the last keyframes, with the front end's motion as the prior (rotated into the world frame).
    const from = Math.max(0, k - o.seqWindow);
    const grid = this.submap(from, k - 1);
    const c = Math.cos(g.pose(k - 1)[2]);
    const s = Math.sin(g.pose(k - 1)[2]);
    const R = new Float64Array([c, -s, 0, s, c, 0, 0, 0, 1]);
    const prior = new Float64Array(9);
    const tmp = new Float64Array(9);
    mul3(R, odoInfo, tmp);
    // prior = R Ω Rᵀ
    for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) prior[i * 3 + j] = tmp[i * 3] * R[j * 3]! + tmp[i * 3 + 1]! * R[j * 3 + 1]! + tmp[i * 3 + 2]! * R[j * 3 + 2]!;
    let m = null;
    if (grid && key.pts.n > 20) {
      // A short correlative search first: wheel odometry can be degrees off after a turn, beyond what Gauss-Newton pulls in.
      const sr = o.seqSearchXY > 0 ? searchScan(grid, key.pts, guess[0], guess[1], guess[2], o.seqSearchXY, o.seqSearchYaw, this.match) : { x: guess[0], y: guess[1], yaw: guess[2] };
      m = matchScan(grid, key.pts, sr.x, sr.y, sr.yaw, prior, this.match);
    }
    if (m && m.valid > 20 && m.inliers / m.valid > 0.5) {
      const z = between(g.pose(k - 1), [m.x, m.y, m.yaw]);
      // Information of the edge: the scan's (rotated into node k-1's frame) plus the front end's.
      const info = rotInfo(m.info, -m.yaw);
      for (let q = 0; q < 9; q++) info[q] += odoInfo[q]!;
      g.addEdge(k - 1, k, z, info, 'seq');
      g.poses[k * 3] = m.x;
      g.poses[k * 3 + 1] = m.y;
      g.poses[k * 3 + 2] = m.yaw;
      this.stats.seqMatched += 1;
    } else {
      g.addEdge(k - 1, k, odo, odoInfo, 'seq');
      this.stats.seqFallback += 1;
    }
    if (k % o.loopEvery === 0) this.tryLoops(k);
    if (this.pendingLoops > 0) {
      g.optimize(o.itersOnline);
      this.stats.optimizations += 1;
      this.pendingLoops = 0;
    }
  }

  private tryLoops(k: number): void {
    const o = this.opts;
    const g = this.graph;
    const here = g.pose(k);
    const half = Math.min(o.loopSearchMax, o.loopSearchBase + o.loopSearchGrowth * this.pathSinceLoop);
    const radius = half + 1;
    // Nearest old keyframes, at least loopHalfWindow apart from each other.
    const cands: { j: number; d: number }[] = [];
    for (let j = 0; j <= k - o.loopMinGap; j++) {
      const p = g.pose(j);
      const d = Math.hypot(p[0] - here[0], p[1] - here[1]);
      if (d < radius) cands.push({ j, d });
    }
    cands.sort((a, b) => a.d - b.d);
    const picked: number[] = [];
    for (const c of cands) {
      if (picked.length >= o.loopCandidates) break;
      if (picked.some((p) => Math.abs(p - c.j) < o.loopHalfWindow)) continue;
      picked.push(c.j);
    }
    for (const j of picked) {
      this.stats.loopTried += 1;
      const lo = Math.max(0, j - o.loopHalfWindow);
      const hi = Math.min(k - o.loopMinGap + o.loopHalfWindow, j + o.loopHalfWindow);
      const grid = this.submap(lo, hi);
      if (!grid) continue;
      const pts = this.keys[k]!.pts;
      if (pts.n < 30) continue;
      const sr = searchScan(grid, pts, here[0], here[1], here[2], half, o.loopSearchYaw, this.match);
      if (sr.score < o.loopMinInliers * 0.8) continue;
      const m = matchScan(grid, pts, sr.x, sr.y, sr.yaw, null, this.match);
      if (m.valid === 0 || m.inliers / m.valid < o.loopMinInliers) continue;
      // Reject a match that a corridor leaves free to slide.
      const vals = new Float64Array(3);
      const vecs = new Float64Array(9);
      const txy = new Float64Array([m.info[0]!, m.info[1]!, 0, m.info[3]!, m.info[4]!, 0, 0, 0, 1e9]);
      eigSym3(txy, vals, vecs);
      if (Math.min(vals[0]!, vals[1]!, vals[2]!) < o.loopMinInfo) continue;
      // The edge: pose k in node j's frame, through the submap (built in world coordinates at the graph's poses).
      const z = between(g.pose(j), [m.x, m.y, m.yaw]);
      g.addEdge(j, k, z, capInfo(rotInfo(m.info, -m.yaw), o.loopMinSigmaXY, o.loopMinSigmaYaw), 'loop');
      this.stats.loopAccepted += 1;
      this.pendingLoops += 1;
      this.pathSinceLoop = 0;
    }
  }

  /** Occupancy submap of keyframes lo..hi (inclusive) at their current graph poses. */
  private submap(lo: number, hi: number): LikelihoodGrid | null {
    if (hi < lo) return null;
    const g = this.graph;
    const R = this.opts.maxRange + 0.5;
    let x0 = Infinity;
    let y0 = Infinity;
    let x1 = -Infinity;
    let y1 = -Infinity;
    for (let q = lo; q <= hi; q++) {
      const p = g.pose(q);
      x0 = Math.min(x0, p[0]);
      y0 = Math.min(y0, p[1]);
      x1 = Math.max(x1, p[0]);
      y1 = Math.max(y1, p[1]);
    }
    // Submap corners sit on one lattice, shifted by gridShift cells (the study's sensitivity check).
    const res = this.opts.res;
    const ox = (Math.floor((x0 - R) / res) - this.opts.gridShift[0]) * res;
    const oy = (Math.floor((y0 - R) / res) - this.opts.gridShift[1]) * res;
    const grid = new LikelihoodGrid(ox, oy, x1 + R - ox + res, y1 + R - oy + res, res, 1);
    for (let q = lo; q <= hi; q++) {
      const p = g.pose(q);
      grid.integrate(p[0], p[1], p[2], this.keys[q]!.ranges, this.opts.maxRange);
    }
    grid.refresh();
    return grid;
  }

  /** Final optimisation; every sweep's pose (x, y, yaw per sweep). */
  finish(): Float64Array {
    const g = this.graph;
    g.optimize(this.opts.itersFinal);
    this.stats.optimizations += 1;
    this.stats.loopDownweighted = g.edges.filter((e) => e.kind === 'loop' && e.weight < 0.1).length;
    const n = this.sweepKey.length;
    const out = new Float64Array(n * 3);
    for (let i = 0; i < n; i++) {
      const k = this.sweepKey[i]!;
      const rel = between(this.keys[k]!.front, [this.sweepFront[i * 3]!, this.sweepFront[i * 3 + 1]!, this.sweepFront[i * 3 + 2]!]);
      const p = compose(g.pose(k), rel);
      out.set(p, i * 3);
    }
    return out;
  }
}

/**
 * World-frame information of a matched pose re-expressed for an edge error, whose translation is in the
 * matched pose's own frame: R(a) Ω R(a)ᵀ with a = -yaw of that pose (yaw row untouched).
 */
function rotInfo(info: ArrayLike<number>, a: number): Float64Array {
  const c = Math.cos(a);
  const s = Math.sin(a);
  const R = [c, -s, 0, s, c, 0, 0, 0, 1];
  const tmp = new Float64Array(9);
  mul3(R, info, tmp);
  const out = new Float64Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) out[i * 3 + j] = tmp[i * 3]! * R[j * 3]! + tmp[i * 3 + 1]! * R[j * 3 + 1]! + tmp[i * 3 + 2]! * R[j * 3 + 2]!;
  return out;
}

/** Information scaled down uniformly so that no direction is surer than sigmaXY (x, y) or sigmaYaw. */
function capInfo(info: Float64Array, sigmaXY: number, sigmaYaw: number): Float64Array {
  let scale = 1;
  if (sigmaXY > 0) {
    const a = info[0]!;
    const b = info[1]!;
    const d = info[4]!;
    const lMax = (a + d) / 2 + Math.sqrt(((a - d) / 2) ** 2 + b * b);
    scale = Math.min(scale, 1 / (sigmaXY * sigmaXY * lMax));
  }
  if (sigmaYaw > 0) scale = Math.min(scale, 1 / (sigmaYaw * sigmaYaw * info[8]!));
  if (!(scale < 1)) return info;
  for (let k = 0; k < 9; k++) info[k]! *= scale;
  return info;
}

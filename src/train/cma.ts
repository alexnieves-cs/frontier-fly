/**
 * CMA-ES (Hansen's (μ/μ_w, λ) evolution strategy with rank-one and rank-μ covariance updates),
 * minimizing. Small and dependency-free: the goal-picker study (scripts/train-picker.ts) searches
 * 3 to 20 parameters with it, where episode returns are too noisy for finite differences.
 */
import { RNG } from '../shared/rng';

export interface CmaState {
  n: number;
  mean: number[];
  sigma: number;
  C: number[][];
  pc: number[];
  ps: number[];
  gen: number;
}

export class Cma {
  readonly n: number;
  readonly lambda: number;
  readonly mu: number;
  private readonly w: number[];
  private readonly mueff: number;
  private readonly cc: number;
  private readonly cs: number;
  private readonly c1: number;
  private readonly cmu: number;
  private readonly damps: number;
  private readonly chiN: number;
  mean: number[];
  sigma: number;
  C: number[][];
  pc: number[];
  ps: number[];
  gen = 0;
  private B: number[][] = [];
  private D: number[] = [];
  private rng: RNG;
  private gauss: number | null = null;

  constructor(mean: number[], sigma: number, opts: { lambda?: number; seed?: number } = {}) {
    const n = mean.length;
    this.n = n;
    this.lambda = opts.lambda ?? 4 + Math.floor(3 * Math.log(n));
    this.mu = Math.floor(this.lambda / 2);
    const raw = Array.from({ length: this.mu }, (_, i) => Math.log(this.mu + 0.5) - Math.log(i + 1));
    const sum = raw.reduce((a, b) => a + b, 0);
    this.w = raw.map((x) => x / sum);
    this.mueff = 1 / this.w.reduce((a, b) => a + b * b, 0);
    const me = this.mueff;
    this.cc = (4 + me / n) / (n + 4 + (2 * me) / n);
    this.cs = (me + 2) / (n + me + 5);
    this.c1 = 2 / ((n + 1.3) ** 2 + me);
    this.cmu = Math.min(1 - this.c1, (2 * (me - 2 + 1 / me)) / ((n + 2) ** 2 + me));
    this.damps = 1 + 2 * Math.max(0, Math.sqrt((me - 1) / (n + 1)) - 1) + this.cs;
    this.chiN = Math.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n));
    this.mean = [...mean];
    this.sigma = sigma;
    this.C = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
    this.pc = new Array(n).fill(0);
    this.ps = new Array(n).fill(0);
    this.rng = new RNG(opts.seed ?? 1, 'cma');
    this.decompose();
  }

  private normal(): number {
    if (this.gauss !== null) {
      const g = this.gauss;
      this.gauss = null;
      return g;
    }
    let u = 0;
    while (u <= 1e-12) u = this.rng.next();
    const v = this.rng.next();
    const r = Math.sqrt(-2 * Math.log(u));
    this.gauss = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v);
  }

  private decompose(): void {
    const { vectors, values } = jacobiEigen(this.C);
    this.B = vectors;
    this.D = values.map((v) => Math.sqrt(Math.max(v, 1e-20)));
  }

  /** λ new points. */
  ask(): number[][] {
    const n = this.n;
    const out: number[][] = [];
    for (let k = 0; k < this.lambda; k++) {
      const z = Array.from({ length: n }, () => this.normal());
      const x = new Array(n);
      for (let i = 0; i < n; i++) {
        let y = 0;
        for (let j = 0; j < n; j++) y += this.B[i]![j]! * this.D[j]! * z[j]!;
        x[i] = this.mean[i]! + this.sigma * y;
      }
      out.push(x);
    }
    return out;
  }

  /** Update from the points ask() returned and their costs (lower is better). */
  tell(xs: number[][], costs: number[]): void {
    const n = this.n;
    const order = costs.map((c, i) => ({ c, i })).sort((a, b) => a.c - b.c).map((o) => o.i);
    const old = this.mean;
    const mean = new Array(n).fill(0);
    for (let k = 0; k < this.mu; k++) for (let i = 0; i < n; i++) mean[i] += this.w[k]! * xs[order[k]!]![i]!;
    const yw = mean.map((m, i) => (m - old[i]!) / this.sigma);
    // C^{-1/2} yw = B D^{-1} B^T yw
    const bt = new Array(n).fill(0);
    for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) bt[j] += this.B[i]![j]! * yw[i]!;
    const inv = new Array(n).fill(0);
    for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) inv[i] += (this.B[i]![j]! * bt[j]!) / this.D[j]!;
    const a = Math.sqrt(this.cs * (2 - this.cs) * this.mueff);
    for (let i = 0; i < n; i++) this.ps[i] = (1 - this.cs) * this.ps[i]! + a * inv[i]!;
    const psn = Math.hypot(...this.ps);
    this.gen += 1;
    const hsig = psn / Math.sqrt(1 - (1 - this.cs) ** (2 * this.gen)) / this.chiN < 1.4 + 2 / (n + 1) ? 1 : 0;
    const b = Math.sqrt(this.cc * (2 - this.cc) * this.mueff);
    for (let i = 0; i < n; i++) this.pc[i] = (1 - this.cc) * this.pc[i]! + hsig * b * yw[i]!;
    const ys = order.slice(0, this.mu).map((k) => xs[k]!.map((x, i) => (x - old[i]!) / this.sigma));
    const keep = 1 - this.c1 - this.cmu;
    for (let i = 0; i < n; i++) {
      for (let j = 0; j < n; j++) {
        let rmu = 0;
        for (let k = 0; k < this.mu; k++) rmu += this.w[k]! * ys[k]![i]! * ys[k]![j]!;
        this.C[i]![j] =
          keep * this.C[i]![j]! +
          this.c1 * (this.pc[i]! * this.pc[j]! + (1 - hsig) * this.cc * (2 - this.cc) * this.C[i]![j]!) +
          this.cmu * rmu;
      }
    }
    this.sigma *= Math.exp((this.cs / this.damps) * (psn / this.chiN - 1));
    this.mean = mean;
    this.decompose();
  }

  state(): CmaState {
    return { n: this.n, mean: [...this.mean], sigma: this.sigma, C: this.C.map((r) => [...r]), pc: [...this.pc], ps: [...this.ps], gen: this.gen };
  }
}

/** Eigen-decomposition of a symmetric matrix by cyclic Jacobi rotations. vectors[i][j] = component i of eigenvector j. */
export function jacobiEigen(M: number[][]): { values: number[]; vectors: number[][] } {
  const n = M.length;
  const a = M.map((r) => [...r]);
  const v: number[][] = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) off += a[i]![j]! ** 2;
    if (off < 1e-22) break;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        const apq = a[p]![q]!;
        if (Math.abs(apq) < 1e-300) continue;
        const theta = (a[q]![q]! - a[p]![p]!) / (2 * apq);
        const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;
        for (let k = 0; k < n; k++) {
          const akp = a[k]![p]!;
          const akq = a[k]![q]!;
          a[k]![p] = c * akp - s * akq;
          a[k]![q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = a[p]![k]!;
          const aqk = a[q]![k]!;
          a[p]![k] = c * apk - s * aqk;
          a[q]![k] = s * apk + c * aqk;
        }
        for (let k = 0; k < n; k++) {
          const vkp = v[k]![p]!;
          const vkq = v[k]![q]!;
          v[k]![p] = c * vkp - s * vkq;
          v[k]![q] = s * vkp + c * vkq;
        }
      }
    }
  }
  return { values: a.map((r, i) => r[i]!), vectors: v };
}

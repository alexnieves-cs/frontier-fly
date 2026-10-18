/**
 * Per-dimension running mean/variance (Welford) and a reward running variance.
 * Count starts at 1e-4 with variance 1 so the first apply() calls stay finite.
 * Reward normalization divides by the running std and does not subtract the mean.
 */

export interface RunningNormState {
  dim: number;
  count: number;
  mean: number[];
  var: number[];
  rewCount: number;
  rewMean: number;
  rewVar: number;
}

const PRIOR_COUNT = 1e-4;
const CLIP = 5;

export class RunningNorm {
  readonly dim: number;
  private count: number;
  private readonly mean: Float64Array;
  private readonly m2: Float64Array;
  private rewCount: number;
  private rewMean: number;
  private rewM2: number;

  constructor(dim: number) {
    if (dim <= 0) throw new Error('RunningNorm dim must be positive');
    this.dim = dim;
    this.count = PRIOR_COUNT;
    this.mean = new Float64Array(dim);
    this.m2 = new Float64Array(dim);
    this.m2.fill(PRIOR_COUNT);
    this.rewCount = PRIOR_COUNT;
    this.rewMean = 0;
    this.rewM2 = PRIOR_COUNT;
  }

  /** n packed samples, each of length dim, row-major. */
  update(x: Float32Array, n: number): void {
    if (n <= 0) return;
    if (x.length < n * this.dim) throw new Error('RunningNorm.update: buffer shorter than n * dim');
    for (let i = 0; i < n; i++) {
      this.count += 1;
      const base = i * this.dim;
      for (let d = 0; d < this.dim; d++) {
        const value = x[base + d]!;
        const delta = value - this.mean[d]!;
        this.mean[d] = this.mean[d]! + delta / this.count;
        const delta2 = value - this.mean[d]!;
        this.m2[d] = this.m2[d]! + delta * delta2;
      }
    }
  }

  /** (x - mean) / sqrt(var + 1e-8), clipped to ±5. Does not update statistics. */
  apply(x: Float32Array, n: number): Float32Array {
    if (x.length < n * this.dim) throw new Error('RunningNorm.apply: buffer shorter than n * dim');
    const out = new Float32Array(n * this.dim);
    const inv = new Float64Array(this.dim);
    for (let d = 0; d < this.dim; d++) {
      inv[d] = 1 / Math.sqrt(this.m2[d]! / this.count + 1e-8);
    }
    for (let i = 0; i < n; i++) {
      const base = i * this.dim;
      for (let d = 0; d < this.dim; d++) {
        let z = (x[base + d]! - this.mean[d]!) * inv[d]!;
        if (z > CLIP) z = CLIP;
        else if (z < -CLIP) z = -CLIP;
        out[base + d] = z;
      }
    }
    return out;
  }

  updateReward(r: number): void {
    this.rewCount += 1;
    const delta = r - this.rewMean;
    this.rewMean += delta / this.rewCount;
    const delta2 = r - this.rewMean;
    this.rewM2 += delta * delta2;
  }

  /** r / sqrt(var + 1e-8). Not mean-centered. */
  normReward(r: number): number {
    const variance = this.rewM2 / this.rewCount;
    return r / Math.sqrt(variance + 1e-8);
  }

  toJSON(): RunningNormState {
    const mean: number[] = [];
    const variance: number[] = [];
    for (let d = 0; d < this.dim; d++) {
      mean.push(this.mean[d]!);
      variance.push(this.m2[d]! / this.count);
    }
    return {
      dim: this.dim,
      count: this.count,
      mean,
      var: variance,
      rewCount: this.rewCount,
      rewMean: this.rewMean,
      rewVar: this.rewM2 / this.rewCount,
    };
  }

  load(state: RunningNormState): void {
    if (state.dim !== this.dim) throw new Error('RunningNorm dim mismatch');
    if (state.mean.length !== this.dim || state.var.length !== this.dim) {
      throw new Error('RunningNorm state length mismatch');
    }
    this.count = state.count;
    this.rewCount = state.rewCount;
    this.rewMean = state.rewMean;
    this.rewM2 = state.rewVar * state.rewCount;
    for (let d = 0; d < this.dim; d++) {
      this.mean[d] = state.mean[d]!;
      this.m2[d] = state.var[d]! * state.count;
    }
  }
}

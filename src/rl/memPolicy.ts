/**
 * The indoor policy with an optional recurrent memory: a small GRU that reads
 * the first hidden layer and feeds the second one. With memDim 0 it is the
 * feed-forward policy of infer.ts, bit for bit on the actor side.
 *
 * Weight buffers: the 13 of infer.ts (the value trunk 7..12 is carried but
 * unused, PPO here trains a separate privileged critic), then, when memDim > 0:
 *
 * 13 gW   [h0 * 3m]   input-to-gate kernel, gate order reset | update | candidate
 * 14 gU   [m * 3m]    state-to-gate kernel
 * 15 gB   [3m]        gate bias
 * 16 wMem [m * h1]    memory into the second hidden layer
 *
 * Forward, per step and env, with memory state s (zero at episode start):
 *   h  = tanh(x W1 + b1)
 *   gx = h gW + gB,  gs = s gU
 *   r = sigmoid(gx_r + gs_r),  z = sigmoid(gx_z + gs_z)
 *   c = tanh(gx_c + r * gs_c)
 *   s' = (1 - z) * s + z * c
 *   h2 = tanh(h W2 + s' wMem + b2)
 *   mean = h2 Wmean + bMean
 * wMem starts at zero, so a memory added to a trained checkpoint does not
 * change its actions until training moves it.
 */
import { RNG } from '../shared/rng';
import type { Checkpoint } from './checkpoint';
import { createGemm, type GemmKernel } from './gemm';
import { gaussianLogp, linear, transpose, WEIGHT_NAMES, weightShapes } from './infer';

export const MEM_WEIGHT_NAMES = ['gW', 'gU', 'gB', 'wMem'] as const;

export function memWeightShapes(actDim: number, obsDim: number, hidden: readonly number[], memDim: number): number[][] {
  const base = weightShapes(obsDim, actDim, hidden);
  if (memDim <= 0) return base;
  const h0 = hidden[0]!;
  const h1 = hidden[1]!;
  return [...base, [h0, 3 * memDim], [memDim, 3 * memDim], [3 * memDim], [memDim, h1]];
}

function numel(shape: readonly number[]): number {
  return shape.reduce((n, d) => n * d, 1);
}

const sigmoid = (v: number) => 1 / (1 + Math.exp(-v));

export interface MemAct {
  action: Float32Array;
  logp: Float32Array;
  /** Pre-tanh means [n * actDim]. */
  mean: Float32Array;
}

export class MemPolicy {
  readonly obsDim: number;
  readonly actDim: number;
  readonly hidden: [number, number];
  readonly memDim: number;
  private readonly weights: Float32Array[];
  private readonly transposed: (Float32Array | null)[];
  private readonly gemm: GemmKernel | null;
  private readonly slot: number[] = [];
  private zeroGateBias: Float32Array;

  constructor(obsDim: number, actDim: number, hidden: readonly number[], memDim = 0, seed = 1) {
    if (hidden.length !== 2) throw new Error('MemPolicy expects 2 hidden layers');
    this.obsDim = obsDim;
    this.actDim = actDim;
    this.hidden = [hidden[0]!, hidden[1]!];
    this.memDim = Math.max(0, memDim | 0);
    const shapes = memWeightShapes(actDim, obsDim, this.hidden, this.memDim);
    this.weights = shapes.map((s) => new Float32Array(numel(s)));
    this.transposed = shapes.map(() => null);
    this.zeroGateBias = new Float32Array(3 * this.memDim);
    const init = new RNG(seed, 'mem-init');
    for (const i of [0, 2, 4, 7, 9, 11]) {
      const w = this.weights[i]!;
      for (let k = 0; k < w.length; k++) w[k] = init.gaussian() * 0.05;
    }
    this.weights[6]!.fill(-0.8);
    if (this.memDim > 0) this.initMemory(init);
    this.gemm = createGemm();
    this.rebuild();
  }

  /** A policy from a checkpoint; `memDim` adds a fresh (inactive) memory to a feed-forward one. */
  static fromCheckpoint(c: Checkpoint, memDim?: number, seed = 1): MemPolicy {
    const m = memDim ?? c.memDim ?? 0;
    const p = new MemPolicy(c.obsDim, c.actDim, c.hidden, m, seed);
    const own = p.getWeights();
    const src = c.weights.map((row) => Float32Array.from(row).buffer);
    p.setWeights(own.map((buf, i) => src[i] ?? buf));
    return p;
  }

  get weightCount(): number {
    return this.weights.length;
  }

  /** Glorot-ish gate kernels, update gate biased closed (keep state), and wMem zero. */
  private initMemory(init: RNG): void {
    const m = this.memDim;
    const h0 = this.hidden[0];
    const gW = this.weights[13]!;
    const gU = this.weights[14]!;
    const sx = Math.sqrt(1 / h0);
    const ss = Math.sqrt(1 / m);
    for (let k = 0; k < gW.length; k++) gW[k] = init.gaussian() * sx;
    for (let k = 0; k < gU.length; k++) gU[k] = init.gaussian() * ss;
    const gB = this.weights[15]!;
    gB.fill(0);
    for (let j = 0; j < m; j++) gB[m + j] = -1;
    this.weights[16]!.fill(0);
  }

  getWeights(): ArrayBuffer[] {
    return this.weights.map((w) => w.slice().buffer);
  }

  setWeights(buffers: ArrayBuffer[]): void {
    if (buffers.length !== this.weights.length) {
      throw new Error(`MemPolicy: expected ${this.weights.length} weight buffers, got ${buffers.length}`);
    }
    buffers.forEach((buf, i) => {
      const src = new Float32Array(buf);
      if (src.length !== this.weights[i]!.length) {
        const name = i < 13 ? WEIGHT_NAMES[i] : MEM_WEIGHT_NAMES[i - 13];
        throw new Error(`MemPolicy: weight ${name} expected ${this.weights[i]!.length} values, got ${src.length}`);
      }
      this.weights[i]!.set(src);
    });
    this.rebuild();
  }

  logStd(): Float32Array {
    return this.weights[6]!.slice();
  }

  /** Zeroed memory state for n envs. */
  initialState(n: number): Float32Array {
    return new Float32Array(n * this.memDim);
  }

  private rebuild(): void {
    const shapes = memWeightShapes(this.actDim, this.obsDim, this.hidden, this.memDim);
    const kernels = this.memDim > 0 ? [0, 2, 4, 13] : [0, 2, 4];
    for (const i of kernels) this.transposed[i] = transpose(this.weights[i]!, shapes[i]![0]!, shapes[i]![1]!);
    if (this.memDim > 0) {
      this.transposed[14] = transpose(this.weights[14]!, this.memDim, 3 * this.memDim);
      this.transposed[16] = transpose(this.weights[16]!, this.memDim, this.hidden[1]);
    }
    this.gemm?.reset();
    this.slot.length = 0;
    if (!this.gemm) return;
    for (const i of kernels) this.slot[i] = this.gemm.upload(this.transposed[i]!);
    for (const i of this.memDim > 0 ? [1, 3, 5, 15] : [1, 3, 5]) this.slot[i] = this.gemm.upload(this.weights[i]!);
  }

  private dense(x: Float32Array, n: number, inDim: number, outDim: number, w: number, b: number, tanhOut: boolean): Float32Array {
    if (this.gemm && this.slot[w] !== undefined && this.slot[b] !== undefined) {
      return this.gemm.apply(x, n, inDim, this.slot[w]!, this.slot[b]!, outDim, tanhOut);
    }
    return linear(x, n, inDim, outDim, this.transposed[w]!, this.weights[b]!, tanhOut);
  }

  /**
   * One step for n envs. `state` [n * memDim] is read and overwritten with the
   * new memory. Returns pre-tanh means.
   */
  forward(obs: Float32Array, n: number, state: Float32Array | null): Float32Array {
    const [h0, h1] = this.hidden;
    const m = this.memDim;
    const h = this.dense(obs, n, this.obsDim, h0, 0, 1, true);
    if (m === 0 || !state) {
      const h2 = this.dense(h, n, h0, h1, 2, 3, true);
      return this.dense(h2, n, h1, this.actDim, 4, 5, false);
    }
    const gx = this.dense(h, n, h0, 3 * m, 13, 15, false);
    const gs = linear(state, n, m, 3 * m, this.transposed[14]!, this.zeroGateBias, false);
    for (let i = 0; i < n; i++) {
      const g = i * 3 * m;
      const s = i * m;
      for (let j = 0; j < m; j++) {
        const r = sigmoid(gx[g + j]! + gs[g + j]!);
        const z = sigmoid(gx[g + m + j]! + gs[g + m + j]!);
        const c = Math.tanh(gx[g + 2 * m + j]! + r * gs[g + 2 * m + j]!);
        state[s + j] = (1 - z) * state[s + j]! + z * c;
      }
    }
    // h2 pre-activation = h W2 + b2 + s' wMem
    const pre = this.dense(h, n, h0, h1, 2, 3, false);
    const fromMem = linear(state, n, m, h1, this.transposed[16]!, new Float32Array(h1), false);
    for (let k = 0; k < pre.length; k++) pre[k] = Math.tanh(pre[k]! + fromMem[k]!);
    return this.dense(pre, n, h1, this.actDim, 4, 5, false);
  }

  /** Act for n envs, advancing `state`. Deterministic acts with the mean. */
  act(obs: Float32Array, n: number, state: Float32Array | null, rng: RNG, deterministic = false): MemAct {
    const mean = this.forward(obs, n, state);
    const logStd = this.weights[6]!;
    const action = new Float32Array(n * this.actDim);
    const logp = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      let lp = 0;
      for (let j = 0; j < this.actDim; j++) {
        const k = i * this.actDim + j;
        const sd = Math.exp(logStd[j]!);
        const u = deterministic ? mean[k]! : mean[k]! + sd * rng.gaussian();
        const a = Math.tanh(u);
        action[k] = a;
        lp += gaussianLogp(u, mean[k]!, sd, logStd[j]!) - Math.log(1 - a * a + 1e-6);
      }
      logp[i] = lp;
    }
    return { action, logp, mean };
  }
}

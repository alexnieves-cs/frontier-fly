/**
 * Weight buffer order (13 Float32 buffers). Inference and the TF.js trainer
 * share this layout. Dense kernels are row-major and input-major, matching a
 * TF.js dense kernel of shape [in, out].
 *
 *  0 w1     [obs * h0]
 *  1 b1     [h0]
 *  2 w2     [h0 * h1]
 *  3 b2     [h1]
 *  4 wMean  [h1 * act]
 *  5 bMean  [act]
 *  6 logStd [act]          unconstrained; std = exp(logStd); init -0.5
 *  7 vw1    [obs * h0]
 *  8 vb1    [h0]
 *  9 vw2    [h0 * h1]
 * 10 vb2    [h1]
 * 11 vOutW  [h1]           value weight
 * 12 vOutB  [1]
 *
 * Forward (separate policy and value trunks):
 *   h  = tanh(x W1 + b1)
 *   h2 = tanh(h W2 + b2)
 *   mean = h2 Wmean + bMean
 *   value trunk is the same tanh stack, then a linear map to a scalar.
 * Action: u = mean + std * gaussian, a = tanh(u).
 * logp (summed over action dims) = gaussian log density of u
 *   minus sum(log(1 - a^2 + 1e-6)).
 * Entropy bonus uses the diagonal Gaussian entropy
 *   0.5 * sum(log(2 * pi * e * std^2)).
 */
import { HIDDEN } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { Policy } from '../shared/types';

export const WEIGHT_NAMES = [
  'w1',
  'b1',
  'w2',
  'b2',
  'wMean',
  'bMean',
  'logStd',
  'vw1',
  'vb1',
  'vw2',
  'vb2',
  'vOutW',
  'vOutB',
] as const;

const LOG_STD_INIT = -0.5;
const WEIGHT_STD = 0.05;
const GAUSS_WEIGHTS = [0, 2, 4, 7, 9, 11];
const LOG_2PI = Math.log(2 * Math.PI);

export function weightShapes(obsDim: number, actDim: number, hidden: readonly number[]): number[][] {
  const h0 = hidden[0] ?? 0;
  const h1 = hidden[1] ?? 0;
  return [
    [obsDim, h0],
    [h0],
    [h0, h1],
    [h1],
    [h1, actDim],
    [actDim],
    [actDim],
    [obsDim, h0],
    [h0],
    [h0, h1],
    [h1],
    [h1, 1],
    [1],
  ];
}

function numel(shape: readonly number[]): number {
  let n = 1;
  for (const d of shape) n *= d;
  return n;
}

export interface ActResult {
  action: Float32Array;
  logp: Float32Array;
  value: Float32Array;
}

function linear(
  x: Float32Array,
  n: number,
  inDim: number,
  outDim: number,
  weight: Float32Array,
  bias: Float32Array,
  tanhOut: boolean,
): Float32Array {
  const y = new Float32Array(n * outDim);
  for (let i = 0; i < n; i++) {
    const x0 = i * inDim;
    const y0 = i * outDim;
    for (let j = 0; j < outDim; j++) {
      let s = bias[j]!;
      for (let k = 0; k < inDim; k++) s += x[x0 + k]! * weight[k * outDim + j]!;
      y[y0 + j] = tanhOut ? Math.tanh(s) : s;
    }
  }
  return y;
}

function gaussianLogp(u: number, mean: number, std: number, logStd: number): number {
  const z = (u - mean) / std;
  return -0.5 * z * z - logStd - 0.5 * LOG_2PI;
}

export class InferPolicy implements Policy {
  readonly obsDim: number;
  readonly actDim: number;
  readonly hidden: number[];
  private readonly weights: Float32Array[];
  private rng: RNG;

  constructor(obsDim: number, actDim: number, hidden: readonly number[] = HIDDEN, seed = 1) {
    if (hidden.length !== 2) {
      throw new Error(`InferPolicy expects 2 hidden layers, received ${hidden.length}`);
    }
    if (obsDim <= 0 || actDim <= 0) throw new Error('obsDim and actDim must be positive');
    const h0 = hidden[0]!;
    const h1 = hidden[1]!;
    if (h0 <= 0 || h1 <= 0) throw new Error('hidden sizes must be positive');
    this.obsDim = obsDim;
    this.actDim = actDim;
    this.hidden = [h0, h1];
    const shapes = weightShapes(obsDim, actDim, this.hidden);
    this.weights = shapes.map((shape) => new Float32Array(numel(shape)));
    const init = new RNG(seed, 'policy-init');
    for (const index of GAUSS_WEIGHTS) {
      const w = this.weights[index]!;
      for (let k = 0; k < w.length; k++) w[k] = init.gaussian() * WEIGHT_STD;
    }
    this.weights[6]!.fill(LOG_STD_INIT);
    this.rng = new RNG(seed, 'policy-act');
  }

  setSeed(seed: number): void {
    this.rng = new RNG(seed, 'policy-act');
  }

  /** Stochastic act() required by Policy. Samples with the internal RNG. */
  act(obs: Float32Array, n: number): ActResult {
    return this.actWith(obs, n, this.rng, false);
  }

  actWith(obs: Float32Array, n: number, rng: RNG, deterministic = false): ActResult {
    if (n < 0) throw new Error('n must be >= 0');
    if (obs.length < n * this.obsDim) throw new Error('observation buffer is short');
    const action = new Float32Array(n * this.actDim);
    const logp = new Float32Array(n);
    if (n === 0) return { action, logp, value: new Float32Array(0) };
    const { mean, value } = this.forward(obs, n);
    const logStd = this.weights[6]!;
    const std = new Float32Array(this.actDim);
    for (let j = 0; j < this.actDim; j++) std[j] = Math.exp(logStd[j]!);
    for (let i = 0; i < n; i++) {
      let lp = 0;
      const base = i * this.actDim;
      for (let j = 0; j < this.actDim; j++) {
        const m = mean[base + j]!;
        const s = std[j]!;
        const u = deterministic ? m : m + s * rng.gaussian();
        const a = Math.tanh(u);
        action[base + j] = a;
        lp += gaussianLogp(u, m, s, logStd[j]!) - Math.log(1 - a * a + 1e-6);
      }
      logp[i] = lp;
    }
    return { action, logp, value };
  }

  getWeights(): ArrayBuffer[] {
    return this.weights.map((src) => {
      const copy = new Float32Array(src.length);
      copy.set(src);
      return copy.buffer;
    });
  }

  setWeights(w: ArrayBuffer[]): void {
    if (w.length !== WEIGHT_NAMES.length) {
      throw new Error(`expected ${WEIGHT_NAMES.length} weight buffers, got ${w.length}`);
    }
    const shapes = weightShapes(this.obsDim, this.actDim, this.hidden);
    for (let i = 0; i < w.length; i++) {
      const src = new Float32Array(w[i]!);
      const expect = numel(shapes[i]!);
      if (src.length !== expect) {
        throw new Error(`weight ${WEIGHT_NAMES[i]} expected ${expect} values, got ${src.length}`);
      }
      this.weights[i]!.set(src);
    }
  }

  private forward(obs: Float32Array, n: number): { mean: Float32Array; value: Float32Array } {
    const h0 = this.hidden[0]!;
    const h1 = this.hidden[1]!;
    const w = this.weights;
    const h = linear(obs, n, this.obsDim, h0, w[0]!, w[1]!, true);
    const h2 = linear(h, n, h0, h1, w[2]!, w[3]!, true);
    const mean = linear(h2, n, h1, this.actDim, w[4]!, w[5]!, false);
    const vh = linear(obs, n, this.obsDim, h0, w[7]!, w[8]!, true);
    const vh2 = linear(vh, n, h0, h1, w[9]!, w[10]!, true);
    const vOutW = w[11]!;
    const vOutB = w[12]![0]!;
    const value = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      let s = vOutB;
      const row = i * h1;
      for (let k = 0; k < h1; k++) s += vh2[row + k]! * vOutW[k]!;
      value[i] = s;
    }
    return { mean, value };
  }
}

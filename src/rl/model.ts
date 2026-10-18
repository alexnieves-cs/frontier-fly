/**
 * TF.js PPO trainer. Weight variables follow the 13-buffer order documented
 * in infer.ts. Node tests pin the CPU backend.
 *
 * Loss = -mean(min(ratio * adv, clip(ratio) * adv))
 *        + 0.5 * MSE(value, returns)
 *        - 0.01 * entropy
 * ratio = exp(newLogp - oldLogp)
 * KL report = abs(mean(oldLogp - newLogp))
 * Advantages are normalized to mean 0 and std 1 once per update.
 */
import * as tf from '@tensorflow/tfjs';
import {
  CLIP_EPS,
  ENTROPY_COEF,
  LEARNING_RATE,
  MINIBATCH,
  PPO_EPOCHS,
  VALUE_COEF,
} from '../shared/constants';
import { RNG } from '../shared/rng';
import type { PpoBatch } from './buffer';
import { WEIGHT_NAMES, weightShapes, type InferPolicy } from './infer';

export interface TrainOpts {
  epochs?: number;
  minibatch?: number;
  clipEps?: number;
  valueCoef?: number;
  entropyCoef?: number;
}

export interface PpoMetrics {
  policyLoss: number;
  valueLoss: number;
  entropy: number;
  kl: number;
}

const LOG_2PI = Math.log(2 * Math.PI);
const LOG_2PIE = Math.log(2 * Math.PI * Math.E);
const SQUASH_EPS = 1e-6;

let nextTrainerId = 0;

function squashedLogp(mean: tf.Tensor, logStd: tf.Tensor, action: tf.Tensor): tf.Tensor {
  const a = tf.clipByValue(action, -1 + SQUASH_EPS, 1 - SQUASH_EPS);
  const u = tf.atanh(a);
  const logStdB = tf.reshape(logStd, [1, -1]);
  const stdB = tf.exp(logStdB);
  const z = tf.div(tf.sub(u, mean), stdB);
  const gauss = tf.sum(
    tf.add(tf.add(tf.mul(tf.square(z), -0.5), tf.neg(logStdB)), -0.5 * LOG_2PI),
    1,
  );
  const corr = tf.sum(tf.log(tf.add(tf.sub(1, tf.square(a)), SQUASH_EPS)), 1);
  return tf.sub(gauss, corr);
}

function diagonalEntropy(logStd: tf.Tensor): tf.Tensor {
  return tf.sum(tf.add(logStd, 0.5 * LOG_2PIE));
}

function normalizeAdvantages(adv: Float32Array, n: number): Float32Array {
  let mean = 0;
  for (let i = 0; i < n; i++) mean += adv[i]!;
  mean /= n;
  let m2 = 0;
  for (let i = 0; i < n; i++) {
    const d = adv[i]! - mean;
    m2 += d * d;
  }
  const std = Math.sqrt(m2 / n + 1e-8);
  const out = new Float32Array(n);
  for (let i = 0; i < n; i++) out[i] = (adv[i]! - mean) / std;
  return out;
}

function shuffled(n: number, rng: RNG): number[] {
  const idx = new Array<number>(n);
  for (let i = 0; i < n; i++) idx[i] = i;
  for (let i = n - 1; i > 0; i--) {
    const j = rng.int(i + 1);
    const tmp = idx[i]!;
    idx[i] = idx[j]!;
    idx[j] = tmp;
  }
  return idx;
}

function gatherFlat(src: Float32Array, indices: number[], stride: number): Float32Array {
  const out = new Float32Array(indices.length * stride);
  for (let i = 0; i < indices.length; i++) {
    const from = indices[i]! * stride;
    for (let j = 0; j < stride; j++) out[i * stride + j] = src[from + j]!;
  }
  return out;
}

export class TfTrainer {
  private readonly id = nextTrainerId++;
  private readonly rng = new RNG(1, 'ppo-minibatch');
  private initPromise: Promise<void> | null = null;
  private initialized = false;
  private vars: tf.Variable[] = [];
  private shapes: number[][] = [];
  private optimizer: tf.Optimizer | null = null;

  constructor(
    private readonly policy: InferPolicy,
    private readonly lr = LEARNING_RATE,
  ) {}

  async ready(): Promise<void> {
    if (!this.initPromise) this.initPromise = this.init();
    await this.initPromise;
  }

  /** Copy TF variables into the inference policy. */
  pullWeights(): void {
    this.assertReady();
    const buffers: ArrayBuffer[] = [];
    for (const variable of this.vars) {
      const data = variable.dataSync();
      const copy = new Float32Array(data.length);
      copy.set(data);
      buffers.push(copy.buffer);
    }
    this.policy.setWeights(buffers);
  }

  /** Copy inference weights into the TF variables. */
  pushWeights(): void {
    this.assertReady();
    const buffers = this.policy.getWeights();
    for (let i = 0; i < this.vars.length; i++) {
      const src = new Float32Array(buffers[i]!);
      const shaped = tf.tensor(src, this.shapes[i], 'float32');
      this.vars[i]!.assign(shaped);
      shaped.dispose();
    }
  }

  async update(batch: PpoBatch, opts?: TrainOpts): Promise<PpoMetrics> {
    await this.ready();
    const n = batch.n;
    if (n <= 0) {
      this.pullWeights();
      return { policyLoss: 0, valueLoss: 0, entropy: 0, kl: 0 };
    }
    const epochs = opts?.epochs ?? PPO_EPOCHS;
    const minibatch = opts?.minibatch ?? MINIBATCH;
    const clipEps = opts?.clipEps ?? CLIP_EPS;
    const valueCoef = opts?.valueCoef ?? VALUE_COEF;
    const entropyCoef = opts?.entropyCoef ?? ENTROPY_COEF;
    const advantages = normalizeAdvantages(batch.advantages, n);
    let totPolicy = 0;
    let totValue = 0;
    let totEntropy = 0;
    let totKl = 0;
    let count = 0;
    for (let epoch = 0; epoch < epochs; epoch++) {
      const order = shuffled(n, this.rng);
      for (let start = 0; start < n; start += minibatch) {
        const mbIdx = order.slice(start, Math.min(n, start + minibatch));
        const metrics = this.applyMinibatch(
          gatherFlat(batch.obs, mbIdx, this.policy.obsDim),
          gatherFlat(batch.actions, mbIdx, this.policy.actDim),
          gatherFlat(batch.logp, mbIdx, 1),
          gatherFlat(advantages, mbIdx, 1),
          gatherFlat(batch.returns, mbIdx, 1),
          mbIdx.length,
          clipEps,
          valueCoef,
          entropyCoef,
        );
        totPolicy += metrics.policyLoss;
        totValue += metrics.valueLoss;
        totEntropy += metrics.entropy;
        totKl += metrics.kl;
        count += 1;
      }
    }
    this.pullWeights();
    if (count === 0) return { policyLoss: 0, valueLoss: 0, entropy: 0, kl: 0 };
    return {
      policyLoss: totPolicy / count,
      valueLoss: totValue / count,
      entropy: totEntropy / count,
      kl: totKl / count,
    };
  }

  private async init(): Promise<void> {
    const ok = await tf.setBackend('cpu');
    if (!ok) throw new Error('failed to set tfjs cpu backend');
    await tf.ready();
    this.shapes = weightShapes(this.policy.obsDim, this.policy.actDim, this.policy.hidden);
    const buffers = this.policy.getWeights();
    this.vars = buffers.map((buf, i) => {
      const data = new Float32Array(buf);
      const tensor = tf.tensor(data, this.shapes[i], 'float32');
      const variable = tf.variable(tensor, true, `ff_${this.id}_${WEIGHT_NAMES[i]}`);
      tensor.dispose();
      return variable;
    });
    this.optimizer = tf.train.adam(this.lr);
    this.initialized = true;
  }

  private assertReady(): void {
    if (!this.initialized || !this.optimizer) throw new Error('Call ready() before using TfTrainer');
  }

  private applyMinibatch(
    obs: Float32Array,
    actions: Float32Array,
    oldLogp: Float32Array,
    adv: Float32Array,
    returns: Float32Array,
    mb: number,
    clipEps: number,
    valueCoef: number,
    entropyCoef: number,
  ): PpoMetrics {
    const obsT = tf.tensor2d(obs, [mb, this.policy.obsDim]);
    const actT = tf.tensor2d(actions, [mb, this.policy.actDim]);
    const oldT = tf.tensor1d(oldLogp);
    const advT = tf.tensor1d(adv);
    const retT = tf.tensor1d(returns);
    const w1 = this.vars[0]!;
    const b1 = this.vars[1]!;
    const w2 = this.vars[2]!;
    const b2 = this.vars[3]!;
    const wMean = this.vars[4]!;
    const bMean = this.vars[5]!;
    const logStd = this.vars[6]!;
    const vw1 = this.vars[7]!;
    const vb1 = this.vars[8]!;
    const vw2 = this.vars[9]!;
    const vb2 = this.vars[10]!;
    const vOutW = this.vars[11]!;
    const vOutB = this.vars[12]!;

    const graph = () => {
      const h = tf.tanh(tf.add(tf.matMul(obsT, w1), b1));
      const h2 = tf.tanh(tf.add(tf.matMul(h, w2), b2));
      const mean = tf.add(tf.matMul(h2, wMean), bMean);
      const vh = tf.tanh(tf.add(tf.matMul(obsT, vw1), vb1));
      const vh2 = tf.tanh(tf.add(tf.matMul(vh, vw2), vb2));
      const value = tf.reshape(tf.add(tf.matMul(vh2, vOutW), vOutB), [mb]);
      const newLogp = squashedLogp(mean, logStd, actT);
      const logRatio = tf.clipByValue(tf.sub(newLogp, oldT), -20, 20);
      const ratio = tf.exp(logRatio);
      const clipped = tf.clipByValue(ratio, 1 - clipEps, 1 + clipEps);
      const policyLoss = tf.neg(tf.mean(tf.minimum(tf.mul(ratio, advT), tf.mul(clipped, advT))));
      const valueLoss = tf.mean(tf.square(tf.sub(value, retT)));
      const entropy = diagonalEntropy(logStd);
      const kl = tf.abs(tf.mean(tf.sub(oldT, newLogp)));
      const loss = tf.add(
        policyLoss,
        tf.sub(tf.mul(valueCoef, valueLoss), tf.mul(entropyCoef, entropy)),
      );
      return { loss, policyLoss, valueLoss, entropy, kl };
    };

    const metrics = tf.tidy(() => {
      const g = graph();
      return {
        policyLoss: g.policyLoss.dataSync()[0] ?? Number.NaN,
        valueLoss: g.valueLoss.dataSync()[0] ?? Number.NaN,
        entropy: g.entropy.dataSync()[0] ?? Number.NaN,
        kl: g.kl.dataSync()[0] ?? Number.NaN,
      };
    });

    const cost = this.optimizer!.minimize(() => graph().loss as tf.Scalar, true, this.vars);
    cost?.dispose();
    tf.dispose([obsT, actT, oldT, advT, retT]);
    return metrics;
  }
}

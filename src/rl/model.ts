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
 *
 * Optional anchor (TrainOpts.anchor), added to the loss:
 *   + klCoef * mean_i KL(pi_ref(.|s_i) || pi(.|s_i))   on the PPO batch states,
 *     closed form for diagonal Gaussians on the pre-tanh u:
 *     sum_j [ logStd_j - refLogStd_j + (refStd_j^2 + (refMean_ij - mean_ij)^2) / (2 std_j^2) - 1/2 ]
 *   + bcCoef * weighted MSE(mean(s_k), atanh(teacher_k))  on a fresh minibatch of the BC dataset
 * Both trunks (policy and value) always receive gradients unless valueOnly is set.
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
  /** Train only the value trunk (vars 7..12). Used to warm up a fresh critic. */
  valueOnly?: boolean;
  anchor?: PpoAnchor;
}

/** Labelled states for BC terms. actions are teacher actions in [-1, 1]. */
export interface BcData {
  obs: Float32Array;
  actions: Float32Array;
  /** Optional per-sample weights [n]; uniform when absent. */
  weights?: Float32Array;
  n: number;
}

export interface PpoAnchor {
  /** Frozen reference policy; its per-sample mean/logStd on the batch states define the KL target. */
  ref?: InferPolicy;
  /** Precomputed reference pre-tanh means on batch.obs [n * actDim] (overrides ref). */
  refMean?: Float32Array;
  /** Reference log std [actDim] (overrides ref). */
  refLogStd?: Float32Array;
  klCoef?: number;
  /** Dataset to draw one BC minibatch from per PPO minibatch. */
  bc?: BcData;
  bcCoef?: number;
  /** BC minibatch size; defaults to the PPO minibatch size. */
  bcMinibatch?: number;
}

export interface PpoMetrics {
  policyLoss: number;
  valueLoss: number;
  entropy: number;
  kl: number;
  /** Mean KL(pi_ref || pi) on the batch, before each step. 0 when no KL anchor. */
  anchorKl?: number;
  /** Mean BC MSE (pre-tanh) on the anchor minibatches. 0 when no BC anchor. */
  bcLoss?: number;
}

const LOG_2PI = Math.log(2 * Math.PI);
const LOG_2PIE = Math.log(2 * Math.PI * Math.E);
const SQUASH_EPS = 1e-6;

let nextTrainerId = 0;
let preferredBackend = 'cpu';

/**
 * Backend that trainers constructed with forceCpu use. Node training scripts
 * call this after importing an optional native backend (tfjs-node registers
 * 'tensorflow'); everything else keeps the pure-JS 'cpu' backend.
 */
export function setTrainerBackend(name: string): void {
  preferredBackend = name;
}

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

/** Closed-form mean over rows of KL(N(refMean, refStd) || N(mean, std)), summed over dims. */
function diagonalKl(refMean: tf.Tensor, refLogStd: tf.Tensor, mean: tf.Tensor, logStd: tf.Tensor): tf.Tensor {
  const ls = tf.reshape(logStd, [1, -1]);
  const rls = tf.reshape(refLogStd, [1, -1]);
  const varP = tf.exp(tf.mul(ls, 2));
  const varR = tf.exp(tf.mul(rls, 2));
  const term = tf.add(
    tf.sub(ls, rls),
    tf.sub(tf.div(tf.add(varR, tf.square(tf.sub(refMean, mean))), tf.mul(varP, 2)), 0.5),
  );
  return tf.mean(tf.sum(term, 1));
}

/** Teacher actions -> pre-tanh targets, matching imitate(). */
export function atanhTargets(actions: Float32Array, count: number): Float32Array {
  const out = new Float32Array(count);
  for (let i = 0; i < count; i++) {
    const clipped = Math.min(0.999, Math.max(-0.999, actions[i] ?? 0));
    out[i] = Math.atanh(clipped);
  }
  return out;
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
  /**
   * TF.js Adam keys its moment slots by position in the var list, so every
   * distinct var subset gets its own optimizer: 'all' (PPO), 'policy'
   * (vars 0..5, imitation) and 'value' (vars 7..12, critic warm-up).
   */
  private readonly subsetOptimizers = new Map<string, tf.Optimizer>();

  constructor(
    private readonly policy: InferPolicy,
    private readonly lr = LEARNING_RATE,
    private readonly forceCpu = true,
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
      return { policyLoss: 0, valueLoss: 0, entropy: 0, kl: 0, anchorKl: 0, bcLoss: 0 };
    }
    const epochs = opts?.epochs ?? PPO_EPOCHS;
    const minibatch = opts?.minibatch ?? MINIBATCH;
    const clipEps = opts?.clipEps ?? CLIP_EPS;
    const valueCoef = opts?.valueCoef ?? VALUE_COEF;
    const entropyCoef = opts?.entropyCoef ?? ENTROPY_COEF;
    const advantages = normalizeAdvantages(batch.advantages, n);
    const actDim = this.policy.actDim;
    const anchor = opts?.anchor;
    const klCoef = anchor?.klCoef ?? 0;
    let refMean: Float32Array | null = null;
    let refLogStd: Float32Array | null = null;
    if (anchor && klCoef > 0) {
      refMean = anchor.refMean ?? null;
      refLogStd = anchor.refLogStd ?? null;
      if (!refMean && anchor.ref) refMean = anchor.ref.meanAndValue(batch.obs, n).mean;
      if (!refLogStd && anchor.ref) refLogStd = anchor.ref.logStd();
      if (!refMean || !refLogStd) throw new Error('KL anchor needs ref or refMean + refLogStd');
      if (refMean.length < n * actDim) throw new Error('KL anchor refMean is short');
    }
    const bcCoef = anchor?.bcCoef ?? 0;
    const bc = anchor?.bc && bcCoef > 0 && anchor.bc.n > 0 ? anchor.bc : null;
    const bcTargets = bc ? atanhTargets(bc.actions, bc.n * actDim) : null;
    const bcMb = Math.max(1, anchor?.bcMinibatch ?? minibatch);
    let totPolicy = 0;
    let totValue = 0;
    let totEntropy = 0;
    let totKl = 0;
    let totAnchorKl = 0;
    let totBc = 0;
    let count = 0;
    for (let epoch = 0; epoch < epochs; epoch++) {
      const order = shuffled(n, this.rng);
      for (let start = 0; start < n; start += minibatch) {
        const mbIdx = order.slice(start, Math.min(n, start + minibatch));
        let anchorArgs: MinibatchAnchor | null = null;
        if (refMean || bc) {
          anchorArgs = { klCoef, bcCoef: 0 };
          if (refMean && refLogStd) {
            anchorArgs.refMean = gatherFlat(refMean, mbIdx, actDim);
            anchorArgs.refLogStd = refLogStd;
          }
          if (bc && bcTargets) {
            const k = Math.min(bcMb, bc.n);
            const idx = new Array<number>(k);
            for (let i = 0; i < k; i++) idx[i] = this.rng.int(bc.n);
            anchorArgs.bcCoef = bcCoef;
            anchorArgs.bcObs = gatherFlat(bc.obs, idx, this.policy.obsDim);
            anchorArgs.bcTargets = gatherFlat(bcTargets, idx, actDim);
            anchorArgs.bcWeights = bc.weights ? gatherFlat(bc.weights, idx, 1) : undefined;
            anchorArgs.bcN = k;
          }
        }
        const metrics = this.applyMinibatch(
          gatherFlat(batch.obs, mbIdx, this.policy.obsDim),
          gatherFlat(batch.actions, mbIdx, actDim),
          gatherFlat(batch.logp, mbIdx, 1),
          gatherFlat(advantages, mbIdx, 1),
          gatherFlat(batch.returns, mbIdx, 1),
          mbIdx.length,
          clipEps,
          valueCoef,
          entropyCoef,
          opts?.valueOnly ?? false,
          anchorArgs,
        );
        totPolicy += metrics.policyLoss;
        totValue += metrics.valueLoss;
        totEntropy += metrics.entropy;
        totKl += metrics.kl;
        totAnchorKl += metrics.anchorKl ?? 0;
        totBc += metrics.bcLoss ?? 0;
        count += 1;
      }
    }
    this.pullWeights();
    if (count === 0) return { policyLoss: 0, valueLoss: 0, entropy: 0, kl: 0, anchorKl: 0, bcLoss: 0 };
    return {
      policyLoss: totPolicy / count,
      valueLoss: totValue / count,
      entropy: totEntropy / count,
      kl: totKl / count,
      anchorKl: totAnchorKl / count,
      bcLoss: totBc / count,
    };
  }

  /**
   * Behavior clone a continuous teacher. Targets are pre-tanh means, so a
   * deterministic act() reproduces the teacher action. With opts.weights the
   * per-minibatch loss is sum_i w_i * mse_i / sum_i w_i.
   */
  async imitate(
    obs: Float32Array,
    actions: Float32Array,
    n: number,
    opts?: {
      epochs?: number;
      minibatch?: number;
      onEpoch?: (epoch: number, loss: number) => void;
      /** Per-sample weights [n]. */
      weights?: Float32Array;
    },
  ): Promise<number> {
    await this.ready();
    if (n <= 0) return 0;
    const epochs = opts?.epochs ?? 4;
    const minibatch = opts?.minibatch ?? 256;
    const weights = opts?.weights;
    if (weights && weights.length < n) throw new Error('imitate: weights shorter than n');
    const targets = atanhTargets(actions, n * this.policy.actDim);
    let last = 0;
    for (let epoch = 0; epoch < epochs; epoch++) {
      const order = shuffled(n, this.rng);
      let total = 0;
      let count = 0;
      for (let start = 0; start < n; start += minibatch) {
        const mbIdx = order.slice(start, Math.min(n, start + minibatch));
        last = this.imitateBatch(
          gatherFlat(obs, mbIdx, this.policy.obsDim),
          gatherFlat(targets, mbIdx, this.policy.actDim),
          mbIdx.length,
          weights ? gatherFlat(weights, mbIdx, 1) : undefined,
        );
        total += last;
        count += 1;
      }
      const mean = count > 0 ? total / count : last;
      opts?.onEpoch?.(epoch, mean);
      last = mean;
    }
    this.pullWeights();
    return last;
  }

  private async init(): Promise<void> {
    if (this.forceCpu) {
      const ok = await tf.setBackend(preferredBackend);
      if (!ok) throw new Error(`failed to set tfjs ${preferredBackend} backend`);
    }
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

  private optimizerFor(subset: 'all' | 'policy' | 'value'): tf.Optimizer {
    if (subset === 'all') return this.optimizer!;
    let opt = this.subsetOptimizers.get(subset);
    if (!opt) {
      opt = tf.train.adam(this.lr);
      this.subsetOptimizers.set(subset, opt);
    }
    return opt;
  }

  private assertReady(): void {
    if (!this.initialized || !this.optimizer) throw new Error('Call ready() before using TfTrainer');
  }

  /** Pre-tanh policy mean for obsT using the current variables. */
  private policyMean(obsT: tf.Tensor): tf.Tensor {
    const h = tf.tanh(tf.add(tf.matMul(obsT, this.vars[0]!), this.vars[1]!));
    const h2 = tf.tanh(tf.add(tf.matMul(h, this.vars[2]!), this.vars[3]!));
    return tf.add(tf.matMul(h2, this.vars[4]!), this.vars[5]!);
  }

  /** Weighted (or plain) mean squared error between mean rows and targets. */
  private static weightedMse(mean: tf.Tensor, tgt: tf.Tensor, w: tf.Tensor | null): tf.Tensor {
    const perRow = tf.mean(tf.square(tf.sub(mean, tgt)), 1);
    if (!w) return tf.mean(perRow);
    return tf.div(tf.sum(tf.mul(perRow, w)), tf.add(tf.sum(w), 1e-8));
  }

  private imitateBatch(obs: Float32Array, targets: Float32Array, mb: number, weights?: Float32Array): number {
    const obsT = tf.tensor2d(obs, [mb, this.policy.obsDim]);
    const tgt = tf.tensor2d(targets, [mb, this.policy.actDim]);
    const wT = weights ? tf.tensor1d(weights) : null;
    const trunk = this.vars.slice(0, 6);
    let before = Number.NaN;
    const cost = this.optimizerFor('policy').minimize(
      () => {
        const loss = TfTrainer.weightedMse(this.policyMean(obsT), tgt, wT) as tf.Scalar;
        before = loss.dataSync()[0] ?? Number.NaN;
        return loss;
      },
      false,
      trunk,
    );
    cost?.dispose();
    tf.dispose(wT ? [obsT, tgt, wT] : [obsT, tgt]);
    return before;
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
    valueOnly: boolean,
    anchor: MinibatchAnchor | null,
  ): PpoMetrics {
    const obsT = tf.tensor2d(obs, [mb, this.policy.obsDim]);
    const actT = tf.tensor2d(actions, [mb, this.policy.actDim]);
    const oldT = tf.tensor1d(oldLogp);
    const advT = tf.tensor1d(adv);
    const retT = tf.tensor1d(returns);
    const owned: tf.Tensor[] = [obsT, actT, oldT, advT, retT];
    const useKl = !!anchor && anchor.klCoef > 0 && !!anchor.refMean && !!anchor.refLogStd;
    const refMeanT = useKl ? tf.tensor2d(anchor.refMean!, [mb, this.policy.actDim]) : null;
    const refLogStdT = useKl ? tf.tensor1d(anchor.refLogStd!) : null;
    const useBc = !!anchor && anchor.bcCoef > 0 && !!anchor.bcObs && (anchor.bcN ?? 0) > 0;
    const bcN = anchor?.bcN ?? 0;
    const bcObsT = useBc ? tf.tensor2d(anchor.bcObs!, [bcN, this.policy.obsDim]) : null;
    const bcTgtT = useBc ? tf.tensor2d(anchor.bcTargets!, [bcN, this.policy.actDim]) : null;
    const bcWT = useBc && anchor.bcWeights ? tf.tensor1d(anchor.bcWeights) : null;
    for (const t of [refMeanT, refLogStdT, bcObsT, bcTgtT, bcWT]) if (t) owned.push(t);
    const logStd = this.vars[6]!;
    const vw1 = this.vars[7]!;
    const vb1 = this.vars[8]!;
    const vw2 = this.vars[9]!;
    const vb2 = this.vars[10]!;
    const vOutW = this.vars[11]!;
    const vOutB = this.vars[12]!;

    const out = { policyLoss: Number.NaN, valueLoss: Number.NaN, entropy: Number.NaN, kl: Number.NaN, anchorKl: 0, bcLoss: 0 };
    const read = (t: tf.Tensor) => t.dataSync()[0] ?? Number.NaN;
    // Metrics are read from the same forward pass the gradient uses, i.e. before the step.
    const lossFn = () => {
      const mean = this.policyMean(obsT);
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
      let loss = tf.add(policyLoss, tf.sub(tf.mul(valueCoef, valueLoss), tf.mul(entropyCoef, entropy)));
      out.policyLoss = read(policyLoss);
      out.valueLoss = read(valueLoss);
      out.entropy = read(entropy);
      out.kl = read(kl);
      if (refMeanT && refLogStdT) {
        const akl = diagonalKl(refMeanT, refLogStdT, mean, logStd);
        out.anchorKl = read(akl);
        loss = tf.add(loss, tf.mul(anchor!.klCoef, akl));
      }
      if (bcObsT && bcTgtT) {
        const bcLoss = TfTrainer.weightedMse(this.policyMean(bcObsT), bcTgtT, bcWT);
        out.bcLoss = read(bcLoss);
        loss = tf.add(loss, tf.mul(anchor!.bcCoef, bcLoss));
      }
      return loss as tf.Scalar;
    };

    const varList = valueOnly ? this.vars.slice(7) : this.vars;
    const cost = this.optimizerFor(valueOnly ? 'value' : 'all').minimize(lossFn, false, varList);
    cost?.dispose();
    tf.dispose(owned);
    return out;
  }
}

interface MinibatchAnchor {
  klCoef: number;
  refMean?: Float32Array;
  refLogStd?: Float32Array;
  bcCoef: number;
  bcObs?: Float32Array;
  bcTargets?: Float32Array;
  bcWeights?: Float32Array;
  bcN?: number;
}

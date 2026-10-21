/**
 * TF.js learner for PPO fine-tuning of MemPolicy (src/rl/memPolicy.ts) with
 * an asymmetric critic.
 *
 * Actor: the policy trunk (buffers 0..6) plus, with memDim > 0, the GRU
 * (13..16), unrolled over fixed-length sequences from each sequence's stored
 * start state (the state the rollout had, so it is slightly stale after an
 * update; the usual PPO-with-memory approximation). The infer.ts value trunk
 * (7..12) is carried along untouched.
 * Critic: a separate tanh MLP on [observation, privileged vector]; it is only
 * used in training and never ships.
 *
 * Actor loss per minibatch, all means over its rows:
 *   ppoCoef * clipped surrogate
 *   + bcCoef * MSE(mean, atanh(teacher))   (frontier-teacher labels on the student's own states)
 *   - entropyCoef * entropy
 * Critic loss: MSE(V(obs, priv), returns), its own Adam.
 */
import * as tf from '@tensorflow/tfjs';
import { RNG } from '../shared/rng';
import type { MemPolicy } from './memPolicy';
import { memWeightShapes } from './memPolicy';
import { atanhTargets } from './model';

/**
 * S sequences of L steps, sequence-major: row s * L + t is step t of sequence s.
 * `starts` marks rows where an episode begins (memory zeroed before the step);
 * `state0` [S * memDim] is each sequence's memory before its first row.
 */
export interface SeqBatch {
  obs: Float32Array;
  priv: Float32Array;
  actions: Float32Array;
  teacher: Float32Array;
  logp: Float32Array;
  advantages: Float32Array;
  returns: Float32Array;
  starts: Uint8Array;
  state0: Float32Array;
  nSeq: number;
  seqLen: number;
}

export interface SeqUpdateOpts {
  epochs: number;
  /** Sequences per minibatch. */
  seqPerBatch: number;
  clipEps: number;
  ppoCoef: number;
  bcCoef: number;
  entropyCoef: number;
  /** Skip the actor (critic warm-up). */
  criticOnly?: boolean;
  /** Skip the critic (pure imitation). */
  actorOnly?: boolean;
  /**
   * Draw the BC term from this replay of earlier labelled sequences (one
   * minibatch of seqPerBatch sequences per step) instead of the update's own
   * rows. A few thousand fresh, correlated rows are too narrow a BC target:
   * imitating only them drags the policy off what it learned from the full
   * DAgger dataset.
   */
  bcReplay?: BcSeqs;
}

/** Labelled sequences for the BC term, laid out like SeqBatch. */
export type BcSeqs = Pick<SeqBatch, 'obs' | 'teacher' | 'starts' | 'state0' | 'nSeq' | 'seqLen'>;

/**
 * Reservoir of labelled sequences (every collected sequence has the same
 * chance to be kept), the BC replay for SeqPpoTrainer.
 */
export class SeqReplay {
  private readonly obs: Float32Array;
  private readonly teacher: Float32Array;
  private readonly starts: Uint8Array;
  private readonly state0: Float32Array;
  private seen = 0;
  size = 0;
  private readonly rng: RNG;

  constructor(
    readonly capacity: number,
    readonly seqLen: number,
    private readonly obsDim: number,
    private readonly actDim: number,
    private readonly memDim: number,
    seed = 1,
  ) {
    this.obs = new Float32Array(capacity * seqLen * obsDim);
    this.teacher = new Float32Array(capacity * seqLen * actDim);
    this.starts = new Uint8Array(capacity * seqLen);
    this.state0 = new Float32Array(capacity * memDim);
    this.rng = new RNG(seed, 'seq-replay');
  }

  add(batch: BcSeqs): void {
    const L = this.seqLen;
    if (batch.seqLen !== L) throw new Error('SeqReplay: sequence length mismatch');
    for (let q = 0; q < batch.nSeq; q++) {
      this.seen += 1;
      let slot = -1;
      if (this.size < this.capacity) slot = this.size++;
      else {
        const j = this.rng.int(this.seen);
        if (j < this.capacity) slot = j;
      }
      if (slot < 0) continue;
      this.obs.set(batch.obs.subarray(q * L * this.obsDim, (q + 1) * L * this.obsDim), slot * L * this.obsDim);
      this.teacher.set(batch.teacher.subarray(q * L * this.actDim, (q + 1) * L * this.actDim), slot * L * this.actDim);
      this.starts.set(batch.starts.subarray(q * L, (q + 1) * L), slot * L);
      if (this.memDim > 0) this.state0.set(batch.state0.subarray(q * this.memDim, (q + 1) * this.memDim), slot * this.memDim);
    }
  }

  view(): BcSeqs {
    const n = this.size;
    const L = this.seqLen;
    return {
      obs: this.obs.subarray(0, n * L * this.obsDim),
      teacher: this.teacher.subarray(0, n * L * this.actDim),
      starts: this.starts.subarray(0, n * L),
      state0: this.state0.subarray(0, n * this.memDim),
      nSeq: n,
      seqLen: L,
    };
  }
}

export interface SeqMetrics {
  policyLoss: number;
  bcLoss: number;
  valueLoss: number;
  /** Mean |old logp - new logp| over the update, the usual approximate KL. */
  kl: number;
  clipFrac: number;
  entropy: number;
}

const LOG_2PI = Math.log(2 * Math.PI);
const LOG_2PIE = Math.log(2 * Math.PI * Math.E);
const SQUASH_EPS = 1e-6;

function squashedLogp(mean: tf.Tensor, logStd: tf.Tensor, action: tf.Tensor): tf.Tensor {
  const a = tf.clipByValue(action, -1 + SQUASH_EPS, 1 - SQUASH_EPS);
  const u = tf.atanh(a);
  const ls = tf.reshape(logStd, [1, -1]);
  const z = tf.div(tf.sub(u, mean), tf.exp(ls));
  const gauss = tf.sum(tf.add(tf.add(tf.mul(tf.square(z), -0.5), tf.neg(ls)), -0.5 * LOG_2PI), 1);
  const corr = tf.sum(tf.log(tf.add(tf.sub(1, tf.square(a)), SQUASH_EPS)), 1);
  return tf.sub(gauss, corr);
}

function gather(src: Float32Array | Uint8Array, rows: number[], stride: number): Float32Array {
  const out = new Float32Array(rows.length * stride);
  for (let i = 0; i < rows.length; i++) {
    const from = rows[i]! * stride;
    for (let j = 0; j < stride; j++) out[i * stride + j] = src[from + j]!;
  }
  return out;
}

export function normalized(x: Float32Array): Float32Array {
  let mean = 0;
  for (const v of x) mean += v;
  mean /= Math.max(1, x.length);
  let m2 = 0;
  for (const v of x) m2 += (v - mean) * (v - mean);
  const std = Math.sqrt(m2 / Math.max(1, x.length) + 1e-8);
  return Float32Array.from(x, (v) => (v - mean) / std);
}

export class SeqPpoTrainer {
  private readonly actor: tf.Variable[] = [];
  private readonly actorIndex: number[] = [];
  private readonly critic: tf.Variable[] = [];
  private readonly actorOpt: tf.Optimizer;
  private readonly criticOpt: tf.Optimizer;
  private readonly bcOpt: tf.Optimizer;
  private readonly rng: RNG;
  private readonly obsDim: number;
  private readonly actDim: number;
  private readonly memDim: number;

  constructor(
    private readonly policy: MemPolicy,
    readonly privDim: number,
    opts: { lr: number; criticLr: number; criticHidden?: [number, number]; seed?: number },
  ) {
    this.obsDim = policy.obsDim;
    this.actDim = policy.actDim;
    this.memDim = policy.memDim;
    this.rng = new RNG(opts.seed ?? 1, 'seq-ppo');
    const shapes = memWeightShapes(this.actDim, this.obsDim, policy.hidden, this.memDim);
    const weights = policy.getWeights();
    const trainable = [0, 1, 2, 3, 4, 5, 6, ...(this.memDim > 0 ? [13, 14, 15, 16] : [])];
    for (const i of trainable) {
      this.actor.push(tf.variable(tf.tensor(new Float32Array(weights[i]!), shapes[i]!), true));
      this.actorIndex.push(i);
    }
    const [c0, c1] = opts.criticHidden ?? [256, 256];
    const init = new RNG(opts.seed ?? 1, 'critic-init');
    const dense = (inDim: number, outDim: number) => {
      const w = new Float32Array(inDim * outDim);
      const s = Math.sqrt(1 / inDim);
      for (let k = 0; k < w.length; k++) w[k] = init.gaussian() * s;
      this.critic.push(tf.variable(tf.tensor2d(w, [inDim, outDim]), true));
      this.critic.push(tf.variable(tf.zeros([outDim]), true));
    };
    dense(this.obsDim + privDim, c0);
    dense(c0, c1);
    dense(c1, 1);
    this.actorOpt = tf.train.adam(opts.lr);
    this.bcOpt = tf.train.adam(opts.lr);
    this.criticOpt = tf.train.adam(opts.criticLr);
  }

  setLearningRate(lr: number): void {
    (this.actorOpt as unknown as { learningRate: number }).learningRate = lr;
    (this.bcOpt as unknown as { learningRate: number }).learningRate = lr;
  }

  private v(index: number): tf.Variable {
    return this.actor[this.actorIndex.indexOf(index)]!;
  }

  /** Pre-tanh means [S * L, actDim] for sequence-major rows. */
  private actorMean(obs: tf.Tensor2D, starts: tf.Tensor2D | null, state0: tf.Tensor2D | null, S: number, L: number): tf.Tensor {
    const h = tf.tanh(tf.add(tf.matMul(obs, this.v(0)), this.v(1)));
    let pre = tf.add(tf.matMul(h, this.v(2)), this.v(3));
    const m = this.memDim;
    if (m > 0 && starts && state0) {
      const gx = tf.reshape(tf.add(tf.matMul(h, this.v(13)), this.v(15)), [S, L, 3 * m]);
      const gU = this.v(14);
      let s: tf.Tensor = state0;
      const outs: tf.Tensor[] = [];
      for (let t = 0; t < L; t++) {
        const keep = tf.sub(1, tf.slice(starts, [0, t], [S, 1]));
        s = tf.mul(s, keep);
        const g = tf.reshape(tf.slice(gx, [0, t, 0], [S, 1, 3 * m]), [S, 3 * m]);
        const gs = tf.matMul(s, gU);
        const r = tf.sigmoid(tf.add(tf.slice(g, [0, 0], [S, m]), tf.slice(gs, [0, 0], [S, m])));
        const z = tf.sigmoid(tf.add(tf.slice(g, [0, m], [S, m]), tf.slice(gs, [0, m], [S, m])));
        const c = tf.tanh(tf.add(tf.slice(g, [0, 2 * m], [S, m]), tf.mul(r, tf.slice(gs, [0, 2 * m], [S, m]))));
        s = tf.add(tf.mul(tf.sub(1, z), s), tf.mul(z, c));
        outs.push(s);
      }
      const mem = tf.reshape(tf.stack(outs, 1), [S * L, m]);
      pre = tf.add(pre, tf.matMul(mem, this.v(16)));
    }
    const h2 = tf.tanh(pre);
    return tf.add(tf.matMul(h2, this.v(4)), this.v(5));
  }

  private criticValue(x: tf.Tensor2D): tf.Tensor {
    const c = this.critic;
    const h = tf.tanh(tf.add(tf.matMul(x, c[0]!), c[1]!));
    const h2 = tf.tanh(tf.add(tf.matMul(h, c[2]!), c[3]!));
    return tf.reshape(tf.add(tf.matMul(h2, c[4]!), c[5]!), [-1]);
  }

  /** Critic values for n rows of observations and privileged vectors. */
  values(obs: Float32Array, priv: Float32Array, n: number): Float32Array {
    const out = new Float32Array(n);
    const chunk = 4096;
    for (let start = 0; start < n; start += chunk) {
      const k = Math.min(chunk, n - start);
      const vals = tf.tidy(() => {
        const o = tf.tensor2d(obs.subarray(start * this.obsDim, (start + k) * this.obsDim), [k, this.obsDim]);
        const p = tf.tensor2d(priv.subarray(start * this.privDim, (start + k) * this.privDim), [k, this.privDim]);
        return this.criticValue(tf.concat([o, p], 1) as tf.Tensor2D);
      });
      out.set(vals.dataSync(), start);
      vals.dispose();
    }
    return out;
  }

  update(batch: SeqBatch, opts: SeqUpdateOpts): SeqMetrics {
    const S = batch.nSeq;
    const L = batch.seqLen;
    const D = this.obsDim;
    const A = this.actDim;
    const P = this.privDim;
    const m = this.memDim;
    const adv = normalized(batch.advantages);
    const targets = atanhTargets(batch.teacher, batch.teacher.length);
    const totals = { policyLoss: 0, bcLoss: 0, valueLoss: 0, kl: 0, clipFrac: 0, entropy: 0 };
    let count = 0;
    const read = (t: tf.Tensor) => t.dataSync()[0] ?? Number.NaN;
    for (let epoch = 0; epoch < opts.epochs; epoch++) {
      const order = Array.from({ length: S }, (_, i) => i);
      for (let i = S - 1; i > 0; i--) {
        const j = this.rng.int(i + 1);
        [order[i], order[j]] = [order[j]!, order[i]!];
      }
      for (let at = 0; at < S; at += opts.seqPerBatch) {
        const seqs = order.slice(at, Math.min(S, at + opts.seqPerBatch));
        const s = seqs.length;
        const rows: number[] = [];
        for (const q of seqs) for (let t = 0; t < L; t++) rows.push(q * L + t);
        const obsT = tf.tensor2d(gather(batch.obs, rows, D), [s * L, D]);
        const owned: tf.Tensor[] = [obsT];
        if (!opts.criticOnly) {
          const actT = tf.tensor2d(gather(batch.actions, rows, A), [s * L, A]);
          const oldT = tf.tensor1d(gather(batch.logp, rows, 1));
          const advT = tf.tensor1d(gather(adv, rows, 1));
          const tgtT = tf.tensor2d(gather(targets, rows, A), [s * L, A]);
          const startsT = m > 0 ? tf.tensor2d(gather(batch.starts, rows, 1), [s, L]) : null;
          const s0T = m > 0 ? tf.tensor2d(gather(batch.state0, seqs, m), [s, m]) : null;
          owned.push(actT, oldT, advT, tgtT);
          if (startsT && s0T) owned.push(startsT, s0T);
          const logStd = this.v(6);
          const replay = opts.bcReplay && opts.bcReplay.nSeq > 0 && opts.bcCoef > 0 ? opts.bcReplay : null;
          let bcIn: { obs: tf.Tensor2D; tgt: tf.Tensor2D; starts: tf.Tensor2D | null; s0: tf.Tensor2D | null; s: number } | null = null;
          if (replay) {
            const k = Math.min(opts.seqPerBatch, replay.nSeq);
            const picks = Array.from({ length: k }, () => this.rng.int(replay.nSeq));
            const rrows: number[] = [];
            for (const q of picks) for (let t = 0; t < L; t++) rrows.push(q * L + t);
            bcIn = {
              obs: tf.tensor2d(gather(replay.obs, rrows, D), [k * L, D]),
              tgt: tf.tensor2d(atanhTargets(gather(replay.teacher, rrows, A), k * L * A), [k * L, A]),
              starts: m > 0 ? tf.tensor2d(gather(replay.starts, rrows, 1), [k, L]) : null,
              s0: m > 0 ? tf.tensor2d(gather(replay.state0, picks, m), [k, m]) : null,
              s: k,
            };
            owned.push(bcIn.obs, bcIn.tgt);
            if (bcIn.starts && bcIn.s0) owned.push(bcIn.starts, bcIn.s0);
          }
          const cost = (opts.ppoCoef > 0 ? this.actorOpt : this.bcOpt).minimize(
            () => {
              const mean = this.actorMean(obsT, startsT, s0T, s, L);
              const bc = bcIn
                ? tf.mean(tf.square(tf.sub(this.actorMean(bcIn.obs, bcIn.starts, bcIn.s0, bcIn.s, L), bcIn.tgt)))
                : tf.mean(tf.square(tf.sub(mean, tgtT)));
              let loss = tf.mul(opts.bcCoef, bc);
              totals.bcLoss += read(bc);
              if (opts.ppoCoef > 0) {
                const newLogp = squashedLogp(mean, logStd, actT);
                const ratio = tf.exp(tf.clipByValue(tf.sub(newLogp, oldT), -20, 20));
                const clipped = tf.clipByValue(ratio, 1 - opts.clipEps, 1 + opts.clipEps);
                const surrogate = tf.neg(tf.mean(tf.minimum(tf.mul(ratio, advT), tf.mul(clipped, advT))));
                const entropy = tf.sum(tf.add(logStd, 0.5 * LOG_2PIE));
                loss = tf.add(loss, tf.sub(tf.mul(opts.ppoCoef, surrogate), tf.mul(opts.entropyCoef, entropy)));
                totals.policyLoss += read(surrogate);
                totals.kl += read(tf.mean(tf.abs(tf.sub(oldT, newLogp))));
                totals.clipFrac += read(tf.mean(tf.cast(tf.greater(tf.abs(tf.sub(ratio, 1)), opts.clipEps), 'float32')));
                totals.entropy += read(entropy);
              }
              return loss as tf.Scalar;
            },
            false,
            opts.ppoCoef > 0 ? this.actor : this.actor.filter((_, k) => this.actorIndex[k] !== 6),
          );
          cost?.dispose();
        }
        if (!opts.actorOnly) {
          const privT = tf.tensor2d(gather(batch.priv, rows, P), [s * L, P]);
          const retT = tf.tensor1d(gather(batch.returns, rows, 1));
          owned.push(privT, retT);
          const cost = this.criticOpt.minimize(
            () => {
              const value = this.criticValue(tf.concat([obsT, privT], 1) as tf.Tensor2D);
              const loss = tf.mean(tf.square(tf.sub(value, retT)));
              totals.valueLoss += read(loss);
              return loss as tf.Scalar;
            },
            false,
            this.critic,
          );
          cost?.dispose();
        }
        tf.dispose(owned);
        count += 1;
      }
    }
    this.pullWeights();
    const k = Math.max(1, count);
    return {
      policyLoss: totals.policyLoss / k,
      bcLoss: totals.bcLoss / k,
      valueLoss: totals.valueLoss / k,
      kl: totals.kl / k,
      clipFrac: totals.clipFrac / k,
      entropy: totals.entropy / k,
    };
  }

  /** Copy the trained variables into the inference policy (the unused value trunk is kept). */
  pullWeights(): void {
    const buffers = this.policy.getWeights();
    this.actor.forEach((v, k) => {
      buffers[this.actorIndex[k]!] = Float32Array.from(v.dataSync()).buffer;
    });
    this.policy.setWeights(buffers);
  }

  /** Set the actor's log std (exploration noise) in both the variables and the policy. */
  setLogStd(values: number[]): void {
    this.v(6).assign(tf.tensor1d(values));
    this.pullWeights();
  }

  criticWeights(): ArrayBuffer[] {
    return this.critic.map((v) => Float32Array.from(v.dataSync()).buffer);
  }

  setCriticWeights(buffers: ArrayBuffer[]): void {
    this.critic.forEach((v, i) => v.assign(tf.tensor(new Float32Array(buffers[i]!), v.shape)));
  }
}

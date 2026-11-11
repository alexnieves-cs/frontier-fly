/**
 * PPO for the MLP goal picker (sim/picker.ts MlpPicker), at the level of goal decisions: one
 * decision is one choice among up to MAX_CANDIDATES candidates, its reward the flown coverage
 * gained until the next decision (percentage points), discounted by the env steps in between
 * (a semi-Markov step). The critic reads the candidates' mean and max features. Plain TypeScript,
 * hand-written gradients: the networks have a few hundred to a few thousand weights.
 */
import { MAX_CANDIDATES, PICK_DIM, type MlpWeights } from '../sim/picker';
import type { PickerEpisodeResult } from './pickerJob';

export interface Decisions {
  m: number;
  n: Int32Array;
  feats: Float32Array; // m × MAX_CANDIDATES × PICK_DIM
  chosen: Int32Array;
  logp: Float32Array;
  adv: Float32Array;
  ret: Float32Array;
}

export interface PpoPickerConfig {
  gamma: number; // per env step
  lambda: number;
  crashCost: number; // percentage points lost on a crash
  clip: number;
  entropy: number;
  lr: number;
  epochs: number;
  minibatch: number;
}

export const PPO_PICKER_DEFAULTS: PpoPickerConfig = { gamma: 0.999, lambda: 0.95, crashCost: 10, clip: 0.2, entropy: 0.003, lr: 3e-4, epochs: 4, minibatch: 512 };

const CRIT_IN = 2 * PICK_DIM;

export interface Critic {
  hidden: number;
  w1: Float64Array;
  b1: Float64Array;
  w2: Float64Array;
  b2: number;
}

export function initCritic(hidden: number, rand: () => number): Critic {
  const g = () => (rand() * 2 - 1) * 0.3;
  return { hidden, w1: Float64Array.from({ length: hidden * CRIT_IN }, g), b1: Float64Array.from({ length: hidden }, g), w2: new Float64Array(hidden), b2: 0 };
}

function criticInput(feats: Float32Array, off: number, n: number, out: Float64Array): void {
  for (let d = 0; d < PICK_DIM; d++) {
    let sum = 0;
    let mx = -Infinity;
    for (let i = 0; i < n; i++) {
      const v = feats[off + i * PICK_DIM + d]!;
      sum += v;
      if (v > mx) mx = v;
    }
    out[d] = sum / n;
    out[PICK_DIM + d] = mx;
  }
}

function criticForward(c: Critic, x: Float64Array, h: Float64Array): number {
  let v = c.b2;
  for (let j = 0; j < c.hidden; j++) {
    let a = c.b1[j]!;
    for (let d = 0; d < CRIT_IN; d++) a += c.w1[j * CRIT_IN + d]! * x[d]!;
    h[j] = Math.tanh(a);
    v += c.w2[j]! * h[j]!;
  }
  return v;
}

export function values(c: Critic, feats: Float32Array, n: Int32Array, m: number): Float32Array {
  const x = new Float64Array(CRIT_IN);
  const h = new Float64Array(c.hidden);
  const out = new Float32Array(m);
  const stride = MAX_CANDIDATES * PICK_DIM;
  for (let i = 0; i < m; i++) {
    criticInput(feats, i * stride, n[i]!, x);
    out[i] = criticForward(c, x, h);
  }
  return out;
}

/** Stack the recorded traces into decisions with GAE advantages and returns (critic values from `critic`). */
export function buildDecisions(eps: PickerEpisodeResult[], critic: Critic, cfg: PpoPickerConfig): Decisions {
  const traces = eps.filter((e) => e.trace && e.trace.steps.length > 0);
  const m = traces.reduce((a, e) => a + e.trace!.steps.length, 0);
  const stride = MAX_CANDIDATES * PICK_DIM;
  const d: Decisions = { m, n: new Int32Array(m), feats: new Float32Array(m * stride), chosen: new Int32Array(m), logp: new Float32Array(m), adv: new Float32Array(m), ret: new Float32Array(m) };
  let o = 0;
  for (const e of traces) {
    const t = e.trace!;
    const k = t.steps.length;
    d.n.set(t.n, o);
    d.feats.set(t.feats, o * stride);
    d.chosen.set(t.chosen, o);
    d.logp.set(t.logp, o);
    const v = values(critic, t.feats, t.n, k);
    let gae = 0;
    for (let i = k - 1; i >= 0; i--) {
      const last = i === k - 1;
      const covNext = last ? e.coverage : t.coverage[i + 1]!;
      let r = (covNext - t.coverage[i]!) * 100;
      if (last && e.crashed) r -= cfg.crashCost;
      const dt = (last ? e.steps : t.steps[i + 1]!) - t.steps[i]!;
      const g = cfg.gamma ** dt;
      const vNext = last ? 0 : v[i + 1]!;
      const delta = r + g * vNext - v[i]!;
      gae = delta + g * cfg.lambda * (last ? 0 : gae);
      d.adv[o + i] = gae;
      d.ret[o + i] = gae + v[i]!;
    }
    o += k;
  }
  // Normalize advantages.
  let mean = 0;
  for (let i = 0; i < m; i++) mean += d.adv[i]!;
  mean /= Math.max(1, m);
  let vr = 0;
  for (let i = 0; i < m; i++) vr += (d.adv[i]! - mean) ** 2;
  const sd = Math.sqrt(vr / Math.max(1, m)) + 1e-8;
  for (let i = 0; i < m; i++) d.adv[i] = (d.adv[i]! - mean) / sd;
  return d;
}

/** Flat views over a picker's weights, for Adam. */
function actorParams(w: MlpWeights): number[][] {
  return [w.lin, w.w1, w.b1, w.w2];
}

class Adam {
  private m: Float64Array[];
  private v: Float64Array[];
  private t = 0;
  constructor(sizes: number[], private lr: number) {
    this.m = sizes.map((s) => new Float64Array(s));
    this.v = sizes.map((s) => new Float64Array(s));
  }
  step(params: (number[] | Float64Array)[], grads: Float64Array[]): void {
    this.t++;
    const b1 = 0.9;
    const b2 = 0.999;
    const c1 = 1 - b1 ** this.t;
    const c2 = 1 - b2 ** this.t;
    params.forEach((p, k) => {
      const g = grads[k]!;
      const m = this.m[k]!;
      const v = this.v[k]!;
      for (let i = 0; i < p.length; i++) {
        m[i] = b1 * m[i]! + (1 - b1) * g[i]!;
        v[i] = b2 * v[i]! + (1 - b2) * g[i]! * g[i]!;
        p[i] = p[i]! - (this.lr * (m[i]! / c1)) / (Math.sqrt(v[i]! / c2) + 1e-8);
      }
    });
  }
}

export class PickerLearner {
  private aOpt: Adam;
  private cOpt: Adam;
  constructor(
    readonly actor: MlpWeights,
    readonly critic: Critic,
    readonly cfg: PpoPickerConfig,
    private rand: () => number,
  ) {
    this.aOpt = new Adam(actorParams(actor).map((p) => p.length), cfg.lr);
    this.cOpt = new Adam([critic.w1.length, critic.b1.length, critic.w2.length, 1], cfg.lr);
  }

  /** Fit the critic only (the actor is left as it is). */
  updateCriticOnly(d: Decisions): { kl: number; clipFrac: number; entropy: number; vLoss: number } {
    return this.update(d, false);
  }

  /** PPO epochs over the decisions; returns mean stats of the last epoch. */
  update(d: Decisions, trainActor = true): { kl: number; clipFrac: number; entropy: number; vLoss: number } {
    const a = this.actor;
    const H = a.hidden;
    const stride = MAX_CANDIDATES * PICK_DIM;
    const idx = Array.from({ length: d.m }, (_, i) => i);
    const s = new Float64Array(MAX_CANDIDATES);
    const hs = new Float64Array(MAX_CANDIDATES * H);
    const p = new Float64Array(MAX_CANDIDATES);
    const cx = new Float64Array(CRIT_IN);
    const ch = new Float64Array(this.critic.hidden);
    let stats = { kl: 0, clipFrac: 0, entropy: 0, vLoss: 0 };
    for (let ep = 0; ep < this.cfg.epochs; ep++) {
      for (let i = idx.length - 1; i > 0; i--) {
        const j = Math.floor(this.rand() * (i + 1));
        [idx[i], idx[j]] = [idx[j]!, idx[i]!];
      }
      let kl = 0;
      let clipN = 0;
      let ent = 0;
      let vl = 0;
      for (let b0 = 0; b0 < idx.length; b0 += this.cfg.minibatch) {
        const batch = idx.slice(b0, b0 + this.cfg.minibatch);
        const gLin = new Float64Array(PICK_DIM);
        const gW1 = new Float64Array(H * PICK_DIM);
        const gB1 = new Float64Array(H);
        const gW2 = new Float64Array(H);
        const c = this.critic;
        const CH = c.hidden;
        const gcW1 = new Float64Array(CH * CRIT_IN);
        const gcB1 = new Float64Array(CH);
        const gcW2 = new Float64Array(CH);
        const gcB2 = new Float64Array(1);
        const inv = 1 / batch.length;
        for (const r of batch) {
          const n = d.n[r]!;
          const off = r * stride;
          // Actor forward.
          let mx = -Infinity;
          for (let i = 0; i < n; i++) {
            let si = 0;
            for (let k = 0; k < PICK_DIM; k++) si += a.lin[k]! * d.feats[off + i * PICK_DIM + k]!;
            for (let j = 0; j < H; j++) {
              let z = a.b1[j]!;
              for (let k = 0; k < PICK_DIM; k++) z += a.w1[j * PICK_DIM + k]! * d.feats[off + i * PICK_DIM + k]!;
              const h = Math.tanh(z);
              hs[i * H + j] = h;
              si += a.w2[j]! * h;
            }
            s[i] = si;
            if (si > mx) mx = si;
          }
          let z = 0;
          for (let i = 0; i < n; i++) z += p[i] = Math.exp(s[i]! - mx);
          for (let i = 0; i < n; i++) p[i]! /= z;
          const k = d.chosen[r]!;
          const logp = Math.log(Math.max(p[k]!, 1e-30));
          const ratio = Math.exp(logp - d.logp[r]!);
          const A = d.adv[r]!;
          kl += d.logp[r]! - logp;
          let h = 0;
          for (let i = 0; i < n; i++) if (p[i]! > 0) h -= p[i]! * Math.log(p[i]!);
          ent += h;
          const clipped = (A > 0 && ratio > 1 + this.cfg.clip) || (A < 0 && ratio < 1 - this.cfg.clip);
          if (clipped) clipN++;
          // dLoss/ds_i, loss = -ratio·A (unclipped) - ent·H
          const gpg = clipped ? 0 : -A * ratio;
          for (let i = 0; i < n; i++) {
            let g = gpg * ((i === k ? 1 : 0) - p[i]!);
            if (p[i]! > 0) g += this.cfg.entropy * p[i]! * (Math.log(p[i]!) + h);
            g *= inv;
            if (g === 0) continue;
            const xo = off + i * PICK_DIM;
            for (let q = 0; q < PICK_DIM; q++) gLin[q]! += g * d.feats[xo + q]!;
            for (let j = 0; j < H; j++) {
              const hj = hs[i * H + j]!;
              gW2[j]! += g * hj;
              const gz = g * a.w2[j]! * (1 - hj * hj);
              if (gz === 0) continue;
              gB1[j]! += gz;
              for (let q = 0; q < PICK_DIM; q++) gW1[j * PICK_DIM + q]! += gz * d.feats[xo + q]!;
            }
          }
          // Critic.
          criticInput(d.feats, off, n, cx);
          const v = criticForward(c, cx, ch);
          const e = v - d.ret[r]!;
          vl += 0.5 * e * e;
          const gv = e * inv;
          gcB2[0]! += gv;
          for (let j = 0; j < CH; j++) {
            gcW2[j]! += gv * ch[j]!;
            const gz = gv * c.w2[j]! * (1 - ch[j]! * ch[j]!);
            gcB1[j]! += gz;
            for (let q = 0; q < CRIT_IN; q++) gcW1[j * CRIT_IN + q]! += gz * cx[q]!;
          }
        }
        if (trainActor) this.aOpt.step(actorParams(a), [gLin, gW1, gB1, gW2]);
        const b2 = [c.b2];
        this.cOpt.step([c.w1, c.b1, c.w2, b2], [gcW1, gcB1, gcW2, gcB2]);
        c.b2 = b2[0]!;
      }
      stats = { kl: kl / d.m, clipFrac: clipN / d.m, entropy: ent / d.m, vLoss: vl / d.m };
    }
    return stats;
  }
}

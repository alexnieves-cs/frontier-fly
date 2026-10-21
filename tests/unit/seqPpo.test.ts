import * as tf from '@tensorflow/tfjs';
import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import { parseCheckpoint, serializeCheckpoint, serializeCheckpointBinary, type Checkpoint } from '../../src/rl/checkpoint';
import { InferPolicy } from '../../src/rl/infer';
import { MemPolicy } from '../../src/rl/memPolicy';
import { SeqPpoTrainer, type SeqBatch } from '../../src/rl/seqPpo';
import { RNG } from '../../src/shared/rng';
import { NodeRolloutPool } from '../../src/train/nodePool';
import { PRIV_DIM, type SeqEvalResult, type SeqRolloutResult } from '../../src/train/ppoJob';

const D = 12;
const A = 4;
const H = [16, 8];

function obsSeq(n: number, seed: number): Float32Array {
  const rng = new RNG(seed);
  return Float32Array.from({ length: n * D }, () => rng.range(-1, 1));
}

function checkpointOf(p: MemPolicy): Checkpoint {
  return {
    version: 1,
    obsVersion: 2,
    obsDim: p.obsDim,
    actDim: p.actDim,
    hidden: [...p.hidden],
    weights: p.getWeights().map((b) => Array.from(new Float32Array(b))),
    obsMean: [],
    obsVar: [],
    obsCount: 1,
    rewVar: 1,
    stage: 3,
    ...(p.memDim > 0 ? { memDim: p.memDim } : {}),
  };
}

describe('MemPolicy', () => {
  beforeAll(async () => {
    await tf.setBackend('cpu');
  });

  it('without memory acts exactly like InferPolicy', () => {
    const ff = new InferPolicy(D, A, H, 3);
    const mem = new MemPolicy(D, A, H, 0);
    mem.setWeights(ff.getWeights());
    const x = obsSeq(5, 1);
    const a = ff.actWith(x, 5, new RNG(1), true).action;
    const b = mem.act(x, 5, null, new RNG(1), true).action;
    for (let i = 0; i < a.length; i++) expect(b[i]).toBeCloseTo(a[i]!, 6);
  });

  it('a fresh memory on a trained policy changes nothing until wMem moves', () => {
    const ff = new InferPolicy(D, A, H, 4);
    const mem = new MemPolicy(D, A, H, 0);
    mem.setWeights(ff.getWeights());
    const withMem = MemPolicy.fromCheckpoint(checkpointOf(mem), 6);
    expect(withMem.memDim).toBe(6);
    const state = withMem.initialState(1);
    for (let t = 0; t < 4; t++) {
      const x = obsSeq(1, 10 + t);
      const a = mem.act(x, 1, null, new RNG(1), true).action;
      const b = withMem.act(x, 1, state, new RNG(1), true).action;
      for (let i = 0; i < A; i++) expect(b[i]).toBeCloseTo(a[i]!, 6);
    }
    // The state still evolves (the GRU runs), it just is not read yet.
    expect(Math.max(...Array.from(state, Math.abs))).toBeGreaterThan(0);
  });

  it('round-trips a memory checkpoint through JSON and binary', () => {
    const p = new MemPolicy(D, A, H, 5, 9);
    const w = p.getWeights();
    new Float32Array(w[16]!).fill(0.1);
    p.setWeights(w);
    const c = checkpointOf(p);
    for (const back of [parseCheckpoint(serializeCheckpoint(c)), parseCheckpoint(serializeCheckpointBinary(c))]) {
      expect(back.memDim).toBe(5);
      expect(back.weights.length).toBe(17);
      const q = MemPolicy.fromCheckpoint(back);
      const x = obsSeq(3, 2);
      const sa = p.initialState(3);
      const sb = q.initialState(3);
      const a = p.act(x, 3, sa, new RNG(1), true).action;
      const b = q.act(x, 3, sb, new RNG(1), true).action;
      for (let i = 0; i < a.length; i++) expect(b[i]).toBeCloseTo(a[i]!, 6);
    }
    const ff = checkpointOf(new MemPolicy(D, A, H, 0));
    expect(parseCheckpoint(serializeCheckpoint(ff)).memDim).toBeUndefined();
    expect(() => parseCheckpoint(serializeCheckpoint({ ...c, memDim: undefined }))).toThrow(/expected 13/);
  });
});

describe('SeqPpoTrainer', () => {
  beforeAll(async () => {
    await tf.setBackend('cpu');
  });

  /** One sequence batch from stepping the policy, with an episode start in the middle. */
  function rollout(p: MemPolicy, S: number, L: number) {
    const rng = new RNG(5);
    const obs = obsSeq(S * L, 7);
    const starts = new Uint8Array(S * L);
    starts[0] = 1;
    starts[Math.floor(L / 2)] = 1;
    const state0 = new Float32Array(S * p.memDim);
    const state = p.initialState(1);
    const actions = new Float32Array(S * L * A);
    const logp = new Float32Array(S * L);
    const means = new Float32Array(S * L * A);
    for (let s = 0; s < S; s++) {
      state0.set(state, s * p.memDim);
      for (let t = 0; t < L; t++) {
        const row = s * L + t;
        if (starts[row]) {
          state.fill(0);
          if (t === 0) state0.fill(0, s * p.memDim, (s + 1) * p.memDim);
        }
        const out = p.act(obs.subarray(row * D, (row + 1) * D), 1, state, rng, false);
        actions.set(out.action, row * A);
        logp[row] = out.logp[0]!;
        means.set(out.mean, row * A);
      }
    }
    return { obs, starts, state0, actions, logp, means };
  }

  it('unrolls the GRU exactly like the inference policy, and PPO lowers the surrogate', () => {
    const p = new MemPolicy(D, A, H, 6, 2);
    const w = p.getWeights();
    const wMem = new Float32Array(w[16]!);
    const r = new RNG(3);
    for (let k = 0; k < wMem.length; k++) wMem[k] = r.gaussian() * 0.5;
    p.setWeights(w);
    const S = 3;
    const L = 5;
    const roll = rollout(p, S, L);
    const trainer = new SeqPpoTrainer(p, PRIV_DIM, { lr: 1e-3, criticLr: 1e-3 });
    // Recompute the logp of the taken actions through the TF unroll: same as the rollout's.
    const advantages = Float32Array.from({ length: S * L }, (_, i) => (i % 2 ? 1 : -1));
    const batch: SeqBatch = {
      obs: roll.obs,
      priv: new Float32Array(S * L * PRIV_DIM),
      actions: roll.actions,
      teacher: roll.actions,
      logp: roll.logp,
      advantages,
      returns: new Float32Array(S * L),
      starts: roll.starts,
      state0: roll.state0,
      nSeq: S,
      seqLen: L,
    };
    const first = trainer.update(batch, { epochs: 1, seqPerBatch: S, clipEps: 0.2, ppoCoef: 1, bcCoef: 0, entropyCoef: 0 });
    // Before the first step new logp == old logp, so the approximate KL is 0 and nothing is clipped.
    expect(first.kl).toBeLessThan(1e-4);
    expect(first.clipFrac).toBe(0);
    let last = first.policyLoss;
    for (let i = 0; i < 20; i++) last = trainer.update(batch, { epochs: 1, seqPerBatch: S, clipEps: 0.2, ppoCoef: 1, bcCoef: 0, entropyCoef: 0, actorOnly: true }).policyLoss;
    expect(last).toBeLessThan(first.policyLoss);
  });

  it('imitation fits teacher labels and the critic fits returns', () => {
    const p = new MemPolicy(D, A, H, 4, 1);
    const S = 4;
    const L = 4;
    const roll = rollout(p, S, L);
    const teacher = Float32Array.from({ length: S * L * A }, (_, i) => (i % A === 0 ? 0.5 : -0.2));
    const priv = Float32Array.from({ length: S * L * PRIV_DIM }, (_, i) => (i % 3) / 3);
    const returns = Float32Array.from({ length: S * L }, (_, i) => priv[i * PRIV_DIM]! * 2);
    const trainer = new SeqPpoTrainer(p, PRIV_DIM, { lr: 3e-3, criticLr: 3e-3 });
    const batch: SeqBatch = {
      obs: roll.obs,
      priv,
      actions: roll.actions,
      teacher,
      logp: roll.logp,
      advantages: new Float32Array(S * L),
      returns,
      starts: roll.starts,
      state0: roll.state0,
      nSeq: S,
      seqLen: L,
    };
    const opts = { epochs: 1, seqPerBatch: S, clipEps: 0.2, ppoCoef: 0, bcCoef: 1, entropyCoef: 0 };
    const first = trainer.update(batch, opts);
    let last = first;
    for (let i = 0; i < 60; i++) last = trainer.update(batch, opts);
    expect(last.bcLoss).toBeLessThan(first.bcLoss * 0.2);
    expect(last.valueLoss).toBeLessThan(first.valueLoss * 0.5);
    const v = trainer.values(roll.obs, priv, S * L);
    expect(v.length).toBe(S * L);
    expect(Number.isFinite(v[0]!)).toBe(true);
  });
});

describe('PPO worker jobs', () => {
  let pool: NodeRolloutPool;
  const T = 8;
  beforeAll(async () => {
    pool = new NodeRolloutPool({
      workers: 2,
      envsPerWorker: 2,
      seed: 5,
      worldPool: { stages: [0, 3], perStage: 2 },
      jobModules: ['src/train/ppoJob.ts'],
      autoDagger: false,
    });
    await pool.init();
  });
  afterAll(async () => {
    await pool.close();
  });

  it('collects sequence arrays with teacher labels, privileged features and memory starts', async () => {
    const policy = new MemPolicy(1088, A, [32, 16], 4, 1);
    const spec = { weights: policy.getWeights(), hidden: [32, 16], memDim: 4 };
    const parts = (await pool.runJobAll('seqRollout', {
      policy: spec,
      stepsPerEnv: T,
      seqLen: 4,
      stageMix: [1, 0, 0, 1],
      nightFraction: 0.5,
      seed: 3,
      resetEnvs: true,
    })) as SeqRolloutResult[];
    expect(parts).toHaveLength(2);
    for (const part of parts) {
      expect(part.obs.length).toBe(2 * T * 1088);
      expect(part.priv.length).toBe(2 * T * PRIV_DIM);
      expect(part.teacher.length).toBe(2 * T * A);
      expect(part.state0.length).toBe(2 * (T / 4) * 4);
      // Each env's first row starts an episode and its first sequence starts from a zero memory.
      expect(part.starts[0]).toBe(1);
      expect(part.starts[T]).toBe(1);
      expect(Array.from(part.state0.subarray(0, 4))).toEqual([0, 0, 0, 0]);
      // The second sequence of env 0 starts from wherever the GRU got to.
      expect(Math.max(...Array.from(part.state0.subarray(4, 8), Math.abs))).toBeGreaterThan(0);
      expect(part.teacher.some((v) => v !== 0)).toBe(true);
    }
  });

  it('evaluates a checkpoint deterministically on fixed seeds', async () => {
    const policy = new MemPolicy(1088, A, [32, 16], 0, 1);
    const spec = { weights: policy.getWeights(), hidden: [32, 16], memDim: 0 };
    const jobs = [{ stage: 0, seed: 66201, lighting: 'day' as const }];
    const a = (await pool.runJob('seqEval', { policy: spec, jobs })) as SeqEvalResult[];
    const b = (await pool.runJob('seqEval', { policy: spec, jobs })) as SeqEvalResult[];
    expect(a[0]!.steps).toBeGreaterThan(0);
    expect(a).toEqual(b);
  });
});

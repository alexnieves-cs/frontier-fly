import { describe, expect, it } from 'vitest';
import raw from '../fixtures/simpleRoom.json';
import {
  collectDagger,
  DaggerDataset,
  daggerTransferables,
  evalLikeConfig,
  trainDagger,
  trainingBuilding,
  type StudentPolicy,
} from '../../src/rl/dagger';
import { ppoFinetune } from '../../src/rl/finetune';
import { InferPolicy } from '../../src/rl/infer';
import { TfTrainer } from '../../src/rl/model';
import { ACT_DIM, HIDDEN, OBS_DIM } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import { buildingFromJson } from '../../src/shared/serialize';
import type { EpisodeConfig } from '../../src/shared/types';
import { createEnv, defaultEpisodeConfig } from '../../src/sim/env';

const spec = buildingFromJson(raw);

function cfg(): EpisodeConfig {
  const c = defaultEpisodeConfig();
  c.maxSteps = 10_000;
  c.stallSteps = 10_000;
  c.maxCollisions = 10_000;
  return c;
}

/** A student that always pushes into the -x/-y corner. */
const cornerStudent: StudentPolicy = {
  actWith: (_obs, n) => ({ action: new Float32Array(n * ACT_DIM).fill(0).map((_, i) => (i % ACT_DIM < 2 ? -1 : 0)) }),
};

describe('DaggerDataset', () => {
  it('aggregates rollouts and keeps weights aligned with samples', () => {
    const ds = new DaggerDataset(3, 2, 10);
    ds.addRollout({
      obs: Float32Array.from([1, 1, 1, 2, 2, 2]),
      actions: Float32Array.from([0.1, 0.2, 0.3, 0.4]),
      weights: Float32Array.from([1, 3]),
      n: 2,
    });
    ds.add([3, 3, 3], [0.5, 0.6]);
    expect(ds.size).toBe(3);
    expect(ds.seen).toBe(3);
    const v = ds.view();
    expect(v.n).toBe(3);
    expect(Array.from(v.obs)).toEqual([1, 1, 1, 2, 2, 2, 3, 3, 3]);
    expect(Array.from(v.weights)).toEqual([1, 3, 1]);
    expect(v.actions[3]).toBeCloseTo(0.4, 6);
    expect(ds.meanWeight()).toBeCloseTo(5 / 3, 6);
  });

  it('fifo overwrites the oldest samples once full', () => {
    const ds = new DaggerDataset(1, 1, 4, { policy: 'fifo' });
    for (let i = 0; i < 6; i++) ds.add([i], [i / 10], i);
    expect(ds.size).toBe(4);
    expect(ds.seen).toBe(6);
    expect(Array.from(ds.view().obs).sort()).toEqual([2, 3, 4, 5]);
    // weight follows its sample
    const v = ds.view();
    for (let i = 0; i < v.n; i++) expect(v.weights[i]).toBe(v.obs[i]);
  });

  it('reservoir keeps a roughly uniform sample of everything seen', () => {
    const ds = new DaggerDataset(1, 1, 200, { policy: 'reservoir', seed: 3 });
    for (let i = 0; i < 2000; i++) ds.add([i], [0], 1);
    expect(ds.size).toBe(200);
    const v = ds.view();
    let early = 0;
    for (let i = 0; i < v.n; i++) if (v.obs[i]! < 1000) early += 1;
    expect(early).toBeGreaterThan(70);
    expect(early).toBeLessThan(130);
  });
});

describe('collectDagger', () => {
  it('returns teacher labels for every visited state with owned, transferable buffers', async () => {
    const env = await createEnv();
    const policy = new InferPolicy(OBS_DIM, ACT_DIM, HIDDEN, 2);
    const r = collectDagger({ env, spec, cfg: cfg(), policy, beta: 0.5, steps: 60, rng: new RNG(4), seed: 4 });
    expect(r.n).toBe(60);
    expect(r.obsDim).toBe(policy.obsDim);
    expect(r.actDim).toBe(ACT_DIM);
    expect(r.obs.length).toBe(60 * policy.obsDim);
    expect(r.actions.length).toBe(60 * ACT_DIM);
    expect(r.weights.length).toBe(60);
    expect(r.teacherSteps).toBeGreaterThan(10);
    expect(r.teacherSteps).toBeLessThan(50);
    for (const buf of daggerTransferables(r)) expect(buf.byteLength).toBeGreaterThan(0);
    // Buffers are exactly sized (sliced), so transferring them hands over no extra memory.
    expect(r.obs.buffer.byteLength).toBe(r.obs.byteLength);
    for (let i = 0; i < r.actions.length; i++) expect(Math.abs(r.actions[i]!)).toBeLessThanOrEqual(1);
    for (let i = 0; i < r.n; i++) expect(r.weights[i]).toBeGreaterThanOrEqual(1);
    env.dispose();
  });

  it('up-weights divergent and stalled states', async () => {
    const env = await createEnv();
    const r = collectDagger({
      env,
      spec,
      cfg: cfg(),
      policy: cornerStudent,
      beta: 0,
      steps: 200,
      rng: new RNG(5),
      seed: 5,
      weighting: { divergeBoost: 1, stallBoost: 2, maxWeight: 10 },
    });
    expect(r.teacherSteps).toBe(0);
    expect(r.divergent).toBeGreaterThan(100);
    expect(r.stalled).toBeGreaterThan(50);
    let maxW = 0;
    for (let i = 0; i < r.n; i++) maxW = Math.max(maxW, r.weights[i]!);
    expect(maxW).toBe(4);
    // The teacher label in the pinned corner points back out (positive x or y).
    const last = r.actions.subarray((r.n - 1) * ACT_DIM, r.n * ACT_DIM);
    expect(Math.max(last[0]!, last[1]!, Math.abs(last[3]!))).toBeGreaterThan(0.1);
    env.dispose();
  });

  it('pure teacher rollouts cover a generated stage-0 building', async () => {
    const env = await createEnv();
    const building = trainingBuilding(0, 12345);
    const r = collectDagger({
      env,
      spec: building,
      cfg: evalLikeConfig(0, building),
      policy: cornerStudent,
      beta: 1,
      steps: 1000,
      rng: new RNG(6),
      seed: building.seed,
    });
    expect(r.teacherSteps).toBe(r.n);
    expect(r.coverage).toBeGreaterThan(0.8);
    env.dispose();
  });

  it('refuses eval seeds for training buildings', () => {
    expect(() => trainingBuilding(0, 101)).toThrow(/eval/);
    expect(() => trainingBuilding(3, 4242)).toThrow(/eval/);
    expect(trainingBuilding(0, 12345).floors.length).toBe(1);
  });
});

function smallPolicy(seed = 3): InferPolicy {
  return new InferPolicy(6, 2, [16, 16], seed);
}

function randomBatch(n: number, obsDim: number, actDim: number, rng: RNG) {
  const obs = new Float32Array(n * obsDim);
  for (let i = 0; i < obs.length; i++) obs[i] = rng.range(-1, 1);
  const actions = new Float32Array(n * actDim);
  for (let i = 0; i < actions.length; i++) actions[i] = rng.range(-0.9, 0.9);
  return { obs, actions };
}

describe('PPO anchors', () => {
  it('KL to the reference is ~0 against itself and positive after perturbation', async () => {
    const policy = smallPolicy();
    const trainer = new TfTrainer(policy, 0);
    await trainer.ready();
    const rng = new RNG(9);
    const n = 64;
    const { obs, actions } = randomBatch(n, 6, 2, rng);
    const logp = policy.actWith(obs, n, new RNG(1), false).logp;
    const batch = { obs, actions, logp, advantages: new Float32Array(n).fill(0), returns: new Float32Array(n), n };
    const same = new InferPolicy(6, 2, [16, 16], 1);
    same.setWeights(policy.getWeights());
    const m0 = await trainer.update(batch, { epochs: 1, minibatch: n, anchor: { ref: same, klCoef: 1 } });
    expect(Math.abs(m0.anchorKl!)).toBeLessThan(1e-5);
    const perturbed = same.getWeights();
    new Float32Array(perturbed[5]!)[0] = 0.5;
    new Float32Array(perturbed[6]!)[1] = -1.5;
    same.setWeights(perturbed);
    const m1 = await trainer.update(batch, { epochs: 1, minibatch: n, anchor: { ref: same, klCoef: 1 } });
    expect(m1.anchorKl!).toBeGreaterThan(0.05);
    // lr 0 keeps the policy fixed, so the KL matches the closed form exactly:
    // dim 0: mean shift d (per sample), dim 1: logStd change only.
    const ls = policy.logStd();
    const refMean = same.meanAndValue(obs, n).mean;
    const curMean = policy.meanAndValue(obs, n).mean;
    let expected = 0;
    for (let i = 0; i < n; i++) {
      const d = refMean[i * 2]! - curMean[i * 2]!;
      expected += (d * d) / (2 * Math.exp(2 * ls[0]!));
    }
    expected /= n;
    const r1 = -1.5;
    expected += ls[1]! - r1 + Math.exp(2 * r1) / (2 * Math.exp(2 * ls[1]!)) - 0.5;
    expect(m1.anchorKl!).toBeCloseTo(expected, 3);
  });

  it('KL anchor pulls the policy toward the reference', async () => {
    const policy = smallPolicy(4);
    const trainer = new TfTrainer(policy, 5e-3);
    await trainer.ready();
    const rng = new RNG(10);
    const n = 128;
    const { obs, actions } = randomBatch(n, 6, 2, rng);
    const ref = new InferPolicy(6, 2, [16, 16], 1);
    const w = policy.getWeights();
    new Float32Array(w[5]!).set([0.8, -0.6]);
    ref.setWeights(w);
    const batch = { obs, actions, logp: new Float32Array(n), advantages: new Float32Array(n), returns: new Float32Array(n), n };
    const first = await trainer.update(batch, { epochs: 1, minibatch: n, entropyCoef: 0, anchor: { ref, klCoef: 1 } });
    let last = first;
    for (let i = 0; i < 30; i++) {
      last = await trainer.update(batch, { epochs: 1, minibatch: n, entropyCoef: 0, anchor: { ref, klCoef: 1 } });
    }
    expect(last.anchorKl!).toBeLessThan(first.anchorKl! * 0.5);
  });

  it('BC anchor pulls the mean toward teacher labels and the value net keeps training', async () => {
    const policy = smallPolicy(5);
    const trainer = new TfTrainer(policy, 5e-3);
    await trainer.ready();
    const rng = new RNG(11);
    const n = 128;
    const { obs, actions } = randomBatch(n, 6, 2, rng);
    const teacher = new Float32Array(n * 2);
    for (let i = 0; i < n; i++) {
      teacher[i * 2] = Math.tanh(obs[i * 6]! * 1.5);
      teacher[i * 2 + 1] = -0.5;
    }
    const bc = { obs, actions: teacher, weights: new Float32Array(n).fill(1), n };
    const returns = new Float32Array(n).fill(2);
    const batch = { obs, actions, logp: new Float32Array(n), advantages: new Float32Array(n), returns, n };
    const err = () => {
      const pred = policy.actWith(obs, n, new RNG(1), true).action;
      let e = 0;
      for (let i = 0; i < pred.length; i++) e += (pred[i]! - teacher[i]!) ** 2;
      return e / pred.length;
    };
    const before = err();
    const v0 = new Float32Array(policy.getWeights()[12]!)[0]!;
    let first = Number.NaN;
    let last = Number.NaN;
    for (let i = 0; i < 40; i++) {
      const m = await trainer.update(batch, {
        epochs: 1,
        minibatch: n,
        entropyCoef: 0,
        anchor: { bc, bcCoef: 1, bcMinibatch: 64 },
      });
      if (i === 0) first = m.bcLoss!;
      last = m.bcLoss!;
    }
    expect(err()).toBeLessThan(before * 0.5);
    expect(last).toBeLessThan(first);
    const v1 = new Float32Array(policy.getWeights()[12]!)[0]!;
    expect(Math.abs(v1 - v0)).toBeGreaterThan(1e-3);
  });

  it('valueOnly leaves the policy trunk untouched', async () => {
    const policy = smallPolicy(6);
    const trainer = new TfTrainer(policy, 1e-2);
    await trainer.ready();
    const rng = new RNG(12);
    const n = 64;
    const { obs, actions } = randomBatch(n, 6, 2, rng);
    const adv = new Float32Array(n);
    for (let i = 0; i < n; i++) adv[i] = rng.range(-1, 1);
    const before = policy.getWeights().map((b) => Array.from(new Float32Array(b)));
    await trainer.update(
      { obs, actions, logp: new Float32Array(n), advantages: adv, returns: new Float32Array(n).fill(1), n },
      { epochs: 2, minibatch: 32, valueOnly: true },
    );
    const after = policy.getWeights().map((b) => Array.from(new Float32Array(b)));
    for (let i = 0; i <= 6; i++) expect(after[i]).toEqual(before[i]);
    expect(after[12]).not.toEqual(before[12]);
  });

  it('weighted imitation favours heavily weighted samples', async () => {
    const obs = new Float32Array(2 * 6);
    obs.set([1, 0, 0, 0, 0, 0], 0);
    obs.set([1, 0, 0, 0, 0, 0], 6);
    // Same state, conflicting labels; the weighted label should win.
    const actions = Float32Array.from([0.6, 0.6, -0.6, -0.6]);
    const policy = smallPolicy(7);
    const trainer = new TfTrainer(policy, 1e-2);
    await trainer.ready();
    await trainer.imitate(obs, actions, 2, { epochs: 200, minibatch: 2, weights: Float32Array.from([9, 1]) });
    const a = policy.actWith(obs, 1, new RNG(1), true).action;
    expect(a[0]!).toBeGreaterThan(0.3);
    expect(a[1]!).toBeGreaterThan(0.3);
  });
});

describe('drivers', () => {
  it('trainDagger and ppoFinetune run end to end and never regress validation', async () => {
    const validation = [{ stage: 0, seed: 7001, lighting: 'day' as const }];
    const iters: number[] = [];
    const dagger = await trainDagger({
      iterations: 1,
      bcEpisodes: 1,
      episodesPerIter: 1,
      maxEpisodeSteps: 120,
      bcEpochs: 1,
      retrainEpochs: 1,
      capacity: 2000,
      validation,
      onIter: (m) => iters.push(m.iter),
    });
    expect(iters).toEqual([0, 1]);
    expect(dagger.metrics[1]!.beta).toBeCloseTo(0.5, 6);
    expect(dagger.dataset.size).toBeGreaterThan(100);
    expect(dagger.checkpoint.weights).toHaveLength(13);
    expect(dagger.checkpoint.obsDim).toBe(OBS_DIM);
    const tuned = await ppoFinetune({
      checkpoint: dagger.checkpoint,
      dataset: dagger.dataset.view(),
      updates: 1,
      valueWarmup: 1,
      horizon: 64,
      epochs: 1,
      minibatch: 32,
      validation,
    });
    expect(tuned.metrics).toHaveLength(2);
    expect(tuned.metrics[0]!.valueOnly).toBe(true);
    expect(tuned.metrics[1]!.bcLoss).toBeGreaterThan(0);
    expect(tuned.bestValidation.score).toBeGreaterThanOrEqual(tuned.startValidation.score);
    expect(tuned.checkpoint.weights).toHaveLength(13);
  }, 60_000);
});

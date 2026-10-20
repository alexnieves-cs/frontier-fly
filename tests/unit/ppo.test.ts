import { describe, expect, it } from 'vitest';
import { parseCheckpoint, serializeCheckpoint, loadCheckpointIdb, saveCheckpointIdb } from '../../src/rl/checkpoint';
import { InferPolicy } from '../../src/rl/infer';
import { TfTrainer } from '../../src/rl/model';
import { computeGae, computeGaeInterleaved } from '../../src/rl/ppo';
import { trainToy } from '../../src/rl/toy';
import { RNG } from '../../src/shared/rng';
import type { Checkpoint } from '../../src/rl/checkpoint';

function buffersFromNumbers(rows: number[][]): ArrayBuffer[] {
  return rows.map((row) => {
    const arr = Float32Array.from(row);
    return arr.buffer.slice(arr.byteOffset, arr.byteOffset + arr.byteLength);
  });
}

function checkpointFrom(policy: InferPolicy, stage = 3): Checkpoint {
  return {
    version: 1,
    obsVersion: 2,
    obsDim: policy.obsDim,
    actDim: policy.actDim,
    hidden: [...policy.hidden],
    weights: policy.getWeights().map((buf) => Array.from(new Float32Array(buf))),
    obsMean: [0, 0, 0, 0],
    obsVar: [1, 1, 1, 1],
    obsCount: 1,
    rewVar: 1,
    stage,
  };
}

describe('policy weights', () => {
  it('round-trips getWeights/setWeights and a JSON checkpoint', () => {
    const obs = new Float32Array([0.2, -0.1, 0.8, 1, -0.3, 0.4, 1.1, 1]);
    const original = new InferPolicy(4, 1, [32, 32], 7);
    const before = original.actWith(obs, 2, new RNG(3), true);
    const copied = original.getWeights();
    new Float32Array(copied[5]!)[0] = 99;
    const untouched = original.actWith(obs, 2, new RNG(3), true);
    expect(Math.abs(before.action[0]! - untouched.action[0]!)).toBeLessThan(1e-5);

    const mutated = original.getWeights().map((buf) => buf.slice(0));
    new Float32Array(mutated[5]!)[0] = (new Float32Array(mutated[5]!)[0] ?? 0) + 0.5;
    const loaded = new InferPolicy(4, 1, [32, 32], 99);
    loaded.setWeights(mutated);
    const again = new InferPolicy(4, 1, [32, 32], 100);
    again.setWeights(loaded.getWeights());

    const a = loaded.actWith(obs, 2, new RNG(5), false);
    const b = again.actWith(obs, 2, new RNG(5), false);
    expect(a.action.length).toBe(b.action.length);
    for (let i = 0; i < a.action.length; i++) {
      expect(Math.abs(a.action[i]! - b.action[i]!)).toBeLessThan(1e-5);
      expect(Math.abs(a.logp[i]! - b.logp[i]!)).toBeLessThan(1e-5);
      expect(Math.abs(a.value[i]! - b.value[i]!)).toBeLessThan(1e-5);
    }
    const changed = loaded.actWith(obs, 1, new RNG(1), true).action[0]!;
    const baseline = original.actWith(obs, 1, new RNG(1), true).action[0]!;
    expect(Math.abs(changed - baseline)).toBeGreaterThan(1e-3);

    const parsed = parseCheckpoint(serializeCheckpoint(checkpointFrom(loaded, 3)));
    expect(parsed.stage).toBe(3);
    expect(parsed.obsDim).toBe(4);
    expect(parsed.actDim).toBe(1);
    expect(parsed.hidden).toEqual([32, 32]);
    expect(parsed.weights).toHaveLength(13);
    const saved = checkpointFrom(loaded, 3).weights;
    parsed.weights.forEach((row, i) => row.forEach((v, j) => expect(Math.fround(v)).toBe(Math.fround(saved[i]![j]!))));
    const fromCkpt = new InferPolicy(4, 1, [32, 32], 3);
    fromCkpt.setWeights(buffersFromNumbers(parsed.weights));
    const c = fromCkpt.actWith(obs, 2, new RNG(5), false);
    for (let i = 0; i < a.action.length; i++) {
      expect(Math.abs(a.action[i]! - c.action[i]!)).toBeLessThan(1e-5);
    }
    expect(() => parseCheckpoint('not json')).toThrow(/invalid checkpoint/);
  });

  it('keeps inference weights aligned through the TF trainer', async () => {
    const policy = new InferPolicy(4, 1, [16, 16], 3);
    const trainer = new TfTrainer(policy, 1e-3);
    await trainer.ready();
    const obs = new Float32Array([0.2, -0.4, 0.5, 1]);
    const before = policy.actWith(obs, 1, new RNG(1), true);
    trainer.pullWeights();
    const after = policy.actWith(obs, 1, new RNG(1), true);
    expect(Math.abs(before.action[0]! - after.action[0]!)).toBeLessThan(1e-5);
    expect(Math.abs(before.value[0]! - after.value[0]!)).toBeLessThan(1e-5);

    const changed = policy.getWeights().map((buf) => buf.slice(0));
    new Float32Array(changed[5]!)[0] = 1.25;
    policy.setWeights(changed);
    const mid = policy.actWith(obs, 1, new RNG(1), true);
    trainer.pushWeights();
    trainer.pullWeights();
    const back = policy.actWith(obs, 1, new RNG(1), true);
    expect(Math.abs(mid.action[0]! - back.action[0]!)).toBeLessThan(1e-5);
    expect(Math.abs(mid.value[0]! - back.value[0]!)).toBeLessThan(1e-5);
  });

  it('returns null from IndexedDB when the platform has none', async () => {
    expect(globalThis.indexedDB).toBeUndefined();
    const policy = new InferPolicy(4, 1, [8, 8], 1);
    const ckpt = checkpointFrom(policy, 0);
    await expect(saveCheckpointIdb(ckpt)).resolves.toBeUndefined();
    await expect(loadCheckpointIdb()).resolves.toBeNull();
  });
});

describe('computeGae', () => {
  it('matches a 2-step hand calculation and bootstrap rules', () => {
    // gamma 0.99, lambda 0.95, rewards [1, 1], values [0.5, 0.5], nextValue 0.5, neither done.
    // delta_1 = 1 + 0.99*0.5 - 0.5 = 0.995
    // delta_0 = 0.995
    // gae_0 = 0.995 + 0.99*0.95*0.995 = 1.9307975
    const open = computeGae({
      rewards: new Float32Array([1, 1]),
      values: new Float32Array([0.5, 0.5]),
      dones: new Uint8Array([0, 0]),
      truncated: new Uint8Array([0, 0]),
      nextValue: 0.5,
    });
    expect(open.advantages[0]).toBeCloseTo(1.9307975, 5);
    expect(open.advantages[1]).toBeCloseTo(0.995, 5);
    expect(open.returns[0]).toBeCloseTo(2.4307975, 5);
    expect(open.returns[1]).toBeCloseTo(1.495, 5);

    // Terminal on the last step ignores nextValue.
    // delta_1 = 1, gae_0 = 0.99*0.95*1 = 0.9405
    const terminal = computeGae({
      rewards: new Float32Array([0, 1]),
      values: new Float32Array([0, 0]),
      dones: new Uint8Array([0, 1]),
      truncated: new Uint8Array([0, 0]),
      nextValue: 100,
    });
    expect(terminal.advantages[0]).toBeCloseTo(0.9405, 5);
    expect(terminal.advantages[1]).toBeCloseTo(1, 5);
    expect(terminal.returns[1]).toBeCloseTo(1, 5);

    // truncated=1 still bootstraps even if done was also set.
    // t=1: delta = 2 + 0.99*3 - 0.7 = 4.27
    // t=0: delta = 1 + 0.99*0.7 - 0.4 = 1.293; gae = 1.293 + 0.99*0.95*4.27 = 5.308935
    const trunc = computeGae({
      rewards: new Float32Array([1, 2]),
      values: new Float32Array([0.4, 0.7]),
      dones: new Uint8Array([1, 0]),
      truncated: new Uint8Array([1, 0]),
      nextValue: 3,
    });
    expect(trunc.advantages[1]).toBeCloseTo(4.27, 5);
    expect(trunc.advantages[0]).toBeCloseTo(5.308935, 5);
    expect(trunc.returns[0]).toBeCloseTo(5.708935, 5);
    expect(trunc.returns[1]).toBeCloseTo(4.97, 5);
  });
});

describe('computeGaeInterleaved', () => {
  it('runs computeGae on each env of each worker block separately', () => {
    // 2 worker blocks x 3 steps x 2 envs; row = block * 6 + t * 2 + env.
    const E = 2;
    const T = 3;
    const B = E * T;
    const n = 2 * B;
    const rng = new RNG(5, 'gae');
    const rewards = Float32Array.from({ length: n }, () => rng.range(-1, 1));
    const values = Float32Array.from({ length: n }, () => rng.range(-1, 1));
    const dones = new Uint8Array(n);
    const truncated = new Uint8Array(n);
    dones[1 * E + 1] = 1; // env 1 of block 0 ends at t = 1
    truncated[B + 0 * E + 0] = 1; // env 0 of block 1 is cut off at t = 0
    dones[B + 0 * E + 0] = 1;
    const got = computeGaeInterleaved({ rewards, values, dones, truncated }, E, B);
    for (let b = 0; b < 2; b++) {
      for (let i = 0; i < E; i++) {
        const rows = Array.from({ length: T }, (_, t) => b * B + t * E + i);
        const pick = <A extends Float32Array | Uint8Array>(a: A, make: (xs: number[]) => A) => make(rows.map((r) => a[r]!));
        const vals = pick(values, (xs) => Float32Array.from(xs));
        const want = computeGae({
          rewards: pick(rewards, (xs) => Float32Array.from(xs)),
          values: vals,
          dones: pick(dones, (xs) => Uint8Array.from(xs)),
          truncated: pick(truncated, (xs) => Uint8Array.from(xs)),
          nextValue: vals[T - 1]!,
        });
        rows.forEach((r, t) => {
          expect(got.advantages[r]).toBeCloseTo(want.advantages[t]!, 6);
          expect(got.returns[r]).toBeCloseTo(want.returns[t]!, 6);
        });
      }
    }
  });

  it('rejects a layout that does not divide the rows', () => {
    const z = new Float32Array(10);
    expect(() => computeGaeInterleaved({ rewards: z, values: z, dones: new Uint8Array(10), truncated: new Uint8Array(10) }, 4)).toThrow();
  });
});

describe('toy point-mass', () => {
  it('solves within 200 updates', async () => {
    const result = await trainToy();
    console.log('toy finalDist', result.finalDist, 'updates', result.updates, 'solved', result.solved);
    expect(result.updates).toBeLessThanOrEqual(200);
    expect(result.solved).toBe(true);
    expect(result.finalDist).toBeLessThan(0.12);
  });
});

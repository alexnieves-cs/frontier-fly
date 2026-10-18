import { describe, expect, it } from 'vitest';
import { OBS_DIM } from '../../src/shared/constants';
import { InferPolicy } from '../../src/rl/infer';
import { createGemm } from '../../src/rl/gemm';
import { createRolloutSab, readRollout, rolloutViews, writeRollout } from '../../src/workers/sab';
import { RNG } from '../../src/shared/rng';

describe('shared rollout buffer', () => {
  it('round-trips a rollout through a SharedArrayBuffer', () => {
    const capacity = 4;
    const sab = createRolloutSab(capacity);
    const view = rolloutViews(sab, capacity);
    const n = 3;
    writeRollout(view, {
      obs: new Float32Array(n * OBS_DIM).fill(0.25),
      actions: new Float32Array(n * 4).fill(-0.5),
      logp: new Float32Array(n).fill(0.1),
      values: new Float32Array(n).fill(1.5),
      rewards: new Float32Array(n).fill(2),
      dones: new Uint8Array(n).fill(0),
      truncated: new Uint8Array(n).fill(1),
      n,
    });
    const other = rolloutViews(sab, capacity);
    const back = readRollout(other);
    expect(back.n).toBe(3);
    expect(back.obs[0]).toBeCloseTo(0.25);
    expect(back.actions[1]).toBeCloseTo(-0.5);
    expect(back.values[2]).toBeCloseTo(1.5);
    expect(back.truncated[0]).toBe(1);
  });
});

describe('simd policy', () => {
  it('loads a kernel and is deterministic', () => {
    expect(createGemm()).not.toBeNull();
    const obs = new Float32Array(OBS_DIM);
    for (let i = 0; i < obs.length; i++) obs[i] = ((i * 17) % 100) / 100;
    const a = new InferPolicy(OBS_DIM, 4, [256, 128], 5);
    const b = new InferPolicy(OBS_DIM, 4, [256, 128], 5);
    const left = a.actWith(obs, 1, new RNG(3), true);
    const right = b.actWith(obs, 1, new RNG(3), true);
    for (let i = 0; i < 4; i++) expect(left.action[i]).toBeCloseTo(right.action[i]!, 4);
    expect(left.value[0]).toBeCloseTo(right.value[0]!, 4);
    expect(Number.isFinite(left.action[0])).toBe(true);
  });
});

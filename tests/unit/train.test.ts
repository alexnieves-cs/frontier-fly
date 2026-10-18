import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { OBS_DIM } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import { Curriculum } from '../../src/rl/curriculum';
import { InferPolicy } from '../../src/rl/infer';
import { RunningNorm } from '../../src/rl/normalize';
import { createEnv } from '../../src/sim/env';
import { collectRollout } from '../../src/workers/rollout';

describe('rollout', () => {
  it('collects a finite stage-0 rollout', async () => {
    const env = await createEnv();
    const curriculum = new Curriculum();
    const params = curriculum.genParams(0, new RNG(3, 'boot'));
    const spec = generateBuilding(params);
    const obs = env.reset(spec, params.seed);
    expect(obs.length).toBe(OBS_DIM);
    const outcome = await collectRollout({
      envs: [env],
      specs: [spec],
      obs: [obs],
      policy: new InferPolicy(OBS_DIM, 4, [256, 128], 3),
      norm: new RunningNorm(OBS_DIM),
      curriculum,
      steps: 48,
      baseSeed: 3,
      stageOverride: 0,
    });
    expect(outcome.buffer.size).toBe(48);
    expect(Number.isFinite(outcome.stepsPerSec)).toBe(true);
    expect(outcome.stepsPerSec).toBeGreaterThan(200);
    const arrays = outcome.buffer.toArrays();
    for (let i = 0; i < arrays.n; i++) {
      expect(Number.isFinite(arrays.rewards[i]!)).toBe(true);
      expect(Number.isFinite(arrays.values[i]!)).toBe(true);
    }
  });
});

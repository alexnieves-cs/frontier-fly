import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { Curriculum } from '../../src/rl/curriculum';
import { RNG } from '../../src/shared/rng';
import { createEnv } from '../../src/sim/env';
import { heuristicAction } from '../../src/sim/heuristic';

describe('heuristic pilot', () => {
  it('returns finite actions over a short flight', async () => {
    const env = await createEnv();
    const params = new Curriculum().genParams(0, new RNG(3, 'h'));
    const spec = generateBuilding(params);
    let obs = env.reset(spec, params.seed);
    let coverage = 0;
    for (let t = 0; t < 40; t++) {
      const action = heuristicAction(obs);
      expect(action.every((value) => Number.isFinite(value) && value >= -1 && value <= 1)).toBe(true);
      const step = env.step(action);
      obs = step.obs;
      coverage = step.info.coverage;
      if (step.done) break;
    }
    expect(coverage).toBeGreaterThanOrEqual(0);
  });
});

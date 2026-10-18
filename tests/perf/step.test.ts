import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { Curriculum } from '../../src/rl/curriculum';
import { RNG } from '../../src/shared/rng';
import { createEnv } from '../../src/sim/env';
import { heuristicAction } from '../../src/sim/heuristic';

describe('perf', () => {
  it('measures a kinematic env step', async () => {
    const env = await createEnv();
    const spec = generateBuilding({ seed: 8, floors: 1, w: 16, d: 16, stage: 2 });
    env.reset(spec, 8);
    const action = new Float32Array([0.4, 0, 0, 0.1]);
    const n = 400;
    for (let i = 0; i < 20; i++) env.step(action);
    const t0 = performance.now();
    for (let i = 0; i < n; i++) {
      const step = env.step(action);
      if (step.done) env.reset(spec, 20 + i);
    }
    const ms = (performance.now() - t0) / n;
    console.info(`kinematic step ${ms.toFixed(3)} ms (${(1000 / ms).toFixed(0)} steps/s)`);
    expect(ms).toBeLessThan(2);
  });

  it('measures a stage-3 env step under the lidar heuristic', async () => {
    const env = await createEnv();
    const params = new Curriculum().genParams(3, new RNG(4242, 'eval-3'));
    const spec = generateBuilding(params);
    let obs = env.reset(spec, params.seed);
    for (let i = 0; i < 50; i++) obs = env.step(heuristicAction(obs)).obs;
    const n = 1500;
    const t0 = performance.now();
    for (let i = 0; i < n; i++) {
      const step = env.step(heuristicAction(obs));
      obs = step.done ? env.reset(spec, 30 + i) : step.obs;
    }
    const ms = (performance.now() - t0) / n;
    console.info(`stage-3 step ${ms.toFixed(3)} ms (${(1000 / ms).toFixed(0)} steps/s), obs ${obs.length}`);
    expect(ms).toBeLessThan(2);
  });
});

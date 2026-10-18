import { describe, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { OBS_DIM } from '../../src/shared/constants';
import { InferPolicy } from '../../src/rl/infer';
import { createEnv } from '../../src/sim/env';
import { heuristicAction } from '../../src/sim/heuristic';

describe('bench', () => {
  it('infer', () => {
    const p = new InferPolicy(OBS_DIM, 4, [256, 128], 1);
    const obs = new Float32Array(8 * OBS_DIM);
    for (let i = 0; i < obs.length; i++) obs[i] = (i % 17) / 17;
    p.act(obs, 8);
    const n = 40;
    const t0 = performance.now();
    for (let i = 0; i < n; i++) p.act(obs, 8);
    const ms = (performance.now() - t0) / (n * 8);
    console.log(`infer ${ms.toFixed(3)} ms/env-step  ${(1000 / ms).toFixed(0)} steps/s`);
  });

  it('env+heuristic', async () => {
    const env = await createEnv();
    const spec = generateBuilding({ seed: 3, floors: 1, w: 10, d: 10, stage: 0 });
    let obs = env.reset(spec, 3);
    const n = 300;
    const t0 = performance.now();
    for (let i = 0; i < n; i++) {
      const step = env.step(heuristicAction(obs));
      obs = step.obs;
      if (step.done) obs = env.reset(spec, 4 + i);
    }
    const ms = (performance.now() - t0) / n;
    console.log(`env ${ms.toFixed(3)} ms  ${(1000 / ms).toFixed(0)} steps/s`);
  });
});

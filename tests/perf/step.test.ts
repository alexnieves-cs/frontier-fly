import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { createEnv } from '../../src/sim/env';

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
});
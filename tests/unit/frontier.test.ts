import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { Curriculum } from '../../src/rl/curriculum';
import { RNG } from '../../src/shared/rng';
import { createEnv, defaultEpisodeConfig } from '../../src/sim/env';
import { frontierAction } from '../../src/sim/frontier';

describe('frontier teacher', () => {
  it('covers a stage-0 room', async () => {
    const env = await createEnv();
    const params = new Curriculum().genParams(0, new RNG(4, 'front'));
    const spec = generateBuilding(params);
    const cfg = defaultEpisodeConfig(0, spec.floors[0]!.rooms.length);
    cfg.maxSteps = 1000;
    cfg.stallSteps = 500;
    cfg.maxCollisions = 8;
    let obs = env.reset(spec, params.seed, cfg);
    let coverage = 0;
    for (let t = 0; t < 1000; t++) {
      const pose = env.snapshot();
      const step = env.step(frontierAction(spec, env.visitedGrid, pose.pos, pose.yaw));
      obs = step.obs;
      coverage = step.info.coverage;
      if (step.done) break;
    }
    expect(obs.length).toBeGreaterThan(0);
    expect(coverage).toBeGreaterThan(0.8);
  });
});

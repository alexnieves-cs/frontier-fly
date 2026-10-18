import { describe, expect, it } from 'vitest';
import raw from '../fixtures/simpleRoom.json';
import { createEnv, defaultEpisodeConfig } from '../../src/sim/env';
import { actionFromKeys } from '../../src/sim/manual';
import { OBS } from '../../src/sim/sensors';
import { buildingFromJson } from '../../src/shared/serialize';
import { OBS_DIM } from '../../src/shared/constants';
import type { EpisodeConfig } from '../../src/shared/types';

const spec = buildingFromJson(raw);

function quietConfig(): EpisodeConfig {
  const cfg = defaultEpisodeConfig();
  cfg.maxSteps = 100_000;
  cfg.stallSteps = 100_000;
  return cfg;
}

describe('explorer env', () => {
  it('is deterministic for a seed and an action sequence', async () => {
    const run = async () => {
      const env = await createEnv();
      const obs0 = Array.from(env.reset(spec, 5, quietConfig()));
      const rewards: number[] = [];
      const last: number[][] = [];
      for (let i = 0; i < 40; i++) {
        const action = new Float32Array([Math.sin(i / 3) * 0.5, Math.cos(i / 5) * 0.3, 0.05, 0.2]);
        const step = env.step(action);
        rewards.push(step.reward);
        if (i % 10 === 0) last.push(Array.from(step.obs));
      }
      return { obs0, rewards, last, pos: env.snapshot().pos };
    };
    const a = await run();
    const b = await run();
    expect(a.obs0).toEqual(b.obs0);
    expect(a.rewards).toEqual(b.rewards);
    expect(a.last).toEqual(b.last);
    expect(a.pos).toEqual(b.pos);
    expect(a.obs0).toHaveLength(OBS_DIM);
  });

  it('penalizes flying into a wall and sees the wall on lidar', async () => {
    const env = await createEnv();
    const obs = env.reset(spec, 1, quietConfig());
    const horizontal = Array.from(obs.slice(OBS.lidar, OBS.lidar + 32));
    const close = horizontal.filter((range) => range < 0.75).length;
    expect(close).toBeGreaterThan(16);
    let minReward = 0;
    let hits = 0;
    const forward = new Float32Array([1, 0, 0, 0]);
    for (let i = 0; i < 100; i++) {
      const step = env.step(forward);
      minReward = Math.min(minReward, step.reward);
      hits = Math.max(hits, step.info.collisions);
    }
    expect(hits).toBeGreaterThan(0);
    expect(minReward).toBeLessThan(-4);
  });

  it('gives no coverage gain while hovering after the first step', async () => {
    const env = await createEnv();
    env.reset(spec, 2, quietConfig());
    const hover = new Float32Array(4);
    const first = env.step(hover);
    const second = env.step(hover);
    expect(second.info.coverage).toBe(first.info.coverage);
    expect(first.reward).toBeLessThan(0.5);
    expect(second.reward).toBeLessThan(0.5);
    expect(second.info.collisions).toBe(0);
  });

  it('steps in under 0.3 ms on average over 10k steps', async () => {
    const env = await createEnv();
    const cfg = quietConfig();
    env.reset(spec, 3, cfg);
    const hover = new Float32Array(4);
    for (let i = 0; i < 50; i++) env.step(hover);
    const n = 10_000;
    const t0 = performance.now();
    for (let i = 0; i < n; i++) env.step(hover);
    const ms = (performance.now() - t0) / n;
    expect(ms).toBeLessThan(0.3);
  });

  it('maps keyboard fly keys into body-frame actions', () => {
    const action = actionFromKeys(new Set(['KeyW', 'KeyA', 'Space', 'KeyQ']));
    expect(Array.from(action)).toEqual([1, 1, 1, -1]);
    const back = actionFromKeys(new Set(['KeyS', 'KeyD', 'ShiftLeft', 'KeyE']));
    expect(Array.from(back)).toEqual([-1, -1, -1, 1]);
  });
});

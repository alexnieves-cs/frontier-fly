import { describe, expect, it } from 'vitest';
import raw from '../fixtures/simpleRoom.json';
import { REWARD } from '../../src/shared/constants';
import { buildingFromJson } from '../../src/shared/serialize';
import type { EpisodeConfig } from '../../src/shared/types';
import { createEnv, defaultEpisodeConfig } from '../../src/sim/env';
import { ProgressTracker } from '../../src/sim/reward';

const spec = buildingFromJson(raw);

function quietConfig(): EpisodeConfig {
  const cfg = defaultEpisodeConfig();
  cfg.maxSteps = 100_000;
  cfg.stallSteps = 100_000;
  cfg.maxCollisions = 100_000;
  return cfg;
}

describe('progress tracker', () => {
  it('ramps the stuck penalty after the grace window and resets on a new voxel', () => {
    const t = new ProgressTracker();
    t.reset(1, 1);
    const hover = [0, 0, 0, 0];
    let p = t.update(1, 1, 1, hover);
    expect(p.stuck).toBe(0);
    for (let i = 0; i < REWARD.stuckGrace; i++) p = t.update(1, 1, 0, hover);
    expect(p.stuck).toBe(0);
    p = t.update(1, 1, 0, hover);
    expect(p.stuck).toBeGreaterThan(0);
    const early = p.stuck;
    for (let i = 0; i < REWARD.stuckRamp; i++) p = t.update(1, 1, 0, hover);
    expect(p.stuck).toBeGreaterThan(early);
    expect(p.stuck).toBeCloseTo(REWARD.stuck, 6);
    p = t.update(1, 1, 1, hover);
    expect(p.stuck).toBe(0);
    expect(p.still).toBe(0);
  });

  it('flags loitering only when not covering and the window is full', () => {
    const t = new ProgressTracker();
    t.reset(0, 0);
    let p = t.update(0.01, 0, 0, [0, 0, 0, 0]);
    expect(p.still).toBe(0);
    for (let i = 0; i < REWARD.stillWindow; i++) p = t.update(0.01 * (i % 2), 0, 0, [0, 0, 0, 0]);
    expect(p.still).toBe(REWARD.still);
    // A steady 0.1 m/step translation is real motion.
    const moving = new ProgressTracker();
    moving.reset(0, 0);
    for (let i = 1; i <= REWARD.stillWindow * 2; i++) p = moving.update(i * 0.1, 0, 0, [0.5, 0, 0, 0]);
    expect(p.still).toBe(0);
  });

  it('counts sign flips of strafe and yaw beyond the deadband', () => {
    const t = new ProgressTracker();
    t.reset(0, 0);
    let flips = 0;
    for (let i = 0; i < 20; i++) {
      const s = i % 2 === 0 ? 0.6 : -0.6;
      flips += t.update(i, 0, 1, [0.5, s, 0, -s]).flip;
    }
    expect(flips).toBeCloseTo(19 * 2 * REWARD.flip, 6);
    const smooth = new ProgressTracker();
    smooth.reset(0, 0);
    let none = 0;
    for (let i = 0; i < 20; i++) none += smooth.update(i, 0, 1, [0.5, 0.4, 0, 0.05 * (i % 2 ? 1 : -1)]).flip;
    expect(none).toBe(0);
  });
});

describe('progress penalties in the env', () => {
  it('stuck and loiter penalties trigger when pinned in a corner', async () => {
    const env = await createEnv();
    env.reset(spec, 1, quietConfig());
    // Drive diagonally into a corner and keep pushing.
    const push = new Float32Array([-1, -1, 0, 0]);
    let last = env.step(push);
    for (let i = 0; i < 300; i++) last = env.step(push);
    const pose = env.snapshot();
    expect(pose.pos[0]).toBeLessThan(1.5);
    expect(pose.pos[1]).toBeLessThan(1.5);
    expect(last.info.penalty).toBeDefined();
    expect(last.info.penalty!.stuck).toBeGreaterThan(0);
    expect(last.info.penalty!.still).toBe(REWARD.still);
    expect(last.info.penalty!.total).toBeCloseTo(last.info.penalty!.stuck + last.info.penalty!.still + last.info.penalty!.flip, 9);
    env.dispose();
  });

  it('oscillation penalty triggers on flip-flopping commands and lowers the reward', async () => {
    const run = async (flipFlop: boolean) => {
      const env = await createEnv();
      env.reset(spec, 4, quietConfig());
      let flip = 0;
      for (let i = 0; i < 40; i++) {
        const yaw = flipFlop ? (i % 2 === 0 ? 0.7 : -0.7) : 0.7;
        const step = env.step(new Float32Array([0, 0, 0, yaw]));
        flip += step.info.penalty!.flip;
      }
      env.dispose();
      return flip;
    };
    expect(await run(true)).toBeGreaterThan(30 * REWARD.flip);
    expect(await run(false)).toBe(0);
  });
});

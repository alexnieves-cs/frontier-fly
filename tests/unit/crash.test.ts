import { describe, expect, it } from 'vitest';
import raw from '../fixtures/simpleRoom.json';
import { CRASH_SPEED, MAX_SPEED_H, REWARD } from '../../src/shared/constants';
import { buildingFromJson } from '../../src/shared/serialize';
import type { EpisodeConfig, StepResult } from '../../src/shared/types';
import { createEnv, defaultEpisodeConfig } from '../../src/sim/env';
import { rawReward } from '../../src/sim/reward';

const spec = buildingFromJson(raw);

function config(crashSpeed?: number): EpisodeConfig {
  const cfg = defaultEpisodeConfig();
  cfg.maxSteps = 2000;
  cfg.stallSteps = 2000;
  cfg.maxCollisions = 1000;
  if (crashSpeed !== undefined) cfg.crashSpeed = crashSpeed;
  return cfg;
}

/** Fly straight along -x at a fraction of full speed until the episode ends or a contact starts. */
async function flyIntoWall(throttle: number, crashSpeed?: number): Promise<{ first: StepResult | null; last: StepResult }> {
  const env = await createEnv();
  env.reset(spec, 1, config(crashSpeed));
  const push = new Float32Array([-throttle, 0, 0, 0]);
  let first: StepResult | null = null;
  let last = env.step(push);
  for (let i = 0; i < 400 && !last.done; i++) {
    last = env.step(push);
    if (!first && last.info.impactSpeed > 0) first = last;
    if (first && i > 40) break;
  }
  env.dispose();
  return { first, last };
}

describe('crash rule', () => {
  it('is strict: a quarter of the env speed limit', () => {
    expect(CRASH_SPEED).toBe(0.5);
    expect(CRASH_SPEED).toBeLessThan(MAX_SPEED_H);
  });

  it('ends the flight on a fast impact and reports it', async () => {
    const { first } = await flyIntoWall(1);
    expect(first).not.toBeNull();
    expect(first!.info.impactSpeed).toBeGreaterThan(CRASH_SPEED);
    expect(first!.info.crashed).toBe(true);
    expect(first!.done).toBe(true);
    expect(first!.truncated).toBe(false);
  });

  it('lets a slow touch go on as a contact', async () => {
    const throttle = (0.4 * CRASH_SPEED) / MAX_SPEED_H;
    const { first, last } = await flyIntoWall(throttle);
    expect(first).not.toBeNull();
    expect(first!.info.impactSpeed).toBeGreaterThan(0);
    expect(first!.info.impactSpeed).toBeLessThanOrEqual(CRASH_SPEED);
    expect(first!.info.crashed).toBe(false);
    expect(last.done).toBe(false);
    expect(last.info.collisions).toBeGreaterThan(0);
  });

  it('counts a fast hit on a second wall while still touching the first', async () => {
    // Touch the -x wall gently, then slide along it at full speed into the side wall. With one
    // flag for "in contact" the second hit was neither a collision nor a crash.
    const env = await createEnv();
    env.reset(spec, 1, config());
    const slow = (0.4 * CRASH_SPEED) / MAX_SPEED_H;
    let last = env.step(new Float32Array([-slow, 0, 0, 0]));
    for (let i = 0; i < 400 && last.info.collisions === 0; i++) last = env.step(new Float32Array([-slow, 0, 0, 0]));
    expect(last.info.collisions).toBe(1);
    expect(last.info.crashed).toBe(false);
    const slide = new Float32Array([-slow, 1, 0, 0]);
    for (let i = 0; i < 400 && !last.done; i++) last = env.step(slide);
    env.dispose();
    expect(last.done).toBe(true);
    expect(last.info.crashed).toBe(true);
    expect(last.info.impactSpeed).toBeGreaterThan(CRASH_SPEED);
    expect(last.info.collisions).toBeGreaterThanOrEqual(2);
  });

  it('can be switched off per episode (FrontierBench v2 and the golden traces)', async () => {
    const { first } = await flyIntoWall(1, Infinity);
    expect(first!.info.impactSpeed).toBeGreaterThan(CRASH_SPEED);
    expect(first!.info.crashed).toBe(false);
    expect(first!.done).toBe(false);
  });

  it('costs more reward than a contact', () => {
    const base = { newVoxels: 0, newRoom: false, newFloor: false, inContact: false, action: [0, 0, 0, 0], prevAction: [0, 0, 0, 0], coverage: 0 };
    const touch = rawReward({ ...base, impact: true });
    const crash = rawReward({ ...base, impact: true, crash: true });
    expect(crash - touch).toBeCloseTo(REWARD.crash, 9);
    expect(REWARD.crash).toBeLessThan(REWARD.collision);
  });
});

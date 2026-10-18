import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { checkpointMismatch, parseCheckpoint } from '../../src/rl/checkpoint';
import { Curriculum } from '../../src/rl/curriculum';
import { InferPolicy } from '../../src/rl/infer';
import { ACT_DIM, HIDDEN, OBS_DIM } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import { createEnv, defaultEpisodeConfig } from '../../src/sim/env';
import { lightingPreset, type LightingPreset } from '../../src/sim/lighting';

async function cover(policy: InferPolicy, stage: number, seed: number, lighting: LightingPreset): Promise<number> {
  const env = await createEnv();
  const params = new Curriculum().genParams(stage, new RNG(seed, `eval-${stage}`));
  const spec = generateBuilding(params);
  const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
  const cfg = defaultEpisodeConfig(stage, nRooms);
  cfg.maxSteps = stage <= 1 ? 1000 : 2400;
  cfg.stallSteps = 500;
  cfg.maxCollisions = 16;
  cfg.dynamics = 'kinematic';
  cfg.lighting = lightingPreset(lighting, nRooms);
  let obs = env.reset(spec, params.seed, cfg);
  let coverage = 0;
  for (let t = 0; t < cfg.maxSteps; t++) {
    const action = policy.actWith(obs, 1, new RNG(1), true).action;
    if (action.length < ACT_DIM) throw new Error('action');
    const step = env.step(action);
    obs = step.obs;
    coverage = step.info.coverage;
    if (step.done) break;
  }
  return coverage;
}

const shipped = parseCheckpoint(readFileSync('public/checkpoints/pretrained.json', 'utf8'));
const mismatch = checkpointMismatch(shipped);

describe('shipped checkpoint', () => {
  it('is refused with a clear reason when its observation layout is stale', () => {
    if (!mismatch) return;
    expect(mismatch).toContain(`v${shipped.obsVersion}`);
    expect(shipped.obsDim).not.toBe(OBS_DIM);
  });

  // The shipped file predates observation v2 until a v2 checkpoint is trained and shipped.
  it.skipIf(mismatch !== null)(`reports stage 0, stage 3, and night versus day${mismatch ? ` (skipped: ${mismatch})` : ''}`, async () => {
    const checkpoint = shipped;
    expect(checkpoint.weights).toHaveLength(13);
    const policy = new InferPolicy(OBS_DIM, ACT_DIM, HIDDEN, 1);
    policy.setWeights(checkpoint.weights.map((row) => Float32Array.from(row).buffer));
    const stage0Seeds = [101, 202, 303, 404];
    const stage3Seeds = [4242, 5150, 6160];
    const stage0: number[] = [];
    const stage0Night: number[] = [];
    for (const seed of stage0Seeds) {
      stage0.push(await cover(policy, 0, seed, 'day'));
      stage0Night.push(await cover(policy, 0, seed, 'night-dark'));
    }
    const stage3: number[] = [];
    const stage3Night: number[] = [];
    for (const seed of stage3Seeds) {
      stage3.push(await cover(policy, 3, seed, 'day'));
      stage3Night.push(await cover(policy, 3, seed, 'night-dark'));
    }
    const mean = (xs: number[]) => xs.reduce((s, v) => s + v, 0) / xs.length;
    const day0 = mean(stage0);
    const night0 = mean(stage0Night);
    const day3 = mean(stage3);
    const night3 = mean(stage3Night);
    console.info(
      `shipped stage0 day ${stage0.map((v) => v.toFixed(3)).join(',')} mean ${day0.toFixed(3)} night ${night0.toFixed(3)} ratio ${(night0 / day0).toFixed(3)}`,
    );
    console.info(
      `shipped stage3 day ${stage3.map((v) => v.toFixed(3)).join(',')} mean ${day3.toFixed(3)} night ${night3.toFixed(3)} ratio ${(day3 > 0 ? night3 / day3 : 0).toFixed(3)}`,
    );
    expect(day0).toBeGreaterThan(0);
  }, 180_000);
});

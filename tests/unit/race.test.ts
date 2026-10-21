import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { parseCheckpoint, SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { InferPolicy } from '../../src/rl/infer';
import { OBS_DIM } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import { influenceFromActions, INPUT_BLOCKS, occlusionBatch } from '../../src/ui/inspector/influence';
import { RaceEngine } from '../../src/ui/race/engine';
import { DEFAULT_PILOTS, RACE_CONDITIONS } from '../../src/ui/race/pilots';
import { matchesStudy, studyEpisode } from '../../src/ui/race/study';
import { CONDITIONS } from '../../scripts/study/coverage-study';

function shippedPolicy() {
  const ck = parseCheckpoint(readFileSync(SHIPPED_CHECKPOINT_PATH));
  const policy = new InferPolicy(ck.obsDim, ck.actDim, ck.hidden, 1);
  policy.setWeights(ck.weights.map((row) => Float32Array.from(row).buffer));
  return policy;
}

describe('pilot race', () => {
  it('mirrors the study conditions', () => {
    expect(RACE_CONDITIONS.map(({ id, stage, seedBase, lighting, quad }) => ({ id, stage, seedBase, lighting, quad: quad ?? 0 }))).toEqual(
      CONDITIONS.map(({ id, stage, seedBase, lighting, quad }) => ({ id, stage, seedBase, lighting, quad: quad ?? 0 })),
    );
  });

  it('reproduces the published study episodes on a study building', async () => {
    const policy = shippedPolicy();
    const rng = new RNG(1);
    const setup = { condition: 'hard', seed: 30001, lighting: 'day' as const, pilots: [...DEFAULT_PILOTS] };
    const engine = await new RaceEngine(setup, (obs) => policy.actWith(obs, 1, rng, true).action.slice()).init();
    while (!engine.finished) engine.advance();
    for (const lane of engine.lanes) {
      const ref = studyEpisode('hard', 30001, 'day', lane.pilot);
      expect(ref, lane.pilot).not.toBeNull();
      const live = { coverage: lane.coverage, collisions: lane.collisions, steps: lane.steps };
      expect(matchesStudy(live, ref!), `${lane.pilot}: live ${JSON.stringify(live)} vs study ${JSON.stringify(ref)}`).toBe(true);
    }
    engine.dispose();
  }, 120_000);

  it('attributes the command to input blocks by blanking each one', () => {
    const policy = shippedPolicy();
    const obs = new Float32Array(OBS_DIM).fill(0.5);
    const batch = occlusionBatch(obs);
    expect(batch.length).toBe((INPUT_BLOCKS.length + 1) * OBS_DIM);
    // Row 0 is untouched; every other row differs from it only inside its own block.
    expect(Array.from(batch.subarray(0, OBS_DIM))).toEqual(Array.from(obs));
    const out = policy.actWith(batch, INPUT_BLOCKS.length + 1, new RNG(1), true).action;
    const infl = influenceFromActions(out);
    expect(infl.length).toBe(INPUT_BLOCKS.length);
    for (const v of infl) expect(v).toBeGreaterThanOrEqual(0);
    expect(Math.max(...infl)).toBeGreaterThan(0);
  });
});

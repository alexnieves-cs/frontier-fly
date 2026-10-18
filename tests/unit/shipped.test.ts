import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { evalCoverage, formatCoverageTable, mean, TARGETS } from '../../src/eval/coverage';
import { checkpointMismatch, parseCheckpoint } from '../../src/rl/checkpoint';
import { InferPolicy } from '../../src/rl/infer';
import { ACT_DIM, OBS_DIM } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';

const shipped = parseCheckpoint(readFileSync('public/checkpoints/pretrained.json', 'utf8'));
const mismatch = checkpointMismatch(shipped);

describe('shipped checkpoint', () => {
  it('is refused with a clear reason when its observation layout is stale', () => {
    if (!mismatch) return;
    expect(mismatch).toContain(`v${shipped.obsVersion}`);
    expect(shipped.obsDim).not.toBe(OBS_DIM);
  });

  // Same protocol as `npm run eval:coverage`. The sim and the policy are deterministic, so these are exact.
  it.skipIf(mismatch !== null)(`holds its reported coverage${mismatch ? ` (skipped: ${mismatch})` : ''}`, async () => {
    expect(shipped.weights).toHaveLength(13);
    const policy = new InferPolicy(OBS_DIM, ACT_DIM, shipped.hidden, 1);
    policy.setWeights(shipped.weights.map((row) => Float32Array.from(row).buffer));
    const rng = new RNG(1);
    const report = await evalCoverage({ act: (obs) => policy.actWith(obs, 1, rng, true).action });
    console.info(formatCoverageTable(report, 'public/checkpoints/pretrained.json'));
    const day0 = mean(report.stage0Day);
    const day3 = mean(report.stage3Day);
    expect(day0).toBeGreaterThanOrEqual(TARGETS.stage0);
    // Stage 3 is below the 60% target (README "Honest limits"); this floor only catches regressions.
    expect(day3).toBeGreaterThanOrEqual(0.55);
    expect(mean(report.stage0Night) / day0).toBeGreaterThanOrEqual(TARGETS.nightToDay);
    expect(mean(report.stage3Night) / day3).toBeGreaterThanOrEqual(TARGETS.nightToDay);
  }, 300_000);
});

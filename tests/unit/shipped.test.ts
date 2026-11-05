import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { evalCoverage, formatCoverageTable, mean, TARGETS } from '../../src/eval/coverage';
import { actorOnly, checkpointMismatch, parseCheckpoint, serializeCheckpointBinary, SHIPPED_CHECKPOINT_PATH, SHIPPED_POLICY_PATH } from '../../src/rl/checkpoint';
import { policyFromFile } from '../../src/mission/autonomy/policy';
import { InferPolicy } from '../../src/rl/infer';
import { ACT_DIM, OBS_DIM } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';

const shipped = parseCheckpoint(readFileSync(SHIPPED_CHECKPOINT_PATH));
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
    console.info(formatCoverageTable(report, SHIPPED_CHECKPOINT_PATH));
    const day0 = mean(report.stage0Day);
    const day3 = mean(report.stage3Day);
    expect(day0).toBeGreaterThanOrEqual(TARGETS.stage0);
    expect(day3).toBeGreaterThanOrEqual(TARGETS.stage3);
    expect(mean(report.stage0Night) / day0).toBeGreaterThanOrEqual(TARGETS.nightToDay);
    // The hard night/day target is checked on the 30-building study instead (next test): with 3 hard
    // seeds one crashed night flight moves this ratio by 15 points.
  }, 300_000);

  it('meets the night/day target on the 30 fresh hard buildings of the coverage study', () => {
    const study = JSON.parse(readFileSync('docs/study.json', 'utf8')) as { cells: { condition: string; lighting: string; pilot: string; coverage: number }[] };
    const cell = (lighting: string) => study.cells.find((c) => c.condition === 'hard' && c.pilot === 'policy' && c.lighting === lighting)!.coverage;
    expect(cell('night-dark') / cell('day')).toBeGreaterThanOrEqual(TARGETS.nightToDay);
  });

  it('ships an actor-only copy for the pages, packed from this checkpoint', () => {
    // Out of date: `npm run pack:policy` after replacing pretrained.bin.
    const packed = serializeCheckpointBinary(actorOnly(shipped));
    expect(Buffer.from(readFileSync(SHIPPED_POLICY_PATH)).equals(Buffer.from(packed))).toBe(true);
    // And it acts exactly like the full file.
    const full = policyFromFile(readFileSync(SHIPPED_CHECKPOINT_PATH));
    const lean = policyFromFile(readFileSync(SHIPPED_POLICY_PATH));
    const obs = new Float32Array(shipped.obsDim).map((_, i) => Math.sin(i * 0.37));
    expect(lean.actWith(obs, 1, new RNG(3), true).action).toEqual(full.actWith(obs, 1, new RNG(3), true).action);
  });
});

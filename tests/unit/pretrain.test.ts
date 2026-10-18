import { mkdirSync, writeFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { serializeCheckpoint } from '../../src/rl/checkpoint';
import { trainExplorer } from '../../src/rl/pretrain';

const updates = Number(process.env.PRETRAIN_UPDATES ?? '0');

describe.skipIf(!updates)('shipped checkpoint', () => {
  it('trains a stage-0 policy and writes public/checkpoints/pretrained.json', async () => {
    const result = await trainExplorer({
      updates,
      envs: Number(process.env.PRETRAIN_ENVS ?? '2'),
      steps: Number(process.env.PRETRAIN_STEPS ?? '96'),
      seed: 11,
      onUpdate: (info) => {
        console.info(
          `pretrain update ${info.update + 1} coverage ${info.meanCoverage.toFixed(3)} episodes ${info.episodes} ${info.stepsPerSec.toFixed(0)} steps/s`,
        );
      },
    });
    expect(Number.isFinite(result.meanCoverage)).toBe(true);
    expect(result.checkpoint.weights).toHaveLength(13);
    mkdirSync('public/checkpoints', { recursive: true });
    writeFileSync('public/checkpoints/pretrained.json', serializeCheckpoint(result.checkpoint));
  }, 600_000);
});

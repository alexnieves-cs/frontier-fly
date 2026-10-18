import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { parseCheckpoint, serializeCheckpoint } from '../../src/rl/checkpoint';
import { continueExplorer, distillExplorer } from '../../src/rl/pretrain';

const updates = Number(process.env.PRETRAIN_UPDATES ?? '0');

describe.skipIf(!updates)('shipped checkpoint', () => {
  it('trains a stage-0 policy and writes public/checkpoints/pretrained.json', async () => {
    const result = process.env.PRETRAIN_CONTINUE
      ? await continueExplorer(parseCheckpoint(readFileSync('public/checkpoints/pretrained.json', 'utf8')), updates)
      : await distillExplorer({
          epochs: updates,
          onEpoch: (epoch, loss) => {
            console.info(`distill epoch ${epoch + 1} mse ${loss.toFixed(4)}`);
          },
        });
    console.info(
      `coverage stage0 ${result.stage0.toFixed(3)} stage3 ${result.stage3.toFixed(3)} night/day ${result.nightToDay.toFixed(3)}`,
    );
    expect(Number.isFinite(result.stage0)).toBe(true);
    expect(result.checkpoint.weights).toHaveLength(13);
    mkdirSync('public/checkpoints', { recursive: true });
    writeFileSync('public/checkpoints/pretrained.json', serializeCheckpoint(result.checkpoint));
  }, 600_000);
});

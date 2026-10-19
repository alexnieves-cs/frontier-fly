import { availableParallelism } from 'node:os';
import { describe, expect, it } from 'vitest';
import { RolloutCore } from '../../src/train/rolloutCore';
import { NodeRolloutPool } from '../../src/train/nodePool';

const MIX = [1, 1, 1, 1, 1];

/** Full table: npx vite-node scripts/bench-throughput.ts */
describe('Node rollout throughput (policy inference + env)', () => {
  it('one thread: rebuild per episode vs WorldPool', async () => {
    const rates: Record<string, number> = {};
    for (const worlds of ['rebuild', 'pool'] as const) {
      const core = await RolloutCore.create({ nEnvs: 8, seed: 3, worldPool: { perStage: 8 } });
      core.collectPpo({ stepsPerEnv: 64, stageMix: MIX, seed: 1, worlds });
      const r = core.collectPpo({ stepsPerEnv: 256, stageMix: MIX, seed: 2, worlds });
      rates[worlds] = r.stepsPerSec;
      core.dispose();
    }
    console.log(`single thread 8 envs: rebuild ${rates.rebuild!.toFixed(0)} steps/s, pool ${rates.pool!.toFixed(0)} steps/s`);
    expect(rates.pool).toBeGreaterThan(300);
  });

  it('NodeRolloutPool across all cores', async () => {
    const workers = availableParallelism();
    const pool = new NodeRolloutPool({ workers, envsPerWorker: 8, seed: 3, worldPool: { perStage: 8, churn: 0.05 } });
    await pool.init();
    await pool.collectPpo({ stepsPerEnv: 64, stageMix: MIX, seed: 1 });
    const r = await pool.collectPpo({ stepsPerEnv: 384, stageMix: MIX, seed: 2 });
    await pool.close();
    console.log(
      `NodeRolloutPool ${workers}x8: ${r.stepsPerSec.toFixed(0)} steps/s total, per worker ${r.perWorker.map((w) => w.stepsPerSec.toFixed(0)).join(' / ')} (target 10000; shared CPUs make this noisy)`,
    );
    expect(r.n).toBe(workers * 8 * 384);
    expect(r.stepsPerSec).toBeGreaterThan(500);
  }, 180_000);
});

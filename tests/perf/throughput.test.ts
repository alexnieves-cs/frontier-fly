import { availableParallelism } from 'node:os';
import { mkdirSync } from 'node:fs';
import { Worker } from 'node:worker_threads';
import { describe, expect, it } from 'vitest';
import * as esbuild from 'esbuild';

describe('multi-worker rollout throughput', () => {
  it('sums env-steps/s across cores', async () => {
    mkdirSync('node_modules/.cache', { recursive: true });
    const outfile = 'node_modules/.cache/frontier-env-thread.mjs';
    await esbuild.build({
      entryPoints: ['tests/perf/envThread.ts'],
      bundle: true,
      platform: 'node',
      format: 'esm',
      outfile,
      external: ['@dimforge/rapier3d-compat'],
    });
    const cores = Math.max(1, Math.min(4, availableParallelism()));
    const heuristic = await Promise.all(
      Array.from({ length: cores }, (_, i) => spawn(outfile, { envs: 2, steps: 1000, seed: 40 + i * 17, mode: 'heuristic' })),
    );
    const policy = await Promise.all(
      Array.from({ length: cores }, (_, i) => spawn(outfile, { envs: 2, steps: 64, seed: 90 + i * 17, mode: 'policy' })),
    );
    const heuristicRate = heuristic.reduce((sum, row) => sum + row.stepsPerSec, 0);
    const policyRate = policy.reduce((sum, row) => sum + row.stepsPerSec, 0);
    console.log(
      `throughput workers=${cores} heuristic=${heuristicRate.toFixed(0)} steps/s policy=${policyRate.toFixed(0)} steps/s target=10000`,
    );
    expect(heuristic.reduce((sum, row) => sum + row.steps, 0)).toBe(cores * 2 * 1000);
    expect(heuristicRate).toBeGreaterThan(2_000 * cores);
  }, 120_000);
});

function spawn(
  file: string,
  workerData: { envs: number; steps: number; seed: number; mode: 'policy' | 'heuristic' },
): Promise<{ steps: number; stepsPerSec: number }> {
  return new Promise((resolve, reject) => {
    const worker = new Worker(new URL(`../../${file}`, import.meta.url), { workerData });
    worker.once('message', (msg: { steps: number; stepsPerSec: number }) => {
      void worker.terminate();
      resolve(msg);
    });
    worker.once('error', (error) => {
      void worker.terminate();
      reject(error);
    });
  });
}

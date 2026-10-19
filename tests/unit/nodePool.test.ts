import { afterAll, beforeAll, describe, expect, it } from 'vitest';
import { evalCoverage } from '../../src/eval/coverage';
import { evalCoverageParallel } from '../../src/eval/coverageParallel';
import { InferPolicy } from '../../src/rl/infer';
import { RunningNorm } from '../../src/rl/normalize';
import { ACT_DIM, HIDDEN, OBS_DIM } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import { mergeDaggerResults, type DaggerJobResult } from '../../src/train/daggerJob';
import { NodeRolloutPool } from '../../src/train/nodePool';

describe('NodeRolloutPool (worker_threads)', () => {
  const pool = new NodeRolloutPool({
    workers: 2,
    envsPerWorker: 3,
    seed: 5,
    worldPool: { perStage: 3 },
    jobModules: ['tests/support/fakeDaggerJobs.ts'],
  });
  const policy = new InferPolicy(OBS_DIM, ACT_DIM, HIDDEN, 21);

  beforeAll(async () => {
    await pool.init();
    await pool.setWeights(policy.getWeights());
  });
  afterAll(async () => {
    await pool.close();
  });

  it('collects env-major PPO arrays from every worker', async () => {
    const T = 40;
    const r = await pool.collectPpo({ stepsPerEnv: T, stageMix: [1, 1, 1, 1, 1], seed: 3 });
    expect(pool.envsTotal).toBe(6);
    expect(r.nEnvs).toBe(6);
    expect(r.n).toBe(6 * T);
    expect(r.obs.length).toBe(r.n * OBS_DIM);
    expect(r.actions.length).toBe(r.n * ACT_DIM);
    expect(r.bootstrap.length).toBe(6);
    expect(r.perWorker).toHaveLength(2);
    expect(r.obsMoments).toBeNull();
    for (const v of [r.logp, r.values, r.rewards, r.bootstrap]) expect(v.every(Number.isFinite)).toBe(true);
    for (const e of r.episodes) expect(e.env).toBeLessThan(6);
  });

  it('is deterministic for a seed', async () => {
    const other = new NodeRolloutPool({ workers: 2, envsPerWorker: 3, seed: 5, worldPool: { perStage: 3 } });
    await other.init();
    await other.setWeights(policy.getWeights());
    const opts = { stepsPerEnv: 30, stageMix: { 0: 1, 3: 1 }, seed: 11, resetEnvs: true };
    const a = await pool.collectPpo(opts);
    const b = await other.collectPpo(opts);
    await other.close();
    expect(a.obs).toEqual(b.obs);
    expect(a.rewards).toEqual(b.rewards);
    expect(a.actions).toEqual(b.actions);
  });

  it('normalizes with a provided norm and returns raw-obs moments to merge', async () => {
    const norm = new RunningNorm(OBS_DIM);
    await pool.setWeights(policy.getWeights(), norm.toJSON());
    const r = await pool.collectPpo({ stepsPerEnv: 10, seed: 4 });
    expect(r.obsMoments?.count).toBe(60);
    norm.merge(r.obsMoments!.count, r.obsMoments!.mean, r.obsMoments!.m2);
    expect(norm.toJSON().count).toBeGreaterThan(59);
    await pool.setWeights(policy.getWeights());
  });

  it('parallel coverage eval reproduces the sequential evalCoverage exactly', async () => {
    const rng = new RNG(1);
    const sequential = await evalCoverage({ act: (obs) => policy.actWith(obs, 1, rng, true).action });
    const parallel = await evalCoverageParallel(pool);
    expect(parallel).toEqual(sequential);
  });

  it('runs job modules (DAgger adapter) on every worker and reports job errors', async () => {
    const results = await pool.runJobAll<DaggerJobResult>('fakeDagger', (w: number) => ({
      seed: w,
      episodes: [
        { stage: 0, beta: 0.5, steps: 25 },
        { stage: 3, beta: 1, steps: 15, seed: 4, lighting: 'day' },
      ],
    }));
    const merged = mergeDaggerResults(results);
    const n = merged.extras.reduce((s, e) => s + (e.n as number), 0);
    expect(merged.extras).toHaveLength(4);
    expect(merged.extras.every((e) => e.pooledPhysics === true)).toBe(true);
    expect(merged.arrays.obs!.length).toBe(n * OBS_DIM);
    expect(merged.arrays.actions!.length).toBe(n * ACT_DIM);
    expect(merged.arrays.weights!.length).toBe(n);
    await expect(pool.runJob('boom', null)).rejects.toThrow(/boom job/);
    await expect(pool.runJob('nope', null)).rejects.toThrow(/unknown job/);
    // The worker survives a failed job.
    const ping = await pool.runJob<{ workerIndex: number; jobs: string[] }>('ping', null, 1);
    expect(ping.workerIndex).toBe(1);
    expect(ping.jobs).toContain('fakeDagger');
  });
});

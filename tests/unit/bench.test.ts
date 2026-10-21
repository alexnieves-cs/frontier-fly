import { beforeAll, describe, expect, it } from 'vitest';
import { BASELINES } from '../../src/bench/baselines';
import { runBenchEpisode } from '../../src/bench/run';
import { BENCH_PLAN_SEEDS, BENCH_SEEDS, BENCH_TASKS, benchRuns, benchSeeds } from '../../src/bench/tasks';
import { readFileSync } from 'node:fs';
import { parsePlanSet, PLAN_SET_PATH } from '../../src/plans';
import { createEnv } from '../../src/sim/env';
import { initPhysics } from '../../src/sim/world';
import example from '../../bench/entries/example/pilot';

beforeAll(async () => {
  await initPhysics();
});

describe('FrontierBench', () => {
  it('keeps its seed blocks apart from each other and from every reported block', () => {
    const all = BENCH_TASKS.flatMap((_, i) => [...benchSeeds('dev', i), ...benchSeeds('test', i)]);
    expect(new Set(all).size).toBe(all.length);
    for (const s of all) expect(s).toBeGreaterThanOrEqual(BENCH_SEEDS.dev.base);
    expect(Math.max(...all)).toBeLessThan(83001);
    expect(BENCH_PLAN_SEEDS.dev).toBeGreaterThan(Math.max(...all));
  });

  it('flies the real-plan task on the plan set', async () => {
    const set = parsePlanSet(JSON.parse(readFileSync(PLAN_SET_PATH, 'utf8')));
    const ti = BENCH_TASKS.findIndex((t) => t.plans);
    expect(benchRuns('test', ti, set)).toHaveLength(60);
    const run = benchRuns('dev', ti, set)[0]!;
    const env = await createEnv();
    const e = runBenchEpisode(env, BENCH_TASKS[ti]!, run, BASELINES.planner!());
    env.dispose();
    expect(e.plan).toBe(run.plan!.record.id);
    expect(e.coverage).toBeGreaterThan(0.2);
  });

  it('flies a baseline and an entry deterministically', async () => {
    const env = await createEnv();
    const task = BENCH_TASKS.find((t) => t.id === 'quad')!;
    const seed = benchSeeds('dev', BENCH_TASKS.indexOf(task))[0]!;
    const a = runBenchEpisode(env, task, { seed }, BASELINES.planner!());
    const b = runBenchEpisode(env, task, { seed }, BASELINES.planner!());
    expect(a).toEqual(b);
    expect(a.coverage).toBeGreaterThan(0.1);
    const room = BENCH_TASKS[0]!;
    const e = runBenchEpisode(env, room, { seed: benchSeeds('dev', 0)[0]! }, await example());
    expect(e.coverage).toBeGreaterThan(0.3);
    env.dispose();
  });
});

import { beforeAll, describe, expect, it } from 'vitest';
import { BASELINES } from '../../src/bench/baselines';
import { runBenchEpisode } from '../../src/bench/run';
import { BENCH_SEEDS, BENCH_TASKS, benchSeeds } from '../../src/bench/tasks';
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
  });

  it('flies a baseline and an entry deterministically', async () => {
    const env = await createEnv();
    const task = BENCH_TASKS.find((t) => t.id === 'quad')!;
    const seed = benchSeeds('dev', BENCH_TASKS.indexOf(task))[0]!;
    const a = runBenchEpisode(env, task, seed, BASELINES.planner!());
    const b = runBenchEpisode(env, task, seed, BASELINES.planner!());
    expect(a).toEqual(b);
    expect(a.coverage).toBeGreaterThan(0.1);
    const room = BENCH_TASKS[0]!;
    const e = runBenchEpisode(env, room, benchSeeds('dev', 0)[0]!, await example());
    expect(e.coverage).toBeGreaterThan(0.3);
    env.dispose();
  });
});

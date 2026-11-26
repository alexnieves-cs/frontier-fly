import { beforeAll, describe, expect, it } from 'vitest';
import { BASELINES } from '../../src/bench/baselines';
import { HARD_SEEDS, HARD_TASKS, hardRuns } from '../../src/bench/hard';
import { benchEpisodeSetup, runBenchEpisode } from '../../src/bench/run';
import { BENCH_SEEDS } from '../../src/bench/tasks';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { FootprintCoverage } from '../../src/eval/footprint';
import { createEnv } from '../../src/sim/env';
import { initPhysics } from '../../src/sim/world';

beforeAll(async () => {
  await initPhysics();
});

describe('FrontierBench Hard', () => {
  it('uses its own seed blocks, apart from each other and from v3', () => {
    const all = HARD_TASKS.flatMap((_, ti) => [...hardRuns('dev', ti), ...hardRuns('test', ti)].map((r) => r.seed));
    expect(new Set(all).size).toBe(all.length);
    for (const s of all) {
      expect(s >= BENCH_SEEDS.dev.base && s < BENCH_SEEDS.test.base + 700).toBe(false);
      expect(s).toBeGreaterThanOrEqual(HARD_SEEDS.dev.base);
    }
  });

  it('flies the 40 m building with the 120 s budget and scores a camera footprint inside the ring', async () => {
    const task = HARD_TASKS.find((t) => t.id === 'large')!;
    const { spec, cfg } = benchEpisodeSetup(task, { seed: 92001 });
    expect(spec.floors[0]!.w).toBe(40);
    expect(cfg.maxSteps).toBe(2400);
    const env = await createEnv();
    const e = runBenchEpisode(env, task, { seed: 92001 }, BASELINES['planner-view']!());
    expect(e.camera).toBeDefined();
    expect(e.camera!.footprint).toBeGreaterThan(0.1);
    // The 90° camera sees a subset of what the 360° ring sees.
    expect(e.camera!.footprint).toBeLessThanOrEqual(e.footprint + 1e-9);
    env.dispose();
  });

  it('the camera footprint only counts rays near the heading', async () => {
    const env = await createEnv();
    const { spec, envSeed, cfg } = evalEpisodeSetup(3, 9101, 'day');
    env.reset(spec, envSeed, cfg);
    const p = env.position;
    const ring = new FootprintCoverage([3]);
    const cam = new FootprintCoverage([3], Math.PI / 2);
    ring.reset(spec);
    cam.reset(spec);
    ring.observe(env.world!, p[0]!, p[1]!, p[2]!);
    cam.observe(env.world!, p[0]!, p[1]!, p[2]!, 0);
    expect(cam.coverage(0)).toBeGreaterThan(0);
    expect(cam.coverage(0)).toBeLessThan(ring.coverage(0));
    env.dispose();
  });
});

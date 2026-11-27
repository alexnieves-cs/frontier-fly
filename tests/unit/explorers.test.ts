import { beforeAll, describe, expect, it } from 'vitest';
import { BASELINES } from '../../src/bench/baselines';
import { runBenchEpisode } from '../../src/bench/run';
import { BENCH_TASKS, benchSeeds } from '../../src/bench/tasks';
import { createEnv } from '../../src/sim/env';
import { initPhysics } from '../../src/sim/world';

beforeAll(async () => {
  await initPhysics();
});

describe('published exploration baselines', () => {
  it('fly deterministically, sensor-only, and see the room they are in', async () => {
    const env = await createEnv();
    const room = BENCH_TASKS[0]!;
    const seed = benchSeeds('dev', 0)[0]!;
    for (const id of ['yamauchi', 'nbvp', 'boustrophedon']) {
      const a = runBenchEpisode(env, room, { seed }, BASELINES[id]!());
      const b = runBenchEpisode(env, room, { seed }, BASELINES[id]!());
      expect(a, id).toEqual(b);
      // Mapping methods stop once the room is mapped; all of them should have looked at most of it.
      expect(a.footprint, id).toBeGreaterThan(0.6);
      expect(a.collisions, id).toBe(0);
    }
    // Boustrophedon lanes are one voxel apart, so it flies through most of the room.
    expect(runBenchEpisode(env, room, { seed }, BASELINES.boustrophedon!()).coverage).toBeGreaterThan(0.8);
    env.dispose();
  });

  it('add greedy information gain, rapid frontier and a random-walk floor, all deterministic', async () => {
    const env = await createEnv();
    const room = BENCH_TASKS[0]!;
    const seed = benchSeeds('dev', 0)[0]!;
    for (const id of ['info-gain', 'rapid-frontier']) {
      const a = runBenchEpisode(env, room, { seed }, BASELINES[id]!());
      expect(runBenchEpisode(env, room, { seed }, BASELINES[id]!()), id).toEqual(a);
      expect(a.footprint, id).toBeGreaterThan(0.6);
      expect(a.collisions, id).toBe(0);
    }
    // The floor keeps no map: it is deterministic (its random stream restarts on reset) and sees something.
    const walk = runBenchEpisode(env, room, { seed }, BASELINES['random-walk']!());
    expect(runBenchEpisode(env, room, { seed }, BASELINES['random-walk']!())).toEqual(walk);
    expect(walk.footprint).toBeGreaterThan(0.1);
    env.dispose();
  });

  it('take shafts on multi-floor buildings through the climber', async () => {
    const env = await createEnv();
    const ti = BENCH_TASKS.findIndex((t) => t.id === 'floors');
    const e = runBenchEpisode(env, BENCH_TASKS[ti]!, { seed: benchSeeds('dev', ti)[0]! }, BASELINES.boustrophedon!());
    expect(e.coverage).toBeGreaterThan(0.2);
    env.dispose();
  });
});

describe('view-first planner', () => {
  it('looks into the building sooner than the sweeping planner and stays deterministic', async () => {
    const env = await createEnv();
    const ti = BENCH_TASKS.findIndex((t) => t.id === 'hard');
    const run = { seed: benchSeeds('dev', ti)[0]! };
    const view = runBenchEpisode(env, BENCH_TASKS[ti]!, run, BASELINES['planner-view']!());
    expect(runBenchEpisode(env, BENCH_TASKS[ti]!, run, BASELINES['planner-view']!())).toEqual(view);
    const sweep = runBenchEpisode(env, BENCH_TASKS[ti]!, run, BASELINES['planner-tracker']!());
    // Same budget: the view phase sees more of the building early, then sweeps with the time left.
    expect(view.area.footprint).toBeGreaterThan(sweep.area.footprint);
    expect(view.footprint).toBeGreaterThanOrEqual(sweep.footprint);
    expect(view.crashed).toBe(false);
    env.dispose();
  });
});

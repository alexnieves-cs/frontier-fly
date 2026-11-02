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

  it('take shafts on multi-floor buildings through the climber', async () => {
    const env = await createEnv();
    const ti = BENCH_TASKS.findIndex((t) => t.id === 'floors');
    const e = runBenchEpisode(env, BENCH_TASKS[ti]!, { seed: benchSeeds('dev', ti)[0]! }, BASELINES.boustrophedon!());
    expect(e.coverage).toBeGreaterThan(0.2);
    env.dispose();
  });
});

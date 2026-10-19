import { describe, expect, it } from 'vitest';
import { STAGE0_SEEDS, STAGE3_SEEDS } from '../../src/eval/coverage';
import { generateBuilding } from '../../src/gen/building';
import { Curriculum } from '../../src/rl/curriculum';
import { RNG } from '../../src/shared/rng';
import { defaultEpisodeConfig, ExplorerEnv } from '../../src/sim/env';
import { PhysicsWorld } from '../../src/sim/world';
import { evalBuildingKeys, WorldPool } from '../../src/sim/worldPool';

describe('WorldPool', () => {
  it('pre-generates bounded, diverse worlds per stage and never an eval building', async () => {
    const pool = await WorldPool.create({ perStage: { 0: 30, 1: 4, 2: 4, 3: 30, 4: 4 }, seed: 3 });
    expect(pool.size).toBe(72);
    expect(pool.stages).toEqual([0, 1, 2, 3, 4]);
    expect(evalBuildingKeys().size).toBe(STAGE0_SEEDS.length + STAGE3_SEEDS.length);
    const curriculum = new Curriculum();
    const evalSeeds = new Set<number>();
    for (const seed of STAGE0_SEEDS) evalSeeds.add(generateBuilding(curriculum.genParams(0, new RNG(seed, 'eval-0'))).seed);
    for (const seed of STAGE3_SEEDS) evalSeeds.add(generateBuilding(curriculum.genParams(3, new RNG(seed, 'eval-3'))).seed);
    const rng = new RNG(1);
    const seen = new Set<number>();
    for (let i = 0; i < 400; i++) {
      const stage = i % 5;
      const world = pool.acquire(stage, rng);
      expect(world.stage).toBe(stage);
      expect(world.physics.spec).toBe(world.spec);
      if (stage === 0 || stage === 3) expect(evalSeeds.has(world.spec.seed)).toBe(false);
      seen.add(world.spec.seed);
      pool.release(world);
    }
    expect(seen.size).toBeGreaterThan(50);
    expect(pool.size).toBe(72);
    pool.free();
  });

  it('churn replaces worlds but frees one only after its last user releases it', async () => {
    const pool = await WorldPool.create({ stages: [0], perStage: 2, churn: 1, seed: 9 });
    const rng = new RNG(2);
    const held = pool.acquire(0, rng);
    for (let i = 0; i < 6; i++) pool.release(pool.acquire(0, rng));
    // `held` was churned out of the pool by now but is still alive for its user.
    expect(() => held.physics.cast(1.5, 1.5, 1, 1, 0, 0, 8)).not.toThrow();
    pool.release(held);
    expect(() => held.physics.cast(1.5, 1.5, 1, 1, 0, 0, 8)).toThrow();
    expect(pool.size).toBe(2);
    expect(pool.builds).toBeGreaterThanOrEqual(8);
    pool.free();
  });

  it('bySeed caches by GenParams with an LRU bound', async () => {
    const pool = await WorldPool.create({ stages: [], maxCached: 2 });
    const params = (seed: number) => new Curriculum().genParams(0, new RNG(seed, 'x'));
    const a = pool.bySeed(params(1));
    expect(pool.bySeed(params(1))).toBe(a);
    pool.release(a);
    pool.release(a);
    pool.release(pool.bySeed(params(2)));
    pool.release(pool.bySeed(params(3)));
    expect(pool.bySeed(params(1))).not.toBe(a);
    pool.free();
  });

  it('an env borrows a pooled world: dispose and reset leave it alive', async () => {
    const pool = await WorldPool.create({ stages: [0], perStage: 1 });
    const world = pool.acquire(0, new RNG(1));
    const env = new ExplorerEnv();
    env.reset(world.spec, 4, defaultEpisodeConfig(), world.physics);
    expect(env.physicsWorld).toBe(world.physics);
    env.step(new Float32Array([0.5, 0, 0, 0]));
    env.reset(world.spec, 5, defaultEpisodeConfig(), world.physics);
    env.dispose();
    expect(() => world.physics.cast(1.5, 1.5, 1, 1, 0, 0, 8)).not.toThrow();
    // A mismatched world is rejected instead of silently using the wrong colliders.
    const other = generateBuilding(new Curriculum().genParams(0, new RNG(77, 'other')));
    expect(() => env.reset(other, 1, undefined, world.physics)).toThrow(/different spec/);
    // Without a world the env builds and owns one, as before.
    env.reset(other, 1);
    expect(env.physicsWorld).toBeInstanceOf(PhysicsWorld);
    expect(env.physicsWorld).not.toBe(world.physics);
    env.dispose();
    pool.free();
  });
});

import { beforeAll, describe, expect, it } from 'vitest';
import { BENCH_PLAN_SEEDS, BENCH_TASKS, benchSeeds } from '../../src/bench/tasks';
import { runTeamBenchEpisode, TEAM_BASELINES, TEAM_BENCH_TASKS, teamBenchSeeds, teamOptions } from '../../src/bench/team';
import { initPhysics } from '../../src/sim/world';

beforeAll(async () => {
  await initPhysics();
});

describe('FrontierBench team track', () => {
  it('keeps its seeds inside 83001-83999, apart from the single-drone track', () => {
    const team = TEAM_BENCH_TASKS.flatMap((t) => [...teamBenchSeeds('dev', t), ...teamBenchSeeds('test', t)]);
    expect(new Set(team).size).toBe(team.length);
    expect(Math.min(...team)).toBeGreaterThanOrEqual(83001);
    expect(Math.max(...team)).toBeLessThan(BENCH_PLAN_SEEDS.dev);
    const single = new Set(BENCH_TASKS.flatMap((_, i) => [...benchSeeds('dev', i), ...benchSeeds('test', i)]));
    for (const s of team) expect(single.has(s)).toBe(false);
  });

  it('holds entries to the rules', () => {
    expect(() => teamOptions({ name: 'x', drones: 5 })).toThrow();
    expect(() => teamOptions({ name: 'x', drones: 2, speed: 3 })).toThrow();
    expect(() => teamOptions({ name: 'x', drones: 1, roles: ['explore', 'relay'] })).toThrow();
    const o = teamOptions({ name: 'x', drones: 3, claims: { goalWeight: 20 }, speed: 1.5 });
    expect(o.comms).toBe('radio');
    expect(o.claims?.goalWeight).toBe(20);
    expect(o.claims?.nearerWeight).toBeGreaterThan(0);
  });

  it('flies a team deterministically', async () => {
    const task = TEAM_BENCH_TASKS.find((t) => t.id === 'hard')!;
    const seed = teamBenchSeeds('dev', task)[0]!;
    const a = await runTeamBenchEpisode(TEAM_BASELINES.team2!, task, seed);
    const b = await runTeamBenchEpisode(TEAM_BASELINES.team2!, task, seed);
    expect(a).toEqual(b);
    expect(a.coverage).toBeGreaterThan(0.3);
  });
});

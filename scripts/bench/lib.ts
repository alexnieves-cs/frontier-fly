// Shared by the bench scripts: loading a pilot and summarizing its flights.
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { BASELINES, policyPilot } from '../../src/bench/baselines';
import type { BenchPilot, BenchPilotFactory } from '../../src/bench/pilot';
import { readFileSync } from 'node:fs';
import { runBenchEpisode, type BenchEpisode } from '../../src/bench/run';
import { parsePlanSet, PLAN_SET_PATH, type PlanSet } from '../../src/plans';
import type { ExplorerEnv } from '../../src/sim/env';
import { BENCH_TASKS, benchRuns, SAFE_COLLISIONS, type BenchSplit } from '../../src/bench/tasks';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { bootstrapCi, loadPolicy } from '../study/common';

export async function loadPilot(id: string): Promise<{ make: () => Promise<BenchPilot>; source: string }> {
  if (id === 'policy' || id === 'guarded') {
    const policy = loadPolicy(SHIPPED_CHECKPOINT_PATH);
    const name = id === 'policy' ? 'Shipped policy (DAgger + PPO)' : 'Shipped policy (DAgger + PPO) under the lidar guard';
    return { make: async () => policyPilot(name, policy, id === 'guarded'), source: `baseline:${id}` };
  }
  if (BASELINES[id]) return { make: async () => BASELINES[id]!(), source: `baseline:${id}` };
  const mod = (await import(pathToFileURL(resolve(id)).href)) as { default: BenchPilotFactory };
  return { make: async () => mod.default(), source: id };
}

export function summarize(episodes: BenchEpisode[]) {
  const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
  const tasks: Record<string, unknown> = {};
  const covs: number[] = [];
  const fps: number[] = [];
  for (const t of BENCH_TASKS) {
    const rs = episodes.filter((e) => e.task === t.id);
    if (!rs.length) continue;
    const cov = rs.map((r) => r.coverage);
    covs.push(mean(cov));
    const half = rs.filter((r) => r.halfS !== null).map((r) => r.halfS!);
    tasks[t.id] = {
      episodes: rs.length,
      coverage: mean(cov),
      coverageCi: bootstrapCi(cov),
      collisionsPerEpisode: mean(rs.map((r) => r.collisions)),
      safeFlights: rs.filter((r) => r.collisions <= SAFE_COLLISIONS).length,
      energyWh: mean(rs.map((r) => r.energyWh)),
      halfS: half.length ? mean(half) : null,
      ...footprintStats(rs, mean),
    };
    if (rs.every((r) => r.footprint !== undefined)) fps.push(mean(rs.map((r) => r.footprint)));
  }
  return { score: mean(covs), ...(fps.length === covs.length ? { footprintScore: mean(fps) } : {}), safeShare: episodes.filter((e) => e.collisions <= SAFE_COLLISIONS).length / Math.max(1, episodes.length), tasks };
}

/** Footprint columns of a task (absent on results written before the footprint metric existed). */
function footprintStats(rs: BenchEpisode[], mean: (xs: number[]) => number) {
  if (!rs.every((r) => r.footprint !== undefined)) return {};
  const fp = rs.map((r) => r.footprint);
  const half = rs.filter((r) => r.footprintHalfS !== null).map((r) => r.footprintHalfS!);
  return {
    footprint: mean(fp),
    footprintCi: bootstrapCi(fp),
    area: { flown: mean(rs.map((r) => r.area.flown)), footprint: mean(rs.map((r) => r.area.footprint)) },
    footprintHalfS: half.length ? mean(half) : null,
  };
}

let planSet: PlanSet | null = null;
export function loadPlanSet(): PlanSet {
  planSet ??= parsePlanSet(JSON.parse(readFileSync(PLAN_SET_PATH, 'utf8')));
  return planSet;
}

/** Fly every task of a split (or the listed ones) with one pilot. */
export function flyAll(env: ExplorerEnv, pilot: BenchPilot, split: BenchSplit, only: string[] = [], log?: (task: string, eps: BenchEpisode[]) => void): BenchEpisode[] {
  const episodes: BenchEpisode[] = [];
  BENCH_TASKS.forEach((task, ti) => {
    if (only.length && !only.includes(task.id)) return;
    const mine: BenchEpisode[] = [];
    for (const run of benchRuns(split, ti, task.plans ? loadPlanSet() : undefined)) mine.push(runBenchEpisode(env, task, run, pilot));
    episodes.push(...mine);
    log?.(task.id, mine);
  });
  return episodes;
}

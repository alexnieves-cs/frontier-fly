// Shared by the bench scripts: loading a pilot and summarizing its flights.
import { resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { BASELINES, policyPilot } from '../../src/bench/baselines';
import type { BenchPilot, BenchPilotFactory } from '../../src/bench/pilot';
import type { BenchEpisode } from '../../src/bench/run';
import { BENCH_TASKS, SAFE_COLLISIONS } from '../../src/bench/tasks';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { bootstrapCi, loadPolicy } from '../study/common';

export async function loadPilot(id: string): Promise<{ make: () => Promise<BenchPilot>; source: string }> {
  if (id === 'policy' || id === 'guarded') {
    const policy = loadPolicy(SHIPPED_CHECKPOINT_PATH);
    const name = id === 'policy' ? 'Shipped policy (DAgger)' : 'Shipped policy under the lidar guard';
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
    };
  }
  return { score: mean(covs), safeShare: episodes.filter((e) => e.collisions <= SAFE_COLLISIONS).length / Math.max(1, episodes.length), tasks };
}

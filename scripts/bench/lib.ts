// Shared by the bench scripts: loading a pilot and summarizing its flights.
import { BASELINES, policyPilot } from '../../src/bench/baselines';
import type { BenchPilot } from '../../src/bench/pilot';
import { readFileSync } from 'node:fs';
import { runBenchEpisode, type BenchEpisode } from '../../src/bench/run';
import { parsePlanSet, PLAN_SET_PATH, type PlanSet } from '../../src/plans';
import type { ExplorerEnv } from '../../src/sim/env';
import { BENCH_TASKS, benchRuns, SAFE_COLLISIONS, type BenchRun, type BenchSplit, type BenchTask } from '../../src/bench/tasks';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { bootstrapCi, loadPolicy } from '../study/common';
import { sandboxedPilot } from './sandbox';

/** A baseline by id, or a submitted pilot module, which runs in the sandbox (sandbox.ts). Call `close` when done. */
export async function loadPilot(id: string): Promise<{ make: () => Promise<BenchPilot & { close?: () => Promise<void> }>; source: string }> {
  if (id === 'policy' || id === 'guarded') {
    const policy = loadPolicy(SHIPPED_CHECKPOINT_PATH);
    const name = id === 'policy' ? 'Shipped policy (DAgger + PPO)' : 'Shipped policy (DAgger + PPO) under the lidar guard';
    return { make: async () => policyPilot(name, policy, id === 'guarded'), source: `baseline:${id}` };
  }
  if (BASELINES[id]) return { make: async () => BASELINES[id]!(), source: `baseline:${id}` };
  return { make: () => sandboxedPilot(id), source: id };
}

export function summarize(episodes: BenchEpisode[], taskList: readonly BenchTask[] = BENCH_TASKS) {
  const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
  const tasks: Record<string, unknown> = {};
  const covs: number[] = [];
  const fps: number[] = [];
  const cams: number[] = [];
  const watches: number[] = [];
  const integrities: number[] = [];
  for (const t of taskList) {
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
      crashedFlights: rs.filter((r) => r.crashed).length,
      energyWh: mean(rs.map((r) => r.energyWh)),
      halfS: half.length ? mean(half) : null,
      ...footprintStats(rs, mean),
      ...cameraStats(rs, mean),
      ...watchStats(rs, mean),
      ...integrityStats(rs, mean),
    };
    if (rs.every((r) => r.watch)) watches.push(mean(rs.map((r) => r.watch!.score)));
    if (rs.every((r) => r.integrity)) integrities.push(mean(rs.map((r) => r.integrity!.score)));
    if (rs.every((r) => r.camera)) cams.push(mean(rs.map((r) => r.camera!.footprint)));
    if (rs.every((r) => r.footprint !== undefined)) fps.push(mean(rs.map((r) => r.footprint)));
  }
  return { score: mean(covs), ...(fps.length === covs.length ? { footprintScore: mean(fps) } : {}), ...(cams.length && cams.length === covs.length ? { cameraScore: mean(cams) } : {}), ...(watches.length && watches.length === covs.length ? { watchScore: mean(watches) } : {}), ...(integrities.length && integrities.length === covs.length ? { integrityScore: mean(integrities) } : {}), safeShare: episodes.filter((e) => e.collisions <= SAFE_COLLISIONS).length / Math.max(1, episodes.length), crashShare: episodes.filter((e) => e.crashed).length / Math.max(1, episodes.length), tasks };
}

/** Camera-footprint columns of a FrontierBench Hard task (absent elsewhere). */
function cameraStats(rs: BenchEpisode[], mean: (xs: number[]) => number) {
  if (!rs.length || !rs.every((r) => r.camera)) return {};
  const c = rs.map((r) => r.camera!.footprint);
  const half = rs.filter((r) => r.camera!.halfS !== null).map((r) => r.camera!.halfS!);
  return { camera: mean(c), cameraCi: bootstrapCi(c), cameraArea: mean(rs.map((r) => r.camera!.area)), cameraHalfS: half.length ? mean(half) : null };
}

/** Overwatch columns of a task (bench/overwatch.ts; absent elsewhere). */
function watchStats(rs: BenchEpisode[], mean: (xs: number[]) => number) {
  if (!rs.length || !rs.every((r) => r.watch)) return {};
  const w = rs.map((r) => r.watch!);
  const found = w.filter((x) => x.found);
  const re = w.filter((x) => x.reacquireS !== null).map((x) => x.reacquireS!);
  return {
    watch: mean(w.map((x) => x.score)),
    watchCi: bootstrapCi(w.map((x) => x.score)),
    found: found.length,
    findS: found.length ? mean(found.map((x) => x.findS!)) : null,
    kept: found.length ? mean(found.map((x) => x.kept)) : null,
    reacquireS: re.length ? mean(re) : null,
    breachS: mean(w.map((x) => x.breachS)),
  };
}

/** Integrity columns of a task (bench/integrity.ts; absent elsewhere). */
function integrityStats(rs: BenchEpisode[], mean: (xs: number[]) => number) {
  if (!rs.length || !rs.every((r) => r.integrity)) return {};
  const g = rs.map((r) => r.integrity!);
  return {
    integrity: mean(g.map((x) => x.score)),
    integrityCi: bootstrapCi(g.map((x) => x.score)),
    availability: mean(g.map((x) => x.availability)),
    hmiRate: mean(g.map((x) => x.hmiRate)),
    badShare: mean(g.map((x) => x.badShare)),
    // Null on a flight with no bad-pose step: the mean is over the flights that had one.
    caught: ((k) => (k.length ? mean(k) : null))(g.map((x) => x.caught).filter((x): x is number => x !== null)),
  };
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

/**
 * A seed block outside the bench's own splits, for studies that need fresh flights (a practice or
 * report-only block recorded in the seed ledger): generated task i flies base + 100 i .. + count - 1,
 * the real-plan task flies `plans` split's plans with env seeds base + 700 + j.
 */
export interface SeedBlock {
  base: number;
  count: number;
  plans: 'dev' | 'eval';
}

function blockRuns(block: SeedBlock, ti: number): BenchRun[] {
  const task = BENCH_TASKS[ti]!;
  if (!task.plans) return Array.from({ length: block.count }, (_, i) => ({ seed: block.base + ti * 100 + i }));
  const set = loadPlanSet();
  return set.plans.filter((p) => p.split === block.plans).map((record, j) => ({ seed: block.base + 700 + j, plan: { set, record } }));
}

/** Fly every task of a split (or the listed ones) with one pilot; with `block`, fly that seed block instead. */
export function flyAll(env: ExplorerEnv, pilot: BenchPilot, split: BenchSplit, only: string[] = [], log?: (task: string, eps: BenchEpisode[]) => void, block?: SeedBlock): BenchEpisode[] {
  const episodes: BenchEpisode[] = [];
  BENCH_TASKS.forEach((task, ti) => {
    if (only.length && !only.includes(task.id)) return;
    const mine: BenchEpisode[] = [];
    const runs = block ? blockRuns(block, ti) : benchRuns(split, ti, task.plans ? loadPlanSet() : undefined);
    for (const run of runs) mine.push(runBenchEpisode(env, task, run, pilot));
    episodes.push(...mine);
    log?.(task.id, mine);
  });
  return episodes;
}

/** Paths where a re-scored summary and a claimed one differ: numbers to 1e-6, everything else exactly. */
export function mismatches(got: unknown, claimed: unknown, path = ''): string[] {
  if (typeof got === 'number') return typeof claimed === 'number' && Math.abs(got - claimed) < 1e-6 ? [] : [path];
  if (got === null || typeof got !== 'object') return got === claimed ? [] : [path];
  if (claimed === null || typeof claimed !== 'object') return [path];
  return Object.entries(got).flatMap(([k, v]) => mismatches(v, (claimed as Record<string, unknown>)[k], path ? `${path}.${k}` : k));
}

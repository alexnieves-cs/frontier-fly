/**
 * FrontierBench team track: up to four sensor-only planners clearing one
 * building together over the lossy, wall-attenuated radio of src/team.
 *
 * An entry is a team configuration (how many drones, which fly as relays, how
 * frontiers are split, cruise speed, how often they talk). The radio, the
 * buildings and the time budget are the bench's and cannot be changed. Every
 * drone runs the SensorPlanner; plugging a different pilot into a team is
 * future work.
 *
 * The seed blocks 83001+ (dev) and 83101+ (test) are used nowhere else; the
 * large condition offsets them by 500.
 */
import { DEFAULT_CLAIMS, type ClaimOptions } from '../team/allocate';
import { TEAM_CONDITIONS, teamEpisodeSetup } from '../team/conditions';
import { runTeam, type Role, type TeamOptions, type TeamResult } from '../team/run';
import type { BenchSplit } from './tasks';

export const TEAM_BENCH_VERSION = 1;
export const TEAM_MAX_DRONES = 4;

export interface TeamEntry {
  name: string;
  drones: number;
  /** Per drone; missing entries explore. */
  roles?: Role[];
  /** Frontier claims (src/team/allocate.ts); null shares maps but does not split frontiers. */
  claims?: Partial<ClaimOptions> | null;
  /** Planner cruise speed, m/s (at most 2, the bench's speed cap). */
  speed?: number;
  /** Team steps between exchange rounds. */
  exchangeEvery?: number;
  /** Team steps between launches through the entry. */
  launchGap?: number;
  /** Mean link margin to the base, dB, below which a relay backs up. */
  relayMargin?: number;
}

export interface TeamBenchTask {
  id: string;
  label: string;
  /** Seed offset inside the track's dev and test blocks. */
  offset: number;
}

export const TEAM_BENCH_TASKS: readonly TeamBenchTask[] = TEAM_CONDITIONS.map((c, i) => ({ id: c.id, label: c.label, offset: i * 500 }));

export const TEAM_BENCH_SEEDS: Record<BenchSplit, { base: number; count: number }> = {
  dev: { base: 83001, count: 10 },
  test: { base: 83101, count: 30 },
};

export function teamBenchSeeds(split: BenchSplit, task: TeamBenchTask): number[] {
  const { base, count } = TEAM_BENCH_SEEDS[split];
  return Array.from({ length: count }, (_, i) => base + task.offset + i);
}

/** The run options an entry flies with; throws on an entry that breaks the rules. */
export function teamOptions(entry: TeamEntry): TeamOptions {
  if (!Number.isInteger(entry.drones) || entry.drones < 1 || entry.drones > TEAM_MAX_DRONES) throw new Error(`a team has 1 to ${TEAM_MAX_DRONES} drones`);
  if (entry.roles && entry.roles.length > entry.drones) throw new Error('more roles than drones');
  if (entry.speed !== undefined && !(entry.speed > 0 && entry.speed <= 2)) throw new Error('speed must be in (0, 2] m/s');
  const opts: TeamOptions = { drones: entry.drones, comms: 'radio' };
  if (entry.roles) opts.roles = entry.roles;
  if (entry.speed !== undefined) opts.speed = entry.speed;
  if (entry.exchangeEvery !== undefined) opts.exchangeEvery = Math.max(1, Math.round(entry.exchangeEvery));
  if (entry.launchGap !== undefined) opts.launchGap = Math.max(1, Math.round(entry.launchGap));
  if (entry.relayMargin !== undefined) opts.relayMargin = entry.relayMargin;
  if (entry.claims === null) opts.claims = null;
  else if (entry.claims) opts.claims = { ...DEFAULT_CLAIMS, ...entry.claims };
  return opts;
}

export interface TeamBenchEpisode {
  task: string;
  seed: number;
  coverage: number;
  baseKnown: number;
  /** Seconds to 80% coverage, null if never. */
  t80: number | null;
  collisions: number;
  droneContacts: number;
  overlap: number;
}

const STEP_S = 0.05;

export async function runTeamBenchEpisode(entry: TeamEntry, task: TeamBenchTask, seed: number): Promise<TeamBenchEpisode> {
  const cond = TEAM_CONDITIONS.find((c) => c.id === task.id)!;
  const s = teamEpisodeSetup(cond, seed);
  const r: TeamResult = await runTeam(s.spec, s.envSeed, s.cfg, teamOptions(entry), seed);
  return {
    task: task.id,
    seed,
    coverage: r.coverage,
    baseKnown: r.baseKnown,
    t80: r.timeTo['80'] === null ? null : r.timeTo['80'] * STEP_S,
    collisions: r.collisions,
    droneContacts: r.droneContacts,
    overlap: r.overlap,
  };
}

/** Fly every task of a split (or the listed ones). */
export async function flyTeam(entry: TeamEntry, split: BenchSplit, only: string[] = [], log?: (task: string, eps: TeamBenchEpisode[]) => void): Promise<TeamBenchEpisode[]> {
  const out: TeamBenchEpisode[] = [];
  for (const task of TEAM_BENCH_TASKS) {
    if (only.length && !only.includes(task.id)) continue;
    const mine: TeamBenchEpisode[] = [];
    for (const seed of teamBenchSeeds(split, task)) mine.push(await runTeamBenchEpisode(entry, task, seed));
    out.push(...mine);
    log?.(task.id, mine);
  }
  return out;
}

const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);

/** Score: mean coverage over the tasks. Also what reached the base, time to 80%, and contacts. */
export function summarizeTeam(episodes: TeamBenchEpisode[]) {
  const tasks: Record<string, { episodes: number; coverage: number; baseKnown: number; reached80: number; t80: number | null; collisionsPerEpisode: number; contactsPerEpisode: number }> = {};
  const covs: number[] = [];
  for (const t of TEAM_BENCH_TASKS) {
    const rs = episodes.filter((e) => e.task === t.id);
    if (!rs.length) continue;
    const t80 = rs.filter((r) => r.t80 !== null).map((r) => r.t80!);
    covs.push(mean(rs.map((r) => r.coverage)));
    tasks[t.id] = {
      episodes: rs.length,
      coverage: mean(rs.map((r) => r.coverage)),
      baseKnown: mean(rs.map((r) => r.baseKnown)),
      reached80: t80.length / rs.length,
      t80: t80.length ? mean(t80) : null,
      collisionsPerEpisode: mean(rs.map((r) => r.collisions)),
      contactsPerEpisode: mean(rs.map((r) => r.droneContacts)),
    };
  }
  return { score: mean(covs), tasks };
}

/** Baseline teams, the drone-team study's main arms. */
export const TEAM_BASELINES: Record<string, TeamEntry> = {
  solo: { name: '1 drone', drones: 1 },
  team2: { name: '2 drones', drones: 2 },
  team3: { name: '3 drones', drones: 3 },
  team4: { name: '4 drones', drones: 4 },
  'team3-noclaims': { name: '3 drones, shared maps, no frontier split', drones: 3, claims: null },
  'team3-relay': { name: '2 explorers + 1 relay', drones: 3, roles: ['explore', 'explore', 'relay'] },
};

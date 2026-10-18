import { generateBuilding } from '../gen/building';
import { Curriculum } from '../rl/curriculum';
import { ACT_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec, EpisodeConfig } from '../shared/types';
import { createEnv, defaultEpisodeConfig, type ExplorerEnv } from '../sim/env';
import { lightingPreset, type LightingPreset } from '../sim/lighting';

/** Fixed eval protocol for the shipped indoor policy. Changing it changes every reported number. */
export const STAGE0_SEEDS = [101, 202, 303, 404];
export const STAGE3_SEEDS = [4242, 5150, 6160];
export const TARGETS = { stage0: 0.8, stage3: 0.6, nightToDay: 0.9 } as const;

export interface CoveragePolicy {
  /** Called once per episode before the first act(). */
  reset?(): void;
  act(obs: Float32Array, env: ExplorerEnv): Float32Array;
}

export interface CoverageReport {
  stage0Day: number[];
  stage0Night: number[];
  stage3Day: number[];
  stage3Night: number[];
}

export function evalSteps(stage: number): number {
  return stage <= 1 ? 1000 : 2400;
}

/** Building, env seed and config of one eval episode. Shared with the parallel eval. */
export function evalEpisodeSetup(
  stage: number,
  seed: number,
  lighting: LightingPreset,
): { spec: BuildingSpec; envSeed: number; cfg: EpisodeConfig & { maxSteps: number } } {
  const params = new Curriculum().genParams(stage, new RNG(seed, `eval-${stage}`));
  const spec = generateBuilding(params);
  const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
  const cfg = defaultEpisodeConfig(stage, nRooms);
  cfg.maxSteps = evalSteps(stage);
  cfg.stallSteps = 500;
  cfg.maxCollisions = 16;
  cfg.dynamics = 'kinematic';
  cfg.lighting = lightingPreset(lighting, nRooms);
  return { spec, envSeed: params.seed, cfg: cfg as EpisodeConfig & { maxSteps: number } };
}

export async function coverEpisode(
  policy: CoveragePolicy,
  stage: number,
  seed: number,
  lighting: LightingPreset,
): Promise<number> {
  const env = await createEnv();
  const { spec, envSeed, cfg } = evalEpisodeSetup(stage, seed, lighting);
  let obs = env.reset(spec, envSeed, cfg);
  policy.reset?.();
  let coverage = 0;
  for (let t = 0; t < cfg.maxSteps; t++) {
    const action = policy.act(obs, env);
    if (action.length < ACT_DIM) throw new Error('action');
    const step = env.step(action);
    obs = step.obs;
    coverage = step.info.coverage;
    if (step.done) break;
  }
  env.dispose();
  return coverage;
}

export async function evalCoverage(policy: CoveragePolicy): Promise<CoverageReport> {
  const report: CoverageReport = { stage0Day: [], stage0Night: [], stage3Day: [], stage3Night: [] };
  for (const seed of STAGE0_SEEDS) {
    report.stage0Day.push(await coverEpisode(policy, 0, seed, 'day'));
    report.stage0Night.push(await coverEpisode(policy, 0, seed, 'night-dark'));
  }
  for (const seed of STAGE3_SEEDS) {
    report.stage3Day.push(await coverEpisode(policy, 3, seed, 'day'));
    report.stage3Night.push(await coverEpisode(policy, 3, seed, 'night-dark'));
  }
  return report;
}

export function mean(xs: number[]): number {
  return xs.length ? xs.reduce((s, v) => s + v, 0) / xs.length : 0;
}

const pct = (v: number) => `${(v * 100).toFixed(1)}%`;

export function formatCoverageTable(report: CoverageReport, label = 'policy'): string {
  const d0 = mean(report.stage0Day);
  const n0 = mean(report.stage0Night);
  const d3 = mean(report.stage3Day);
  const n3 = mean(report.stage3Night);
  const ratio = d0 > 0 ? n0 / d0 : 0;
  const ratio3 = d3 > 0 ? n3 / d3 : 0;
  const mark = (ok: boolean) => (ok ? 'met' : 'MISSED');
  const rows = [
    ['Metric', 'Per seed', 'Mean', 'Target', 'Status'],
    [
      `Stage 0 day (${STAGE0_SEEDS.join('/')})`,
      report.stage0Day.map(pct).join(', '),
      pct(d0),
      `>= ${pct(TARGETS.stage0)}`,
      mark(d0 >= TARGETS.stage0),
    ],
    [
      `Stage 3 day (${STAGE3_SEEDS.join('/')})`,
      report.stage3Day.map(pct).join(', '),
      pct(d3),
      `>= ${pct(TARGETS.stage3)}`,
      mark(d3 >= TARGETS.stage3),
    ],
    ['Stage 0 night-dark', report.stage0Night.map(pct).join(', '), pct(n0), '', ''],
    ['Stage 3 night-dark', report.stage3Night.map(pct).join(', '), pct(n3), '', ''],
    ['Night / day, stage 0', '', ratio.toFixed(3), `>= ${TARGETS.nightToDay}`, mark(ratio >= TARGETS.nightToDay)],
    ['Night / day, stage 3', '', ratio3.toFixed(3), `>= ${TARGETS.nightToDay}`, mark(ratio3 >= TARGETS.nightToDay)],
  ];
  const widths = rows[0]!.map((_, c) => Math.max(...rows.map((r) => r[c]!.length)));
  const line = (r: string[]) => `| ${r.map((cell, c) => cell.padEnd(widths[c]!)).join(' | ')} |`;
  const sep = `|${widths.map((w) => '-'.repeat(w + 2)).join('|')}|`;
  return [`Coverage: ${label}`, line(rows[0]!), sep, ...rows.slice(1).map(line)].join('\n');
}

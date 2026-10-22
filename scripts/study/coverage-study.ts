// Coverage study: every pilot on 30 fresh buildings per condition, with 95% confidence intervals.
//
//   npm run study:coverage                 # all shards in parallel, then merge into docs/study.json
//   STUDY_ENTRY=coverage-study vite-node scripts/study/coverage-study.ts -- --shard 0 --of 4 --out /tmp/s0.jsonl [--conditions hard] [--pilots policy,guarded]
//   STUDY_ENTRY=coverage-study vite-node scripts/study/coverage-study.ts -- --merge /tmp/s0.jsonl ... --out docs/study.json
//
// Seeds 20001+, 30001+, 40001+ and 50001+ are drawn only here. Training draws curriculum seeds,
// checkpoints are selected on 9001-9166, and the sensor planner and policy guard were tuned on
// 9101-9108, so none of these buildings informed any choice being measured.
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { OracleGain } from '../../src/eval/gainLabels';
import { classicalPilot, climbingPilot, guardedAction } from '../../src/eval/pilots';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { RNG } from '../../src/shared/rng';
import { sampleDynParams, sampleNoise, defaultDynParams } from '../../src/sim/dynamics/dr';
import { createEnv, type ExplorerEnv } from '../../src/sim/env';
import { frontierAction } from '../../src/sim/frontier';
import { LearnedGain, parseGainModel, type GainModelData } from '../../src/sim/mapPredict';
import { QUAD_BRAKING } from '../../src/sim/sensorPlanner';
import { heuristicAction } from '../../src/sim/heuristic';
import type { LightingPreset } from '../../src/sim/lighting';
import { waypointAction } from '../../src/sim/waypointPilot';
import { bootstrapCi, loadPolicy } from './common';

export { bootstrapCi, loadPolicy };

export const SEEDS_PER_CONDITION = 30;

export interface Condition {
  id: string;
  label: string;
  stage: number;
  seedBase: number;
  lighting: LightingPreset[];
  /** Quadrotor dynamics with domain randomization at this scale (mass, thrust, drag, motor lag, sensor noise, latency). */
  quad?: number;
}

export const CONDITIONS: Condition[] = [
  { id: 'easy', label: 'Easy: one room (stage 0)', stage: 0, seedBase: 20001, lighting: ['day', 'night-dark'] },
  { id: 'hard', label: 'Hard: rooms, hallways, loops (stage 3)', stage: 3, seedBase: 30001, lighting: ['day', 'night-dark'] },
  { id: 'floors', label: 'Multi-floor with shafts (stage 4)', stage: 4, seedBase: 40001, lighting: ['day'] },
  { id: 'quad', label: 'Hard, quadrotor dynamics + noise', stage: 3, seedBase: 50001, lighting: ['day', 'night-dark'], quad: 1 },
];

export const PILOTS = [
  'teacher',
  'classical',
  'classicalBrake',
  'classicalTrack',
  'classicalBrakeTrack',
  'classicalClimb',
  'classicalPredict',
  'classicalOracleGain',
  'policy',
  'guarded',
  'guardedBrake',
  'waypoint',
  'heuristic',
] as const;
export type PilotId = (typeof PILOTS)[number];
/**
 * Pilots that only run in some conditions: the braking pilots where the airframe is the quad, the
 * climbing planner where there is more than one floor, and the map-prediction arms on the kinematic
 * hard and multi-floor buildings.
 */
export const PILOT_CONDITIONS: Partial<Record<PilotId, string[]>> = {
  classicalBrake: ['quad'],
  classicalTrack: ['quad'],
  classicalBrakeTrack: ['quad'],
  guardedBrake: ['quad'],
  classicalClimb: ['floors'],
  classicalPredict: ['hard', 'floors'],
  classicalOracleGain: ['hard', 'floors'],
};
/** Share of the step budget the climbing planner gives each floor before it takes a known shaft. */
export const CLIMB_FLOOR_SHARE = 0.8;
/** Path cost (voxel steps) per voxel of predicted hidden gain, for the map-prediction arms. */
export const GAIN_WEIGHT = 0.3;

export interface Job {
  condition: string;
  stage: number;
  seed: number;
  lighting: LightingPreset;
  quad: number;
  pilot: PilotId;
}

export interface Result extends Job {
  coverage: number;
  collisions: number;
  steps: number;
  floorsReached: number;
  floors: number;
}

export function jobs(): Job[] {
  const out: Job[] = [];
  for (const c of CONDITIONS) {
    for (let i = 0; i < SEEDS_PER_CONDITION; i++) {
      for (const lighting of c.lighting) {
        for (const pilot of PILOTS) if (!PILOT_CONDITIONS[pilot] || PILOT_CONDITIONS[pilot].includes(c.id)) out.push({ condition: c.id, stage: c.stage, seed: c.seedBase + i, lighting, quad: c.quad ?? 0, pilot });
      }
    }
  }
  return out;
}

type Act = (obs: Float32Array, env: ExplorerEnv) => Float32Array;


export const GAIN_MODEL_PATH = 'public/models/mappred.json';
let gainData: GainModelData | null = null;
function gainModel(): GainModelData {
  gainData ??= parseGainModel(readFileSync(GAIN_MODEL_PATH, 'utf8'));
  return gainData;
}

export interface Episode {
  env: ExplorerEnv;
  spec: ReturnType<typeof evalEpisodeSetup>['spec'];
  cfg: ReturnType<typeof evalEpisodeSetup>['cfg'];
  obs: Float32Array;
  act: Act;
}

/**
 * Build the env and pilot for a job, reset and ready to step. `setup` replaces the generated
 * building (the real-plan study passes converted floor plans).
 */
export async function startEpisode(
  job: Job,
  policy: (obs: Float32Array) => Float32Array,
  setup: ReturnType<typeof evalEpisodeSetup> = evalEpisodeSetup(job.stage, job.seed, job.lighting),
): Promise<Episode> {
  const env = await createEnv();
  const { spec, envSeed, cfg } = setup;
  if (job.quad > 0) {
    const rng = new RNG(job.seed, 'study-dr');
    cfg.dynamics = 'quad';
    cfg.dyn = sampleDynParams(rng, defaultDynParams(), job.quad);
    cfg.noise = sampleNoise(rng, job.quad, job.lighting !== 'day');
  }
  let act: Act;
  if (job.pilot === 'teacher') {
    act = (_obs, e) => {
      const s = e.snapshot();
      return frontierAction(spec, e.visitedGrid, s.pos, s.yaw);
    };
  } else if (job.pilot === 'classical') {
    const p = classicalPilot();
    p.reset();
    act = p.act;
  } else if (job.pilot === 'classicalBrake') {
    const p = classicalPilot({ braking: QUAD_BRAKING });
    p.reset();
    act = p.act;
  } else if (job.pilot === 'classicalTrack' || job.pilot === 'classicalBrakeTrack') {
    // Quad-aware trajectory tracking (sim/tracker.ts), defaults tuned on 9101-9108.
    const p = classicalPilot({ tracker: {}, ...(job.pilot === 'classicalBrakeTrack' ? { braking: QUAD_BRAKING } : {}) });
    p.reset();
    act = p.act;
  } else if (job.pilot === 'classicalClimb') {
    const p = climbingPilot({}, { floorSteps: (CLIMB_FLOOR_SHARE * cfg.maxSteps) / spec.floors.length });
    p.reset();
    act = p.act;
  } else if (job.pilot === 'classicalPredict' || job.pilot === 'classicalOracleGain') {
    const gain = job.pilot === 'classicalPredict' ? new LearnedGain(gainModel()) : new OracleGain(spec);
    const p = classicalPilot({ gain, gainWeight: GAIN_WEIGHT });
    p.reset();
    act = p.act;
  } else if (job.pilot === 'policy') {
    act = (obs) => policy(obs);
  } else if (job.pilot === 'guarded') {
    act = (obs, e) => guardedAction(policy(obs), obs, e);
  } else if (job.pilot === 'guardedBrake') {
    act = (obs, e) => guardedAction(policy(obs), obs, e, QUAD_BRAKING);
  } else if (job.pilot === 'waypoint') {
    act = (obs) => waypointAction(obs);
  } else {
    act = (obs) => heuristicAction(obs);
  }
  const obs = env.reset(spec, envSeed, cfg);
  return { env, spec, cfg, obs, act };
}

export async function runJob(
  job: Job,
  policy: (obs: Float32Array) => Float32Array,
  setup?: ReturnType<typeof evalEpisodeSetup>,
): Promise<Result> {
  const { env, spec, cfg, act, obs: first } = await startEpisode(job, policy, setup);
  let obs = first;
  let coverage = 0;
  let collisions = 0;
  let steps = 0;
  const floors = new Set<number>();
  for (let t = 0; t < cfg.maxSteps; t++) {
    const step = env.step(act(obs, env));
    obs = step.obs;
    coverage = step.info.coverage;
    collisions = step.info.collisions;
    floors.add(step.info.floor);
    steps = t + 1;
    if (step.done) break;
  }
  env.dispose();
  return { ...job, coverage, collisions, steps, floorsReached: floors.size, floors: spec.floors.length };
}

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}


export function summarizeStudy(rows: Result[]) {
  const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
  const cells = [];
  for (const c of CONDITIONS) {
    for (const lighting of c.lighting) {
      for (const pilot of PILOTS) {
        const rs = rows.filter((r) => r.condition === c.id && r.lighting === lighting && r.pilot === pilot);
        if (!rs.length) continue;
        const cov = rs.map((r) => r.coverage);
        cells.push({
          condition: c.id,
          lighting,
          pilot,
          episodes: rs.length,
          coverage: mean(cov),
          coverageCi: bootstrapCi(cov),
          collisionsPerEpisode: mean(rs.map((r) => r.collisions)),
          episodesWithCollision: rs.filter((r) => r.collisions > 0).length,
          reachedAnotherFloor: rs.filter((r) => r.floorsReached > 1).length,
        });
      }
    }
  }
  return cells;
}

// Other studies import this module; only run the CLI when it is the entry script. vite-node drops the
// script path from process.argv, so the launchers (parallel.mjs, ablations.ts) name it in STUDY_ENTRY.
const MAIN = process.env.STUDY_ENTRY === 'coverage-study' || process.argv.some((a) => a.endsWith('coverage-study.ts'));

if (MAIN && process.argv.includes('--merge')) {
  const outPath = arg('out', 'docs/study.json');
  const files = process.argv.slice(process.argv.indexOf('--merge') + 1).filter((a) => a.endsWith('.jsonl'));
  const rows: Result[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as Result),
  );
  const report = {
    schema: 1,
    commit: execSync('git rev-parse --short HEAD').toString().trim(),
    checkpoint: arg('checkpoint', SHIPPED_CHECKPOINT_PATH),
    seedsPerCondition: SEEDS_PER_CONDITION,
    conditions: CONDITIONS,
    pilots: PILOTS,
    cells: summarizeStudy(rows),
    episodes: rows,
  };
  writeFileSync(outPath, `${JSON.stringify(report, null, 2)}\n`);
  console.log(`wrote ${outPath} from ${rows.length} episodes`);
} else if (MAIN && process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `study-${shard}.jsonl`);
  const policy = loadPolicy(arg('checkpoint', SHIPPED_CHECKPOINT_PATH));
  // Optional subsets, e.g. `--conditions hard --pilots policy,guarded` for the ablation checkpoints.
  const conditions = arg('conditions', '').split(',').filter(Boolean);
  const pilots = arg('pilots', '').split(',').filter(Boolean);
  const mine = jobs()
    .filter((j) => (!conditions.length || conditions.includes(j.condition)) && (!pilots.length || pilots.includes(j.pilot)))
    .filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    const r = await runJob(mine[i]!, policy);
    lines.push(JSON.stringify(r));
    if (i % 25 === 0) {
      writeFileSync(out, lines.join('\n') + '\n');
      console.log(`shard ${shard}: ${i + 1}/${mine.length} (${Math.round((Date.now() - t0) / 1000)} s)`);
    }
  }
  writeFileSync(out, lines.join('\n') + '\n');
  console.log(`shard ${shard}: done, ${mine.length} episodes in ${Math.round((Date.now() - t0) / 1000)} s`);
}

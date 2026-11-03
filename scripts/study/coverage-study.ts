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
import { evalEpisodeSetup, evalSteps } from '../../src/eval/coverage';
import { applyHazards } from '../../src/gen/hazards';
import type { HazardParams } from '../../src/shared/types';
import { EpisodeFootprint, FOOTPRINT_META, summarizeFootprint, withoutCurve, type FootprintRecord } from '../../src/eval/footprint';
import { OracleGain } from '../../src/eval/gainLabels';
import { classicalPilot, climbingPilot, guardedAction } from '../../src/eval/pilots';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { CRASH_SPEED } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import { sampleDynParams, sampleNoise, defaultDynParams } from '../../src/sim/dynamics/dr';
import { createEnv, type ExplorerEnv } from '../../src/sim/env';
import { frontierAction } from '../../src/sim/frontier';
import { LearnedGain, parseGainModel, type GainModelData } from '../../src/sim/mapPredict';
import { QUAD_BRAKING } from '../../src/sim/lidarSafety';
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
  /** Indoor hazards built into every building of the condition (gen/hazards.ts). */
  hazards?: HazardParams;
}

/**
 * Hazard conditions (shut and ajar doors, glass and mirrors, smoke) and their paired control share one
 * fresh block, 81001-81030: each seed is the same stage-3 building with and without the hazard.
 * 81901-81916 is the hazard development block; nothing here was tuned on 81001+.
 */
export const HAZARD_SEED_BASE = 81001;
export const HAZARD_CONDITIONS: Condition[] = [
  { id: 'hazard-control', label: 'Hazard control: the same hard buildings, no hazards', stage: 3, seedBase: HAZARD_SEED_BASE, lighting: ['day'] },
  { id: 'doors', label: 'Shut and ajar interior doors', stage: 3, seedBase: HAZARD_SEED_BASE, lighting: ['day'], hazards: { closedDoors: 0.35, ajarDoors: 0.6 } },
  { id: 'glass', label: 'Glass rooms and mirrors', stage: 3, seedBase: HAZARD_SEED_BASE, lighting: ['day'], hazards: { glassRooms: 0.5, mirrors: 2 } },
  { id: 'smoke', label: 'Smoke (visibility 1.5-4 m)', stage: 3, seedBase: HAZARD_SEED_BASE, lighting: ['day'], hazards: { smokeRooms: 0.4, smokeVisibility: [1.5, 4] } },
];
/** Pilots the hazard conditions fly: the privileged teacher as a reference, the planner, the policy alone and guarded. */
export const HAZARD_PILOTS: PilotId[] = ['teacher', 'classical', 'policy', 'guarded'];
/**
 * Pass conditions, fixed before the first run on 81001+. Each pilot below must, in every hazard
 * condition: end at most HAZARD_PASS.maxCrashes of 30 flights in a crash, and cover at least
 * HAZARD_PASS.coverageRatio of its own coverage on the paired control.
 */
export const HAZARD_PASS = { pilots: ['classical', 'guarded'] as PilotId[], maxCrashes: 2, coverageRatio: 0.9 } as const;

export const CONDITIONS: Condition[] = [
  { id: 'easy', label: 'Easy: one room (stage 0)', stage: 0, seedBase: 20001, lighting: ['day', 'night-dark'] },
  { id: 'hard', label: 'Hard: rooms, hallways, loops (stage 3)', stage: 3, seedBase: 30001, lighting: ['day', 'night-dark'] },
  { id: 'floors', label: 'Multi-floor with shafts (stage 4)', stage: 4, seedBase: 40001, lighting: ['day'] },
  { id: 'quad', label: 'Hard, quadrotor dynamics + noise', stage: 3, seedBase: 50001, lighting: ['day', 'night-dark'], quad: 1 },
  ...HAZARD_CONDITIONS,
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

export interface Result extends Job, Partial<FootprintRecord> {
  /** Flown coverage: voxels the drone center passed through (the env's own metric). */
  coverage: number;
  collisions: number;
  /** The crash rule (shared/constants.ts CRASH_SPEED) ended the flight. */
  crashed?: boolean;
  /** Speed into the surface of every contact, m/s, in order; the last one is the crash when `crashed`. */
  impacts?: number[];
  steps: number;
  floorsReached: number;
  floors: number;
}

/** Whether a pilot flies a condition: hazard conditions fly HAZARD_PILOTS, the rest follow PILOT_CONDITIONS. */
export function fliesIn(pilot: PilotId, c: Condition): boolean {
  if (HAZARD_CONDITIONS.includes(c)) return HAZARD_PILOTS.includes(pilot);
  return !PILOT_CONDITIONS[pilot] || PILOT_CONDITIONS[pilot].includes(c.id);
}

/** The building, env seed and config of a job, with the condition's hazards built in. */
export function jobSetup(job: Job): ReturnType<typeof evalEpisodeSetup> {
  const setup = evalEpisodeSetup(job.stage, job.seed, job.lighting);
  const hazards = CONDITIONS.find((c) => c.id === job.condition)?.hazards;
  if (hazards) setup.spec = applyHazards(setup.spec, hazards);
  return setup;
}

export function jobs(): Job[] {
  const out: Job[] = [];
  for (const c of CONDITIONS) {
    for (let i = 0; i < SEEDS_PER_CONDITION; i++) {
      for (const lighting of c.lighting) {
        for (const pilot of PILOTS) if (fliesIn(pilot, c)) out.push({ condition: c.id, stage: c.stage, seed: c.seedBase + i, lighting, quad: c.quad ?? 0, pilot });
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
  setup: ReturnType<typeof evalEpisodeSetup> = jobSetup(job),
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
  const impacts: number[] = [];
  let crashed = false;
  const fp = new EpisodeFootprint();
  fp.start(env, spec, cfg.maxSteps);
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
    if (step.info.impactSpeed > 0) impacts.push(Math.round(step.info.impactSpeed * 1000) / 1000);
    crashed = step.info.crashed;
    floors.add(step.info.floor);
    steps = t + 1;
    fp.step(env, steps, coverage);
    if (step.done) break;
  }
  env.dispose();
  return { ...job, coverage, collisions, crashed, impacts, steps, floorsReached: floors.size, floors: spec.floors.length, ...fp.finish(coverage) };
}

/**
 * Shard of the i-th job. Jobs cycle through pilots, and some pilots (map prediction) are several
 * times slower than others, so a plain `i % of` can hand one shard all of them. A fixed shuffle
 * spreads them out; which shard runs a job never changes its result.
 */
const shardOrder = new Map<number, number[]>();
function shardOf(i: number, of: number): number {
  const n = jobs().length;
  let order = shardOrder.get(of);
  if (!order) {
    const rng = new RNG(1, 'study-shards');
    order = Array.from({ length: n }, (_, k) => k % of);
    for (let k = n - 1; k > 0; k--) {
      const j = rng.int(k + 1);
      [order[k], order[j]] = [order[j]!, order[k]!];
    }
    shardOrder.set(of, order);
  }
  return order[i % n]!;
}

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}


/** Crash-rule columns of a cell: flights the rule ended, the crash-free share with its interval, and contact speeds. */
function crashCells(rs: Result[]) {
  const free: number[] = rs.map((r) => (r.crashed ? 0 : 1));
  const speeds = rs.flatMap((r) => r.impacts ?? []).sort((a, b) => a - b);
  return {
    crashes: rs.length - free.reduce((a, b) => a + b, 0),
    crashFree: free.reduce((a, b) => a + b, 0) / rs.length,
    crashFreeCi: bootstrapCi(free),
    medianContactSpeed: speeds.length ? speeds[Math.floor((speeds.length - 1) / 2)]! : null,
  };
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
        const fp = rs.every((r) => r.footprint && r.curve) ? { footprintStats: summarizeFootprint(rs as FootprintRecord[], evalSteps(c.stage) / 20, bootstrapCi) } : {};
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
          ...(rs.every((r) => r.crashed !== undefined) ? crashCells(rs) : {}),
          ...fp,
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
    crashSpeed: CRASH_SPEED,
    commit: execSync('git rev-parse --short HEAD').toString().trim(),
    checkpoint: arg('checkpoint', SHIPPED_CHECKPOINT_PATH),
    seedsPerCondition: SEEDS_PER_CONDITION,
    footprint: FOOTPRINT_META,
    conditions: CONDITIONS,
    pilots: PILOTS,
    cells: summarizeStudy(rows),
    episodes: rows.map(withoutCurve),
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
    .filter((_, i) => shardOf(i, of) === shard);
  // Development runs: `--dev-base 81901 --dev-seeds 8` flies the first seeds of another block instead.
  const devBase = Number(arg('dev-base', '0'));
  if (devBase) {
    const keep = Number(arg('dev-seeds', String(SEEDS_PER_CONDITION)));
    const base = (j: Job) => CONDITIONS.find((c) => c.id === j.condition)!.seedBase;
    mine.splice(0, mine.length, ...mine.filter((j) => j.seed - base(j) < keep).map((j) => ({ ...j, seed: devBase + j.seed - base(j) })));
  }
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

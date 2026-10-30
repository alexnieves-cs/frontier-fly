// Speed and energy study: how coverage, collisions and energy change with the cruise-speed cap,
// for the sensor planner (with and without momentum-aware goal choice) and the shipped policy.
//
//   npm run study:speed                 # all shards in parallel, then merge into docs/speed-study.json
//   vite-node scripts/study/speed-study.ts -- --shard 0 --of 4 --out /tmp/s0.jsonl
//
// Uses the coverage study's held-out hard (30001+) and quad (50001+) buildings, with the same
// domain randomization. The momentum weight and the speed sweep were chosen on 9101-9108 only
// (scripts/study/speed-diag.ts).
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { evalEpisodeSetup, evalSteps } from '../../src/eval/coverage';
import { EpisodeFootprint, FOOTPRINT_META, summarizeFootprint, withoutCurve, type FootprintRecord } from '../../src/eval/footprint';
import { classicalPilot, guardedAction } from '../../src/eval/pilots';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { RNG } from '../../src/shared/rng';
import { defaultDynParams, sampleDynParams, sampleNoise } from '../../src/sim/dynamics/dr';
import { createEnv, type ExplorerEnv } from '../../src/sim/env';
import { type SensorPlannerOptions } from '../../src/sim/sensorPlanner';
import { QUAD_BRAKING } from '../../src/sim/lidarSafety';
import { bootstrapCi, loadPolicy } from './common';

export const SPEED_SEEDS = 30;
export const SPEEDS = [2, 3, 4, 5];
/** Momentum weight for the goal choice, tuned on 9101-9108 (speed-diag.ts --mom). */
export const MOMENTUM = 1;

export const SPEED_CONDITIONS = [
  { id: 'hard', label: 'Hard, kinematic drone', stage: 3, seedBase: 30001, quad: 0 },
  { id: 'quad', label: 'Hard, quadrotor dynamics + noise', stage: 3, seedBase: 50001, quad: 1 },
] as const;

export const SPEED_PILOTS = ['planner', 'plannerMomentum', 'plannerBrake', 'plannerBrakeMomentum', 'plannerTrack', 'plannerBrakeTrack', 'policy', 'guarded'] as const;
type SpeedPilot = (typeof SPEED_PILOTS)[number];
/** The braking-filter and tracking planners fly only the quad, as in the coverage study. */
const QUAD_ONLY: SpeedPilot[] = ['plannerBrake', 'plannerBrakeMomentum', 'plannerTrack', 'plannerBrakeTrack'];

interface Job {
  condition: string;
  stage: number;
  seed: number;
  quad: number;
  pilot: SpeedPilot;
  speed: number;
}

interface Result extends Job, Partial<FootprintRecord> {
  /** Flown coverage (the env's metric). */
  coverage: number;
  collisions: number;
  energyWh: number;
  /** Seconds until half the building was flown, or null if it never was. */
  halfS: number | null;
}

function jobs(): Job[] {
  const out: Job[] = [];
  for (const c of SPEED_CONDITIONS)
    for (let i = 0; i < SPEED_SEEDS; i++)
      for (const speed of SPEEDS)
        for (const pilot of SPEED_PILOTS)
          if (c.quad || !QUAD_ONLY.includes(pilot)) out.push({ condition: c.id, stage: c.stage, seed: c.seedBase + i, quad: c.quad, pilot, speed });
  return out;
}

function plannerOptions(pilot: SpeedPilot, speed: number): SensorPlannerOptions {
  const o: SensorPlannerOptions = { speed };
  if (pilot === 'plannerBrake' || pilot === 'plannerBrakeMomentum' || pilot === 'plannerBrakeTrack') o.braking = QUAD_BRAKING;
  // Quad-aware trajectory tracking (sim/tracker.ts), defaults tuned on 9101-9108.
  if (pilot === 'plannerTrack' || pilot === 'plannerBrakeTrack') o.tracker = {};
  if (pilot === 'plannerMomentum' || pilot === 'plannerBrakeMomentum') o.momentum = MOMENTUM;
  return o;
}

async function runJob(env: ExplorerEnv, job: Job, policy: (obs: Float32Array) => Float32Array): Promise<Result> {
  const { spec, envSeed, cfg } = evalEpisodeSetup(job.stage, job.seed, 'day');
  if (job.quad > 0) {
    const rng = new RNG(job.seed, 'study-dr');
    cfg.dynamics = 'quad';
    cfg.dyn = sampleDynParams(rng, defaultDynParams(), job.quad);
    cfg.noise = sampleNoise(rng, job.quad, false);
  }
  cfg.speedH = job.speed;
  let act: (obs: Float32Array, e: ExplorerEnv) => Float32Array;
  if (job.pilot === 'policy') act = (obs) => policy(obs);
  else if (job.pilot === 'guarded') act = (obs, e) => guardedAction(policy(obs), obs, e, job.quad ? QUAD_BRAKING : undefined);
  else {
    const p = classicalPilot(plannerOptions(job.pilot, job.speed));
    p.reset();
    act = p.act;
  }
  let obs = env.reset(spec, envSeed, cfg);
  const fp = new EpisodeFootprint();
  fp.start(env, spec, cfg.maxSteps);
  let info = { coverage: 0, collisions: 0, energyWh: 0 };
  let halfS: number | null = null;
  for (let t = 0; t < cfg.maxSteps; t++) {
    const s = env.step(act(obs, env));
    obs = s.obs;
    info = s.info;
    if (halfS === null && info.coverage >= 0.5) halfS = (t + 1) / 20;
    fp.step(env, t + 1, info.coverage);
    if (s.done) break;
  }
  return { ...job, coverage: info.coverage, collisions: info.collisions, energyWh: info.energyWh, halfS, ...fp.finish(info.coverage) };
}

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

function summarize(rows: Result[]) {
  const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
  const cells = [];
  for (const c of SPEED_CONDITIONS)
    for (const pilot of SPEED_PILOTS)
      for (const speed of SPEEDS) {
        const rs = rows.filter((r) => r.condition === c.id && r.pilot === pilot && r.speed === speed);
        if (!rs.length) continue;
        const cov = rs.map((r) => r.coverage);
        const wh = rs.map((r) => r.energyWh);
        const half = rs.filter((r) => r.halfS !== null).map((r) => r.halfS!);
        const fp = rs.every((r) => r.footprint && r.curve) ? { footprintStats: summarizeFootprint(rs as FootprintRecord[], evalSteps(c.stage) / 20, bootstrapCi) } : {};
        cells.push({
          condition: c.id,
          pilot,
          speed,
          episodes: rs.length,
          coverage: mean(cov),
          coverageCi: bootstrapCi(cov),
          collisionsPerEpisode: mean(rs.map((r) => r.collisions)),
          episodesWithCollision: rs.filter((r) => r.collisions > 0).length,
          energyWh: mean(wh),
          coveragePerWh: mean(cov) / Math.max(1e-9, mean(wh)),
          reachedHalf: half.length,
          halfS: half.length ? mean(half) : null,
          ...fp,
        });
      }
  return cells;
}

if (process.argv.includes('--merge')) {
  const outPath = arg('out', 'docs/speed-study.json');
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
    checkpoint: SHIPPED_CHECKPOINT_PATH,
    seedsPerCell: SPEED_SEEDS,
    speeds: SPEEDS,
    momentum: MOMENTUM,
    footprint: FOOTPRINT_META,
    conditions: SPEED_CONDITIONS,
    pilots: SPEED_PILOTS,
    cells: summarize(rows),
    episodes: rows.map(withoutCurve),
  };
  writeFileSync(outPath, `${JSON.stringify(report, null, 2)}\n`);
  console.log(`wrote ${outPath} from ${rows.length} episodes`);
} else if (process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `speed-${shard}.jsonl`);
  const policy = loadPolicy(SHIPPED_CHECKPOINT_PATH);
  const env = await createEnv();
  // Optional subset, e.g. `--pilots plannerTrack,plannerBrakeTrack` when adding arms.
  const pilots = arg('pilots', '').split(',').filter(Boolean);
  const mine = jobs()
    .filter((j) => !pilots.length || pilots.includes(j.pilot))
    .filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    lines.push(JSON.stringify(await runJob(env, mine[i]!, policy)));
    if (i % 25 === 0) {
      writeFileSync(out, lines.join('\n') + '\n');
      console.log(`shard ${shard}: ${i + 1}/${mine.length} (${Math.round((Date.now() - t0) / 1000)} s)`);
    }
  }
  writeFileSync(out, lines.join('\n') + '\n');
  env.dispose();
  console.log(`shard ${shard}: done, ${mine.length} episodes in ${Math.round((Date.now() - t0) / 1000)} s`);
}

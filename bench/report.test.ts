// Measures the shipped policy and the simulator and writes docs/benchmarks.json.
// Run with `npm run bench`; `npm run docs` renders the JSON into docs/BENCHMARKS.md and the README.
// Nothing here is a pass/fail gate. It records what this machine measured.
import { createHash } from 'node:crypto';
import { execSync } from 'node:child_process';
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { availableParallelism, cpus } from 'node:os';
import { Worker } from 'node:worker_threads';
import * as esbuild from 'esbuild';
import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../src/gen/building';
import { parseCheckpoint, type Checkpoint } from '../src/rl/checkpoint';
import { Curriculum } from '../src/rl/curriculum';
import { InferPolicy } from '../src/rl/infer';
import { RunningNorm } from '../src/rl/normalize';
import { ACT_DIM, HIDDEN, OBS_DIM } from '../src/shared/constants';
import { RNG } from '../src/shared/rng';
import { createEnv, defaultEpisodeConfig } from '../src/sim/env';
import { heuristicAction } from '../src/sim/heuristic';
import { lightingPreset, type LightingPreset } from '../src/sim/lighting';

const CHECKPOINT = process.env.BENCH_CHECKPOINT ?? 'public/checkpoints/pretrained.json';
const OUT = process.env.BENCH_OUT ?? 'docs/benchmarks.json';

function seedList(name: string, fallback: number[]): number[] {
  const raw = process.env[name];
  if (!raw) return fallback;
  return raw.split(',').map((s) => Number(s.trim())).filter((n) => Number.isFinite(n));
}

// Stage 0 is a single small floor. Stage 3 adds hallways and more rooms. None of these seeds
// is drawn by the curriculum during training, so every building here is new to the policy.
const EASY = { stage: 0, seeds: seedList('BENCH_EASY_SEEDS', [101, 202, 303, 404, 505, 606, 707, 808]) };
const HARD = { stage: 3, seeds: seedList('BENCH_HARD_SEEDS', [4242, 5150, 6160, 7171, 8282, 9393]) };
const LIGHTING: LightingPreset[] = ['day', 'night-dark'];

type Pilot = (obs: Float32Array) => Float32Array;

/**
 * `browser` is the in-browser "Policy" pilot in src/ui/app.ts: RunningNorm.apply (which clips to ±5), then the
 * deterministic mean action. `raw` skips normalization, as tests/unit/shipped.test.ts does. Both are reported
 * because they do not fly the same.
 */
function loadPilot(checkpoint: Checkpoint, mode: 'browser' | 'raw'): Pilot {
  const hidden = checkpoint.hidden.length === 2 ? checkpoint.hidden : HIDDEN;
  const policy = new InferPolicy(checkpoint.obsDim || OBS_DIM, checkpoint.actDim || ACT_DIM, hidden, 1);
  policy.setWeights(checkpoint.weights.map((row) => Float32Array.from(row).buffer));
  const norm = new RunningNorm(policy.obsDim);
  if (checkpoint.obsMean.length === policy.obsDim) {
    norm.load({
      dim: policy.obsDim,
      count: Math.max(1, checkpoint.obsCount),
      mean: checkpoint.obsMean,
      var: checkpoint.obsVar,
      rewCount: 1,
      rewMean: 0,
      rewVar: checkpoint.rewVar || 1,
    });
  }
  const rng = new RNG(1);
  if (mode === 'raw') return (obs) => policy.actWith(obs, 1, rng, true).action;
  return (obs) => policy.actWith(norm.apply(obs, 1), 1, rng, true).action;
}

interface Episode {
  stage: number;
  seed: number;
  lighting: LightingPreset;
  coverage: number;
  collisions: number;
  steps: number;
}

async function fly(pilot: Pilot, stage: number, seed: number, lighting: LightingPreset): Promise<Episode> {
  const env = await createEnv();
  const params = new Curriculum().genParams(stage, new RNG(seed, `eval-${stage}`));
  const spec = generateBuilding(params);
  const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
  const cfg = defaultEpisodeConfig(stage, nRooms);
  cfg.maxSteps = stage <= 1 ? 1000 : 2400;
  cfg.stallSteps = 500;
  cfg.maxCollisions = 16;
  cfg.dynamics = 'kinematic';
  cfg.lighting = lightingPreset(lighting, nRooms);
  let obs = env.reset(spec, params.seed, cfg);
  let coverage = 0;
  let collisions = 0;
  let steps = 0;
  for (let t = 0; t < cfg.maxSteps; t++) {
    const step = env.step(pilot(obs));
    obs = step.obs;
    coverage = step.info.coverage;
    collisions = step.info.collisions;
    steps = t + 1;
    if (step.done) break;
  }
  return { stage, seed, lighting, coverage, collisions, steps };
}

async function sweep(pilot: Pilot): Promise<Episode[]> {
  const rows: Episode[] = [];
  for (const group of [EASY, HARD]) {
    for (const seed of group.seeds) {
      for (const lighting of LIGHTING) rows.push(await fly(pilot, group.stage, seed, lighting));
    }
  }
  return rows;
}

async function stepMs(dynamics: 'kinematic' | 'quad'): Promise<number> {
  const env = await createEnv();
  const spec = generateBuilding({ seed: 8, floors: 1, w: 16, d: 16, stage: 2 });
  const cfg = defaultEpisodeConfig(2, 1);
  cfg.dynamics = dynamics;
  env.reset(spec, 8, cfg);
  const action = new Float32Array([0.4, 0, 0, 0.1]);
  for (let i = 0; i < 40; i++) env.step(action);
  const n = dynamics === 'quad' ? 300 : 1000;
  const t0 = performance.now();
  for (let i = 0; i < n; i++) {
    if (env.step(action).done) env.reset(spec, 20 + i, cfg);
  }
  return (performance.now() - t0) / n;
}

function inferMs(batch: number): number {
  const p = new InferPolicy(OBS_DIM, ACT_DIM, HIDDEN, 1);
  const obs = new Float32Array(batch * OBS_DIM);
  for (let i = 0; i < obs.length; i++) obs[i] = (i % 17) / 17;
  for (let i = 0; i < 10; i++) p.act(obs, batch);
  const n = 200;
  const t0 = performance.now();
  for (let i = 0; i < n; i++) p.act(obs, batch);
  return (performance.now() - t0) / (n * batch);
}

async function workerRates(): Promise<{ workers: number; heuristic: number; policy: number }> {
  mkdirSync('node_modules/.cache', { recursive: true });
  const outfile = 'node_modules/.cache/frontier-env-thread.mjs';
  await esbuild.build({
    entryPoints: ['tests/perf/envThread.ts'],
    bundle: true,
    platform: 'node',
    format: 'esm',
    outfile,
    external: ['@dimforge/rapier3d-compat'],
  });
  const workers = Math.max(1, Math.min(4, availableParallelism()));
  const run = (mode: 'heuristic' | 'policy', steps: number) =>
    Promise.all(
      Array.from(
        { length: workers },
        (_, i) =>
          new Promise<{ stepsPerSec: number }>((resolve, reject) => {
            const worker = new Worker(new URL(`../${outfile}`, import.meta.url), {
              workerData: { envs: 2, steps, seed: 40 + i * 17, mode },
            });
            worker.once('message', (msg: { stepsPerSec: number }) => {
              void worker.terminate();
              resolve(msg);
            });
            worker.once('error', reject);
          }),
      ),
    );
  const heuristic = (await run('heuristic', 400)).reduce((s, r) => s + r.stepsPerSec, 0);
  const policy = (await run('policy', 128)).reduce((s, r) => s + r.stepsPerSec, 0);
  return { workers, heuristic, policy };
}

function git(cmd: string): string | null {
  try {
    return execSync(`git ${cmd}`, { encoding: 'utf8' }).trim();
  } catch {
    return null;
  }
}

describe('benchmark report', () => {
  it('measures coverage and throughput', async () => {
    const text = readFileSync(CHECKPOINT, 'utf8');
    const checkpoint = parseCheckpoint(text);
    const policy = await sweep(loadPilot(checkpoint, 'browser'));
    const policyRaw = await sweep(loadPilot(checkpoint, 'raw'));
    const heuristic = await sweep(heuristicAction);
    const throughput = {
      kinematicStepMs: await stepMs('kinematic'),
      quadStepMs: await stepMs('quad'),
      inferMsPerEnvBatch8: inferMs(8),
      ...(await workerRates()),
    };
    const report = {
      schema: 1,
      measuredAt: new Date().toISOString(),
      commit: git('rev-parse HEAD'),
      dirty: (git('status --porcelain') ?? '') !== '',
      machine: {
        node: process.version,
        cpu: cpus()[0]?.model ?? 'unknown',
        cores: availableParallelism(),
        ci: Boolean(process.env.CI),
      },
      checkpoint: {
        path: CHECKPOINT,
        sha256: createHash('sha256').update(text).digest('hex'),
        stage: checkpoint.stage,
        hidden: checkpoint.hidden,
      },
      protocol: {
        dynamics: 'kinematic',
        deterministic: true,
        maxSteps: { easy: 1000, hard: 2400 },
        stallSteps: 500,
        maxCollisions: 16,
        lighting: LIGHTING,
        easy: EASY,
        hard: HARD,
      },
      coverage: { policy, policyRaw, heuristic },
      throughput,
    };
    writeFileSync(OUT, `${JSON.stringify(report, null, 2)}\n`);
    console.info(`wrote ${OUT}`);
    expect(policy.length).toBe((EASY.seeds.length + HARD.seeds.length) * LIGHTING.length);
  }, 900_000);
});

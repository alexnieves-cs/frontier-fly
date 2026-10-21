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
import { evalEpisodeSetup, STAGE0_SEEDS, STAGE3_SEEDS } from '../src/eval/coverage';
import { generateBuilding } from '../src/gen/building';
import { parseCheckpoint, type Checkpoint, SHIPPED_CHECKPOINT_PATH } from '../src/rl/checkpoint';
import { InferPolicy } from '../src/rl/infer';
import { RunningNorm } from '../src/rl/normalize';
import { ACT_DIM, HIDDEN, OBS_DIM } from '../src/shared/constants';
import { RNG } from '../src/shared/rng';
import { createEnv, defaultEpisodeConfig } from '../src/sim/env';
import { frontierAction } from '../src/sim/frontier';
import { heuristicAction } from '../src/sim/heuristic';
import { waypointAction } from '../src/sim/waypointPilot';
import type { LightingPreset } from '../src/sim/lighting';

const CHECKPOINT = process.env.BENCH_CHECKPOINT ?? SHIPPED_CHECKPOINT_PATH;
const OUT = process.env.BENCH_OUT ?? 'docs/benchmarks.json';

function seedList(name: string, fallback: number[]): number[] {
  const raw = process.env[name];
  if (!raw) return fallback;
  return raw.split(',').map((s) => Number(s.trim())).filter((n) => Number.isFinite(n));
}

// The fixed eval protocol in src/eval/coverage.ts: stage 0 is a single small floor, stage 3 adds
// hallways and more rooms. None of these seeds is used in training or checkpoint selection.
const EASY = { stage: 0, seeds: seedList('BENCH_EASY_SEEDS', STAGE0_SEEDS) };
const HARD = { stage: 3, seeds: seedList('BENCH_HARD_SEEDS', STAGE3_SEEDS) };
const LIGHTING: LightingPreset[] = ['day', 'night-dark'];

type Env = Awaited<ReturnType<typeof createEnv>>;
type Pilot = (obs: Float32Array, env: Env) => Float32Array;

/**
 * `browser` is the in-browser "Policy" pilot in src/ui/app.ts: raw observations when the checkpoint's
 * normalizer is the identity, RunningNorm.apply (which clips to ±5) otherwise, then the deterministic mean
 * action. `raw` always skips normalization, as `npm run eval:coverage` does.
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
  const identity = checkpoint.obsMean.every((v) => v === 0) && checkpoint.obsVar.every((v) => v === 1);
  if (mode === 'raw' || identity) return (obs) => policy.actWith(obs, 1, rng, true).action;
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
  const { spec, envSeed, cfg } = evalEpisodeSetup(stage, seed, lighting);
  let obs = env.reset(spec, envSeed, cfg);
  let coverage = 0;
  let collisions = 0;
  let steps = 0;
  for (let t = 0; t < cfg.maxSteps; t++) {
    const step = env.step(pilot(obs, env));
    obs = step.obs;
    coverage = step.info.coverage;
    collisions = step.info.collisions;
    steps = t + 1;
    if (step.done) break;
  }
  env.dispose();
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
  const run = (mode: 'heuristic' | 'policy', steps: number, envs: number, extra: { pool?: boolean; warmup?: number } = {}) =>
    Promise.all(
      Array.from(
        { length: workers },
        (_, i) =>
          new Promise<{ stepsPerSec: number }>((resolve, reject) => {
            const worker = new Worker(new URL(`../${outfile}`, import.meta.url), {
              workerData: { envs, steps, seed: 40 + i * 17, mode, ...extra },
            });
            worker.once('message', (msg: { stepsPerSec: number }) => {
              void worker.terminate();
              resolve(msg);
            });
            worker.once('error', reject);
          }),
      ),
    );
  const heuristic = (await run('heuristic', 400, 2)).reduce((s, r) => s + r.stepsPerSec, 0);
  // The browser training loop: 8 envs per worker (one batched forward pass per step), resets on pooled worlds.
  const policy = (await run('policy', 512, 8, { pool: true, warmup: 128 })).reduce((s, r) => s + r.stepsPerSec, 0);
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
    const checkpoint = parseCheckpoint(readFileSync(CHECKPOINT));
    const policy = await sweep(loadPilot(checkpoint, 'browser'));
    const policyRaw = await sweep(loadPilot(checkpoint, 'raw'));
    const heuristic = await sweep(heuristicAction);
    // Stronger references: a controller that only follows the navigation block's waypoint (same
    // observation as the policy, no learning), and the privileged frontier teacher the policy imitates.
    const waypoint = await sweep(waypointAction);
    const teacher = await sweep((_obs, env) => {
      const s = env.snapshot();
      return frontierAction(env.building!, env.visitedGrid, s.pos, s.yaw);
    });
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
      // Only code and the checkpoint count. Generated docs and media are expected to be uncommitted here.
      dirty: (git('status --porcelain -- src public bench tests package.json') ?? '') !== '',
      machine: {
        node: process.version,
        cpu: cpus()[0]?.model ?? 'unknown',
        cores: availableParallelism(),
        ci: Boolean(process.env.CI),
      },
      checkpoint: {
        path: CHECKPOINT,
        sha256: createHash('sha256').update(readFileSync(CHECKPOINT)).digest('hex'),
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
      coverage: { policy, policyRaw, heuristic, waypoint, teacher },
      throughput,
    };
    writeFileSync(OUT, `${JSON.stringify(report, null, 2)}\n`);
    console.info(`wrote ${OUT}`);
    expect(policy.length).toBe((EASY.seeds.length + HARD.seeds.length) * LIGHTING.length);
  }, 900_000);
});

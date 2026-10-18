/**
 * Rollout throughput, env steps/s:
 *   (i)   env only, random actions, rebuild per episode (generateBuilding + Rapier world)
 *   (ii)  env + batched policy inference, rebuild per episode     (one thread)
 *   (iii) env + batched policy inference, WorldPool                (one thread)
 *   (iv)  NodeRolloutPool on worker_threads, WorldPool             (all cores)
 *   (iv-r) same, rebuild per episode
 * Workload: untrained policy, stage mix uniform over 0..4, workers/rollout.ts
 * training configs (short crash-heavy episodes, like early training).
 *
 *   npx vite-node scripts/bench-throughput.ts [--workers N] [--envs 8] [--steps 256] [--only i,ii,iii,iv,iv-r]
 */
import { availableParallelism } from 'node:os';
import { generateBuilding } from '../src/gen/building';
import { Curriculum } from '../src/rl/curriculum';
import { ACT_DIM, OBS_DIM } from '../src/shared/constants';
import { RNG } from '../src/shared/rng';
import { ExplorerEnv } from '../src/sim/env';
import { initPhysics } from '../src/sim/world';
import { NodeRolloutPool } from '../src/train/nodePool';
import { RolloutCore } from '../src/train/rolloutCore';
import { makeConfig } from '../src/workers/rollout';

const args = process.argv.slice(2);
const flag = (name: string, fallback: number) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? Number(args[i + 1]) : fallback;
};
const only = (() => {
  const i = args.indexOf('--only');
  return i >= 0 ? new Set(args[i + 1]!.split(',')) : null;
})();
const want = (k: string) => !only || only.has(k);
const workers = flag('workers', availableParallelism());
const envs = flag('envs', 8);
const steps = flag('steps', 256);
const mix = [1, 1, 1, 1, 1];
const rows: [string, string, string][] = [];

function envOnly(totalSteps: number): number {
  const curriculum = new Curriculum();
  const rng = new RNG(7, 'bench-env');
  const list = Array.from({ length: envs }, () => new ExplorerEnv());
  const reset = (env: ExplorerEnv) => {
    const stage = rng.int(5);
    const spec = generateBuilding(curriculum.genParams(stage, rng));
    const seed = rng.int(1e9);
    const nRooms = spec.floors.reduce((n, f) => n + f.rooms.length, 0);
    env.reset(spec, seed, makeConfig(stage, seed, nRooms, curriculum.dynamicsFor(stage, new RNG(seed, 'dyn'))));
  };
  list.forEach(reset);
  const action = new Float32Array(ACT_DIM);
  const run = (n: number) => {
    for (let t = 0; t < n; t++) {
      for (const env of list) {
        for (let j = 0; j < ACT_DIM; j++) action[j] = rng.range(-1, 1);
        if (env.step(action).done) reset(env);
      }
    }
  };
  run(Math.ceil(totalSteps / envs / 4));
  const t0 = performance.now();
  const per = Math.ceil(totalSteps / envs);
  run(per);
  const rate = (per * envs * 1000) / (performance.now() - t0);
  list.forEach((e) => e.dispose());
  return rate;
}

async function single(worlds: 'pool' | 'rebuild'): Promise<number> {
  const core = await RolloutCore.create({ nEnvs: envs, seed: 3, worldPool: { perStage: 16 } });
  core.collectPpo({ stepsPerEnv: Math.ceil(steps / 2), stageMix: mix, seed: 1, worlds });
  let total = 0;
  let ms = 0;
  let episodes = 0;
  for (let k = 0; k < 3; k++) {
    const r = core.collectPpo({ stepsPerEnv: steps, stageMix: mix, seed: 2 + k, worlds });
    total += r.n;
    ms += r.ms;
    episodes += r.episodes.length;
  }
  core.dispose();
  console.log(`  ${worlds}: mean episode length ${(total / Math.max(1, episodes)).toFixed(0)} steps`);
  return (total * 1000) / ms;
}

async function parallel(worlds: 'pool' | 'rebuild') {
  const pool = new NodeRolloutPool({ workers, envsPerWorker: envs, seed: 3, worldPool: { perStage: 16, churn: 0.05 } });
  await pool.init();
  await pool.collectPpo({ stepsPerEnv: Math.ceil(steps / 2), stageMix: mix, seed: 1, worlds });
  let total = 0;
  let wall = 0;
  const perWorker = new Array(workers).fill(0);
  const reps = 3;
  for (let k = 0; k < reps; k++) {
    const r = await pool.collectPpo({ stepsPerEnv: steps, stageMix: mix, seed: 2 + k, worlds });
    total += r.n;
    wall += r.wallMs;
    r.perWorker.forEach((w, i) => (perWorker[i] += w.stepsPerSec / reps));
  }
  await pool.close();
  return { total: (total * 1000) / wall, perWorker };
}

async function main() {
  await initPhysics();
  console.log(`OBS_DIM ${OBS_DIM}, ${envs} envs per thread, ${steps} steps per env per collect, ${availableParallelism()} cores`);
  if (want('i')) rows.push(['(i) env only, rebuild/episode, 1 thread', envOnly(envs * steps).toFixed(0), '']);
  if (want('ii')) rows.push(['(ii) env + policy, rebuild/episode, 1 thread', (await single('rebuild')).toFixed(0), '']);
  if (want('iii')) rows.push(['(iii) env + policy, WorldPool, 1 thread', (await single('pool')).toFixed(0), '']);
  for (const [key, worlds] of [['iv', 'pool'], ['iv-r', 'rebuild']] as const) {
    if (!want(key)) continue;
    const r = await parallel(worlds);
    rows.push([
      `(${key}) NodeRolloutPool ${workers}x${envs}, ${worlds === 'pool' ? 'WorldPool' : 'rebuild/episode'}`,
      r.total.toFixed(0),
      r.perWorker.map((v) => v.toFixed(0)).join(' / '),
    ]);
  }
  console.log('| scenario | steps/s total | steps/s per worker |');
  console.log('|---|---|---|');
  for (const [a, b, c] of rows) console.log(`| ${a} | ${b} | ${c} |`);
}

void main();

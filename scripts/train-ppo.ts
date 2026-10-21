/**
 * PPO fine-tuning of the shipped DAgger policy on its own flights, with a
 * privileged critic and an optional GRU memory (src/rl/memPolicy.ts,
 * src/rl/seqPpo.ts, src/train/ppoJob.ts). Cloud CPU: rollouts on every core,
 * the learner on tfjs-node when it is installed.
 *
 * Usage: npm run train:ppo -- --out /tmp/ppo.bin [--init public/checkpoints/pretrained.bin]
 *          [--mem 0] [--updates 400] [--steps-per-env 512] [--seq-len 16] [--envs 8]
 *          [--lr 1e-5] [--critic-lr 3e-4] [--clip 0.1] [--epochs 3] [--seq-batch 32]
 *          [--bc 0.05] [--ppo 1] [--log-std -2.3] [--critic-warmup 8] [--imitate 0]
 *          [--eval-every 20] [--reward-scale 0.1] [--stage-mix 0.2,0.1,0.2,0.5] [--night 0.25] [--replay 300000]
 *        npm run train:ppo -- --validate-only ckpt.bin [--set tune|report]
 *
 * The defaults are the recipe of the shipped checkpoint's first 400 updates;
 * it then ran 400 more with --init on that run's best checkpoint and --seed 47
 * (docs/REPORT.md, "Fine-tuning on its own flights"). --bc a:b anneals the BC
 * weight linearly over the PPO updates.
 *
 * --imitate N runs N rounds of on-policy imitation (BC on the teacher's labels
 * of the student's own states, i.e. DAgger with beta 0) before PPO; with --mem
 * it is how the fresh memory first learns something.
 *
 * Checkpoints are picked on the tuning block 66001-66299 (never the eval seeds
 * 101/202/303/404 and 4242/5150/6160, never the report block 68001-68299).
 */
import { readFileSync, writeFileSync } from 'node:fs';
import { availableParallelism } from 'node:os';
import * as tf from '@tensorflow/tfjs';
import { checkpointMismatch, parseCheckpoint, serializeCheckpointFor, type Checkpoint } from '../src/rl/checkpoint';
import { MemPolicy } from '../src/rl/memPolicy';
import { computeGae } from '../src/rl/ppo';
import { SeqPpoTrainer, SeqReplay, type SeqBatch } from '../src/rl/seqPpo';
import { ACT_DIM, OBS_DIM, OBS_VERSION } from '../src/shared/constants';
import { NodeRolloutPool } from '../src/train/nodePool';
import { PRIV_DIM, type SeqEvalResult, type SeqRolloutResult } from '../src/train/ppoJob';
import type { EvalJob } from '../src/train/rolloutCore';
import { useFastestBackend } from './lib/tfBackend';

const args = process.argv.slice(2);
const arg = (name: string) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const num = (name: string, fallback: number) => (arg(name) == null ? fallback : Number(arg(name)));
const pct = (v: number) => `${(v * 100).toFixed(1)}%`;
const mean = (xs: number[]) => xs.reduce((s, v) => s + v, 0) / Math.max(1, xs.length);

/** "a:b" linearly from a to b over the PPO updates, or a constant. */
function schedule(name: string, fallback: string): (frac: number) => number {
  const [a, b] = (arg(name) ?? fallback).split(':').map(Number);
  return (frac) => (b == null || Number.isNaN(b) ? a! : a! + (b - a!) * Math.min(1, Math.max(0, frac)));
}

/** Held-out episodes. 'tune' picks checkpoints; 'report' is only ever reported. */
export function seedSet(name: 'tune' | 'report'): EvalJob[] {
  const base = name === 'tune' ? 66_000 : 68_000;
  const jobs: EvalJob[] = [];
  for (let i = 1; i <= 32; i++) jobs.push({ stage: 3, seed: base + i, lighting: 'day' });
  for (let i = 1; i <= 8; i++) jobs.push({ stage: 3, seed: base + 100 + i, lighting: 'night-dark' });
  for (let i = 1; i <= 6; i++) jobs.push({ stage: 0, seed: base + 200 + i, lighting: 'day' });
  for (let i = 1; i <= 2; i++) jobs.push({ stage: 0, seed: base + 250 + i, lighting: 'night-dark' });
  return jobs;
}

interface Score {
  small: number;
  large: number;
  largeNight: number;
  collisions: number;
  score: number;
}

async function evaluate(pool: NodeRolloutPool, workers: number, policy: MemPolicy, jobs: EvalJob[]): Promise<Score> {
  const spec = { weights: policy.getWeights(), hidden: [...policy.hidden], memDim: policy.memDim };
  // Longest episodes first, round robin, so workers finish together.
  const order = jobs.map((job, i) => ({ job, i })).sort((a, b) => b.job.stage - a.job.stage);
  const per: { job: EvalJob; i: number }[][] = Array.from({ length: workers }, () => []);
  order.forEach((x, k) => per[k % workers]!.push(x));
  const parts = (await pool.runJobAll('seqEval', (w: number) => ({ policy: spec, jobs: per[w]!.map((x) => x.job) }))) as SeqEvalResult[][];
  const results = new Array<SeqEvalResult>(jobs.length);
  per.forEach((list, w) => list.forEach((x, k) => (results[x.i] = parts[w]![k]!)));
  const pick = (f: (j: EvalJob) => boolean) => results.filter((_, i) => f(jobs[i]!));
  const small = mean(pick((j) => j.stage <= 1).map((r) => r.coverage));
  const large = mean(pick((j) => j.stage >= 2).map((r) => r.coverage));
  const largeNight = mean(pick((j) => j.stage >= 2 && j.lighting !== 'day').map((r) => r.coverage));
  const collisions = mean(pick((j) => j.stage >= 2).map((r) => r.collisions));
  // Same weighting as train-indoor's validation: the stage-3 bar is the hard one.
  return { small, large, largeNight, collisions, score: 0.4 * small + 0.6 * large };
}

const fmt = (s: Score) =>
  `small ${pct(s.small)} large ${pct(s.large)} (night ${pct(s.largeNight)}) coll ${s.collisions.toFixed(2)} score ${pct(s.score)}`;

function checkpointOf(policy: MemPolicy, base: Checkpoint): Checkpoint {
  return {
    ...base,
    version: 1,
    obsVersion: OBS_VERSION,
    hidden: [...policy.hidden],
    weights: policy.getWeights().map((b) => Array.from(new Float32Array(b))),
    ...(policy.memDim > 0 ? { memDim: policy.memDim } : { memDim: undefined }),
  };
}

function write(path: string, c: Checkpoint): void {
  const clean = { ...c };
  if (!clean.memDim) delete clean.memDim;
  writeFileSync(path, serializeCheckpointFor(path, clean));
}

async function main() {
  const started = Date.now();
  const backend = await useFastestBackend();
  await tf.setBackend(backend);
  await tf.ready();
  const workers = num('workers', Math.max(1, availableParallelism()));
  const envs = num('envs', 8);
  const pool = new NodeRolloutPool({
    workers,
    envsPerWorker: envs,
    seed: num('seed', 23),
    worldPool: { stages: [0, 1, 2, 3], perStage: num('per-stage', 32), churn: 0.15 },
    jobModules: ['src/train/ppoJob.ts'],
    autoDagger: false,
  });
  await pool.init();

  const validateOnly = arg('validate-only');
  if (validateOnly) {
    const c = parseCheckpoint(readFileSync(validateOnly));
    const set = (arg('set') ?? 'tune') as 'tune' | 'report';
    const s = await evaluate(pool, workers, MemPolicy.fromCheckpoint(c), seedSet(set));
    console.log(`${validateOnly} [${set}] ${fmt(s)}`);
    await pool.close();
    return;
  }

  const out = arg('out') ?? '/tmp/frontier-fly-ppo.bin';
  const stem = out.replace(/\.(bin|json)$/, '');
  const init = parseCheckpoint(readFileSync(arg('init') ?? 'public/checkpoints/pretrained.bin'));
  const mismatch = checkpointMismatch(init);
  if (mismatch) throw new Error(mismatch);
  const memDim = num('mem', init.memDim ?? 0);
  const policy = MemPolicy.fromCheckpoint(init, memDim, num('seed', 23));
  const trainer = new SeqPpoTrainer(policy, PRIV_DIM, { lr: num('lr', 1e-5), criticLr: num('critic-lr', 3e-4), seed: num('seed', 23) });
  const logStd = num('log-std', -2.3);
  trainer.setLogStd(new Array(ACT_DIM).fill(logStd));
  console.log(`backend ${backend}, ${workers} workers x ${envs} envs, mem ${memDim}, init ${arg('init') ?? 'shipped'}`);

  const tune = seedSet('tune');
  let best = { label: 'init', score: await evaluate(pool, workers, policy, tune), weights: policy.getWeights() };
  console.log(`init ${fmt(best.score)}  (${((Date.now() - started) / 1000).toFixed(0)} s)`);

  const T = num('steps-per-env', 512);
  const L = num('seq-len', 16);
  const imitate = num('imitate', 0);
  const warmup = num('critic-warmup', 8);
  const updates = num('updates', 400);
  const evalEvery = num('eval-every', 20);
  const rewardScale = num('reward-scale', 0.1);
  const stageMix = (arg('stage-mix') ?? '0.2,0.1,0.2,0.5').split(',').map(Number);
  const bc = schedule('bc', '0.05');
  const ppoCoef = num('ppo', 1);
  const total = imitate + warmup + updates;
  let resetEnvs = true;
  // BC replay: every labelled sequence collected in this run, reservoir-sampled (--replay 0 trains BC on the fresh batch only).
  const replayRows = num('replay', 300_000);
  const replay = replayRows > 0 ? new SeqReplay(Math.floor(replayRows / L), L, OBS_DIM, ACT_DIM, memDim, num('seed', 23)) : null;
  for (let it = 0; it < total; it++) {
    const t0 = Date.now();
    const phase = it < imitate ? 'imitate' : it < imitate + warmup ? 'critic' : 'ppo';
    const frac = phase === 'ppo' ? (it - imitate - warmup) / Math.max(1, updates - 1) : 0;
    const spec = { weights: policy.getWeights(), hidden: [...policy.hidden], memDim };
    const parts = (await pool.runJobAll('seqRollout', {
      policy: spec,
      stepsPerEnv: T,
      seqLen: L,
      stageMix,
      nightFraction: num('night', 0.25),
      seed: 1000 + it * 7919,
      resetEnvs,
    })) as SeqRolloutResult[];
    resetEnvs = false;
    const tCollect = (Date.now() - t0) / 1000;

    // Concatenate workers: env-major rows are already sequence-major (T / L sequences per env).
    const cat = <K extends keyof SeqRolloutResult>(key: K, Ctor: Float32ArrayConstructor | Uint8ArrayConstructor) => {
      const arrays = parts.map((p) => p[key] as unknown as Float32Array | Uint8Array);
      const o = new Ctor(arrays.reduce((n, a) => n + a.length, 0));
      let at = 0;
      for (const a of arrays) {
        o.set(a, at);
        at += a.length;
      }
      return o;
    };
    const obs = cat('obs', Float32Array) as Float32Array;
    const priv = cat('priv', Float32Array) as Float32Array;
    const rewards = cat('rewards', Float32Array) as Float32Array;
    const dones = cat('dones', Uint8Array) as Uint8Array;
    const lastObs = cat('lastObs', Float32Array) as Float32Array;
    const lastPriv = cat('lastPriv', Float32Array) as Float32Array;
    const E = parts.reduce((n, p) => n + p.nEnvs, 0);
    const n = E * T;
    const values = trainer.values(obs, priv, n);
    const boot = trainer.values(lastObs, lastPriv, E);
    const advantages = new Float32Array(n);
    const returns = new Float32Array(n);
    for (let e = 0; e < E; e++) {
      const sl = (a: Float32Array | Uint8Array) => a.subarray(e * T, (e + 1) * T);
      const r = Float32Array.from(sl(rewards) as Float32Array, (v) => v * rewardScale);
      const g = computeGae({ rewards: r, values: sl(values) as Float32Array, dones: sl(dones) as Uint8Array, truncated: new Uint8Array(T), nextValue: boot[e]! });
      advantages.set(g.advantages, e * T);
      returns.set(g.returns, e * T);
    }
    const batch: SeqBatch = {
      obs,
      priv,
      actions: cat('actions', Float32Array) as Float32Array,
      teacher: cat('teacher', Float32Array) as Float32Array,
      logp: cat('logp', Float32Array) as Float32Array,
      advantages,
      returns,
      starts: cat('starts', Uint8Array) as Uint8Array,
      state0: cat('state0', Float32Array) as Float32Array,
      nSeq: n / L,
      seqLen: L,
    };
    replay?.add(batch);
    const m = trainer.update(batch, {
      epochs: num('epochs', 3),
      seqPerBatch: num('seq-batch', 32),
      clipEps: num('clip', 0.1),
      ppoCoef: phase === 'ppo' ? ppoCoef : 0,
      bcCoef: phase === 'imitate' ? 1 : phase === 'ppo' ? bc(frac) : 0,
      entropyCoef: num('entropy', 0),
      criticOnly: phase === 'critic',
      bcReplay: replay?.view(),
    });
    const eps = parts.flatMap((p) => p.episodes);
    const hard = eps.filter((e) => e.stage === 3);
    console.log(
      `${phase} ${it} eps ${eps.length} hardCov ${pct(mean(hard.map((e) => e.coverage)))} (${hard.length}) ` +
        `coll ${mean(hard.map((e) => e.collisions)).toFixed(2)} pi ${m.policyLoss.toFixed(4)} bc ${m.bcLoss.toFixed(4)} ` +
        `v ${m.valueLoss.toFixed(4)} kl ${m.kl.toFixed(4)} clip ${m.clipFrac.toFixed(3)} ` +
        `${(n / tCollect).toFixed(0)} steps/s collect ${tCollect.toFixed(0)}s total ${((Date.now() - t0) / 1000).toFixed(0)}s`,
    );
    const trained = it - imitate - warmup + 1;
    const evalNow = phase === 'imitate' ? (it + 1) % Math.max(1, Math.min(evalEvery, imitate)) === 0 : phase === 'ppo' && (trained % evalEvery === 0 || it === total - 1);
    if (evalNow) {
      const s = await evaluate(pool, workers, policy, tune);
      const label = phase === 'imitate' ? `imitate ${it + 1}` : `ppo ${trained}`;
      console.log(`  val ${label} ${fmt(s)}`);
      write(`${stem}.${phase === 'imitate' ? 'i' : 'u'}${phase === 'imitate' ? it + 1 : trained}.bin`, checkpointOf(policy, init));
      if (s.score > best.score.score) best = { label, score: s, weights: policy.getWeights() };
    }
  }
  policy.setWeights(best.weights);
  write(out, checkpointOf(policy, init));
  console.log(`best ${best.label} ${fmt(best.score)} -> ${out}`);
  await pool.close();
  console.log(`done in ${((Date.now() - started) / 1000).toFixed(0)} s, obs ${OBS_DIM}`);
}

void main();

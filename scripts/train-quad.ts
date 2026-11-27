/**
 * Train the quad student (docs/QUAD_STUDENT.md): DAgger from the default quad pilot on randomized
 * quadrotor episodes, then (optionally) more rounds at beta 0. Validation flies the student alone,
 * with no braking filter or guard, on practice seeds only.
 *
 * Usage: npm run train:quad -- --out-dir /tmp/quad [--iters 12] [--episodes 64] [--first 96]
 *          [--beta0 1] [--beta-decay 0.6] [--noise 0.15] [--epochs 2] [--capacity 1500000]
 *          [--lr 1e-3] [--hidden 512,256] [--val 16] [--init ckpt.bin] [--workers N] [--seed 1]
 *
 * Seeds (docs/QUAD_STUDENT.md, roadmap seed ledger): training buildings 170001-179999, practice
 * validation 180001-180400 (day and night). Nothing here flies the gate or report blocks.
 */
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { availableParallelism } from 'node:os';
import { join } from 'node:path';
import { ACT_DIM, OBS_DIM, OBS_VERSION } from '../src/shared/constants';
import { RNG } from '../src/shared/rng';
import { parseCheckpoint, serializeCheckpointFor, type Checkpoint } from '../src/rl/checkpoint';
import { InferPolicy } from '../src/rl/infer';
import { TfTrainer } from '../src/rl/model';
import { paddedWeights, STUDENT_DIM, STUDENT_HIDDEN } from '../src/rl/quadStudent';
import { NodeRolloutPool } from '../src/train/nodePool';
import type { PickerEpisode } from '../src/train/pickerJob';
import { HEAD, HEAD_ROW, MAPS, type DaggerChunk, type DaggerPayload, type EvalEpisode, type RolloutChunk, type RolloutPayload, type StudentWeights } from '../src/train/quadDistillJob';
import { useFastestBackend } from './lib/tfBackend';

const args = process.argv.slice(2);
const arg = (name: string) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const num = (name: string, fallback: number) => (arg(name) == null ? fallback : Number(arg(name)));
const pct = (v: number) => `${(v * 100).toFixed(1)}%`;

export const TRAIN_SEEDS = [170001, 179999] as const;
export const PRACTICE_BASE = 180001;

/** Aggregated DAgger states, the oldest dropped first once full. */
class Dataset {
  readonly head: Float32Array;
  readonly maps: Uint8Array;
  readonly labels: Float32Array;
  readonly weights: Float32Array;
  n = 0;
  private next = 0;
  constructor(readonly capacity: number) {
    this.head = new Float32Array(capacity * HEAD_ROW);
    this.maps = new Uint8Array(capacity * MAPS);
    this.labels = new Float32Array(capacity * ACT_DIM);
    this.weights = new Float32Array(capacity);
  }
  add(c: DaggerChunk): void {
    for (let r = 0; r < c.n; r++) {
      const k = this.next;
      this.head.set(c.head.subarray(r * HEAD_ROW, (r + 1) * HEAD_ROW), k * HEAD_ROW);
      this.maps.set(c.maps.subarray(r * MAPS, (r + 1) * MAPS), k * MAPS);
      this.labels.set(c.labels.subarray(r * ACT_DIM, (r + 1) * ACT_DIM), k * ACT_DIM);
      this.weights[k] = c.weights[r]!;
      this.next = (k + 1) % this.capacity;
      this.n = Math.min(this.capacity, this.n + 1);
    }
  }
  /** Rows `idx` as full student inputs, labels and weights. */
  gather(idx: Int32Array): { obs: Float32Array; act: Float32Array; w: Float32Array } {
    const m = idx.length;
    const obs = new Float32Array(m * STUDENT_DIM);
    const act = new Float32Array(m * ACT_DIM);
    const w = new Float32Array(m);
    for (let i = 0; i < m; i++) {
      const k = idx[i]!;
      const o = i * STUDENT_DIM;
      const h = k * HEAD_ROW;
      for (let j = 0; j < HEAD; j++) obs[o + j] = this.head[h + j]!;
      for (let j = 0; j < MAPS; j++) obs[o + HEAD + j] = this.maps[k * MAPS + j]! / 255;
      for (let j = 0; j < STUDENT_DIM - OBS_DIM; j++) obs[o + OBS_DIM + j] = this.head[h + HEAD + j]!;
      act.set(this.labels.subarray(k * ACT_DIM, (k + 1) * ACT_DIM), i * ACT_DIM);
      w[i] = this.weights[k]!;
    }
    return { obs, act, w };
  }
}

function trainEpisodes(rng: RNG, n: number): PickerEpisode[] {
  const out: PickerEpisode[] = [];
  for (let i = 0; i < n; i++) {
    const seed = TRAIN_SEEDS[0] + Math.floor(rng.next() * (TRAIN_SEEDS[1] - TRAIN_SEEDS[0] + 1));
    out.push({ stage: 3, seed, lighting: rng.next() < 0.5 ? 'day' : 'night-dark', quad: 1 });
  }
  return out;
}

export function practiceEpisodes(n: number, base = PRACTICE_BASE): PickerEpisode[] {
  const out: PickerEpisode[] = [];
  for (let i = 0; i < n; i++) for (const lighting of ['day', 'night-dark'] as const) out.push({ stage: 3, seed: base + i, lighting, quad: 1 });
  return out;
}

function split<T>(xs: T[], k: number): T[][] {
  const out: T[][] = Array.from({ length: k }, () => []);
  xs.forEach((x, i) => out[i % k]!.push(x));
  return out;
}

export async function evalEpisodes(pool: NodeRolloutPool, workers: number, eps: PickerEpisode[], student: StudentWeights | null, shield = false): Promise<EvalEpisode[]> {
  const parts = await Promise.all(split(eps, workers).map((list, w) => (list.length ? pool.runJob<EvalEpisode[]>('quadEval', { episodes: list, student, shield }, w) : Promise.resolve([]))));
  return parts.flat();
}

export function summary(rs: EvalEpisode[]): { coverage: number; crashes: number; n: number; score: number; interventionRate: number } {
  const n = rs.length;
  const steps = rs.reduce((a, r) => a + r.steps, 0);
  const interventionRate = rs.reduce((a, r) => a + r.interventions, 0) / Math.max(1, steps);
  const coverage = rs.reduce((a, r) => a + r.coverage, 0) / Math.max(1, n);
  const crashes = rs.filter((r) => r.crashed).length;
  // Selection score: a crash costs a quarter of a building.
  return { coverage, crashes, n, score: coverage - (0.25 * crashes) / Math.max(1, n), interventionRate };
}

function toCheckpoint(policy: InferPolicy): Checkpoint {
  return {
    version: 1,
    obsVersion: OBS_VERSION,
    obsDim: STUDENT_DIM,
    actDim: ACT_DIM,
    hidden: [...policy.hidden],
    weights: policy.getWeights().map((b) => Array.from(new Float32Array(b))),
    obsMean: [],
    obsVar: [],
    obsCount: 0,
    rewVar: 1,
    stage: 3,
  };
}

/** Rows of a rollout chunk as full student inputs. */
function unpack(c: DaggerChunk, obs: Float32Array, offset: number): void {
  for (let r = 0; r < c.n; r++) {
    const o = (offset + r) * STUDENT_DIM;
    const h = r * HEAD_ROW;
    for (let j = 0; j < HEAD; j++) obs[o + j] = c.head[h + j]!;
    for (let j = 0; j < MAPS; j++) obs[o + HEAD + j] = c.maps[r * MAPS + j]! / 255;
    for (let j = 0; j < STUDENT_DIM - OBS_DIM; j++) obs[o + OBS_DIM + j] = c.head[h + HEAD + j]!;
  }
}

/**
 * PPO from a distilled student: stochastic on-policy quad episodes, crash penalty, GAE, and an
 * imitation anchor on the expert's labels for the same states (weight decaying to bcEnd).
 */
async function reinforce(pool: NodeRolloutPool, workers: number, policy: InferPolicy, trainer: TfTrainer, outDir: string, hidden: number[], rng: RNG, val: PickerEpisode[]) {
  const updates = num('rl', 0);
  const perUpdate = num('rl-episodes', 16);
  const gamma = num('gamma', 0.995);
  const lambda = num('lambda', 0.95);
  const [bc0, bc1] = (arg('bc') ?? '1:0.1').split(':').map(Number);
  const warmup = num('critic-warmup', 8);
  const valEvery = num('val-every', 5);
  // --shield: train behind the braking shield (with a penalty for every correction it makes).
  const shield = args.includes('--shield');
  // Exploration noise: a fixed state-independent log std (trained by PPO afterwards).
  const w = policy.getWeights();
  new Float32Array(w[6]!).fill(num('log-std', -2));
  policy.setWeights(w);
  trainer.pushWeights();
  trainer.setLearningRate(num('lr', 1e-4));
  const log: unknown[] = [];
  const s0 = summary(await evalEpisodes(pool, workers, val, { hidden, weights: policy.getWeights() }));
  console.log(`rl start: alone ${pct(s0.coverage)} crashes ${s0.crashes}/${s0.n} score ${pct(s0.score)}`);
  let best = s0.score;
  writeFileSync(join(outDir, 'best.bin'), serializeCheckpointFor('x.bin', toCheckpoint(policy)));
  for (let u = 1; u <= updates; u++) {
    const t0 = Date.now();
    const eps = trainEpisodes(rng, perUpdate);
    const student = { hidden, weights: policy.getWeights() };
    const chunks = await Promise.all(
      split(eps, workers).map((list, k) =>
        pool.runJob<RolloutChunk>('quadRollout', { episodes: list, student, crashPenalty: num('crash-penalty', -40), rewardScale: num('reward-scale', 0.1), labels: true, shield, shieldPenalty: num('shield-penalty', 2), seed: u * 1000 + k } satisfies RolloutPayload, k),
      ),
    );
    const n = chunks.reduce((a, c) => a + c.n, 0);
    const obs = new Float32Array(n * STUDENT_DIM);
    const actions = new Float32Array(n * ACT_DIM);
    const labels = new Float32Array(n * ACT_DIM);
    const logp = new Float32Array(n);
    const adv = new Float32Array(n);
    const ret = new Float32Array(n);
    let off = 0;
    let crashes = 0;
    let cov = 0;
    let neps = 0;
    for (const c of chunks) {
      unpack(c, obs, off);
      actions.set(c.actions, off * ACT_DIM);
      labels.set(c.labels, off * ACT_DIM);
      logp.set(c.logp, off);
      // GAE per chunk; every episode ends inside its chunk (done = terminal, value 0 after).
      let next = 0;
      let gae = 0;
      for (let i = c.n - 1; i >= 0; i--) {
        if (c.dones[i]) {
          next = 0;
          gae = 0;
        }
        const delta = c.rewards[i]! + gamma * next - c.values[i]!;
        gae = delta + gamma * lambda * gae;
        adv[off + i] = gae;
        ret[off + i] = gae + c.values[i]!;
        next = c.values[i]!;
      }
      for (const e of c.episodes) {
        crashes += e.crashed ? 1 : 0;
        cov += e.coverage;
        neps++;
      }
      off += c.n;
    }
    const frac = (u - 1) / Math.max(1, updates - 1);
    const bcCoef = bc0! + (bc1! - bc0!) * frac;
    const valueOnly = u <= warmup;
    const m = await trainer.update(
      { obs, actions, logp, advantages: adv, returns: ret, n },
      { epochs: num('ppo-epochs', 3), minibatch: num('minibatch', 1024), clipEps: num('clip', 0.1), entropyCoef: 0, valueOnly, anchor: valueOnly ? undefined : { bc: { obs, actions: labels, n }, bcCoef } },
    );
    const entry: Record<string, unknown> = { update: u, steps: n, rolloutCoverage: cov / neps, crashes, episodes: neps, bcCoef, ...m, seconds: (Date.now() - t0) / 1000 };
    let line = `rl ${u} steps ${n} rollout ${pct(cov / neps)} crash ${crashes}/${neps} pi ${m.policyLoss.toFixed(3)} v ${m.valueLoss.toFixed(3)} kl ${m.kl.toFixed(4)} bc ${(m.bcLoss ?? 0).toFixed(3)}x${bcCoef.toFixed(2)}${valueOnly ? ' (critic warm-up)' : ''}`;
    if (u % valEvery === 0 || u === updates) {
      const s = summary(await evalEpisodes(pool, workers, val, { hidden, weights: policy.getWeights() }));
      entry.val = s;
      line += ` | alone ${pct(s.coverage)} crashes ${s.crashes}/${s.n} score ${pct(s.score)}`;
      // Shielded runs are picked on the shielded score.
      let pick = s.score;
      if (shield) {
        const g = summary(await evalEpisodes(pool, workers, val, { hidden, weights: policy.getWeights() }, true));
        entry.valShield = g;
        line += ` | shielded ${pct(g.coverage)} crashes ${g.crashes}/${g.n} interventions ${pct(g.interventionRate)}`;
        pick = g.score;
      }
      const ck = toCheckpoint(policy);
      writeFileSync(join(outDir, 'last.bin'), serializeCheckpointFor('x.bin', ck));
      if (pick > best) {
        best = pick;
        writeFileSync(join(outDir, 'best.bin'), serializeCheckpointFor('x.bin', ck));
      }
    }
    log.push(entry);
    console.log(`${line} ${((Date.now() - t0) / 1000).toFixed(0)}s`);
    writeFileSync(join(outDir, 'log.json'), JSON.stringify({ args, log }, null, 1));
  }
  console.log(`rl best practice score ${pct(best)}`);
}

async function main() {
  const started = Date.now();
  const backend = await useFastestBackend();
  const workers = num('workers', Math.max(1, availableParallelism()));
  const outDir = arg('out-dir') ?? '/tmp/frontier-fly-quad';
  mkdirSync(outDir, { recursive: true });
  const hidden = (arg('hidden') ?? STUDENT_HIDDEN.join(',')).split(',').map(Number);
  const seed = num('seed', 1);
  const rng = new RNG(seed, 'train-quad');
  const pool = new NodeRolloutPool({ workers, envsPerWorker: 1, seed, worldPool: { stages: [0], perStage: 1 }, jobModules: ['src/train/quadDistillJob.ts'], autoDagger: false });
  await pool.init();

  const policy = new InferPolicy(STUDENT_DIM, ACT_DIM, hidden, seed);
  const init = arg('init');
  if (init) policy.setWeights(paddedWeights(parseCheckpoint(readFileSync(init))));
  const trainer = new TfTrainer(policy, num('lr', 1e-3));
  await trainer.ready();
  const data = new Dataset(num('capacity', 1_500_000));
  const iters = num('iters', 12);
  const perIter = num('episodes', 64);
  const first = num('first', 96);
  const beta0 = num('beta0', 1);
  const betaDecay = num('beta-decay', 0.6);
  const betaMin = num('beta-min', 0);
  const noise = num('noise', 0.15);
  const epochs = num('epochs', 2);
  const chunk = num('chunk', 65536);
  // Per-dimension loss weights [vx, vy, vz, yaw]: the small vertical and yaw labels matter more than their size.
  const actionWeights = (arg('action-weights') ?? '1,1,4,2').split(',').map(Number);
  const val = practiceEpisodes(num('val', 16));
  console.log(`backend ${backend}, ${workers} workers, hidden ${hidden}, ${val.length} practice flights, init ${init ?? 'none'}`);

  if (args.includes('--expert-baseline')) {
    const s = summary(await evalEpisodes(pool, workers, val, null));
    console.log(`expert on practice: ${pct(s.coverage)} crashes ${s.crashes}/${s.n}`);
  }

  if (num('rl', 0) > 0 && iters === 0) {
    if (!init) throw new Error('--rl with --iters 0 needs --init (a distilled student)');
    await reinforce(pool, workers, policy, trainer, outDir, hidden, rng, val);
    console.log(`done in ${((Date.now() - started) / 60000).toFixed(1)} min`);
    await pool.close();
    return;
  }
  const log: unknown[] = [];
  let best = -Infinity;
  for (let it = 0; it <= iters; it++) {
    const t0 = Date.now();
    const beta = it === 0 && !init ? 1 : Math.max(betaMin, beta0 * betaDecay ** it);
    const eps = trainEpisodes(rng, it === 0 ? first : perIter);
    const student: StudentWeights | null = beta >= 1 ? null : { hidden, weights: policy.getWeights() };
    const payload = (list: PickerEpisode[], w: number): DaggerPayload => ({
      episodes: list,
      student,
      beta,
      noise,
      crashBoost: num('crash-boost', 3),
      crashLookback: num('crash-lookback', 30),
      divergeBoost: num('diverge-boost', 1.5),
      brakeBoost: num('brake-boost', 1),
      vzNoise: num('vz-noise', 0.3),
      seed: seed * 100000 + it * 100 + w,
    });
    const chunks = await Promise.all(split(eps, workers).map((list, w) => pool.runJob<DaggerChunk>('quadDagger', payload(list, w), w)));
    let rows = 0;
    let crashes = 0;
    let cov = 0;
    for (const c of chunks) {
      data.add(c);
      rows += c.n;
      for (const e of c.episodes) {
        crashes += e.crashed ? 1 : 0;
        cov += e.coverage;
      }
    }
    const tCollect = (Date.now() - t0) / 1000;
    // Train: `epochs` passes over the aggregate, in shuffled chunks.
    let loss = 0;
    for (let ep = 0; ep < epochs; ep++) {
      const order = new Int32Array(data.n);
      for (let i = 0; i < data.n; i++) order[i] = i;
      for (let i = data.n - 1; i > 0; i--) {
        const j = rng.int(i + 1);
        const tmp = order[i]!;
        order[i] = order[j]!;
        order[j] = tmp;
      }
      for (let s = 0; s < data.n; s += chunk) {
        const b = data.gather(order.subarray(s, Math.min(data.n, s + chunk)));
        loss = await trainer.imitate(b.obs, b.act, b.w.length, { epochs: 1, minibatch: num('minibatch', 512), weights: b.w, actionWeights, actionSpace: !args.includes('--atanh') });
      }
    }
    const tTrain = (Date.now() - t0) / 1000 - tCollect;
    const res = await evalEpisodes(pool, workers, val, { hidden, weights: policy.getWeights() });
    const s = summary(res);
    const entry = { iter: it, beta, rows, data: data.n, rolloutCoverage: cov / eps.length, rolloutCrashes: crashes, loss, val: s, seconds: (Date.now() - t0) / 1000 };
    log.push(entry);
    console.log(
      `iter ${it} beta ${beta.toFixed(2)} rows ${rows} data ${data.n} rollout ${pct(cov / eps.length)} crash ${crashes}/${eps.length} ` +
        `loss ${loss.toFixed(4)} | alone ${pct(s.coverage)} crashes ${s.crashes}/${s.n} score ${pct(s.score)} | collect ${tCollect.toFixed(0)}s train ${tTrain.toFixed(0)}s`,
    );
    const ck = toCheckpoint(policy);
    writeFileSync(join(outDir, args.includes('--keep-all') ? `iter-${it}.bin` : 'last.bin'), serializeCheckpointFor('x.bin', ck));
    if (s.score > best) {
      best = s.score;
      writeFileSync(join(outDir, 'best.bin'), serializeCheckpointFor('x.bin', ck));
    }
    writeFileSync(join(outDir, 'log.json'), JSON.stringify({ args, log }, null, 1));
  }
  console.log(`dagger done in ${((Date.now() - started) / 60000).toFixed(1)} min; best practice score ${pct(best)}`);
  if (num('rl', 0) > 0) {
    // Then PPO from the best DAgger student, in its own directory.
    policy.setWeights(paddedWeights(parseCheckpoint(readFileSync(join(outDir, 'best.bin')))));
    trainer.pushWeights();
    const rlDir = join(outDir, 'rl');
    mkdirSync(rlDir, { recursive: true });
    await reinforce(pool, workers, policy, trainer, rlDir, hidden, rng, practiceEpisodes(num('rl-val', num('val', 16))));
  }
  console.log(`done in ${((Date.now() - started) / 60000).toFixed(1)} min`);
  await pool.close();
}

void main();

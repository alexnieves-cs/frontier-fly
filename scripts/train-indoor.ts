/**
 * Train the shipped indoor policy: behavior cloning on frontier-teacher
 * rollouts, then DAgger rounds collected in parallel on every core, keeping the
 * round that scores best on a held-out validation set. An optional anchored PPO
 * fine-tune (KL + BC anchors, src/rl/finetune.ts) runs last and is kept only if
 * it improves validation.
 *
 * Usage: npm run train:indoor -- --out /tmp/indoor.json [--rounds 6] [--bc-episodes 240]
 *        [--episodes 80] [--bc-epochs 10] [--epochs 4] [--beta0 0.6] [--beta-decay 0.7]
 *        [--capacity 600000] [--workers N] [--hidden 512,256] [--lr 1e-3] [--ppo-updates 0]
 *        [--weighted]   (up-weight divergent/stalled states; it collapsed in our runs)
 * The defaults are the run that produced public/checkpoints/pretrained.json.
 *        npm run train:indoor -- --validate-only ckpt.json --hidden 512,256   (score a checkpoint)
 *
 * Install the optional native backend first for ~50x faster learning:
 *   npm install --no-save @tensorflow/tfjs-node@4.22.0
 * Never writes public/checkpoints unless --out says so. Eval seeds are excluded
 * from training (WorldPool) and validation uses seeds the eval never uses.
 */
import { readFileSync, writeFileSync } from 'node:fs';
import { availableParallelism } from 'node:os';
import { parseCheckpoint, serializeCheckpoint } from '../src/rl/checkpoint';
import { checkpointFromPolicy, DaggerDataset, policyFromCheckpoint } from '../src/rl/dagger';
import { ppoFinetune } from '../src/rl/finetune';
import { InferPolicy } from '../src/rl/infer';
import { TfTrainer } from '../src/rl/model';
import { ACT_DIM, OBS_DIM } from '../src/shared/constants';
import type { LightingPreset } from '../src/sim/lighting';
import type { DaggerEpisode } from '../src/train/daggerJob';
import { mergeDaggerResults, type DaggerJobResult } from '../src/train/daggerJob';
import { NodeRolloutPool } from '../src/train/nodePool';
import type { EvalJob } from '../src/train/rolloutCore';
import { useFastestBackend } from './lib/tfBackend';

const args = process.argv.slice(2);
const arg = (name: string) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const num = (name: string, fallback: number) => (arg(name) == null ? fallback : Number(arg(name)));
const pct = (v: number) => `${(v * 100).toFixed(1)}%`;

const STAGES = [0, 1, 2, 3];
const STAGE_WEIGHTS = [0.25, 0.15, 0.2, 0.4];

/** Held-out validation: seeds the eval protocol never uses (it uses 101/202/303/404 and 4242/5150/6160). */
function validationJobs(): EvalJob[] {
  const jobs: EvalJob[] = [];
  // Stage 3 coverage varies a lot per building, so it gets the most seeds.
  for (let i = 0; i < 6; i++) jobs.push({ stage: 0, seed: 9001 + i, lighting: 'day' });
  for (let i = 0; i < 16; i++) jobs.push({ stage: 3, seed: 9101 + i, lighting: 'day' });
  for (let i = 0; i < 2; i++) jobs.push({ stage: 0, seed: 9051 + i, lighting: 'night-dark' });
  for (let i = 0; i < 6; i++) jobs.push({ stage: 3, seed: 9151 + i, lighting: 'night-dark' });
  return jobs;
}

interface Validation {
  small: number;
  large: number;
  score: number;
}

async function validate(pool: NodeRolloutPool, jobs: EvalJob[]): Promise<Validation> {
  const results = await pool.evaluate(jobs);
  const small: number[] = [];
  const large: number[] = [];
  results.forEach((r, i) => (jobs[i]!.stage <= 1 ? small : large).push(r.coverage));
  const avg = (xs: number[]) => xs.reduce((s, v) => s + v, 0) / Math.max(1, xs.length);
  const s = avg(small);
  const l = avg(large);
  // The stage-3 bar is the hard one; weight it more so selection does not trade it away.
  return { small: s, large: l, score: 0.4 * s + 0.6 * l };
}

function pickStage(u: number): number {
  let total = 0;
  for (const w of STAGE_WEIGHTS) total += w;
  let x = u * total;
  for (let i = 0; i < STAGES.length; i++) {
    x -= STAGE_WEIGHTS[i]!;
    if (x <= 0) return STAGES[i]!;
  }
  return STAGES[STAGES.length - 1]!;
}

function episodes(count: number, beta: number, round: number): DaggerEpisode[] {
  const out: DaggerEpisode[] = [];
  for (let i = 0; i < count; i++) {
    // Deterministic stage/lighting schedule per round (golden-ratio sequence).
    const u = (((round + 1) * 0.754877666 + i * 0.618033989) % 1 + 1) % 1;
    const stage = pickStage(u);
    const lighting: LightingPreset | 'sampled' = i % 4 === 3 ? 'night-dark' : 'day';
    out.push({
      stage,
      beta,
      steps: stage <= 1 ? 1000 : 2400,
      lighting,
      episode: { maxSteps: stage <= 1 ? 1000 : 2400, stallSteps: 500, maxCollisions: 16 },
    });
  }
  return out;
}

async function collect(pool: NodeRolloutPool, workers: number, eps: DaggerEpisode[], seed: number): Promise<DaggerJobResult> {
  const per: DaggerEpisode[][] = Array.from({ length: workers }, () => []);
  // Longest episodes first, round robin, so workers finish together.
  const sorted = [...eps].sort((a, b) => b.steps - a.steps);
  sorted.forEach((ep, i) => per[i % workers]!.push(ep));
  const results = (await pool.runJobAll('dagger', (w: number) => ({ episodes: per[w], seed: seed + w * 7919 }))) as DaggerJobResult[];
  return mergeDaggerResults(results);
}

async function main() {
  const out = arg('out') ?? '/tmp/frontier-fly-indoor.json';
  const rounds = num('rounds', 6);
  const workers = num('workers', Math.max(1, availableParallelism()));
  const hidden = (arg('hidden') ?? '512,256').split(',').map(Number);
  const started = Date.now();
  const backend = await useFastestBackend();
  console.log(`learner backend ${backend}, ${workers} rollout workers, hidden ${hidden.join('x')}`);

  const pool = new NodeRolloutPool({
    workers,
    envsPerWorker: 4,
    seed: num('seed', 11),
    hidden,
    worldPool: { stages: STAGES, perStage: num('per-stage', 48), churn: 0.15 },
  });
  await pool.init();

  const scoreOnly = arg('validate-only');
  if (scoreOnly) {
    pool.setWeights(policyFromCheckpoint(parseCheckpoint(readFileSync(scoreOnly, 'utf8'))).getWeights());
    const val = await validate(pool, validationJobs());
    console.log(`val small ${pct(val.small)} large ${pct(val.large)} score ${pct(val.score)}`);
    await pool.close();
    return;
  }

  const policy = new InferPolicy(OBS_DIM, ACT_DIM, hidden, num('seed', 11));
  const trainer = new TfTrainer(policy, num('lr', 1e-3));
  await trainer.ready();
  const dataset = new DaggerDataset(OBS_DIM, ACT_DIM, num('capacity', 600_000), { policy: 'reservoir', seed: 11 });
  const valJobs = validationJobs();
  let best: { round: number; val: Validation; weights: ArrayBuffer[] } | null = null;

  for (let round = 0; round <= rounds; round++) {
    const t0 = Date.now();
    const beta = round === 0 ? 1 : num('beta0', 0.6) * Math.pow(num('beta-decay', 0.7), round - 1);
    const count = round === 0 ? num('bc-episodes', 240) : num('episodes', 80);
    pool.setWeights(policy.getWeights());
    const data = await collect(pool, workers, episodes(count, beta, round), 1000 + round * 104_729);
    const obs = data.arrays.obs!;
    const actions = data.arrays.actions!;
    const weights = data.arrays.weights!;
    const n = actions.length / ACT_DIM;
    dataset.addRollout({ obs, actions, weights: round === 0 || !args.includes('--weighted') ? undefined : weights, n, obsDim: OBS_DIM });
    const cov = data.extras.map((e) => Number(e.coverage ?? 0));
    const tCollect = (Date.now() - t0) / 1000;
    const view = dataset.view();
    const loss = await trainer.imitate(view.obs, view.actions, view.n, {
      epochs: round === 0 ? num('bc-epochs', 10) : num('epochs', 4),
      minibatch: 512,
      weights: view.weights,
    });
    pool.setWeights(policy.getWeights());
    const val = await validate(pool, valJobs);
    console.log(
      `round ${round} beta ${beta.toFixed(2)} +${n} samples (data ${dataset.size}) rolloutCov ${pct(cov.reduce((s, v) => s + v, 0) / Math.max(1, cov.length))} ` +
        `loss ${loss.toFixed(4)} val small ${pct(val.small)} large ${pct(val.large)} score ${pct(val.score)} ` +
        `collect ${tCollect.toFixed(0)}s total ${((Date.now() - t0) / 1000).toFixed(0)}s`,
    );
    if (!best || val.score > best.val.score) best = { round, val, weights: policy.getWeights() };
    writeFileSync(out.replace(/\.json$/, '') + `.r${round}.json`, serializeCheckpoint(checkpointFromPolicy(policy)));
  }
  policy.setWeights(best!.weights);
  console.log(`best round ${best!.round} val ${pct(best!.val.score)}`);
  writeFileSync(out, serializeCheckpoint(checkpointFromPolicy(policy)));

  const ppoUpdates = num('ppo-updates', 0);
  if (ppoUpdates > 0) {
    const tuned = await ppoFinetune({
      checkpoint: checkpointFromPolicy(policy),
      dataset: dataset.view(),
      updates: ppoUpdates,
      seed: num('seed', 11),
      onUpdate: (m) =>
        console.log(
          `ppo ${m.update} anchorKl ${m.anchorKl.toFixed(4)} bc ${m.bcLoss.toFixed(4)} cov ${pct(m.rolloutCoverage)}` +
            (m.validation ? ` val ${pct(m.validation.score)}` : ''),
        ),
    });
    policy.setWeights(tuned.checkpoint.weights.map((row) => Float32Array.from(row).buffer));
    pool.setWeights(policy.getWeights());
    const val = await validate(pool, valJobs);
    console.log(`ppo val ${pct(val.score)} vs dagger ${pct(best!.val.score)}`);
    const ppoOut = out.replace(/\.json$/, '') + '.ppo.json';
    writeFileSync(ppoOut, serializeCheckpoint(tuned.checkpoint));
    if (val.score > best!.val.score) {
      writeFileSync(out, serializeCheckpoint(tuned.checkpoint));
      console.log(`kept the PPO fine-tune in ${out}`);
    }
  }
  await pool.close();
  console.log(`done in ${((Date.now() - started) / 1000).toFixed(0)} s`);
}

void main();

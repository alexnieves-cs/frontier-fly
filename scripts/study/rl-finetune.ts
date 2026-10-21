/**
 * Held-out comparison of RL fine-tuned checkpoints against the shipped DAgger
 * policy, written to docs/rl-finetune.json (REPORT.md reads it).
 *
 * Usage: npx vite-node scripts/study/rl-finetune.ts -- label=path [label=path ...] [--out docs/rl-finetune.json]
 *        [--logs id:path,id:path]   (train:ppo logs whose validation lines become the learning curves)
 *        [--shipped label]   (the arm that ships; default the last)
 *        [--recipes recipes.json]   ({ id: { label, flags } } for those runs)
 *
 * The first checkpoint is the baseline. Sets (none of them used to pick checkpoints):
 *  - protocol: the fixed coverage eval (stage 0 seeds 101/202/303/404, stage 3 4242/5150/6160), day and night-dark
 *  - study:    the coverage study's hard buildings, stage 3 seeds 30001-30030, day and night-dark, and easy 20001-20030 day
 *  - report:   this track's report block, 68001-68299 (scripts/train-ppo.ts seedSet('report'))
 * Differences are paired per episode, with a 95% bootstrap interval.
 */
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { availableParallelism } from 'node:os';
import { STAGE0_SEEDS, STAGE3_SEEDS } from '../../src/eval/coverage';
import { parseCheckpoint } from '../../src/rl/checkpoint';
import { RNG } from '../../src/shared/rng';
import { NodeRolloutPool } from '../../src/train/nodePool';
import type { SeqEvalResult } from '../../src/train/ppoJob';
import type { EvalJob } from '../../src/train/rolloutCore';

const args = process.argv.slice(2);
const flag = (name: string) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const outPath = flag('out') ?? 'docs/rl-finetune.json';
const logs = (flag('logs') ?? '').split(',').filter(Boolean).map((x) => {
  const at = x.indexOf(':');
  return { id: x.slice(0, at), path: x.slice(at + 1) };
});
const recipes = flag('recipes') ? (JSON.parse(readFileSync(flag('recipes')!, 'utf8')) as Record<string, { label: string; flags: string }>) : {};
const arms = args.filter((a, i) => a.includes('=') && !args[i - 1]?.startsWith('--')).map((a) => {
  const [label, path] = a.split('=');
  return { label: label!, path: path! };
});

interface Cell {
  set: string;
  stage: number;
  lighting: string;
}

function jobs(): (EvalJob & { set: string })[] {
  const out: (EvalJob & { set: string })[] = [];
  for (const lighting of ['day', 'night-dark'] as const) {
    for (const seed of STAGE0_SEEDS) out.push({ set: 'protocol', stage: 0, seed, lighting });
    for (const seed of STAGE3_SEEDS) out.push({ set: 'protocol', stage: 3, seed, lighting });
    for (let i = 0; i < 30; i++) out.push({ set: 'study', stage: 3, seed: 30001 + i, lighting });
  }
  for (let i = 0; i < 30; i++) out.push({ set: 'study', stage: 0, seed: 20001 + i, lighting: 'day' });
  for (let i = 1; i <= 32; i++) out.push({ set: 'report', stage: 3, seed: 68_000 + i, lighting: 'day' });
  for (let i = 1; i <= 8; i++) out.push({ set: 'report', stage: 3, seed: 68_100 + i, lighting: 'night-dark' });
  for (let i = 1; i <= 6; i++) out.push({ set: 'report', stage: 0, seed: 68_200 + i, lighting: 'day' });
  for (let i = 1; i <= 2; i++) out.push({ set: 'report', stage: 0, seed: 68_250 + i, lighting: 'night-dark' });
  return out;
}

/** Validation lines of a train:ppo log: "init ..." and "  val ppo N ..." / "  val imitate N ...". */
function curveOf(log: string): { update: number; small: number; large: number; collisions: number; score: number }[] {
  const out: { update: number; small: number; large: number; collisions: number; score: number }[] = [];
  const re = /^(?:init|\s+val (?:ppo|imitate) (\d+)) small ([\d.]+)% large ([\d.]+)% \(night [\d.]+%\) coll ([\d.]+) score ([\d.]+)%/;
  for (const line of log.split('\n')) {
    const m = re.exec(line);
    if (m) out.push({ update: m[1] ? Number(m[1]) : 0, small: Number(m[2]) / 100, large: Number(m[3]) / 100, collisions: Number(m[4]), score: Number(m[5]) / 100 });
  }
  return out;
}

const mean = (xs: number[]) => xs.reduce((s, v) => s + v, 0) / Math.max(1, xs.length);

function bootstrap(diffs: number[], rng: RNG): [number, number] {
  const means: number[] = [];
  for (let b = 0; b < 2000; b++) {
    let s = 0;
    for (let i = 0; i < diffs.length; i++) s += diffs[rng.int(diffs.length)]!;
    means.push(s / diffs.length);
  }
  means.sort((a, b) => a - b);
  return [means[50]!, means[1949]!];
}

async function main() {
  if (arms.length === 0) throw new Error('give at least one label=path');
  const workers = Math.max(1, availableParallelism());
  const pool = new NodeRolloutPool({ workers, envsPerWorker: 1, worldPool: { stages: [0], perStage: 1 }, jobModules: ['src/train/ppoJob.ts'], autoDagger: false });
  await pool.init();
  const all = jobs();
  const results: Record<string, SeqEvalResult[]> = {};
  for (const arm of arms) {
    const c = parseCheckpoint(readFileSync(arm.path));
    const spec = { weights: c.weights.map((row) => Float32Array.from(row).buffer), hidden: c.hidden, memDim: c.memDim ?? 0 };
    const order = all.map((job, i) => ({ job, i })).sort((a, b) => b.job.stage - a.job.stage);
    const per: { job: EvalJob; i: number }[][] = Array.from({ length: workers }, () => []);
    order.forEach((x, k) => per[k % workers]!.push(x));
    const t0 = Date.now();
    const parts = (await pool.runJobAll('seqEval', (w: number) => ({
      policy: spec,
      jobs: per[w]!.map((x) => ({ stage: x.job.stage, seed: x.job.seed, lighting: x.job.lighting })),
    }))) as SeqEvalResult[][];
    const res = new Array<SeqEvalResult>(all.length);
    per.forEach((list, w) => list.forEach((x, k) => (res[x.i] = parts[w]![k]!)));
    results[arm.label] = res;
    console.log(`${arm.label}: ${all.length} episodes in ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  }
  await pool.close();

  const cells: Cell[] = [];
  for (const j of all) if (!cells.some((c) => c.set === j.set && c.stage === j.stage && c.lighting === j.lighting)) cells.push({ set: j.set, stage: j.stage, lighting: j.lighting });
  const base = arms[0]!.label;
  const rng = new RNG(7, 'bootstrap');
  const table = cells.map((cell) => {
    const idx = all.map((j, i) => (j.set === cell.set && j.stage === cell.stage && j.lighting === cell.lighting ? i : -1)).filter((i) => i >= 0);
    const row: Record<string, unknown> = { ...cell, episodes: idx.length };
    for (const arm of arms) {
      const r = results[arm.label]!;
      const cov = idx.map((i) => r[i]!.coverage);
      const entry: Record<string, unknown> = { coverage: mean(cov), collisionsPerEpisode: mean(idx.map((i) => r[i]!.collisions)) };
      if (arm.label !== base) {
        const diffs = idx.map((i) => r[i]!.coverage - results[base]![i]!.coverage);
        entry.deltaVsBase = mean(diffs);
        entry.deltaCi = bootstrap(diffs, rng);
      }
      row[arm.label] = entry;
    }
    return row;
  });
  // Pooled hard-building summary over every stage-3 episode not used for selection.
  const hardIdx = all.map((j, i) => (j.stage === 3 ? i : -1)).filter((i) => i >= 0);
  const pooled: Record<string, unknown> = { episodes: hardIdx.length };
  for (const arm of arms) {
    const r = results[arm.label]!;
    const entry: Record<string, unknown> = { coverage: mean(hardIdx.map((i) => r[i]!.coverage)), collisionsPerEpisode: mean(hardIdx.map((i) => r[i]!.collisions)) };
    if (arm.label !== base) {
      const diffs = hardIdx.map((i) => r[i]!.coverage - results[base]![i]!.coverage);
      entry.deltaVsBase = mean(diffs);
      entry.deltaCi = bootstrap(diffs, rng);
    }
    pooled[arm.label] = entry;
  }
  const doc = {
    schema: 1,
    commit: execSync('git rev-parse --short HEAD').toString().trim(),
    baseline: base,
    /** The arm that now ships as public/checkpoints/pretrained.bin. */
    shipped: flag('shipped') ?? arms.at(-1)!.label,
    arms: arms.map((a) => ({ label: a.label, checkpoint: a.path })),
    selection: 'checkpoints picked on the tuning block 66001-66299 only',
    hardPooled: pooled,
    cells: table,
    runs: logs.map(({ id, path }) => ({ id, ...recipes[id], curve: curveOf(readFileSync(path, 'utf8')) })),
  };
  writeFileSync(outPath, JSON.stringify(doc, null, 1) + '\n');
  for (const row of table) {
    const parts = arms.map((a) => {
      const e = row[a.label] as { coverage: number; deltaVsBase?: number };
      return `${a.label} ${(e.coverage * 100).toFixed(1)}%${e.deltaVsBase != null ? ` (${(e.deltaVsBase * 100).toFixed(1)})` : ''}`;
    });
    console.log(`${row.set} s${row.stage} ${row.lighting} n=${row.episodes}: ${parts.join('  ')}`);
  }
  console.log(`hard pooled: ${JSON.stringify(pooled)}`);
  console.log(`wrote ${outPath}`);
}

void main();

/**
 * Held-out comparison of RL fine-tuned checkpoints against the shipped DAgger
 * policy, written to docs/rl-finetune.json (REPORT.md reads it).
 *
 * Usage: npx vite-node scripts/study/rl-finetune.ts -- label=path [label=path ...] [--out docs/rl-finetune.json]
 *        [--logs id:path,id:path]   (train:ppo logs whose validation lines become the learning curves)
 *        [--shipped label]   (the arm that ships; default the last)
 *        [--recipes recipes.json]   ({ id: { label, flags, offset? } } for those runs; offset shifts a continued run's updates)
 *        [--runs-from old.json]   (carry over the learning curves of an earlier report whose logs are gone)
 *        [--shard k --of n --out shard.jsonl] then [--merge shard-*.jsonl]   (how .github/workflows/study.yml runs it)
 *
 * Every flight runs under the crash rule (src/shared/constants.ts CRASH_SPEED): a crash ends the
 * episode, and each cell counts the flights it ended.
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
import { CRASH_SPEED } from '../../src/shared/constants';
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
const recipes = flag('recipes') ? (JSON.parse(readFileSync(flag('recipes')!, 'utf8')) as Record<string, { label: string; flags: string; offset?: number }>) : {};
const arms = args.filter((a, i) => a.includes('=') && !a.startsWith('--') && !a.endsWith('.jsonl') && !args[i - 1]?.startsWith('--')).map((a) => {
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
  const re = /^(?:init|\s+val (?:ppo|imitate) (\d+)) small ([\d.]+)% large ([\d.]+)% \(night [\d.]+%\) coll ([\d.]+)(?: crash [\d.]+%)?(?: quad [\d.]+% \(crash [\d.]+%\))? score (-?[\d.]+)%/;
  for (const line of log.split('\n')) {
    const m = re.exec(line);
    if (m) out.push({ update: m[1] ? Number(m[1]) : 0, small: Number(m[2]) / 100, large: Number(m[3]) / 100, collisions: Number(m[4]), score: Number(m[5]) / 100 });
  }
  // A log with no validation line means the format moved again: an empty curve must not pass quietly.
  if (!out.length) throw new Error('rl-finetune: no validation lines matched in a train:ppo log (did its format change?)');
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

/** One flight of one arm: `i` indexes jobs(). */
interface Row {
  arm: string;
  i: number;
  coverage: number;
  collisions: number;
  crashed: boolean;
}

/** Fly the jobs of shard `shard` of `of` for every arm. */
async function fly(shard: number, of: number, workers: number): Promise<Row[]> {
  const pool = new NodeRolloutPool({ workers, envsPerWorker: 1, worldPool: { stages: [0], perStage: 1 }, jobModules: ['src/train/ppoJob.ts'], autoDagger: false });
  await pool.init();
  const all = jobs();
  const rows: Row[] = [];
  for (const arm of arms) {
    const c = parseCheckpoint(readFileSync(arm.path));
    const spec = { weights: c.weights.map((row) => Float32Array.from(row).buffer), hidden: c.hidden, memDim: c.memDim ?? 0 };
    const order = all
      .map((job, i) => ({ job, i }))
      .filter((x) => x.i % of === shard)
      .sort((a, b) => b.job.stage - a.job.stage);
    const per: { job: EvalJob; i: number }[][] = Array.from({ length: workers }, () => []);
    order.forEach((x, k) => per[k % workers]!.push(x));
    const t0 = Date.now();
    const parts = (await pool.runJobAll('seqEval', (w: number) => ({
      policy: spec,
      jobs: per[w]!.map((x) => ({ stage: x.job.stage, seed: x.job.seed, lighting: x.job.lighting })),
    }))) as SeqEvalResult[][];
    per.forEach((list, w) =>
      list.forEach((x, k) => {
        const r = parts[w]![k]!;
        rows.push({ arm: arm.label, i: x.i, coverage: r.coverage, collisions: r.collisions, crashed: r.crashed });
      }),
    );
    console.log(`${arm.label}: ${order.length} episodes in ${((Date.now() - t0) / 1000).toFixed(0)} s`);
  }
  await pool.close();
  return rows;
}

function report(rows: Row[]) {
  const all = jobs();
  const results: Record<string, Row[]> = {};
  for (const arm of arms) {
    const res = new Array<Row>(all.length);
    for (const r of rows) if (r.arm === arm.label) res[r.i] = r;
    for (let i = 0; i < all.length; i++) if (!res[i]) throw new Error(`rl-finetune: arm ${arm.label} has no flight for job ${i}`);
    results[arm.label] = res;
  }

  const cells: Cell[] = [];
  for (const j of all) if (!cells.some((c) => c.set === j.set && c.stage === j.stage && c.lighting === j.lighting)) cells.push({ set: j.set, stage: j.stage, lighting: j.lighting });
  const base = arms[0]!.label;
  const rng = new RNG(7, 'bootstrap');
  const entryOf = (label: string, idx: number[]) => {
    const r = results[label]!;
    const entry: Record<string, unknown> = {
      coverage: mean(idx.map((i) => r[i]!.coverage)),
      collisionsPerEpisode: mean(idx.map((i) => r[i]!.collisions)),
      // Flights the crash rule ended.
      crashes: idx.filter((i) => r[i]!.crashed).length,
    };
    if (label !== base) {
      const diffs = idx.map((i) => r[i]!.coverage - results[base]![i]!.coverage);
      entry.deltaVsBase = mean(diffs);
      entry.deltaCi = bootstrap(diffs, rng);
    }
    return entry;
  };
  const table = cells.map((cell) => {
    const idx = all.map((j, i) => (j.set === cell.set && j.stage === cell.stage && j.lighting === cell.lighting ? i : -1)).filter((i) => i >= 0);
    const row: Record<string, unknown> = { ...cell, episodes: idx.length };
    for (const arm of arms) row[arm.label] = entryOf(arm.label, idx);
    return row;
  });
  // Pooled hard-building summary over every stage-3 episode not used for selection.
  const hardIdx = all.map((j, i) => (j.stage === 3 ? i : -1)).filter((i) => i >= 0);
  const pooled: Record<string, unknown> = { episodes: hardIdx.length };
  for (const arm of arms) pooled[arm.label] = entryOf(arm.label, hardIdx);
  // Learning curves come from train:ppo logs; `--runs-from` carries over the curves of an earlier
  // report whose logs are gone (they are training history, not something this script measures).
  const carried = flag('runs-from') ? ((JSON.parse(readFileSync(flag('runs-from')!, 'utf8')) as { runs?: unknown[] }).runs ?? []) : [];
  const doc = {
    schema: 2,
    commit: execSync('git rev-parse --short HEAD').toString().trim(),
    crashSpeed: CRASH_SPEED,
    baseline: base,
    /** The arm that now ships as public/checkpoints/pretrained.bin. */
    shipped: flag('shipped') ?? arms.at(-1)!.label,
    arms: arms.map((a) => ({ label: a.label, checkpoint: a.path })),
    selection: 'checkpoints picked on the tuning block 66001-66299 only',
    hardPooled: pooled,
    cells: table,
    runs: [
      ...carried,
      ...logs.map(({ id, path }) => {
        const r = recipes[id] ?? { label: id, flags: '' };
        const offset = r.offset ?? 0;
        return { id, label: r.label, flags: r.flags, curve: curveOf(readFileSync(path, 'utf8')).map((p) => ({ ...p, update: p.update + offset })) };
      }),
    ],
  };
  writeFileSync(outPath, JSON.stringify(doc, null, 1) + '\n');
  for (const row of table) {
    const parts = arms.map((a) => {
      const e = row[a.label] as { coverage: number; crashes: number; deltaVsBase?: number };
      return `${a.label} ${(e.coverage * 100).toFixed(1)}%${e.deltaVsBase != null ? ` (${(e.deltaVsBase * 100).toFixed(1)})` : ''} crash ${e.crashes}`;
    });
    console.log(`${row.set} s${row.stage} ${row.lighting} n=${row.episodes}: ${parts.join('  ')}`);
  }
  console.log(`hard pooled: ${JSON.stringify(pooled)}`);
  console.log(`wrote ${outPath}`);
}

async function main() {
  if (arms.length === 0) throw new Error('give at least one label=path');
  if (args.includes('--merge')) {
    // Shards written by --shard (study.yml): every .jsonl argument.
    const files = args.filter((a) => a.endsWith('.jsonl'));
    report(files.flatMap((f) => readFileSync(f, 'utf8').split('\n').filter(Boolean).map((l) => JSON.parse(l) as Row)));
  } else if (args.includes('--shard')) {
    // One process per core on a runner, so one rollout worker each.
    const rows = await fly(Number(flag('shard')), Number(flag('of') ?? 1), 1);
    writeFileSync(outPath, rows.map((r) => JSON.stringify(r)).join('\n') + '\n');
    console.log(`wrote ${outPath}: ${rows.length} flights`);
  } else {
    report(await fly(0, 1, Math.max(1, availableParallelism())));
  }
}

void main();

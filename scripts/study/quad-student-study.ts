/**
 * The quad student's held-out runs (docs/QUAD_STUDENT.md): the shipped checkpoint alone and behind
 * the braking shield, against the default quad pilot (its teacher) and the planner-only quad
 * pilot, paired on the same day and night quad episodes. Sharded for study.yml.
 *
 *   vite-node scripts/study/quad-student-study.ts -- --block gate [--shard k --of n] --out x.jsonl
 *   vite-node scripts/study/quad-student-study.ts -- --block gate --merge shards/*.jsonl --out docs/quad-student-gate.json
 *
 * Blocks (roadmap seed ledger): gate 181001-181160, report 185001-185332, published 50001-50030.
 * Each is flown once, after the checkpoint and gain were frozen on practice seeds 180001-180400.
 */
import { appendFileSync, readFileSync, writeFileSync } from 'node:fs';
import { parseCheckpoint } from '../../src/rl/checkpoint';
import { paddedWeights, QUAD_STUDENT_GAIN } from '../../src/rl/quadStudent';
import { NodeRolloutPool } from '../../src/train/nodePool';
import type { PickerEpisode } from '../../src/train/pickerJob';
import type { EvalEpisode, EvalPayload } from '../../src/train/quadDistillJob';

const args = process.argv.slice(2);
const arg = (name: string) => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};

const BLOCKS = { gate: [181001, 160], report: [185001, 332], published: [50001, 30] } as const;
const ARMS = ['student-shield', 'student-alone', 'default-pilot', 'planner-only'] as const;
type Arm = (typeof ARMS)[number];
interface Row extends EvalEpisode {
  arm: Arm;
}

const block = (arg('block') ?? 'gate') as keyof typeof BLOCKS;
if (!(block in BLOCKS)) throw new Error(`--block is one of ${Object.keys(BLOCKS).join(', ')}`);
const out = arg('out') ?? `/tmp/quad-student-${block}.jsonl`;
const ckptPath = arg('ckpt') ?? 'public/checkpoints/quad-student.bin';

function summarize(rows: Row[]) {
  const arms: Record<string, unknown> = {};
  for (const arm of ARMS) {
    const rs = rows.filter((r) => r.arm === arm);
    if (!rs.length) continue;
    const steps = rs.reduce((a, r) => a + r.steps, 0);
    const cov = rs.map((r) => r.coverage);
    const mean = cov.reduce((a, b) => a + b, 0) / rs.length;
    const sd = Math.sqrt(cov.reduce((a, c) => a + (c - mean) ** 2, 0) / Math.max(1, rs.length - 1));
    arms[arm] = {
      flights: rs.length,
      coverage: mean,
      ci95: [mean - (1.96 * sd) / Math.sqrt(rs.length), mean + (1.96 * sd) / Math.sqrt(rs.length)],
      crashes: rs.filter((r) => r.crashed).length,
      interventionRate: rs.reduce((a, r) => a + r.interventions, 0) / Math.max(1, steps),
      day: rs.filter((r) => r.lighting === 'day').reduce((a, r) => a + r.coverage, 0) / Math.max(1, rs.filter((r) => r.lighting === 'day').length),
      night: rs.filter((r) => r.lighting !== 'day').reduce((a, r) => a + r.coverage, 0) / Math.max(1, rs.filter((r) => r.lighting !== 'day').length),
    };
  }
  // Paired difference, student behind the shield minus each control, per flight.
  const key = (r: Row) => `${r.seed}:${r.lighting}`;
  const shield = new Map(rows.filter((r) => r.arm === 'student-shield').map((r) => [key(r), r.coverage]));
  const paired: Record<string, unknown> = {};
  for (const arm of ['default-pilot', 'planner-only', 'student-alone'] as const) {
    const d = rows.filter((r) => r.arm === arm && shield.has(key(r))).map((r) => shield.get(key(r))! - r.coverage);
    if (!d.length) continue;
    const m = d.reduce((a, b) => a + b, 0) / d.length;
    const sd = Math.sqrt(d.reduce((a, x) => a + (x - m) ** 2, 0) / Math.max(1, d.length - 1));
    paired[`student-shield minus ${arm}`] = { mean: m, ci95: [m - (1.96 * sd) / Math.sqrt(d.length), m + (1.96 * sd) / Math.sqrt(d.length)], flights: d.length };
  }
  return { arms, paired };
}

const merge = args.indexOf('--merge');
if (merge >= 0) {
  // The files run from --merge to the next flag (study.yml appends its args after them).
  const end = args.findIndex((a, i) => i > merge && a.startsWith('--'));
  const files = args.slice(merge + 1, end < 0 ? undefined : end);
  const rows: Row[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as Row),
  );
  const [base, n] = BLOCKS[block];
  const result = { block, seeds: [base, base + n - 1], lightings: ['day', 'night-dark'], gain: QUAD_STUDENT_GAIN, checkpoint: ckptPath, ...summarize(rows), flights: rows };
  writeFileSync(out, JSON.stringify(result, null, 1));
  for (const [arm, s] of Object.entries(result.arms)) {
    const v = s as { coverage: number; crashes: number; flights: number; interventionRate: number };
    console.log(`${arm.padEnd(15)} ${(v.coverage * 100).toFixed(1)}% crashes ${v.crashes}/${v.flights}${v.interventionRate ? ` interventions ${(v.interventionRate * 100).toFixed(1)}%` : ''}`);
  }
  console.log(JSON.stringify(result.paired));
} else {
  const shard = Number(arg('shard') ?? 0);
  const of = Number(arg('of') ?? 1);
  const [base, n] = BLOCKS[block];
  const jobs: { arm: Arm; ep: PickerEpisode }[] = [];
  for (let i = 0; i < n; i++) for (const lighting of ['day', 'night-dark'] as const) for (const arm of ARMS) jobs.push({ arm, ep: { stage: 3, seed: base + i, lighting, quad: 1 } });
  const mine = jobs.filter((_, i) => i % of === shard);
  const ck = parseCheckpoint(readFileSync(ckptPath));
  const student = { hidden: ck.hidden, weights: paddedWeights(ck) };
  const pool = new NodeRolloutPool({ workers: 1, envsPerWorker: 1, seed: 1, worldPool: { stages: [0], perStage: 1 }, jobModules: ['src/train/quadDistillJob.ts'], autoDagger: false });
  await pool.init();
  writeFileSync(out, '');
  for (const { arm, ep } of mine) {
    const payload: EvalPayload =
      arm === 'student-shield'
        ? { episodes: [ep], student, shield: true, gain: QUAD_STUDENT_GAIN }
        : arm === 'student-alone'
          ? { episodes: [ep], student, gain: QUAD_STUDENT_GAIN }
          : arm === 'default-pilot'
            ? { episodes: [ep], student: null }
            : { episodes: [ep], student: null, expert: { noPicker: true } };
    const [r] = await pool.runJob<EvalEpisode[]>('quadEval', payload, 0);
    appendFileSync(out, JSON.stringify({ ...r, arm }) + '\n');
  }
  await pool.close();
}

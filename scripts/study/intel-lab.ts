// Real-data check of the indoor pose estimator (src/loc): the Intel Research Lab (Seattle) log,
// scored with the Freiburg SLAM benchmark's relation-based error (Kümmerle et al. 2009).
//
//   npm run study:intel                       # every arm at every grid offset, in parallel; writes docs/intel-lab.json
//   vite-node scripts/study/intel-lab.ts -- --arms fused --out /tmp/i.json
//   vite-node scripts/study/intel-lab.ts -- --job fused 0 [--poses /tmp/poses.txt]   # one run, JSON on stdout
//
// Data: data/intel-lab/ (vendored, CC BY; see data/intel-lab/README.md), refetched by
// scripts/loc/fetch-intel.sh. The estimator runs with its shipped parameters: nothing was tuned on
// this log. Arms that change a parameter say which one and why.
import { execSync, spawn } from 'node:child_process';
import { availableParallelism } from 'node:os';
import { readFileSync, writeFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { odomFrame, parseFlaser, parseRelations, relationError, ringScan, type LaserLog, type Relation, type RelationError } from '../../src/loc/carmen';
import { defaultEstimatorOptions, PoseEstimator, type EstimatorOptions } from '../../src/loc/estimator';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

interface Arm {
  id: string;
  label: string;
  /** Lidar range the arm uses, m. */
  maxRange: number;
  /** null: the raw wheel odometry, no estimator. */
  estimator: Partial<EstimatorOptions> | null;
}

/**
 * Sweeps 0 .. DEV_SWEEPS-1 (the first 22% of the log, and only relations under 10 s inside them)
 * are the only data the adapted arm's three values were picked on: a 24-point grid over the cell
 * size (0.2, 0.1, 0.05 m), the Manhattan fix (on, off), the odometry noise (0.25, 0.05 m/√m) and the
 * lidar range (8, 20 m). The held-out score uses only relations whose sweeps both come later.
 */
export const DEV_SWEEPS = 3000;

/** Parameters for a wheeled robot with a SICK LMS, picked on the dev sweeps (see DEV_SWEEPS). */
const ADAPTED: Partial<EstimatorOptions> = { res: 0.1, odomRel: 0.05 };

const ARMS: Arm[] = [
  { id: 'odom', label: 'Raw wheel odometry (no lidar)', maxRange: 8, estimator: null },
  { id: 'fused', label: 'Shipped drone estimator, as is (20 cm cells, flow-sensor noise), lidar cut to 8 m', maxRange: 8, estimator: {} },
  { id: 'adapted', label: 'Same estimator, 10 cm cells and wheel-odometry noise (picked on the dev sweeps)', maxRange: 8, estimator: ADAPTED },
  { id: 'adapted-noManhattan', label: 'Adapted, without the Manhattan heading fix', maxRange: 8, estimator: { ...ADAPTED, manhattan: false } },
  { id: 'adapted-scan', label: 'Adapted, scan matching only (no odometry between sweeps)', maxRange: 8, estimator: { ...ADAPTED, mode: 'scan' } },
];

/** Where the map grid's corner sits, in cells: the same log at five sub-cell shifts of the grid. */
export const GRID_OFFSETS: [number, number][] = [
  [0, 0],
  [0.2, 0.4],
  [0.4, 0.8],
  [0.6, 0.2],
  [0.8, 0.6],
];

type Summary = Omit<RelationError, 'perTrans' | 'perRot'>;

/** One arm at one grid offset. */
interface Run {
  arm: string;
  offset: number;
  error: Summary;
  /** Relations whose sweeps both come after the dev sweeps. */
  heldOut: Summary;
  /** Relations spanning under 10 s (local tracking) and the rest (loop closures: the same place minutes apart). */
  local: Summary;
  loops: Summary;
  /** Median and 90th percentile of the per-relation errors (m, deg): the means are dominated by a tail. */
  quantiles: { transMedian: number; transP90: number; rotMedian: number; rotP90: number };
  /** Matched / rejected sweeps, re-acquisitions, heading fixes. */
  stats: Record<string, number> | null;
  /** Sweeps the estimator called itself lost. */
  lostSweeps: number;
  ms: number;
  /** Per-relation errors (m, deg), rounded, at grid offset 0 only. */
  perRelation?: { trans: number[]; rot: number[] };
}

/** Mean over grid offsets, with the spread of the headline numbers across them. */
export interface ArmResult {
  id: string;
  label: string;
  maxRange: number;
  offsets: number;
  error: Summary;
  heldOut: Summary;
  local: Summary;
  loops: Summary;
  quantiles: Run['quantiles'];
  /** Lowest and highest mean error over the grid offsets. */
  transRange: [number, number];
  rotRange: [number, number];
  stats: Record<string, number> | null;
  lostSweeps: number;
  msPerSweep: number;
  /** At grid offset 0. */
  perRelation: { trans: number[]; rot: number[] };
}

function round(x: number, d = 4): number {
  return Number.isFinite(x) ? Math.round(x * 10 ** d) / 10 ** d : x;
}

function q(xs: Float64Array, p: number): number {
  const s = Array.from(xs).filter(Number.isFinite).sort((a, b) => a - b);
  return s.length ? s[Math.min(s.length - 1, Math.floor(p * s.length))]! : NaN;
}

function strip(e: RelationError): Summary {
  return { n: e.n, missing: e.missing, transAbs: e.transAbs, transSq: e.transSq, rotAbs: e.rotAbs, rotSq: e.rotSq };
}

function runArm(log: LaserLog, arm: Arm, offset: [number, number]): { pose: Float64Array; stats: Record<string, number> | null; lost: number; ms: number } {
  const t0 = performance.now();
  const pose = new Float64Array(log.n * 3);
  if (!arm.estimator) {
    pose.set(log.odom);
    return { pose, stats: null, lost: 0, ms: performance.now() - t0 };
  }
  let x0 = Infinity;
  let y0 = Infinity;
  let x1 = -Infinity;
  let y1 = -Infinity;
  for (let i = 0; i < log.n; i++) {
    x0 = Math.min(x0, log.odom[i * 3]!);
    x1 = Math.max(x1, log.odom[i * 3]!);
    y0 = Math.min(y0, log.odom[i * 3 + 1]!);
    y1 = Math.max(y1, log.odom[i * 3 + 1]!);
  }
  // Odometry drifts metres over the run; leave room for the map on every side.
  const base = { ...defaultEstimatorOptions({ x0: 0, y0: 0, x1: 0, y1: 0 }), ...arm.estimator };
  const pad = 15 + offset[0] * base.res;
  const padY = 15 + offset[1] * base.res;
  const opts = { ...base, bounds: { x0: Math.floor(x0) - pad, y0: Math.floor(y0) - padY, x1: x1 + 15, y1: y1 + 15 } };
  const est = new PoseEstimator(opts);
  est.reset([log.odom[0]!, log.odom[1]!, 0], log.odom[2]!, 0, 0.01);
  est.correct(ringScan(log, 0, arm.maxRange));
  pose.set([est.x, est.y, est.yaw], 0);
  let lost = 0;
  for (let i = 1; i < log.n; i++) {
    const e = est.step(odomFrame(log, i, arm.maxRange));
    if (e.health.state === 'lost') lost += 1;
    pose[i * 3] = e.pos[0];
    pose[i * 3 + 1] = e.pos[1];
    pose[i * 3 + 2] = e.yaw;
  }
  return { pose, stats: { ...est.stats }, lost, ms: performance.now() - t0 };
}

function loadLog(dir: string): { log: LaserLog; rels: Relation[] } {
  const log = parseFlaser(gunzipSync(readFileSync(`${dir}/intel.flaser.gz`)).toString('utf8'));
  const rels = parseRelations(readFileSync(`${dir}/intel.relations`, 'utf8'));
  return { log, rels };
}

function job(armId: string, offset: number): Run {
  const { log, rels } = loadLog(arg('data', 'data/intel-lab'));
  const index = new Map<number, number>();
  log.t.forEach((ts, i) => index.set(ts, i));
  const arm = ARMS.find((a) => a.id === armId);
  if (!arm) throw new Error(`unknown arm ${armId}`);
  const r = runArm(log, arm, GRID_OFFSETS[offset]!);
  const err = relationError(log.t, r.pose, rels);
  const posesOut = arg('poses', '');
  if (posesOut) {
    const lines = Array.from({ length: log.n }, (_, i) => `${log.t[i]!.toFixed(6)} ${r.pose[i * 3]!.toFixed(4)} ${r.pose[i * 3 + 1]!.toFixed(4)} ${r.pose[i * 3 + 2]!.toFixed(5)}`);
    writeFileSync(posesOut, lines.join('\n') + '\n');
  }
  return {
    arm: armId,
    offset,
    error: strip(err),
    heldOut: strip(relationError(log.t, r.pose, rels.filter((x) => index.get(x.t1)! >= DEV_SWEEPS && index.get(x.t2)! >= DEV_SWEEPS))),
    local: strip(relationError(log.t, r.pose, rels.filter((x) => Math.abs(x.t2 - x.t1) < 10))),
    loops: strip(relationError(log.t, r.pose, rels.filter((x) => Math.abs(x.t2 - x.t1) >= 10))),
    quantiles: { transMedian: round(q(err.perTrans, 0.5)), transP90: round(q(err.perTrans, 0.9)), rotMedian: round(q(err.perRot, 0.5), 3), rotP90: round(q(err.perRot, 0.9), 3) },
    stats: r.stats,
    lostSweeps: r.lost,
    ms: Math.round(r.ms),
    perRelation: offset === 0 ? { trans: Array.from(err.perTrans, (x) => round(x)), rot: Array.from(err.perRot, (x) => round(x, 3)) } : undefined,
  };
}

function runJob(armId: string, offset: number): Promise<Run> {
  return new Promise((resolve, reject) => {
    const child = spawn('npx', ['vite-node', 'scripts/study/intel-lab.ts', '--', '--job', armId, String(offset), '--data', arg('data', 'data/intel-lab')], { stdio: ['ignore', 'pipe', 'inherit'] });
    let out = '';
    child.stdout.on('data', (d) => (out += d));
    child.on('exit', (code) => {
      const line = out.split('\n').find((l) => l.startsWith('RUN '));
      if (code === 0 && line) resolve(JSON.parse(line.slice(4)) as Run);
      else reject(new Error(`job ${armId} ${offset} exited ${code}`));
    });
  });
}

/** Mean of each statistic over the runs. */
function meanSummary(xs: Summary[]): Summary {
  const avg = (f: (s: Summary) => number) => xs.reduce((a, s) => a + f(s), 0) / xs.length;
  const st = (k: 'transAbs' | 'transSq' | 'rotAbs' | 'rotSq') => ({ mean: avg((s) => s[k].mean), std: avg((s) => s[k].std), max: Math.max(...xs.map((s) => s[k].max)) });
  return { n: xs[0]!.n, missing: xs[0]!.missing, transAbs: st('transAbs'), transSq: st('transSq'), rotAbs: st('rotAbs'), rotSq: st('rotSq') };
}

function aggregate(arm: Arm, runs: Run[], sweeps: number): ArmResult {
  runs.sort((a, b) => a.offset - b.offset);
  const avg = (f: (r: Run) => number) => runs.reduce((a, r) => a + f(r), 0) / runs.length;
  const trans = runs.map((r) => r.error.transAbs.mean);
  const rot = runs.map((r) => r.error.rotAbs.mean);
  return {
    id: arm.id,
    label: arm.label,
    maxRange: arm.maxRange,
    offsets: runs.length,
    error: meanSummary(runs.map((r) => r.error)),
    heldOut: meanSummary(runs.map((r) => r.heldOut)),
    local: meanSummary(runs.map((r) => r.local)),
    loops: meanSummary(runs.map((r) => r.loops)),
    quantiles: {
      transMedian: round(avg((r) => r.quantiles.transMedian)),
      transP90: round(avg((r) => r.quantiles.transP90)),
      rotMedian: round(avg((r) => r.quantiles.rotMedian), 3),
      rotP90: round(avg((r) => r.quantiles.rotP90), 3),
    },
    transRange: [round(Math.min(...trans)), round(Math.max(...trans))],
    rotRange: [round(Math.min(...rot), 3), round(Math.max(...rot), 3)],
    stats: runs[0]!.stats,
    lostSweeps: Math.round(avg((r) => r.lostSweeps)),
    msPerSweep: round(avg((r) => r.ms) / sweeps, 3),
    perRelation: runs[0]!.perRelation!,
  };
}

if (process.argv.includes('--job')) {
  const i = process.argv.indexOf('--job');
  const run = job(process.argv[i + 1]!, Number(process.argv[i + 2] ?? '0'));
  console.log(`RUN ${JSON.stringify(run)}`);
} else {
  const { log, rels } = loadLog(arg('data', 'data/intel-lab'));
  const armIds = arg('arms', '').split(',').filter(Boolean);
  const arms = armIds.length ? ARMS.filter((a) => armIds.includes(a.id)) : ARMS;
  const jobs = arms.flatMap((a) => (a.estimator ? GRID_OFFSETS.map((_, k) => [a.id, k] as const) : [[a.id, 0] as const]));
  const runs: Run[] = [];
  let next = 0;
  const workers = Array.from({ length: Math.min(jobs.length, Number(process.env.STUDY_SHARDS ?? availableParallelism())) }, async () => {
    while (next < jobs.length) {
      const [id, k] = jobs[next++]!;
      const r = await runJob(id, k);
      console.log(`${id.padEnd(20)} offset ${k}  trans ${r.error.transAbs.mean.toFixed(3)} m  rot ${r.error.rotAbs.mean.toFixed(2)}°  (${r.ms} ms)`);
      runs.push(r);
    }
  });
  await Promise.all(workers);
  const results = arms.map((a) => aggregate(a, runs.filter((r) => r.arm === a.id), log.n));
  for (const a of results) {
    console.log(
      `${a.id.padEnd(20)} trans ${a.error.transAbs.mean.toFixed(3)} ± ${a.error.transAbs.std.toFixed(3)} m (offsets ${a.transRange[0].toFixed(3)}–${a.transRange[1].toFixed(3)})  ` +
        `rot ${a.error.rotAbs.mean.toFixed(2)} ± ${a.error.rotAbs.std.toFixed(2)}°  held-out ${a.heldOut.transAbs.mean.toFixed(3)} m  local ${a.local.transAbs.mean.toFixed(3)} m  loops ${a.loops.transAbs.mean.toFixed(3)} m`,
    );
  }
  const out = arg('out', 'docs/intel-lab.json');
  const report = {
    commit: execSync('git rev-parse --short HEAD').toString().trim(),
    dataset: {
      name: 'Intel Research Lab, Seattle (Radish), Freiburg SLAM benchmark relations',
      sweeps: log.n,
      beams: log.beams,
      durationSec: round(log.t[log.n - 1]! - log.t[0]!, 1),
      relations: rels.length,
      devSweeps: DEV_SWEEPS,
      gridOffsets: GRID_OFFSETS,
    },
    metric: 'Relation-based error (Kümmerle et al. 2009): per relation, |trans| and |rot| of δ*⁻¹ δ; mean ± std over relations, averaged over the grid offsets',
    arms: results,
  };
  writeFileSync(out, `${JSON.stringify(report, null, 1)}\n`);
  console.log(`wrote ${out}`);
}

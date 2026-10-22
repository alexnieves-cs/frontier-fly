// Real-data check of the indoor pose estimator (src/loc): the Intel Research Lab (Seattle) log,
// scored with the Freiburg SLAM benchmark's relation-based error (Kümmerle et al. 2009).
//
//   npm run study:intel                       # all arms, writes docs/intel-lab.json
//   vite-node scripts/study/intel-lab.ts -- --arms fused --out /tmp/i.json [--poses /tmp/poses.txt]
//
// Data: data/intel-lab/ (vendored, CC BY; see data/intel-lab/README.md), refetched by
// scripts/loc/fetch-intel.sh. The estimator runs with its shipped parameters: nothing was tuned on
// this log. Arms that change a parameter say which one and why.
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { odomFrame, parseFlaser, parseRelations, relationError, ringScan, type LaserLog, type RelationError } from '../../src/loc/carmen';
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

export interface ArmResult {
  id: string;
  label: string;
  maxRange: number;
  error: Omit<RelationError, 'perTrans' | 'perRot'>;
  /** Relations whose sweeps both come after the dev sweeps. */
  heldOut: Omit<RelationError, 'perTrans' | 'perRot'>;
  /** Relations spanning under 10 s (local tracking) and the rest (loop closures: the same place minutes apart). */
  local: Omit<RelationError, 'perTrans' | 'perRot'>;
  loops: Omit<RelationError, 'perTrans' | 'perRot'>;
  /** Share of sweeps the estimator matched / rejected, re-acquisitions, heading fixes. */
  stats: Record<string, number> | null;
  /** Ticks the estimator called itself lost. */
  lostSweeps: number;
  /** Wall-clock, ms, and per sweep. */
  ms: number;
  msPerSweep: number;
  /** Median and 90th percentile of the per-relation errors (m, deg): the means are dominated by a tail. */
  quantiles: { transMedian: number; transP90: number; rotMedian: number; rotP90: number };
  /** Per-relation errors, translational (m) and rotational (deg), rounded, for plots and spot checks. */
  perRelation: { trans: number[]; rot: number[] };
}

function round(x: number, d = 4): number {
  return Number.isFinite(x) ? Math.round(x * 10 ** d) / 10 ** d : x;
}

function q(xs: Float64Array, p: number): number {
  const s = Array.from(xs).filter(Number.isFinite).sort((a, b) => a - b);
  return s.length ? s[Math.min(s.length - 1, Math.floor(p * s.length))]! : NaN;
}

function runArm(log: LaserLog, arm: Arm): { pose: Float64Array; stats: Record<string, number> | null; lost: number; ms: number } {
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
  const pad = 15;
  const opts = { ...defaultEstimatorOptions({ x0: x0 - pad, y0: y0 - pad, x1: x1 + pad, y1: y1 + pad }), ...arm.estimator };
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

const dir = arg('data', 'data/intel-lab');
const log = parseFlaser(gunzipSync(readFileSync(`${dir}/intel.flaser.gz`)).toString('utf8'));
const rels = parseRelations(readFileSync(`${dir}/intel.relations`, 'utf8'));
const index = new Map<number, number>();
log.t.forEach((ts, i) => index.set(ts, i));
const armIds = arg('arms', '').split(',').filter(Boolean);
const arms = armIds.length ? ARMS.filter((a) => armIds.includes(a.id)) : ARMS;
const results: ArmResult[] = [];
for (const arm of arms) {
  const r = runArm(log, arm);
  const strip = (e: RelationError): Omit<RelationError, 'perTrans' | 'perRot'> => ({
    n: e.n,
    missing: e.missing,
    transAbs: e.transAbs,
    transSq: e.transSq,
    rotAbs: e.rotAbs,
    rotSq: e.rotSq,
  });
  const err = relationError(log.t, r.pose, rels);
  const { perTrans, perRot } = err;
  const rest = strip(err);
  const heldOut = strip(relationError(log.t, r.pose, rels.filter((q) => index.get(q.t1)! >= DEV_SWEEPS && index.get(q.t2)! >= DEV_SWEEPS)));
  const local = strip(relationError(log.t, r.pose, rels.filter((q) => Math.abs(q.t2 - q.t1) < 10)));
  const loops = strip(relationError(log.t, r.pose, rels.filter((q) => Math.abs(q.t2 - q.t1) >= 10)));
  results.push({
    id: arm.id,
    label: arm.label,
    maxRange: arm.maxRange,
    error: rest,
    heldOut,
    quantiles: { transMedian: round(q(perTrans, 0.5)), transP90: round(q(perTrans, 0.9)), rotMedian: round(q(perRot, 0.5), 3), rotP90: round(q(perRot, 0.9), 3) },
    local,
    loops,
    stats: r.stats,
    lostSweeps: r.lost,
    ms: Math.round(r.ms),
    msPerSweep: round(r.ms / log.n, 3),
    perRelation: { trans: Array.from(perTrans, (x) => round(x)), rot: Array.from(perRot, (x) => round(x, 3)) },
  });
  console.log(
    `${arm.id.padEnd(18)} trans ${err.transAbs.mean.toFixed(3)} ± ${err.transAbs.std.toFixed(3)} m  ` +
      `rot ${err.rotAbs.mean.toFixed(2)} ± ${err.rotAbs.std.toFixed(2)}°  ` +
      `sq ${err.transSq.mean.toFixed(4)} m² / ${err.rotSq.mean.toFixed(2)}°²  (${err.n} rel, ${err.missing} missing, ${Math.round(r.ms)} ms)  ` +
      `held-out ${heldOut.transAbs.mean.toFixed(3)} m / ${heldOut.rotAbs.mean.toFixed(2)}° (${heldOut.n})  local ${local.transAbs.mean.toFixed(3)} m  loops ${loops.transAbs.mean.toFixed(3)} m`,
  );
  const posesOut = arg('poses', '');
  if (posesOut && arms.length === 1) {
    const lines = Array.from({ length: log.n }, (_, i) => `${log.t[i]!.toFixed(6)} ${r.pose[i * 3]!.toFixed(4)} ${r.pose[i * 3 + 1]!.toFixed(4)} ${r.pose[i * 3 + 2]!.toFixed(5)}`);
    writeFileSync(posesOut, lines.join('\n') + '\n');
  }
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
  },
  metric: 'Relation-based error (Kümmerle et al. 2009): per relation, |trans| and |rot| of δ*⁻¹ δ; mean ± std over relations',
  arms: results,
};
writeFileSync(out, `${JSON.stringify(report, null, 1)}\n`);
console.log(`wrote ${out}`);

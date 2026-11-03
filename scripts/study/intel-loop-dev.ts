// Dev runs for the loop closer (src/loc/graph.ts) on the Intel Research Lab log: only the first
// DEV_SWEEPS sweeps are read and only relations with both sweeps inside them are scored, so the
// parameters picked here never see the held-out part of the log.
//
//   vite-node scripts/study/intel-loop-dev.ts -- --front odom --opts '{"res":0.1}' [--sweeps 3000]
//
// --front odom (wheel odometry, which the estimator's odometry mode reproduces exactly) | adapted | adapted-noManhattan.
// The defaults in defaultLoopClosureOptions were picked this way, with --front odom: keyframes every
// 0.25 m / 0.15 rad instead of 0.5 m / 0.35 rad, a 30-keyframe sequential submap instead of 12, a
// short correlative search before each sequential match, and 5 cm instead of 10 cm cells.
import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { odomFrame, parseFlaser, parseRelations, relationError, ringScan, type LaserLog } from '../../src/loc/carmen';
import { defaultEstimatorOptions, PoseEstimator } from '../../src/loc/estimator';
import { LoopCloser } from '../../src/loc/graph';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const DEV = Number(arg('sweeps', '3000'));
const full = parseFlaser(gunzipSync(readFileSync('data/intel-lab/intel.flaser.gz')).toString('utf8'));
const n = Math.min(DEV, full.n);
const log: LaserLog = {
  ...full,
  n,
  t: full.t.slice(0, n),
  dt: full.dt.slice(0, n),
  odom: full.odom.slice(0, n * 3),
  ranges: full.ranges.slice(0, n * full.beams),
};
const index = new Map<number, number>();
log.t.forEach((ts, i) => index.set(ts, i));
const rels = parseRelations(readFileSync('data/intel-lab/intel.relations', 'utf8')).filter((r) => index.has(r.t1) && index.has(r.t2));
const maxRange = Number(arg('range', '8'));
const front = arg('front', 'odom');

const t0 = performance.now();
let pose: Float64Array = new Float64Array(n * 3);
if (front === 'odom') pose.set(log.odom);
else {
  const est0 = front === 'adapted-noManhattan' ? { res: 0.1, odomRel: 0.05, manhattan: false } : { res: 0.1, odomRel: 0.05 };
  let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
  for (let i = 0; i < full.n; i++) {
    x0 = Math.min(x0, full.odom[i * 3]!);
    x1 = Math.max(x1, full.odom[i * 3]!);
    y0 = Math.min(y0, full.odom[i * 3 + 1]!);
    y1 = Math.max(y1, full.odom[i * 3 + 1]!);
  }
  const base = { ...defaultEstimatorOptions({ x0: 0, y0: 0, x1: 0, y1: 0 }), ...est0 };
  const opts = { ...base, bounds: { x0: Math.floor(x0) - 15, y0: Math.floor(y0) - 15, x1: x1 + 15, y1: y1 + 15 } };
  const est = new PoseEstimator(opts);
  est.reset([log.odom[0]!, log.odom[1]!, 0], log.odom[2]!, 0, 0.01);
  est.correct(ringScan(log, 0, maxRange));
  pose.set([est.x, est.y, est.yaw], 0);
  for (let i = 1; i < n; i++) {
    const e = est.step(odomFrame(log, i, maxRange));
    pose.set([e.pos[0], e.pos[1], e.yaw], i * 3);
  }
}
const before = relationError(log.t, pose, rels);
const t1 = performance.now();
const lc = new LoopCloser({ maxRange, ...(JSON.parse(arg('opts', '{}')) as object) });
for (let i = 0; i < n; i++) lc.add(pose.subarray(i * 3, i * 3 + 3), ringScan(log, i, maxRange).ranges);
pose = lc.finish();
const after = relationError(log.t, pose, rels);
const local = rels.filter((r) => Math.abs(r.t2 - r.t1) < 10);
const loops = rels.filter((r) => Math.abs(r.t2 - r.t1) >= 10);
const f = (x: number) => x.toFixed(3);
console.log(
  JSON.stringify({
    front,
    rels: rels.length,
    before: [f(before.transAbs.mean), f(before.rotAbs.mean)],
    after: [f(after.transAbs.mean), f(after.rotAbs.mean)],
    afterLocal: f(relationError(log.t, pose, local).transAbs.mean),
    afterLoops: f(relationError(log.t, pose, loops).transAbs.mean),
    worst: f(after.transAbs.max),
    stats: lc.stats,
    frontMs: Math.round(t1 - t0),
    loopMs: Math.round(performance.now() - t1),
  }),
);

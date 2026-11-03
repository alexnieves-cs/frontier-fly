// Offline check of the pose-graph loop closer (src/loc/graph.ts) on the drone's own flights, before it is
// wired into missions. Each job's trajectory is flown once by the sensor-only planner on the true pose (as
// in loc-study.ts) and replayed open loop; the loop closer is fed every sweep with the front end's pose.
// Scored at every tick on the pose the graph has at that moment (causal, what a drone flying on it would
// see), against the truth in the building frame, and against the truth after removing the entry offset.
//
//   vite-node scripts/study/loc-loop.ts -- [--dev] [--seeds 10] [--opts '{"maxRange":8}'] [--out file.json]
import { writeFileSync } from 'node:fs';
import { locBounds, locDevJobs, locJobs, recordTrajectory, type LocJob } from '../../src/eval/localization';
import { LoopCloser, type LoopClosureOptions } from '../../src/loc/graph';
import { replay } from '../../src/loc/replay';
import { locNoise } from '../../src/loc/sensors';
import { LIDAR_MAX } from '../../src/shared/constants';
import { PhysicsWorld } from '../../src/sim/world';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

interface Track {
  sq: number;
  max: number;
  sqMap: number;
  n: number;
  e0: [number, number] | null;
}
const track = (): Track => ({ sq: 0, max: 0, sqMap: 0, n: 0, e0: null });
function score(t: Track, ex: number, ey: number): void {
  t.sq += ex * ex + ey * ey;
  t.max = Math.max(t.max, Math.hypot(ex, ey));
  if (!t.e0) t.e0 = [ex, ey];
  t.sqMap += (ex - t.e0[0]) ** 2 + (ey - t.e0[1]) ** 2;
  t.n += 1;
}
const rmse = (t: Track) => Math.sqrt(t.sq / Math.max(1, t.n));
const rmseMap = (t: Track) => Math.sqrt(t.sqMap / Math.max(1, t.n));

const opts = JSON.parse(arg('opts', '{}')) as Partial<LoopClosureOptions>;
const jobs: LocJob[] = process.argv.includes('--dev') ? locDevJobs() : locJobs(Number(arg('seeds', '10'))).filter((j) => j.condition === 'hard');
const shard = Number(arg('shard', '0'));
const of = Number(arg('of', '1'));
const rows: Record<string, unknown>[] = [];
for (const job of jobs.filter((_, i) => i % of === shard)) {
  const { spec, log, dark } = await recordTrajectory(job);
  if (spec.floors.length > 1) continue;
  const world = new PhysicsWorld(spec);
  const out: Record<string, unknown> = { seed: job.seed, lighting: job.lighting };
  for (const mode of ['fused', 'odometry'] as const) {
    const lc = new LoopCloser({ maxRange: LIDAR_MAX, ...opts });
    const front = track();
    const graph = track();
    let ms = 0;
    replay(log, world, locBounds(spec), 1, spec.floorHeight, job.seed, {
      noise: locNoise(dark),
      estimator: mode === 'odometry' ? { mode: 'odometry' } : {},
      onTick: (truth, est, _i, frame) => {
        score(front, est.pos[0] - truth.pos[0], est.pos[1] - truth.pos[1]);
        if (frame.scan) {
          const t0 = performance.now();
          lc.add([est.pos[0], est.pos[1], est.yaw], frame.scan.ranges);
          ms += performance.now() - t0;
        }
        const p = lc.current();
        // Between sweeps, carry the graph's correction of the newest sweep onto the front end's pose.
        score(graph, p[0] - truth.pos[0], p[1] - truth.pos[1]);
      },
    });
    out[mode] = {
      front: [rmse(front), rmseMap(front), front.max].map((x) => +x.toFixed(3)),
      graph: [rmse(graph), rmseMap(graph), graph.max].map((x) => +x.toFixed(3)),
      loops: `${lc.stats.loopAccepted}/${lc.stats.loopTried}`,
      keys: lc.stats.keyframes,
      msPerSweep: +(ms / Math.max(1, log.poses.length / 7 / 2)).toFixed(2),
    };
  }
  world.free();
  rows.push(out);
  console.log(JSON.stringify(out));
}
const mean = (k: 'fused' | 'odometry', w: 'front' | 'graph', i: number) =>
  rows.reduce((s, r) => s + ((r[k] as Record<string, number[]>)[w]![i] ?? 0), 0) / Math.max(1, rows.length);
const summary: Record<string, string> = {};
for (const k of ['fused', 'odometry'] as const)
  for (const w of ['front', 'graph'] as const)
    summary[`${k}.${w}`] = `rmse ${mean(k, w, 0).toFixed(3)} m, vs own map ${mean(k, w, 1).toFixed(3)} m, worst ${mean(k, w, 2).toFixed(3)} m`;
console.log(JSON.stringify({ n: rows.length, ...summary }, null, 1));
const o = arg('out', '');
if (o) writeFileSync(o, JSON.stringify({ rows, summary }, null, 1));

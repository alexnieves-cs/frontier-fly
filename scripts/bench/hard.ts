// Score pilots on FrontierBench Hard (src/bench/hard.ts) and write bench/results/hard/<id>.json.
//
//   npm run bench:hard -- --pilot planner-view --split dev --tasks sprint
//   npm run bench:hard -- --only planner,planner-view,nbvp --split test     # baselines, scored once
//   npm run bench:hard -- --track drift --pilot planner-view --split dev --truth   # drift tasks on the true pose (control)
//   npm run bench:hard -- --only planner-view,nbvp --base 95001 --count 30 --out bench/results/hard-95001  # a fresh ledger block
//
import { execSync } from 'node:child_process';
import { mkdirSync, writeFileSync } from 'node:fs';
import { HARD_TRACKS, HARD_VERSION, LOOP_IN_FLIGHT, trackRuns, trackTasks, type HardTrack } from '../../src/bench/hard';
import type { BenchTask } from '../../src/bench/tasks';
import { runBenchEpisode, type BenchEpisode } from '../../src/bench/run';
import { createEnv } from '../../src/sim/env';
import { loadPilot, summarize } from './lib';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const base = Number(arg('base', '0'));
const block = base > 0 ? { base, count: Number(arg('count', '30')) } : null;
const split = block ? 'report' : (arg('split', 'dev') as 'dev' | 'test');
if (split !== 'dev' && split !== 'test' && !block) throw new Error('--split dev|test, or --base N [--count M]');
const only = arg('tasks', '').split(',').filter(Boolean);
const ids = arg('only', arg('pilot', 'planner')).split(',');
const track = arg('track', 'hard') as HardTrack;
if (!HARD_TRACKS.includes(track)) throw new Error(`--track ${HARD_TRACKS.join('|')}`);
// --truth flies the drift tasks on the true pose: the paired control that prices the estimate.
// --healthy flies the sensor tasks without their faults (still on the estimate): the control that prices the faults.
const truth = process.argv.includes('--truth');
const healthy = process.argv.includes('--healthy');
// --loop flies the drift tasks on the in-flight loop closer's pose (hard.ts LOOP_IN_FLIGHT).
const loop = process.argv.includes('--loop');
if (truth && loop) throw new Error('--truth and --loop exclude each other');
const strip = (t: BenchTask): BenchTask =>
  truth ? { ...t, drift: false, faults: undefined } : healthy ? { ...t, faults: undefined } : loop && t.drift ? { ...t, driftProfile: { ...t.driftProfile, ...LOOP_IN_FLIGHT } } : t;
const TASKS = trackTasks(track).map(strip);
// A loop-closure row sits on the same board as the estimator-only rows (file <id>-loop.json).
const outDir = arg('out', split === 'test' ? (track === 'hard' ? 'bench/results/hard' : `bench/results/hard-${track}`) : '');
const commit = execSync('git rev-parse --short HEAD').toString().trim();
const env = await createEnv();
for (const id of ids) {
  const { make, source } = await loadPilot(id);
  const pilot = await make();
  const t0 = Date.now();
  const episodes: BenchEpisode[] = [];
  TASKS.forEach((task, ti) => {
    if (only.length && !only.includes(task.id)) return;
    const mine = trackRuns(track, block ?? (split as 'dev' | 'test'), ti).map((run) => runBenchEpisode(env, task, run, pilot));
    episodes.push(...mine);
    const m = (f: (e: BenchEpisode) => number) => ((mine.reduce((a, e) => a + f(e), 0) / mine.length) * 100).toFixed(1);
    if (task.person) console.log(`${id} ${task.id}: in view ${m((e) => e.watch!.score)}%, found ${mine.filter((e) => e.watch!.found).length}/${mine.length}, kept ${m((e) => e.watch!.kept)}% after the find, standoff breaches ${(mine.reduce((a, e) => a + e.watch!.breachS, 0) / mine.length).toFixed(1)} s mean`);
    if (task.integrity) console.log(`${id} ${task.id}: integrity ${m((e) => e.integrity!.score)}%, available ${m((e) => e.integrity!.availability)}%, HMI ${m((e) => e.integrity!.hmiRate)}%, bad pose ${m((e) => e.integrity!.badShare)}%, caught ${m((e) => e.integrity!.caught ?? 0)}% (0 with no bad pose)`);
    console.log(`${id} ${task.id}: camera ${m((e) => e.camera!.footprint)}%,${task.home ? ` returned ${mine.filter((e) => e.returned).length}/${mine.length}, seen ${m((e) => e.camera!.seen ?? 0)}%,` : ''} ring ${m((e) => e.footprint)}%, flown ${m((e) => e.coverage)}%, crashes ${mine.filter((e) => e.crashed).length}/${mine.length}${task.drift ? `, pose error ${(mine.reduce((a, e) => a + e.drift!.mean, 0) / mine.length).toFixed(2)} m mean, ${Math.max(...mine.map((e) => e.drift!.max)).toFixed(2)} m worst` : ''} (${Math.round((Date.now() - t0) / 1000)} s)`);
  });
  await pilot.close?.();
  const tasks = TASKS.filter((t) => !only.length || only.includes(t.id));
  const result = { hard: HARD_VERSION, ...(track !== 'hard' ? { track } : {}), ...(truth ? { truth } : {}), ...(healthy ? { healthy } : {}), ...(loop ? { loop: LOOP_IN_FLIGHT } : {}), name: loop ? `${pilot.name} + loop closure` : pilot.name, source, split, ...(block ? { block } : {}), commit, baseline: source.startsWith('baseline:'), taskDefs: tasks, ...summarize(episodes, tasks), episodes };
  if (result.watchScore !== undefined) console.log(`${id}: overwatch score ${(result.watchScore * 100).toFixed(1)}%`);
  if (result.integrityScore !== undefined) console.log(`${id}: integrity score ${(result.integrityScore * 100).toFixed(1)}%`);
  console.log(`${id}: camera score ${((result.cameraScore ?? 0) * 100).toFixed(1)}%, ring ${((result.footprintScore ?? 0) * 100).toFixed(1)}%, flown ${(result.score * 100).toFixed(1)}%, crashes ${episodes.filter((e) => e.crashed).length}/${episodes.length}`);
  if (outDir) {
    mkdirSync(outDir, { recursive: true });
    writeFileSync(`${outDir}/${id}${loop ? '-loop' : ''}.json`, `${JSON.stringify(result, null, 2)}\n`);
  }
}
env.dispose();

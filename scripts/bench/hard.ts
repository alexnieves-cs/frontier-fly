// Score pilots on FrontierBench Hard (src/bench/hard.ts) and write bench/results/hard/<id>.json.
//
//   npm run bench:hard -- --pilot planner-view --split dev --tasks sprint
//   npm run bench:hard -- --only planner,planner-view,nbvp --split test     # baselines, scored once
//   npm run bench:hard -- --track drift --pilot planner-view --split dev --truth   # drift tasks on the true pose (control)
//   npm run bench:hard -- --only planner-view,nbvp --base 95001 --count 30 --out bench/results/hard-95001  # a fresh ledger block
//
import { execSync } from 'node:child_process';
import { mkdirSync, writeFileSync } from 'node:fs';
import { LOOP_IN_FLIGHT, HARD_VERSION, trackRuns, trackTasks, type DriftTrack } from '../../src/bench/hard';
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
const track = arg('track', 'hard') as DriftTrack;
if (!['hard', 'return', 'drift', 'drift-hard'].includes(track)) throw new Error('--track hard|return|drift|drift-hard');
// --truth flies the drift tasks on the true pose: the paired control that prices the estimate.
const truth = process.argv.includes('--truth');
// --loop flies the drift tasks on the in-flight loop closer's pose (hard.ts LOOP_IN_FLIGHT).
const loop = process.argv.includes('--loop');
if (truth && loop) throw new Error('--truth and --loop exclude each other');
const TASKS = trackTasks(track).map((t) => (!t.drift ? t : truth ? { ...t, drift: false } : loop ? { ...t, driftProfile: { ...t.driftProfile, ...LOOP_IN_FLIGHT } } : t));
// A loop-closure row sits on the same board as the estimator-only rows (file <id>-loop.json); the true-pose control has its own folder.
const outDir = arg('out', split === 'test' ? (track === 'hard' ? 'bench/results/hard' : `bench/results/hard-${track}${truth ? '-truth' : ''}`) : '');
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
    console.log(`${id} ${task.id}: camera ${m((e) => e.camera!.footprint)}%,${task.home ? ` returned ${mine.filter((e) => e.returned).length}/${mine.length}, seen ${m((e) => e.camera!.seen ?? 0)}%,` : ''} ring ${m((e) => e.footprint)}%, flown ${m((e) => e.coverage)}%, crashes ${mine.filter((e) => e.crashed).length}/${mine.length}${task.drift ? `, pose error ${(mine.reduce((a, e) => a + e.drift!.mean, 0) / mine.length).toFixed(2)} m mean, ${Math.max(...mine.map((e) => e.drift!.max)).toFixed(2)} m worst` : ''} (${Math.round((Date.now() - t0) / 1000)} s)`);
  });
  await pilot.close?.();
  const tasks = TASKS.filter((t) => !only.length || only.includes(t.id));
  const result = { hard: HARD_VERSION, ...(track !== 'hard' ? { track } : {}), ...(truth ? { truth } : {}), ...(loop ? { loop: LOOP_IN_FLIGHT } : {}), name: loop ? `${pilot.name} + loop closure` : pilot.name, source, split, ...(block ? { block } : {}), commit, baseline: source.startsWith('baseline:'), taskDefs: tasks, ...summarize(episodes, tasks), episodes };
  console.log(`${id}: camera score ${((result.cameraScore ?? 0) * 100).toFixed(1)}%, ring ${((result.footprintScore ?? 0) * 100).toFixed(1)}%, flown ${(result.score * 100).toFixed(1)}%, crashes ${episodes.filter((e) => e.crashed).length}/${episodes.length}`);
  if (outDir) {
    mkdirSync(outDir, { recursive: true });
    writeFileSync(`${outDir}/${id}${loop ? '-loop' : ''}.json`, `${JSON.stringify(result, null, 2)}\n`);
  }
}
env.dispose();

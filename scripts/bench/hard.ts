// Score pilots on FrontierBench Hard (src/bench/hard.ts) and write bench/results/hard/<id>.json.
//
//   npm run bench:hard -- --pilot planner-view --split dev --tasks sprint
//   npm run bench:hard -- --only planner,planner-view,nbvp --split test     # baselines, scored once
//
import { execSync } from 'node:child_process';
import { mkdirSync, writeFileSync } from 'node:fs';
import { HARD_TASKS, HARD_VERSION, hardRuns } from '../../src/bench/hard';
import { runBenchEpisode, type BenchEpisode } from '../../src/bench/run';
import { createEnv } from '../../src/sim/env';
import { loadPilot, summarize } from './lib';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const split = arg('split', 'dev') as 'dev' | 'test';
if (split !== 'dev' && split !== 'test') throw new Error('--split dev|test');
const only = arg('tasks', '').split(',').filter(Boolean);
const ids = arg('only', arg('pilot', 'planner')).split(',');
const outDir = arg('out', split === 'test' ? 'bench/results/hard' : '');
const commit = execSync('git rev-parse --short HEAD').toString().trim();
const env = await createEnv();
for (const id of ids) {
  const { make, source } = await loadPilot(id);
  const pilot = await make();
  const t0 = Date.now();
  const episodes: BenchEpisode[] = [];
  HARD_TASKS.forEach((task, ti) => {
    if (only.length && !only.includes(task.id)) return;
    const mine = hardRuns(split, ti).map((run) => runBenchEpisode(env, task, run, pilot));
    episodes.push(...mine);
    const m = (f: (e: BenchEpisode) => number) => ((mine.reduce((a, e) => a + f(e), 0) / mine.length) * 100).toFixed(1);
    console.log(`${id} ${task.id}: camera ${m((e) => e.camera!.footprint)}%, ring ${m((e) => e.footprint)}%, flown ${m((e) => e.coverage)}%, crashes ${mine.filter((e) => e.crashed).length}/${mine.length} (${Math.round((Date.now() - t0) / 1000)} s)`);
  });
  await pilot.close?.();
  const tasks = HARD_TASKS.filter((t) => !only.length || only.includes(t.id));
  const result = { hard: HARD_VERSION, name: pilot.name, source, split, commit, baseline: source.startsWith('baseline:'), taskDefs: tasks, ...summarize(episodes, tasks), episodes };
  console.log(`${id}: camera score ${((result.cameraScore ?? 0) * 100).toFixed(1)}%, ring ${((result.footprintScore ?? 0) * 100).toFixed(1)}%, flown ${(result.score * 100).toFixed(1)}%, crashes ${episodes.filter((e) => e.crashed).length}/${episodes.length}`);
  if (outDir) {
    mkdirSync(outDir, { recursive: true });
    writeFileSync(`${outDir}/${id}.json`, `${JSON.stringify(result, null, 2)}\n`);
  }
}
env.dispose();

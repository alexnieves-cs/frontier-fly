// Score every baseline on the FrontierBench test split and write bench/results/<id>.json.
// npm run docs then renders docs/LEADERBOARD.md from those and bench/entries/*/result.json.
//
//   npm run bench:board                         # all baselines, one after another
//   npm run bench:board -- --only planner,guarded
import { execSync } from 'node:child_process';
import { mkdirSync, writeFileSync } from 'node:fs';
import { runBenchEpisode, type BenchEpisode } from '../../src/bench/run';
import { BENCH_TASKS, BENCH_VERSION, benchSeeds } from '../../src/bench/tasks';
import { createEnv } from '../../src/sim/env';
import { loadPilot, summarize } from './lib';

const IDS = ['planner', 'planner-momentum', 'guarded', 'policy', 'waypoint', 'heuristic'];
const i = process.argv.indexOf('--only');
const ids = i >= 0 ? process.argv[i + 1]!.split(',') : IDS;
const commit = execSync('git rev-parse --short HEAD').toString().trim();
mkdirSync('bench/results', { recursive: true });
const env = await createEnv();
for (const id of ids) {
  const { make, source } = await loadPilot(id);
  const pilot = await make();
  const episodes: BenchEpisode[] = [];
  BENCH_TASKS.forEach((task, ti) => {
    for (const seed of benchSeeds('test', ti)) episodes.push(runBenchEpisode(env, task, seed, pilot));
  });
  const result = { bench: BENCH_VERSION, name: pilot.name, source, split: 'test', commit, baseline: true, taskDefs: BENCH_TASKS, ...summarize(episodes), episodes };
  writeFileSync(`bench/results/${id}.json`, `${JSON.stringify(result, null, 2)}\n`);
  console.log(`${id}: score ${(result.score * 100).toFixed(1)}%`);
}
env.dispose();

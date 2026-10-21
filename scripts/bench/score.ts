// Score a pilot on FrontierBench (src/bench/tasks.ts).
//
//   npm run bench:score -- --pilot planner --split dev
//   npm run bench:score -- --pilot ./bench/entries/my-pilot/pilot.ts --split test --out bench/entries/my-pilot/result.json
//
// --pilot is a baseline id (planner, planner-momentum, policy, guarded, waypoint, heuristic) or a
// module whose default export is a BenchPilotFactory (src/bench/pilot.ts). --tasks narrows the run.
import { execSync } from 'node:child_process';
import { writeFileSync } from 'node:fs';
import { BENCH_TASKS, BENCH_VERSION, type BenchSplit } from '../../src/bench/tasks';
import { createEnv } from '../../src/sim/env';
import { flyAll, loadPilot, summarize } from './lib';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const split = arg('split', 'dev') as BenchSplit;
const only = arg('tasks', '').split(',').filter(Boolean);
const { make, source } = await loadPilot(arg('pilot', 'planner'));
const pilot = await make();
const env = await createEnv();
const t0 = Date.now();
const episodes = flyAll(env, pilot, split, only, (task, rs) =>
  console.log(`${task}: ${((rs.reduce((a, e) => a + e.coverage, 0) / rs.length) * 100).toFixed(1)}% over ${rs.length} flights (${Math.round((Date.now() - t0) / 1000)} s)`),
);
env.dispose();
const result = {
  bench: BENCH_VERSION,
  name: pilot.name,
  source,
  split,
  commit: execSync('git rev-parse --short HEAD').toString().trim(),
  taskDefs: BENCH_TASKS,
  ...summarize(episodes),
  episodes,
};
console.log(`${pilot.name}: score ${(result.score * 100).toFixed(1)}%, ${(result.safeShare * 100).toFixed(0)}% of flights touched nothing`);
const out = arg('out', '');
if (out) writeFileSync(out, `${JSON.stringify(result, null, 2)}\n`);

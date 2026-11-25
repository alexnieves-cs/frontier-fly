// Score a pilot on FrontierBench (src/bench/tasks.ts).
//
//   npm run bench:score -- --pilot planner --split dev
//   npm run bench:score -- --pilot ./bench/entries/my-pilot/pilot.ts --split test --out bench/entries/my-pilot/result.json
//
// --pilot is a baseline id (planner, planner-momentum, policy, guarded, waypoint, heuristic,
// yamauchi, nbvp, boustrophedon) or a
// module whose default export is a BenchPilotFactory (src/bench/pilot.ts). --tasks narrows the run.
//
// --base N flies a seed block outside the bench's splits instead (task i: N + 100 i, --count flights,
// default 30; apartments: the --plans dev|eval plans, default eval, env seeds N + 700 + j). Use it
// only for blocks recorded in the seed ledger; the result says which block it flew.
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
const baseArg = arg('base', '');
const block = baseArg ? { base: Number(baseArg), count: Number(arg('count', '30')), plans: arg('plans', 'eval') as 'dev' | 'eval' } : undefined;
if (block && !(Number.isInteger(block.base) && block.base > 0 && block.count > 0 && ['dev', 'eval'].includes(block.plans))) throw new Error('--base needs a positive integer, --count > 0, --plans dev|eval');
const pilot = await make();
const env = await createEnv();
const t0 = Date.now();
const episodes = flyAll(env, pilot, split, only, (task, rs) =>
  console.log(`${task}: ${((rs.reduce((a, e) => a + e.coverage, 0) / rs.length) * 100).toFixed(1)}% over ${rs.length} flights (${Math.round((Date.now() - t0) / 1000)} s)`),
  block,
);
env.dispose();
await pilot.close?.();
const result = {
  bench: BENCH_VERSION,
  name: pilot.name,
  source,
  split: block ? 'block' : split,
  ...(block ? { block } : {}),
  commit: execSync('git rev-parse --short HEAD').toString().trim(),
  taskDefs: BENCH_TASKS,
  ...summarize(episodes),
  episodes,
};
console.log(`${pilot.name}: score ${(result.score * 100).toFixed(1)}%, ${(result.safeShare * 100).toFixed(0)}% of flights touched nothing`);
const out = arg('out', '');
if (out) writeFileSync(out, `${JSON.stringify(result, null, 2)}\n`);

// Re-score submitted FrontierBench entries from their code and check the committed result.
// CI runs this on every pull request that touches bench/entries/.
//
//   npm run bench:verify                       # every entry
//   npm run bench:verify -- bench/entries/example
import { existsSync, readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { runBenchEpisode, type BenchEpisode } from '../../src/bench/run';
import { BENCH_TASKS, BENCH_VERSION, benchSeeds } from '../../src/bench/tasks';
import { createEnv } from '../../src/sim/env';
import { loadPilot, summarize } from './lib';

const args = process.argv.slice(2).filter((a) => a !== '--');
const dirs = args.length ? args : readdirSync('bench/entries').map((d) => join('bench/entries', d));
const env = await createEnv();
let failed = 0;
for (const dir of dirs) {
  const resultPath = join(dir, 'result.json');
  const pilotPath = join(dir, 'pilot.ts');
  if (!existsSync(resultPath) || !existsSync(pilotPath)) {
    console.log(`${dir}: needs pilot.ts and result.json`);
    failed += 1;
    continue;
  }
  const claimed = JSON.parse(readFileSync(resultPath, 'utf8')) as { bench: number; split: string; score: number };
  const { make } = await loadPilot(pilotPath);
  const pilot = await make();
  const episodes: BenchEpisode[] = [];
  BENCH_TASKS.forEach((task, ti) => {
    for (const seed of benchSeeds('test', ti)) episodes.push(runBenchEpisode(env, task, seed, pilot));
  });
  const got = summarize(episodes);
  // Flights are deterministic, so a re-score matches to rounding.
  const ok = claimed.bench === BENCH_VERSION && claimed.split === 'test' && Math.abs(got.score - claimed.score) < 1e-6;
  console.log(`${dir}: claimed ${(claimed.score * 100).toFixed(2)}%, re-scored ${(got.score * 100).toFixed(2)}% ${ok ? 'ok' : 'MISMATCH'}`);
  if (!ok) failed += 1;
}
env.dispose();
if (failed) process.exit(1);

// Re-score submitted FrontierBench entries from their code and check the committed result.
// CI runs this on every pull request that touches bench/entries/.
//
//   npm run bench:verify                       # every entry
//   npm run bench:verify -- bench/entries/example
import { existsSync, readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { BENCH_VERSION } from '../../src/bench/tasks';
import { createEnv } from '../../src/sim/env';
import { flyAll, loadPilot, mismatches, summarize } from './lib';

const args = process.argv.slice(2).filter((a) => a !== '--');
const dirs = args.length ? args : readdirSync('bench/entries').map((d) => join('bench/entries', d));
const env = await createEnv();
// Until every entry has been judged, an early exit (a pilot calling process.exit, a crash) is a failure.
process.exitCode = 1;
let failed = 0;
for (const dir of dirs) {
  const resultPath = join(dir, 'result.json');
  const pilotPath = join(dir, 'pilot.ts');
  // A team-track-only entry (team.ts) is verified by bench:team -- --verify.
  if (!existsSync(resultPath) && !existsSync(pilotPath) && existsSync(join(dir, 'team.ts'))) continue;
  if (!existsSync(resultPath) || !existsSync(pilotPath)) {
    console.log(`${dir}: needs pilot.ts and result.json`);
    failed += 1;
    continue;
  }
  const claimed = JSON.parse(readFileSync(resultPath, 'utf8')) as { bench: number; split: string; score: number; name?: string; baseline?: boolean } & Record<string, unknown>;
  const { make } = await loadPilot(pilotPath);
  const pilot = await make();
  const episodes = flyAll(env, pilot, 'test');
  const got = summarize(episodes);
  // Flights are deterministic, so a re-score matches to rounding. The leaderboard prints more than
  // the score (per-task coverage, footprint, the share of flights that touched nothing, the name),
  // so every summary field has to match, and an entry may not call itself a baseline.
  const diffs = [
    ...(claimed.bench === BENCH_VERSION ? [] : ['bench']),
    ...(claimed.split === 'test' ? [] : ['split']),
    ...(claimed.baseline ? ['baseline (entries are never baselines)'] : []),
    ...(claimed.name === pilot.name ? [] : ['name']),
    ...mismatches(got, claimed),
  ];
  const ok = diffs.length === 0;
  console.log(`${dir}: claimed ${(claimed.score * 100).toFixed(2)}%, re-scored ${(got.score * 100).toFixed(2)}% ${ok ? 'ok' : `MISMATCH in ${diffs.join(', ')}`}`);
  if (!ok) failed += 1;
}
env.dispose();
process.exitCode = failed ? 1 : 0;

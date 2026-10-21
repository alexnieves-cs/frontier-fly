// FrontierBench team track (src/bench/team.ts): score a team entry, re-score the baselines, or verify entries.
//
//   npm run bench:team -- --entry bench/entries/<name>/team.ts --split dev
//   npm run bench:team -- --entry bench/entries/<name>/team.ts --split test --out bench/entries/<name>/team-result.json
//   npm run bench:team -- --board                 # every baseline team on test, into bench/results/team/
//   npm run bench:team -- --verify [dirs]         # re-score entries that have team.ts and team-result.json
import { execSync } from 'node:child_process';
import { existsSync, mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { pathToFileURL } from 'node:url';
import { flyTeam, summarizeTeam, TEAM_BASELINES, TEAM_BENCH_TASKS, TEAM_BENCH_VERSION, type TeamEntry } from '../../src/bench/team';
import type { BenchSplit } from '../../src/bench/tasks';

const argv = process.argv.slice(2).filter((a) => a !== '--');
const flag = (k: string) => {
  const i = argv.indexOf(k);
  return i >= 0 ? argv[i + 1] : undefined;
};
const commit = execSync('git rev-parse --short HEAD').toString().trim();

async function loadEntry(id: string): Promise<TeamEntry> {
  if (TEAM_BASELINES[id]) return TEAM_BASELINES[id]!;
  const mod = (await import(pathToFileURL(resolve(id)).href)) as { default: TeamEntry };
  return mod.default;
}

async function score(entry: TeamEntry, split: BenchSplit, source: string, baseline: boolean, only: string[] = []) {
  const t0 = Date.now();
  const episodes = await flyTeam(entry, split, only, (task, eps) => {
    const s = summarizeTeam(eps).tasks[task]!;
    console.log(`  ${task}: coverage ${(s.coverage * 100).toFixed(1)}%, base knows ${(s.baseKnown * 100).toFixed(1)}%, 80% reached on ${Math.round(s.reached80 * 100)}% (${((Date.now() - t0) / 1000).toFixed(0)} s)`);
  });
  return { teamBench: TEAM_BENCH_VERSION, name: entry.name, entry, source, split, commit, baseline, taskDefs: TEAM_BENCH_TASKS, ...summarizeTeam(episodes), episodes };
}

if (argv.includes('--board')) {
  mkdirSync('bench/results/team', { recursive: true });
  const only = flag('--only')?.split(',');
  for (const id of Object.keys(TEAM_BASELINES)) {
    if (only && !only.includes(id)) continue;
    console.log(id);
    const r = await score(TEAM_BASELINES[id]!, 'test', `baseline:${id}`, true);
    writeFileSync(`bench/results/team/${id}.json`, `${JSON.stringify(r, null, 2)}\n`);
    console.log(`${id}: score ${(r.score * 100).toFixed(1)}%`);
  }
} else if (argv.includes('--verify')) {
  const dirs = argv.filter((a) => !a.startsWith('--'));
  const all = dirs.length ? dirs : readdirSync('bench/entries').map((d) => join('bench/entries', d));
  let failed = 0;
  for (const dir of all) {
    const resultPath = join(dir, 'team-result.json');
    const entryPath = join(dir, 'team.ts');
    if (!existsSync(entryPath) && !existsSync(resultPath)) continue;
    if (!existsSync(entryPath) || !existsSync(resultPath)) {
      console.log(`${dir}: a team entry needs team.ts and team-result.json`);
      failed += 1;
      continue;
    }
    const claimed = JSON.parse(readFileSync(resultPath, 'utf8')) as { teamBench: number; split: string; score: number };
    const got = await score(await loadEntry(entryPath), 'test', entryPath, false);
    // Team flights are deterministic too, so a re-score matches to rounding.
    const ok = claimed.teamBench === TEAM_BENCH_VERSION && claimed.split === 'test' && Math.abs(got.score - claimed.score) < 1e-6;
    console.log(`${dir}: claimed ${(claimed.score * 100).toFixed(2)}%, re-scored ${(got.score * 100).toFixed(2)}% ${ok ? 'ok' : 'MISMATCH'}`);
    if (!ok) failed += 1;
  }
  if (failed) process.exit(1);
} else {
  const id = flag('--entry');
  const split = (flag('--split') ?? 'dev') as BenchSplit;
  if (!id || (split !== 'dev' && split !== 'test')) throw new Error('usage: --entry <team.ts or baseline id> --split dev|test [--only hard,large] [--out file]');
  const r = await score(await loadEntry(id), split, id, Boolean(TEAM_BASELINES[id]), flag('--only')?.split(','));
  console.log(`score ${(r.score * 100).toFixed(1)}%`);
  const out = flag('--out');
  if (out) writeFileSync(out, `${JSON.stringify(r, null, 2)}\n`);
}

// Mission benchmark in parallel shards (src/eval/missions.ts), merged into docs/mission-bench.json.
//
//   npm run study:missions
//   vite-node scripts/study/mission-study.ts -- --shard 0 --of 4 --out /tmp/m0.jsonl
//   vite-node scripts/study/mission-study.ts -- --merge /tmp/m0.jsonl /tmp/m1.jsonl ... --out docs/mission-bench.json
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import {
  flyMission,
  MISSION_SEED_BASE,
  MISSION_SEEDS_PER_SCENARIO,
  missionBenchReport,
  MISSION_MODES,
  missionJobs,
  type MissionFlight,
  type MissionMode,
} from '../../src/eval/missions';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const seeds = Number(arg('seeds', String(MISSION_SEEDS_PER_SCENARIO)));
const base = Number(arg('base', String(MISSION_SEED_BASE)));
// Comma lists, for tuning runs: --modes sensor,sensorLoc --scenarios compound-clear
const modes = arg('modes', MISSION_MODES.join(',')).split(',') as MissionMode[];
const scenarioArg = arg('scenarios', '');

if (process.argv.includes('--merge')) {
  // Shard files are the bare arguments after --merge; every other --flag takes one value.
  const rest = process.argv.slice(process.argv.indexOf('--merge') + 1);
  const files = rest.filter((a, i) => !a.startsWith('--') && !(i > 0 && rest[i - 1]!.startsWith('--')));
  const flights: MissionFlight[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as MissionFlight),
  );
  const commit = execSync('git rev-parse --short HEAD').toString().trim();
  const out = arg('out', 'docs/mission-bench.json');
  writeFileSync(out, `${JSON.stringify(missionBenchReport(flights, commit, seeds, base), null, 2)}\n`);
  console.log(`wrote ${out} from ${flights.length} flights`);
} else if (process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `missions-${shard}.jsonl`);
  const mine = missionJobs(seeds, base, modes, scenarioArg ? scenarioArg.split(',') : undefined).filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    const f = await flyMission(mine[i]!);
    lines.push(JSON.stringify(f));
    writeFileSync(out, lines.join('\n') + '\n');
    console.log(
      `shard ${shard}: ${i + 1}/${mine.length} ${f.scenario} ${f.seed} ${f.indoor} ${f.outcome} (${Math.round((Date.now() - t0) / 1000)} s)`,
    );
  }
}

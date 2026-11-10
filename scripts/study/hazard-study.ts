// Hazard mission bench (src/eval/hazardBench.ts) in parallel shards, merged into docs/hazard-missions.json.
//
//   node scripts/study/parallel.mjs hazard-study docs/hazard-missions.json
//   vite-node scripts/study/hazard-study.ts -- --shard 0 --of 4 --out /tmp/h0.jsonl [--base 81901 --seeds 8 --modes sensor --scenarios smoke-cabin]
//   vite-node scripts/study/hazard-study.ts -- --merge /tmp/h0.jsonl ... --out docs/hazard-missions.json
//
// The thermal follow-up (smoke scenarios with and without a thermal camera, fresh block 87001+):
//   node scripts/study/parallel.mjs hazard-study docs/hazard-thermal.json --base 87001 --scenarios smoke-occupied,smoke-cabin --modes sensor,sensorThermal,policy,policyThermal
//
// The hazard coverage rows come from the coverage study's hazard conditions:
//   node scripts/study/parallel.mjs coverage-study docs/hazard-coverage.json --conditions hazard-control,doors,glass,smoke
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import {
  flyHazard,
  HAZARD_MISSION_SEED_BASE,
  HAZARD_MISSION_SEEDS,
  HAZARD_ALL_MODES,
  HAZARD_MODES,
  hazardBenchReport,
  hazardJobs,
  type HazardFlight,
  type HazardMode,
} from '../../src/eval/hazardBench';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const seeds = Number(arg('seeds', String(HAZARD_MISSION_SEEDS)));
const base = Number(arg('base', String(HAZARD_MISSION_SEED_BASE)));
const modes = arg('modes', HAZARD_MODES.join(','))
  .split(',')
  .filter(Boolean) as HazardMode[];
const bad = modes.filter((m) => !HAZARD_ALL_MODES.includes(m));
if (bad.length) throw new Error(`Unknown hazard mode(s): ${bad.join(', ')}. Known: ${HAZARD_ALL_MODES.join(', ')}.`);
const scenarios = arg('scenarios', '').split(',').filter(Boolean);

if (process.argv.includes('--merge')) {
  const rest = process.argv.slice(process.argv.indexOf('--merge') + 1);
  const files = rest.filter((a) => a.endsWith('.jsonl'));
  const flights: HazardFlight[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as HazardFlight),
  );
  const commit = execSync('git rev-parse --short HEAD').toString().trim();
  const out = arg('out', 'docs/hazard-missions.json');
  writeFileSync(out, `${JSON.stringify(hazardBenchReport(flights, commit, seeds, base), null, 2)}\n`);
  console.log(`wrote ${out} from ${flights.length} flights`);
} else if (process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `hazards-${shard}.jsonl`);
  const mine = hazardJobs(seeds, base, modes, scenarios.length ? scenarios : undefined).filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    const f = await flyHazard(mine[i]!);
    lines.push(JSON.stringify(f));
    writeFileSync(out, lines.join('\n') + '\n');
    console.log(`shard ${shard}: ${i + 1}/${mine.length} ${f.scenario} ${f.seed} ${f.mode} ${f.outcome} fc=${f.falseClears} doors=${f.shutDoors ?? 0} (${Math.round((Date.now() - t0) / 1000)} s)`);
  }
}

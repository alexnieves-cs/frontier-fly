// Fault bench in parallel shards (src/eval/faults.ts), merged into docs/fault-bench.json.
//
//   npm run study:faults
//   vite-node scripts/study/fault-study.ts -- --shard 0 --of 4 --out /tmp/f0.jsonl
//   vite-node scripts/study/fault-study.ts -- --merge /tmp/f0.jsonl /tmp/f1.jsonl ... --out docs/fault-bench.json
//   --fault <id>[,<id>...] runs only those fault scenarios, --seeds N changes the seeds per mission scenario, --base B the first seed,
//   --vehicles quad,kinematic the airframes (default quad).
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { FAULT_SEED_BASE, FAULT_SEEDS_PER_SCENARIO, faultBenchReport, faultJobs, flyFault, type FaultFlight, type FaultVehicle } from '../../src/eval/faults';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const seeds = Number(arg('seeds', String(FAULT_SEEDS_PER_SCENARIO)));
const only = arg('fault', '').split(',').filter(Boolean);
// Development runs use --base 9101 (the dev block); the published bench uses the fresh default.
const base = Number(arg('base', String(FAULT_SEED_BASE)));
const vehicles = arg('vehicles', 'quad').split(',') as FaultVehicle[];

if (process.argv.includes('--merge')) {
  const files = process.argv.slice(process.argv.indexOf('--merge') + 1).filter((a) => a.endsWith('.jsonl'));
  const flights: FaultFlight[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as FaultFlight),
  );
  const commit = execSync('git rev-parse --short HEAD').toString().trim();
  const out = arg('out', 'docs/fault-bench.json');
  writeFileSync(out, `${JSON.stringify(faultBenchReport(flights, commit, seeds, base), null, 2)}\n`);
  console.log(`wrote ${out} from ${flights.length} flights`);
} else if (process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `faults-${shard}.jsonl`);
  const mine = faultJobs(seeds, base, vehicles)
    .filter((j) => !only.length || only.includes(j.fault))
    .filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    const f = await flyFault(mine[i]!);
    lines.push(JSON.stringify(f));
    writeFileSync(out, lines.join('\n') + '\n');
    console.log(
      `shard ${shard}: ${i + 1}/${mine.length} ${f.vehicle} ${f.fault} ${f.scenario} ${f.seed} ${f.response}${f.safe ? '' : ` UNSAFE (${f.unsafeReasons.join(', ')})`} (${Math.round((Date.now() - t0) / 1000)} s)`,
    );
  }
}

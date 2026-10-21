// Outdoor search benchmark in parallel shards (src/eval/sar.ts), merged into docs/sar-bench.json.
//
//   npm run study:sar
//   vite-node scripts/study/sar-study.ts -- --shard 0 --of 4 --out /tmp/s0.jsonl [--dev]
//   vite-node scripts/study/sar-study.ts -- --merge /tmp/s0.jsonl ... --out docs/sar-bench.json
//
// --dev flies the development seeds (7100-7139), the only ones the planner and prior were tuned on.
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { flySar, sarBenchReport, sarJobs, sweepWidthSpacing, sarScene, SAR_DEV_SEED_BASE, SAR_SEED_BASE, type SarFlight } from '../../src/eval/sar';
import { calibratePod } from '../../src/eval/sarCalibration';
import { drawBelief } from '../../src/eval/sarFigure';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const dev = process.argv.includes('--dev');
const scale = Number(arg('scale', '1'));

if (process.argv.includes('--merge')) {
  const out = arg('out', 'docs/sar-bench.json');
  const files = process.argv.slice(process.argv.indexOf('--merge') + 1).filter((a) => !a.startsWith('--') && a !== out && a !== arg('scale', ''));
  const flights: SarFlight[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as SarFlight),
  );
  flights.sort((a, b) => a.condition.localeCompare(b.condition) || a.seed - b.seed || a.method.localeCompare(b.method) || Number(!!a.blind) - Number(!!b.blind));
  const commit = execSync('git rev-parse --short HEAD').toString().trim();
  // The calibration set is drawn on development seeds, never on the evaluation worlds.
  const calibration = calibratePod(SAR_DEV_SEED_BASE, 20);
  const day = sarScene(SAR_SEED_BASE, 'day');
  const night = sarScene(SAR_SEED_BASE, 'night');
  const meta = {
    dev,
    seedBase: dev ? SAR_DEV_SEED_BASE : SAR_SEED_BASE,
    sweepWidthM: { day: sweepWidthSpacing(day), night: sweepWidthSpacing(night) },
    podCalibration: calibration,
  };
  writeFileSync(out, `${JSON.stringify(sarBenchReport(flights, commit, meta), null, 1)}\n`);
  console.log(`wrote ${out} from ${flights.length} flights`);
  // The illustration is drawn on a development seed, so it can be redrawn without touching evaluation worlds.
  await drawBelief();
} else if (process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `sar-${shard}.jsonl`);
  const mine = sarJobs(dev, scale).filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    const job = mine[i]!;
    const f = await flySar(job.seed, job.condition, job.method, job.blind);
    lines.push(JSON.stringify(f));
    writeFileSync(out, lines.join('\n') + '\n');
    console.log(
      `shard ${shard}: ${i + 1}/${mine.length} ${f.condition} ${f.seed} ${f.method}${f.blind ? ' blind' : ''} ${f.found ? `found ${f.tFind} s` : f.outcome} (${Math.round((Date.now() - t0) / 1000)} s)`,
    );
  }
}

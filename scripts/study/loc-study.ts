// Localization study: the indoor pose estimator (src/loc) replayed open loop over trajectories
// the sensor-only planner flew, on fresh buildings, against dead reckoning and scan-only baselines.
//
//   npm run study:loc                                   # all shards in parallel, then merge into docs/localization.json
//   vite-node scripts/study/loc-study.ts -- --shard 0 --of 4 --out /tmp/l0.jsonl [--dev] [--seeds 5] [--arms fused,odometry]
//   vite-node scripts/study/loc-study.ts -- --merge /tmp/l0.jsonl ... --out docs/localization.json
//
// Seeds 60001+ (hard) and 62001+ (easy) are drawn only here and never tuned on. --dev runs the
// tuning set (stage 3, seeds 7001-7012), the only buildings any estimator parameter was chosen on.
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import {
  evaluateTrajectory,
  LOC_ARMS,
  LOC_CONDITIONS,
  LOC_DEV_SEED_BASE,
  LOC_DEV_SEEDS,
  locDevJobs,
  locJobs,
  recordTrajectory,
  type LocRow,
} from '../../src/eval/localization';
import { RNG } from '../../src/shared/rng';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

/** Bootstrap 95% interval of the mean, deterministic (same as coverage-study.ts, which can't be imported: it runs on import). */
function bootstrapCi(xs: number[], resamples = 2000): [number, number] {
  if (!xs.length) return [0, 0];
  const rng = new RNG(7, 'bootstrap');
  const means: number[] = [];
  for (let b = 0; b < resamples; b++) {
    let sum = 0;
    for (let i = 0; i < xs.length; i++) sum += xs[Math.floor(rng.next() * xs.length)]!;
    means.push(sum / xs.length);
  }
  means.sort((a, b) => a - b);
  return [means[Math.floor(resamples * 0.025)]!, means[Math.floor(resamples * 0.975)]!];
}

const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
const median = (xs: number[]) => {
  if (!xs.length) return 0;
  const s = [...xs].sort((a, b) => a - b);
  return s.length % 2 ? s[(s.length - 1) / 2]! : (s[s.length / 2 - 1]! + s[s.length / 2]!) / 2;
};

export function summarizeLoc(rows: LocRow[]) {
  const cells = [];
  const groups = new Map<string, LocRow[]>();
  for (const r of rows) {
    const key = `${r.condition}|${r.lighting}|${r.arm}`;
    const g = groups.get(key) ?? [];
    g.push(r);
    groups.set(key, g);
  }
  for (const [key, rs] of groups) {
    const [condition, lighting, arm] = key.split('|');
    const ate = rs.map((r) => r.metrics.ateRmse);
    const big = rs.reduce((n, r) => n + r.metrics.bigErrTicks, 0);
    const bigFlagged = rs.reduce((n, r) => n + r.metrics.bigErrTicks * r.metrics.failDetected, 0);
    const lost = rs.reduce((n, r) => n + r.metrics.lostTicks, 0);
    const lostRight = rs.reduce((n, r) => n + r.metrics.lostTicks * r.metrics.lostPrecision, 0);
    const recovered = rs.filter((r) => r.recoverTicks >= 0);
    cells.push({
      condition,
      lighting,
      arm,
      flights: rs.length,
      pathM: mean(rs.map((r) => r.metrics.pathM)),
      ateRmse: mean(ate),
      ateRmseCi: bootstrapCi(ate),
      ateMedian: median(ate),
      ateMapRmse: mean(rs.map((r) => r.metrics.ateMapRmse)),
      ateMapRmseCi: bootstrapCi(rs.map((r) => r.metrics.ateMapRmse)),
      ateMax: Math.max(...rs.map((r) => r.metrics.ateMax)),
      finalErr: mean(rs.map((r) => r.metrics.finalErr)),
      driftPct: mean(rs.map((r) => r.metrics.driftPct)),
      rpe5m: mean(rs.map((r) => r.metrics.rpe5m)),
      yawRmseDeg: mean(rs.map((r) => r.metrics.yawRmseDeg)),
      zRmse: mean(rs.map((r) => r.metrics.zRmse)),
      within95: mean(rs.map((r) => r.metrics.within95)),
      within95Abs: mean(rs.map((r) => r.metrics.within95Abs)),
      nees: median(rs.map((r) => r.metrics.nees)),
      flightsOver50cm: rs.filter((r) => r.metrics.bigErrTicks > 0).length,
      failDetected: big ? bigFlagged / big : null,
      lostPrecision: lost ? lostRight / lost : null,
      seenIoU: mean(rs.map((r) => r.seenIoU)),
      seenFalseFree: mean(rs.map((r) => r.seenFalseFree)),
      recovered: rs.some((r) => r.arm === 'kidnap') ? recovered.length : null,
      recoverSec: recovered.length ? median(recovered.map((r) => r.recoverTicks * 0.05)) : null,
      msPerFlight: mean(rs.map((r) => r.ms)),
    });
  }
  return cells;
}

if (process.argv.includes('--merge')) {
  const outPath = arg('out', 'docs/localization.json');
  const files = process.argv.slice(process.argv.indexOf('--merge') + 1).filter((a) => !a.startsWith('--') && a !== outPath);
  const rows: LocRow[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as LocRow),
  );
  const report = {
    commit: execSync('git rev-parse --short HEAD').toString().trim(),
    dev: process.argv.includes('--dev'),
    seeds: process.argv.includes('--dev')
      ? { dev: `${LOC_DEV_SEED_BASE}-${LOC_DEV_SEED_BASE + LOC_DEV_SEEDS - 1}` }
      : Object.fromEntries(LOC_CONDITIONS.map((c) => [c.id, c.seedBase])),
    arms: LOC_ARMS.map((a) => ({ id: a.id, label: a.label })),
    conditions: LOC_CONDITIONS.map((c) => ({ id: c.id, label: c.label })),
    cells: summarizeLoc(rows),
    /** Shipped-estimator flights that declared itself lost while within 0.5 m of the truth. */
    falseLostFlights: rows.filter((r) => r.arm === 'fused' && r.metrics.lostTicks > 0 && r.metrics.lostPrecision < 0.5).length,
  };
  writeFileSync(outPath, `${JSON.stringify(report, null, 2)}\n`);
  console.log(`wrote ${outPath} from ${rows.length} rows`);
} else if (process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `loc-${shard}.jsonl`);
  const armIds = arg('arms', '').split(',').filter(Boolean);
  const arms = armIds.length ? LOC_ARMS.filter((a) => armIds.includes(a.id)) : LOC_ARMS;
  const overrides = JSON.parse(arg('override', '{}'));
  const all = process.argv.includes('--dev') ? locDevJobs() : locJobs(Number(arg('seeds', '30')));
  const mine = all.filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    const job = mine[i]!;
    const { spec, log, dark } = await recordTrajectory(job);
    const rows = evaluateTrajectory(job, spec, log, dark, arms, overrides);
    for (const r of rows) lines.push(JSON.stringify(r));
    writeFileSync(out, lines.join('\n') + '\n');
    const f = rows.find((r) => r.arm === 'fused') ?? rows[0]!;
    console.log(
      `shard ${shard}: ${i + 1}/${mine.length} ${job.condition} ${job.seed} ${job.lighting} ${f.arm} ate ${f.metrics.ateRmse.toFixed(3)} max ${f.metrics.ateMax.toFixed(2)} in95 ${f.metrics.within95.toFixed(2)} (${Math.round((Date.now() - t0) / 1000)} s)`,
    );
  }
}

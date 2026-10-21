// Drone-team study: 1 to 4 sensor-only planners sharing maps over a lossy, wall-attenuated radio.
//
//   npm run study:team                     # all shards in parallel, then merge into docs/team-bench.json
//   vite-node scripts/study/team-study.ts -- --shard 0 --of 4 --out /tmp/t0.jsonl
//   vite-node scripts/study/team-study.ts -- --tune --out docs/team-tuning.json   # claim weights, tuning seeds only
//
// Seeds: the claim weights were tuned on 66001-66016 (both conditions). The eval blocks 70001+ (hard)
// and 71001+ (large) are drawn only here, after tuning, and no choice was made on them.
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { bootstrapCi } from './common';
import { DEFAULT_CLAIMS, type ClaimOptions } from '../../src/team/allocate';
import { TEAM_CONDITIONS, teamEpisodeSetup } from '../../src/team/conditions';
import { runTeam, TIME_KEYS, type Comms, type Role, type TeamResult } from '../../src/team/run';
import { DEFAULT_BATTERY } from '../../src/mission/autonomy/battery';

export const SEEDS_PER_CONDITION = 30;
export const EVAL_SEED_BASE: Record<string, number> = { hard: 70001, large: 71001 };
export const TUNE_SEEDS = Array.from({ length: 16 }, (_, i) => 66001 + i);

export interface Arm {
  id: string;
  label: string;
  drones: number;
  comms: Comms;
  roles?: Role[];
  claims?: ClaimOptions | null;
  txPower?: number;
  /** Conditions this arm runs in (all when absent). */
  only?: string[];
}

export const ARMS: Arm[] = [
  { id: 'solo', label: '1 drone', drones: 1, comms: 'radio' },
  { id: 'team2', label: '2 drones', drones: 2, comms: 'radio' },
  { id: 'team3', label: '3 drones', drones: 3, comms: 'radio' },
  { id: 'team4', label: '4 drones', drones: 4, comms: 'radio' },
  { id: 'team3-none', label: '3 drones, no radio (each flies alone)', drones: 3, comms: 'none' },
  { id: 'team3-noclaims', label: '3 drones, shared maps, no frontier split', drones: 3, comms: 'radio', claims: null },
  { id: 'team3-perfect', label: '3 drones, perfect radio', drones: 3, comms: 'perfect' },
  { id: 'team3-relay', label: '2 explorers + 1 relay', drones: 3, comms: 'radio', roles: ['explore', 'explore', 'relay'] },
  { id: 'team3-weak', label: '3 drones, weak radio (-10 dBm)', drones: 3, comms: 'radio', txPower: -10, only: ['large'] },
  { id: 'team3-weak-relay', label: '2 explorers + 1 relay, weak radio (-10 dBm)', drones: 3, comms: 'radio', txPower: -10, roles: ['explore', 'explore', 'relay'], only: ['large'] },
];

export interface Job {
  condition: string;
  seed: number;
  arm: string;
}

export type Row = Job & Omit<TeamResult, 'curve' | 'baseCurve' | 'tracks'> & { curve: number[]; baseCurve: number[] };

export function jobs(): Job[] {
  const out: Job[] = [];
  for (const c of TEAM_CONDITIONS) {
    for (let i = 0; i < SEEDS_PER_CONDITION; i++) {
      for (const a of ARMS) if (!a.only || a.only.includes(c.id)) out.push({ condition: c.id, seed: EVAL_SEED_BASE[c.id]! + i, arm: a.id });
    }
  }
  return out;
}

export async function runJob(job: Job, claimsOverride?: ClaimOptions): Promise<Row> {
  const cond = TEAM_CONDITIONS.find((c) => c.id === job.condition)!;
  const arm = ARMS.find((a) => a.id === job.arm)!;
  const { spec, envSeed, cfg } = teamEpisodeSetup(cond, job.seed);
  const claims = arm.claims === null ? null : (claimsOverride ?? arm.claims ?? DEFAULT_CLAIMS);
  const r = await runTeam(spec, envSeed, cfg, { drones: arm.drones, comms: arm.comms, roles: arm.roles, claims, radio: arm.txPower !== undefined ? { txPower: arm.txPower } : undefined }, job.seed);
  // Curves are kept at 5 s resolution in the JSON.
  const thin = (xs: number[]) => xs.filter((_, i) => i % 5 === 4).map((v) => Math.round(v * 1e4) / 1e4);
  return { ...job, ...r, curve: thin(r.curve), baseCurve: thin(r.baseCurve) };
}

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
const sum = (xs: number[]) => xs.reduce((a, b) => a + b, 0);

/** Paired difference arm - base over the same seeds, with a bootstrap interval. */
function paired(rows: Row[], condition: string, arm: string, base: string, key: (r: Row) => number) {
  const a = new Map(rows.filter((r) => r.condition === condition && r.arm === arm).map((r) => [r.seed, key(r)]));
  const diffs = rows.filter((r) => r.condition === condition && r.arm === base && a.has(r.seed)).map((r) => a.get(r.seed)! - key(r));
  return { mean: mean(diffs), ci: bootstrapCi(diffs), n: diffs.length };
}

export function summarize(rows: Row[]) {
  const cells = [];
  for (const c of TEAM_CONDITIONS) {
    for (const a of ARMS) {
      const rs = rows.filter((r) => r.condition === c.id && r.arm === a.id);
      if (!rs.length) continue;
      const cov = rs.map((r) => r.coverage);
      const minutes = c.maxSteps / 20 / 60;
      const timeTo: Record<string, { reached: number; medianSeconds: number | null }> = {};
      for (const k of TIME_KEYS) {
        const ts = rs.map((r) => r.timeTo[k]).filter((t): t is number => t !== null).sort((x, y) => x - y);
        // Median over all episodes, counting a miss as slower than any hit; null when half or more missed.
        timeTo[k] = { reached: ts.length, medianSeconds: ts.length * 2 > rs.length ? ts[Math.floor(rs.length / 2)]! / 20 : null };
      }
      cells.push({
        condition: c.id,
        arm: a.id,
        label: a.label,
        drones: a.drones,
        episodes: rs.length,
        coverage: mean(cov),
        coverageCi: bootstrapCi(cov),
        baseKnown: mean(rs.map((r) => r.baseKnown)),
        awarenessGap: mean(rs.map((r) => r.awarenessGap)),
        awarenessGapMax: mean(rs.map((r) => r.awarenessGapMax)),
        // Coverage per drone-minute of the full budget, in percentage points.
        coveragePerDroneMinute: (mean(cov) * 100) / (a.drones * minutes),
        timeTo,
        overlap: mean(rs.map((r) => r.overlap)),
        // Battery energy (DEFAULT_BATTERY power model): the team's total, the most any one drone used as a share of
        // its usable pack, and coverage points per team Wh.
        energyWh: mean(rs.map((r) => sum(r.energyWh))),
        maxPackUsed: mean(rs.map((r) => Math.max(...r.energyWh) / (DEFAULT_BATTERY.capacityWh * DEFAULT_BATTERY.usableFraction))),
        coveragePerWh: mean(rs.map((r) => (r.coverage * 100) / Math.max(1e-9, sum(r.energyWh)))),
        collisionsPerEpisode: mean(rs.map((r) => r.collisions)),
        droneContactsPerEpisode: mean(rs.map((r) => r.droneContacts)),
        delivery: mean(rs.map((r) => r.packets.delivered / Math.max(1, r.packets.sent))),
        kilobytesPerDroneMinute: mean(rs.map((r) => r.packets.bytes / 1000 / (a.drones * (r.steps / 20 / 60)))),
        coverageVsSolo: a.id === 'solo' ? null : paired(rows, c.id, a.id, 'solo', (r) => r.coverage),
        coverageVsTeam3: a.drones === 3 && a.id !== 'team3' ? paired(rows, c.id, a.id, 'team3', (r) => r.coverage) : null,
      });
    }
  }
  return cells;
}

if (process.argv.includes('--tune')) {
  // Grid over the claim weights on the tuning seeds, 3 drones, both conditions.
  const out = arg('out', 'docs/team-tuning.json');
  const grid: ClaimOptions[] = [];
  // Round 1: both weights, no goal order. Round 2 (--priority): only lower-numbered teammates' goals count.
  const priority = process.argv.includes('--priority');
  const goals = priority ? [12, 24, 48] : [0, 12, 24, 48];
  const nearer = priority ? [0, 8, 16] : [0, 4, 8, 16];
  for (const goalWeight of goals) for (const nearerWeight of nearer) grid.push({ ...DEFAULT_CLAIMS, goalWeight, nearerWeight, priority });
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const results = [];
  for (let g = 0; g < grid.length; g++) {
    if (g % of !== shard) continue;
    const claims = grid[g]!;
    const rows: Row[] = [];
    for (const condition of ['hard', 'large']) for (const seed of TUNE_SEEDS) rows.push(await runJob({ condition, seed, arm: 'team3' }, claims));
    const by = (c: string) => rows.filter((r) => r.condition === c);
    const res = {
      claims,
      hardCoverage: mean(by('hard').map((r) => r.coverage)),
      hardSecondsTo80: mean(by('hard').map((r) => (r.timeTo['80'] ?? 2400) / 20)),
      largeCoverage: mean(by('large').map((r) => r.coverage)),
    };
    results.push(res);
    console.log(JSON.stringify(res));
  }
  writeFileSync(out, `${JSON.stringify(results, null, 2)}\n`);
} else if (process.argv.includes('--merge')) {
  const outPath = arg('out', 'docs/team-bench.json');
  const files = process.argv.slice(process.argv.indexOf('--merge') + 1).filter((a) => !a.startsWith('--') && a !== outPath);
  const rows: Row[] = files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as Row),
  );
  rows.sort((a, b) => a.condition.localeCompare(b.condition) || a.arm.localeCompare(b.arm) || a.seed - b.seed);
  const report = {
    schema: 1,
    commit: execSync('git rev-parse --short HEAD').toString().trim(),
    seedsPerCondition: SEEDS_PER_CONDITION,
    evalSeedBase: EVAL_SEED_BASE,
    tuneSeeds: [TUNE_SEEDS[0], TUNE_SEEDS[TUNE_SEEDS.length - 1]],
    claims: DEFAULT_CLAIMS,
    conditions: TEAM_CONDITIONS,
    arms: ARMS,
    cells: summarize(rows),
    episodes: rows,
  };
  writeFileSync(outPath, `${JSON.stringify(report, null, 1)}\n`);
  console.log(`wrote ${outPath} from ${rows.length} episodes`);
} else if (process.argv.includes('--shard')) {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `team-${shard}.jsonl`);
  const arms = arg('arms', '').split(',').filter(Boolean);
  const mine = jobs()
    .filter((j) => !arms.length || arms.includes(j.arm))
    .filter((_, i) => i % of === shard);
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < mine.length; i++) {
    lines.push(JSON.stringify(await runJob(mine[i]!)));
    if (i % 20 === 0) {
      writeFileSync(out, lines.join('\n') + '\n');
      console.log(`shard ${shard}: ${i + 1}/${mine.length} (${Math.round((Date.now() - t0) / 1000)} s)`);
    }
  }
  writeFileSync(out, lines.join('\n') + '\n');
  console.log(`shard ${shard}: done, ${mine.length} episodes in ${Math.round((Date.now() - t0) / 1000)} s`);
}

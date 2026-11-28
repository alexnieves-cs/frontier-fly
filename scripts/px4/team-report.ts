/**
 * Score the PX4 team study (scripts/px4/team.ts) and print the report tables.
 *
 *   npx vite-node scripts/px4/team-report.ts -- --dir DIR [--conds hard,large] [--out docs/px4-team.json]
 *
 * Reads DIR/<cond>/summary.json. Only buildings that every arm and team size has finished count, so
 * every comparison is paired by building. Coverage intervals are bootstrap 95% intervals of the mean
 * (of the paired difference, for the comparisons). Time to 50% and 80% is the median over buildings,
 * with a building that never got there counted as never; "not reached" means half or more never did.
 */
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { join, resolve } from 'node:path';
import { bootstrapCi } from '../study/common';

interface Px4Drone {
  ekfErrRmsM: number;
  ekfErrMaxM: number;
  trackErrRmsMps: number;
  wallStops: number;
  lockstepTimeouts: number;
  offboardDrops: number;
  finalMode: string;
}
interface Episode {
  arm: string;
  cond: string;
  seed: number;
  drones: number;
  coverage: number;
  t50: number | null;
  t80: number | null;
  baseKnown: number;
  collisions: number;
  crashes: number;
  droneContacts: number;
  overlap: number;
  steps: number;
  failed?: string;
  px4?: Px4Drone[];
}

const argv = process.argv.slice(2);
const arg = (name: string, def?: string): string => {
  const i = argv.indexOf(`--${name}`);
  if (i >= 0 && argv[i + 1]) return argv[i + 1]!;
  if (def === undefined) throw new Error(`missing --${name}`);
  return def;
};
const dir = resolve(arg('dir'));
const conds = arg('conds', 'hard,large').split(',');
const outPath = resolve(arg('out', 'docs/px4-team.json'));
const ARMS = ['px4', 'quad', 'kinematic'];
const ARM_LABEL: Record<string, string> = { px4: 'PX4 in the loop', quad: 'Quad model', kinematic: 'Kinematic' };
const COND_LABEL: Record<string, string> = { hard: 'Hard 20 m', large: 'Large 40 m' };
const SIZES = [1, 2, 3, 4];

const pct = (x: number) => `${(x * 100).toFixed(1)}%`;
const pts = (x: number) => `${x >= 0 ? '+' : '−'}${Math.abs(x * 100).toFixed(1)} pts`;
const ci = ([lo, hi]: [number, number]) => `${pts(lo)} to ${pts(hi)}`;
const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
function median(xs: (number | null)[]): number | null {
  const v = xs.map((x) => (x === null ? Infinity : x)).sort((a, b) => a - b);
  if (!v.length) return null;
  const m = v.length % 2 ? v[(v.length - 1) / 2]! : (v[v.length / 2 - 1]! + v[v.length / 2]!) / 2;
  return Number.isFinite(m) ? m : null;
}
const time = (x: number | null) => (x === null ? 'not reached' : `${Math.round(x)} s`);

const report: Record<string, unknown> = { dir, buildings: {}, rows: [], comparisons: [] };
const rows: Record<string, unknown>[] = [];
const comparisons: Record<string, unknown>[] = [];

for (const cond of conds) {
  const path = join(dir, cond, 'summary.json');
  if (!existsSync(path)) continue;
  const eps: Episode[] = JSON.parse(readFileSync(path, 'utf8')).episodes.filter((e: Episode) => e.cond === cond);
  const key = (arm: string, n: number, seed: number) => `${arm}:${n}:${seed}`;
  const byKey = new Map(eps.map((e) => [key(e.arm, e.drones, e.seed), e]));
  const seeds = [...new Set(eps.map((e) => e.seed))].sort((a, b) => a - b).filter((s) => ARMS.every((a) => SIZES.every((n) => byKey.has(key(a, n, s)))));
  (report.buildings as Record<string, number[]>)[cond] = seeds;
  const get = (arm: string, n: number) => seeds.map((s) => byKey.get(key(arm, n, s))!);

  console.log(`\n${COND_LABEL[cond] ?? cond}: ${seeds.length} buildings (${seeds[0]}–${seeds[seeds.length - 1]})\n`);
  console.log('| Airframe | Team | Searched (95% CI) | vs 1 drone | Median time to 50% | Median time to 80% | Reached 80% | Drone pairs within 0.35 m | Wall hits | Flights ended by a crash |');
  console.log('| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |');
  for (const arm of ARMS) {
    const solo = get(arm, 1);
    for (const n of SIZES) {
      const e = get(arm, n);
      const cov = e.map((x) => x.coverage);
      const [lo, hi] = bootstrapCi(cov);
      const diff = n > 1 ? e.map((x, i) => x.coverage - solo[i]!.coverage) : null;
      const row = {
        cond,
        arm,
        drones: n,
        buildings: e.length,
        coverage: mean(cov),
        coverageCi: [lo, hi],
        vsSolo: diff ? mean(diff) : null,
        vsSoloCi: diff ? bootstrapCi(diff) : null,
        t50: median(e.map((x) => x.t50)),
        t80: median(e.map((x) => x.t80)),
        reached80: e.filter((x) => x.t80 !== null).length,
        contactSteps: e.reduce((s, x) => s + x.droneContacts, 0),
        episodesWithContact: e.filter((x) => x.droneContacts > 0).length,
        collisions: e.reduce((s, x) => s + x.collisions, 0),
        crashes: e.reduce((s, x) => s + x.crashes, 0),
        flights: e.length * n,
        failed: e.filter((x) => x.failed).length,
        overlap: mean(e.map((x) => x.overlap)),
        baseKnown: mean(e.map((x) => x.baseKnown)),
        px4: e[0]?.px4
          ? {
              ekfRmsMean: mean(e.flatMap((x) => x.px4!.map((d) => d.ekfErrRmsM))),
              ekfRmsWorst: Math.max(...e.flatMap((x) => x.px4!.map((d) => d.ekfErrRmsM))),
              ekfMaxWorst: Math.max(...e.flatMap((x) => x.px4!.map((d) => d.ekfErrMaxM))),
              trackErrRmsMean: mean(e.flatMap((x) => x.px4!.map((d) => d.trackErrRmsMps))),
              offboardDrops: e.reduce((s, x) => s + x.px4!.reduce((a, d) => a + d.offboardDrops, 0), 0),
              wallStops: e.reduce((s, x) => s + x.px4!.reduce((a, d) => a + d.wallStops, 0), 0),
              notOffboardAtEnd: e.reduce((s, x) => s + x.px4!.filter((d) => d.finalMode !== 'offboard').length, 0),
              // Lockstep steps where PX4's motor outputs did not come back in time and the last ones were held.
              lockstepMissMean: mean(e.flatMap((x) => x.px4!.map((d) => d.lockstepTimeouts))),
              lockstepMissWorst: Math.max(...e.flatMap((x) => x.px4!.map((d) => d.lockstepTimeouts))),
            }
          : null,
      };
      rows.push(row);
      const contacts = row.contactSteps ? `${row.contactSteps} steps in ${row.episodesWithContact} of ${e.length}` : '0';
      console.log(
        `| ${ARM_LABEL[arm]} | ${n} drone${n > 1 ? 's' : ''} | ${pct(row.coverage)} (${pct(lo)}–${pct(hi)}) | ${diff ? pts(row.vsSolo!) : '—'} | ${time(row.t50)} | ${time(row.t80)} | ${row.reached80} of ${e.length} | ${contacts} | ${row.collisions} | ${row.crashes} of ${row.flights} |`,
      );
    }
  }

  // The airframe swap: same pilot, same spacing rule, same buildings; PX4 against the env's own model.
  console.log(`\nPX4 against the quad model, ${COND_LABEL[cond] ?? cond}, paired by building:\n`);
  for (const n of SIZES) {
    const a = get('px4', n);
    const b = get('quad', n);
    const d = a.map((x, i) => x.coverage - b[i]!.coverage);
    const c = { cond, drones: n, px4MinusQuad: mean(d), ci: bootstrapCi(d) };
    comparisons.push(c);
    console.log(`- ${n} drone${n > 1 ? 's' : ''}: ${pts(c.px4MinusQuad)} (95% CI ${ci(c.ci as [number, number])})`);
  }

  const px = rows.filter((r) => r.cond === cond && r.arm === 'px4') as { drones: number; px4: Record<string, number> | null; failed: number }[];
  console.log(`\nPX4 health, ${COND_LABEL[cond] ?? cond}:\n`);
  for (const r of px) {
    if (!r.px4) continue;
    console.log(
      `- ${r.drones} drone${r.drones > 1 ? 's' : ''}: EKF2 horizontal error RMS ${r.px4.ekfRmsMean!.toFixed(2)} m mean, ${r.px4.ekfRmsWorst!.toFixed(2)} m worst drone, ${r.px4.ekfMaxWorst!.toFixed(2)} m worst instant; velocity tracking error ${r.px4.trackErrRmsMean!.toFixed(2)} m/s RMS; offboard drops ${r.px4.offboardDrops}; wall stops ${r.px4.wallStops}; not in offboard at the end ${r.px4.notOffboardAtEnd}; lockstep steps without PX4's answer ${r.px4.lockstepMissMean!.toFixed(1)} per flight (worst ${r.px4.lockstepMissWorst}); failed episodes ${r.failed}`,
    );
  }
}

report.rows = rows;
report.comparisons = comparisons;
writeFileSync(outPath, `${JSON.stringify(report, null, 1)}\n`);
console.log(`\nwrote ${outPath}`);

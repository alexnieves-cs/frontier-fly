// The false-clear fix study. The failure atlas (docs/atlas) found the drone's one way to declare an
// occupied building clear: a hiding person's track closes with a mean confidence under the unresolved
// floor (0.70), so it is rejected as a lookalike and the building clears. Two fixes, and both together:
//
//   V0  shipped autopilot
//   V1  a thermal camera beside the visible one (people are warm, lookalikes are not)
//   V2  a lower unresolved floor (software only; lookalikes that score high now block a clear)
//   V3  both
//
// Each variant flies the same cases, paired:
//   --set fc     the 40 known false clears: 28 from the importance-sampled estimate, 12 from Monte Carlo
//   --set mc     the first --n cases of the Monte Carlo block, which prices what each fix costs
//                (incomplete outcomes on empty buildings, false alarms) under the operating distribution
//   --set fresh  --n new cases from the nominal distribution at --base, a block no other study used
//
//   vite-node scripts/study/falseclear-variants.ts -- --set fc --part 0 --of 4 --out out/fc-0.jsonl
//   vite-node scripts/study/falseclear-variants.ts -- --summary out/fc-*.jsonl
import { mkdirSync, readFileSync, writeFileSync, appendFileSync, existsSync } from 'node:fs';
import { dirname } from 'node:path';
import { flyCase, type AtlasFlight, type AtlasVariant } from '../../src/eval/atlas/flight';
import { nominalProposal, sampleCase, type AtlasCase } from '../../src/eval/atlas/space';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] && !process.argv[i + 1]!.startsWith('--') ? process.argv[i + 1]! : fallback;
}

export const FLOOR_V2 = Number(arg('floor', '0.62'));
export const VARIANTS: Record<string, AtlasVariant> = {
  V0: {},
  V1: { thermal: true },
  V2: { unresolvedFloor: FLOOR_V2 },
  V3: { thermal: true, unresolvedFloor: FLOOR_V2 },
};

interface Row extends AtlasFlight {
  variant: string;
  set: string;
  source?: string;
}

const results = (f: string): AtlasFlight[] => (JSON.parse(readFileSync(f, 'utf8')) as { results: AtlasFlight[] }).results;

function caseSet(set: string, n: number, base: number): { c: AtlasCase; source: string }[] {
  if (set === 'fc') {
    return [
      ...results('docs/atlas/is-falseClear.json').filter((r) => r.falseClear).map((r) => ({ c: r.case, source: 'is' })),
      ...results('docs/atlas/mc.json').filter((r) => r.falseClear).map((r) => ({ c: r.case, source: 'mc' })),
    ];
  }
  if (set === 'mc') return results('docs/atlas/mc.json').slice(0, n).map((r) => ({ c: r.case, source: 'mc' }));
  if (set === 'fresh') {
    const p = nominalProposal();
    return Array.from({ length: n }, (_, i) => ({ c: sampleCase(p, base, i), source: `fresh-${base}` }));
  }
  throw new Error(`unknown set ${set}`);
}

async function fly(): Promise<void> {
  const set = arg('set', 'fc');
  const part = Number(arg('part', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `out/falseclear/${set}-${part}.jsonl`);
  const variants = arg('variants', Object.keys(VARIANTS).join(',')).split(',');
  const cases = caseSet(set, Number(arg('n', '200')), Number(arg('base', '281001')));
  mkdirSync(dirname(out) || '.', { recursive: true });
  // Resume: skip what this shard already flew.
  const done = new Set(existsSync(out) ? readFileSync(out, 'utf8').split('\n').filter(Boolean).map((l) => {
    const r = JSON.parse(l) as Row;
    return `${r.variant}:${r.source}:${r.case.i}:${r.case.layoutSeed}`;
  }) : []);
  const jobs = cases.flatMap((x) => variants.map((v) => ({ ...x, v }))).filter((_, j) => j % of === part);
  const t0 = Date.now();
  let flown = 0;
  for (const job of jobs) {
    const key = `${job.v}:${job.source}:${job.c.i}:${job.c.layoutSeed}`;
    if (done.has(key)) continue;
    const f = await flyCase(job.c, undefined, VARIANTS[job.v]!);
    appendFileSync(out, JSON.stringify({ variant: job.v, set, source: job.source, ...f }) + '\n');
    flown++;
    if (flown % 5 === 0) console.log(`${set} ${part}/${of}: ${flown}/${jobs.length - done.size} (${Math.round((Date.now() - t0) / 1000)} s)`);
  }
}

/** Wilson 95% interval for k of n. */
function wilson(k: number, n: number): [number, number] {
  if (!n) return [0, 1];
  const z = 1.96, p = k / n, d = 1 + (z * z) / n;
  const c = (p + (z * z) / (2 * n)) / d, h = (z * Math.sqrt((p * (1 - p)) / n + (z * z) / (4 * n * n))) / d;
  return [Math.max(0, c - h), Math.min(1, c + h)];
}

function summary(files: string[]): void {
  const rows = files.flatMap((f) => readFileSync(f, 'utf8').split('\n').filter(Boolean).map((l) => JSON.parse(l) as Row));
  const sets = [...new Set(rows.map((r) => r.set))];
  const table: Record<string, Record<string, unknown>> = {};
  for (const set of sets) {
    for (const v of Object.keys(VARIANTS)) {
      const rs = rows.filter((r) => r.set === set && r.variant === v);
      if (!rs.length) continue;
      const n = rs.length;
      const count = (p: (r: Row) => boolean) => rs.filter(p).length;
      const fc = count((r) => r.falseClear);
      const occupied = rs.filter((r) => r.targetConfirmed !== null);
      const empty = rs.filter((r) => r.targetConfirmed === null);
      table[`${set}/${v}`] = {
        n,
        falseClear: fc,
        falseClearCI: wilson(fc, n).map((x) => +x.toFixed(4)),
        expected: +(count((r) => r.expected) / n).toFixed(3),
        collisions: count((r) => r.collision),
        falseAlarms: rs.reduce((s, r) => s + (r.falseAlarms ?? 0), 0),
        unresolved: rs.reduce((s, r) => s + (r.unresolved ?? 0), 0),
        occupiedConfirmed: `${occupied.filter((r) => r.targetConfirmed).length}/${occupied.length}`,
        emptyExpected: `${empty.filter((r) => r.expected).length}/${empty.length}`,
        meanDurationSec: Math.round(rs.reduce((s, r) => s + r.durationSec, 0) / n),
      };
    }
  }
  console.table(table);
  const out = arg('out', '');
  if (out) writeFileSync(out, JSON.stringify({ schema: 'frontier-fly/falseclear-variants@1', floorV2: FLOOR_V2, variants: VARIANTS, table }, null, 2) + '\n');
}

if (process.argv.includes('--summary')) {
  const i = process.argv.indexOf('--summary');
  summary(process.argv.slice(i + 1).filter((a) => !a.startsWith('--') && a.endsWith('.jsonl')));
} else {
  await fly();
}

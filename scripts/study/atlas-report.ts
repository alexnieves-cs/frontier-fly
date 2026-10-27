// Folds the failure atlas's three studies into the page's data (docs/atlas.json) and docs/ATLAS.md.
//
//   vite-node scripts/study/atlas-report.ts -- [--dir docs/atlas]
//
// Reads <dir>/search.json, <dir>/is-falseClear.json, <dir>/is-collision.json and <dir>/mc.json, the
// merged outputs of scripts/study/atlas-study.ts.
import { readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { ATLAS_EVENTS, type AtlasEvent, type AtlasFlight } from '../../src/eval/atlas/flight';
import type { RateEstimate, Slice } from '../../src/eval/atlas/estimate';
import { encodeCase } from '../../src/eval/atlas/replay';
import { decode, DIMS } from '../../src/eval/atlas/space';
import { wilson } from '../../src/eval/missions';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const dir = arg('dir', 'docs/atlas');
const out = arg('out', 'docs/atlas.json');
const md = arg('md', 'docs/ATLAS.md');
const read = <T>(f: string): T => JSON.parse(readFileSync(join(dir, f), 'utf8')) as T;

interface SearchFile {
  commit: string;
  mode: string;
  seedBase: number;
  settings: Record<string, number>;
  events: Record<AtlasEvent, { generations: { event: string; gen: number; flights: number; failures: Record<AtlasEvent, number>; level?: number }[]; shift: unknown }>;
  flights: (AtlasFlight & { event: string; gen: number })[];
}
interface RateFile {
  commit: string;
  seedBase: number;
  flights: number;
  rates: Record<string, RateEstimate>;
  slices?: Record<AtlasEvent, Slice[]>;
  results: AtlasFlight[];
}

const search = read<SearchFile>('search.json');
const mc = read<RateFile>('mc.json');
const is = Object.fromEntries(ATLAS_EVENTS.map((e) => [e, read<RateFile>(`is-${e}.json`)])) as Record<AtlasEvent, RateFile>;

/** One failing flight as the page lists it. */
function row(f: AtlasFlight, event: AtlasEvent, source: string) {
  const d = decode(f.case);
  return {
    event,
    source,
    code: encodeCase(f.case),
    seed: f.case.layoutSeed,
    mission: d.mission,
    buildings: d.buildings,
    personState: d.mission === 'find-person' || d.mission === 'sweep-occupied' ? d.personState : null,
    object: d.mission === 'find-object' ? d.object : null,
    decoys: d.decoys,
    lights: d.lights,
    hour: Math.round(d.timeOfDay * 10) / 10,
    windMps: Math.round(d.wind.meanMps * 10) / 10,
    fault: d.fault,
    outcome: f.outcome,
    targetConf: f.targetConf,
    where: event === 'collision' ? (f.firstCollision ? `${f.firstCollision.phase}${f.firstCollision.inside ? ' (inside)' : ''}` : null) : null,
    tag: event === 'collision' ? (f.clearance?.tag ?? null) : null,
  };
}

const failures = [
  ...ATLAS_EVENTS.flatMap((e) => mc.results.filter((f) => f[e]).map((f) => row(f, e, 'monte-carlo'))),
  ...ATLAS_EVENTS.flatMap((e) => is[e].results.filter((f) => f[e]).map((f) => row(f, e, 'importance'))),
  ...ATLAS_EVENTS.flatMap((e) => search.flights.filter((f) => f.event === e && f[e]).map((f) => row(f, e, 'search'))),
];

/** Rates on the slice of nominal flights a reader cares about most: missions with someone or something inside. */
function conditional(pred: (f: AtlasFlight) => boolean, e: AtlasEvent) {
  const mine = mc.results.filter(pred);
  const k = mine.filter((f) => f[e]).length;
  return { flights: mine.length, failures: k, p: mine.length ? k / mine.length : 0, ci: wilson(k, mine.length) };
}
const occupied = (f: AtlasFlight) => decode(f.case).mission !== 'sweep-empty';
const sweepOccupied = (f: AtlasFlight) => decode(f.case).mission === 'sweep-occupied';

const data = {
  schema: 1,
  commit: { search: search.commit, mc: mc.commit, is: Object.fromEntries(ATLAS_EVENTS.map((e) => [e, is[e].commit])) },
  mode: search.mode,
  settings: search.settings,
  seeds: { search: search.seedBase, mc: mc.seedBase, is: Object.fromEntries(ATLAS_EVENTS.map((e) => [e, is[e].seedBase])) },
  dims: DIMS.map((d) => ({ id: d.id, label: d.label, values: d.values, nominal: d.nominal })),
  rates: Object.fromEntries(ATLAS_EVENTS.map((e) => [e, { importance: is[e].rates[e], monteCarlo: mc.rates[e] }])),
  conditional: {
    falseClearWithTarget: conditional(occupied, 'falseClear'),
    falseClearSweepOccupied: conditional(sweepOccupied, 'falseClear'),
  },
  slices: mc.slices,
  search: Object.fromEntries(
    ATLAS_EVENTS.map((e) => [e, { generations: search.events[e].generations.map(({ gen, flights, failures, level }) => ({ gen, flights, failures: failures[e], level: level ?? null })), shift: search.events[e].shift }]),
  ),
  flights: { search: search.flights.length, mc: mc.flights, is: Object.fromEntries(ATLAS_EVENTS.map((e) => [e, is[e].flights])) },
  failures,
};
writeFileSync(out, `${JSON.stringify(data)}\n`);

// ------------------------------------------------------------ ATLAS.md

const pct = (x: number, d = 2) => `${(x * 100).toFixed(d)}%`;
const ci = (c: [number, number], d = 2) => `${pct(c[0], d)} to ${pct(c[1], d)}`;
const label = (e: AtlasEvent) => (e === 'falseClear' ? 'False clear' : 'Collision');
const lines: string[] = [
  '# Failure atlas',
  '',
  '<!-- Generated by scripts/study/atlas-report.ts from docs/atlas/*.json. Do not edit by hand. -->',
  '',
  `How often the shipped autopilot (\`${search.mode}\`: the trained policy indoors with the sensor-only planner as backup, on the quadrotor) declares a building clear while someone is inside it, and how often it touches something, over a declared mix of missions, sites, light, wind and faults. The search that finds the failures, every rate and the cases behind them are on the [atlas page](https://alexnieves-cs.github.io/frontier-fly/atlas.html), where each failing flight replays in the browser.`,
  '',
  '## Rates per flight',
  '',
  '| Event | Importance sampling | Flights | Plain Monte Carlo | Flights |',
  '| --- | --- | --- | --- | --- |',
  ...ATLAS_EVENTS.map((e) => {
    const r = data.rates[e]!;
    return `| ${label(e)} | ${pct(r.importance.p)} (95% CI ${ci(r.importance.ci)}) | ${r.importance.flights} (${r.importance.failures} failing, ESS ${Math.round(r.importance.ess)}) | ${pct(r.monteCarlo.p)} (${ci(r.monteCarlo.ci)}) | ${r.monteCarlo.flights} (${r.monteCarlo.failures} failing) |`;
  }),
  '',
  `On missions with someone or something to find (the rest are sweeps of empty sites, which cannot false-clear), the plain Monte Carlo false-clear rate is ${pct(data.conditional.falseClearWithTarget.p)} (${ci(data.conditional.falseClearWithTarget.ci)}, ${data.conditional.falseClearWithTarget.failures} of ${data.conditional.falseClearWithTarget.flights}); on sweeps of an occupied site it is ${pct(data.conditional.falseClearSweepOccupied.p)} (${ci(data.conditional.falseClearSweepOccupied.ci)}).`,
  '',
  '## Where the failures are',
  '',
  'Conditional rates from the plain Monte Carlo flights, the five values with the highest rate per event (Wilson 95% intervals):',
  '',
  ...ATLAS_EVENTS.flatMap((e) => {
    const top = [...(mc.slices?.[e] ?? [])].filter((s) => s.flights >= 30).sort((a, b) => b.rate - a.rate).slice(0, 5);
    return [`**${label(e)}**`, '', '| Dimension | Value | Rate | Flights |', '| --- | --- | --- | --- |', ...top.map((s) => `| ${DIMS.find((d) => d.id === s.dim)!.label} | ${s.value} | ${pct(s.rate, 1)} (${ci(s.ci, 1)}) | ${s.flights} |`), ''];
  }),
  '## Method',
  '',
  `- **Operating distribution.** Every flight is a draw from a declared distribution over ${DIMS.length} dimensions (src/eval/atlas/space.ts): mission kind (find a person 35%, sweep an occupied site 25%, sweep an empty site 20%, find an object 20%), 1-3 buildings, the person's pose, the object, decoys, interior lights, time of day (uniform; flashlight on at night), mean wind 6u² m/s with direction, gusts and arrival time, and a sensor or airframe fault on 40% of flights with its onset, severity and bearing. The layout and target placement come from two seeds that are never searched. The rates are rates under this distribution; the conditional rates let a reader re-weight it.`,
  `- **Search.** Cross-entropy method: each generation of ${search.events.falseClear.generations[0]?.flights ?? 0} flights keeps the top ${search.settings.rho! * 100}% by a score that reaches 1 exactly on the event (false clear: one minus the best detector confidence on the target; collision: how close the drone came to any solid but the ground), weights each kept flight by its likelihood ratio to the nominal, and refits every dimension's probabilities (smoothing ${search.settings.smoothing}, floor ${search.settings.floor}). ${search.flights.length} search flights on seeds from ${search.seedBase}.`,
  `- **Importance sampling.** Fresh flights from a mixture of ${search.settings.alpha! * 100}% nominal and the learned proposal, so no weight exceeds ${(1 / search.settings.alpha!).toFixed(1)}; the rate is the mean weighted indicator with a normal 95% interval. Plain Monte Carlo draws from the nominal itself on its own seeds, as a check.`,
  `- **Seeds.** Search ${search.seedBase}+, importance sampling ${ATLAS_EVENTS.map((e) => `${label(e).toLowerCase()} ${is[e].seedBase}+`).join(', ')}, Monte Carlo ${mc.seedBase}+. No other study uses these blocks, and nothing was tuned on them.`,
  '',
];
writeFileSync(md, lines.join('\n'));
console.log(`wrote ${out} (${failures.length} failing flights) and ${md}`);

// Folds the failure atlas's three studies into the page's data (docs/atlas.json) and docs/ATLAS.md.
//
//   vite-node scripts/study/atlas-report.ts -- [--dir docs/atlas]
//
// Reads <dir>/search.json (and an earlier <dir>/search-v1.json, for events the latest search did not
// cover), <dir>/is-falseClear.json, <dir>/is-collision.json and <dir>/mc.json, the merged outputs of
// scripts/study/atlas-study.ts.
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
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
  events: Partial<Record<AtlasEvent, { learned: unknown; generations: { event: string; gen: number; flights: number; failures: Record<AtlasEvent, number>; level?: number }[]; shift: unknown }>>;
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

const searches = ['search.json', 'search-v1.json'].filter((f) => existsSync(join(dir, f))).map((f) => read<SearchFile>(f));
const search = searches[0]!;
/** The latest search that learned a proposal for the event. */
const searchOf = (e: AtlasEvent): SearchFile => searches.find((s) => s.events[e]?.learned) ?? search;
const searchFlights = searches.flatMap((s) => s.flights.map((f) => ({ ...f, seedBase: s.seedBase })));
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
  ...ATLAS_EVENTS.flatMap((e) => searchFlights.filter((f) => f[e]).map((f) => row(f, e, 'search'))),
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
  seeds: { search: Object.fromEntries(ATLAS_EVENTS.map((e) => [e, searchOf(e).seedBase])), mc: mc.seedBase, is: Object.fromEntries(ATLAS_EVENTS.map((e) => [e, is[e].seedBase])) },
  dims: DIMS.map((d) => ({ id: d.id, label: d.label, values: d.values, nominal: d.nominal })),
  rates: Object.fromEntries(ATLAS_EVENTS.map((e) => [e, { importance: is[e].rates[e], monteCarlo: mc.rates[e] }])),
  conditional: {
    falseClearWithTarget: conditional(occupied, 'falseClear'),
    falseClearSweepOccupied: conditional(sweepOccupied, 'falseClear'),
  },
  slices: mc.slices,
  search: Object.fromEntries(
    ATLAS_EVENTS.map((e) => {
      const ev = searchOf(e).events[e]!;
      return [e, { generations: ev.generations.filter((g) => g.event !== 'nominal').map(({ gen, flights, failures, level }) => ({ gen, flights, failures: failures[e], level: level ?? null })), shift: ev.shift }];
    }),
  ),
  flights: { search: searchFlights.length, mc: mc.flights, is: Object.fromEntries(ATLAS_EVENTS.map((e) => [e, is[e].flights])) },
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
  `- **Search.** Cross-entropy method: each generation of ${searchOf('falseClear').events.falseClear?.generations[0]?.flights ?? 0} flights keeps the top ${search.settings.rho! * 100}% by a score that reaches 1 exactly on the event, weights each kept flight by its likelihood ratio to the nominal, and refits only the dimensions where the kept flights differ clearly from their batch (chi-square at 0.999; smoothing ${search.settings.smoothing}, floor ${search.settings.floor}). The collision score is how close the drone came to any solid but the ground. The false-clear score rewards a target seen but never confirmed, plus the share of buildings declared clear; a first search (seeds ${searches[searches.length - 1]!.seedBase}+) scored a target never seen at all highest and climbed toward flights that saw nothing, so it was re-run with that score on seeds ${searchOf('falseClear').seedBase}+. ${searchFlights.length} search flights in all.`,
  `- **Importance sampling.** Fresh flights from a mixture of ${search.settings.alpha! * 100}% nominal and the learned proposal, so no weight exceeds ${(1 / search.settings.alpha!).toFixed(1)}; the rate is the mean weighted indicator with a normal 95% interval. Plain Monte Carlo draws from the nominal itself on its own seeds, as a check.`,
  `- **Seeds.** Search ${[...new Set(searches.map((x) => x.seedBase))].join(' and ')}+, importance sampling ${ATLAS_EVENTS.map((e) => `${label(e).toLowerCase()} ${is[e].seedBase}+`).join(', ')}, Monte Carlo ${mc.seedBase}+. No other study uses these blocks, and nothing was tuned on them.`,
  '',
];
writeFileSync(md, lines.join('\n'));
console.log(`wrote ${out} (${failures.length} failing flights) and ${md}`);

// Failure atlas (src/eval/atlas): a cross-entropy search for the missions that end in a false clear
// or a collision, then the rate of each under the declared operating distribution, estimated twice:
// importance-sampled from the learned proposal, and by plain Monte Carlo as a check.
//
// Three studies, each a sharded run of this script (locally through parallel.mjs, or on GitHub's
// runners through .github/workflows/study.yml):
//
//   search    vite-node scripts/study/atlas-study.ts -- --search --base 200001 --out docs/atlas-search.json
//             Shard 0 leads: it draws each generation, flies it in --procs child processes, and
//             updates the proposal. The other shards exit at once.
//   estimate  ... -- --estimate falseClear --from docs/atlas-search.json --n 2000 --base 210001 (sharded)
//   mc        ... -- --mc --n 6000 --base 220001 (sharded)
//
// Each shard writes JSONL; --merge <files> --out <json> writes the result. Seeds: every block here is
// used by no other study; a re-run moves to a fresh block.
import { spawn } from 'node:child_process';
import { execSync } from 'node:child_process';
import { mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { ATLAS_EVENTS, ATLAS_MODE, flyCase, type AtlasEvent, type AtlasFlight } from '../../src/eval/atlas/flight';
import { ceUpdate, estimationProposal, importanceRate, monteCarloRate, proposalShift, slices, ALPHA, FLOOR, RHO, SMOOTHING } from '../../src/eval/atlas/estimate';
import { DIMS, nominalProposal, sampleCase, type AtlasCase, type Proposal } from '../../src/eval/atlas/space';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] && !process.argv[i + 1]!.startsWith('--') ? process.argv[i + 1]! : fallback;
}
const has = (name: string) => process.argv.includes(`--${name}`);
const commit = () => execSync('git rev-parse --short HEAD').toString().trim();

const readJsonl = <T>(files: string[]): T[] =>
  files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as T),
  );

/** Flies cases one after another, appending each result to `out` as it lands. */
async function flyAll(cases: AtlasCase[], out: string, label: string, tag: Record<string, unknown> = {}): Promise<void> {
  mkdirSync(dirname(out) || '.', { recursive: true });
  writeFileSync(out, '');
  const lines: string[] = [];
  const t0 = Date.now();
  for (let i = 0; i < cases.length; i++) {
    const f = await flyCase(cases[i]!);
    lines.push(JSON.stringify({ ...tag, ...f }));
    writeFileSync(out, lines.join('\n') + '\n');
    if ((i + 1) % 10 === 0 || i + 1 === cases.length) {
      console.log(`${label}: ${i + 1}/${cases.length} (${Math.round((Date.now() - t0) / 1000)} s)`);
    }
  }
}

/** Search generations fly in child processes, one per core. */
async function flyParallel(cases: AtlasCase[], dir: string, procs: number): Promise<AtlasFlight[]> {
  mkdirSync(dir, { recursive: true });
  const casesFile = join(dir, 'cases.json');
  writeFileSync(casesFile, JSON.stringify(cases));
  const outs = Array.from({ length: procs }, (_, j) => join(dir, `part-${j}.jsonl`));
  await Promise.all(
    outs.map(
      (out, j) =>
        new Promise<void>((resolve, reject) => {
          const child = spawn('npx', ['vite-node', 'scripts/study/atlas-study.ts', '--', '--cases', casesFile, '--part', String(j), '--of', String(procs), '--out', out], {
            stdio: ['ignore', 'ignore', 'inherit'],
          });
          child.on('exit', (code) => (code === 0 ? resolve() : reject(new Error(`part ${j} exited with ${code}`))));
        }),
    ),
  );
  const flights = readJsonl<AtlasFlight>(outs);
  return flights.sort((a, b) => a.case.i - b.case.i);
}

interface Generation {
  event: AtlasEvent | 'nominal';
  gen: number;
  flights: number;
  failures: Record<AtlasEvent, number>;
  level?: number;
  elite?: number;
  proposal: Proposal;
}

async function search(): Promise<void> {
  const shard = Number(arg('shard', '0'));
  const out = arg('out', 'atlas-search.jsonl');
  if (shard !== 0) {
    writeFileSync(out, '');
    return;
  }
  const base = Number(arg('base', '200001'));
  const batch = Number(arg('batch', '400'));
  const gens = Number(arg('gens', '6'));
  const procs = Number(arg('procs', '4'));
  const work = arg('work', 'atlas-work');
  const lines: string[] = [];
  const log = (record: object) => {
    lines.push(JSON.stringify(record));
    writeFileSync(out, lines.join('\n') + '\n');
  };
  const count = (fl: AtlasFlight[]) => Object.fromEntries(ATLAS_EVENTS.map((e) => [e, fl.filter((f) => f[e]).length])) as Record<AtlasEvent, number>;
  let next = 0;
  const draw = (proposal: Proposal) => Array.from({ length: batch }, () => sampleCase(proposal, base, next++));

  // Generation 0 draws from the nominal and seeds both searches.
  const nominal = nominalProposal();
  const first = await flyParallel(draw(nominal), join(work, 'g0'), procs);
  for (const f of first) log({ kind: 'flight', event: 'nominal', gen: 0, ...f });
  log({ kind: 'generation', event: 'nominal', gen: 0, flights: first.length, failures: count(first), proposal: nominal } satisfies Generation & { kind: string });

  for (const event of ATLAS_EVENTS) {
    let proposal = nominal;
    let flights = first;
    for (let g = 1; g <= gens; g++) {
      const step = ceUpdate(proposal, flights, event);
      log({ kind: 'generation', event, gen: g - 1, flights: flights.length, failures: count(flights), level: step.level, elite: step.elite, proposal } satisfies Generation & { kind: string });
      console.log(`${event} gen ${g - 1}: level ${step.level.toFixed(3)}, ${step.failures}/${flights.length} failures`);
      proposal = step.next;
      flights = await flyParallel(draw(proposal), join(work, `${event}-g${g}`), procs);
      for (const f of flights) log({ kind: 'flight', event, gen: g, ...f });
    }
    const last = ceUpdate(proposal, flights, event);
    log({ kind: 'generation', event, gen: gens, flights: flights.length, failures: count(flights), level: last.level, elite: last.elite, proposal } satisfies Generation & { kind: string });
    // The proposal the estimate uses is the update from the last generation.
    log({ kind: 'learned', event, proposal: last.next });
  }
}

function mergeSearch(files: string[], outPath: string): void {
  const records = readJsonl<Record<string, unknown>>(files);
  const generations = records.filter((r) => r.kind === 'generation') as unknown as Generation[];
  const learned = Object.fromEntries(records.filter((r) => r.kind === 'learned').map((r) => [r.event as string, r.proposal as Proposal]));
  const flights = records.filter((r) => r.kind === 'flight');
  const report = {
    schema: 1,
    commit: commit(),
    mode: ATLAS_MODE,
    seedBase: Number(arg('base', '200001')),
    settings: { rho: RHO, smoothing: SMOOTHING, floor: FLOOR, alpha: ALPHA },
    dims: DIMS,
    events: Object.fromEntries(
      ATLAS_EVENTS.map((e) => [
        e,
        {
          generations: generations.filter((g) => g.event === e || g.event === 'nominal').map((g) => ({ ...g, proposal: undefined })),
          learned: learned[e] ?? null,
          shift: learned[e] ? proposalShift(learned[e]!) : null,
        },
      ]),
    ),
    flights,
  };
  writeFileSync(outPath, `${JSON.stringify(report)}\n`);
  console.log(`wrote ${outPath}: ${flights.length} search flights`);
}

async function shardRun(): Promise<void> {
  const shard = Number(arg('shard', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `atlas-${shard}.jsonl`);
  const n = Number(arg('n', '1000'));
  const base = Number(arg('base', '220001'));
  const proposal = samplingProposal();
  const mine = Array.from({ length: n }, (_, i) => i)
    .filter((i) => i % of === shard)
    .map((i) => sampleCase(proposal, base, i));
  await flyAll(mine, out, `shard ${shard}`);
}

/** The estimate's sampling distribution: the nominal for --mc, the mixture over the learned proposal for --estimate. */
function samplingProposal(): Proposal {
  if (has('mc')) return nominalProposal();
  const event = arg('estimate', '') as AtlasEvent;
  const search = JSON.parse(readFileSync(arg('from', 'docs/atlas-search.json'), 'utf8')) as { events: Record<AtlasEvent, { learned: Proposal }> };
  return estimationProposal(search.events[event].learned);
}

function mergeRate(files: string[], outPath: string): void {
  const flights = readJsonl<AtlasFlight>(files).sort((a, b) => a.case.i - b.case.i);
  const mc = has('mc');
  const event = arg('estimate', '') as AtlasEvent;
  const proposal = samplingProposal();
  const rates = mc
    ? Object.fromEntries(ATLAS_EVENTS.map((e) => [e, monteCarloRate(flights, e)]))
    : { [event]: importanceRate(flights, event, proposal) };
  const report = {
    schema: 1,
    commit: commit(),
    mode: ATLAS_MODE,
    method: mc ? 'monte-carlo' : 'importance',
    event: mc ? null : event,
    seedBase: Number(arg('base', mc ? '220001' : '210001')),
    flights: flights.length,
    rates,
    ...(mc ? { slices: Object.fromEntries(ATLAS_EVENTS.map((e) => [e, slices(flights, e)])) } : { proposal }),
    results: flights,
  };
  writeFileSync(outPath, `${JSON.stringify(report)}\n`);
  console.log(`wrote ${outPath}: ${JSON.stringify(rates)}`);
}

const searching = has('search');
if (has('cases')) {
  // A search generation's child process.
  const cases = JSON.parse(readFileSync(arg('cases', ''), 'utf8')) as AtlasCase[];
  const part = Number(arg('part', '0'));
  const of = Number(arg('of', '1'));
  await flyAll(cases.filter((_, i) => i % of === part), arg('out', 'part.jsonl'), `part ${part}`);
} else if (has('merge')) {
  const rest = process.argv.slice(process.argv.indexOf('--merge') + 1);
  const files = rest.filter((a, i) => !a.startsWith('--') && !(i > 0 && rest[i - 1]!.startsWith('--')));
  const out = arg('out', 'docs/atlas.json');
  if (searching) mergeSearch(files, out);
  else mergeRate(files, out);
} else if (searching) {
  await search();
} else if (has('shard')) {
  await shardRun();
}

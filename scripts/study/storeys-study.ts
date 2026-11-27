// Multi-floor study, round 2: generated multi-floor buildings and real multi-storey homes, each flown
// under three budgets, by the planner and the published exploration methods.
//
//   vite-node scripts/study/storeys-study.ts -- --out docs/storeys-study.json
//   vite-node scripts/study/storeys-study.ts -- --pilots planner,nbvp --out /tmp/a.json     # a shard
//   vite-node scripts/study/storeys-study.ts -- --merge /tmp/a.json /tmp/b.json --out docs/storeys-study.json [--control main.json]
//   vite-node scripts/study/storeys-study.ts -- --split dev ...                             # dev seeds (tune freely)
//
// Budgets:
//   bench      120 s, whatever the number of floors (FrontierBench's `floors` task).
//   per-floor  120 s per floor.
//   battery    the airframe's own pack (shared/airframe.ts, mission/autonomy/battery.ts): the flight
//              explores until 70% of the usable energy is spent (30% held for the way home and the
//              landing reserve), or until the pilot or the env ends it (95% flown coverage, 25 s with
//              nothing new, a crash), with a 20 min cap.
// Coverage is the env's flown coverage (0.5 m voxels flown through); footprint is the share of the
// reachable building that came within 3 m of the drone by ground-truth rays (src/eval/footprint.ts),
// the way exploration papers count it. Flown coverage is capped by speed: at 2 m/s a flight enters at
// most four voxels a second, which `ceiling` gives per flight.
//
// Seeds (roadmap/next-wave-tracks.md): test = generated block 88101-88130 and the 60 eval homes with
// env seeds 88201+, plus the 12 held-out homes (public/plans/swiss-homes-holdout.json, env seeds
// 88801+), report only; dev = FrontierBench dev 81401-81420 and the 20 dev homes (88601+).
import { execSync } from 'node:child_process';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { BASELINES } from '../../src/bench/baselines';
import { benchEpisodeSetup } from '../../src/bench/run';
import { BENCH_TASKS } from '../../src/bench/tasks';
import { EpisodeFootprint, FOOTPRINT_RADIUS } from '../../src/eval/footprint';
import { DEFAULT_BATTERY } from '../../src/mission/autonomy/battery';
import { HOME_HOLDOUT_PATH, HOME_HOLDOUT_SEED_BASE, HOME_SET_PATH, homeBuilding, homeEpisodeSetup, parseHomeSet } from '../../src/plans/homes';
import { VOXEL } from '../../src/shared/constants';
import { createEnv } from '../../src/sim/env';
import { sensorFrame } from '../../src/sim/sensorPlanner';
import { bootstrapCi } from './common';

export const STOREYS_TEST = { generated: { base: 88101, count: 30 } } as const;
export const STOREYS_DEV = { generated: { base: 81401, count: 20 } } as const;
export const BUDGETS = [
  { id: 'bench', label: '120 s (FrontierBench)' },
  { id: 'per-floor', label: '120 s per floor' },
  { id: 'battery', label: 'Battery (explore until 70% of the usable pack)' },
] as const;
type BudgetId = (typeof BUDGETS)[number]['id'];
export const PILOTS = ['planner', 'planner-view', 'boustrophedon', 'nbvp', 'yamauchi'] as const;
/** Share of the usable pack the battery budget may spend exploring; the rest is for the way home and the reserve. */
export const BATTERY_EXPLORE_SHARE = 0.7;
const BATTERY_CAP_STEPS = 20 * 60 * 20;
const USABLE_WH = DEFAULT_BATTERY.capacityWh * DEFAULT_BATTERY.usableFraction;

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

interface Flight {
  set: 'generated' | 'homes' | 'holdout';
  id: string;
  seed: number;
  floors: number;
  reachableVoxels: number;
  steps: number;
  coverage: number;
  footprint: number;
  floorsReached: number;
  collisions: number;
  /** Energy spent, % of the usable pack. */
  batteryPct: number;
  /** Flown-coverage bound at 2 m/s for the flight's time. */
  ceiling: number;
  /** Seconds to reach 50% / 90% footprint and 50% / 90% flown coverage, null if never. */
  t50Footprint: number | null;
  t90Footprint: number | null;
  t50Coverage: number | null;
  t90Coverage: number | null;
}

const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);

function summarize(flights: Flight[]) {
  const times = (k: keyof Flight) => flights.map((f) => f[k] as number | null).filter((v): v is number => v !== null);
  return {
    flights: flights.length,
    coverage: mean(flights.map((f) => f.coverage)),
    coverageCi: bootstrapCi(flights.map((f) => f.coverage)),
    footprint: mean(flights.map((f) => f.footprint)),
    footprintCi: bootstrapCi(flights.map((f) => f.footprint)),
    ceiling: mean(flights.map((f) => f.ceiling)),
    seconds: mean(flights.map((f) => f.steps / 20)),
    batteryPct: mean(flights.map((f) => f.batteryPct)),
    allFloorsReached: flights.filter((f) => f.floorsReached === f.floors).length,
    flightsWithCollision: flights.filter((f) => f.collisions > 0).length,
    reached90Footprint: times('t90Footprint').length,
    t90Footprint: mean(times('t90Footprint')),
    reached90Coverage: times('t90Coverage').length,
    t90Coverage: mean(times('t90Coverage')),
  };
}

async function fly(split: 'dev' | 'test', pilots: string[]): Promise<{ pilot: string; budget: BudgetId; set: Flight['set']; flights: Flight[] }[]> {
  const task = BENCH_TASKS.find((t) => t.id === 'floors')!;
  const homeSet = parseHomeSet(JSON.parse(readFileSync(HOME_SET_PATH, 'utf8')));
  const homes = homeSet.homes.filter((h) => h.split === (split === 'dev' ? 'dev' : 'eval'));
  const holdoutSet = split === 'test' && existsSync(HOME_HOLDOUT_PATH) ? parseHomeSet(JSON.parse(readFileSync(HOME_HOLDOUT_PATH, 'utf8'))) : null;
  const wanted = new Set(arg('sets', 'generated,homes,holdout').split(','));
  const gen = split === 'dev' ? STOREYS_DEV.generated : STOREYS_TEST.generated;
  const episodes = [
    ...Array.from({ length: gen.count }, (_, i) => ({ set: 'generated' as const, id: `gen-${gen.base + i}`, setup: () => benchEpisodeSetup(task, { seed: gen.base + i }) })),
    ...homes.map((h) => ({ set: 'homes' as const, id: h.id, setup: () => homeEpisodeSetup(homeSet, h, 'day') })),
    ...(holdoutSet?.homes ?? []).map((h) => ({ set: 'holdout' as const, id: h.id, setup: () => homeEpisodeSetup(holdoutSet!, h, 'day', 0, HOME_HOLDOUT_SEED_BASE) })),
  ].filter((ep) => wanted.has(ep.set));
  for (const h of holdoutSet?.homes ?? []) if (!homeBuilding(h)) throw new Error(`home ${h.id} does not convert`);
  for (const h of homes) if (!homeBuilding(h)) throw new Error(`home ${h.id} does not convert`);
  const env = await createEnv();
  const out = [];
  for (const pilotId of pilots) {
    for (const budget of BUDGETS) {
      const bySet: Record<Flight['set'], Flight[]> = { generated: [], homes: [], holdout: [] };
      for (const ep of episodes) {
        const { spec, envSeed, cfg } = ep.setup();
        const floors = spec.floors.length;
        if (budget.id === 'per-floor') cfg.maxSteps *= floors;
        if (budget.id === 'battery') cfg.maxSteps = BATTERY_CAP_STEPS;
        const pilot = BASELINES[pilotId]!();
        pilot.reset({ speedH: task.speedH, maxSteps: cfg.maxSteps, dynamics: task.dynamics });
        let obs = env.reset(spec, envSeed, cfg);
        const fp = new EpisodeFootprint();
        fp.start(env, spec, cfg.maxSteps);
        const head = fp.fp.radii.indexOf(FOOTPRINT_RADIUS);
        const first: Record<string, number | null> = { f50: null, f90: null, c50: null, c90: null };
        let info: { coverage: number; collisions: number; energyWh: number };
        let steps = 0;
        for (;;) {
          const s = env.step(pilot.act(obs, sensorFrame(env, obs)));
          obs = s.obs;
          info = s.info;
          steps += 1;
          fp.step(env, steps, info.coverage);
          const seen = fp.fp.coverage(head);
          if (first.f50 === null && seen >= 0.5) first.f50 = steps / 20;
          if (first.f90 === null && seen >= 0.9) first.f90 = steps / 20;
          if (first.c50 === null && info.coverage >= 0.5) first.c50 = steps / 20;
          if (first.c90 === null && info.coverage >= 0.9) first.c90 = steps / 20;
          if (s.done || steps >= cfg.maxSteps) break;
          if (budget.id === 'battery' && info.energyWh >= BATTERY_EXPLORE_SHARE * USABLE_WH) break;
        }
        const visited = env.visitedGrid;
        const per = visited.length / floors;
        let reached = 0;
        for (let f = 0; f < floors; f++) {
          let n = 0;
          for (let i = 0; i < per; i++) n += visited[f * per + i]!;
          // A floor counts as reached once the drone has flown a few voxels of it, not just poked into its shaft.
          if (n >= 8) reached += 1;
        }
        bySet[ep.set].push({
          set: ep.set,
          id: ep.id,
          seed: envSeed,
          floors,
          reachableVoxels: spec.reachableVoxels,
          steps,
          coverage: round4(info.coverage),
          footprint: fp.finish(info.coverage).footprint[String(FOOTPRINT_RADIUS)]!,
          floorsReached: reached,
          collisions: info.collisions,
          batteryPct: round4((info.energyWh / USABLE_WH) * 100),
          ceiling: round4(Math.min(1, (steps * task.speedH) / 20 / VOXEL / spec.reachableVoxels)),
          t50Footprint: first.f50,
          t90Footprint: first.f90,
          t50Coverage: first.c50,
          t90Coverage: first.c90,
        });
      }
      for (const set of ['generated', 'homes', 'holdout'] as const) {
        if (bySet[set].length === 0) continue;
        out.push({ pilot: pilotId, budget: budget.id, set, flights: bySet[set] });
        const s = summarize(bySet[set]);
        console.log(
          `${pilotId} ${budget.id} ${set}: flown ${(s.coverage * 100).toFixed(1)}%, footprint ${(s.footprint * 100).toFixed(1)}%, ${s.seconds.toFixed(0)} s, battery ${s.batteryPct.toFixed(1)}%, all floors ${s.allFloorsReached}/${s.flights}, contact ${s.flightsWithCollision}`,
        );
      }
    }
  }
  env.dispose();
  return out;
}

const round4 = (v: number) => Math.round(v * 1e4) / 1e4;

type Raw = { pilot: string; budget: BudgetId; set: Flight['set']; flights: Flight[] }[];

function cellsOf(raw: Raw) {
  return raw.map((r) => ({ pilot: r.pilot, budget: r.budget, set: r.set, ...summarize(r.flights), episodes: r.flights }));
}

const out = arg('out', 'docs/storeys-study.json');
const split = arg('split', 'test') as 'dev' | 'test';
let raw: Raw;
const mergeAt = process.argv.indexOf('--merge');
if (mergeAt >= 0) {
  const files = process.argv.slice(mergeAt + 1).filter((a) => a.endsWith('.json') && a !== out && a !== arg('control', '') && a !== arg('first', ''));
  raw = files.flatMap((f) => (JSON.parse(readFileSync(f, 'utf8')) as { raw: Raw }).raw);
} else {
  raw = await fly(split, arg('pilots', PILOTS.join(',')).split(','));
}
const controlPath = arg('control', '');
// --first: an earlier report run of the same study on the same seeds (its summaries are kept, not its flights).
const firstPath = arg('first', '');
const first = firstPath ? (JSON.parse(readFileSync(firstPath, 'utf8')) as { commit: string; cells: { pilot: string; budget: string; set: string }[] }) : null;
const control = controlPath ? (JSON.parse(readFileSync(controlPath, 'utf8')) as { commit: string; cells: { pilot: string; budget: string; set: string }[] }) : null;
const homeSet = parseHomeSet(JSON.parse(readFileSync(HOME_SET_PATH, 'utf8')));
const result = {
  schema: 1,
  commit: execSync('git rev-parse --short HEAD').toString().trim(),
  split,
  seeds:
    split === 'test'
      ? { generated: STOREYS_TEST.generated, homes: { envBase: 88201, count: homeSet.homes.filter((h) => h.split === 'eval').length }, holdout: { envBase: HOME_HOLDOUT_SEED_BASE, file: HOME_HOLDOUT_PATH } }
      : { generated: STOREYS_DEV.generated, homes: { envBase: 88601 } },
  budgets: BUDGETS,
  batteryExploreShare: BATTERY_EXPLORE_SHARE,
  homes: { dataset: homeSet.dataset.name, version: homeSet.dataset.version, license: homeSet.dataset.license, file: HOME_SET_PATH },
  cells: cellsOf(raw),
  // Shards keep the raw flights for --merge; the merged file has them under cells[].episodes.
  ...(mergeAt < 0 ? { raw } : {}),
  ...(control ? { control: { commit: control.commit, cells: control.cells.map((c) => ({ ...c, episodes: undefined })) } } : {}),
  ...(first ? { firstRun: { commit: first.commit, cells: first.cells.map((c) => ({ ...c, episodes: undefined })) } } : {}),
};
writeFileSync(out, `${JSON.stringify(result, null, 1)}\n`);

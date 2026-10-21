// Outdoor search benchmark: a lost person placed by a behavior simulator (or uniformly at random),
// searched for by the lane sweep and by the probability-of-area search on the same worlds.
// scripts/study/sar-study.ts runs it in shards and writes docs/sar-bench.json.
import type { OutdoorSearchOptions, MissionWorld, RunRecord } from '../mission/autonomy/contracts';
import { createPerceptionSource } from '../mission/autonomy/perception';
import { runMission } from '../mission/autonomy/runner';
import { plantPersonAt, simulateLostPerson, type Strategy } from '../mission/autonomy/sar/lostPerson';
import { sweepWidth } from '../mission/autonomy/sar/pod';
import { cruiseAglFor } from '../mission/autonomy/planner/coverage';
import { DETECT_PERIOD } from '../mission/autonomy/phases/shared';
import { OUTDOOR_LIMITS } from '../mission/autonomy/vehicle';
import { buildMissionWorld, missionForWorld, plantingObstacles } from '../mission/autonomy/world';
import type { Mission } from '../mission/types';
import { RNG } from '../shared/rng';
import { containsPoint } from '../world/outdoor/geofence';
import { wilson } from './missions';

/**
 * Seeds. 7100-7139 are development seeds: the planner, prior and POD model were tuned on them and on
 * nothing else. 7600 onward are the evaluation seeds, flown once with settings frozen beforehand.
 */
export const SAR_DEV_SEED_BASE = 7100;
export const SAR_DEV_SEEDS = 40;
export const SAR_SEED_BASE = 7600;
export const SAR_SEEDS = 100;
export const SAR_TIME_BUDGET_SEC = 900;

/** Where the truth comes from, and the light. */
export const SAR_CONDITIONS = ['day', 'uniform', 'night'] as const;
export type SarCondition = (typeof SAR_CONDITIONS)[number];

/** Seeds per condition: the main condition gets the most. */
export const SAR_CONDITION_SEEDS: Record<SarCondition, number> = { day: 100, uniform: 60, night: 60 };

export const SAR_METHODS = ['lanes', 'lanes-sw', 'poa', 'poa-greedy', 'poa-ring', 'poa-flat'] as const;
export type SarMethod = (typeof SAR_METHODS)[number];

export const SAR_METHOD_LABEL: Record<SarMethod, string> = {
  lanes: 'Lane sweep (shipped)',
  'lanes-sw': 'Lane sweep, sweep-width spacing, buildings last',
  poa: 'Probability search',
  'poa-greedy': 'Probability search, one-leg greedy',
  'poa-ring': 'Probability search, distance ring only',
  'poa-flat': 'Probability search, flat prior',
};

export interface SarScene {
  seed: number;
  condition: SarCondition;
  world: MissionWorld;
  mission: Mission;
  /** The same world with nobody in it, for blind calibration flights. */
  empty: { world: MissionWorld; mission: Mission };
  /** Last known point: what the searcher is told. */
  ipp: [number, number];
  /** Where the person is: outdoors at xy, or inside a building. */
  truth: { strategy: Strategy | 'uniform'; xy: [number, number]; building: string | null; state: string; dIpp: number };
}

const TIME: Record<SarCondition, { timeOfDay: number; sensorMode: 'rgb' | 'ir'; flashlightOn: boolean; interiorLights: 'on' | 'off' }> = {
  day: { timeOfDay: 13, sensorMode: 'rgb', flashlightOn: false, interiorLights: 'on' },
  uniform: { timeOfDay: 13, sensorMode: 'rgb', flashlightOn: false, interiorLights: 'on' },
  // Searches after dark fly a thermal camera; the RGB detector sees almost nothing outdoors at night.
  night: { timeOfDay: 23, sensorMode: 'ir', flashlightOn: true, interiorLights: 'off' },
};

function insideFootprint(b: MissionWorld['buildings'][number], x: number, y: number): boolean {
  const xs = b.footprint.map((p) => p.x);
  const ys = b.footprint.map((p) => p.y);
  return x >= Math.min(...xs) && x < Math.max(...xs) && y >= Math.min(...ys) && y < Math.max(...ys);
}

function freeSpot(world: MissionWorld, rng: RNG, pick: () => [number, number]): [number, number] {
  const blocked = plantingObstacles(world.outdoor);
  for (let i = 0; i < 400; i++) {
    const [x, y] = pick();
    if (!containsPoint(world.outdoor.geofence, x, y)) continue;
    if (blocked.some((b) => x >= b.minX - 1 && x < b.maxX + 1 && y >= b.minY - 1 && y < b.maxY + 1)) continue;
    return [x, y];
  }
  return [world.home[0] + rng.range(-3, 3), world.home[1] + rng.range(-3, 3)];
}

/** The seeded world, last known point and person for one benchmark flight. Same seed, same scene. */
export function sarScene(seed: number, condition: SarCondition): SarScene {
  const light = TIME[condition];
  const base = buildMissionWorld({ seed, ...light, targets: [] });
  const rng = new RNG(seed, 'sar-scene');
  const home = base.home;
  // Last seen 8-20 m from the command post, on open ground.
  const ipp = freeSpot(base, rng, () => {
    const a = rng.range(0, Math.PI * 2);
    const r = rng.range(8, 20);
    return [home[0] + Math.cos(a) * r, home[1] + Math.sin(a) * r];
  });
  let strategy: Strategy | 'uniform';
  let xy: [number, number];
  let building: number | null = null;
  let state: 'lying' | 'sitting' | 'standing';
  if (condition === 'uniform') {
    const fence = base.outdoor.geofence.polygon;
    const xs = fence.map(([x]) => x);
    const ys = fence.map(([, y]) => y);
    // Uniform over open ground and building floor area alike, matching the uniform prior.
    const pick = (): [number, number] => [rng.range(Math.min(...xs), Math.max(...xs)), rng.range(Math.min(...ys), Math.max(...ys))];
    const blocked = plantingObstacles(base.outdoor);
    xy = freeSpot(base, rng, pick);
    for (let i = 0; i < 400; i++) {
      const [x, y] = pick();
      if (!containsPoint(base.outdoor.geofence, x, y)) continue;
      const inside = base.buildings.findIndex((b) => insideFootprint(b, x, y));
      if (inside >= 0) {
        building = inside;
        xy = [x, y];
        break;
      }
      if (blocked.some((b) => x >= b.minX - 1 && x < b.maxX + 1 && y >= b.minY - 1 && y < b.maxY + 1)) continue;
      xy = [x, y];
      break;
    }
    strategy = 'uniform';
    state = rng.pick(['lying', 'sitting', 'standing'] as const);
  } else {
    const lost = simulateLostPerson(base, ipp, seed);
    strategy = lost.strategy;
    xy = lost.xy;
    building = lost.building;
    state = lost.state === 'lying' || lost.state === 'sitting' ? lost.state : 'standing';
  }
  let world: MissionWorld;
  if (building !== null) {
    world = buildMissionWorld({ seed, ...light, targets: [{ classId: 'person', state: 'lying', where: 'indoor', building }] });
  } else {
    const person = plantPersonAt(base, xy, state, seed);
    if (!person) throw new Error(`sar scene ${seed}: no spot for the person near (${xy[0]}, ${xy[1]})`);
    xy = [person.feet[0], person.feet[1]];
    world = { ...base, outdoorScene: { ...base.outdoorScene, targets: [person] } };
  }
  const brief = {
    missionType: 'find_person' as const,
    name: 'Overdue hiker',
    targets: [{ id: 'tgt-hiker', kind: 'person' as const, label: 'Overdue hiker (simulated)', minCount: 1, minConfidence: null }],
    requiredConfidence: 0.8,
    entryPolicy: 'auto' as const,
    budgets: { batteryPct: 80, timeSec: SAR_TIME_BUDGET_SEC },
  };
  const mission = missionForWorld(world, brief);
  const buildingId = building === null ? null : world.buildings[building]!.id;
  const empty = { world: base, mission: missionForWorld(base, brief) };
  return { seed, condition, world, mission, empty, ipp, truth: { strategy, xy, building: buildingId, state, dIpp: Math.hypot(xy[0] - ipp[0], xy[1] - ipp[1]) } };
}

/** Lane spacing equal to the camera's effective sweep width at cruise, for a coverage factor of one. */
export function sweepWidthSpacing(scene: SarScene): number {
  const agl = cruiseAglFor(scene.mission);
  const sensor = { timeOfDay: scene.world.timeOfDay, sensorMode: scene.world.sensorMode, flashlightOn: scene.world.flashlightOn };
  return sweepWidth(agl - 0.5, OUTDOOR_LIMITS.speedH, DETECT_PERIOD, sensor);
}

export function searchOptions(method: SarMethod, scene: SarScene): OutdoorSearchOptions {
  switch (method) {
    case 'lanes':
      return { mode: 'lanes' };
    case 'lanes-sw':
      return { mode: 'lanes', laneSpacingM: sweepWidthSpacing(scene), buildingsLast: true };
    case 'poa':
      return { mode: 'probability', lastKnown: scene.ipp };
    case 'poa-greedy':
      return { mode: 'probability', lastKnown: scene.ipp, planner: { depth: 1, beam: 1, terminal: false, commit: 1 } };
    case 'poa-ring':
      return { mode: 'probability', lastKnown: scene.ipp, prior: 'ring' };
    case 'poa-flat':
      return { mode: 'probability', lastKnown: scene.ipp, prior: 'uniform' };
  }
}

export interface SarFlight {
  seed: number;
  condition: SarCondition;
  method: SarMethod;
  /** Flown with nobody in the world: only the model's POS curve is used, for calibration. */
  blind?: boolean;
  strategy: string;
  where: 'outdoor' | 'building';
  dIpp: number;
  found: boolean;
  /** Seconds to the confirmation of the person. Null when not found. */
  tFind: number | null;
  outcome: string;
  durationSec: number;
  batteryUsedPct: number;
  pathM: number;
  buildingsEntered: number;
  falseAlarms: number;
  collisions: number;
  landedHome: boolean;
  /** The model's cumulative probability of success every 5 s (probability methods only). */
  pos: [number, number][] | null;
  /** Share of the prior within aerial reach (probability methods only). */
  reachable: number | null;
}

function pathLength(run: RunRecord): number {
  let m = 0;
  for (let i = 1; i < run.track.length; i++) {
    const a = run.track[i - 1]!.pos;
    const b = run.track[i]!.pos;
    m += Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
  }
  return m;
}

export async function flySar(seed: number, condition: SarCondition, method: SarMethod, blind = false): Promise<SarFlight> {
  const scene = sarScene(seed, condition);
  const { world, mission } = blind ? scene.empty : scene;
  const source = createPerceptionSource(world);
  const run = await runMission(mission, world, source, {
    dt: 0.05,
    indoor: 'frontier',
    operator: () => 'approve',
    search: searchOptions(method, scene),
  });
  const hit = run.confirmed.find((c) => c.classId === 'person' && c.truth?.kind === 'tp');
  return {
    seed,
    condition,
    method,
    ...(blind ? { blind } : {}),
    strategy: scene.truth.strategy,
    where: scene.truth.building ? 'building' : 'outdoor',
    dIpp: Math.round(scene.truth.dIpp * 10) / 10,
    found: !!hit,
    tFind: hit ? Math.round(hit.tSec * 10) / 10 : null,
    outcome: run.outcome,
    durationSec: Math.round(run.tSec),
    batteryUsedPct: Math.round((run.batteryStartPct - run.batteryEndPct) * 10) / 10,
    pathM: Math.round(pathLength(run)),
    buildingsEntered: run.entries.filter((e) => e.decision !== 'denied').length,
    falseAlarms: run.confirmed.filter((c) => c.truth?.kind !== 'tp').length,
    collisions: run.collisions.length,
    landedHome: run.landedHome,
    pos: run.search ? run.search.pos.map((s) => [s.tSec, s.pos] as [number, number]) : null,
    reachable: run.search ? Math.round(run.search.reachable * 1e4) / 1e4 : null,
  };
}

export interface SarJob {
  seed: number;
  condition: SarCondition;
  method: SarMethod;
  blind?: boolean;
}

/**
 * Every (seed, condition, method) flight, plus one blind probability-search flight per scene: the
 * same search with nobody to find, flown to the end of the budget, so the model's POS curve is
 * known at every time and not only up to a find.
 */
export function sarJobs(dev = false, scale = 1): SarJob[] {
  const out: SarJob[] = [];
  for (const condition of SAR_CONDITIONS) {
    const n = dev ? (condition === 'day' ? SAR_DEV_SEEDS : Math.round(SAR_DEV_SEEDS / 2)) : Math.round(SAR_CONDITION_SEEDS[condition] * scale);
    const base = dev ? SAR_DEV_SEED_BASE : SAR_SEED_BASE;
    for (let i = 0; i < n; i++) {
      for (const method of SAR_METHODS) out.push({ seed: base + i, condition, method });
      out.push({ seed: base + i, condition, method: 'poa', blind: true });
    }
  }
  return out;
}

// ------------------------------------------------------------ statistics

/** Fraction of flights that found the person by each time in `grid` (seconds). Not-found flights are censored at the budget. */
export function findCurve(flights: readonly SarFlight[], grid: readonly number[]): number[] {
  return grid.map((t) => (flights.length ? flights.filter((f) => f.tFind !== null && f.tFind <= t).length / flights.length : 0));
}

/** Restricted mean time to find: mean of min(tFind, budget), counting misses as the full budget. */
export function restrictedMean(flights: readonly SarFlight[], budget = SAR_TIME_BUDGET_SEC): number {
  if (!flights.length) return 0;
  return flights.reduce((s, f) => s + Math.min(f.tFind ?? budget, budget), 0) / flights.length;
}

function quantile(xs: number[], q: number): number {
  if (!xs.length) return NaN;
  const s = [...xs].sort((a, b) => a - b);
  const i = (s.length - 1) * q;
  const lo = Math.floor(i);
  const hi = Math.ceil(i);
  return s[lo]! + (s[hi]! - s[lo]!) * (i - lo);
}

/** Median time to find with misses at +infinity (null when half or more were missed). */
export function medianFind(flights: readonly SarFlight[]): number | null {
  const ts = flights.map((f) => f.tFind ?? Infinity).sort((a, b) => a - b);
  if (!ts.length) return null;
  const m = quantile(ts.filter(Number.isFinite).concat(ts.filter((t) => !Number.isFinite(t)).map(() => 1e9)), 0.5);
  return m >= 1e9 ? null : m;
}

/**
 * Paired bootstrap over seeds of the difference in restricted mean time to find (b minus a) and in
 * found fraction. Both methods flew the same scenes, so pairing removes the scene-to-scene spread.
 */
export function pairedBootstrap(a: readonly SarFlight[], b: readonly SarFlight[], reps = 4000, seed = 1): { dMean: [number, number, number]; dFound: [number, number, number] } {
  const bySeed = new Map<number, SarFlight>();
  for (const f of b) bySeed.set(f.seed, f);
  const pairs = a.filter((f) => bySeed.has(f.seed)).map((f) => [f, bySeed.get(f.seed)!] as const);
  const rng = new RNG(seed, 'bootstrap');
  const t = (f: SarFlight) => Math.min(f.tFind ?? SAR_TIME_BUDGET_SEC, SAR_TIME_BUDGET_SEC);
  const stat = (idx: number[]) => {
    let dm = 0;
    let df = 0;
    for (const i of idx) {
      const [fa, fb] = pairs[i]!;
      dm += t(fb) - t(fa);
      df += Number(fb.found) - Number(fa.found);
    }
    return [dm / idx.length, df / idx.length] as const;
  };
  const all = pairs.map((_, i) => i);
  const [m0, f0] = stat(all);
  const ms: number[] = [];
  const fs: number[] = [];
  for (let r = 0; r < reps; r++) {
    const idx = all.map(() => rng.int(all.length));
    const [m, f] = stat(idx);
    ms.push(m);
    fs.push(f);
  }
  return { dMean: [m0, quantile(ms, 0.025), quantile(ms, 0.975)], dFound: [f0, quantile(fs, 0.025), quantile(fs, 0.975)] };
}

export function summarizeSar(flights: readonly SarFlight[]) {
  const n = flights.length;
  const k = flights.filter((f) => f.found).length;
  const mean = (xs: number[]) => (xs.length ? xs.reduce((s, x) => s + x, 0) / xs.length : 0);
  return {
    flights: n,
    found: n ? k / n : 0,
    foundCi: wilson(k, n),
    medianFindSec: medianFind(flights),
    restrictedMeanSec: restrictedMean(flights),
    within300: n ? flights.filter((f) => f.tFind !== null && f.tFind <= 300).length / n : 0,
    meanBatteryUsedPct: mean(flights.map((f) => f.batteryUsedPct)),
    meanPathM: mean(flights.map((f) => f.pathM)),
    falseAlarms: flights.reduce((s, f) => s + f.falseAlarms, 0),
    collisions: flights.reduce((s, f) => s + f.collisions, 0),
    landedHome: n ? flights.filter((f) => f.landedHome).length / n : 0,
  };
}

export const CURVE_GRID = Array.from({ length: 31 }, (_, i) => i * 30);

/**
 * Is the belief calibrated? The model's mean POS at each grid time, from the blind flights (the
 * same searches with nobody to find, so the curve is never cut short by a find), against the
 * share of the real flights that had found the person by then. The benchmark always puts the
 * person within aerial reach, so POS is conditioned on that (divided by the reachable share).
 */
export function posCalibration(
  blind: readonly SarFlight[],
  real: readonly SarFlight[],
  grid = CURVE_GRID,
): { t: number; predicted: number; observed: number }[] {
  const withPos = blind.filter((f) => f.pos && f.pos.length && f.reachable);
  return grid.map((t) => {
    const predicted = withPos.length
      ? withPos.reduce((s, f) => {
          let v = 0;
          for (const [ts, p] of f.pos!) if (ts <= t) v = p;
          return s + Math.min(1, v / f.reachable!);
        }, 0) / withPos.length
      : 0;
    const observed = real.length ? real.filter((f) => f.tFind !== null && f.tFind <= t).length / real.length : 0;
    return { t, predicted, observed };
  });
}

export function sarBenchReport(flights: SarFlight[], commit: string, meta: Record<string, unknown> = {}) {
  const conditions = Object.fromEntries(
    SAR_CONDITIONS.map((c) => {
      const all = flights.filter((f) => f.condition === c);
      const mine = all.filter((f) => !f.blind);
      const methods = Object.fromEntries(
        SAR_METHODS.map((m) => {
          const fm = mine.filter((f) => f.method === m);
          return [
            m,
            {
              ...summarizeSar(fm),
              curve: findCurve(fm, CURVE_GRID),
              outdoor: summarizeSar(fm.filter((f) => f.where === 'outdoor')),
              building: summarizeSar(fm.filter((f) => f.where === 'building')),
            },
          ];
        }),
      );
      const lanes = mine.filter((f) => f.method === 'lanes');
      const vsLanes = Object.fromEntries(
        SAR_METHODS.filter((m) => m !== 'lanes').map((m) => [m, pairedBootstrap(lanes, mine.filter((f) => f.method === m))]),
      );
      const sw = mine.filter((f) => f.method === 'lanes-sw');
      const poaVsSweep = pairedBootstrap(sw, mine.filter((f) => f.method === 'poa'));
      const strategies: Record<string, Record<string, ReturnType<typeof summarizeSar>>> = {};
      for (const s of new Set(mine.map((f) => f.strategy))) {
        strategies[s] = Object.fromEntries(SAR_METHODS.map((m) => [m, summarizeSar(mine.filter((f) => f.method === m && f.strategy === s))]));
      }
      return [
        c,
        {
          scenes: new Set(mine.map((f) => f.seed)).size,
          methods,
          vsLanes,
          poaVsSweep,
          strategies,
          calibration: posCalibration(
            all.filter((f) => f.blind && f.method === 'poa'),
            mine.filter((f) => f.method === 'poa'),
          ),
        },
      ];
    }),
  );
  // POS curves are summarized in `calibration`; per-flight they would make the file ten times larger.
  const slim = flights.map((f) => ({ ...f, pos: undefined }));
  return { schema: 1, commit, budgetSec: SAR_TIME_BUDGET_SEC, seedBase: SAR_SEED_BASE, curveGrid: CURVE_GRID, ...meta, conditions, flights: slim };
}

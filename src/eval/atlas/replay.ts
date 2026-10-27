// Atlas cases as links: a case packs into a short URL-safe code, and the Missions page
// (demo.html?atlas=<code>) rebuilds the same world, mission and faults and flies it live.
import type { RunnerOptions } from '../../mission/autonomy/contracts';
import { createPerceptionSource } from '../../mission/autonomy/perception';
import type { LoadedScenario, Scenario } from '../../mission/autonomy/scenarios';
import { buildMissionWorld, missionForWorld } from '../../mission/autonomy/world';
import { caseSetup, DIMS, type AtlasCase } from './space';

/** Packs a case: index, layout seed, placement seed, then category and value per dimension (full precision, so the replay is exact). */
export function encodeCase(c: AtlasCase): string {
  const dims = DIMS.map((d, j) => (d.continuous ? `${c.k[j]}:${c.u[j]}` : String(c.k[j])));
  return [c.i, c.layoutSeed, c.placementSeed, ...dims].join('_');
}

export function decodeCase(code: string): AtlasCase | null {
  const parts = code.split('_');
  if (parts.length !== DIMS.length + 3) return null;
  const [i, layoutSeed, placementSeed] = parts.slice(0, 3).map(Number) as [number, number, number];
  const k: number[] = [];
  const u: number[] = [];
  for (let j = 0; j < DIMS.length; j++) {
    const [kk, uu] = parts[j + 3]!.split(':');
    const cat = Number(kk);
    if (!Number.isInteger(cat) || cat < 0 || cat >= DIMS[j]!.values.length) return null;
    k.push(cat);
    u.push(DIMS[j]!.continuous ? Number(uu) : 0);
  }
  if (![i, layoutSeed, placementSeed].every(Number.isInteger) || u.some((x) => !(x >= 0 && x < 1))) return null;
  return { i, layoutSeed, placementSeed, k, u };
}

const MISSION_TITLES = {
  'find-person': 'Find a missing person',
  'sweep-occupied': 'Confirm clear, with someone inside',
  'sweep-empty': 'Confirm an empty site is clear',
  'find-object': 'Find a dropped object',
} as const;

/** A replayable scenario for an atlas case: the world, mission and faults the study flew, on the quadrotor. */
export function loadAtlasCase(c: AtlasCase): LoadedScenario {
  const setup = caseSetup(c);
  const d = setup.decoded;
  const world = buildMissionWorld(setup.world);
  const mission = missionForWorld(world, setup.mission);
  const options: RunnerOptions = {
    dt: 0.05,
    vehicle: 'quad',
    ...(setup.faults.length ? { faults: { plan: setup.faults, seed: world.seed } } : {}),
  };
  const fault = d.fault === 'none' ? 'no fault' : d.fault.replace('-', ' ');
  const scenario: Scenario = {
    id: 'cabin-day',
    title: `Atlas case ${c.layoutSeed}`,
    blurb: `${MISSION_TITLES[d.mission]}: a flight from the failure atlas, rebuilt from its seeds. ${d.buildings} building${d.buildings > 1 ? 's' : ''}, ${fault}, ${d.wind.meanMps.toFixed(1)} m/s wind from ${Math.round(d.wind.dirDeg)}° after ${Math.round(d.wind.onsetSec)} s.`,
    tags: ['Atlas', `${d.buildings} building${d.buildings > 1 ? 's' : ''}`, d.mission.startsWith('sweep') ? 'Ask to enter' : 'Auto entry'],
    world: setup.world,
    mission: setup.mission,
    runner: options,
    expect: { outcome: d.mission === 'sweep-empty' ? 'clear' : 'found', landedHome: true },
  };
  return { scenario, world, mission, source: createPerceptionSource(world), options };
}

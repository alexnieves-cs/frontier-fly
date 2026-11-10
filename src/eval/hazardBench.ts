// Hazard mission bench: shut and ajar doors, glass and mirrors, and smoke in the woods-to-building
// missions, on a fresh seed block, flown by the sensor planner and the policy with hazard awareness
// and by the sensor planner without it (the control). scripts/study/hazard-study.ts runs it in shards.
import { findDoorways } from '../gen/hazards';
import type { HazardParams } from '../shared/types';
import { generateOutdoor } from '../world/outdoor/generate';
import type { Scenario } from '../mission/autonomy/scenarios';
import { SCENARIOS } from '../mission/autonomy/scenarios';
import { createPerceptionSource } from '../mission/autonomy/perception';
import { flyMission, OCCUPIED_SWEEP, summarizeMissions, wilson, type MissionFlight } from './missions';

/** Fresh block, never flown before this bench. Development flights used 81901-81999 only. */
export const HAZARD_MISSION_SEED_BASE = 86501;
export const HAZARD_MISSION_SEEDS = 30;

export const DOOR_HAZARDS: HazardParams = { closedDoors: 0.35, ajarDoors: 0.6 };
export const GLASS_HAZARDS: HazardParams = { glassRooms: 0.5, mirrors: 2 };
export const SMOKE_HAZARDS: HazardParams = { smokeRooms: 0.5, smokeVisibility: [1.5, 4] };

const cabin = SCENARIOS.find((s) => s.id === 'cabin-day')!;

/** A hazard scenario; `rooms` names a building that must have interior doorways (see hazardSeeds). */
export type HazardScenario = Scenario & { rooms?: number };

function withHazards(base: Scenario, id: string, title: string, hazards: HazardParams, rooms?: number): HazardScenario {
  return { ...base, id: id as Scenario['id'], title, world: { ...base.world, hazards }, ...(rooms != null ? { rooms } : {}) };
}

/** The compound sweep with the person in the first building, the one that can have several rooms. */
const OCCUPIED_FIRST: Scenario = { ...OCCUPIED_SWEEP, world: { ...OCCUPIED_SWEEP.world, targets: [{ classId: 'person', state: 'lying', where: 'indoor', building: 0 }] } };

/**
 * The hazard scenarios. The sweeps have a person lying inside, so a clear called there is a false
 * clear. Doors and glass need interior walls: most mission shells are one room (the second building of
 * a compound never has a doorway), so those scenarios fly the first seeds of the block whose first
 * building has one. Every room of a mission shell also has its own outside opening, so a shut interior
 * door slows the search but cannot hide anyone from a drone that goes round the outside.
 */
export const HAZARD_SCENARIOS: readonly HazardScenario[] = [
  withHazards(OCCUPIED_FIRST, 'doors-occupied', 'Sweep with someone inside, shut and ajar doors', DOOR_HAZARDS, 0),
  withHazards(OCCUPIED_SWEEP, 'smoke-occupied', 'Sweep with someone inside, smoke', SMOKE_HAZARDS),
  withHazards(cabin, 'smoke-cabin', 'Injured hiker in a smoky cabin', SMOKE_HAZARDS),
  withHazards(cabin, 'glass-cabin', 'Injured hiker, glass rooms and mirrors', GLASS_HAZARDS, 0),
];

/**
 * The scenario's seeds: the first `n` from `base` up, or, for a scenario that needs rooms, the first
 * `n` whose named building has an interior doorway before any hazard (a rule fixed before any run).
 */
export function hazardSeeds(scenario: HazardScenario, n: number, base: number): number[] {
  const out: number[] = [];
  for (let seed = base; out.length < n && seed < base + 100 * n; seed++) {
    if (scenario.rooms == null) out.push(seed);
    else {
      const shell = generateOutdoor({ seed, buildings: scenario.world.buildings }).buildings[scenario.rooms];
      if (shell && findDoorways(shell.spec).length > 0) out.push(seed);
    }
  }
  return out;
}

/** `sensorNaive` is the sensor planner with hazard awareness off: the control. */
export const HAZARD_MODES = ['sensor', 'policy', 'sensorNaive'] as const;
/**
 * The hazard-aware explorers with a thermal camera beside the visible one (perception/model.ts
 * THERMAL). Not part of the bench above or its pass conditions: the thermal follow-up flies these
 * and their camera-only twins on the smoke scenarios, on its own fresh block.
 */
export const HAZARD_THERMAL_MODES = ['sensorThermal', 'policyThermal'] as const;
export const HAZARD_ALL_MODES = [...HAZARD_MODES, ...HAZARD_THERMAL_MODES] as const;
export type HazardMode = (typeof HAZARD_ALL_MODES)[number];

/** The thermal follow-up: fresh block, never flown before it. Development flights used 81901-81999 only. */
export const HAZARD_THERMAL_SEED_BASE = 87001;
export const HAZARD_THERMAL_SCENARIOS = ['smoke-occupied', 'smoke-cabin'] as const;
export const HAZARD_THERMAL_RUN_MODES: readonly HazardMode[] = ['sensor', 'sensorThermal', 'policy', 'policyThermal'];

export interface HazardJob {
  scenario: string;
  seed: number;
  mode: HazardMode;
}

export interface HazardFlight extends MissionFlight {
  mode: HazardMode;
}

/**
 * Pass conditions, fixed before the first flight on 86501+. With awareness on (sensor and policy):
 * no false clear in any sweep, the hiker found in at least `found` of the smoke and glass cabin flights.
 */
export const HAZARD_MISSION_PASS = { modes: ['sensor', 'policy'] as HazardMode[], falseClears: 0, found: 0.8 } as const;

export function hazardJobs(seeds = HAZARD_MISSION_SEEDS, base = HAZARD_MISSION_SEED_BASE, modes: readonly HazardMode[] = HAZARD_MODES, scenarios?: readonly string[]): HazardJob[] {
  const out: HazardJob[] = [];
  for (const s of HAZARD_SCENARIOS) {
    if (scenarios && !scenarios.includes(s.id)) continue;
    for (const seed of hazardSeeds(s, seeds, base)) for (const mode of modes) out.push({ scenario: s.id, seed, mode });
  }
  return out;
}

export async function flyHazard(job: HazardJob): Promise<HazardFlight> {
  const indoor = job.mode.startsWith('policy') ? 'policy' : 'sensor';
  const thermal = job.mode.endsWith('Thermal');
  const flight = await flyMission({ scenario: job.scenario, seed: job.seed, indoor }, (world) => createPerceptionSource(world, { thermal }), undefined, {
    scenarios: HAZARD_SCENARIOS,
    runner: { hazardAware: job.mode !== 'sensorNaive' },
  });
  return { ...flight, mode: job.mode };
}

export function hazardBenchReport(flights: HazardFlight[], commit: string, seeds: number, base: number) {
  const cells = [];
  for (const s of HAZARD_SCENARIOS) {
    for (const mode of HAZARD_ALL_MODES) {
      const fs = flights.filter((f) => f.scenario === s.id && f.mode === mode);
      if (!fs.length) continue;
      const found = fs.filter((f) => f.outcome === 'found').length;
      const withFalseClear = fs.filter((f) => f.falseClears > 0).length;
      cells.push({
        scenario: s.id,
        mode,
        ...summarizeMissions(fs),
        found,
        foundRate: found / fs.length,
        foundCi: wilson(found, fs.length),
        flightsWithFalseClear: withFalseClear,
        falseClearCi: wilson(withFalseClear, fs.length),
        flightsWithShutDoors: fs.filter((f) => (f.shutDoors ?? 0) > 0).length,
      });
    }
  }
  const aware = cells.filter((c) => HAZARD_MISSION_PASS.modes.includes(c.mode));
  const falseClears = aware.reduce((a, c) => a + c.falseClears, 0);
  const cabins = aware.filter((c) => c.scenario.endsWith('-cabin'));
  const pass = {
    falseClears: { value: falseClears, target: HAZARD_MISSION_PASS.falseClears, met: falseClears <= HAZARD_MISSION_PASS.falseClears },
    found: cabins.map((c) => ({ scenario: c.scenario, mode: c.mode, value: c.foundRate, target: HAZARD_MISSION_PASS.found, met: c.foundRate >= HAZARD_MISSION_PASS.found })),
  };
  const scenarios = HAZARD_SCENARIOS.map((s) => ({ id: s.id, title: s.title, hazards: s.world.hazards, seeds: hazardSeeds(s, seeds, base) }));
  return { schema: 1, commit, seeds, seedBase: base, scenarios, pass, cells, flights };
}

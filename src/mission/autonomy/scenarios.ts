import type { TargetClass } from '../../targets/types';
import type { Mission } from '../types';
import type {
  EntryRecord,
  LiveDetectionSource,
  MissionForWorldOptions,
  MissionWorld,
  MissionWorldOptions,
  RunnerOptions,
  RunOutcome,
} from './contracts';
import { createPerceptionSource } from './perception';
import { buildMissionWorld, missionForWorld } from './world';

/** What a scenario's seeded run is expected to produce. Tests and the demo read this. */
export interface ScenarioExpect {
  outcome: RunOutcome;
  /** Class of the confirmed true positive for find missions. */
  findClass?: TargetClass;
  /** Every building must be declared clear (sweep missions). */
  allBuildingsClear?: boolean;
  /** At least one entry with this decision must be recorded. */
  entryDecision?: EntryRecord['decision'];
  landedHome: boolean;
}

export interface Scenario {
  id: ScenarioId;
  title: string;
  /** One line for the picker card. */
  blurb: string;
  /** Short tags for the picker card: light, building count, entry policy. */
  tags: string[];
  world: MissionWorldOptions;
  mission: MissionForWorldOptions;
  runner: RunnerOptions;
  expect: ScenarioExpect;
}

export type ScenarioId = 'cabin-day' | 'compound-clear' | 'backpack-night';

const approve: NonNullable<RunnerOptions['operator']> = () => 'approve';

/*
 * Seed choice. generateOutdoor gives each shell exactly one passable opening
 * and cycles its kind with the seed. Every seed below builds on its first
 * attempt (world.seed equals the requested seed, so nothing was retried) and
 * every shell's passable opening is a wall opening on the ground floor, never
 * a roof hatch, so the entry is a level fly-through:
 *   20  one shell, garage gap 1.55 x 1.65 m, the shortest corridor (6 cells).
 *   37  two shells, open window 0.80 x 0.95 m and garage gap 1.55 x 1.65 m.
 *   17  one shell, open window 0.80 x 0.95 m, 7-cell corridor.
 * The airframe needs 0.38 m, so the narrowest entry keeps 0.42 m of margin.
 */
export const SCENARIOS: readonly Scenario[] = [
  {
    id: 'cabin-day',
    title: 'Injured hiker in a cabin',
    blurb: 'Daylight search of a wooded lot. A simulated hiker is lying injured inside the cabin. The drone sweeps the woods, picks the safest opening, flies in, confirms the person, and returns home.',
    tags: ['Day', '1 building', 'Auto entry'],
    world: {
      seed: 20,
      buildings: 1,
      timeOfDay: 13,
      flashlightOn: false,
      interiorLights: 'on',
      targets: [{ classId: 'person', state: 'lying', where: 'indoor', building: 0 }],
      decoys: 1,
    },
    mission: {
      missionType: 'find_person',
      name: 'Cabin search',
      targets: [{ id: 'tgt-person', kind: 'person', label: 'Injured hiker (simulated)', minCount: 1, minConfidence: null }],
      requiredConfidence: 0.8,
      entryPolicy: 'auto',
      budgets: { batteryPct: 80, timeSec: 900 },
    },
    runner: { dt: 0.05, indoor: 'auto' },
    expect: { outcome: 'found', findClass: 'person', entryDecision: 'auto', landedHome: true },
  },
  {
    id: 'compound-clear',
    title: 'Confirm a compound is clear',
    blurb: 'Safety sweep of two buildings. Nobody is inside, only look-alike decoys. Each entry waits for an operator approval, and the drone must prove every room empty before it calls the site clear.',
    tags: ['Day', '2 buildings', 'Ask to enter'],
    world: {
      seed: 37,
      buildings: 2,
      timeOfDay: 10.5,
      flashlightOn: false,
      interiorLights: 'on',
      targets: [],
      decoys: 2,
    },
    mission: {
      missionType: 'safety_sweep',
      name: 'Compound sweep',
      targets: [{ id: 'tgt-clear', kind: 'clearance', label: 'Confirm both buildings are clear', minCount: 1, minConfidence: null }],
      requiredConfidence: 0.8,
      entryPolicy: 'ask',
      budgets: { batteryPct: 85, timeSec: 1200 },
    },
    runner: { dt: 0.05, indoor: 'auto', operator: approve },
    expect: { outcome: 'clear', allBuildingsClear: true, entryDecision: 'approved', landedHome: true },
  },
  {
    id: 'backpack-night',
    title: 'Lost backpack at night',
    blurb: 'Night search with the flashlight on and every interior light off. The drone hunts for a dropped backpack inside a dark building and has to tell it apart from a look-alike bag.',
    tags: ['Night', '1 building', 'Auto entry'],
    world: {
      seed: 17,
      buildings: 1,
      timeOfDay: 23,
      flashlightOn: true,
      interiorLights: 'off',
      targets: [{ classId: 'backpack', where: 'indoor', building: 0 }],
      decoys: 1,
    },
    mission: {
      missionType: 'find_objects',
      name: 'Night backpack search',
      targets: [{ id: 'tgt-backpack', kind: 'object', label: 'Backpack', minCount: 1, minConfidence: null }],
      requiredConfidence: 0.8,
      entryPolicy: 'auto',
      budgets: { batteryPct: 80, timeSec: 900 },
    },
    runner: { dt: 0.05, indoor: 'auto' },
    expect: { outcome: 'found', findClass: 'backpack', entryDecision: 'auto', landedHome: true },
  },
];

export const DEFAULT_SCENARIO: ScenarioId = 'cabin-day';

export function isScenarioId(id: string | null | undefined): id is ScenarioId {
  return SCENARIOS.some((scenario) => scenario.id === id);
}

export function scenarioById(id: string): Scenario {
  const scenario = SCENARIOS.find((item) => item.id === id);
  if (!scenario) throw new Error(`unknown scenario ${id}`);
  return scenario;
}

export interface LoadedScenario {
  scenario: Scenario;
  world: MissionWorld;
  mission: Mission;
  source: LiveDetectionSource;
  options: RunnerOptions;
}

/** Build the seeded world, its mission, a fresh detector, and runner options. */
export function loadScenario(id: string): LoadedScenario {
  const scenario = scenarioById(id);
  const world = buildMissionWorld(scenario.world);
  const mission = missionForWorld(world, scenario.mission);
  const source = createPerceptionSource(world);
  return { scenario, world, mission, source, options: { ...scenario.runner } };
}

import { evalEpisodeSetup } from '../eval/coverage';
import type { BuildingSpec, EpisodeConfig } from '../shared/types';
import { defaultEpisodeConfig, policyEpisodeLimits } from '../sim/env';
import { lightingPreset } from '../sim/lighting';
import { generateLargeBuilding } from './largeBuilding';

/** A team bench condition: which buildings, and how long the team has. */
export interface TeamCondition {
  id: string;
  label: string;
  /** Building side, meters (stage 3 generator: rooms, hallways, loops). */
  size: number;
  maxSteps: number;
}

export const TEAM_CONDITIONS: TeamCondition[] = [
  // The single-drone bench's hard buildings and budget (2400 steps = 120 s).
  { id: 'hard', label: 'Hard, 20 x 20 m, 120 s', size: 20, maxSteps: 2400 },
  // Four hard floor plans tiled 2 x 2 (team/largeBuilding.ts): four times the area, twice the time,
  // and the radio no longer reaches everywhere.
  { id: 'large', label: 'Large, 40 x 40 m, 240 s', size: 40, maxSteps: 4800 },
];

/** Building and episode config for a team episode. The 20 m condition is exactly the coverage bench's stage-3 setup. */
export function teamEpisodeSetup(cond: TeamCondition, seed: number): { spec: BuildingSpec; envSeed: number; cfg: EpisodeConfig & { maxSteps: number } } {
  if (cond.size === 20) {
    const s = evalEpisodeSetup(3, seed, 'day');
    s.cfg.maxSteps = cond.maxSteps;
    return s;
  }
  const spec = generateLargeBuilding(seed);
  const nRooms = spec.floors.reduce((n, f) => n + f.rooms.length, 0);
  const cfg = defaultEpisodeConfig(3, nRooms);
  Object.assign(cfg, policyEpisodeLimits(3), { maxSteps: cond.maxSteps });
  cfg.dynamics = 'kinematic';
  cfg.lighting = lightingPreset('day', nRooms);
  return { spec, envSeed: spec.seed, cfg: cfg as EpisodeConfig & { maxSteps: number } };
}

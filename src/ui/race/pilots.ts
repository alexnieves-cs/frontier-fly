import { evalEpisodeSetup } from '../../eval/coverage';
import { classicalPilot, guardedAction } from '../../eval/pilots';
import { RNG } from '../../shared/rng';
import type { BuildingSpec, EpisodeConfig } from '../../shared/types';
import { defaultDynParams, sampleDynParams, sampleNoise } from '../../sim/dynamics/dr';
import type { ExplorerEnv } from '../../sim/env';
import { frontierAction } from '../../sim/frontier';
import { heuristicAction } from '../../sim/heuristic';
import type { LightingPreset } from '../../sim/lighting';
import { waypointAction } from '../../sim/waypointPilot';

/**
 * Race setup. Each condition and pilot here is the same episode the coverage
 * study flies (scripts/study/coverage-study.ts startEpisode), so a race on a
 * study seed reproduces that study episode step for step. tests/unit/race.test.ts
 * checks this against docs/study.json.
 */

export const RACE_PILOTS = ['policy', 'classical', 'waypoint', 'teacher', 'guarded', 'heuristic'] as const;
export type RacePilot = (typeof RACE_PILOTS)[number];

/** The four lanes shown by default: learned, sensor-only classical, frontier waypoint, and the privileged oracle. */
export const DEFAULT_PILOTS: RacePilot[] = ['policy', 'classical', 'waypoint', 'teacher'];

export interface PilotInfo {
  name: string;
  short: string;
  /** What the pilot is allowed to know. */
  knows: string;
  color: string;
  /** True when the pilot reads the observation vector the inspector shows. */
  readsObs: boolean;
}

export const PILOT_INFO: Record<RacePilot, PilotInfo> = {
  policy: {
    name: 'Learned policy',
    short: 'Policy',
    knows: 'Lidar and its own maps. A 1088 → 256 → 128 → 4 network trained by imitation (DAgger).',
    color: '#56e0ff',
    readsObs: true,
  },
  classical: {
    name: 'Sensor planner',
    short: 'Planner',
    knows: 'Lidar and its own occupancy map. Hand-written frontier search with a safety filter.',
    color: '#3dff9a',
    readsObs: false,
  },
  waypoint: {
    name: 'Frontier follower',
    short: 'Follower',
    knows: 'The same navigation cue the policy gets, steered straight at with no learning.',
    color: '#ffd166',
    readsObs: true,
  },
  teacher: {
    name: 'Oracle',
    short: 'Oracle',
    knows: 'The true floor plan and furniture. The teacher the policy imitates, not a fair rival.',
    color: '#c79bff',
    readsObs: false,
  },
  guarded: {
    name: 'Guarded policy',
    short: 'Guarded',
    knows: 'The learned policy with the lidar guard that vetoes moves into walls.',
    color: '#7fb8ff',
    readsObs: true,
  },
  heuristic: {
    name: 'Lidar heuristic',
    short: 'Heuristic',
    knows: 'Only the lidar ring. Wall following with no map.',
    color: '#ff8fa3',
    readsObs: true,
  },
};

export interface RaceCondition {
  id: string;
  label: string;
  stage: number;
  /** First study seed. The study flies seedBase .. seedBase + 29. */
  seedBase: number;
  lighting: LightingPreset[];
  quad?: number;
}

/** Mirrors CONDITIONS in scripts/study/coverage-study.ts. */
export const RACE_CONDITIONS: RaceCondition[] = [
  { id: 'easy', label: 'Easy: one room', stage: 0, seedBase: 20001, lighting: ['day', 'night-dark'] },
  { id: 'hard', label: 'Hard: rooms, hallways, loops', stage: 3, seedBase: 30001, lighting: ['day', 'night-dark'] },
  { id: 'floors', label: 'Multi-floor', stage: 4, seedBase: 40001, lighting: ['day'] },
  { id: 'quad', label: 'Hard, quadrotor + noise', stage: 3, seedBase: 50001, lighting: ['day', 'night-dark'], quad: 1 },
];

export const STUDY_SEEDS = 30;

export function conditionById(id: string | null | undefined): RaceCondition {
  return RACE_CONDITIONS.find((c) => c.id === id) ?? RACE_CONDITIONS[1]!;
}

export function isRacePilot(id: string): id is RacePilot {
  return (RACE_PILOTS as readonly string[]).includes(id);
}

export interface RaceSetup {
  condition: string;
  seed: number;
  lighting: LightingPreset;
  pilots: RacePilot[];
}

export type Act = (obs: Float32Array, env: ExplorerEnv) => Float32Array;
export type PolicyFn = (obs: Float32Array) => Float32Array;

/** Building, env seed and episode config for one race. Every lane gets the same three. */
export function raceEpisode(setup: RaceSetup): { spec: BuildingSpec; envSeed: number; cfg: EpisodeConfig & { maxSteps: number } } {
  const c = conditionById(setup.condition);
  const { spec, envSeed, cfg } = evalEpisodeSetup(c.stage, setup.seed, setup.lighting);
  if (c.quad) {
    const rng = new RNG(setup.seed, 'study-dr');
    cfg.dynamics = 'quad';
    cfg.dyn = sampleDynParams(rng, defaultDynParams(), c.quad);
    cfg.noise = sampleNoise(rng, c.quad, setup.lighting !== 'day');
  }
  return { spec, envSeed, cfg };
}

/** The controller for one lane. `policy` is the shipped network, deterministic. */
export function pilotAct(pilot: RacePilot, spec: BuildingSpec, policy: PolicyFn | null): Act {
  if (pilot === 'teacher') {
    return (_obs, env) => {
      const s = env.snapshot();
      return frontierAction(spec, env.visitedGrid, s.pos, s.yaw);
    };
  }
  if (pilot === 'classical') {
    const p = classicalPilot();
    p.reset();
    return p.act;
  }
  if (pilot === 'waypoint') return (obs) => waypointAction(obs);
  if (pilot === 'heuristic') return (obs) => heuristicAction(obs);
  if (!policy) throw new Error(`The ${PILOT_INFO[pilot].name.toLowerCase()} needs the shipped checkpoint.`);
  if (pilot === 'guarded') return (obs, env) => guardedAction(policy(obs), obs, env);
  return (obs) => policy(obs);
}

export type EndReason = 'running' | 'complete' | 'time' | 'stalled' | 'crashed' | 'left';

/** Why an episode ended, from the env's own done rules (src/sim/env.ts step). */
export function endReason(done: boolean, steps: number, maxSteps: number, coverage: number, truncated: boolean, collisions: number, maxCollisions: number): EndReason {
  if (!done) return 'running';
  if (coverage >= 0.95) return 'complete';
  if (collisions > maxCollisions) return 'crashed';
  if (truncated) return 'stalled';
  if (steps >= maxSteps) return 'time';
  return 'left';
}

export const END_LABELS: Record<EndReason, string> = {
  running: 'Flying',
  complete: 'Searched it all',
  time: 'Time up',
  stalled: 'Stalled, no new area',
  crashed: 'Too many collisions',
  left: 'Left the building',
};

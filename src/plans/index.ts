import type { BuildingSpec, EpisodeConfig } from '../shared/types';
import { defaultEpisodeConfig, policyEpisodeLimits } from '../sim/env';
import { lightingPreset, type LightingPreset } from '../sim/lighting';
import { planToBuilding, type ConvertedPlan } from './convert';
import type { PlanRecord, PlanSet } from './types';

export { planToBuilding, planSeed, type ConvertedPlan } from './convert';
export type { PlanRecord, PlanSet } from './types';

/** Real apartments from Swiss Dwellings 3.0.0 (CC BY 4.0), served next to the app. */
export const PLAN_SET_PATH = 'public/plans/swiss-dwellings.json';
export const PLAN_SET_URL = 'plans/swiss-dwellings.json';

/**
 * Real plans are flown with the stage-3 protocol: their size and room count
 * match the hard generated buildings, and the policy's time feature is in
 * distribution with the stage-3 step budget.
 */
export const PLAN_STAGE = 3;

/** Env seed blocks for real-plan episodes (lighting draws, sensor noise). eval: 72001+, dev: 74001+. */
export const PLAN_SEED_BASE = { dev: 74001, eval: 72001 } as const;

export function parsePlanSet(raw: unknown): PlanSet {
  const set = raw as PlanSet;
  if (!set || !Array.isArray(set.plans) || !set.dataset) throw new Error('not a plan set');
  return set;
}

const cache = new WeakMap<PlanRecord, ConvertedPlan | null>();

/** Converted (and furnished) building for a plan, cached per record. */
export function planBuilding(plan: PlanRecord): ConvertedPlan | null {
  if (!cache.has(plan)) cache.set(plan, planToBuilding(plan));
  return cache.get(plan)!;
}

/** Env seed of a plan: its split's block, plus its index within that split, plus 100 per repeat. */
export function planEnvSeed(set: PlanSet, plan: PlanRecord, rep = 0): number {
  const index = set.plans.filter((p) => p.split === plan.split).indexOf(plan);
  return PLAN_SEED_BASE[plan.split] + Math.max(0, index) + 100 * rep;
}

/** Same shape as evalEpisodeSetup, for a real plan. */
export function planEpisodeSetup(
  set: PlanSet,
  plan: PlanRecord,
  lighting: LightingPreset,
  rep = 0,
): { spec: BuildingSpec; envSeed: number; cfg: EpisodeConfig & { maxSteps: number } } {
  const conv = planBuilding(plan);
  if (!conv) throw new Error(`plan ${plan.id} does not convert`);
  const spec = conv.spec;
  const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
  const cfg = defaultEpisodeConfig(PLAN_STAGE, nRooms);
  Object.assign(cfg, policyEpisodeLimits(PLAN_STAGE));
  cfg.dynamics = 'kinematic';
  cfg.lighting = lightingPreset(lighting, nRooms);
  return { spec, envSeed: planEnvSeed(set, plan, rep), cfg: cfg as EpisodeConfig & { maxSteps: number } };
}

import type { BuildingSpec, EpisodeConfig } from '../shared/types';
import { defaultEpisodeConfig, policyEpisodeLimits } from '../sim/env';
import { lightingPreset, type LightingPreset } from '../sim/lighting';
import { floorsToBuilding, planSeed, type ConvertedPlan } from './convert';
import type { HomeRecord, HomeSet } from './types';

/**
 * Real multi-storey homes (maisonettes) from Swiss Dwellings 3.0.0 (CC BY 4.0), each storey joined to
 * the next by an open shaft where its real staircase is (scripts/plans/import-swiss-homes.ts).
 */
export const HOME_SET_PATH = 'public/plans/swiss-homes.json';
export const HOME_SET_URL = 'plans/swiss-homes.json';
/** A second, smaller held-out set: the further eval homes the same selection rule yields, none in HOME_SET_PATH. */
export const HOME_HOLDOUT_PATH = 'public/plans/swiss-homes-holdout.json';
/** Env seeds of the held-out set (report only). */
export const HOME_HOLDOUT_SEED_BASE = 88801;

/** Homes fly the multi-floor protocol (FrontierBench's `floors` task is stage 4). */
export const HOME_STAGE = 4;

/** Env seed blocks for real-home episodes. eval: 88201+ (report only), dev: 88601+ (tune freely); +100 per repeat. */
export const HOME_SEED_BASE = { dev: 88601, eval: 88201 } as const;

export function parseHomeSet(raw: unknown): HomeSet {
  const set = raw as HomeSet;
  if (!set || !Array.isArray(set.homes) || !set.dataset) throw new Error('not a home set');
  return set;
}

/** Converted (and furnished) building for a home record. */
export function homeToBuilding(home: HomeRecord, opts: { furniture?: boolean } = {}): ConvertedPlan | null {
  return floorsToBuilding(home.storeys, planSeed(home.id), opts);
}

const cache = new WeakMap<HomeRecord, ConvertedPlan | null>();

export function homeBuilding(home: HomeRecord): ConvertedPlan | null {
  if (!cache.has(home)) cache.set(home, homeToBuilding(home));
  return cache.get(home)!;
}

export function homeEnvSeed(set: HomeSet, home: HomeRecord, rep = 0, base: number = HOME_SEED_BASE[home.split]): number {
  const index = set.homes.filter((h) => h.split === home.split).indexOf(home);
  return base + Math.max(0, index) + 100 * rep;
}

/** Same shape as evalEpisodeSetup, for a real home: the stage-4 limits (120 s), day lighting by default. */
export function homeEpisodeSetup(
  set: HomeSet,
  home: HomeRecord,
  lighting: LightingPreset,
  rep = 0,
  seedBase?: number,
): { spec: BuildingSpec; envSeed: number; cfg: EpisodeConfig & { maxSteps: number } } {
  const conv = homeBuilding(home);
  if (!conv) throw new Error(`home ${home.id} does not convert`);
  const spec = conv.spec;
  const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
  const cfg = defaultEpisodeConfig(HOME_STAGE, nRooms);
  Object.assign(cfg, policyEpisodeLimits(HOME_STAGE));
  cfg.dynamics = 'kinematic';
  cfg.lighting = lightingPreset(lighting, nRooms);
  return { spec, envSeed: homeEnvSeed(set, home, rep, seedBase), cfg: cfg as EpisodeConfig & { maxSteps: number } };
}

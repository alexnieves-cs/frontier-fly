/**
 * FrontierBench: a frozen set of indoor exploration tasks with fixed seeds,
 * so anyone can score a pilot the same way the report scores ours.
 *
 * Each task is a building stage, an airframe, lighting and a speed cap. Each
 * has a dev split (tune on it freely) and a test split (score on it once).
 * The test seeds are public, as everything in a static repository is, so
 * the leaderboard runs on trust plus re-scoring: CI scores every entry's
 * pilot again from its code.
 *
 * The seed blocks 81001+ (dev) and 82001+ (test) are used nowhere else.
 */
import type { LightingPreset } from '../sim/lighting';

export const BENCH_VERSION = 1;

export interface BenchTask {
  id: string;
  label: string;
  stage: number;
  dynamics: 'kinematic' | 'quad';
  lighting: LightingPreset;
  /** Speed a full action asks for, m/s. */
  speedH: number;
  /** Domain randomization scale for the quadrotor (mass, thrust, drag, lag, noise, latency). */
  dr: number;
}

export const BENCH_TASKS: readonly BenchTask[] = [
  { id: 'room', label: 'One room', stage: 0, dynamics: 'kinematic', lighting: 'day', speedH: 2, dr: 0 },
  { id: 'hard', label: 'Rooms, hallways, loops', stage: 3, dynamics: 'kinematic', lighting: 'day', speedH: 2, dr: 0 },
  { id: 'dark', label: 'Rooms, hallways, loops, lights off', stage: 3, dynamics: 'kinematic', lighting: 'night-dark', speedH: 2, dr: 0 },
  { id: 'fast', label: 'Rooms, hallways, loops at 4 m/s', stage: 3, dynamics: 'kinematic', lighting: 'day', speedH: 4, dr: 0 },
  { id: 'floors', label: 'Several floors joined by shafts', stage: 4, dynamics: 'kinematic', lighting: 'day', speedH: 2, dr: 0 },
  { id: 'quad', label: 'Quadrotor with randomized dynamics and noise', stage: 3, dynamics: 'quad', lighting: 'day', speedH: 2, dr: 1 },
];

export type BenchSplit = 'dev' | 'test';
export const BENCH_SEEDS: Record<BenchSplit, { base: number; count: number }> = {
  dev: { base: 81001, count: 20 },
  test: { base: 82001, count: 30 },
};

/** Seeds of a split; each task offsets the block so no two tasks share a building seed. */
export function benchSeeds(split: BenchSplit, taskIndex: number): number[] {
  const { base, count } = BENCH_SEEDS[split];
  return Array.from({ length: count }, (_, i) => base + taskIndex * 100 + i);
}

/** A flight counts as safe when it touched nothing. */
export const SAFE_COLLISIONS = 0;

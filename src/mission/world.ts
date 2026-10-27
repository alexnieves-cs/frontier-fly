import type { Point } from './types';

/**
 * Site context for the mission map: footprints drawn under the plan.
 * `outdoorWorldProvider` (autonomy/world.ts) derives them from the same
 * generated world the runner flies.
 */
export interface WorldFootprint {
  id: string;
  kind: 'building' | 'road' | 'water' | 'canopy';
  label: string;
  polygon: Point[];
  heightM: number | null;
}

export interface WorldSnapshot {
  provider: string;
  seed: number;
  siteName: string;
  footprints: WorldFootprint[];
}

export interface WorldProvider {
  readonly id: string;
  load(seed: number): WorldSnapshot;
}

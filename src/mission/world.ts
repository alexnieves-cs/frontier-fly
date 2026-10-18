import type { Point } from './types';

/**
 * Site context for the mission map.
 *
 * The outdoor world generator (`src/world/outdoor`, owned by another workstream)
 * is not imported here. A future adapter can implement `WorldProvider` and
 * replace `mockWorldProvider`.
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

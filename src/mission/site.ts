import { rect } from './polygon';
import type { NoFlyZone, Point } from './types';
import type { WorldFootprint } from './world';

const SITE_NAMES = ['North warehouse row', 'River road yard', 'Civic annex', 'Practice field', 'Depot apron'] as const;

export interface SiteLayout {
  seed: number;
  siteName: string;
  geofence: Point[];
  footprints: WorldFootprint[];
  noFly: NoFlyZone;
  start: Point;
}

/** Deterministic practice site. Geometry is fixed; the seed only selects the site name. */
export function siteLayout(seed: number): SiteLayout {
  const siteName = SITE_NAMES[(seed >>> 0) % SITE_NAMES.length] ?? SITE_NAMES[0];
  const substation = rect(42, 18, 16, 12);
  return {
    seed: seed >>> 0,
    siteName,
    geofence: [
      { x: 2, y: 2 },
      { x: 88, y: 2 },
      { x: 92, y: 28 },
      { x: 84, y: 58 },
      { x: 6, y: 54 },
      { x: 2, y: 26 },
    ],
    footprints: [
      { id: 'road-south', kind: 'road', label: 'Access road', polygon: rect(0, 0, 96, 6), heightM: null },
      { id: 'bldg-a', kind: 'building', label: 'Warehouse A', polygon: rect(14, 16, 20, 12), heightM: 8 },
      { id: 'bldg-substation', kind: 'building', label: 'Substation', polygon: substation, heightM: 4 },
      { id: 'bldg-c', kind: 'building', label: 'Warehouse C', polygon: rect(62, 36, 16, 12), heightM: 9 },
      { id: 'water-east', kind: 'water', label: 'Retention pond', polygon: rect(96, 20, 16, 18), heightM: null },
      { id: 'canopy-north', kind: 'canopy', label: 'Tree line', polygon: rect(10, 48, 40, 4), heightM: 6 },
    ],
    noFly: {
      id: 'nf-substation',
      name: 'Substation',
      polygon: { points: substation },
      reason: 'Do not overfly the substation.',
    },
    start: { x: 10, y: 10 },
  };
}

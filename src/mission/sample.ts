import { siteLayout } from './site';
import type { Mission, MissionType, TargetSpec } from './types';

export function suggestedTargets(type: MissionType): TargetSpec[] {
  switch (type) {
    case 'find_objects':
      return [{ id: 'tgt-object', kind: 'object', label: 'High-visibility marker', minCount: 1, minConfidence: null }];
    case 'find_person':
      return [{ id: 'tgt-person', kind: 'person', label: 'Simulated missing person', minCount: 1, minConfidence: null }];
    case 'safety_sweep':
      return [{ id: 'tgt-clear', kind: 'clearance', label: 'Confirm the area is clear', minCount: 1, minConfidence: null }];
    case 'building_inspection':
      return [{ id: 'tgt-structure', kind: 'structure', label: 'Roof and facade', minCount: 1, minConfidence: null }];
    case 'composite':
      return [
        { id: 'tgt-person', kind: 'person', label: 'Simulated missing person', minCount: 1, minConfidence: null },
        { id: 'tgt-hazard', kind: 'hazard', label: 'Visible hazard', minCount: 1, minConfidence: null },
      ];
  }
}

/** A valid, seed-addressed practice mission on the mock site. */
export function sampleMission(seed: number): Mission {
  const site = siteLayout(seed);
  return {
    schemaVersion: 1,
    id: `msn-${site.seed}`,
    name: site.siteName,
    seed: site.seed,
    notes: 'Civilian search. The person is simulated. No real emergency traffic.',
    geofence: { points: site.geofence.map((point) => ({ ...point })) },
    noFlyZones: [{ ...site.noFly, polygon: { points: site.noFly.polygon.points.map((point) => ({ ...point })) } }],
    altitude: { minM: 3, maxM: 25 },
    missionType: 'find_person',
    targets: suggestedTargets('find_person'),
    requiredConfidence: 0.8,
    entryPolicy: 'ask',
    budgets: { batteryPct: 40, timeSec: 600 },
    startPose: { x: site.start.x, y: site.start.y, z: 6, yawRad: 0.6 },
  };
}

/** Empty geofence so an operator can draw. Not valid until the ring and start are set. */
export function blankMission(seed: number): Mission {
  const base = sampleMission(seed);
  return {
    ...base,
    id: `draft-${base.seed}`,
    name: 'Untitled search',
    notes: '',
    geofence: { points: [] },
    noFlyZones: [],
    missionType: 'safety_sweep',
    targets: suggestedTargets('safety_sweep'),
    entryPolicy: 'never',
    startPose: { x: 0, y: 0, z: base.startPose.z, yawRad: 0 },
  };
}

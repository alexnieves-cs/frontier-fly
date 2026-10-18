import { RNG } from '../shared/rng';
import type { Detection, DetectionSource } from './detections';
import { centroid, pointInRing, roundTo } from './polygon';
import { closestAlong, pathLength, planPath, pointAtDistance } from './path';
import type { Mission, TargetKind } from './types';
import type { WorldSnapshot } from './world';

const ENTRY_KINDS = new Set<TargetKind>(['person', 'structure', 'hazard']);

function clamp(n: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, n));
}

export const mockDetectionSource: DetectionSource = {
  id: 'mock-detections',
  catalog(mission: Mission, world: WorldSnapshot): Detection[] {
    const path = planPath(mission);
    const total = pathLength(path);
    const enterable = world.footprints.filter((footprint) => {
      if (footprint.kind !== 'building') return false;
      const center = centroid(footprint.polygon);
      return !mission.noFlyZones.some((zone) => zone.polygon.points.length >= 3 && pointInRing(center, zone.polygon.points));
    });
    return mission.targets.map((target, index) => {
      const rng = new RNG(mission.seed >>> 0, `det:${target.id}`);
      const floor = target.minConfidence ?? mission.requiredConfidence;
      const confidence = roundTo(clamp(floor + rng.range(0.02, 0.12), floor, 0.99), 2);
      const building = enterable.length > 0 ? enterable[index % enterable.length]! : null;
      const wantsEntry = ENTRY_KINDS.has(target.kind);
      let x = mission.startPose.x;
      let y = mission.startPose.y;
      let entryRequested = false;
      let structureId: string | null = null;
      if (building && wantsEntry) {
        const center = centroid(building.polygon);
        const hit = closestAlong(path, center);
        if (hit.gap <= 18) {
          x = roundTo(center.x, 2);
          y = roundTo(center.y, 2);
          entryRequested = true;
          structureId = building.id;
        } else {
          const at = pointAtDistance(path, hit.along);
          x = roundTo(at.x, 2);
          y = roundTo(at.y, 2);
        }
      } else {
        const at = pointAtDistance(path, (total * (index + 1)) / (mission.targets.length + 1));
        x = roundTo(at.x, 2);
        y = roundTo(at.y, 2);
      }
      return {
        id: `det-${target.id}`,
        tSec: 0,
        kind: target.kind,
        label: target.label,
        confidence,
        x,
        y,
        z: mission.startPose.z,
        structureId,
        entryRequested,
      };
    });
  },
};

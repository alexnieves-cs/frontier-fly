/** Mission-page adapters over the generated outdoor world. */
import { buildMissionWorld, missionForWorld, siteNameFor } from './autonomy/world';
import type { Mission } from './types';

export {
  buildingLabels,
  canopyFootprints,
  outdoorWorldProvider,
  siteNameFor,
  worldFootprints,
} from './autonomy/world';
export { createPerceptionDetectionSource, perceptionDetectionSource, SWEEP_AGL_M } from './perceptionSource';

/** The page's default mission: find the simulated person in this seed's woods, asking before entry. */
export function woodsSampleMission(seed: number): Mission {
  const world = buildMissionWorld({ seed: seed >>> 0 });
  return missionForWorld(world, {
    missionType: 'find_person',
    name: `${siteNameFor(seed)} search`,
    entryPolicy: 'ask',
    budgets: { batteryPct: 60, timeSec: 900 },
  });
}

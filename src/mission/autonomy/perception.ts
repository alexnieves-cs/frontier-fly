/* eslint-disable @typescript-eslint/no-unused-vars */
import type { ClearReport } from '../../perception/seenEmpty';
import type { ClearScope, LiveDetectionSource, MissionWorld, Observation } from './contracts';

// STARTER: sees nothing and never clears. The world workstream replaces it.
export function createPerceptionSource(world: MissionWorld): LiveDetectionSource {
  return {
    id: `perception-stub-${world.seed}`,
    observe(_pose, tSec): Observation {
      return { tSec, contacts: [] };
    },
    clearStatus(_scope: ClearScope): ClearReport {
      return { status: 'incomplete', seenEmpty: 0, occupied: 0, unseen: 1, total: 1 };
    },
    truthTargets() {
      return [];
    },
  };
}

import type { Mission, TargetKind } from './types';
import type { WorldSnapshot } from './world';

/**
 * A contact raised during a mission.
 *
 * Perception is not implemented in this workstream. `DetectionSource` is the
 * seam a detector can implement. The editor ships `mockDetectionSource`.
 */
export interface Detection {
  id: string;
  /** Seconds from launch. Zero until the session schedules the contact. */
  tSec: number;
  kind: TargetKind;
  label: string;
  confidence: number;
  x: number;
  y: number;
  z: number;
  structureId: string | null;
  /** True when confirming the contact means crossing into a structure. */
  entryRequested: boolean;
}

export interface DetectionSource {
  readonly id: string;
  catalog(mission: Mission, world: WorldSnapshot): Detection[];
}

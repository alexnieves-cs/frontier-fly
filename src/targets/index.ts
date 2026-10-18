export { CLIP_NAMES, HUMANOID, HUMANOID_SCALE } from './assets';
export { DECOY_LOOKS_LIKE, OBJECT_IDS, PERSON_SIZE } from './catalog';
export { frameOnTargets } from './frame';
export type { Frame } from './frame';
export { resolveScene, resolveTarget } from './motion';
export { placeIndoor, placeOutdoor } from './placement';
export { JOINT_NAMES, samplePersonMotion } from './skeleton';
export type { JointSample, PersonMotionSample } from './skeleton';
export { PERSON_STATES } from './types';
export type {
  OutdoorArea,
  PersonState,
  PlaceOptions,
  PlacedTarget,
  ResolvedTarget,
  TargetScene,
  Vec3,
} from './types';
export { placementIssues } from './validate';

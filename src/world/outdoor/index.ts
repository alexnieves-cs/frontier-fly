export { COLLIDER_BUDGET, OUTDOOR_LIDAR_MAX, PASS_RADIUS, REQUIRED_CLEARANCE } from './constants';
export { measureClearance, doorPose } from './clearance';
export { fingerprint } from './fingerprint';
export { generateOutdoor } from './generate';
export { initOutdoorPhysics, OutdoorPhysics } from './physics';
export { sampleHeight } from './terrain';
export type { Opening, OpeningKind, OutdoorParams, OutdoorWorld, TreeInstance } from './types';
export { entrancePaths, validateOutdoor } from './validate';

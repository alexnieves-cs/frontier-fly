import { DRONE_RADIUS } from '../../shared/constants';

/** World extent along both horizontal axes, in meters. */
export const WORLD_SIZE = 80;
/** Heightfield samples along one edge. */
export const TERRAIN_N = 65;

/**
 * Extra air the opening must leave around the airframe.
 * The sphere that must pass has radius DRONE_RADIUS + margin/2.
 */
export const CLEARANCE_MARGIN = 0.08;
export const PASS_RADIUS = DRONE_RADIUS + CLEARANCE_MARGIN / 2;
export const REQUIRED_CLEARANCE = PASS_RADIUS * 2;

export const DOOR_THICKNESS = 0.05;
export const COLLIDER_BUDGET = 900;
export const OUTDOOR_LIDAR_MAX = 16;
export const MAX_BUILDINGS = 4;

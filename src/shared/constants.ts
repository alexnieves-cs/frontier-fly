export const CELL = {
  EMPTY: 0,
  WALL: 1,
  DOOR: 2,
  SHAFT: 3,
} as const;

export const SURFACE = {
  DRYWALL: 0,
  CONCRETE: 1,
  WOOD: 2,
  GLASS: 3,
  METAL: 4,
} as const;

/** Observation contract version. v1 = 306 floats (the first shipped checkpoint). See src/sim/OBS_LAYOUT.md. */
export const OBS_VERSION = 2;
export const ACT_DIM = 4;

export const CELL_SIZE = 1;
export const FLOOR_HEIGHT = 3;
export const VOXEL = 0.5;
export const DRONE_RADIUS = 0.15;

export const MAX_SPEED_H = 2;
export const MAX_SPEED_V = 1;
export const MAX_YAW_RATE = Math.PI / 2;

export const KINEMATIC_TAU = 0.15;
export const PHYSICS_DT = 1 / 400;
export const CONTROL_DT = 1 / 50;
export const POLICY_DT = 1 / 20;
export const PHYSICS_SUBSTEPS = 20;
export const CONTROL_EVERY = 8;

export const LIDAR_H = 32;
export const LIDAR_TILT = 8;
export const LIDAR_MAX = 8;
export const MAP_SIZE = 16;
export const MAP_CELLS = MAP_SIZE * MAP_SIZE;
/** Coarse egocentric map: 16x16 cells, each COARSE_VOXELS x COARSE_VOXELS fine voxels (2 m). */
export const COARSE_VOXELS = 4;
/** Frontier/navigation feature block length. */
export const NAV_FEATURES = 12;

/**
 * Observation v2 offsets (see src/sim/OBS_LAYOUT.md). Indices 0–47 keep their
 * v1 meaning so lidar-only code (sim/heuristic.ts) reads the same slots.
 */
export const OBS = {
  lidar: 0,
  tilt: 32,
  vel: 40,
  yawRate: 43,
  sinYaw: 44,
  cosYaw: 45,
  alt: 46,
  ceil: 47,
  roll: 48,
  pitch: 49,
  floor: 50,
  time: 51,
  /** Navigation block: 12 floats from the sensor-only seen-free map. */
  nav: 52,
  wpDir: 52,
  pathLen: 54,
  hasFrontier: 55,
  clusterDir: 56,
  clusterDist: 58,
  clusterMass: 59,
  stall: 60,
  bfsTruncated: 61,
  frontierTotal: 62,
  frontierDist: 63,
  localVisited: 64,
  localFrontier: 64 + MAP_CELLS,
  coarseVisited: 64 + 2 * MAP_CELLS,
  coarseFrontier: 64 + 3 * MAP_CELLS,
  end: 64 + 4 * MAP_CELLS,
} as const;

export const OBS_DIM = OBS.end;

export const GAMMA = 0.99;
export const GAE_LAMBDA = 0.95;
export const CLIP_EPS = 0.2;
export const ENTROPY_COEF = 0.01;
export const VALUE_COEF = 0.5;
export const LEARNING_RATE = 3e-4;
export const PPO_EPOCHS = 4;
export const MINIBATCH = 512;

export const COVERAGE_SUCCESS = 0.95;
export const SUCCESS_BONUS = 20;
export const MAX_HARD_COLLISIONS = 3;
export const STALL_STEPS = 200;

export const REWARD = {
  newVoxel: 1,
  newRoom: 5,
  newFloor: 10,
  collision: -5,
  contact: -0.1,
  smooth: 0.01,
  effort: 0.001,
  time: 0.005,
  /** Steps without a new voxel before the stuck penalty starts. */
  stuckGrace: 40,
  /** Steps past the grace window over which the stuck penalty ramps to its max. */
  stuckRamp: 160,
  /** Max per-step stuck penalty (reached after grace + ramp steps without progress). */
  stuck: 0.05,
  /** Window (steps) for the net-displacement check. */
  stillWindow: 20,
  /** Net displacement (m) over the window below which the drone counts as loitering. */
  stillDist: 0.3,
  /** Per-step penalty while loitering without covering. */
  still: 0.02,
  /** Commands below this magnitude never count as a sign flip. */
  flipDeadband: 0.15,
  /** Penalty per sign flip of the lateral (strafe) or yaw command. */
  flip: 0.01,
} as const;

export const HIDDEN = [256, 128] as const;

export const GRAVITY = 9.81;

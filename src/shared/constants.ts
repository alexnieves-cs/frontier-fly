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

export const OBS_DIM = 306;
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
export const MAP_CELLS = MAP_SIZE * MAP_SIZE - 2;

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

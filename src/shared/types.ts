export type Cell = 0 | 1 | 2 | 3;

export interface Room {
  id: number;
  x: number;
  z: number;
  w: number;
  d: number;
}

export interface FloorGrid {
  w: number;
  d: number;
  cells: Uint8Array;
  rooms: Room[];
  /** Per-cell surface class: 0 drywall, 1 concrete, 2 wood, 3 glass, 4 metal. */
  surfaces?: Uint8Array;
}

export type PropKind = 'table' | 'chair' | 'shelf' | 'plant' | 'sofa';

/** One furniture piece, in renderer convention: x and z horizontal (z = sim y), y up, rot about +y. */
export interface PropPlacement {
  kind: PropKind;
  x: number;
  y: number;
  z: number;
  rot: number;
  floor: number;
}

/**
 * Solid furniture part in sim space (x, y horizontal, z up). The box is
 * oriented by `yaw` about +z: its local x axis points along (cos yaw, sin yaw).
 * hx/hy/hz are half extents in that local frame.
 */
export interface FurnitureBox {
  floor: number;
  /** Index into FurniturePlan.props. */
  prop: number;
  cx: number;
  cy: number;
  cz: number;
  hx: number;
  hy: number;
  hz: number;
  yaw: number;
  /** SURFACE class reported to the lidar. */
  surface: number;
}

/** Seeded furniture layout shared by physics, the coverage denominator, the teacher, and the renderer. */
export interface FurniturePlan {
  props: PropPlacement[];
  boxes: FurnitureBox[];
}

export interface BuildingSpec {
  seed: number;
  cellSize: number;
  floorHeight: number;
  floors: FloorGrid[];
  spawn: [number, number, number];
  /**
   * Coverage denominator: 0.5 m voxels the drone center can reach at flight
   * altitude (floor + 1 m). See computeFlightMap in src/gen/validate.ts.
   */
  reachableVoxels: number;
  /** Solid furniture. Absent on hand-written fixtures and older JSON: those buildings are empty. */
  furniture?: FurniturePlan;
}

export interface GenParams {
  seed: number;
  floors: number;
  w: number;
  d: number;
  stage: number;
}

export interface StepInfo {
  coverage: number;
  collisions: number;
  floor: number;
  newRoom: boolean;
  newFloor: boolean;
  success: boolean;
  /** Progress penalties subtracted from this step's reward (see sim/reward.ts ProgressTracker). */
  penalty?: { stuck: number; still: number; flip: number; total: number };
}

export interface StepResult {
  obs: Float32Array;
  reward: number;
  done: boolean;
  truncated: boolean;
  info: StepInfo;
}

export interface DroneState {
  pos: [number, number, number];
  vel: [number, number, number];
  yaw: number;
  t: number;
  /** Body-to-world quaternion, xyzw (Three.js order). */
  quat: [number, number, number, number];
  motors: Float32Array;
  roll: number;
  pitch: number;
}

export interface DynParams {
  mass: number;
  inertia: [number, number, number];
  arm: number;
  kf: number;
  km: number;
  motorTau: number;
  dragLin: [number, number, number];
  dragQuad: [number, number, number];
}

export interface NoiseParams {
  gyroSigma: number;
  accelSigma: number;
  biasRW: number;
  baroSigma: number;
  lidarSigma: number;
  lidarDropout: number;
  latencyTicks: number;
  /** Multiplier on IMU bias random walk. 1.5 in low light when the stretch flag is on. */
  lowLightImu: number;
}

export interface Dynamics {
  reset(s: DroneState, p: DynParams): void;
  step(cmd: Float32Array, dt: number): DroneState;
}

export type SensorMode = 'rgb' | 'ir' | 'depth';

export interface LightingState {
  timeOfDay: number;
  interiorLightsOn: Uint8Array;
  flashlightOn: boolean;
  sensorMode: SensorMode;
}

export interface EpisodeConfig {
  stage: number;
  dynamics: 'kinematic' | 'quad';
  dyn: DynParams;
  noise: NoiseParams;
  lighting: LightingState;
  /** Horizontal wind in world x/y (m/s). Applied as a velocity bias in kinematic mode and a force in quad mode. */
  wind: [number, number];
  /** Optional overrides used by tests and the curriculum. */
  maxSteps?: number;
  stallSteps?: number;
  /** Hard collisions before the episode ends. Defaults to MAX_HARD_COLLISIONS. */
  maxCollisions?: number;
}

export interface Env {
  reset(spec: BuildingSpec, seed: number, cfg?: EpisodeConfig): Float32Array;
  step(action: Float32Array): StepResult;
  snapshot(): DroneState;
}

export type CameraMode = 'fpv' | 'chase' | 'overview';
export type QualityPreset = 'low' | 'high';

export interface Renderer {
  load(spec: BuildingSpec): void;
  update(s: DroneState, l: LightingState): void;
  setCamera(m: CameraMode): void;
  setQuality(q: QualityPreset): void;
  dispose(): void;
}

export interface Policy {
  act(
    obs: Float32Array,
    n: number,
  ): { action: Float32Array; logp: Float32Array; value: Float32Array };
  getWeights(): ArrayBuffer[];
  setWeights(w: ArrayBuffer[]): void;
}

export interface SkySample {
  sunDir: [number, number, number];
  sunIntensity: number;
  sunColor: [number, number, number];
  ambient: number;
  skyColor: [number, number, number];
  night: number;
}

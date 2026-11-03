import type { BatteryConfig } from '../mission/autonomy/battery';
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
  /** Per-cell surface class: 0 drywall, 1 concrete, 2 wood, 3 glass, 4 metal, 5 shut door, 6 mirror. */
  surfaces?: Uint8Array;
}

/** `door` is an ajar interior door leaf placed by gen/hazards.ts, not by the furniture planner. */
export type PropKind = 'table' | 'chair' | 'shelf' | 'plant' | 'sofa' | 'door';

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
  /** Shut and ajar doors, glass rooms, mirrors and smoke (gen/hazards.ts). Absent: none of them. */
  hazards?: HazardLayout;
}

/** What gen/hazards.ts asks for. Shares are 0..1; a missing field adds none of that hazard. */
export interface HazardParams {
  /** Share of interior doorways shut (never below HAZARD_MIN_REACHABLE of the building's reachable cells). */
  closedDoors?: number;
  /** Share of the doorways left open that get a leaf swung part-way into the opening. */
  ajarDoors?: number;
  /** Share of rooms walled in glass (the interior side; the outer wall keeps its windows). */
  glassRooms?: number;
  /** Mirror panels per floor, on interior walls. */
  mirrors?: number;
  /** Share of rooms filled with smoke. */
  smokeRooms?: number;
  /** Visibility range in the smoke, meters, drawn once per building from [min, max]. */
  smokeVisibility?: [number, number];
}

/** A shut doorway: its leaf cells (now walls with SURFACE.DOOR) on one floor. */
export interface ClosedDoor {
  floor: number;
  /** Cell indices (z * w + x) of the leaf. */
  cells: number[];
}

/** Smoke: per-floor cell masks (1 = smoke) and the visibility in it. */
export interface SmokeField {
  /** Meteorological visibility, meters: contrast falls to 2% over this distance. */
  visibility: number;
  cells: Uint8Array[];
}

/** The hazards a building was given, recorded on the spec so every sensor model and renderer agrees. */
export interface HazardLayout {
  params: HazardParams;
  closedDoors: ClosedDoor[];
  /** Ajar leaves added to spec.furniture (prop kind 'door'). */
  ajarDoors: number;
  glassRooms: number;
  mirrors: number;
  smoke: SmokeField | null;
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
  /** Speed into the surface of a contact that started this step, m/s (0 without one). */
  impactSpeed: number;
  /** The crash rule ended the flight: a contact started faster than the crash speed. */
  crashed: boolean;
  floor: number;
  newRoom: boolean;
  newFloor: boolean;
  success: boolean;
  /** Electrical energy used so far by the airframe flying this track (mission/autonomy/battery.ts; FrontierBench v2 keeps its own model), Wh. */
  energyWh: number;
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
  /** Horizontal speed setpoint cap, m/s (default MAX_SPEED_H). */
  setSpeedLimit?(speedH: number): void;
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
  /** Impact speed into a surface that counts as a crash and ends the episode, m/s. Defaults to CRASH_SPEED. */
  crashSpeed?: number;
  /** Power model the reported energyWh is integrated with. Defaults to DEFAULT_BATTERY (shared/airframe.ts). */
  energyModel?: BatteryConfig;
  /**
   * Horizontal speed a full action asks for, m/s. Defaults to MAX_SPEED_H
   * (2 m/s), which every shipped checkpoint was trained at.
   */
  speedH?: number;
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

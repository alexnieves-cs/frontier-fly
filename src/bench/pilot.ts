/**
 * What a FrontierBench pilot sees and returns. A pilot gets the observation
 * vector (OBS_LAYOUT.md) and the sensor frame: its own pose, lidar, the voxels
 * it has seen and flown, and its measured velocity. It never sees the floor
 * plan. It returns the env's action: forward, left, up and yaw rate, each in
 * [-1, 1], scaled by the task's speed cap.
 */
import type { SensorFrame } from '../sim/sensorPlanner';

export interface BenchContext {
  /** Speed a full horizontal action asks for this episode, m/s. */
  speedH: number;
  /** Policy steps in this episode (20 per second). */
  maxSteps: number;
  dynamics: 'kinematic' | 'quad';
}

export interface BenchPilot {
  /** A short name for tables. */
  readonly name: string;
  reset(ctx: BenchContext): void;
  act(obs: Float32Array, frame: SensorFrame): Float32Array;
}

/** A pilot module default-exports one of these. */
export type BenchPilotFactory = () => BenchPilot | Promise<BenchPilot>;

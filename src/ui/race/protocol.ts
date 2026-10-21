import type { BuildingSpec } from '../../shared/types';
import type { EndReason, RacePilot, RaceSetup } from './pilots';

/** Messages between the race page and src/ui/race/race.worker.ts. */

export type ToWorker =
  | { type: 'init'; checkpointUrl: string }
  | { type: 'start'; setup: RaceSetup; rate: number }
  /** Policy steps per second of wall time; 0 runs as fast as the worker can. */
  | { type: 'rate'; rate: number }
  | { type: 'pause'; paused: boolean }
  | { type: 'inspect'; lane: number };

export interface LaneFrame {
  pos: [number, number, number];
  yaw: number;
  coverage: number;
  collisions: number;
  steps: number;
  floor: number;
  reason: EndReason;
  action: [number, number, number, number];
}

export interface InspectFrame {
  lane: number;
  pilot: RacePilot;
  obs: Float32Array;
  action: Float32Array;
  /** Command shift when each input block is blanked (src/ui/inspector/influence.ts), policy lanes only. */
  influence: Float32Array | null;
  /** What the shipped policy would command from this observation, for lanes flown by something else. */
  policyAction: Float32Array | null;
}

export type FromWorker =
  | { type: 'ready'; policy: boolean; note: string }
  | { type: 'error'; message: string }
  | { type: 'started'; setup: RaceSetup; spec: BuildingSpec; maxSteps: number; maxCollisions: number; stepsPerSecond: number }
  | {
      type: 'frame';
      step: number;
      lanes: LaneFrame[];
      /** Per lane: visited voxels then seen-free voxels, both in the env's grid layout. */
      visited: Uint8Array[];
      seen: Uint8Array[];
      inspect: InspectFrame | null;
      finished: boolean;
      /** Measured policy steps per second of wall time, per lane. */
      throughput: number;
    };

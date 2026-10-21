import { FLOOR_HEIGHT, LIDAR_H, LIDAR_MAX, MAX_SPEED_H, OBS } from '../shared/constants';
import { Climber, type ClimberOptions } from '../sim/climber';
import type { ExplorerEnv } from '../sim/env';
import {
  commandToAction,
  guardAction,
  OccupancyMap,
  type BrakingModel,
  SensorPlanner,
  type SensorFrame,
  type SensorPlannerOptions,
} from '../sim/sensorPlanner';

const lidarScratch = new Float64Array(LIDAR_H);

/** World-frame horizontal velocity from the observation's IMU estimate (body frame, normalized). */
export function measuredVelocity(obs: Float32Array, yaw: number): [number, number] {
  const f = obs[OBS.vel]! * MAX_SPEED_H;
  const l = obs[OBS.vel + 1]! * MAX_SPEED_H;
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  return [cy * f - sy * l, sy * f + cy * l];
}

/** The sensor frame a pilot in the coverage env may use: its pose, lidar, and its own maps. */
export function sensorFrame(env: ExplorerEnv, obs: Float32Array): SensorFrame {
  const spec = env.building!;
  const s = env.snapshot();
  for (let i = 0; i < LIDAR_H; i++) lidarScratch[i] = obs[OBS.lidar + i]! * LIDAR_MAX;
  return {
    seen: env.seenGrid,
    visited: env.visitedGrid,
    vw: spec.floors[0]!.w * 2,
    vd: spec.floors[0]!.d * 2,
    floors: spec.floors.length,
    floorHeight: spec.floorHeight,
    pos: s.pos,
    yaw: s.yaw,
    lidar: lidarScratch,
    alt: obs[OBS.alt]! * FLOOR_HEIGHT,
    vel: measuredVelocity(obs, s.yaw),
  };
}

/** Classical sensor-only explorer as a coverage-env pilot. */
export function classicalPilot(opts: SensorPlannerOptions = {}): { reset(): void; act(obs: Float32Array, env: ExplorerEnv): Float32Array } {
  const planner = new SensorPlanner(opts);
  const out = new Float32Array(4);
  let occ: OccupancyMap | null = null;
  let seen: Uint8Array | null = null;
  return {
    reset: () => planner.reset(),
    act(obs, env) {
      const f = sensorFrame(env, obs);
      // A new episode hands the env a new seen grid; start a fresh occupancy grid with it.
      if (!occ || env.seenGrid !== seen || occ.occupied.length !== f.seen.length) {
        seen = env.seenGrid;
        occ = new OccupancyMap(f.vw, f.vd, f.floors);
      }
      const floor = Math.max(0, Math.min(f.floors - 1, Math.floor(f.pos[2] / f.floorHeight)));
      occ.mark(floor, f.pos[0], f.pos[1], f.yaw, f.lidar);
      f.occupied = occ.occupied;
      return commandToAction(planner.act(f), f.yaw, out).slice();
    },
  };
}

/** Sensor-only planner that takes shafts between floors (sim/climber.ts) as a coverage-env pilot. */
export function climbingPilot(opts: SensorPlannerOptions = {}, climb: ClimberOptions = {}): { reset(): void; act(obs: Float32Array, env: ExplorerEnv): Float32Array; climber: Climber } {
  const climber = new Climber(new SensorPlanner(opts), climb);
  const out = new Float32Array(4);
  let occ: OccupancyMap | null = null;
  let seen: Uint8Array | null = null;
  return {
    climber,
    reset: () => climber.reset(),
    act(obs, env) {
      const f = sensorFrame(env, obs);
      if (!occ || env.seenGrid !== seen || occ.occupied.length !== f.seen.length) {
        seen = env.seenGrid;
        occ = new OccupancyMap(f.vw, f.vd, f.floors);
      }
      const floor = Math.max(0, Math.min(f.floors - 1, Math.floor(f.pos[2] / f.floorHeight)));
      occ.mark(floor, f.pos[0], f.pos[1], f.yaw, f.lidar);
      f.occupied = occ.occupied;
      return commandToAction(climber.act(f, obs[OBS.ceil]! * FLOOR_HEIGHT), f.yaw, out).slice();
    },
  };
}

/**
 * The policy guard (sim/sensorPlanner.ts guardAction) for a coverage-env
 * pilot: it reads the drone's height and yaw and the horizontal lidar in the
 * same observation.
 */
export function guardedAction(action: Float32Array, obs: Float32Array, env: ExplorerEnv, braking?: BrakingModel): Float32Array {
  const s = env.snapshot();
  const spec = env.building!;
  const floor = Math.max(0, Math.min(spec.floors.length - 1, Math.floor(s.pos[2] / spec.floorHeight)));
  for (let i = 0; i < LIDAR_H; i++) lidarScratch[i] = obs[OBS.lidar + i]! * LIDAR_MAX;
  const brake = braking ? { model: braking, vel: measuredVelocity(obs, s.yaw) } : undefined;
  return guardAction(action, s.pos[2] - floor * spec.floorHeight, s.yaw, lidarScratch, undefined, undefined, brake);
}

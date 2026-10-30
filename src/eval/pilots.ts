import { FLOOR_HEIGHT, LIDAR_H, LIDAR_MAX, OBS } from '../shared/constants';
import { Climber, type ClimberOptions } from '../sim/climber';
import type { ExplorerEnv } from '../sim/env';
import { commandToAction, measuredVelocity, sensorFrame, SensorPlanner, type SensorPlannerOptions } from '../sim/sensorPlanner';
import { guardAction, type BrakingModel } from '../sim/lidarSafety';
import { OccupancyMap } from '../sim/sensorMap';

const lidarScratch = new Float64Array(LIDAR_H);

/** Classical sensor-only explorer as a coverage-env pilot. */
export function classicalPilot(opts: SensorPlannerOptions = {}): { reset(): void; act(obs: Float32Array, env: ExplorerEnv): Float32Array; planner: SensorPlanner } {
  const planner = new SensorPlanner(opts);
  const out = new Float32Array(4);
  let occ: OccupancyMap | null = null;
  let seen: Uint8Array | null = null;
  return {
    planner,
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
      return commandToAction(planner.act(f), f.yaw, out, env.speedLimit).slice();
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
      return commandToAction(climber.act(f, obs[OBS.ceil]! * FLOOR_HEIGHT), f.yaw, out, env.speedLimit).slice();
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
  return guardAction(action, s.pos[2] - floor * spec.floorHeight, s.yaw, lidarScratch, undefined, undefined, brake, env.speedLimit);
}

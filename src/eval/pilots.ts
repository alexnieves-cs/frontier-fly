import { FLOOR_HEIGHT, LIDAR_H, LIDAR_MAX, OBS } from '../shared/constants';
import type { ExplorerEnv } from '../sim/env';
import {
  commandToAction,
  guardAction,
  OccupancyMap,
  SensorPlanner,
  type SensorFrame,
  type SensorPlannerOptions,
} from '../sim/sensorPlanner';

const lidarScratch = new Float64Array(LIDAR_H);

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

/**
 * The policy guard (sim/sensorPlanner.ts guardAction) for a coverage-env
 * pilot: it reads the drone's height and yaw and the horizontal lidar in the
 * same observation.
 */
export function guardedAction(action: Float32Array, obs: Float32Array, env: ExplorerEnv): Float32Array {
  const s = env.snapshot();
  const spec = env.building!;
  const floor = Math.max(0, Math.min(spec.floors.length - 1, Math.floor(s.pos[2] / spec.floorHeight)));
  for (let i = 0; i < LIDAR_H; i++) lidarScratch[i] = obs[OBS.lidar + i]! * LIDAR_MAX;
  return guardAction(action, s.pos[2] - floor * spec.floorHeight, s.yaw, lidarScratch);
}

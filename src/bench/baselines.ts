/** The pilots the leaderboard starts with, written against the bench pilot API only. */
import { FLOOR_HEIGHT, OBS } from '../shared/constants';
import { Climber } from '../sim/climber';
import { heuristicAction } from '../sim/heuristic';
import { commandToAction, guardAction, OccupancyMap, QUAD_BRAKING, SensorPlanner, type SensorFrame, type SensorPlannerOptions } from '../sim/sensorPlanner';
import { waypointAction } from '../sim/waypointPilot';
import type { BenchContext, BenchPilot } from './pilot';

/**
 * The sensor-only planner, with its own lidar occupancy map. On the
 * quadrotor it brakes with the fitted braking model, and on buildings with
 * more than one floor it takes shafts (sim/climber.ts).
 */
export function plannerPilot(name: string, opts: SensorPlannerOptions = {}): BenchPilot {
  const out = new Float32Array(4);
  let ctx: BenchContext = { speedH: 2, maxSteps: 2400, dynamics: 'kinematic' };
  let planner = new SensorPlanner(opts);
  let climber: Climber | null = null;
  let occ: OccupancyMap | null = null;
  return {
    name,
    reset(c) {
      ctx = c;
      planner = new SensorPlanner({ speed: c.speedH, ...(c.dynamics === 'quad' ? { braking: QUAD_BRAKING } : {}), ...opts });
      climber = null;
      occ = null;
    },
    act(obs: Float32Array, f: SensorFrame) {
      if (!occ || occ.occupied.length !== f.seen.length) occ = new OccupancyMap(f.vw, f.vd, f.floors);
      const floor = Math.max(0, Math.min(f.floors - 1, Math.floor(f.pos[2] / f.floorHeight)));
      occ.mark(floor, f.pos[0], f.pos[1], f.yaw, f.lidar);
      const frame = { ...f, occupied: occ.occupied };
      if (f.floors > 1) {
        climber ??= new Climber(planner, { floorSteps: (0.8 * ctx.maxSteps) / f.floors });
        return commandToAction(climber.act(frame, obs[OBS.ceil]! * FLOOR_HEIGHT), f.yaw, out, ctx.speedH).slice();
      }
      return commandToAction(planner.act(frame), f.yaw, out, ctx.speedH).slice();
    },
  };
}

/** A pilot that maps each observation to an action and keeps no state. */
export function statelessPilot(name: string, act: (obs: Float32Array) => Float32Array): BenchPilot {
  return { name, reset() {}, act: (obs) => act(obs) };
}

/**
 * A learned policy, optionally under the lidar guard (the planner's velocity
 * filter plus a hold at 1.15 m; the braking filter on the quadrotor).
 */
export function policyPilot(name: string, policy: (obs: Float32Array) => Float32Array, guarded: boolean): BenchPilot {
  let ctx: BenchContext = { speedH: 2, maxSteps: 2400, dynamics: 'kinematic' };
  return {
    name,
    reset(c) {
      ctx = c;
    },
    act(obs, f) {
      const a = policy(obs);
      if (!guarded) return a;
      const floor = Math.max(0, Math.min(f.floors - 1, Math.floor(f.pos[2] / f.floorHeight)));
      const brake = ctx.dynamics === 'quad' ? { model: QUAD_BRAKING, vel: f.vel ?? null } : undefined;
      return guardAction(a, f.pos[2] - floor * f.floorHeight, f.yaw, f.lidar, undefined, undefined, brake, ctx.speedH);
    },
  };
}

export const BASELINES: Record<string, () => BenchPilot> = {
  planner: () => plannerPilot('Sensor-only planner'),
  'planner-momentum': () => plannerPilot('Sensor-only planner + momentum', { momentum: 1 }),
  waypoint: () => statelessPilot('Frontier waypoint follower', waypointAction),
  heuristic: () => statelessPilot('Lidar wall-follower', heuristicAction),
};

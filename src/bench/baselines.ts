/** The pilots the leaderboard starts with, written against the bench pilot API only. */
import { OBS } from '../shared/constants';
import { Climber } from '../sim/climber';
import { heuristicAction } from '../sim/heuristic';
import { commandToAction, SensorPlanner, type SensorFrame, type SensorPlannerOptions } from '../sim/sensorPlanner';
import { guardAction, QUAD_BRAKING } from '../sim/lidarSafety';
import { OccupancyMap } from '../sim/sensorMap';
import { waypointAction } from '../sim/waypointPilot';
import type { Explorer } from '../sim/explorers/common';
import { NearestFrontierExplorer } from '../sim/explorers/nearestFrontier';
import { NbvpExplorer } from '../sim/explorers/nbvp';
import { BoustrophedonExplorer } from '../sim/explorers/boustrophedon';
import type { BenchContext, BenchPilot } from './pilot';

/**
 * The return supervisor every explorer pilot shares on return tasks (BenchContext.home), so the
 * tasks compare exploration, not homing code. Every `every` steps it plans the known-free path back
 * to take-off; once that path at `speed` × the task speed, plus `reserve` seconds, no longer fits in
 * the time left, it stops the explorer and flies the sensor-only planner home.
 */
export const HOME_RETURN = { speed: 0.9, reserve: 5, every: 10 } as const;

/**
 * A pilot over any exploration strategy on the sensor frame, with its own
 * lidar occupancy map. On buildings with more than one floor it takes shafts
 * (sim/climber.ts) between floors.
 */
export function explorerPilot(name: string, make: (ctx: BenchContext) => Explorer): BenchPilot & { climberRef(): Climber | null } {
  const out = new Float32Array(4);
  let ctx: BenchContext = { speedH: 2, maxSteps: 2400, dynamics: 'kinematic' };
  let explorer: Explorer = make(ctx);
  let climber: Climber | null = null;
  let occ: OccupancyMap | null = null;
  let homer: SensorPlanner | null = null;
  let steps = 0;
  return {
    name,
    /** The floor switcher of the current flight, once the building has more than one floor (diagnostics). */
    climberRef: () => climber,
    reset(c) {
      ctx = c;
      explorer = make(c);
      climber = null;
      occ = null;
      steps = 0;
      homer = c.home ? new SensorPlanner({ speed: c.speedH, ...(c.dynamics === 'quad' ? { braking: QUAD_BRAKING, tracker: {}, brakingMargin: VIEW_QUAD_MARGIN } : {}) }) : null;
    },
    act(obs: Float32Array, f: SensorFrame) {
      if (!occ || occ.occupied.length !== f.seen.length) occ = new OccupancyMap(f.vw, f.vd, f.floors);
      const floor = Math.max(0, Math.min(f.floors - 1, Math.floor(f.pos[2] / f.floorHeight)));
      occ.mark(floor, f.pos[0], f.pos[1], f.yaw, f.lidar);
      const frame = { ...f, occupied: occ.occupied };
      if (homer) {
        if (!homer.homing && steps % HOME_RETURN.every === 0) {
          const left = (ctx.maxSteps - steps) / 20;
          const d = homer.homeDistance(frame);
          // No known path home (a map glitch): keep exploring and ask again next time.
          if (d < Infinity && d / (HOME_RETURN.speed * ctx.speedH) + HOME_RETURN.reserve >= left) homer.homing = true;
        }
        steps += 1;
        if (homer.homing) return commandToAction(homer.act(frame), f.yaw, out, ctx.speedH).slice();
      }
      if (f.floors > 1) {
        climber ??= new Climber(explorer, { floorSteps: (0.8 * ctx.maxSteps) / f.floors, searcher: () => shaftSearcher(ctx) });
        return commandToAction(climber.act(frame, obs[OBS.ceil]! * f.floorHeight), f.yaw, out, ctx.speedH).slice();
      }
      const cmd = explorer.act(frame);
      // An explorer that has finished goes home at once.
      if (homer && !cmd.exploring) homer.homing = true;
      return commandToAction(cmd, f.yaw, out, ctx.speedH).slice();
    },
  };
}

/**
 * The shaft search every explorer pilot shares on multi-floor buildings (sim/climber.ts `searcher`):
 * the plain sensor-only planner sweeping 1 m lanes of unflown voxels, with the quad tracker and braking on the
 * quadrotor, so the tasks compare exploration, not stair finding.
 */
function shaftSearcher(ctx: BenchContext): Explorer {
  return new SensorPlanner({ speed: ctx.speedH, lanes: true, ...(ctx.dynamics === 'quad' ? { braking: QUAD_BRAKING, tracker: {} } : {}) });
}

/**
 * The sensor-only planner. On the quadrotor it brakes with the fitted braking
 * model, and on buildings with more than one floor it takes shafts.
 */
export function plannerPilot(name: string, opts: SensorPlannerOptions = {}): BenchPilot {
  return explorerPilot(name, (c) => new SensorPlanner({ speed: c.speedH, ...(c.dynamics === 'quad' ? { braking: QUAD_BRAKING } : {}), ...opts }));
}

/** Airframe options a published baseline flies with: cruise at the cap, and the braking filter on the quadrotor. */
const airframe = (c: BenchContext) => ({ speed: c.speedH, ...(c.dynamics === 'quad' ? { braking: QUAD_BRAKING } : {}) });

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

/** Braking margin of the view-first planner on the quadrotor, meters. */
export const VIEW_QUAD_MARGIN = 0.3;

/** View-first options on the quadrotor: aim a 90° forward camera, yawing at up to 0.8 rad/s. */
export const VIEW_LOOK = { look: Math.PI / 2, lookYawRate: 0.8 } as const;

export const BASELINES: Record<string, () => BenchPilot> = {
  planner: () => plannerPilot('Sensor-only planner'),
  'planner-momentum': () => plannerPilot('Sensor-only planner + momentum', { momentum: 1 }),
  // The quad-aware planner the report flies on the quadrotor: the trajectory tracker (sim/tracker.ts)
  // shapes its command there; on the kinematic airframe it is the plain planner.
  'planner-tracker': () => explorerPilot('Sensor-only planner + quad tracker', (c) => new SensorPlanner({ speed: c.speedH, ...(c.dynamics === 'quad' ? { braking: QUAD_BRAKING, tracker: {} } : {}) })),
  // View-first planner: goals are voxels the drone has not yet had in lidar line of sight within 3 m
  // (the footprint metric's radius), then it sweeps unflown voxels with the time left. On the
  // quadrotor it flies the tracker and keeps 0.30 m of braking margin (0.25 default), picked on the
  // practice block 90501-90600 (docs/VIEW_FIRST.md): view goals mean longer legs at cruise.
  // On the quadrotor it also aims a forward camera (VIEW_LOOK) at what it has not had in view, turning
  // at up to 0.8 rad/s (the tracker's own yaw cap is 0.3), picked on FrontierBench Hard dev 92001+.
  // Its quadrotor safety filter is map-backed (mapFill): a lidar ray with no return brakes for the walls and
  // unknown space its own map has along it, and a lidar the map keeps contradicting slows it to 0.6x
  // (degraded-sensor track, round 4, chosen on dev 140001+; no change on the healthy tracks' dev splits).
  'planner-view': () =>
    explorerPilot('Sensor-only planner, view first', (c) =>
      new SensorPlanner({ speed: c.speedH, ...(c.dynamics === 'quad' ? { view: VIEW_LOOK, braking: QUAD_BRAKING, tracker: {}, brakingMargin: VIEW_QUAD_MARGIN, mapFill: true } : { view: {} }) }),
    ),
  waypoint: () => statelessPilot('Frontier waypoint follower', waypointAction),
  heuristic: () => statelessPilot('Lidar wall-follower', heuristicAction),
  // Published exploration methods (src/sim/explorers), on the same maps, flight layer and safety filter.
  yamauchi: () => explorerPilot('Nearest frontier (Yamauchi 1997)', (c) => new NearestFrontierExplorer(airframe(c))),
  nbvp: () => explorerPilot('Next-best-view RRT (NBVP, Bircher et al. 2016)', (c) => new NbvpExplorer(airframe(c))),
  boustrophedon: () => explorerPilot('Boustrophedon coverage (BA*, Viet et al. 2013)', (c) => new BoustrophedonExplorer(airframe(c))),
};

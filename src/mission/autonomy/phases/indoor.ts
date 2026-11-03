import type { LightingState } from '../../../shared/types';
import { nightFactor } from '../../../sim/lighting';
import { FLASH_HALF } from '../../../perception/model';
import type { V3 } from '../contracts';
import { FlyThrough } from '../planner/flythrough';
import {
  FrontierExplorer,
  IndoorSensing,
  IndoorVisibility,
  PolicyExplorer,
  SensorExplorer,
  type LidarWorld,
  sensorNav,
  shellLidar,
  toLocal,
  toWorld,
} from '../planner/indoor';
import { ENTRY_SIGMA_XY, ENTRY_SIGMA_YAW, Localizer } from '../localizer';
import { QuadVehicle } from '../quadVehicle';
import { type VehicleCommand } from '../vehicle';
import {
  type Job,
  MAX_EXPLORE_SEC,
  POLICY_MAX_COLLISIONS,
  POLICY_MAX_SEC,
  POLICY_STALL_SEC,
  WRAP_UP_SEC,
  type Waypoint,
  fmt,
} from './shared';
import type { MissionCore } from './core';
import type { Pilot } from './pilot';

/** Where the indoor phase hands over: a last look at an open contact, the clear call, the search, or home. */
export interface IndoorExits {
  /** Start the last re-view of an open contact in this building; false when there is none. */
  lastLook(buildingId: string): boolean;
  declareIfClear(scope: string): boolean;
  continueMission(): void;
  startReturnOutside(): void;
  startLand(reason: string): void;
}

/**
 * Inside a building: policy or sensor-only exploration, re-views of open tracks, and the way back out.
 * Owns which explorer flew (for the record) and the running policy explorer.
 */
export class IndoorPhase {
  /** The record credits the policy unless a fallback handed a building to the frontier planner. */
  policyClean = false;
  private policyFell = false;
  frontierRan = false;
  /** The running policy explorer, so a confirmation can hand control back and an exit can free it. */
  private policyRun: { resume: () => void; release: () => void } | null = null;

  constructor(
    private readonly c: MissionCore,
    private readonly pilot: Pilot,
    private readonly go: IndoorExits,
  ) {}

  /** The way back to the exploration a confirmation interrupts: the same policy run if it is still the one flying. */
  resumeIndoor(): () => void {
    const policy = this.policyRun;
    return () => (policy && this.policyRun === policy ? policy.resume() : this.exploreNext());
  }

  /** Stop the policy explorer, if one is running. */
  release(): void {
    this.policyRun?.release();
  }

  // ----------------------------------------------------------- indoor

  startExplore(): void {
    const v = this.c.visit!;
    v.exploreStart = this.c.t;
    v.pass = 0;
    v.policyCollisions = 0;
    if (this.c.oracle) {
      v.vis = new IndoorVisibility(v.grid);
      v.explorer = new FrontierExplorer(v.grid, v.vis);
      v.vis.observe(toLocal(v.grid, this.c.vehicle.pos), this.c.vehicle.yaw);
      this.c.log('info', `Oracle frontier exploration of ${v.b.label} on the true floor plan: ${v.vis.cells.length} cells to see.`);
      this.exploreNext();
      return;
    }
    // The drone's own maps for this visit, from lidar cast against the real collision world.
    // The policy and the fallback share them.
    const lighting: LightingState = {
      timeOfDay: this.c.world.timeOfDay,
      interiorLightsOn: v.b.interiorLightsOn,
      flashlightOn: this.c.world.flashlightOn,
      sensorMode: this.c.world.sensorMode,
    };
    const lidar = shellLidar(v.grid, (...a) => this.c.physics.cast(...a));
    if (this.c.options.localization === 'estimated') this.startLocalization(lighting, lidar);
    this.c.aperture = null;
    // Back after a battery swap, the visit carries the maps of the first one (EntryPhase.startBuilding).
    if (v.sensing) this.c.log('info', `Resuming the search of ${v.b.label} on the maps from the first visit.`);
    v.sensing ??= new IndoorSensing(v.grid, this.c.loc ? this.c.loc.frameLidar(lidar) : lidar, lighting, this.c.world.seed);
    v.sensing.lastNewVoxelSec = this.c.t;
    if (this.c.vehicle instanceof QuadVehicle) v.sensing.braking = this.c.vehicle.braking;
    v.sensing.rangeHook = (lidar) => this.pilot.lidarHook(lidar);
    // Trust the camera only where it can see: in the dark without IR, that is the flashlight cone.
    const dark = nightFactor(lighting) > 0.5 && lighting.sensorMode === 'rgb';
    if (dark) {
      v.sensing.lookHalf = 0;
      v.sensing.lookCone = lighting.flashlightOn ? FLASH_HALF * 0.9 : 0;
      v.sensing.lookMin = 1.7;
    }
    v.nav = sensorNav(v.grid, v.sensing);
    if (this.c.policy) {
      this.startPolicy();
      return;
    }
    this.c.log('info', `Sensor-only frontier exploration of ${v.b.label}.`);
    this.startSensorExplore();
  }

  /** Hand the visit to the pose estimator, starting from a doorway fix with the entry error. */
  startLocalization(lighting: LightingState, lidar: LidarWorld): void {
    const v = this.c.visit!;
    this.c.endLocalization();
    let h = this.c.world.seed >>> 0;
    for (const ch of v.b.id) h = Math.imul(h ^ ch.charCodeAt(0), 0x01000193) >>> 0;
    const dark = nightFactor(lighting) > 0.5 && !lighting.interiorLightsOn;
    // The estimator starts from the aperture tracker's last reading: where the drone measured itself
    // against the opening it crossed, in the building frame the opening was measured in.
    const a = this.c.aperture!;
    const entry = { pos: toLocal(v.grid, a.pos), yaw: a.yaw, sigmaXY: ENTRY_SIGMA_XY, sigmaYaw: ENTRY_SIGMA_YAW };
    this.c.loc = new Localizer(v.grid, lidar, h, dark, this.c.localTruth(), this.c.options.locFault ?? null, entry);
    this.c.log('info', `Flying ${v.b.label} on the estimated pose (lidar scan matching, IMU and optical flow).`);
  }

  startPolicy(): void {
    const v = this.c.visit!;
    const pe = new PolicyExplorer(v.grid, this.c.policy!, v.sensing!);
    v.sensing!.lastNewVoxelSec = this.c.t;
    const start = this.c.t;
    let ended = false;
    this.c.log('info', `Trained policy exploring ${v.b.label}.`);
    // The record credits the policy unless a later fallback hands the building to the frontier planner.
    if (!this.policyFell) this.policyClean = true;
    const release = () => {
      if (ended) return;
      ended = true;
      this.policyRun = null;
    };
    const end = (why: string, next: 'done' | 'fallback' | 'continue') => {
      if (ended) return;
      release();
      if (next === 'fallback') {
        this.policyClean = false;
        this.policyFell = true;
        this.c.log('warn', `Policy explorer ${why}. Falling back to frontier exploration.`);
      } else {
        this.c.log('info', `Policy explorer ${why}.`);
      }
      // The policy only flies coverage; looking around for targets is the planner's look phase.
      this.startSensorExplore(next === 'done' ? 'look' : 'sweep');
    };
    const job: Job = {
      phase: 'indoor_explore',
      plan: () => [],
      tick: (): VehicleCommand => {
        const local = toLocal(v.grid, this.c.navPos);
        if (this.c.t - v.sensing!.lastNewVoxelSec > POLICY_STALL_SEC) end(`stalled (no new coverage for ${POLICY_STALL_SEC} s)`, 'fallback');
        else if (v.policyCollisions >= POLICY_MAX_COLLISIONS) end(`collided ${v.policyCollisions} times`, 'fallback');
        else if (pe.exhausted)
          end(
            `found no frontier left in its map (${fmt(v.sensing!.mappedCoverage() * 100, 0)}% of mapped free space flown). Planner takes the look phase`,
            'done',
          );
        else if (this.c.t - start > POLICY_MAX_SEC) end(`reached its ${POLICY_MAX_SEC} s slot`, 'continue');
        if (ended) return { vel: [0, 0, 0] };
        const cmd = pe.act(local, this.c.navVel, this.c.navYaw, this.c.t);
        return { vel: cmd.vel, yawRate: cmd.yawRate };
      },
    };
    this.policyRun = {
      resume: () => {
        // Time spent confirming does not count as a stall.
        v.sensing!.lastNewVoxelSec = this.c.t;
        this.c.setJob(job);
      },
      release,
    };
    this.c.setJob(job);
  }

  /** Sensor-only frontier exploration on the visit's maps, until no frontier is left or the slot ends. */
  startSensorExplore(start: 'sweep' | 'look' = 'sweep'): void {
    const v = this.c.visit!;
    this.frontierRan = true;
    const se = new SensorExplorer(v.grid, v.sensing!, 1, start);
    let wrapping = false;
    const job: Job = {
      phase: 'indoor_explore',
      plan: () => [],
      tick: (): VehicleCommand => {
        if (v.sensorJob !== job) return { vel: [0, 0, 0] };
        const local = toLocal(v.grid, this.c.navPos);
        // The slot is per floor. When it runs out, the explorer first gets back to the entry floor.
        const used = this.c.t - v.exploreStart - MAX_EXPLORE_SEC * v.grid.floors;
        if (used > 0 && !wrapping) {
          wrapping = true;
          this.c.log(
            'warn',
            `Exploration time slot for ${v.b.label} used up with ${fmt(v.sensing!.mappedCoverage() * 100, 0)}% of mapped free space flown.`,
          );
          se.wrapUp(local);
        }
        if (used > WRAP_UP_SEC) {
          v.sensorJob = null;
          this.explorationDone();
          return { vel: [0, 0, 0] };
        }
        const cmd = se.act(local, this.c.navVel, this.c.navYaw, this.c.t);
        if (se.exhausted) {
          v.searchComplete = !wrapping;
          v.sensorJob = null;
          this.explorationDone();
          return { vel: [0, 0, 0] };
        }
        return { vel: cmd.vel, yawRate: cmd.yawRate };
      },
    };
    v.sensorJob = job;
    this.c.setJob(job);
  }

  exploreNext(): void {
    const v = this.c.visit;
    if (!v) return;
    if (v.sensorJob) {
      this.c.setJob(v.sensorJob);
      return;
    }
    // Sensor exploration already ended (a last look at a contact interrupted it): finish up.
    if (v.sensing) {
      this.explorationDone();
      return;
    }
    if (!v.vis || !v.explorer) return;
    this.frontierRan = true;
    if (this.c.t - v.exploreStart > MAX_EXPLORE_SEC) {
      this.c.log('warn', `Exploration time slot for ${v.b.label} used up at ${fmt(v.vis.coverage() * 100, 0)}% seen.`);
      this.explorationDone();
      return;
    }
    const local = toLocal(v.grid, this.c.vehicle.pos);
    const goal = v.explorer.next(local);
    if (!goal) {
      v.searchComplete = true;
      this.explorationDone();
      return;
    }
    const wps: Waypoint[] = goal.path.map((p, i) => ({
      pos: toWorld(v.grid, p),
      spin: i === goal.path.length - 1 && goal.spin,
    }));
    this.c.setJob(this.pilot.followJob('indoor_explore', wps, () => this.exploreNext(), 0.2));
  }

  explorationDone(): void {
    const v = this.c.visit!;
    // An open contact in this building gets one more look before leaving.
    if (!this.c.returning && this.go.lastLook(v.b.id)) return;
    if (v.vis) this.c.log('info', `${v.b.label} explored: ${fmt(v.vis.coverage() * 100, 0)}% of cells seen by the oracle planner.`);
    else if (v.sensing)
      this.c.log(
        'info',
        `${v.b.label} explored: ${fmt(v.sensing.mappedCoverage() * 100, 0)}% of mapped free space flown, ${fmt(v.sensing.lookedCoverage() * 100, 0)}% looked at by the drone's own estimate.`,
      );
    if (this.c.mission.missionType === 'safety_sweep') {
      const ok = this.go.declareIfClear(v.b.id);
      const timeLeft = this.c.maxSim - this.c.t;
      if (!ok && v.pass === 0 && timeLeft > 240) {
        v.pass = 1;
        v.exploreStart = this.c.t;
        this.c.log('action', `Second, closer pass through ${v.b.label}.`);
        v.searchComplete = false;
        if (v.vis) {
          v.vis = new IndoorVisibility(v.grid, 3.5);
          v.explorer = new FrontierExplorer(v.grid, v.vis, 2.5);
          this.exploreNext();
        } else {
          // Look again from closer: forget what was looked at and trust a shorter range.
          v.sensing!.looked.fill(0);
          v.sensing!.lookRange = 3.5;
          this.startSensorExplore('look');
        }
        return;
      }
    }
    this.startExit('continue');
  }

  // ----------------------------------------------------------- exit and home

  startExit(after: 'continue' | 'rth'): void {
    this.policyRun?.release();
    const v = this.c.visit;
    // Only a building the drone got into counts as searched in the probability map.
    if (v && this.c.inside) this.c.sar?.searched(v.b.id);
    if (v) v.sensorJob = null;
    if (!v || !this.c.inside || !v.axis) {
      if (after === 'rth') this.go.startReturnOutside();
      else this.leaveBuilding();
      return;
    }
    const ft = new FlyThrough(v.axis, 'out');
    const local = toLocal(v.grid, this.c.navPos);
    const goal = toLocal(v.grid, ft.start);
    const route = (from: V3): Waypoint[] | null => v.nav.path(from, goal)?.map((p) => ({ pos: toWorld(v.grid, p) })) ?? null;
    let wps = route(local) ?? trailOut(this.c.crumbs, this.c.navPos, ft.start);
    if (!wps) {
      // No route from here on the map (the drone can be somewhere the map calls blocked): retrace the
      // breadcrumbs, which the drone has already flown, until a route opens up. Never fly straight at the opening.
      // Point by point: the outdoor casts that let retrace() take shortcuts don't see interior floors.
      wps = [];
      const crumbs = this.c.crumbs;
      for (let i = crumbs.length - 1; i >= 0; i--) {
        const p = crumbs[i]!;
        wps.push({ pos: p, tol: 0.2, flown: true });
        const rest = (crumbs.length - 1 - i) % 5 === 4 || i === 0 ? route(toLocal(v.grid, p)) : null;
        if (rest) {
          wps.push(...rest);
          break;
        }
      }
    }
    this.c.log('action', `Leaving ${v.b.label} through opening ${v.opening?.id ?? -1}.`);
    this.c.setJob(this.pilot.followJob('exit', wps, () => this.crossOut(after), 0.2));
  }

  crossOut(after: 'continue' | 'rth'): void {
    const v = this.c.visit!;
    // The fly-through servoes on the opening in view: the estimate's job ends at its start.
    this.c.endLocalization();
    this.pilot.trackAperture();
    const ft = new FlyThrough(v.axis!, 'out');
    this.c.setPhase('exit');
    this.c.setJob(
      this.pilot.flyThroughJob(ft, (ok) => {
        if (ok) {
          this.c.inside = null;
          this.c.aperture = null;
          this.c.log('action', `Outside ${v.b.label}.`);
          if (after === 'rth' || this.c.returning) this.go.startReturnOutside();
          else this.leaveBuilding();
          return;
        }
        v.aborts += 1;
        this.c.log('warn', `Exit fly-through aborted: ${ft.abortReason}. Retrying.`);
        if (v.aborts < 8) this.crossOut(after);
        else this.go.startLand('cannot exit the building');
      }),
    );
  }

  /** Climb away from the current building to cruise, then continue. */
  leaveBuilding(): void {
    const route = this.pilot.climbRoute();
    this.c.setJob(
      this.pilot.followJob(
        'exit',
        route,
        () => {
          this.c.visit = null;
          if (this.c.returning) this.go.startReturnOutside();
          else this.go.continueMission();
        },
        0.3,
      ),
    );
  }
}

/** Crumbs this close (and on the same level) are joined: the drone flew through both. */
const TRAIL_JOIN_M = 0.6;
const TRAIL_JOIN_DZ = 0.25;
/** The trail must pass this close to the opening's inside end to lead out. */
const TRAIL_REACH_M = 1.5;

/**
 * The shortest way out along the breadcrumbs: a graph of the points already flown, consecutive
 * crumbs linked and loops cut where the trail passed within TRAIL_JOIN_M of itself, searched from
 * the drone to the crumb nearest the opening's inside end. Every leg is one the drone flew (or
 * nearly), so the descents a stairwell needs are allowed. Null when the trail never reached the opening.
 */
export function trailOut(crumbs: readonly V3[], from: V3, goal: V3): Waypoint[] | null {
  const n = crumbs.length;
  if (n < 2) return null;
  const d3 = (a: V3, b: V3) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
  let start = -1;
  let end = -1;
  let bestS = Infinity;
  let bestE = TRAIL_REACH_M;
  for (let i = 0; i < n; i++) {
    const ds = d3(crumbs[i]!, from);
    if (ds < bestS) {
      bestS = ds;
      start = i;
    }
    const de = d3(crumbs[i]!, goal);
    if (de < bestE) {
      bestE = de;
      end = i;
    }
  }
  if (end < 0 || bestS > TRAIL_REACH_M) return null;
  const cost = new Float64Array(n).fill(Infinity);
  const prev = new Int32Array(n).fill(-1);
  const done = new Uint8Array(n);
  cost[start] = 0;
  for (;;) {
    let u = -1;
    for (let i = 0; i < n; i++) if (!done[i] && cost[i]! < Infinity && (u < 0 || cost[i]! < cost[u]!)) u = i;
    if (u < 0 || u === end) break;
    done[u] = 1;
    const a = crumbs[u]!;
    for (let w = 0; w < n; w++) {
      if (done[w]) continue;
      const b = crumbs[w]!;
      const dd = d3(a, b);
      const linked = Math.abs(w - u) === 1 || (dd < TRAIL_JOIN_M && Math.abs(a[2] - b[2]) < TRAIL_JOIN_DZ);
      if (linked && cost[u]! + dd < cost[w]!) {
        cost[w] = cost[u]! + dd;
        prev[w] = u;
      }
    }
  }
  if (cost[end] === Infinity) return null;
  const path: V3[] = [];
  for (let i = end; i >= 0; i = prev[i]!) path.push(crumbs[i]!);
  path.reverse();
  const wps: Waypoint[] = path.map((p) => ({ pos: [p[0], p[1], p[2]] as V3, tol: 0.2, flown: true }));
  wps.push({ pos: [goal[0], goal[1], goal[2]], flown: true });
  return wps;
}

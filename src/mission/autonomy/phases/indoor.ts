import type { LightingState } from '../../../shared/types';
import { nightFactor } from '../../../sim/lighting';
import { FLASH_HALF } from '../../../perception/model';
import type { V3 } from '../contracts';
import { FlyThrough } from '../planner/flythrough';
import { columnClear, cruiseZ } from '../planner/geometry';
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
import { EntryPhases } from './entry';

/** Inside a building: policy or sensor-only exploration, re-views of open tracks, and the way back out. */
export abstract class IndoorPhases extends EntryPhases {
  // ----------------------------------------------------------- indoor

  protected startExplore(): void {
    const v = this.visit!;
    v.exploreStart = this.t;
    v.pass = 0;
    v.policyCollisions = 0;
    if (this.oracle) {
      v.vis = new IndoorVisibility(v.grid);
      v.explorer = new FrontierExplorer(v.grid, v.vis);
      v.vis.observe(toLocal(v.grid, this.vehicle.pos), this.vehicle.yaw);
      this.log('info', `Oracle frontier exploration of ${v.b.label} on the true floor plan: ${v.vis.cells.length} cells to see.`);
      this.exploreNext();
      return;
    }
    // The drone's own maps for this visit, from lidar cast against the real collision world.
    // The policy and the fallback share them.
    const lighting: LightingState = {
      timeOfDay: this.world.timeOfDay,
      interiorLightsOn: v.b.interiorLightsOn,
      flashlightOn: this.world.flashlightOn,
      sensorMode: this.world.sensorMode,
    };
    const lidar = shellLidar(v.grid, (...a) => this.physics.cast(...a));
    if (this.options.localization === 'estimated') this.startLocalization(lighting, lidar);
    this.aperture = null;
    v.sensing = new IndoorSensing(v.grid, this.loc ? this.loc.frameLidar(lidar) : lidar, lighting, this.world.seed);
    v.sensing.lastNewVoxelSec = this.t;
    if (this.vehicle instanceof QuadVehicle) v.sensing.braking = this.vehicle.braking;
    const extra = this.options.lidarFault;
    v.sensing.rangeHook = (lidar, tSec) => {
      extra?.(lidar, tSec);
      this.lidarHook(lidar);
    };
    // Trust the camera only where it can see: in the dark without IR, that is the flashlight cone.
    const dark = nightFactor(lighting) > 0.5 && lighting.sensorMode === 'rgb';
    if (dark) {
      v.sensing.lookHalf = 0;
      v.sensing.lookCone = lighting.flashlightOn ? FLASH_HALF * 0.9 : 0;
      v.sensing.lookMin = 1.7;
    }
    v.nav = sensorNav(v.grid, v.sensing);
    if (this.policy) {
      this.startPolicy();
      return;
    }
    this.log('info', `Sensor-only frontier exploration of ${v.b.label}.`);
    this.startSensorExplore();
  }

  /** Hand the visit to the pose estimator, starting from a doorway fix with the entry error. */
  protected startLocalization(lighting: LightingState, lidar: LidarWorld): void {
    const v = this.visit!;
    this.endLocalization();
    let h = this.world.seed >>> 0;
    for (const ch of v.b.id) h = Math.imul(h ^ ch.charCodeAt(0), 0x01000193) >>> 0;
    const dark = nightFactor(lighting) > 0.5 && !lighting.interiorLightsOn;
    // The estimator starts from the aperture tracker's last reading: where the drone measured itself
    // against the opening it crossed, in the building frame the opening was measured in.
    const a = this.aperture!;
    const entry = { pos: toLocal(v.grid, a.pos), yaw: a.yaw, sigmaXY: ENTRY_SIGMA_XY, sigmaYaw: ENTRY_SIGMA_YAW };
    this.loc = new Localizer(v.grid, lidar, h, dark, this.localTruth(), this.options.locFault ?? null, entry);
    this.log('info', `Flying ${v.b.label} on the estimated pose (lidar scan matching, IMU and optical flow).`);
  }

  protected startPolicy(): void {
    const v = this.visit!;
    const pe = new PolicyExplorer(v.grid, this.policy!, v.sensing!);
    v.sensing!.lastNewVoxelSec = this.t;
    const start = this.t;
    let ended = false;
    this.log('info', `Trained policy exploring ${v.b.label}.`);
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
        this.log('warn', `Policy explorer ${why}. Falling back to frontier exploration.`);
      } else {
        this.log('info', `Policy explorer ${why}.`);
      }
      // The policy only flies coverage; looking around for targets is the planner's look phase.
      this.startSensorExplore(next === 'done' ? 'look' : 'sweep');
    };
    const job: Job = {
      phase: 'indoor_explore',
      plan: () => [],
      tick: (): VehicleCommand => {
        const local = toLocal(v.grid, this.navPos);
        if (this.t - v.sensing!.lastNewVoxelSec > POLICY_STALL_SEC) end(`stalled (no new coverage for ${POLICY_STALL_SEC} s)`, 'fallback');
        else if (v.policyCollisions >= POLICY_MAX_COLLISIONS) end(`collided ${v.policyCollisions} times`, 'fallback');
        else if (pe.exhausted)
          end(
            `found no frontier left in its map (${fmt(v.sensing!.mappedCoverage() * 100, 0)}% of mapped free space flown). Planner takes the look phase`,
            'done',
          );
        else if (this.t - start > POLICY_MAX_SEC) end(`reached its ${POLICY_MAX_SEC} s slot`, 'continue');
        if (ended) return { vel: [0, 0, 0] };
        const cmd = pe.act(local, this.navVel, this.navYaw, this.t);
        return { vel: cmd.vel, yawRate: cmd.yawRate };
      },
    };
    this.policyRun = {
      resume: () => {
        // Time spent confirming does not count as a stall.
        v.sensing!.lastNewVoxelSec = this.t;
        this.setJob(job);
      },
      release,
    };
    this.setJob(job);
  }

  /** Sensor-only frontier exploration on the visit's maps, until no frontier is left or the slot ends. */
  protected startSensorExplore(start: 'sweep' | 'look' = 'sweep'): void {
    const v = this.visit!;
    this.frontierRan = true;
    const se = new SensorExplorer(v.grid, v.sensing!, 1, start);
    let wrapping = false;
    const job: Job = {
      phase: 'indoor_explore',
      plan: () => [],
      tick: (): VehicleCommand => {
        if (v.sensorJob !== job) return { vel: [0, 0, 0] };
        const local = toLocal(v.grid, this.navPos);
        // The slot is per floor. When it runs out, the explorer first gets back to the entry floor.
        const used = this.t - v.exploreStart - MAX_EXPLORE_SEC * v.grid.floors;
        if (used > 0 && !wrapping) {
          wrapping = true;
          this.log(
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
        const cmd = se.act(local, this.navVel, this.navYaw, this.t);
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
    this.setJob(job);
  }

  protected exploreNext(): void {
    const v = this.visit;
    if (!v) return;
    if (v.sensorJob) {
      this.setJob(v.sensorJob);
      return;
    }
    // Sensor exploration already ended (a last look at a contact interrupted it): finish up.
    if (v.sensing) {
      this.explorationDone();
      return;
    }
    if (!v.vis || !v.explorer) return;
    this.frontierRan = true;
    if (this.t - v.exploreStart > MAX_EXPLORE_SEC) {
      this.log('warn', `Exploration time slot for ${v.b.label} used up at ${fmt(v.vis.coverage() * 100, 0)}% seen.`);
      this.explorationDone();
      return;
    }
    const local = toLocal(v.grid, this.vehicle.pos);
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
    this.setJob(this.followJob('indoor_explore', wps, () => this.exploreNext(), 0.2));
  }

  protected explorationDone(): void {
    const v = this.visit!;
    // An open contact in this building gets one more look before leaving.
    const open = this.tracker.tracks.find(
      (tr) => tr.status === 'open' && tr.buildingId === v.b.id && this.attempted.get(tr.id) !== Infinity && !this.finalLook.has(tr.id),
    );
    if (open && !this.returning) {
      this.finalLook.add(open.id);
      this.startConfirm(open);
      return;
    }
    if (v.vis) this.log('info', `${v.b.label} explored: ${fmt(v.vis.coverage() * 100, 0)}% of cells seen by the oracle planner.`);
    else if (v.sensing)
      this.log(
        'info',
        `${v.b.label} explored: ${fmt(v.sensing.mappedCoverage() * 100, 0)}% of mapped free space flown, ${fmt(v.sensing.lookedCoverage() * 100, 0)}% looked at by the drone's own estimate.`,
      );
    if (this.mission.missionType === 'safety_sweep') {
      const ok = this.declareIfClear(v.b.id);
      const timeLeft = this.maxSim - this.t;
      if (!ok && v.pass === 0 && timeLeft > 240) {
        v.pass = 1;
        v.exploreStart = this.t;
        this.log('action', `Second, closer pass through ${v.b.label}.`);
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

  protected startExit(after: 'continue' | 'rth'): void {
    this.policyRun?.release();
    const v = this.visit;
    if (v) v.sensorJob = null;
    if (!v || !this.inside || !v.axis) {
      if (after === 'rth') this.startReturnOutside();
      else this.leaveBuilding();
      return;
    }
    const ft = new FlyThrough(v.axis, 'out');
    const local = toLocal(v.grid, this.navPos);
    const goal = toLocal(v.grid, ft.start);
    const route = (from: V3): Waypoint[] | null => v.nav.path(from, goal)?.map((p) => ({ pos: toWorld(v.grid, p) })) ?? null;
    let wps = route(local);
    if (!wps) {
      // No route from here on the map (the drone can be somewhere the map calls blocked): retrace the
      // breadcrumbs, which the drone has already flown, until a route opens up. Never fly straight at the opening.
      // Point by point: the outdoor casts that let retrace() take shortcuts don't see interior floors.
      wps = [];
      const crumbs = this.crumbs;
      for (let i = crumbs.length - 1; i >= 0; i--) {
        const p = crumbs[i]!;
        wps.push({ pos: p, tol: 0.2 });
        const rest = (crumbs.length - 1 - i) % 5 === 4 || i === 0 ? route(toLocal(v.grid, p)) : null;
        if (rest) {
          wps.push(...rest);
          break;
        }
      }
    }
    this.log('action', `Leaving ${v.b.label} through opening ${v.opening?.id ?? -1}.`);
    this.setJob(this.followJob('exit', wps, () => this.crossOut(after), 0.2));
  }

  protected crossOut(after: 'continue' | 'rth'): void {
    const v = this.visit!;
    // The fly-through servoes on the opening in view: the estimate's job ends at its start.
    this.endLocalization();
    this.trackAperture();
    const ft = new FlyThrough(v.axis!, 'out');
    this.setPhase('exit');
    this.setJob(
      this.flyThroughJob(ft, (ok) => {
        if (ok) {
          this.inside = null;
          this.aperture = null;
          this.log('action', `Outside ${v.b.label}.`);
          if (after === 'rth' || this.returning) this.startReturnOutside();
          else this.leaveBuilding();
          return;
        }
        v.aborts += 1;
        this.log('warn', `Exit fly-through aborted: ${ft.abortReason}. Retrying.`);
        if (v.aborts < 8) this.crossOut(after);
        else this.startLand('cannot exit the building');
      }),
    );
  }

  /** Climb away from the current building to cruise, then continue. */
  protected leaveBuilding(): void {
    const route = this.climbRoute();
    this.setJob(
      this.followJob(
        'exit',
        route,
        () => {
          this.visit = null;
          if (this.returning) this.startReturnOutside();
          else this.continueMission();
        },
        0.3,
      ),
    );
  }

  protected climbRoute(): Waypoint[] {
    const pos = this.vehicle.pos;
    const top = cruiseZ(this.air, pos[0], pos[1], this.agl);
    if (pos[2] >= top - 0.5) return [];
    if (columnClear(this.physics, pos[0], pos[1], pos[2], top, 0.45)) return [{ pos: [pos[0], pos[1], top] }];
    const open = (p: V3) => columnClear(this.physics, p[0], p[1], p[2], cruiseZ(this.air, p[0], p[1], this.agl), 0.45);
    const back = this.retrace(this.visit?.trail.length ? this.visit.trail : this.crumbs, open);
    const end = back.length ? back[back.length - 1]!.pos : pos;
    if (back.length && !open(end)) this.log('warn', 'No open column found on the way back. Climbing anyway.');
    return [...back, { pos: [end[0], end[1], cruiseZ(this.air, end[0], end[1], this.agl)] }];
  }
}

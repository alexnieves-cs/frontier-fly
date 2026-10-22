import { defaultEstimatorOptions, PoseEstimator } from '../../loc/estimator';
import { LocScorer, type LocMetrics } from '../../loc/metrics';
import { LocSensors, locNoise } from '../../loc/sensors';
import type { PoseEstimate } from '../../loc/types';
import { RNG } from '../../shared/rng';
import type { V3 } from './contracts';
import { readLocSensors, type LocSensorHook } from './locSensors';
import type { IndoorGrid, LidarWorld } from './planner/indoor';

/** 1-sigma error of the pose the estimator starts from at the doorway, m and rad. */
export const ENTRY_SIGMA_XY = 0.1;
export const ENTRY_SIGMA_YAW = 0.03;

/** What the vehicle really did this tick, in the building-local frame. */
export interface LocalTruth {
  pos: V3;
  yaw: number;
  vel: V3;
  /** World-frame acceleration, m/s^2. */
  accel: V3;
  yawRate: number;
  dt: number;
}

/** One building visit flown on the estimated pose. */
export interface LocVisitRecord {
  building: string;
  metrics: LocMetrics;
  /** Times the estimate went lost, and the seconds spent lost. */
  lostEvents: number;
  lostSec: number;
  recoveries: number;
  sweeps: number;
  matched: number;
}

/**
 * The drone's own pose inside one building: the src/loc estimator fed by
 * simulated IMU, optical flow, lidar and altimeter readings. The sensors are
 * the only thing here that reads the truth. Everything else in the visit
 * (maps, planners, the shield) runs in the estimate's frame, and
 * `frameLidar` maps the drone's lidar rays from that frame into the real
 * building, so the maps it paints carry exactly the error the estimate has.
 */
export class Localizer {
  readonly est: PoseEstimator;
  private readonly sensors: LocSensors;
  private readonly scorer = new LocScorer();
  private current: PoseEstimate;
  private truth: LocalTruth;
  private vz = 0;
  private lastZ: number;
  private wasLost = false;
  lostEvents = 0;
  lostTicks = 0;
  private ticks = 0;
  private dt = 0.05;

  constructor(
    readonly grid: IndoorGrid,
    /** Lidar against the real building, in the true local frame. */
    world: LidarWorld,
    seed: number,
    dark: boolean,
    start: LocalTruth,
    private readonly hook: LocSensorHook | null = null,
    /**
     * The pose the estimator starts from, the drone's own measurement (the aperture tracker at the
     * doorway), with its 1-sigma. Without it, the start is `start` plus an error drawn at the
     * ENTRY_SIGMA levels, for offline studies that have no tracker.
     */
    entry: { pos: V3; yaw: number; sigmaXY: number; sigmaYaw: number } | null = null,
  ) {
    const w = grid.w * grid.cs;
    const d = grid.d * grid.cs;
    this.est = new PoseEstimator(defaultEstimatorOptions({ x0: -2, y0: -2, x1: w + 2, y1: d + 2 }, grid.floors, grid.H));
    this.sensors = new LocSensors(world, seed, locNoise(dark));
    if (entry) {
      this.est.reset([entry.pos[0], entry.pos[1], entry.pos[2]], entry.yaw, entry.sigmaXY, entry.sigmaYaw);
    } else {
      const rng = new RNG(seed >>> 0, 'loc-entry');
      this.est.reset(
        [start.pos[0] + ENTRY_SIGMA_XY * rng.gaussian(), start.pos[1] + ENTRY_SIGMA_XY * rng.gaussian(), start.pos[2]],
        start.yaw + ENTRY_SIGMA_YAW * rng.gaussian(),
        ENTRY_SIGMA_XY,
        ENTRY_SIGMA_YAW,
      );
    }
    this.truth = start;
    this.current = this.est.estimate();
    this.lastZ = this.current.pos[2];
  }

  /** Feed one tick of sensor readings taken at the vehicle's true pose. */
  step(truth: LocalTruth, tSec: number): PoseEstimate {
    // The gyro is read at the IMU's own rate and integrated over the control tick, so the rate the
    // estimator gets is the tick's mean: the heading change over the tick, not the body rate at its end.
    const dyaw = truth.yaw - this.truth.yaw;
    truth = { ...truth, yawRate: Math.atan2(Math.sin(dyaw), Math.cos(dyaw)) / Math.max(truth.dt, 1e-6) };
    this.truth = truth;
    this.dt = truth.dt;
    const frame = readLocSensors(this.sensors, truth);
    this.hook?.(frame, tSec);
    const e = this.est.step(frame);
    this.scorer.add(truth, e);
    this.vz += (((e.pos[2] - this.lastZ) / Math.max(truth.dt, 1e-6) - this.vz) * truth.dt) / 0.2;
    this.lastZ = e.pos[2];
    this.current = e;
    this.ticks += 1;
    const lost = e.health.state === 'lost';
    if (lost) this.lostTicks += 1;
    if (lost && !this.wasLost) this.lostEvents += 1;
    this.wasLost = lost;
    return e;
  }

  get estimate(): PoseEstimate {
    return this.current;
  }

  get lost(): boolean {
    return this.current.health.state === 'lost';
  }

  /** Estimated pose in the building-local frame. */
  get pos(): V3 {
    const p = this.current.pos;
    return [p[0], p[1], p[2]];
  }

  get yaw(): number {
    return this.current.yaw;
  }

  get vel(): V3 {
    const v = this.current.vel;
    return [v[0], v[1], this.vz];
  }

  /** Heading error, true minus estimated: rotate a command by this to fly it in the real world. */
  get yawError(): number {
    const d = this.truth.yaw - this.current.yaw;
    return Math.atan2(Math.sin(d), Math.cos(d));
  }

  /**
   * The lidar as the estimate's frame sees it: a ray cast from an estimated
   * pose is cast from the true pose with the heading error applied, so its
   * return is what the real sensor would get, and the map it paints sits where
   * the estimate thinks the walls are.
   */
  frameLidar(real: LidarWorld): LidarWorld {
    return {
      cast: (x, y, z, dx, dy, dz, maxDist) => {
        const e = this.current.pos;
        const t = this.truth.pos;
        const c = Math.cos(this.yawError);
        const s = Math.sin(this.yawError);
        const rx = x - e[0];
        const ry = y - e[1];
        return real.cast(
          t[0] + c * rx - s * ry,
          t[1] + s * rx + c * ry,
          t[2] + (z - e[2]),
          c * dx - s * dy,
          s * dx + c * dy,
          dz,
          maxDist,
        );
      },
    };
  }

  /**
   * A point the camera placed relative to the true pose, re-placed relative
   * to the estimate (building-local frame): what the drone would compute from
   * the same bounding box and range with its own pose.
   */
  toEstimateFrame(local: V3): V3 {
    const e = this.current.pos;
    const t = this.truth.pos;
    const c = Math.cos(-this.yawError);
    const s = Math.sin(-this.yawError);
    const rx = local[0] - t[0];
    const ry = local[1] - t[1];
    return [e[0] + c * rx - s * ry, e[1] + s * rx + c * ry, e[2] + (local[2] - t[2])];
  }

  /** A world-frame direction as the estimate's frame sees it. */
  toEstimateDir(d: V3): V3 {
    const c = Math.cos(-this.yawError);
    const s = Math.sin(-this.yawError);
    return [c * d[0] - s * d[1], s * d[0] + c * d[1], d[2]];
  }

  record(building: string): LocVisitRecord {
    const st = this.est.stats;
    return {
      building,
      metrics: this.scorer.result(),
      lostEvents: this.lostEvents,
      lostSec: this.lostTicks * this.dt,
      recoveries: st.recoveries,
      sweeps: st.sweeps,
      matched: st.matched,
    };
  }

  get flownTicks(): number {
    return this.ticks;
  }
}

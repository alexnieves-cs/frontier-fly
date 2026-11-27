/**
 * The drift track's sensor frame (FrontierBench Hard round 3, bench/hard.ts DRIFT_TASKS). On every
 * other track a pilot reads its true pose, and its seen and flown maps are painted from that pose.
 * Here nothing in the pilot's loop reads the truth: the pose is the src/loc estimator's (IMU,
 * optical flow, range finder and a 10 Hz lidar matched against its own map), the seen and flown
 * maps are painted from that estimate, and the measured velocity is rotated by the estimated
 * heading. A pilot's command is still flown in the real body frame, so a heading error turns
 * every command, the way it does on a real drone.
 *
 * The scoring does not change: the camera footprint, the crash rule and the return radius are
 * measured on the true pose. One piece of floor-plan knowledge stays, the same as on the other
 * tracks: seen voxels are limited to the flyable mask, so a voxel beside a shelf cannot become a
 * frontier that never clears.
 */
import * as C from '../shared/constants';
import { defaultEstimatorOptions, PoseEstimator } from '../loc/estimator';
import { LocSensors, locNoise, type TruthSample } from '../loc/sensors';
import type { PoseEstimate } from '../loc/types';
import { RNG } from '../shared/rng';
import { voxelLayout } from '../shared/voxels';
import { SeenMap } from '../sim/seenMap';
import { measuredVelocity, type SensorFrame } from '../sim/sensorPlanner';
import type { ExplorerEnv } from '../sim/env';

/** 1-sigma error of the take-off pose the estimator starts from, m and rad (as in missions). */
const START_SIGMA_XY = 0.1;
const START_SIGMA_YAW = 0.03;

/**
 * Sensor faults of the degraded-sensor track (round 4, hard.ts SENSOR_TASKS). They hit what the pilot
 * reads and what the estimator reads alike: a dead lidar ray reads as no return, a dead flow sensor
 * reports nothing. A fault starts at take-off, or at a time drawn from `failAt` (seconds) per seed.
 */
export interface SensorFaults {
  /** A contiguous dead lidar sector this wide, radians, at a body angle drawn per seed. */
  lidarSector?: number;
  /** Only every k-th lidar ray returns (a sparse or partly blocked scanner). */
  lidarKeep?: number;
  /** The optical-flow sensor reports nothing. */
  flowDead?: boolean;
  /** The faults start at a time drawn uniformly from this range, s; otherwise at take-off. */
  failAt?: readonly [number, number];
}

/** How far the estimate was from the truth over a flight, horizontal metres. */
export interface DriftStats {
  mean: number;
  max: number;
  final: number;
  /** Ticks the estimator called itself lost. */
  lostTicks: number;
}

export class DriftFrames {
  private readonly est: PoseEstimator;
  private readonly sensors: LocSensors;
  private readonly seen = new SeenMap();
  private readonly visited: Uint8Array;
  private readonly lidar = new Float64Array(C.LIDAR_H);
  private readonly vw: number;
  private readonly vd: number;
  private readonly floors: number;
  private readonly floorHeight: number;
  private prevVel: [number, number, number];
  private prevYaw: number;
  private current: PoseEstimate;
  private errSum = 0;
  private errMax = 0;
  private errLast = 0;
  private ticks = 0;
  private readonly faults: SensorFaults | null;
  /** Body angle where the dead lidar sector starts, and the policy step the faults start. */
  private readonly sector0: number;
  private readonly failStep: number;
  private lost = 0;

  constructor(
    private readonly env: ExplorerEnv,
    seed: number,
    dark: boolean,
    faults: SensorFaults | null = null,
  ) {
    const spec = env.building!;
    const world = env.world!;
    ({ vw: this.vw, vd: this.vd, floors: this.floors } = voxelLayout(spec));
    this.floorHeight = spec.floorHeight;
    const w = this.vw * C.VOXEL;
    const d = this.vd * C.VOXEL;
    this.est = new PoseEstimator(defaultEstimatorOptions({ x0: -2, y0: -2, x1: w + 2, y1: d + 2 }, this.floors, this.floorHeight));
    this.sensors = new LocSensors(world, seed, locNoise(dark));
    const s = env.snapshot();
    const rng = new RNG(seed >>> 0, 'drift-start');
    this.est.reset([s.pos[0] + START_SIGMA_XY * rng.gaussian(), s.pos[1] + START_SIGMA_XY * rng.gaussian(), s.pos[2]], s.yaw + START_SIGMA_YAW * rng.gaussian(), START_SIGMA_XY, START_SIGMA_YAW);
    this.current = this.est.estimate();
    this.seen.reset(spec);
    this.visited = new Uint8Array(this.seen.data.length);
    this.prevVel = [s.vel[0], s.vel[1], s.vel[2]];
    this.prevYaw = s.yaw;
    this.faults = faults;
    const frng = new RNG(seed >>> 0, 'sensor-faults');
    this.sector0 = frng.next() * Math.PI * 2;
    this.failStep = faults?.failAt ? Math.round((faults.failAt[0] + frng.next() * (faults.failAt[1] - faults.failAt[0])) / C.POLICY_DT) : 0;
  }

  /** Whether ray i of n (body angle i * 2π / n) is dead right now. */
  private dead(i: number, n: number): boolean {
    const f = this.faults;
    if (!f || this.ticks < this.failStep) return false;
    if (f.lidarKeep && f.lidarKeep > 1 && i % f.lidarKeep !== 0) return true;
    if (f.lidarSector) {
      const a = (i / n) * Math.PI * 2 - this.sector0;
      const w = a - Math.floor(a / (Math.PI * 2)) * Math.PI * 2;
      if (w < f.lidarSector) return true;
    }
    return false;
  }

  /** Feed one policy step of sensor readings, then build the pilot's frame from the estimate. */
  frame(obs: Float32Array, first = false): SensorFrame {
    const s = this.env.snapshot();
    if (!first) {
      const dt = C.POLICY_DT;
      const dyaw = s.yaw - this.prevYaw;
      const truth: TruthSample = {
        pos: [s.pos[0], s.pos[1], s.pos[2]],
        yaw: s.yaw,
        vel: [s.vel[0], s.vel[1], s.vel[2]],
        accel: [(s.vel[0] - this.prevVel[0]) / dt, (s.vel[1] - this.prevVel[1]) / dt, (s.vel[2] - this.prevVel[2]) / dt],
        yawRate: Math.atan2(Math.sin(dyaw), Math.cos(dyaw)) / dt,
        dt,
      };
      const lf = this.sensors.read(truth);
      if (this.faults && this.ticks >= this.failStep) {
        if (this.faults.flowDead) lf.flow = null;
        if (lf.scan) {
          const r = lf.scan.ranges;
          for (let i = 0; i < r.length; i++) if (this.dead(i, r.length)) r[i] = lf.scan.maxRange;
        }
      }
      this.current = this.est.step(lf);
      this.prevVel = truth.vel;
      this.prevYaw = s.yaw;
    }
    const e = this.current;
    const err = Math.hypot(e.pos[0] - s.pos[0], e.pos[1] - s.pos[1]);
    this.errSum += err;
    this.errMax = Math.max(this.errMax, err);
    this.errLast = err;
    this.ticks += 1;
    if (e.health.state === 'lost') this.lost += 1;

    for (let i = 0; i < C.LIDAR_H; i++) this.lidar[i] = this.dead(i, C.LIDAR_H) ? C.LIDAR_MAX : obs[C.OBS.lidar + i]! * C.LIDAR_MAX;
    const floor = Math.floor(e.pos[2] / this.floorHeight);
    this.seen.markRing(e.pos[0], e.pos[1], floor, e.yaw, this.lidar);
    const vx = Math.floor(e.pos[0] / C.VOXEL);
    const vy = Math.floor(e.pos[1] / C.VOXEL);
    if (floor >= 0 && floor < this.floors && vx >= 0 && vy >= 0 && vx < this.vw && vy < this.vd) this.visited[floor * this.vw * this.vd + vy * this.vw + vx] = 1;
    return {
      seen: this.seen.data,
      visited: this.visited,
      vw: this.vw,
      vd: this.vd,
      floors: this.floors,
      floorHeight: this.floorHeight,
      pos: [e.pos[0], e.pos[1], e.pos[2]],
      yaw: e.yaw,
      lidar: this.lidar,
      alt: obs[C.OBS.alt]! * this.floorHeight,
      vel: measuredVelocity(obs, e.yaw),
      timeLeft: obs[C.OBS.time]!,
    };
  }

  stats(): DriftStats {
    return { mean: this.errSum / Math.max(1, this.ticks), max: this.errMax, final: this.errLast, lostTicks: this.lost };
  }
}

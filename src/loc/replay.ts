import { defaultEstimatorOptions, PoseEstimator, type EstimatorOptions } from './estimator';
import { LocScorer, type LocMetrics } from './metrics';
import { LocSensors, locNoise, type Caster, type LocNoise, type TruthSample } from './sensors';
import type { V3 } from './types';
import { RNG } from '../shared/rng';

/** A flown trajectory: truth pose and world velocity every tick. */
export interface TrajectoryLog {
  dt: number;
  /** Per tick: x, y, z, yaw, vx, vy, vz. */
  poses: Float32Array;
}

export const LOG_STRIDE = 7;

export function logTicks(log: TrajectoryLog): number {
  return Math.floor(log.poses.length / LOG_STRIDE);
}

/** Truth for tick i, with acceleration and yaw rate by finite differences (what an ideal IMU measures). */
export function truthAt(log: TrajectoryLog, i: number): TruthSample {
  const p = log.poses;
  const o = i * LOG_STRIDE;
  const q = Math.max(0, i - 1) * LOG_STRIDE;
  const dt = log.dt;
  let dyaw = p[o + 3]! - p[q + 3]!;
  dyaw = Math.atan2(Math.sin(dyaw), Math.cos(dyaw));
  return {
    pos: [p[o]!, p[o + 1]!, p[o + 2]!],
    yaw: p[o + 3]!,
    vel: [p[o + 4]!, p[o + 5]!, p[o + 6]!],
    accel: i === 0 ? [0, 0, 0] : [(p[o + 4]! - p[q + 4]!) / dt, (p[o + 5]! - p[q + 5]!) / dt, (p[o + 6]! - p[q + 6]!) / dt],
    yawRate: i === 0 ? 0 : dyaw / dt,
    dt,
  };
}

export interface ReplayOptions {
  noise?: LocNoise;
  estimator?: Partial<EstimatorOptions>;
  /** 1-sigma error of the starting pose the estimator is handed, m and rad. */
  initSigmaXY?: number;
  initSigmaYaw?: number;
  /** Called every tick with the truth and the estimate (study hooks, maps). */
  onTick?: (truth: TruthSample, est: ReturnType<PoseEstimator['step']>, i: number) => void;
  /**
   * Kidnapped robot: at this tick the estimate jumps by this offset with its
   * covariance unchanged, as if the drone were displaced (a collision spin, a
   * gust) without its sensors noticing. Applied to the estimate rather than
   * the truth so the drone is never placed inside a wall.
   */
  kidnap?: { tick: number; offset: V3; yaw?: number };
}

export interface ReplayResult {
  metrics: LocMetrics;
  stats: PoseEstimator['stats'];
}

/**
 * Run the sensor model and the estimator open loop over a logged trajectory:
 * the estimate never steers the drone, so every estimator variant sees the
 * same path. The sensors cast against `world`, the same collision world the
 * trajectory was flown in.
 */
export function replay(
  log: TrajectoryLog,
  world: Caster,
  bounds: EstimatorOptions['bounds'],
  floors: number,
  floorHeight: number,
  seed: number,
  opts: ReplayOptions = {},
): ReplayResult {
  const sensors = new LocSensors(world, seed, opts.noise ?? locNoise());
  const est = new PoseEstimator({ ...defaultEstimatorOptions(bounds, floors, floorHeight), ...opts.estimator });
  const scorer = new LocScorer();
  const rng = new RNG(seed >>> 0, 'loc-init');
  const sxy = opts.initSigmaXY ?? 0.1;
  const syaw = opts.initSigmaYaw ?? 0.03;
  const first = truthAt(log, 0);
  est.reset(
    [first.pos[0] + sxy * rng.gaussian(), first.pos[1] + sxy * rng.gaussian(), first.pos[2]],
    first.yaw + syaw * rng.gaussian(),
    Math.max(sxy, 0.02),
    Math.max(syaw, 0.005),
  );
  const n = logTicks(log);
  for (let i = 0; i < n; i++) {
    const truth = truthAt(log, i);
    if (opts.kidnap && i === opts.kidnap.tick) {
      est.x += opts.kidnap.offset[0];
      est.y += opts.kidnap.offset[1];
      est.yaw += opts.kidnap.yaw ?? 0;
    }
    const e = est.step(sensors.read(truth));
    scorer.add(truth, e);
    opts.onTick?.(truth, e, i);
  }
  return { metrics: scorer.result(), stats: { ...est.stats } };
}

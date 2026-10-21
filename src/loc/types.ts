/**
 * Sensor samples and outputs of the indoor pose estimator (src/loc). Every
 * sample is what a real drone's sensors report, in the body frame, with the
 * noise of src/loc/sensors.ts already applied. Body x is forward, body y is
 * left, z is up; yaw is counter-clockwise from world +x.
 */

export type V2 = [number, number];
export type V3 = [number, number, number];

/** Gyro and accelerometer, one sample per estimator tick. */
export interface ImuSample {
  /** Seconds since the previous sample. */
  dt: number;
  /** Angular rate, rad/s, body frame. */
  gyro: V3;
  /** Specific force minus gravity, m/s^2, body frame (zero when hovering). */
  accel: V3;
}

/** Downward optical-flow sensor (PMW3901 / PX4FLOW class) scaled by the range finder. */
export interface FlowSample {
  /** Horizontal velocity over the ground, m/s, body frame. */
  vel: V2;
  /** 0..1 texture and light quality the sensor reports. */
  quality: number;
}

/** One sweep of the horizontal 2D lidar. */
export interface ScanSample {
  /** Beam i points at yaw + i * 2π / ranges.length. Metres; >= maxRange - 1e-3 is a miss or dropout. */
  ranges: Float64Array;
  maxRange: number;
}

/** Downward range finder and barometer. */
export interface AltSample {
  /** Metres to whatever is below, or null on a dropout. */
  down: number | null;
  /** Barometric height above the entry point's datum, metres (noisy, drifts slowly). */
  baro: number;
}

/** All of one tick's sensor data. */
export interface LocFrame {
  imu: ImuSample;
  /** Null when the flow sensor dropped this tick. */
  flow: FlowSample | null;
  /** Null on ticks without a new sweep. */
  scan: ScanSample | null;
  alt: AltSample;
}

export type TrackState = 'tracking' | 'degraded' | 'lost';

/** What the estimator believes about its own accuracy. A failsafe reads this, never the truth. */
export interface PoseHealth {
  /** False once the estimator declares itself lost. */
  ok: boolean;
  /** 1-sigma horizontal position error in the building frame along the worst axis, metres. */
  sigmaM: number;
  /** The same relative to the drone's own map (what collision avoidance and exploration feel). */
  sigmaMapM: number;
  sigmaYawRad: number;
  state: TrackState;
}

export interface PoseEstimate {
  pos: V3;
  yaw: number;
  /** World-frame velocity, m/s. */
  vel: V3;
  /** Row-major 3x3 covariance over (x, y, yaw), relative to the drone's own map. */
  cov: Float64Array;
  health: PoseHealth;
}

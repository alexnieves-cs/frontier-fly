import type { LocSensors, TruthSample } from '../../loc/sensors';
import type { LocFrame } from '../../loc/types';

/**
 * Rewrites one tick of localization sensor readings in place before the
 * estimator sees them: a fault model's entry point (lidar sector dropout,
 * range noise, a frozen or missing flow or IMU sample).
 */
export type LocSensorHook = (frame: LocFrame, tSec: number) => void;

/**
 * One tick of what the drone's localization sensors report (IMU, optical
 * flow, lidar sweep, altimeter), taken at the vehicle's true pose. This is
 * the only place an indoor visit flown on the estimate reads the truth.
 */
export function readLocSensors(sensors: LocSensors, truth: TruthSample): LocFrame {
  return sensors.read(truth);
}

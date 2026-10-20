import { illuminationOf, recallProbability } from '../../../perception/model';
import type { SensorMode } from '../../../shared/types';
import { FLASH_HALF } from '../../../perception/model';
import { GIMBAL_ASPECT, GIMBAL_FOV_Y, GIMBAL_RANGE_SCALE } from '../perception';

/*
 * Probability of detection (POD) for the downward gimbal camera.
 *
 * The planner's model of its own sensor: for a camera at height h over a
 * ground cell, at offset (along, across) in the image frame, the chance one
 * frame yields a person contact. It is the detector's analytic recall curve
 * (src/perception/model.ts) evaluated at the person's expected distance,
 * illumination, camera speed and image area. Occlusion comes from the
 * grid's sky-view map: a person whose sightline to the camera passes through
 * a crown or a building is hidden. It is a model: the detector ray-casts
 * several sightlines to the body's box and draws per frame. `scripts/study/sar-study.ts`
 * measures how well this model predicts the detector (the calibration
 * section of the report), and POD_CALIBRATION scales it to match.
 */

/** Height of a person's body center above the ground, averaged over lying, sitting and standing. */
export const PERSON_CENTER_M = 0.5;
/** Effective ground-projected area of a person seen from above, m². */
export const PERSON_AREA_M2 = 0.5;
/**
 * Scale on the per-frame model, fitted on development seeds so the model's
 * predicted detection rate matches the detector's. See the calibration study.
 */
export const POD_CALIBRATION = 1;

const TAN_Y = Math.tan(GIMBAL_FOV_Y / 2);
const TAN_X = TAN_Y * GIMBAL_ASPECT;

export interface SensorConditions {
  timeOfDay: number;
  sensorMode: SensorMode;
  flashlightOn: boolean;
}

/** Ground half extents of the image at a height above the person's center: along the nose, across it. */
export function frameHalfExtents(h: number): { along: number; across: number } {
  return { along: h * TAN_Y, across: h * TAN_X };
}

/**
 * One frame's chance of a person contact for a camera `h` meters above the
 * person's center, the person at image-frame offset (along, across), the
 * camera moving at `speed`, `occlusion` of the sightline blocked (0..1).
 */
export function framePod(h: number, along: number, across: number, speed: number, occlusion: number, sensor: SensorConditions): number {
  if (h <= 0.3) return 0;
  if (Math.abs(along) > h * TAN_Y || Math.abs(across) > h * TAN_X) return 0;
  const dist = Math.sqrt(along * along + across * across + h * h);
  const angle = Math.acos(h / dist);
  const illumination = illuminationOf({
    timeOfDay: sensor.timeOfDay,
    interiorOn: false,
    flashlightOn: sensor.flashlightOn,
    inFlashlight: sensor.flashlightOn && angle <= FLASH_HALF,
    sensorMode: sensor.sensorMode,
  });
  const area = PERSON_AREA_M2 / (4 * h * h * TAN_X * TAN_Y);
  const p = recallProbability({
    classId: 'person',
    distance: dist / GIMBAL_RANGE_SCALE,
    occlusion,
    illumination,
    speed: 0,
    cameraSpeed: speed,
    area,
  });
  return Math.min(0.985, p * POD_CALIBRATION);
}

/**
 * Effective sweep width of a straight pass, meters: the integral over
 * lateral offset of the chance the person is detected at least once while
 * the camera flies past. The classical lane spacing for a coverage factor of
 * one (Koopman, "Search and Screening", 1946; the IAMSAR manual's sweep width).
 */
export function sweepWidth(h: number, speed: number, framePeriod: number, sensor: SensorConditions): number {
  const { along, across } = frameHalfExtents(h);
  const step = Math.max(0.05, speed * framePeriod);
  let width = 0;
  const dx = 0.25;
  for (let lateral = -across + dx / 2; lateral < across; lateral += dx) {
    let miss = 1;
    for (let a = -along; a <= along; a += step) miss *= 1 - framePod(h, a, lateral, speed, 0, sensor);
    width += (1 - miss) * dx;
  }
  return width;
}

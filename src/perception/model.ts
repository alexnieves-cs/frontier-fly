import { skyFromTime } from '../sim/lighting';
import type { SensorMode } from '../shared/types';
import type { TargetClass } from '../targets/types';
import { clamp } from './vec';

/** Calibrated per-class sensor parameters. Distances are meters. */
export interface ClassProfile {
  /** Distance where recall is 0.5 in clear daylight, with no occlusion and no blur. */
  d50: number;
  dWidth: number;
  /** Exponent on illumination. Small objects are punished harder after dark. */
  lightSensitivity: number;
  /** Confidence when the target is well resolved. */
  confidence: number;
  /** Fraction of the frame below which recall falls off. */
  minArea: number;
  /** Motion-blur weight. */
  blur: number;
}

export const CLASS_PROFILE: Record<TargetClass, ClassProfile> = {
  person: { d50: 9, dWidth: 2, lightSensitivity: 1, confidence: 0.92, minArea: 0.0012, blur: 1.4 },
  backpack: { d50: 6.5, dWidth: 1.5, lightSensitivity: 1.15, confidence: 0.88, minArea: 0.0007, blur: 1.1 },
  phone: { d50: 3.2, dWidth: 0.75, lightSensitivity: 1.75, confidence: 0.8, minArea: 0.00022, blur: 1.6 },
  toolbox: { d50: 7, dWidth: 1.6, lightSensitivity: 1.05, confidence: 0.87, minArea: 0.0007, blur: 1.1 },
  medkit: { d50: 6, dWidth: 1.4, lightSensitivity: 1.1, confidence: 0.86, minArea: 0.00055, blur: 1.1 },
  bottle: { d50: 4, dWidth: 1, lightSensitivity: 1.35, confidence: 0.78, minArea: 0.00028, blur: 1.2 },
  radio: { d50: 3.6, dWidth: 0.9, lightSensitivity: 1.5, confidence: 0.78, minArea: 0.00028, blur: 1.3 },
  helmet: { d50: 5.5, dWidth: 1.3, lightSensitivity: 1.15, confidence: 0.84, minArea: 0.00045, blur: 1.1 },
  blanket: { d50: 7.5, dWidth: 1.7, lightSensitivity: 1.05, confidence: 0.84, minArea: 0.0009, blur: 0.8 },
  mannequin: { d50: 9, dWidth: 2, lightSensitivity: 1, confidence: 0.7, minArea: 0.0012, blur: 0.4 },
  coat: { d50: 7, dWidth: 1.8, lightSensitivity: 1.1, confidence: 0.62, minArea: 0.0009, blur: 0.3 },
  decoy_pack: { d50: 6.5, dWidth: 1.5, lightSensitivity: 1.15, confidence: 0.6, minArea: 0.0007, blur: 0.4 },
  decoy_case: { d50: 7, dWidth: 1.6, lightSensitivity: 1.05, confidence: 0.6, minArea: 0.0007, blur: 0.4 },
};

/**
 * Probability a well-seen decoy is emitted as its lookalike class.
 * Multiplied by the lookalike's visibility factors, so false positives
 * sit on the decoy and fall off with distance, darkness, and occlusion.
 */
export const DECOY_FP_BASE: Record<'mannequin' | 'coat' | 'decoy_pack' | 'decoy_case', number> = {
  mannequin: 0.58,
  coat: 0.2,
  decoy_pack: 0.48,
  decoy_case: 0.42,
};

/** Conditional label given that a real target was detected. Rows sum to 1. */
export const CONFUSION: Record<TargetClass, Partial<Record<TargetClass, number>>> = {
  person: { person: 0.93, mannequin: 0.05, coat: 0.02 },
  backpack: { backpack: 0.9, decoy_pack: 0.07, toolbox: 0.03 },
  phone: { phone: 0.88, radio: 0.1, bottle: 0.02 },
  toolbox: { toolbox: 0.88, decoy_case: 0.07, medkit: 0.05 },
  medkit: { medkit: 0.9, toolbox: 0.08, decoy_case: 0.02 },
  bottle: { bottle: 0.92, phone: 0.05, radio: 0.03 },
  radio: { radio: 0.9, phone: 0.08, bottle: 0.02 },
  helmet: { helmet: 0.94, backpack: 0.06 },
  blanket: { blanket: 0.95, backpack: 0.05 },
  mannequin: { mannequin: 1 },
  coat: { coat: 1 },
  decoy_pack: { decoy_pack: 1 },
  decoy_case: { decoy_case: 1 },
};

/** Half-angle of the drone flashlight, radians. */
export const FLASH_HALF = (22 * Math.PI) / 180;

/** A cell is not "seen" for confirm-clear below this illumination. */
export const SEEN_LIGHT_MIN = 0.34;
export const SEEN_OCCLUSION_MAX = 0.55;
export const RESOLVE_DISTANCE = { person: 12, object: 8 } as const;

export interface RecallInput {
  classId: TargetClass;
  distance: number;
  occlusion: number;
  illumination: number;
  speed: number;
  cameraSpeed: number;
  /** Clipped bbox area as a fraction of the frame. Use 1 for a pure distance curve. */
  area: number;
}

/** Analytic recall before the Bernoulli draw. Deterministic. */
export function recallProbability(input: RecallInput): number {
  const profile = CLASS_PROFILE[input.classId];
  const dist = 1 / (1 + Math.exp((input.distance - profile.d50) / profile.dWidth));
  const occ = Math.pow(Math.max(0, 1 - input.occlusion), 1.35);
  const light = Math.pow(clamp(input.illumination, 0.02, 1), profile.lightSensitivity);
  const blur = (input.speed + input.cameraSpeed) / Math.max(input.distance, 0.4);
  const motion = 1 / (1 + profile.blur * blur);
  const area = input.area < profile.minArea ? input.area / profile.minArea : 1;
  return clamp(dist * occ * light * motion * area, 0, 0.985);
}

export function illuminationOf(args: {
  timeOfDay: number;
  interiorOn: boolean;
  flashlightOn: boolean;
  inFlashlight: boolean;
  sensorMode: SensorMode;
}): number {
  const night = skyFromTime(args.timeOfDay).night;
  let illum = 1 - night * 0.92;
  if (args.interiorOn) illum = Math.max(illum, 0.75);
  if (args.flashlightOn && args.inFlashlight) illum = Math.max(illum, 0.82);
  if (args.sensorMode === 'ir') illum = Math.max(illum, 0.8);
  if (args.sensorMode === 'depth') illum = Math.max(illum, 0.95);
  return clamp(illum, 0.05, 1);
}

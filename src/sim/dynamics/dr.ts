import type { DynParams, NoiseParams } from '../../shared/types';
import { GRAVITY } from '../../shared/constants';
import type { RNG } from '../../shared/rng';

const HOVER_W = 700;

function cloneDyn(base: DynParams): DynParams {
  return {
    mass: base.mass,
    inertia: [base.inertia[0], base.inertia[1], base.inertia[2]],
    arm: base.arm,
    kf: base.kf,
    km: base.km,
    motorTau: base.motorTau,
    dragLin: [base.dragLin[0], base.dragLin[1], base.dragLin[2]],
    dragQuad: [base.dragQuad[0], base.dragQuad[1], base.dragQuad[2]],
  };
}

function scalePos(rng: RNG, value: number, scale: number): number {
  const factor = 1 + rng.range(-0.2, 0.2) * scale;
  const out = value * factor;
  return out > 1e-9 ? out : 1e-9;
}

/** Hover rotor speed is about 700 rad/s for the given mass. */
export function defaultDynParams(mass = 0.45): DynParams {
  const hoverForcePerMotor = (mass * GRAVITY) / 4;
  const kf = hoverForcePerMotor / (HOVER_W * HOVER_W);
  const km = kf * 0.02;
  return {
    mass,
    inertia: [0.0022, 0.0024, 0.0036],
    arm: 0.1,
    kf,
    km,
    motorTau: 0.03,
    dragLin: [0.12, 0.12, 0.18],
    dragQuad: [0.05, 0.05, 0.06],
  };
}

export function zeroNoise(): NoiseParams {
  return {
    gyroSigma: 0,
    accelSigma: 0,
    biasRW: 0,
    baroSigma: 0,
    lidarSigma: 0,
    lidarDropout: 0,
    latencyTicks: 0,
    lowLightImu: 1,
  };
}

/**
 * scale 0 copies `base` and does not draw random numbers.
 * Otherwise mass, kf, km, motorTau, and each drag coefficient are multiplied by
 * (1 + U(-0.2, 0.2) * scale). Inertia scales with mass. Arm is unchanged.
 */
export function sampleDynParams(rng: RNG, base: DynParams, scale: number): DynParams {
  if (scale === 0) return cloneDyn(base);
  const mass = scalePos(rng, base.mass, scale);
  const ratio = base.mass > 1e-9 ? mass / base.mass : 1;
  return {
    mass,
    inertia: [base.inertia[0] * ratio, base.inertia[1] * ratio, base.inertia[2] * ratio],
    arm: base.arm,
    kf: scalePos(rng, base.kf, scale),
    km: scalePos(rng, base.km, scale),
    motorTau: scalePos(rng, base.motorTau, scale),
    dragLin: [
      scalePos(rng, base.dragLin[0], scale),
      scalePos(rng, base.dragLin[1], scale),
      scalePos(rng, base.dragLin[2], scale),
    ],
    dragQuad: [
      scalePos(rng, base.dragQuad[0], scale),
      scalePos(rng, base.dragQuad[1], scale),
      scalePos(rng, base.dragQuad[2], scale),
    ],
  };
}

/**
 * scale 0 is zero noise (lowLightImu is still 1.5 when `lowLight` is set).
 * Does not draw random numbers when scale is 0.
 */
export function sampleNoise(rng: RNG, scale: number, lowLight: boolean): NoiseParams {
  const lowLightImu = lowLight ? 1.5 : 1;
  if (scale === 0) {
    return {
      gyroSigma: 0,
      accelSigma: 0,
      biasRW: 0,
      baroSigma: 0,
      lidarSigma: 0,
      lidarDropout: 0,
      latencyTicks: 0,
      lowLightImu,
    };
  }
  const u = (): number => rng.range(0.5, 2);
  return {
    gyroSigma: 0.01 * u() * scale,
    accelSigma: 0.05 * u() * scale,
    biasRW: 0.001 * u() * scale * lowLightImu,
    baroSigma: 0.1 * u() * scale,
    lidarSigma: 0.02 * u() * scale,
    lidarDropout: 0.01 * u() * scale,
    latencyTicks: scale < 0.5 ? 0 : rng.int(3),
    lowLightImu,
  };
}

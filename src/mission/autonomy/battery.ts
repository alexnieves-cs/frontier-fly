import type { BatteryModel, V3 } from './contracts';

/**
 * Small-quad energy model from rotor momentum theory.
 *
 * Airframe defaults describe a 1.2 kg quad on 10 inch props with a 4S
 * 5000 mAh pack (14.8 V x 5 Ah = 74 Wh). Sources:
 * - Ideal hover power P = T^1.5 / sqrt(2 rho A): Leishman, Principles of
 *   Helicopter Aerodynamics, 2nd ed., ch. 2 (momentum theory).
 * - Forward-flight induced velocity from Glauert's relation
 *   v_i^2 (V^2 + v_i^2) = v_h^4 (Leishman ch. 2, level flight, small tilt).
 * - Small-rotor figure of merit 0.5-0.7 (Bohorquez et al., "Rotor design for
 *   MAVs", JAHS 2010); 0.6 used here.
 * - Motor plus ESC efficiency about 0.75 at hover throttle (typical outrunner
 *   bench data); 4S LiPo usable energy about 85% of nominal for a long pack
 *   life (do not drain below ~3.5 V per cell).
 * - Parasite drag area CdA about 0.03 m^2 for a ~0.06 m^2 frontal body at
 *   Cd ~0.5.
 *
 * Percentages (remainingPct, usedPct, costPct) are of the usable energy, the
 * number a flight controller's battery gauge shows. Everything is
 * deterministic: no noise, no temperature or sag model.
 */

export const GRAVITY = 9.81;
/** Sea-level ISA air density, kg/m^3. */
export const AIR_DENSITY = 1.225;

export interface BatteryConfig {
  /** All-up mass, kg. */
  massKg: number;
  rotors: number;
  rotorDiameterM: number;
  /** Ideal over actual rotor power at hover, 0..1. */
  figureOfMerit: number;
  /** Shaft power over electrical power (motors and ESCs), 0..1. */
  electricalEfficiency: number;
  /** Flight controller, radios, camera, LEDs, watts. */
  avionicsW: number;
  /** Parasite drag area, m^2. */
  cdaM2: number;
  /** Nominal pack energy, Wh. */
  capacityWh: number;
  /** Usable fraction of the nominal pack, 0..1. */
  usableFraction: number;
  /** Cruise ground speed used by costPct, m/s. */
  cruiseMps: number;
  /** Climb and descent rate used by costPct, m/s. */
  climbMps: number;
  airDensity: number;
  /** Fraction of m g |vz| that descent saves, 0..1. Descent is never free. */
  descentRecovery: number;
}

export const DEFAULT_BATTERY: BatteryConfig = {
  massKg: 1.2,
  rotors: 4,
  rotorDiameterM: 0.254,
  figureOfMerit: 0.6,
  electricalEfficiency: 0.75,
  avionicsW: 8,
  cdaM2: 0.03,
  capacityWh: 74,
  usableFraction: 0.85,
  cruiseMps: 8,
  climbMps: 2,
  airDensity: AIR_DENSITY,
  descentRecovery: 0.3,
};

/** Total rotor disk area, m^2. */
export function diskArea(config: BatteryConfig): number {
  const r = config.rotorDiameterM / 2;
  return config.rotors * Math.PI * r * r;
}

/** Electrical power for a velocity and acceleration with a given config, watts. */
export function powerFor(config: BatteryConfig, vel: V3, accel: V3): number {
  const m = config.massKg;
  const rho = config.airDensity;
  const area = diskArea(config);
  const eta = config.electricalEfficiency;
  const speed = Math.hypot(vel[0], vel[1], vel[2]);
  const hSpeed = Math.hypot(vel[0], vel[1]);
  // Drag opposes velocity; thrust must cancel gravity, drag, and supply acceleration.
  const dragK = 0.5 * rho * config.cdaM2 * speed;
  const tx = m * accel[0] + dragK * vel[0];
  const ty = m * accel[1] + dragK * vel[1];
  const tz = m * (GRAVITY + accel[2]) + dragK * vel[2];
  const thrust = Math.max(0.2 * m * GRAVITY, Math.hypot(tx, ty, tz));
  // Glauert: v_i^2 = (-V^2 + sqrt(V^4 + 4 v_h^4)) / 2, with v_h the hover induced velocity.
  const vh2 = thrust / (2 * rho * area);
  const vi = Math.sqrt((-hSpeed * hSpeed + Math.sqrt(hSpeed ** 4 + 4 * vh2 * vh2)) / 2);
  const induced = (thrust * vi) / config.figureOfMerit;
  const parasite = 0.5 * rho * config.cdaM2 * speed ** 3;
  const vz = vel[2];
  const climb = vz >= 0 ? m * GRAVITY * vz : m * GRAVITY * vz * config.descentRecovery;
  // Kinetic energy rate. No regenerative braking.
  const kinetic = Math.max(0, m * (accel[0] * vel[0] + accel[1] * vel[1] + accel[2] * vel[2]));
  const shaft = Math.max(0.3 * induced, induced + parasite + climb + kinetic);
  return shaft / eta + config.avionicsW;
}

/** Electrical hover power, watts. */
export function hoverPowerW(config: BatteryConfig = DEFAULT_BATTERY): number {
  return powerFor(config, [0, 0, 0], [0, 0, 0]);
}

/** Minutes of hover on a full usable pack. */
export function hoverEnduranceMin(config: BatteryConfig = DEFAULT_BATTERY): number {
  return ((config.capacityWh * config.usableFraction) / hoverPowerW(config)) * 60;
}

/** Energy to fly `distanceM` at cruise then climb (or descend) `climbM`, Wh. */
export function legEnergyWh(config: BatteryConfig, distanceM: number, climbM: number): number {
  const d = Math.max(0, distanceM);
  const cruiseW = powerFor(config, [config.cruiseMps, 0, 0], [0, 0, 0]);
  const vz = climbM >= 0 ? config.climbMps : -config.climbMps;
  const climbW = powerFor(config, [0, 0, vz], [0, 0, 0]);
  const sec = d / config.cruiseMps;
  const climbSec = Math.abs(climbM) / config.climbMps;
  return (cruiseW * sec + climbW * climbSec) / 3600;
}

/**
 * Momentum-theory battery. Accepts a partial config, or a bare nominal pack
 * size in Wh for the starter call shape `createBattery(60)`.
 */
export function createBattery(config?: Partial<BatteryConfig> | number): BatteryModel {
  const cfg: BatteryConfig =
    typeof config === 'number' ? { ...DEFAULT_BATTERY, capacityWh: config } : { ...DEFAULT_BATTERY, ...config };
  const usableWh = cfg.capacityWh * cfg.usableFraction;
  let usedWh = 0;
  return {
    capacityWh: cfg.capacityWh,
    powerFor(vel: V3, accel: V3) {
      return powerFor(cfg, vel, accel);
    },
    draw(powerW, dtSec) {
      if (powerW > 0 && dtSec > 0) usedWh = Math.min(usableWh, usedWh + (powerW * dtSec) / 3600);
    },
    remainingPct() {
      return Math.max(0, 100 - (usedWh / usableWh) * 100);
    },
    usedPct() {
      return (usedWh / usableWh) * 100;
    },
    costPct(distanceM, climbM) {
      return (legEnergyWh(cfg, distanceM, climbM) / usableWh) * 100;
    },
  };
}

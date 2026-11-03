/**
 * A LiPo pack's voltage under load, and PX4's battery estimator reading it.
 *
 * battery.ts counts energy and nothing else: the gauge is the true fraction of
 * usable energy left, and the motors are as strong on an empty pack as on a
 * full one. A real pack sags. Its open-circuit voltage falls with charge, and
 * under load it drops further by I·R across the cells' internal resistance.
 * That costs thrust (motor speed goes with voltage) and it is what a flight
 * controller's gauge reads.
 *
 * - Open-circuit voltage per cell: Chen & Rincón-Mora, "Accurate electrical
 *   battery model capable of predicting runtime and I-V performance", IEEE
 *   Trans. Energy Conversion 21(2), 2006, eq. (3), fitted to a lithium-polymer
 *   cell: V_oc(s) = -1.031 e^(-35 s) + 3.685 + 0.2156 s - 0.1178 s^2 + 0.3201 s^3.
 *   It reads 4.10 V full and falls steeply below about 10%.
 * - Internal resistance: 12 mΩ per cell, an assumption for a new 2420 mAh
 *   pack (resistance scales roughly with 1 / capacity; a 5000 mAh hobby pack
 *   is about 6 mΩ, and PX4's own default, BAT1_R_INTERNAL, is 5 mΩ).
 *
 * `px4StateOfCharge` is PX4 v1.15.4's estimator (src/lib/battery/battery.cpp,
 * Battery::calculateStateOfChargeVoltageBased and estimateStateOfCharge),
 * ported line for line with PX4's default parameters, so the validation can
 * ask what a real PX4 would report on this pack.
 */

import { AIRFRAME } from '../../shared/airframe';

export interface PackConfig {
  cells: number;
  capacityAh: number;
  /** Per cell, ohms. */
  rInternalOhm: number;
}

/** The one airframe's pack (shared/airframe.ts). */
export const MISSION_PACK: PackConfig = { cells: AIRFRAME.pack.cells, capacityAh: AIRFRAME.pack.mAh / 1000, rInternalOhm: 0.012 };

/** Open-circuit cell voltage at state of charge s (0..1). */
export function cellOcv(s: number): number {
  const x = Math.max(0, Math.min(1, s));
  return -1.031 * Math.exp(-35 * x) + 3.685 + 0.2156 * x - 0.1178 * x * x + 0.3201 * x * x * x;
}

/** Cell voltage under a pack current, V. */
export function cellVoltage(pack: PackConfig, s: number, currentA: number): number {
  return cellOcv(s) - currentA * pack.rInternalOhm;
}

/**
 * Pack current for an electrical power draw, A: P = (n V_oc - n R I) I, the
 * smaller root. NaN when the pack cannot deliver that power.
 */
export function packCurrent(pack: PackConfig, s: number, powerW: number): number {
  const voc = pack.cells * cellOcv(s);
  const r = pack.cells * pack.rInternalOhm;
  const disc = voc * voc - 4 * r * powerW;
  if (disc < 0) return Number.NaN;
  return (voc - Math.sqrt(disc)) / (2 * r);
}

/** A full pack's cell voltage under no load: the reference for motor top speed. */
export const FULL_CELL_V = cellOcv(1);

/**
 * Motor top speed on this pack as a fraction of a full, unloaded pack's.
 * A brushless motor turns at about Kv times the voltage it gets.
 */
export function motorSpeedScale(pack: PackConfig, s: number, currentA: number): number {
  return Math.max(0, cellVoltage(pack, s, currentA) / FULL_CELL_V);
}

/** PX4 v1.15.4 battery parameter defaults (src/lib/battery/module.yaml). */
export const PX4_BATTERY_DEFAULTS = { vEmpty: 3.6, vCharged: 4.05, rInternal: 0.005, vLoadDrop: 0.1 };

/** PX4's voltage-based state of charge from pack voltage and current. */
export function px4VoltageSoc(cells: number, voltageV: number, currentA: number, p = PX4_BATTERY_DEFAULTS): number {
  let cell = voltageV / cells;
  if (p.rInternal >= 0 && currentA > 1e-7) cell += p.rInternal * currentA;
  const t = (cell - p.vEmpty) / (p.vCharged - p.vEmpty);
  return Math.max(0, Math.min(1, t));
}

/**
 * PX4's reported state of charge over a flight, with the capacity known: it
 * starts from the voltage estimate, counts coulombs, and leans toward the
 * voltage estimate as the voltage gets low. Samples are at PX4's battery rate.
 */
export class Px4BatteryEstimator {
  private soc = -1;
  private dischargedMah = 0;
  private vFilt = 0;
  private iFilt = 0;
  constructor(
    private readonly cells: number,
    private readonly capacityMah: number,
    private readonly p = PX4_BATTERY_DEFAULTS,
  ) {}

  update(voltageV: number, currentA: number, dt: number): number {
    if (this.soc < 0) {
      this.vFilt = voltageV;
      this.iFilt = currentA;
    } else {
      // PX4's AlphaFilter with time constants 1 s (voltage) and 0.5 s (current).
      this.vFilt += (dt / (dt + 1)) * (voltageV - this.vFilt);
      this.iFilt += (dt / (dt + 0.5)) * (currentA - this.iFilt);
    }
    const loopMah = currentA * 1e3 * (dt / 3600);
    this.dischargedMah += loopMah;
    const vSoc = px4VoltageSoc(this.cells, this.vFilt, this.iFilt, this.p);
    if (this.soc < 0) {
      this.soc = vSoc;
      return this.soc;
    }
    const w = 3e-2 * (1 - vSoc);
    this.soc = (1 - w) * this.soc + w * vSoc;
    this.soc -= loopMah / this.capacityMah;
    this.soc = Math.max(this.soc, 0);
    this.soc = Math.min(Math.max(1 - this.dischargedMah / this.capacityMah, 0), this.soc);
    return this.soc;
  }
}

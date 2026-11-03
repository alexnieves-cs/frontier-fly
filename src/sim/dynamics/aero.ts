/**
 * Near-surface aerodynamics and battery sag for QuadrotorDynamics, opt in.
 *
 * QuadrotorDynamics without an AeroModel keeps the model every checkpoint and
 * benchmark was made with: thrust grows linearly by 8% per meter below 0.4 m
 * of world height (`legacyGroundEffect`), there is no ceiling effect, and the
 * motors reach the same top speed on a full pack and an empty one. The
 * physics validation (docs/PHYSICS_VALIDATION.md) compares that with
 * published rotor measurements and found it far too weak near the ground and
 * missing the ceiling entirely, so this module holds the measured models:
 *
 * - Ground effect: Cheeseman & Bennett, "The effect of the ground on a
 *   helicopter rotor in forward flight", ARC R&M 3021 (1955), in hover:
 *   T_IGE / T_OGE = 1 / (1 - (R / 4z)^2). Sanchez-Cuevas, Heredia & Ollero,
 *   "Experimental approach to the aerodynamic effects produced in multirotors
 *   flying close to obstacles" (ROBOT 2017), measured it on small multirotor
 *   rotors and found the classical model holds (about 1.2 at z/R = 0.5).
 * - Ceiling effect: the same group's fit to a 9.4 inch rotor under a ceiling,
 *   "Multirotor UAS for bridge inspection by contact using the ceiling effect"
 *   (ICUAS 2017): T_ICE / T_OCE = 1 / (1 - (1 / K1) (R / (z + K2))^2) with
 *   K1 = 6.924 and K2 = 3.782 cm. Their rotor radius is 11.94 cm, so here K2
 *   is scaled with the rotor: K2 = 0.3168 R.
 *
 * z is the distance from the rotor plane to the surface. Both models diverge
 * as z goes to zero, past where they were measured, so each is held at its
 * value at a floor distance (z/R = 0.5 ground, 0.2 ceiling).
 */

/** The linear ground effect QuadrotorDynamics has always used: world height, not height above ground. */
export function legacyGroundEffect(worldZ: number): number {
  if (worldZ >= 0.4) return 1;
  const depth = worldZ > 0 ? 0.4 - worldZ : 0.4;
  return 1 + 0.08 * depth;
}

const GROUND_MIN_ZR = 0.5;
const CEILING_K1 = 6.924;
/** 3.782 cm on an 11.94 cm rotor. */
const CEILING_K2_R = 3.782 / 11.94;
const CEILING_MIN_ZR = 0.2;

/** Cheeseman-Bennett hover ground effect for a rotor of radius `r` at `z` above the ground. */
export function groundEffect(z: number, r: number): number {
  if (!(z > 0) || !(r > 0)) return groundEffect(GROUND_MIN_ZR * r, r);
  const zr = Math.max(GROUND_MIN_ZR, z / r);
  const k = 1 / (4 * zr);
  return 1 / (1 - k * k);
}

/** Sanchez-Cuevas ceiling effect for a rotor of radius `r` at `z` below the ceiling. */
export function ceilingEffect(z: number, r: number): number {
  if (!(r > 0)) return 1;
  const zr = Math.max(CEILING_MIN_ZR, Number.isFinite(z) ? z / r : Infinity);
  if (!Number.isFinite(zr)) return 1;
  const k = 1 / (zr + CEILING_K2_R);
  return 1 / (1 - (k * k) / CEILING_K1);
}

/** Distances from the rotor plane to the nearest surface below and above, m. Infinity where there is none. */
export interface Clearance {
  below: number;
  above: number;
}

export interface AeroOptions {
  /** Rotor radius, m. */
  rotorRadiusM: number;
  /** Rotor plane height above the body center, m. */
  rotorOffsetM?: number;
  /** Surfaces around a body-center position (x, y, z up). Default: flat ground at z = 0, open sky. */
  clearance?: (x: number, y: number, z: number) => Clearance;
  /** Ground effect on (default) or off. */
  ground?: boolean;
  /** Ceiling effect on (default) or off. */
  ceiling?: boolean;
}

export class AeroModel {
  readonly rotorRadiusM: number;
  readonly rotorOffsetM: number;
  readonly ground: boolean;
  readonly ceiling: boolean;
  clearance: (x: number, y: number, z: number) => Clearance;
  /**
   * Motor top speed as a fraction of a full pack's, 0..1. A brushless motor's
   * speed scales with the voltage the pack holds under load, so a sagging pack
   * lowers the thrust ceiling (thrust goes with speed squared). Set it from a
   * pack model each step (src/mission/autonomy/pack.ts).
   */
  motorMaxScale = 1;
  private x = 0;
  private y = 0;

  constructor(o: AeroOptions) {
    this.rotorRadiusM = o.rotorRadiusM;
    this.rotorOffsetM = o.rotorOffsetM ?? 0;
    this.ground = o.ground ?? true;
    this.ceiling = o.ceiling ?? true;
    this.clearance = o.clearance ?? ((_x, _y, z) => ({ below: z, above: Infinity }));
  }

  /** Where the vehicle is horizontally, for the clearance lookup. QuadrotorDynamics passes only its height. */
  setPosition(x: number, y: number): void {
    this.x = x;
    this.y = y;
  }

  /** Thrust multiplier at body-center height z. */
  thrustFactor(z: number): number {
    const c = this.clearance(this.x, this.y, z);
    const r = this.rotorRadiusM;
    let k = 1;
    if (this.ground) k *= groundEffect(c.below + this.rotorOffsetM, r);
    if (this.ceiling) k *= ceilingEffect(c.above - this.rotorOffsetM, r);
    return k;
  }
}

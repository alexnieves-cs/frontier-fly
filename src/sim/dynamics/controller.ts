import type { DynParams } from '../../shared/types';
import { GRAVITY, MAX_SPEED_H, MAX_SPEED_V, MAX_YAW_RATE } from '../../shared/constants';

/** Direct P gains. Positive pitch is nose-down, so +vx error commands +pitch. */
const VEL_KP = 2.2;
const ALT_KP = 8;
const ALT_KI = 4;
const ATT_KP = 12;
const RATE_KP = 0.18;
const RATE_KD = 0.01;
const YAW_RATE_KP = 0.15;
const MAX_TILT = 0.4;
/**
 * Body-rate setpoint cap, rad/s (about 230°/s, near PX4's 220°/s default).
 * At 8 rad/s a hard reversal asked for more differential thrust than the
 * motors have, the mixer clipped it out of the collective, and light-thrust
 * airframes flipped.
 */
const MAX_RATE = 4;
const MOTOR_MAX = 1100;
const AZ_LIM = 8;

function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}

function wrapPi(a: number): number {
  const pi = Math.PI;
  if (a >= -pi && a <= pi) return a;
  const t = a + pi;
  const w = t - Math.floor(t / (2 * pi)) * (2 * pi);
  return w - pi;
}

/**
 * Angle-mode cascade: body velocity → tilt, altitude → collective,
 * attitude P → rate, rate P/D → torque, then X-mixer to rotor speeds.
 * Allocation-free; writes `wCmd`.
 */
export class AngleController {
  private mass = 0.45;
  private kf = 1e-5;
  private a = 0.0707;
  private b = 0.02;
  private fMax = 1;
  private integZ = 0;

  setParams(p: DynParams): void {
    this.mass = p.mass > 1e-6 ? p.mass : 1e-6;
    this.kf = p.kf > 1e-12 ? p.kf : 1e-12;
    const arm = p.arm > 1e-4 ? p.arm : 1e-4;
    this.a = arm * Math.SQRT1_2;
    const ratio = p.km / this.kf;
    this.b = Math.abs(ratio) > 1e-6 ? ratio : ratio < 0 ? -1e-6 : 1e-6;
    this.fMax = this.kf * MOTOR_MAX * MOTOR_MAX;
  }

  reset(): void {
    this.integZ = 0;
  }

  compute(
    roll: number,
    pitch: number,
    yaw: number,
    vx: number,
    vy: number,
    vz: number,
    wx: number,
    wy: number,
    wz: number,
    cmd: Float32Array,
    dt: number,
    wCmd: Float32Array,
  ): void {
    const vxCmd = clamp(cmd[0] ?? 0, -MAX_SPEED_H, MAX_SPEED_H);
    const vyCmd = clamp(cmd[1] ?? 0, -MAX_SPEED_H, MAX_SPEED_H);
    const vzCmd = clamp(cmd[2] ?? 0, -MAX_SPEED_V, MAX_SPEED_V);
    const yawCmd = clamp(cmd[3] ?? 0, -MAX_YAW_RATE, MAX_YAW_RATE);

    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const vxB = vx * cy + vy * sy;
    const vyB = -vx * sy + vy * cy;

    const pitchDes = clamp(VEL_KP * (vxCmd - vxB), -MAX_TILT, MAX_TILT);
    const rollDes = clamp(-VEL_KP * (vyCmd - vyB), -MAX_TILT, MAX_TILT);

    const evz = vzCmd - vz;
    let az = ALT_KP * evz + ALT_KI * this.integZ;
    let azSat = false;
    if (az > AZ_LIM) {
      az = AZ_LIM;
      azSat = true;
    } else if (az < -AZ_LIM) {
      az = -AZ_LIM;
      azSat = true;
    }

    const cosTilt = Math.cos(roll) * Math.cos(pitch);
    const denom = cosTilt > 0.5 ? cosTilt : 0.5;
    const thrustMax = 4 * this.fMax;
    let thrust = (this.mass * (GRAVITY + az)) / denom;
    let thrustSat = false;
    if (thrust > thrustMax) {
      thrust = thrustMax;
      thrustSat = true;
    } else if (thrust < 0) {
      thrust = 0;
      thrustSat = true;
    }
    const sat = azSat || thrustSat;
    if (!sat || (az >= AZ_LIM && evz < 0) || (az <= -AZ_LIM && evz > 0)) {
      this.integZ += evz * dt;
      if (this.integZ > 2) this.integZ = 2;
      else if (this.integZ < -2) this.integZ = -2;
    }

    const pSp = clamp(ATT_KP * wrapPi(rollDes - roll), -MAX_RATE, MAX_RATE);
    const qSp = clamp(ATT_KP * wrapPi(pitchDes - pitch), -MAX_RATE, MAX_RATE);
    const tx = RATE_KP * (pSp - wx) - RATE_KD * wx;
    const ty = RATE_KP * (qSp - wy) - RATE_KD * wy;
    const tz = YAW_RATE_KP * (yawCmd - wz) - RATE_KD * wz;

    this.mix(thrust, tx, ty, tz, wCmd);
  }

  private mix(thrust: number, tx: number, ty: number, tz: number, wCmd: Float32Array): void {
    const inv4 = 0.25;
    const inv4a = inv4 / this.a;
    const inv4b = inv4 / this.b;
    const fMax = this.fMax;
    let avg = thrust * inv4;
    if (avg > fMax) avg = fMax;
    else if (avg < 0) avg = 0;

    let d0 = (-tx - ty) * inv4a + tz * inv4b;
    let d1 = (-tx + ty) * inv4a - tz * inv4b;
    let d2 = (tx + ty) * inv4a + tz * inv4b;
    let d3 = (tx - ty) * inv4a - tz * inv4b;

    let scale = 1;
    if (d0 > 1e-12) {
      const s = (fMax - avg) / d0;
      if (s < scale) scale = s;
    } else if (d0 < -1e-12) {
      const s = (0 - avg) / d0;
      if (s < scale) scale = s;
    }
    if (d1 > 1e-12) {
      const s = (fMax - avg) / d1;
      if (s < scale) scale = s;
    } else if (d1 < -1e-12) {
      const s = (0 - avg) / d1;
      if (s < scale) scale = s;
    }
    if (d2 > 1e-12) {
      const s = (fMax - avg) / d2;
      if (s < scale) scale = s;
    } else if (d2 < -1e-12) {
      const s = (0 - avg) / d2;
      if (s < scale) scale = s;
    }
    if (d3 > 1e-12) {
      const s = (fMax - avg) / d3;
      if (s < scale) scale = s;
    } else if (d3 < -1e-12) {
      const s = (0 - avg) / d3;
      if (s < scale) scale = s;
    }
    if (scale < 0) scale = 0;
    if (scale < 1) {
      d0 *= scale;
      d1 *= scale;
      d2 *= scale;
      d3 *= scale;
    }

    const kf = this.kf;
    wCmd[0] = Math.sqrt(Math.max(avg + d0, 0) / kf);
    wCmd[1] = Math.sqrt(Math.max(avg + d1, 0) / kf);
    wCmd[2] = Math.sqrt(Math.max(avg + d2, 0) / kf);
    wCmd[3] = Math.sqrt(Math.max(avg + d3, 0) / kf);
  }
}

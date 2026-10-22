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
  private speedH = MAX_SPEED_H;
  private speedV = MAX_SPEED_V;
  private yawRateMax = MAX_YAW_RATE;
  private gain = 1;
  private maxTilt = MAX_TILT;
  /** A tilt limit was set: clamp the tilt vector. The default keeps the per-axis clamp every checkpoint was trained on. */
  private tiltNorm = false;
  /** Give up collective before roll and pitch when the motors saturate. Off keeps the mixer the checkpoints were trained on. */
  private attitudeFirst = false;
  private readonly rp = new Float64Array(4);
  private velKi = 0;
  private readonly velI = new Float64Array(2);

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
    this.velI[0] = 0;
    this.velI[1] = 0;
  }

  /**
   * Integral gain on the horizontal velocity error, rad of tilt per (m/s·s), as PX4's MPC_XY_VEL_I. Off (0)
   * the velocity loop is P only, the loop every checkpoint was trained on; in a steady wind that leaves the
   * drone drifting downwind at the speed whose error buys the tilt. Acts only with a tilt limit set.
   */
  setVelocityIntegral(ki: number): void {
    this.velKi = ki;
    if (ki <= 0) {
      this.velI[0] = 0;
      this.velI[1] = 0;
    }
  }

  /**
   * Setpoint caps for a different airframe (the mission quad flies faster
   * outdoors than the coverage env's micro quad) and a scale on the rate-loop
   * gains, which a flight controller tunes to the airframe's inertia, and
   * the tilt limit (PX4's MPC_TILTMAX_AIR), which sets the strongest wind
   * the airframe can hold position in.
   */
  setLimits(speedH: number, speedV: number, yawRate: number, rateGain = 1, maxTilt?: number): void {
    this.speedH = speedH;
    this.speedV = speedV;
    this.yawRateMax = yawRate;
    this.gain = rateGain;
    this.tiltNorm = maxTilt !== undefined;
    this.maxTilt = maxTilt ?? MAX_TILT;
  }

  /**
   * Mixer priority. On, a saturated mixer lowers (or raises) the collective to keep the full
   * roll/pitch/yaw differential, as PX4's mixer does; off, it scales the differential down instead,
   * which at full climb thrust leaves no attitude authority and the airframe runs away tilted.
   */
  setAttitudeFirst(on: boolean): void {
    this.attitudeFirst = on;
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
    const vxCmd = clamp(cmd[0] ?? 0, -this.speedH, this.speedH);
    const vyCmd = clamp(cmd[1] ?? 0, -this.speedH, this.speedH);
    const vzCmd = clamp(cmd[2] ?? 0, -this.speedV, this.speedV);
    const yawCmd = clamp(cmd[3] ?? 0, -this.yawRateMax, this.yawRateMax);

    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const vxB = vx * cy + vy * sy;
    const vyB = -vx * sy + vy * cy;

    let pitchDes: number;
    let rollDes: number;
    if (this.tiltNorm) {
      // A set tilt limit bounds the tilt vector, as PX4 does: clamping each axis alone lets roll and pitch
      // together lean the airframe ~40% past it, where the collective can no longer hold height.
      const exB = vxCmd - vxB;
      const eyB = vyCmd - vyB;
      // The integral lives in the world frame, where a steady wind is steady, and turns into the body frame.
      const ix = this.velI[0] * cy + this.velI[1] * sy;
      const iy = -this.velI[0] * sy + this.velI[1] * cy;
      pitchDes = VEL_KP * exB + ix;
      rollDes = -(VEL_KP * eyB + iy);
      const n = Math.hypot(pitchDes, rollDes);
      if (n > this.maxTilt) {
        pitchDes *= this.maxTilt / n;
        rollDes *= this.maxTilt / n;
      }
      if (this.velKi > 0) {
        // Anti-windup: hold the integral while the tilt limit clips, unless the error unwinds it.
        const exW = exB * cy - eyB * sy;
        const eyW = exB * sy + eyB * cy;
        const dx = this.velKi * exW * dt;
        const dy = this.velKi * eyW * dt;
        if (n <= this.maxTilt || dx * this.velI[0] + dy * this.velI[1] < 0) {
          this.velI[0] += dx;
          this.velI[1] += dy;
          const m = Math.hypot(this.velI[0], this.velI[1]);
          if (m > this.maxTilt) {
            this.velI[0] *= this.maxTilt / m;
            this.velI[1] *= this.maxTilt / m;
          }
        }
      }
    } else {
      pitchDes = clamp(VEL_KP * (vxCmd - vxB), -MAX_TILT, MAX_TILT);
      rollDes = clamp(-VEL_KP * (vyCmd - vyB), -MAX_TILT, MAX_TILT);
    }

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
    const g = this.gain;
    const tx = g * (RATE_KP * (pSp - wx) - RATE_KD * wx);
    const ty = g * (RATE_KP * (qSp - wy) - RATE_KD * wy);
    const tz = g * (YAW_RATE_KP * (yawCmd - wz) - RATE_KD * wz);

    this.mix(thrust, tx, ty, tz, wCmd);
  }

  private mix(thrust: number, tx: number, ty: number, tz: number, wCmd: Float32Array): void {
    if (this.attitudeFirst) {
      this.mixAttitudeFirst(thrust, tx, ty, tz, wCmd);
      return;
    }
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
  /**
   * PX4's priority: roll and pitch first, moving the collective to make room for them, then yaw with
   * whatever headroom is left. Yaw needs far more differential per unit torque than roll and pitch,
   * so mixed alongside them it takes the motors' range a hard stop needs.
   */
  private mixAttitudeFirst(thrust: number, tx: number, ty: number, tz: number, wCmd: Float32Array): void {
    const fMax = this.fMax;
    const ka = 0.25 / this.a;
    const kb = 0.25 / this.b;
    const rp = this.rp;
    rp[0] = (-tx - ty) * ka;
    rp[1] = (-tx + ty) * ka;
    rp[2] = (tx + ty) * ka;
    rp[3] = (tx - ty) * ka;
    let avg = Math.min(Math.max(thrust * 0.25, 0), fMax);
    let hi = Math.max(rp[0], rp[1], rp[2], rp[3]);
    let lo = Math.min(rp[0], rp[1], rp[2], rp[3]);
    // Roll and pitch need more range than the motors have: keep the direction, scale the size.
    if (hi - lo > fMax) {
      const k = fMax / (hi - lo);
      for (let i = 0; i < 4; i++) rp[i] = rp[i]! * k;
      hi *= k;
      lo *= k;
    }
    if (avg + hi > fMax) avg = fMax - hi;
    if (avg + lo < 0) avg = -lo;
    // Yaw, signs alternating by rotor spin direction, scaled into the room that is left.
    let ys = 1;
    for (let i = 0; i < 4; i++) {
      const y = (i % 2 === 0 ? tz : -tz) * kb;
      const f = avg + rp[i]!;
      if (y > 1e-12) ys = Math.min(ys, (fMax - f) / y);
      else if (y < -1e-12) ys = Math.min(ys, -f / y);
    }
    if (ys < 0) ys = 0;
    const kf = this.kf;
    for (let i = 0; i < 4; i++) {
      const y = (i % 2 === 0 ? tz : -tz) * kb * ys;
      wCmd[i] = Math.sqrt(Math.max(avg + rp[i]! + y, 0) / kf);
    }
  }
}

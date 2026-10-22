import { DRONE_RADIUS, KINEMATIC_TAU } from '../../shared/constants';
import { containsPoint, inNoFly } from '../../world/outdoor/geofence';
import type { OutdoorPhysics } from '../../world/outdoor/physics';
import { sampleHeight } from '../../world/outdoor/terrain';
import type { OutdoorWorld } from '../../world/outdoor/types';
import type { BatteryModel, DronePose, V3 } from './contracts';

/**
 * Deterministic kinematic multirotor for autonomous missions. Velocity
 * setpoints pass through a first-order lag, then acceleration and jerk
 * limits, then speed caps. Motion goes through OutdoorPhysics.move, which
 * slides along solids; a collision is the start of a contact, so sliding
 * along a wall for many ticks counts once. Every tick draws battery power
 * for the current velocity and acceleration.
 *
 * OutdoorPhysics.move steps the Rapier world twice per call, which dominates
 * a headless run. When a bundle of rays along the motion shows open air well
 * past the swept sphere, the step is taken directly with the same geofence,
 * no-fly, and terrain-floor rules; anything close falls through to move().
 */

export interface VehicleLimits {
  /** Horizontal speed cap, m/s. */
  speedH: number;
  /** Vertical speed cap, m/s. */
  speedV: number;
  /** Acceleration magnitude cap, m/s^2. */
  accel: number;
  /** Rate of change of acceleration cap, m/s^3. */
  jerk: number;
  /** Yaw rate cap, rad/s. */
  yawRate: number;
  /** Velocity lag time constant, s. */
  tau: number;
  /** Quad only: the flight controller's tilt-vector limit, rad. Unset keeps the controller's default (23° per axis). */
  maxTilt?: number;
  /** Quad only, with maxTilt: plan leans the attitude rate cap can take back out (calm air; off in wind for gust authority). */
  shapeTilt?: boolean;
}

export const OUTDOOR_LIMITS: VehicleLimits = {
  speedH: 5.5,
  speedV: 1.5,
  accel: 3,
  jerk: 12,
  yawRate: Math.PI / 2,
  tau: KINEMATIC_TAU,
};

export const INDOOR_LIMITS: VehicleLimits = {
  speedH: 1,
  speedV: 0.6,
  accel: 1.5,
  jerk: 8,
  yawRate: Math.PI / 2,
  tau: KINEMATIC_TAU,
};

export interface VehicleCommand {
  /** World-frame velocity setpoint, m/s. */
  vel: V3;
  /** Heading to turn toward at the yaw rate limit. Ignored when yawRate is set. */
  yaw?: number | null;
  /** Explicit yaw rate, rad/s. */
  yawRate?: number;
}

/** Outside influences on one step. */
export interface StepOptions {
  /** Wind, a world-frame air velocity in m/s. The airframe moves relative to it. */
  wind?: V3;
}

/** What an IMU on the airframe would read, noise-free, body frame (x forward, y left, z up). */
export interface ImuReading {
  /** Specific force, m/s²: acceleration minus gravity, so a hover reads [0, 0, +9.81]. */
  accelBody: V3;
  /** Body rates, rad/s. */
  gyroBody: V3;
}

export interface VehicleStep {
  /** A new contact started this tick. */
  collided: boolean;
  /** Any contact this tick (including sliding). */
  touching: boolean;
  powerW: number;
}

/** Contacts closer than this to the previous one belong to the same event, s. */
export const CONTACT_GAP_SEC = 0.5;

const LOOK_MARGIN = 0.6;
const BUNDLE_RADIUS = DRONE_RADIUS + 0.12;

function clampMag2(x: number, y: number, cap: number): [number, number] {
  const m = Math.hypot(x, y);
  if (m <= cap || m < 1e-12) return [x, y];
  return [(x / m) * cap, (y / m) * cap];
}

function wrap(a: number): number {
  const tau = Math.PI * 2;
  return ((a % tau) + tau) % tau;
}

const GRAVITY = 9.81;

/**
 * The mission airframe. The base class is the kinematic multirotor described
 * above; QuadVehicle (quadVehicle.ts) flies the same commands through rigid-body
 * quadrotor dynamics and a flight controller.
 */
export class Vehicle {
  readonly kind: 'kinematic' | 'quad' = 'kinematic';
  pos: V3;
  vel: V3 = [0, 0, 0];
  accel: V3 = [0, 0, 0];
  yaw: number;
  yawRate = 0;
  tSec = 0;
  /** Contact events so far. */
  contacts = 0;
  /** Distance flown, m. */
  odometer = 0;
  private lastTouch = -Infinity;
  /** Thrust authority, 0..1 overall (see setThrustAuthority). */
  protected authority = 1;

  /** Ticks resolved by OutdoorPhysics.move versus the open-air fast path. */
  physicsMoves = 0;
  freeMoves = 0;

  constructor(
    protected readonly physics: OutdoorPhysics,
    readonly battery: BatteryModel,
    start: V3,
    yaw: number,
    protected readonly world: OutdoorWorld | null = null,
  ) {
    this.pos = [start[0], start[1], start[2]];
    this.yaw = wrap(yaw);
  }

  /**
   * Scale the airframe's thrust authority, 0..1: one number for the whole
   * airframe, or one per motor (front-right, rear-right, rear-left,
   * front-left). The kinematic vehicle has no motors, so it uses the weakest
   * motor's value to scale its acceleration and speed caps.
   */
  setThrustAuthority(k: number | readonly number[]): void {
    this.authority = Math.max(0, Math.min(1, typeof k === 'number' ? k : Math.min(...k)));
  }

  /** Noise-free IMU reading for the last step. */
  imu(): ImuReading {
    const c = Math.cos(this.yaw);
    const s = Math.sin(this.yaw);
    const a = this.accel;
    return {
      accelBody: [c * a[0] + s * a[1], -s * a[0] + c * a[1], a[2] + GRAVITY],
      gyroBody: [0, 0, this.yawRate],
    };
  }

  pose(): DronePose {
    return { pos: [this.pos[0], this.pos[1], this.pos[2]], yaw: this.yaw, vel: [this.vel[0], this.vel[1], this.vel[2]] };
  }

  step(cmd: VehicleCommand, dt: number, base: VehicleLimits, opts: StepOptions = {}): VehicleStep {
    const limits = this.authority < 1 ? { ...base, speedH: base.speedH * this.authority, accel: base.accel * this.authority } : base;
    const [cx, cy] = clampMag2(cmd.vel[0], cmd.vel[1], limits.speedH);
    const cz = Math.max(-limits.speedV, Math.min(limits.speedV, cmd.vel[2]));
    // First-order lag gives the desired acceleration; then limit accel and jerk.
    const lag = 1 - Math.exp(-dt / limits.tau);
    let ax = ((cx - this.vel[0]) * lag) / dt;
    let ay = ((cy - this.vel[1]) * lag) / dt;
    let az = ((cz - this.vel[2]) * lag) / dt;
    const am = Math.hypot(ax, ay, az);
    if (am > limits.accel) {
      const s = limits.accel / am;
      ax *= s;
      ay *= s;
      az *= s;
    }
    const dj = limits.jerk * dt;
    ax = this.accel[0] + Math.max(-dj, Math.min(dj, ax - this.accel[0]));
    ay = this.accel[1] + Math.max(-dj, Math.min(dj, ay - this.accel[1]));
    az = this.accel[2] + Math.max(-dj, Math.min(dj, az - this.accel[2]));
    let vx = this.vel[0] + ax * dt;
    let vy = this.vel[1] + ay * dt;
    let vz = this.vel[2] + az * dt;
    [vx, vy] = clampMag2(vx, vy, Math.max(limits.speedH, Math.hypot(this.vel[0], this.vel[1])));
    // Yaw.
    let rate = cmd.yawRate ?? 0;
    if (cmd.yawRate === undefined && cmd.yaw !== undefined && cmd.yaw !== null) {
      let err = wrap(cmd.yaw) - this.yaw;
      if (err > Math.PI) err -= Math.PI * 2;
      if (err < -Math.PI) err += Math.PI * 2;
      rate = err / Math.max(dt, 0.25);
    }
    rate = Math.max(-limits.yawRate, Math.min(limits.yawRate, rate));
    this.yaw = wrap(this.yaw + rate * dt);
    this.yawRate = rate;

    const before: V3 = [this.pos[0], this.pos[1], this.pos[2]];
    const w = opts.wind;
    // The commanded velocity is through the air; the wind carries the airframe with it.
    if (w) {
      vx += w[0];
      vy += w[1];
      vz += w[2];
    }
    const delta: V3 = [vx * dt, vy * dt, vz * dt];
    const moved = this.move(before, delta);
    this.pos = [moved.pos[0], moved.pos[1], moved.pos[2]];
    // The velocity that actually happened after sliding.
    const real: V3 = [(this.pos[0] - before[0]) / dt, (this.pos[1] - before[1]) / dt, (this.pos[2] - before[2]) / dt];
    const blocked = Math.hypot(real[0] - vx, real[1] - vy, real[2] - vz) > 1e-3;
    if (blocked) {
      vx = real[0];
      vy = real[1];
      vz = real[2];
    }
    if (w && !blocked) {
      // Keep the air-relative velocity as the vehicle's state.
      vx -= w[0];
      vy -= w[1];
      vz -= w[2];
    }
    this.accel = [(vx - this.vel[0]) / dt, (vy - this.vel[1]) / dt, (vz - this.vel[2]) / dt];
    if (blocked) this.accel = [ax, ay, az];
    this.vel = [vx, vy, vz];
    return this.finishStep(before, dt, moved.hit);
  }

  /** Bookkeeping shared by every airframe: odometer, clock, contact events, battery. */
  protected finishStep(before: V3, dt: number, touching: boolean): VehicleStep {
    this.odometer += Math.hypot(this.pos[0] - before[0], this.pos[1] - before[1], this.pos[2] - before[2]);
    this.tSec += dt;
    const collided = touching && this.tSec - this.lastTouch > CONTACT_GAP_SEC;
    if (touching) this.lastTouch = this.tSec;
    if (collided) this.contacts += 1;
    const powerW = this.battery.powerFor(this.vel, this.accel);
    this.battery.draw(powerW, dt);
    return { collided, touching, powerW };
  }

  /** Move by delta, sliding along solids; the open-air fast path when nothing is near. */
  protected move(pos: V3, delta: V3): { pos: V3; hit: boolean } {
    return this.openAir(pos, delta) ?? this.physicsMove(pos, delta);
  }

  private physicsMove(pos: V3, delta: V3): { pos: V3; hit: boolean } {
    this.physicsMoves += 1;
    const m = this.physics.move(pos, delta);
    return { pos: [m.pos[0], m.pos[1], m.pos[2]], hit: m.hit };
  }

  /** Direct step when rays along the motion find nothing within the swept sphere plus a margin. */
  private openAir(pos: V3, delta: V3): { pos: V3; hit: boolean } | null {
    const world = this.world;
    if (!world) return null;
    const l = Math.hypot(delta[0], delta[1], delta[2]);
    if (l < 1e-9) {
      this.freeMoves += 1;
      return { pos: [pos[0], pos[1], pos[2]], hit: false };
    }
    const next: V3 = [pos[0] + delta[0], pos[1] + delta[1], pos[2] + delta[2]];
    if (!containsPoint(world.geofence, next[0], next[1]) || inNoFly(world.noFly, next[0], next[1], next[2])) return null;
    if (next[2] < sampleHeight(world.terrain, next[0], next[1]) + DRONE_RADIUS + 0.3) return null;
    const d: V3 = [delta[0] / l, delta[1] / l, delta[2] / l];
    let side: V3 = [-d[1], d[0], 0];
    const sl = Math.hypot(side[0], side[1]);
    side = sl < 1e-3 ? [1, 0, 0] : [side[0] / sl, side[1] / sl, 0];
    const up: V3 = [side[1] * d[2] - side[2] * d[1], side[2] * d[0] - side[0] * d[2], side[0] * d[1] - side[1] * d[0]];
    const reach = l + LOOK_MARGIN;
    for (let k = -1; k < 8; k++) {
      let ox = pos[0];
      let oy = pos[1];
      let oz = pos[2];
      if (k >= 0) {
        const a = (k / 8) * Math.PI * 2;
        const cs = Math.cos(a) * BUNDLE_RADIUS;
        const sn = Math.sin(a) * BUNDLE_RADIUS;
        ox += side[0] * cs + up[0] * sn;
        oy += side[1] * cs + up[1] * sn;
        oz += side[2] * cs + up[2] * sn;
      }
      if (this.physics.cast(ox, oy, oz, d[0], d[1], d[2], reach)) return null;
    }
    // Sideways too, so a wall the drone grazes at a shallow angle is never skipped.
    for (const s of [side, up]) {
      for (const sign of [1, -1]) {
        if (this.physics.cast(next[0], next[1], next[2], s[0] * sign, s[1] * sign, s[2] * sign, BUNDLE_RADIUS + 0.2)) return null;
      }
    }
    this.freeMoves += 1;
    return { pos: next, hit: false };
  }
}

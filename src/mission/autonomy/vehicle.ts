import { KINEMATIC_TAU } from '../../shared/constants';
import type { OutdoorPhysics } from '../../world/outdoor/physics';
import type { BatteryModel, DronePose, V3 } from './contracts';

/**
 * Deterministic kinematic multirotor for autonomous missions. Velocity
 * setpoints pass through a first-order lag, then acceleration and jerk
 * limits, then speed caps. Motion goes through OutdoorPhysics.move, which
 * slides along solids; a collision is the start of a contact, so sliding
 * along a wall for many ticks counts once. Every tick draws battery power
 * for the current velocity and acceleration.
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

export interface VehicleStep {
  /** A new contact started this tick. */
  collided: boolean;
  /** Any contact this tick (including sliding). */
  touching: boolean;
  powerW: number;
}

/** Contacts closer than this to the previous one belong to the same event, s. */
export const CONTACT_GAP_SEC = 0.5;

function clampMag2(x: number, y: number, cap: number): [number, number] {
  const m = Math.hypot(x, y);
  if (m <= cap || m < 1e-12) return [x, y];
  return [(x / m) * cap, (y / m) * cap];
}

function wrap(a: number): number {
  const tau = Math.PI * 2;
  return ((a % tau) + tau) % tau;
}

export class Vehicle {
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

  constructor(
    private readonly physics: OutdoorPhysics,
    readonly battery: BatteryModel,
    start: V3,
    yaw: number,
  ) {
    this.pos = [start[0], start[1], start[2]];
    this.yaw = wrap(yaw);
  }

  pose(): DronePose {
    return { pos: [this.pos[0], this.pos[1], this.pos[2]], yaw: this.yaw, vel: [this.vel[0], this.vel[1], this.vel[2]] };
  }

  step(cmd: VehicleCommand, dt: number, limits: VehicleLimits): VehicleStep {
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
    const moved = this.physics.move(before, [vx * dt, vy * dt, vz * dt]);
    this.pos = [moved.pos[0], moved.pos[1], moved.pos[2]];
    // The velocity that actually happened after sliding.
    const real: V3 = [(this.pos[0] - before[0]) / dt, (this.pos[1] - before[1]) / dt, (this.pos[2] - before[2]) / dt];
    const blocked = Math.hypot(real[0] - vx, real[1] - vy, real[2] - vz) > 1e-3;
    if (blocked) {
      vx = real[0];
      vy = real[1];
      vz = real[2];
    }
    this.accel = [(vx - this.vel[0]) / dt, (vy - this.vel[1]) / dt, (vz - this.vel[2]) / dt];
    if (blocked) this.accel = [ax, ay, az];
    this.vel = [vx, vy, vz];
    this.odometer += Math.hypot(this.pos[0] - before[0], this.pos[1] - before[1], this.pos[2] - before[2]);
    this.tSec += dt;

    const touching = moved.hit;
    const collided = touching && this.tSec - this.lastTouch > CONTACT_GAP_SEC;
    if (touching) this.lastTouch = this.tSec;
    if (collided) this.contacts += 1;
    const powerW = this.battery.powerFor(this.vel, this.accel);
    this.battery.draw(powerW, dt);
    return { collided, touching, powerW };
  }
}

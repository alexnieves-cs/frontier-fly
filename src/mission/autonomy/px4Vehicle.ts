import { DRONE_RADIUS } from '../../shared/constants';
import type { OutdoorPhysics } from '../../world/outdoor/physics';
import { sampleHeight } from '../../world/outdoor/terrain';
import type { OutdoorWorld } from '../../world/outdoor/types';
import type { BatteryModel, V3 } from './contracts';
import { MISSION_QUAD_BRAKING } from './quadVehicle';
import { Vehicle, type ImuReading, type StepOptions, type VehicleCommand, type VehicleLimits, type VehicleStep } from './vehicle';

/**
 * The mission airframe flown by PX4.
 *
 * Every tick the runner's velocity command goes to PX4 SITL as an offboard setpoint, and PX4's
 * estimator and controllers fly frontier-fly's own rigid body (src/px4/frontierPlant.ts, the same
 * airframe and dynamics QuadVehicle flies) for one tick in lockstep. What comes back is where the
 * airframe really went. The plant knows the terrain but not the buildings or trees, so each tick's
 * motion is resolved against the world here, as every airframe's is (Vehicle.move): a wall stops the
 * airframe, and the plant is put back where the wall left it before the next tick.
 *
 * Indoors the GNSS receiver loses the sky. With `vision` (the default) PX4 then gets no fix and
 * flies on external vision: the position and velocity of an onboard estimator (VIO or lidar odometry),
 * which the caller supplies.
 */

/** One tick's command to PX4, NED from the pad. */
export interface Px4LinkCommand {
  dt: number;
  velNed: V3;
  /** Yaw rate about down, rad/s. */
  yawRateDown: number;
  windNed: V3;
  teleport: { posNed: V3; velNed: V3 } | null;
  thrust: readonly number[];
  gpsFix: boolean;
  ev: { posNed: V3; velNed: V3; posVar: number; velVar: number; reset: number } | null;
}

/** The plant and PX4 after a tick. */
export interface Px4LinkState {
  simSec: number;
  posNed: V3;
  velNed: V3;
  /** Body FRD to NED, w x y z. */
  q: [number, number, number, number];
  omegaFrd: V3;
  specificForceFrd: V3;
  armed: boolean;
  customMode: number;
  onGround: boolean;
  motors: [number, number, number, number];
  /** PX4's own position estimate (LOCAL_POSITION_NED). */
  ekfPosNed: V3;
  ekfVelNed: V3;
  landedState: number;
  timeouts: number;
}

export interface Px4Link {
  advance(cmd: Px4LinkCommand): Px4LinkState;
}

/** Where the onboard estimator thinks the drone is, world frame, when GNSS is gone. */
export type VisionSource = (truth: { pos: V3; vel: V3; tSec: number }) => { pos: V3; vel: V3; posVar: number; velVar: number; reset: number } | null;

export interface Px4VehicleOptions {
  /** True where the GNSS receiver has no sky (inside a building). */
  gnssDenied?: (pos: V3) => boolean;
  /** External vision for PX4 while GNSS is denied. */
  vision?: VisionSource;
}

/** Height of the body centre over the ground at rest, m: under the runner's 0.4 m touchdown cut. */
export const PX4_REST_M = DRONE_RADIUS + 0.05;

const enu = (n: V3): V3 => [n[1], n[0], -n[2]];
const ned = (e: V3): V3 => [e[1], e[0], -e[2]];

export class Px4Vehicle extends Vehicle {
  override readonly kind = 'quad';
  /** PX4 flies the same airframe, so the indoor braking filter keeps the same stopping model. */
  readonly braking = MISSION_QUAD_BRAKING;
  /** Last state PX4 and the plant reported. */
  last: Px4LinkState;
  /** Ticks flown with GNSS denied, and with external vision going to PX4. */
  deniedTicks = 0;
  visionTicks = 0;
  /** GNSS denied, and external vision sent, on the last tick. */
  lastDenied = false;
  lastVision = false;
  /** The velocity setpoint sent to PX4 on the last tick, world frame (after the speed caps). */
  lastSetpoint: V3 = [0, 0, 0];
  /** World point of the plant's NED origin: the ground under the start, plus the resting clearance. */
  readonly origin: V3;
  private readonly thrust = [1, 1, 1, 1];
  private pendingTeleport: { posNed: V3; velNed: V3 } | null = null;
  private roll = 0;
  private pitch = 0;

  constructor(
    physics: OutdoorPhysics,
    battery: BatteryModel,
    start: V3,
    yaw: number,
    world: OutdoorWorld | null,
    private readonly link: Px4Link,
    first: Px4LinkState,
    private readonly opts: Px4VehicleOptions = {},
  ) {
    super(physics, battery, start, yaw, world);
    // PX4 boots and arms on the ground, so the plant starts on it: the mission's home pose sits 1.7 m
    // over the pad (the walking camera's height), and the drone rests PX4_REST_M over the terrain.
    this.origin = [start[0], start[1], world ? sampleHeight(world.terrain, start[0], start[1]) + PX4_REST_M : start[2]];
    this.last = first;
    this.absorb(first);
    this.pos = this.worldPos(first.posNed);
  }

  override setThrustAuthority(k: number | readonly number[]): void {
    super.setThrustAuthority(k);
    for (let i = 0; i < 4; i++) this.thrust[i] = Math.max(0, Math.min(1, typeof k === 'number' ? k : (k[i] ?? 1)));
  }

  override imu(): ImuReading {
    const f = this.last.specificForceFrd;
    const w = this.last.omegaFrd;
    return { accelBody: [f[0], -f[1], -f[2]], gyroBody: [w[0], -w[1], -w[2]] };
  }

  get attitude(): { roll: number; pitch: number } {
    return { roll: this.roll, pitch: this.pitch };
  }

  private worldPos(posNed: V3): V3 {
    const p = enu(posNed);
    return [this.origin[0] + p[0], this.origin[1] + p[1], this.origin[2] + p[2]];
  }

  private absorb(s: Px4LinkState): void {
    const [w, x, y, z] = s.q;
    // NED/FRD Euler angles; heading from north clockwise becomes ENU yaw from east counter-clockwise.
    const psi = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    this.roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    // FLU pitch is positive nose down, FRD nose up.
    this.pitch = -Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
    const yaw = Math.PI / 2 - psi;
    this.yaw = ((yaw % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2);
    this.yawRate = -s.omegaFrd[2];
  }

  override step(cmd: VehicleCommand, dt: number, limits: VehicleLimits, opts: StepOptions = {}): VehicleStep {
    // Speed caps on the setpoint, as the mission's own flight controller applies them.
    let [vx, vy] = [cmd.vel[0], cmd.vel[1]];
    const h = Math.hypot(vx, vy);
    if (h > limits.speedH) {
      vx *= limits.speedH / h;
      vy *= limits.speedH / h;
    }
    const vz = Math.max(-limits.speedV, Math.min(limits.speedV, cmd.vel[2]));
    let rate = cmd.yawRate ?? 0;
    if (cmd.yawRate === undefined && cmd.yaw !== undefined && cmd.yaw !== null) {
      let err = cmd.yaw - this.yaw;
      err -= Math.round(err / (Math.PI * 2)) * Math.PI * 2;
      rate = err / Math.max(dt, 0.25);
    }
    rate = Math.max(-limits.yawRate, Math.min(limits.yawRate, rate));

    this.lastSetpoint = [vx, vy, vz];
    const before: V3 = [this.pos[0], this.pos[1], this.pos[2]];
    const denied = this.opts.gnssDenied?.(before) ?? false;
    let ev: Px4LinkCommand['ev'] = null;
    this.lastDenied = denied;
    this.lastVision = false;
    if (denied) {
      this.deniedTicks++;
      const v = this.opts.vision?.({ pos: before, vel: this.vel, tSec: this.tSec });
      if (v) {
        this.visionTicks++;
        this.lastVision = true;
        const p: V3 = [v.pos[0] - this.origin[0], v.pos[1] - this.origin[1], v.pos[2] - this.origin[2]];
        ev = { posNed: ned(p), velNed: ned(v.vel), posVar: v.posVar, velVar: v.velVar, reset: v.reset };
      }
    }
    const w = opts.wind ?? [0, 0, 0];
    const s = this.link.advance({
      dt,
      velNed: ned([vx, vy, vz]),
      yawRateDown: -rate,
      windNed: ned(w),
      teleport: this.pendingTeleport,
      thrust: this.thrust,
      gpsFix: !denied,
      ev,
    });
    this.pendingTeleport = null;
    this.last = s;
    const prevVel: V3 = [this.vel[0], this.vel[1], this.vel[2]];
    const truePos = this.worldPos(s.posNed);
    const delta: V3 = [truePos[0] - before[0], truePos[1] - before[1], truePos[2] - before[2]];
    const moved = this.move(before, delta);
    let lost = 0;
    let vel = enu(s.velNed);
    if (moved.hit) {
      const real: V3 = [(moved.pos[0] - before[0]) / dt, (moved.pos[1] - before[1]) / dt, (moved.pos[2] - before[2]) / dt];
      lost = Math.hypot(real[0] - delta[0] / dt, real[1] - delta[1] / dt, real[2] - delta[2] / dt);
      vel = real;
      this.pendingTeleport = {
        posNed: ned([moved.pos[0] - this.origin[0], moved.pos[1] - this.origin[1], moved.pos[2] - this.origin[2]]),
        velNed: ned(real),
      };
    }
    this.pos = [moved.pos[0], moved.pos[1], moved.pos[2]];
    this.vel = vel;
    this.accel = [(vel[0] - prevVel[0]) / dt, (vel[1] - prevVel[1]) / dt, (vel[2] - prevVel[2]) / dt];
    this.absorb(s);
    return this.finishStep(before, dt, moved.hit, lost);
  }
}

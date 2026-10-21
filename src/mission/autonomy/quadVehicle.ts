import { makeDroneState } from '../../sim/drone';
import { defaultDynParams } from '../../sim/dynamics/dr';
import { QuadrotorDynamics } from '../../sim/dynamics/quad';
import { QUAD_BRAKING, type BrakingModel } from '../../sim/sensorPlanner';
import type { DroneState, DynParams } from '../../shared/types';
import type { OutdoorPhysics } from '../../world/outdoor/physics';
import type { OutdoorWorld } from '../../world/outdoor/types';
import type { BatteryModel, V3 } from './contracts';
import { Vehicle, type ImuReading, type StepOptions, type VehicleCommand, type VehicleLimits, type VehicleStep } from './vehicle';

/** Physics rate inside one mission tick, Hz (the coverage env's quad runs at the same rate). */
const PHYSICS_HZ = 400;

/**
 * The mission airframe as a quadrotor: the 1.2 kg quad the battery model
 * describes (battery.ts), on the same rigid-body model and angle-mode flight
 * controller as the coverage env's micro quad (sim/dynamics). Motor speeds at
 * hover and the drag-to-mass ratio match that airframe, so thrust-to-weight is
 * the same; inertia grows with mass and the rate-loop gains with it, which is
 * what a flight controller tune does for a bigger frame.
 */
export function missionQuadParams(): DynParams {
  const mass = 1.2;
  const p = defaultDynParams(mass);
  const k = mass / 0.45;
  return {
    ...p,
    inertia: [p.inertia[0] * k, p.inertia[1] * k, p.inertia[2] * k],
    dragLin: [p.dragLin[0] * k, p.dragLin[1] * k, p.dragLin[2] * k],
    dragQuad: [p.dragQuad[0] * k, p.dragQuad[1] * k, p.dragQuad[2] * k],
  };
}

/**
 * Stopping model for the mission quad, which flies indoors at 1 m/s. The
 * coverage env's fit (QUAD_BRAKING) is from the same controller and the same
 * thrust-to-weight, so it carries over; a test checks it against this airframe.
 */
export const MISSION_QUAD_BRAKING: BrakingModel = QUAD_BRAKING;

/**
 * Mission vehicle on quadrotor dynamics. Each tick it converts the world-frame
 * velocity command to the body frame and runs the flight controller and
 * rigid-body physics at 400 Hz, then resolves the tick's motion against the
 * world once, the same way the kinematic vehicle does (slide along solids,
 * contact events, battery draw). Speed and yaw-rate caps come from the
 * mission's VehicleLimits; acceleration is whatever the airframe can do.
 */
export class QuadVehicle extends Vehicle {
  override readonly kind = 'quad';
  readonly braking = MISSION_QUAD_BRAKING;
  private readonly dyn = new QuadrotorDynamics();
  private readonly state: DroneState;
  private readonly cmd = new Float32Array(4);
  private readonly params: DynParams;

  constructor(physics: OutdoorPhysics, battery: BatteryModel, start: V3, yaw: number, world: OutdoorWorld | null = null, params = missionQuadParams()) {
    super(physics, battery, start, yaw, world);
    this.params = params;
    this.state = makeDroneState([start[0], start[1], start[2]], yaw > Math.PI ? yaw - Math.PI * 2 : yaw);
    this.dyn.reset(this.state, params);
    this.dyn.controller.setLimits(1, 1, 1, params.inertia[0] / defaultDynParams().inertia[0]);
  }

  override setThrustAuthority(k: number | readonly number[]): void {
    super.setThrustAuthority(k);
    for (let i = 0; i < 4; i++) this.dyn.thrustScale[i] = Math.max(0, Math.min(1, typeof k === 'number' ? k : (k[i] ?? 1)));
  }

  override imu(): ImuReading {
    const f = this.dyn.specificForce;
    const w = this.dyn.bodyRates;
    return { accelBody: [f[0], f[1], f[2]], gyroBody: [w[0] ?? 0, w[1] ?? 0, w[2] ?? 0] };
  }

  /** Roll and pitch, rad (for the HUD and the localization thread's IMU model). */
  get attitude(): { roll: number; pitch: number } {
    return { roll: this.state.roll ?? 0, pitch: this.state.pitch ?? 0 };
  }

  override step(cmd: VehicleCommand, dt: number, limits: VehicleLimits, opts: StepOptions = {}): VehicleStep {
    const ctrl = this.dyn.controller;
    ctrl.setLimits(limits.speedH, limits.speedV, limits.yawRate, this.params.inertia[0] / defaultDynParams().inertia[0], limits.maxTilt);
    const w = opts.wind;
    this.dyn.wind[0] = w ? w[0] : 0;
    this.dyn.wind[1] = w ? w[1] : 0;
    this.dyn.wind[2] = w ? w[2] : 0;

    let rate = cmd.yawRate ?? 0;
    if (cmd.yawRate === undefined && cmd.yaw !== undefined && cmd.yaw !== null) {
      let err = cmd.yaw - this.state.yaw;
      err -= Math.round(err / (Math.PI * 2)) * Math.PI * 2;
      rate = err / Math.max(dt, 0.25);
    }
    const s = this.state;
    const before: V3 = [this.pos[0], this.pos[1], this.pos[2]];
    s.pos[0] = before[0];
    s.pos[1] = before[1];
    s.pos[2] = before[2];
    const n = Math.max(1, Math.round(dt * PHYSICS_HZ));
    const h = dt / n;
    const prevVel: V3 = [s.vel[0], s.vel[1], s.vel[2]];
    for (let i = 0; i < n; i++) {
      // The controller works in the body frame; re-express the world command as the airframe turns.
      const c = Math.cos(s.yaw);
      const sn = Math.sin(s.yaw);
      this.cmd[0] = c * cmd.vel[0] + sn * cmd.vel[1];
      this.cmd[1] = -sn * cmd.vel[0] + c * cmd.vel[1];
      this.cmd[2] = cmd.vel[2];
      this.cmd[3] = rate;
      this.dyn.step(this.cmd, h);
    }
    const delta: V3 = [s.pos[0] - before[0], s.pos[1] - before[1], s.pos[2] - before[2]];
    const moved = this.move(before, delta);
    this.pos = [moved.pos[0], moved.pos[1], moved.pos[2]];
    if (moved.hit) {
      // The tick's motion was cut short by a surface: keep only the velocity that happened.
      for (let k = 0; k < 3; k++) {
        const real = (this.pos[k]! - before[k]!) / dt;
        if (Math.abs(real - delta[k]! / dt) > 1e-3) s.vel[k] = real;
      }
    }
    s.pos[0] = this.pos[0];
    s.pos[1] = this.pos[1];
    s.pos[2] = this.pos[2];
    this.vel = [s.vel[0], s.vel[1], s.vel[2]];
    this.accel = [(s.vel[0] - prevVel[0]) / dt, (s.vel[1] - prevVel[1]) / dt, (s.vel[2] - prevVel[2]) / dt];
    this.yaw = ((s.yaw % (Math.PI * 2)) + Math.PI * 2) % (Math.PI * 2);
    this.yawRate = this.dyn.bodyRates[2] ?? 0;
    return this.finishStep(before, dt, moved.hit);
  }
}

import { KINEMATIC_TAU, MAX_SPEED_H, MAX_SPEED_V, MAX_YAW_RATE } from '../../shared/constants';
import type { DroneState, DynParams, Dynamics } from '../../shared/types';

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

/**
 * First-order velocity lag. `cmd` is already in physical units:
 * body-frame vx/vy (m/s), world vz (m/s), yaw rate (rad/s).
 */
export class KinematicDynamics implements Dynamics {
  private state: DroneState | null = null;

  reset(s: DroneState, p: DynParams): void {
    void p;
    this.state = s;
    if (s.motors.length < 4) s.motors = new Float32Array(4);
  }

  step(cmd: Float32Array, dt: number): DroneState {
    const s = this.state;
    if (!s) throw new Error('KinematicDynamics.step before reset');
    const vx = clamp(cmd[0] ?? 0, -MAX_SPEED_H, MAX_SPEED_H);
    const vy = clamp(cmd[1] ?? 0, -MAX_SPEED_H, MAX_SPEED_H);
    const vz = clamp(cmd[2] ?? 0, -MAX_SPEED_V, MAX_SPEED_V);
    const yawRate = clamp(cmd[3] ?? 0, -MAX_YAW_RATE, MAX_YAW_RATE);
    const cy = Math.cos(s.yaw);
    const sy = Math.sin(s.yaw);
    const targetX = cy * vx - sy * vy;
    const targetY = sy * vx + cy * vy;
    const lag = 1 - Math.exp(-dt / KINEMATIC_TAU);
    s.vel[0] += (targetX - s.vel[0]) * lag;
    s.vel[1] += (targetY - s.vel[1]) * lag;
    s.vel[2] += (vz - s.vel[2]) * lag;
    s.yaw += yawRate * dt;
    const tau = Math.PI * 2;
    s.yaw = ((s.yaw % tau) + tau) % tau;
    s.pos[0] += s.vel[0] * dt;
    s.pos[1] += s.vel[1] * dt;
    s.pos[2] += s.vel[2] * dt;
    s.t += dt;
    s.roll = 0;
    s.pitch = 0;
    const half = s.yaw * 0.5;
    s.quat[0] = 0;
    s.quat[1] = 0;
    s.quat[2] = Math.sin(half);
    s.quat[3] = Math.cos(half);
    const spin = 500 + Math.hypot(s.vel[0], s.vel[1], s.vel[2]) * 80;
    s.motors[0] = spin;
    s.motors[1] = spin;
    s.motors[2] = spin;
    s.motors[3] = spin;
    return s;
  }
}

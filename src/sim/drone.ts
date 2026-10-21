import { MAX_SPEED_H, MAX_SPEED_V, MAX_YAW_RATE } from '../shared/constants';
import type { DroneState } from '../shared/types';

export function makeDroneState(pos: [number, number, number], yaw = 0): DroneState {
  const half = yaw * 0.5;
  return {
    pos: [pos[0], pos[1], pos[2]],
    vel: [0, 0, 0],
    yaw,
    t: 0,
    quat: [0, 0, Math.sin(half), Math.cos(half)],
    motors: new Float32Array(4),
    roll: 0,
    pitch: 0,
  };
}

export function cloneDroneState(s: DroneState): DroneState {
  return {
    pos: [s.pos[0], s.pos[1], s.pos[2]],
    vel: [s.vel[0], s.vel[1], s.vel[2]],
    yaw: s.yaw,
    t: s.t,
    quat: [s.quat[0], s.quat[1], s.quat[2], s.quat[3]],
    motors: new Float32Array(s.motors),
    roll: s.roll,
    pitch: s.pitch,
  };
}

/** Map a tanh action in [-1, 1] to body-frame setpoints. `speedH` is the horizontal speed a full action asks for, m/s. */
export function scaleAction(action: ArrayLike<number>, out: Float32Array, speedH = MAX_SPEED_H): Float32Array {
  const clamp = (v: number, lo: number, hi: number) => (v < lo ? lo : v > hi ? hi : v);
  out[0] = clamp(action[0] ?? 0, -1, 1) * speedH;
  out[1] = clamp(action[1] ?? 0, -1, 1) * speedH;
  out[2] = clamp(action[2] ?? 0, -1, 1) * MAX_SPEED_V;
  out[3] = clamp(action[3] ?? 0, -1, 1) * MAX_YAW_RATE;
  return out;
}

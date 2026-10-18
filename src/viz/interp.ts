import type { DroneState } from '../shared/types';

export function lerpState(a: DroneState, b: DroneState, t: number): DroneState {
  const u = t < 0 ? 0 : t > 1 ? 1 : t;
  const mix = (p: number, q: number) => p + (q - p) * u;
  let dy = b.yaw - a.yaw;
  if (dy > Math.PI) dy -= Math.PI * 2;
  if (dy < -Math.PI) dy += Math.PI * 2;
  return {
    pos: [mix(a.pos[0], b.pos[0]), mix(a.pos[1], b.pos[1]), mix(a.pos[2], b.pos[2])],
    vel: [mix(a.vel[0], b.vel[0]), mix(a.vel[1], b.vel[1]), mix(a.vel[2], b.vel[2])],
    yaw: a.yaw + dy * u,
    t: mix(a.t, b.t),
    quat: [
      mix(a.quat[0], b.quat[0]),
      mix(a.quat[1], b.quat[1]),
      mix(a.quat[2], b.quat[2]),
      mix(a.quat[3], b.quat[3]),
    ],
    motors: new Float32Array([
      mix(a.motors[0] ?? 0, b.motors[0] ?? 0),
      mix(a.motors[1] ?? 0, b.motors[1] ?? 0),
      mix(a.motors[2] ?? 0, b.motors[2] ?? 0),
      mix(a.motors[3] ?? 0, b.motors[3] ?? 0),
    ]),
    roll: mix(a.roll, b.roll),
    pitch: mix(a.pitch, b.pitch),
  };
}

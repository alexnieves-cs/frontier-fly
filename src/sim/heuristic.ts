import { LIDAR_H, OBS } from '../shared/constants';

/** Cheap lidar explorer used for shot cameras and as a light-invariant baseline. */
export function heuristicAction(obs: Float32Array): Float32Array {
  let front = 1;
  let left = 0;
  let right = 0;
  for (let i = 0; i < LIDAR_H; i++) {
    const range = obs[OBS.lidar + i] ?? 0;
    const ang = (i / LIDAR_H) * Math.PI * 2;
    const wrapped = ang > Math.PI ? ang - Math.PI * 2 : ang;
    if (Math.abs(wrapped) < 0.5) front = Math.min(front, range);
    if (wrapped > 0.2 && wrapped < 2.2) left += range;
    if (wrapped < -0.2 && wrapped > -2.2) right += range;
  }
  const alt = obs[OBS.alt] ?? 0.34;
  const ceil = obs[OBS.ceil] ?? 1;
  let vz = Math.max(-0.6, Math.min(0.6, (0.36 - alt) * 4));
  if (ceil < 0.16) vz = Math.min(vz, -0.35);
  const side = left - right;
  const forward = front > 0.16 ? Math.min(0.8, (front - 0.08) * 2.2) : -0.2;
  const yaw = front > 0.16 ? Math.max(-0.45, Math.min(0.45, side * 0.025)) : side >= 0 ? 0.85 : -0.85;
  return new Float32Array([forward, Math.max(-0.35, Math.min(0.35, side * 0.01)), vz, yaw]);
}

import { LIDAR_H } from '../shared/constants';

/** Steer toward the farthest lidar return. Light-invariant aside from sensor noise. */
export function heuristicAction(obs: Float32Array): Float32Array {
  let best = -1;
  let bestI = 0;
  for (let i = 0; i < LIDAR_H; i++) {
    const range = obs[i] ?? 0;
    if (range > best) {
      best = range;
      bestI = i;
    }
  }
  let angle = (bestI / LIDAR_H) * Math.PI * 2;
  if (angle > Math.PI) angle -= Math.PI * 2;
  const yaw = Math.max(-1, Math.min(1, angle / (Math.PI / 2)));
  const forward = best > 0.22 ? 0.85 : -0.45;
  const alt = obs[46] ?? 0.3;
  const vz = Math.max(-1, Math.min(1, (0.34 - alt) * 3));
  const ceil = obs[47] ?? 1;
  const climb = ceil < 0.15 ? -0.4 : vz;
  return new Float32Array([forward, 0, climb, yaw * 0.75]);
}

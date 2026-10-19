import { FLOOR_HEIGHT, OBS } from '../shared/constants';

/**
 * Non-learned baseline that reads the same observation as the policy and
 * simply steers along its navigation block: turn toward `wpDir`, fly toward
 * it, and hold 1 m above whatever is below. It shows how much of the policy's
 * coverage comes from following the frontier waypoint it is handed.
 */
export function waypointAction(obs: Float32Array): Float32Array {
  const fx = obs[OBS.wpDir]!;
  const fy = obs[OBS.wpDir + 1]!;
  const vz = clamp((1 - obs[OBS.alt]! * FLOOR_HEIGHT) * 3, -0.6, 0.6);
  if (obs[OBS.hasFrontier]! < 0.5) return new Float32Array([0, 0, vz, 0.5]);
  const err = Math.atan2(fy, fx);
  return new Float32Array([clamp(fx * 0.9, -0.35, 0.9), clamp(fy * 0.75, -0.75, 0.75), vz, clamp(err / (Math.PI / 3), -0.8, 0.8)]);
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

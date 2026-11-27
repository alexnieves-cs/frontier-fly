// The joint mission's facade re-anchor (scripts/joint/joint-mission.ts, anchor arm). Modelled, not simulated from
// lidar rays: see the arm's description there.
import type { MissionWorld, V3 } from '../../src/mission/autonomy/contracts';
import { RNG } from '../../src/shared/rng';

/** Facade re-anchor (anchor arm): in range of a building's outline, error per axis, and how fast it wanders. */
export const ANCHOR_RANGE_M = 20;
export const ANCHOR_SIGMA_M = 0.15;
export const ANCHOR_TAU_S = 5;
/** Visual-inertial dead-reckoning drift away from buildings, as a share of the distance flown. */
export const VIO_DRIFT = 0.01;

function outlineDistance(p: V3, b: MissionWorld['buildings'][number]): number {
  const f = b.footprint;
  let inside = false;
  let best = Infinity;
  for (let i = 0, j = f.length - 1; i < f.length; j = i++) {
    const a = f[j]!, c = f[i]!;
    if ((c.y > p[1]) !== (a.y > p[1]) && p[0] < ((a.x - c.x) * (p[1] - c.y)) / (a.y - c.y) + c.x) inside = !inside;
    const dx = c.x - a.x, dy = c.y - a.y;
    const u = Math.max(0, Math.min(1, ((p[0] - a.x) * dx + (p[1] - a.y) * dy) / Math.max(1e-9, dx * dx + dy * dy)));
    best = Math.min(best, Math.hypot(p[0] - (a.x + u * dx), p[1] - (a.y + u * dy)));
  }
  return inside ? 0 : best;
}

/** The anchor arm's outdoor error: the DSS handoff and drift until a facade is in range, then the facade residual. */
export function anchoredError(world: MissionWorld, handoff: V3, seed: number): (t: number, p: V3) => V3 {
  const rng = new RNG(seed >>> 0, 'joint-anchor');
  const unit = (): [number, number] => {
    const a = rng.next() * Math.PI * 2;
    return [Math.cos(a), Math.sin(a)];
  };
  let base: [number, number] = [handoff[0], handoff[1]];
  let dir = unit();
  let since = 0;
  let last: V3 | null = null;
  let lastT = -1;
  let near = false;
  let res: [number, number] = [0, 0];
  let out: V3 = [handoff[0], handoff[1], 0];
  return (t, p) => {
    if (t === lastT) return out;
    const dt = lastT < 0 ? 0 : t - lastT;
    lastT = t;
    if (last) since += Math.hypot(p[0] - last[0], p[1] - last[1]);
    last = [p[0], p[1], p[2]];
    const inRange = world.buildings.some((b) => outlineDistance(p, b) <= ANCHOR_RANGE_M);
    if (inRange) {
      if (!near) res = [ANCHOR_SIGMA_M * rng.gaussian(), ANCHOR_SIGMA_M * rng.gaussian()];
      else {
        const a = Math.exp(-dt / ANCHOR_TAU_S), b = Math.sqrt(1 - a * a);
        res = [res[0] * a + ANCHOR_SIGMA_M * b * rng.gaussian(), res[1] * a + ANCHOR_SIGMA_M * b * rng.gaussian()];
      }
      near = true;
      out = [res[0], res[1], 0];
      return out;
    }
    if (near) {
      // Leaving the facade: dead reckoning again, from the last residual.
      near = false;
      base = res;
      dir = unit();
      since = 0;
    }
    out = [base[0] + dir[0] * VIO_DRIFT * since, base[1] + dir[1] * VIO_DRIFT * since, 0];
    return out;
  };
}

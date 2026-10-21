import type { Caster, TruthSample } from '../../src/loc/sensors';
import { LOG_STRIDE, type TrajectoryLog } from '../../src/loc/replay';

/** A 2D world of wall segments, extruded to any height: an analytic lidar target for estimator tests. */
export class SegmentWorld implements Caster {
  readonly segs: [number, number, number, number][] = [];

  /** Axis-aligned room outline. */
  box(x0: number, y0: number, x1: number, y1: number): this {
    this.segs.push([x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]);
    return this;
  }

  wall(ax: number, ay: number, bx: number, by: number): this {
    this.segs.push([ax, ay, bx, by]);
    return this;
  }

  cast(x: number, y: number, z: number, dx: number, dy: number, dz: number, maxDist: number): { dist: number; surface: number } | null {
    const len = Math.hypot(dx, dy, dz);
    if (len < 1e-12) return null;
    // Straight down: a floor at z = 0.
    if (Math.abs(dz) / len > 0.99) return dz < 0 && z < maxDist ? { dist: z, surface: 0 } : null;
    const ux = dx / len;
    const uy = dy / len;
    let best = Infinity;
    for (const [ax, ay, bx, by] of this.segs) {
      const ex = bx - ax;
      const ey = by - ay;
      const den = ux * ey - uy * ex;
      if (Math.abs(den) < 1e-12) continue;
      const t = ((ax - x) * ey - (ay - y) * ex) / den;
      const s = ((ax - x) * uy - (ay - y) * ux) / den;
      if (t > 1e-9 && s >= 0 && s <= 1 && t < best) best = t;
    }
    return best <= maxDist ? { dist: best, surface: 0 } : null;
  }
}

/** Log a trajectory from per-tick (x, y, yaw) waypoints at height z, with velocities by differences. */
export function makeLog(points: [number, number, number][], z = 1, dt = 0.05): TrajectoryLog {
  const poses = new Float32Array(points.length * LOG_STRIDE);
  for (let i = 0; i < points.length; i++) {
    const [x, y, yaw] = points[i]!;
    const [px, py] = points[Math.max(0, i - 1)]!;
    poses.set([x, y, z, yaw, (x - px) / dt, (y - py) / dt, 0], i * LOG_STRIDE);
  }
  return { dt, poses };
}

/** A lap around a rectangle, turning to face the direction of travel, at `speed` m/s. */
export function lap(x0: number, y0: number, x1: number, y1: number, speed = 1, laps = 1, dt = 0.05): [number, number, number][] {
  const corners: [number, number][] = [
    [x0, y0],
    [x1, y0],
    [x1, y1],
    [x0, y1],
  ];
  const out: [number, number, number][] = [];
  let yaw = 0;
  for (let l = 0; l < laps; l++) {
    for (let c = 0; c < 4; c++) {
      const [ax, ay] = corners[c]!;
      const [bx, by] = corners[(c + 1) % 4]!;
      const target = Math.atan2(by - ay, bx - ax);
      // Turn in place at the corner at 90°/s, then fly the side.
      while (Math.abs(Math.atan2(Math.sin(target - yaw), Math.cos(target - yaw))) > 1e-6) {
        const err = Math.atan2(Math.sin(target - yaw), Math.cos(target - yaw));
        yaw += Math.sign(err) * Math.min(Math.abs(err), (Math.PI / 2) * dt);
        out.push([ax, ay, yaw]);
      }
      const n = Math.max(1, Math.round(Math.hypot(bx - ax, by - ay) / (speed * dt)));
      for (let i = 1; i <= n; i++) out.push([ax + ((bx - ax) * i) / n, ay + ((by - ay) * i) / n, yaw]);
    }
  }
  return out;
}

export function truthOf(points: [number, number, number][], i: number, dt = 0.05): TruthSample {
  const [x, y, yaw] = points[i]!;
  const [px, py, pyaw] = points[Math.max(0, i - 1)]!;
  return { pos: [x, y, 1], yaw, vel: [(x - px) / dt, (y - py) / dt, 0], accel: [0, 0, 0], yawRate: (yaw - pyaw) / dt, dt };
}

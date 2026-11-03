import { FlyThrough } from '../planner/flythrough';
import { SWEEP_RADIUS, columnClear, cruiseZ, dist, len, segmentClear, sub } from '../planner/geometry';
import { toLocal } from '../planner/indoor';
import { ApertureSensor } from '../openingSensor';
import type { Phase, V3 } from '../contracts';
import { descentClear } from '../../../sim/lidarSafety';
import { INDOOR_LIMITS, type VehicleCommand } from '../vehicle';
import type { MissionCore } from './core';
import { type Job, PLAN_ACCEL, SHIELD_STALL_SEC, type Waypoint, hold, scaleTo } from './shared';

/**
 * The motion primitives every phase flies with: waypoint following, holds, the indoor lidar shield,
 * breadcrumb retracing, climbs to cruise, and the opening fly-through on the aperture tracker. A
 * primitive returns a Job and calls back when it ends; which phase comes next is the caller's
 * business, never the Pilot's.
 */
/** Breadcrumbs kept, 0.3 m apart: about 600 m, enough to lead out of any building searched on one pack. */
const MAX_CRUMBS = 2000;

export class Pilot {
  constructor(private readonly c: MissionCore) {}

  breadcrumb(): void {
    const p = this.c.navPos;
    const last = this.c.crumbs[this.c.crumbs.length - 1];
    if (!last || dist(last, p) >= 0.3) {
      this.c.crumbs.push([p[0], p[1], p[2]]);
      if (this.c.crumbs.length > MAX_CRUMBS) this.c.crumbs.shift();
    }
    const v = this.c.visit;
    if (v && !this.c.inside && v.standoff) {
      const tl = v.trail[v.trail.length - 1];
      if (!tl || dist(tl, p) >= 0.3) v.trail.push([p[0], p[1], p[2]]);
    }
  }

  /**
   * Retrace flown positions (known clear) backwards until `enough` holds. String-pulled with casts that
   * clear solids by the same sweep every outdoor leg keeps: a shortcut is a new leg, flown by a vehicle
   * that does not hold the straight line exactly.
   */
  retrace(trail: readonly V3[], enough: (p: V3) => boolean): Waypoint[] {
    const out: Waypoint[] = [];
    let cur: V3 = this.c.navPos;
    let i = trail.length - 1;
    for (let guard = 0; i >= 0 && guard < 400 && !enough(cur); guard++) {
      let j = i;
      for (let k = Math.max(0, i - 40); k < i; k++) {
        if (segmentClear(this.c.physics, cur, trail[k]!, SWEEP_RADIUS)) {
          j = k;
          break;
        }
      }
      cur = trail[j]!;
      out.push({ pos: cur, tol: 0.2 });
      i = j - 1;
    }
    return out;
  }

  followJob(phase: Phase, route: Waypoint[], onDone: () => void, passTol?: number): Job {
    // Close to obstacles, brake to a stop before turning onto a new path.
    const v0 = this.c.navVel;
    const p0 = this.c.navPos;
    const wps =
      passTol !== undefined && passTol <= 0.3 && Math.hypot(v0[0], v0[1], v0[2]) > 0.15 && route.length
        ? [
            {
              pos: [p0[0], p0[1], p0[2]] as V3,
              tol: 0.12,
            },
            ...route,
          ]
        : route;
    let idx = 0;
    let dwell = -1;
    let spin = -1;
    let called = false;
    // Progress watch for shielded indoor moves: a waypoint the lidar shield will not let the drone
    // reach (structure closer than the map knew) is skipped rather than flown at forever.
    let bestD = Infinity;
    let sinceBest = 0;
    const tolFor = (i: number): number => {
      const wp = wps[i]!;
      if (wp.tol !== undefined) return wp.tol;
      return passTol ?? 0.25;
    };
    const isStop = (i: number): boolean => {
      const wp = wps[i]!;
      if (i === wps.length - 1 || wp.dwell || wp.spin) return true;
      return false;
    };
    const cornerSpeed = (i: number): number => {
      if (isStop(i)) return 0;
      const prev = i === 0 ? this.c.navPos : wps[i - 1]!.pos;
      const a = sub(wps[i]!.pos, prev);
      const b = sub(wps[i + 1]!.pos, wps[i]!.pos);
      const la = len(a);
      const lb = len(b);
      if (la < 1e-6 || lb < 1e-6) return 0.5;
      const cos = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb);
      if (cos > 0.9) return 99;
      if (cos > 0.7) return 1.2;
      // Sharp corners are full stops so the drone never swings wide near obstacles.
      return 0;
    };
    return {
      phase,
      plan: () => wps.slice(idx).map((w) => w.pos),
      tick: (): VehicleCommand => {
        const pos = this.c.navPos;
        while (idx < wps.length) {
          const wp = wps[idx]!;
          const d = dist(pos, wp.pos);
          const stop = isStop(idx) || cornerSpeed(idx) === 0;
          const tol = stop ? 0.12 : tolFor(idx);
          const slow = len(this.c.navVel) < 0.3;
          if (d < tol && (!stop || slow)) {
            if (wp.spin && spin < 0 && dwell < 0) spin = Math.PI * 2;
            if (spin > 0) {
              const rate = INDOOR_LIMITS.yawRate;
              spin -= rate * this.c.dt;
              if (spin <= 0) spin = 0;
              return { vel: hold(pos, wp.pos), yawRate: rate };
            }
            if (dwell === -1) dwell = wp.dwell ?? 0;
            if (dwell > 0) {
              dwell -= this.c.dt;
              return { vel: hold(pos, wp.pos), yaw: wp.yaw ?? null };
            }
            idx += 1;
            dwell = -1;
            spin = -1;
            bestD = Infinity;
            sinceBest = 0;
            continue;
          }
          // Speed from the distance to the next stop or corner.
          let ahead = d;
          let vEnd = 0;
          for (let k = idx; k < wps.length; k++) {
            const cs = cornerSpeed(k);
            if (cs < 99) {
              vEnd = cs;
              break;
            }
            ahead += dist(wps[k]!.pos, wps[k + 1]!.pos);
          }
          const speed = Math.min(99, Math.sqrt(vEnd * vEnd + 2 * PLAN_ACCEL * ahead), stop ? 1.6 * d + 0.03 : 99);
          if (d < bestD - 0.05) {
            bestD = d;
            sinceBest = 0;
          } else if ((sinceBest += this.c.dt) > SHIELD_STALL_SEC && this.c.inside && this.c.visit?.sensing && phase !== 'fly_through') {
            idx += 1;
            bestD = Infinity;
            sinceBest = 0;
            continue;
          }
          const dir = sub(wp.pos, pos);
          const want = scaleTo(dir, speed, this.c.limits());
          const vel = this.shielded(phase, want, wp.flown);
          const moving = Math.hypot(vel[0], vel[1]) > 0.3;
          // With a failed lidar arc, turn the middle of the healthy arc toward the direction of travel. The
          // wanted direction, not the shielded one: the shield treats the blind arc as blocked, so facing
          // it the drone barely moves and would never turn to see where it is going.
          const blind = this.blindYawOffset();
          const yaw =
            wp.yaw !== undefined && wp.yaw !== null && d < 4
              ? wp.yaw
              : blind !== 0 && Math.hypot(want[0], want[1]) > 0.1
                ? Math.atan2(want[1], want[0]) - blind
                : moving
                  ? Math.atan2(vel[1], vel[0])
                  : null;
          return { vel, yaw };
        }
        if (!called) {
          called = true;
          onDone();
        }
        return { vel: [0, 0, 0] };
      },
    };
  }

  /**
   * Indoors on the sensor-only map, routed moves (re-views, the way out) pass through the same lidar
   * shield as the explorers: the map is young and a route can graze structure the lidar sees now.
   */
  shielded(phase: Phase, vel: V3, flown = false): V3 {
    const v = this.c.visit;
    if (!this.c.inside || !v?.sensing || phase === 'fly_through') return vel;
    const local = toLocal(v.grid, this.c.navPos);
    v.sensing.sense(local, this.c.navVel, this.c.navYaw, this.c.t);
    const [vx, vy] = v.sensing.filter(vel[0], vel[1], this.c.navYaw);
    const vz = vel[2] < 0 && !flown && !descentClear(v.sensing.lidar) ? 0 : vel[2];
    return [vx, vy, vz];
  }

  /**
   * IndoorSensing range hook, called on every indoor lidar reading before anything uses it: inject
   * lidar faults, keep the ring as measured (and what the drone's map predicts for it) for the
   * failsafe, and blank the beams the failsafe distrusts. NaN reads as blocked to every filter.
   */
  lidarHook(lidar: Float64Array): void {
    this.c.faults?.lidarRing(lidar);
    const v = this.c.visit;
    if (v?.sensing) {
      this.c.lastExpected = v.sensing.expectedRanges(toLocal(v.grid, this.c.vehicle.pos), this.c.vehicle.yaw, new Float64Array(lidar.length));
      this.c.lastRing = Float64Array.from(lidar);
    }
    for (const i of this.c.health?.unknownBeams ?? []) lidar[i] = NaN;
  }

  /**
   * Body angle (rad, counter-clockwise from the nose) of the middle of the widest healthy lidar arc
   * while the failsafe distrusts some beams; 0 when the lidar is healthy.
   */
  blindYawOffset(): number {
    const bad = this.c.health?.unknownBeams;
    if (!bad || bad.length === 0) return 0;
    const n = this.c.visit?.sensing?.lidar.length ?? 32;
    const blocked = new Set(bad);
    if (blocked.size >= n) return 0;
    let best = 0;
    let bestGap = -1;
    for (let i = 0; i < n; i++) {
      if (blocked.has(i)) continue;
      let gap = n;
      for (const j of blocked) gap = Math.min(gap, Math.abs(i - j), n - Math.abs(i - j));
      if (gap > bestGap) {
        bestGap = gap;
        best = i;
      }
    }
    return (best / n) * Math.PI * 2;
  }

  holdJob(phase: Phase, seconds: number, onDone: () => void): Job {
    const p = this.c.navPos;
    const at: V3 = [p[0], p[1], p[2]];
    const end = this.c.t + seconds;
    let called = false;
    return {
      phase,
      plan: () => [],
      tick: (): VehicleCommand => {
        if (this.c.t >= end - 1e-9 && !called) {
          called = true;
          onDone();
        }
        return { vel: hold(this.c.navPos, at) };
      },
    };
  }

  /** Climb to cruise from wherever the drone is, when it is below it. */
  climbToCruise(): Waypoint[] {
    const pos = this.c.vehicle.pos;
    const z = cruiseZ(this.c.air, pos[0], pos[1], this.c.agl);
    if (pos[2] >= z - 1) return [];
    return [{ pos: [pos[0], pos[1], z], tol: 0.6 }];
  }

  /**
   * Lock the aperture tracker on the opening for a crossing. The sensor model is handed the real
   * opening, as the lidar is handed the real walls; the controller only ever sees its readings.
   */
  trackAperture(): void {
    const v = this.c.visit!;
    const real = v.b.shell.openings.find((o) => o.id === v.opening!.id)!;
    let h = (this.c.world.seed ^ 0x5bd1e995) >>> 0;
    h = Math.imul(h ^ (v.opening!.id + 1), 0x01000193) >>> 0;
    h = Math.imul(h ^ (v.aborts + 1), 0x01000193) >>> 0;
    h = Math.imul(h ^ this.c.entries.length, 0x01000193) >>> 0;
    this.c.aperture = new ApertureSensor(real, v.opening!, h);
  }

  flyThroughJob(ft: FlyThrough, onDone: (ok: boolean) => void): Job {
    let called = false;
    return {
      phase: this.c.phase === 'exit' ? 'exit' : 'fly_through',
      plan: () => [ft.start, ft.end],
      tick: (): VehicleCommand => {
        // The crossing flies on the aperture tracker; until its first reading, hold.
        const a = this.c.aperture;
        if (!a?.hasReading) return { vel: [0, 0, 0] };
        const s = ft.step(a.pos, a.vel, this.c.lastCollided);
        if ((s.state === 'done' || s.state === 'aborted') && !called) {
          called = true;
          onDone(s.state === 'done');
        }
        // Velocities and the heading are commanded in the frame the opening was measured in, which is the
        // flight controller's own: they go to the airframe as they are.
        return s.cmd;
      },
    };
  }
  climbRoute(): Waypoint[] {
    const pos = this.c.vehicle.pos;
    const top = cruiseZ(this.c.air, pos[0], pos[1], this.c.agl);
    if (pos[2] >= top - 0.5) return [];
    if (columnClear(this.c.physics, pos[0], pos[1], pos[2], top, 0.45)) return [{ pos: [pos[0], pos[1], top] }];
    const open = (p: V3) => columnClear(this.c.physics, p[0], p[1], p[2], cruiseZ(this.c.air, p[0], p[1], this.c.agl), 0.45);
    const back = this.retrace(this.c.visit?.trail.length ? this.c.visit.trail : this.c.crumbs, open);
    const end = back.length ? back[back.length - 1]!.pos : pos;
    if (back.length && !open(end)) this.c.log('warn', 'No open column found on the way back. Climbing anyway.');
    return [...back, { pos: [end[0], end[1], cruiseZ(this.c.air, end[0], end[1], this.c.agl)] }];
  }
}

import { sampleHeight } from '../../../world/outdoor/terrain';
import { containsPoint } from '../../../world/outdoor/geofence';
import type { V3 } from '../contracts';
import { columnClear, dist, segmentClear } from '../planner/geometry';
import { landedAtHome, landingCommand, planReturn } from '../planner/home';
import { floorOf } from '../planner/indoor';
import type { VehicleCommand } from '../vehicle';
import type { MissionCore } from './core';
import type { Pilot } from './pilot';
import { fmt, type Waypoint } from './shared';

/** Landing position integral gain, 1/s^2, and its cap, m/s. */
const LAND_TRIM_KI = 0.6;
const LAND_TRIM_MAX = 3;
/** Emergency landings: open-column radius, m, search radius, m, and how close over it before descending, m. */
const LAND_COLUMN_M = 0.8;
const LAND_SEARCH_M = 15;
const LAND_ALIGN_M = 0.5;
/** The same in severe wind, when the drone drifts on the way down: a wider clearing, searched further out. */
const LAND_COLUMN_WINDY_M = 3;
const LAND_SEARCH_WINDY_M = 40;
/**
 * An emergency landing that has not got over its column in this long, s, lands on the move instead:
 * it descends wherever the column under it, and under where it will be in LAND_LOOKAHEAD_S, is open.
 * In wind the airframe can't hold position in, it would otherwise hover beside its column until the
 * time limit.
 */
const LAND_ALIGN_TIMEOUT_S = 10;
const LAND_LOOKAHEAD_S = 1.5;
/** Height a weak airframe may lose on the way to its landing column, m. */
const LAND_SINK_M = 2;
/** Height over the range-finder ground at which a landing cuts the motors, m. */
const LAND_CUT_AGL = 0.4;

/** Where the way home hands over: out of the building first, or the end of the run. */
export interface HomeExits {
  startExit(after: 'continue' | 'rth'): void;
  finish(): void;
}

/** Return home and landing: the route home, landing on the pad or in place, and touchdown. */
export class HomePhase {
  constructor(
    private readonly c: MissionCore,
    private readonly pilot: Pilot,
    private readonly go: HomeExits,
  ) {}

  startReturn(reason: string): void {
    if (this.c.finished) return;
    if (!this.c.returning) this.c.log('action', `Returning home: ${reason}.`);
    this.c.returning = true;
    this.c.suspended = null;
    this.c.recoverRank = 0;
    this.c.pendingEntry = null;
    if (this.c.inside) {
      this.go.startExit('rth');
      return;
    }
    this.startReturnOutside();
  }

  startReturnOutside(): void {
    this.c.returning = true;
    const climb = this.pilot.climbRoute();
    const from = climb.length ? climb[climb.length - 1]!.pos : this.c.vehicle.pos;
    const plan = planReturn(this.c.physics, this.c.air, from, this.c.world.home, null, this.c.agl);
    const wps: Waypoint[] = [...climb.map((w) => ({ ...w, tol: 0.3 })), ...plan.waypoints.map((p) => ({ pos: p, tol: 1.5 }))];
    this.c.visit = null;
    this.c.setJob(this.pilot.followJob('return_home', wps, () => this.startLand('home'), 1.5));
  }

  startLand(reason: string): void {
    let target: V3 = reason === 'home' ? this.c.world.home : [this.c.vehicle.pos[0], this.c.vehicle.pos[1], this.c.vehicle.pos[2]];
    if (reason !== 'home') {
      this.c.landingEmergency = true;
      // In wind the airframe can't hold position in, look for a wider clearing: the drone drifts on the way down.
      const windy = !!this.c.health?.flags.has('wind_severe');
      const site = this.c.inside ? null : ((windy ? this.landingSite(LAND_COLUMN_WINDY_M, LAND_SEARCH_WINDY_M) : null) ?? this.landingSite());
      if (site && dist(site, this.c.vehicle.pos) > 0.3) {
        target = site;
        this.c.log('warn', `Landing in place: ${reason}. Moving ${fmt(dist(site, this.c.vehicle.pos), 1)} m to clear ground first.`);
      } else this.c.log('warn', `Landing in place: ${reason}.`);
    } else {
      this.c.log('action', 'Over the pad. Landing.');
    }
    const v = this.c.visit;
    const indoorGround = this.c.inside && v ? v.box.baseZ + floorOf(v.grid, this.c.vehicle.pos[2] - v.box.baseZ) * v.grid.H + 0.1 : null;
    // Outdoors the range finder gives the ground under the drone, wherever the position estimate puts it.
    const groundNow = (): number => {
      if (indoorGround !== null) return indoorGround;
      const p = this.c.truePos ?? this.c.vehicle.pos;
      return sampleHeight(this.c.world.outdoor.terrain, p[0], p[1]);
    };
    let settle = 0;
    let stuck = 0;
    // Integral on the horizontal error, so a steady wind doesn't leave the drone off the pad.
    const trim: [number, number] = [0, 0];
    let touchedAt: V3 | null = null;
    const landStart = this.c.t;
    let moveCheckAt = -Infinity;
    let moveOpen = false;
    let movingLogged = false;
    let gnssHoldLogged = false;
    this.c.setJob({
      phase: 'land',
      plan: () => [target],
      tick: (): VehicleCommand => {
        const pos = this.c.vehicle.pos;
        const g = groundNow();
        const c = landingCommand(pos, target, g);
        // Over the chosen column before descending, so the drone never sinks onto a roof or into canopy.
        let aligned = Math.hypot(target[0] - pos[0], target[1] - pos[1]) <= LAND_ALIGN_M;
        if (!aligned && !this.c.inside && reason !== 'home' && this.c.t - landStart > LAND_ALIGN_TIMEOUT_S) {
          // Re-checked a few times a second; the ray casts are not free.
          if (this.c.t - moveCheckAt >= 0.25) {
            moveCheckAt = this.c.t;
            const v = this.c.vehicle.vel;
            const ahead: [number, number] = [pos[0] + v[0] * LAND_LOOKAHEAD_S, pos[1] + v[1] * LAND_LOOKAHEAD_S];
            const open = (x: number, y: number): boolean =>
              columnClear(this.c.physics, x, y, sampleHeight(this.c.world.outdoor.terrain, x, y) + 0.2, pos[2], LAND_COLUMN_M);
            const was = moveOpen;
            moveOpen = open(pos[0], pos[1]) && open(ahead[0], ahead[1]) && segmentClear(this.c.physics, pos, [ahead[0], ahead[1], pos[2]]);
            if (moveOpen && !was && !movingLogged) {
              movingLogged = true;
              this.c.log('warn', `Can't hold over the landing column in this wind. Descending where the ground below is open.`);
            }
          }
          if (moveOpen) aligned = true;
          // Over trees or a roof and drifting: hold height and keep working toward the column.
          else c.vel[2] = Math.max(c.vel[2]!, 0);
        }
        if (!aligned && !c.touched) c.vel[2] = Math.max(c.vel[2]!, 0);
        // Over the pad on dead reckoning, the pad may be metres away and the column below may be trees or
        // a roof. Hold height until the fix is back; the failsafe lands after gnssLandSec if it never is.
        if (reason === 'home' && this.c.health?.flags.has('gnss_lost') && !c.touched) {
          if (!gnssHoldLogged) {
            gnssHoldLogged = true;
            this.c.log('warn', 'Over the pad on dead reckoning. Holding height until the GNSS fix is back.');
          }
          c.vel[2] = Math.max(c.vel[2]!, 0);
          aligned = false;
        }
        for (let i = 0; i < 2; i++) {
          trim[i] = Math.max(-LAND_TRIM_MAX, Math.min(LAND_TRIM_MAX, trim[i]! + (target[i]! - pos[i]!) * LAND_TRIM_KI * this.c.dt));
          c.vel[i] = c.vel[i]! + trim[i]!;
        }
        // Like a flight controller's land detector: in the last few tens of centimetres over measured ground,
        // cut the motors and drop, rather than hover in ground effect where the wind can slide the drone.
        if (aligned && pos[2] - g <= LAND_CUT_AGL) {
          this.touchdown([pos[0], pos[1], pos[2]]);
          return { vel: [0, 0, 0] };
        }
        // Resting on something above the expected ground also counts.
        stuck = aligned && this.c.vehicle.vel[2] > -0.03 && !c.touched ? stuck + this.c.dt : 0;
        if (c.touched || stuck > 1.5) {
          // Where it first touched: on the ground the motors idle and the wind no longer moves it.
          touchedAt ??= [pos[0], pos[1], pos[2]];
          settle += this.c.dt;
          if (settle > 0.5) this.touchdown(touchedAt);
        }
        return { vel: c.vel };
      },
    });
  }

  /**
   * Nearest point at the current height, within LAND_SEARCH_M, with an open column down to the
   * ground and a clear straight line to it. Null when there is none (land straight down then).
   */
  landingSite(column = LAND_COLUMN_M, search = LAND_SEARCH_M): V3 | null {
    const pos = this.c.vehicle.pos;
    const terrain = this.c.world.outdoor.terrain;
    const fence = this.c.world.outdoor.geofence;
    // Inside the geofence by the column's width too: the fence stops the drone, and a site past it can't be reached.
    const inFence = (x: number, y: number): boolean =>
      containsPoint(fence, x - column, y - column) && containsPoint(fence, x + column, y - column) && containsPoint(fence, x - column, y + column) && containsPoint(fence, x + column, y + column);
    const open = (x: number, y: number): boolean => inFence(x, y) && columnClear(this.c.physics, x, y, sampleHeight(terrain, x, y) + 0.2, pos[2], column);
    if (open(pos[0], pos[1])) return [pos[0], pos[1], pos[2]];
    for (let r = 1; r <= search; r += 1) {
      const n = Math.max(8, Math.round(r * 4));
      for (let k = 0; k < n; k++) {
        const a = (k / n) * Math.PI * 2;
        const p: V3 = [pos[0] + Math.cos(a) * r, pos[1] + Math.sin(a) * r, pos[2]];
        // A drone that can't hold height sinks on the way over, so the leg must also be clear lower down.
        const low = (q: V3): V3 => [q[0], q[1], q[2] - LAND_SINK_M];
        if (open(p[0], p[1]) && segmentClear(this.c.physics, pos, p) && segmentClear(this.c.physics, low(pos), low(p))) return p;
      }
    }
    return null;
  }

  touchdown(at: V3 = this.c.vehicle.pos): void {
    if (this.c.finished) return;
    this.c.landedHome = !this.c.inside && landedAtHome(at, this.c.world.home);
    this.c.log('action', this.c.landedHome ? 'Landed at home.' : `Landed away from home at ${this.c.where(at)}.`);
    this.go.finish();
  }
}

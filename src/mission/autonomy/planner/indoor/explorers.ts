/** The explorers missions fly inside a building: the trained policy and the classical sensor-only planner. */
import { type InferPolicy } from '../../../../rl/infer';
import { MAX_SPEED_H, VOXEL } from '../../../../shared/constants';
import { scaleAction } from '../../../../sim/drone';
import { descentClear, guardAction } from '../../../../sim/lidarSafety';
import { sensorClearance, sensorLos, sensorPath } from '../../../../sim/sensorMap';
import { SensorPlanner } from '../../../../sim/sensorPlanner';
import { type V3 } from '../../contracts';
import { clamp, wrapAngle } from '../geometry';
import { INDOOR_FLY_Z, type IndoorGrid, floorOf } from './grid';
import { INDOOR_FILTER_HORIZON, type IndoorSensing } from './sensing';

/** What the explorer needs from a policy: a deterministic mean action. */
export type ActingPolicy = Pick<InferPolicy, 'actWith'>;

export interface PolicyCommand {
  /** Local-frame velocity setpoint (axes match the world frame). */
  vel: V3;
  yawRate: number;
}

/** Steps in a row without a frontier before an explorer reports the building done. */
const DONE_STEPS = 20;

/**
 * Trained-policy explorer. Takes the policy's mean action on the shared
 * sensing's observation, caps speed for indoor flight, and runs it through
 * the sensor-only guard (sim/sensorPlanner.ts guardAction): an altitude nudge
 * and a lidar filter that cuts velocity only at imminent contact.
 */
export class PolicyExplorer {
  private readonly scaled = new Float32Array(4);
  private readonly guarded = new Float32Array(4);
  private idle = 0;

  constructor(
    readonly grid: IndoorGrid,
    readonly policy: ActingPolicy,
    readonly sensing: IndoorSensing,
    readonly speedCap = 1,
  ) {}

  /** No frontier left in the drone's own map for DONE_STEPS steps. */
  get exhausted(): boolean {
    return this.idle >= DONE_STEPS;
  }

  /** One policy step from a local pose. */
  act(local: V3, vel: V3, yaw: number, tSec: number): PolicyCommand {
    const obs = this.sensing.sense(local, vel, yaw, tSec);
    this.idle = this.sensing.frontierLeft ? 0 : this.idle + 1;
    const action = this.policy.actWith(obs, 1, this.sensing.rng, true).action;
    const f = floorOf(this.grid, local[2]);
    const brake = this.sensing.braking ? { model: this.sensing.braking, vel: [vel[0], vel[1]] as const } : undefined;
    guardAction(action, local[2] - f * this.grid.H, yaw, this.sensing.lidar, this.guarded, INDOOR_FILTER_HORIZON, brake);
    scaleAction(this.guarded, this.scaled);
    const k = this.speedCap / MAX_SPEED_H;
    const bx = this.scaled[0]! * k;
    const by = this.scaled[1]! * k;
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const vx = cy * bx - sy * by;
    const vy = sy * bx + cy * by;
    const vz = clamp(this.scaled[2]!, -0.6, 0.6);
    // Hold a sane height band on the current floor.
    const zLo = f * this.grid.H + 0.5;
    const zHi = (f + 1) * this.grid.H - 0.5;
    const vzSafe = local[2] < zLo ? Math.max(vz, 0.3) : local[2] > zHi ? Math.min(vz, -0.3) : vz;
    this.sensing.yawRate = this.scaled[3]!;
    return { vel: [vx, vy, vzSafe], yawRate: this.sensing.yawRate };
  }

  /** Visited voxels over reachable voxels. */
  coverage(): number {
    return this.sensing.coverage();
  }
}

/** A viewpoint is worth flying to when it would show at least this many unlooked voxels. */
const MIN_LOOK_GAIN = 4;

/**
 * Classical sensor-only explorer (sim/sensorPlanner.ts SensorPlanner) on the
 * shared sensing. It is the mission's indoor fallback and the baseline the
 * policy is measured against. Per floor: sweep (fly every known-free voxel,
 * facing the direction of travel), then look (fly to the viewpoint that would
 * show the most voxels the camera has not looked at yet, per meter of travel,
 * and turn a full circle there) until no viewpoint is worth it. Then it takes
 * a shaft its up and down range finders found to a floor it has not searched,
 * and finally returns to the floor it entered on.
 */
export class SensorExplorer {
  private readonly planner: SensorPlanner;
  private idle = 0;
  private phase: 'sweep' | 'look' | 'shaft' | 'done';
  private route: V3[] = [];
  private spin = 0;
  private entryFloor = -1;
  private readonly floorsDone = new Set<number>();
  /** Shaft leg: hover point (local xy) and the floor to reach. */
  private shaft: { x: number; y: number; to: number } | null = null;
  /** Centring on the shaft before climbing: the axis being probed, its direction, and the edges found. */
  private centring: { axis: 0 | 1; dir: 1 | -1; open: boolean; from: number; ends: number[]; ticks: number } | null = null;
  private centred = false;
  /** Progress watch on the climb: closest the drone has come to the target height, and ticks since. */
  private climbBest = Infinity;
  private followBest = Infinity;
  private followStall = 0;
  private climbStall = 0;

  constructor(
    readonly grid: IndoorGrid,
    readonly sensing: IndoorSensing,
    readonly speedCap = 1,
    start: 'sweep' | 'look' = 'sweep',
  ) {
    this.planner = new SensorPlanner({
      speed: speedCap,
      flyHeight: INDOOR_FLY_Z,
      horizon: INDOOR_FILTER_HORIZON,
      braking: sensing.braking ?? undefined,
    });
    this.phase = start;
  }

  get exhausted(): boolean {
    return this.phase === 'done';
  }

  get phaseName(): 'sweep' | 'look' | 'shaft' | 'done' {
    return this.phase;
  }

  /**
   * Out of time: stop searching and get back to the entry floor (the way out),
   * through the shaft if the drone is on another floor or climbing.
   */
  wrapUp(local: V3): void {
    if (this.phase === 'done') return;
    for (let f = 0; f < this.grid.floors; f++) this.floorsDone.add(f);
    const f = floorOf(this.grid, local[2]);
    if (this.phase === 'shaft' && this.shaft) {
      // Already in the column: climb or descend straight to the entry floor.
      if (!this.route.length) this.shaft.to = this.entryFloor;
      else if (f === this.entryFloor) this.phase = 'done';
      return;
    }
    if (f === this.entryFloor) {
      this.phase = 'done';
      return;
    }
    this.route = [];
    this.spin = 0;
    this.phase = 'look';
    this.floorDone(local, 0);
  }

  act(local: V3, vel: V3, yaw: number, tSec: number): PolicyCommand {
    this.sensing.sense(local, vel, yaw, tSec);
    if (this.entryFloor < 0) this.entryFloor = floorOf(this.grid, local[2]);
    if (this.phase === 'sweep') {
      const cmd = this.planner.act(this.sensing.frame(local, yaw));
      this.idle = cmd.exploring ? 0 : this.idle + 1;
      if (this.idle >= DONE_STEPS) {
        this.idle = 0;
        this.phase = 'look';
      } else {
        this.sensing.yawRate = cmd.yawRate;
        return { vel: [cmd.vx, cmd.vy, cmd.vz], yawRate: cmd.yawRate };
      }
    }
    if (this.phase === 'look') return this.lookStep(local, yaw);
    if (this.phase === 'shaft') return this.shaftStep(local, yaw);
    return { vel: [0, 0, 0], yawRate: 0 };
  }

  private hold(local: V3): number {
    const f = floorOf(this.grid, local[2]);
    const vz = clamp((f * this.grid.H + INDOOR_FLY_Z - local[2]) * 2, -0.6, 0.6);
    return vz < 0 && !descentClear(this.sensing.lidar) ? 0 : vz;
  }

  /** Follow this.route on the current floor; true once the last point is reached. */
  private follow(local: V3, yaw: number, out: { cmd: PolicyCommand | null }): boolean {
    const target = this.route[0]!;
    const dx = target[0] - local[0];
    const dy = target[1] - local[1];
    const d = Math.hypot(dx, dy);
    // A waypoint the shield will not let the drone reach (it hugs a wall the map put a hair too far)
    // is skipped after a while, as the runner's routed moves do.
    if (d < this.followBest - 0.03) {
      this.followBest = d;
      this.followStall = 0;
    } else if ((this.followStall += 1) > FOLLOW_STALL_TICKS && this.route.length > 1) {
      this.route.shift();
      this.followBest = Infinity;
      this.followStall = 0;
      out.cmd = { vel: [0, 0, this.hold(local)], yawRate: 0 };
      return false;
    }
    if (d < (this.route.length === 1 ? 0.12 : 0.2)) {
      this.route.shift();
      this.followBest = Infinity;
      this.followStall = 0;
      out.cmd = { vel: [0, 0, this.hold(local)], yawRate: 0 };
      return this.route.length === 0;
    }
    const sp = Math.min(this.speedCap, 0.3 + d);
    const [vx, vy] = this.sensing.filter((dx / d) * sp, (dy / d) * sp, yaw);
    const yawRate = clamp(wrapAngle(Math.atan2(dy, dx) - yaw) * 2, -MAX_YAW_RATE_INDOOR, MAX_YAW_RATE_INDOOR);
    this.sensing.yawRate = yawRate;
    out.cmd = { vel: [vx, vy, this.hold(local)], yawRate };
    return false;
  }

  private lookStep(local: V3, yaw: number): PolicyCommand {
    const f = floorOf(this.grid, local[2]);
    if (this.spin > 0) {
      this.spin -= 1;
      this.sensing.yawRate = MAX_YAW_RATE_INDOOR;
      return { vel: [0, 0, this.hold(local)], yawRate: MAX_YAW_RATE_INDOOR };
    }
    if (!this.route.length) {
      const goal = bestViewpoint(this.sensing, f, local);
      const path = goal ? sensorPath(this.sensing, f, [local[0], local[1]], [goal[0], goal[1]]) : null;
      if (!path) return this.floorDone(local, yaw);
      this.route = path.map(([x, y]) => [x, y, f * this.grid.H + INDOOR_FLY_Z] as V3);
    }
    const out: { cmd: PolicyCommand | null } = { cmd: null };
    if (this.follow(local, yaw, out)) this.spin = Math.ceil((Math.PI * 2) / MAX_YAW_RATE_INDOOR / 0.05) + 2;
    return out.cmd!;
  }

  /** This floor is searched: pick a shaft to an unsearched floor, or head back to the entry floor, or stop. */
  private floorDone(local: V3, yaw: number): PolicyCommand {
    const f = floorOf(this.grid, local[2]);
    this.floorsDone.add(f);
    const want: number[] = [];
    for (const g of [f + 1, f - 1]) if (g >= 0 && g < this.grid.floors && !this.floorsDone.has(g)) want.push(g);
    if (!want.length && f !== this.entryFloor) want.push(f > this.entryFloor ? f - 1 : f + 1);
    for (const g of want) {
      const at = this.sensing.shaftPoint(f, g > f ? 'up' : 'down', local);
      if (!at) continue;
      const path = sensorPath(this.sensing, f, [local[0], local[1]], at);
      if (!path) continue;
      this.route = path.map(([x, y]) => [x, y, f * this.grid.H + INDOOR_FLY_Z] as V3);
      this.shaft = { x: at[0], y: at[1], to: g };
      this.centred = false;
      this.centring = null;
      this.climbBest = Infinity;
      this.climbStall = 0;
      this.phase = 'shaft';
      return this.shaftStep(local, yaw);
    }
    this.phase = 'done';
    return { vel: [0, 0, 0], yawRate: 0 };
  }

  private shaftStep(local: V3, yaw: number): PolicyCommand {
    const shaft = this.shaft!;
    if (this.route.length) {
      const out: { cmd: PolicyCommand | null } = { cmd: null };
      this.follow(local, yaw, out);
      return out.cmd!;
    }
    if (!this.centred) return this.centreStep(local, yaw);
    // In the column: hold xy on the shaft centre and climb or descend to the next floor's flight height.
    const zTarget = shaft.to * this.grid.H + INDOOR_FLY_Z;
    const ez = zTarget - local[2];
    if (Math.abs(ez) < 0.08) {
      this.shaft = null;
      this.planner.reset();
      this.phase = this.floorsDone.has(shaft.to) ? 'look' : 'sweep';
      if (this.phase === 'look' && shaft.to === this.entryFloor) this.phase = 'done';
      return { vel: [0, 0, 0], yawRate: 0 };
    }
    // A climb that stops gaining height is not a shaft (open air over a doorway, say): forget it and
    // pick again. Only before the drone has left its floor; once between floors it keeps going.
    if (Math.abs(ez) < this.climbBest - 0.02) {
      this.climbBest = Math.abs(ez);
      this.climbStall = 0;
    } else if ((this.climbStall += 1) > CLIMB_STALL_TICKS && Math.abs(ez) > this.grid.H - 0.5) {
      const f = floorOf(this.grid, local[2]);
      this.sensing.forgetShaft(f, shaft.to > f ? 'up' : 'down', [shaft.x, shaft.y]);
      this.shaft = null;
      this.phase = 'look';
      return this.floorDone(local, yaw);
    }
    const vx = clamp((shaft.x - local[0]) * 2, -0.3, 0.3);
    const vy = clamp((shaft.y - local[1]) * 2, -0.3, 0.3);
    return { vel: [vx, vy, clamp(ez * 1.5, -0.5, 0.5)], yawRate: 0 };
  }

  /**
   * Before a climb or descent, find the hole's edges with the range finder:
   * fly slowly along each building axis until it stops seeing through, both
   * ways, and centre between the edges. The shaft point alone is only known
   * to be somewhere over the hole, possibly at its rim, and the airframe is
   * wider than a point.
   */
  private centreStep(local: V3, yaw: number): PolicyCommand {
    const shaft = this.shaft!;
    const f = floorOf(this.grid, local[2]);
    const open = this.sensing.seesThrough(shaft.to > f ? 'up' : 'down', local[2] - f * this.grid.H);
    const at = [shaft.x, shaft.y];
    const c = (this.centring ??= { axis: 0, dir: 1, open: false, from: at[0]!, ends: [], ticks: 0 });
    const p = local[c.axis]!;
    c.open ||= open;
    c.ticks += 1;
    // An edge: the reading closed after being open, or the probe ran out of room or time (a wall the
    // shield will not let it approach).
    if ((c.open && !open) || Math.abs(p - c.from) > CENTRE_PROBE_M || c.ticks > CENTRE_PROBE_TICKS) {
      c.ends.push(p);
      c.open = false;
      c.ticks = 0;
      if (c.dir === 1) c.dir = -1;
      else {
        at[c.axis] = (c.ends[0]! + c.ends[1]!) / 2;
        shaft.x = at[0]!;
        shaft.y = at[1]!;
        if (c.axis === 1) {
          this.centred = true;
          this.centring = null;
          return { vel: [0, 0, this.hold(local)], yawRate: 0 };
        }
        this.centring = { axis: 1, dir: 1, open: false, from: at[1]!, ends: [], ticks: 0 };
      }
    }
    const k = this.centring!;
    const v: [number, number] = [clamp((at[0]! - local[0]) * 2, -0.3, 0.3), clamp((at[1]! - local[1]) * 2, -0.3, 0.3)];
    v[k.axis] = k.dir * CENTRE_PROBE_SPEED;
    const [vx, vy] = this.sensing.filter(v[0], v[1], yaw);
    return { vel: [vx, vy, this.hold(local)], yawRate: 0 };
  }
}

/** Speed and reach of the range-finder probe that centres the drone on a shaft, m/s and m. */
const CENTRE_PROBE_SPEED = 0.3;
const CENTRE_PROBE_M = 0.9;
const CENTRE_PROBE_TICKS = 120;
/** A climb or descent that gains no height for this many ticks gives the shaft up. */
const CLIMB_STALL_TICKS = 60;
/** Ticks without progress toward a route waypoint before it is skipped (not the last one). */
const FOLLOW_STALL_TICKS = 60;

/** Yaw rate for turning in place indoors, rad/s (the vehicle's indoor limit). */
const MAX_YAW_RATE_INDOOR = Math.PI / 2;

/**
 * The reachable known-free voxel that would show the most unlooked voxels
 * per meter of travel (gain / (1 + 0.3 * path length)), judged on the
 * drone's own map. Null when no voxel would show MIN_LOOK_GAIN.
 */
export function bestViewpoint(sensing: IndoorSensing, floor: number, from: V3): [number, number] | null {
  const { vw, vd } = sensing;
  const base = floor * vw * vd;
  const open = (x: number, y: number) => {
    if (x < 0 || y < 0 || x >= vw || y >= vd) return false;
    const k = base + y * vw + x;
    return sensing.visited[k] === 1 || (sensing.seen[k] === 1 && sensing.occupied[k] !== 1);
  };
  const unlooked: number[] = [];
  for (let y = 0; y < vd; y++) for (let x = 0; x < vw; x++) if (open(x, y) && !sensing.looked[base + y * vw + x]) unlooked.push(y * vw + x);
  if (unlooked.length < MIN_LOOK_GAIN) return null;
  // BFS path lengths from the drone over known-free voxels.
  const dist = new Float32Array(vw * vd).fill(Infinity);
  const sx = clamp(Math.floor(from[0] / VOXEL), 0, vw - 1);
  const sy = clamp(Math.floor(from[1] / VOXEL), 0, vd - 1);
  const queue = [sy * vw + sx];
  dist[sy * vw + sx] = 0;
  for (let h = 0; h < queue.length; h++) {
    const k = queue[h]!;
    const x = k % vw;
    const y = (k - x) / vw;
    for (const [dx, dy] of [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
    ] as const) {
      const nx = x + dx;
      const ny = y + dy;
      if (!open(nx, ny)) continue;
      const nk = ny * vw + nx;
      if (dist[nk]! <= dist[k]! + 1) continue;
      dist[nk] = dist[k]! + 1;
      queue.push(nk);
    }
  }
  const range = sensing.lookRange - 0.5;
  let best: [number, number] | null = null;
  let bestScore = 0;
  for (const k of queue) {
    const x = k % vw;
    const y = (k - x) / vw;
    // Viewpoints keep a voxel of known free air around them.
    if (sensorClearance(sensing, floor, (x + 0.5) * VOXEL, (y + 0.5) * VOXEL) < 2 * VOXEL) continue;
    const px = (x + 0.5) * VOXEL;
    const py = (y + 0.5) * VOXEL;
    let gain = 0;
    for (const u of unlooked) {
      const ux = ((u % vw) + 0.5) * VOXEL;
      const uy = (Math.floor(u / vw) + 0.5) * VOXEL;
      const dd = (ux - px) ** 2 + (uy - py) ** 2;
      if (dd > range * range || dd < sensing.lookMin * sensing.lookMin) continue;
      if (sensorLos(sensing, floor, px, py, ux, uy)) gain += 1;
    }
    if (gain < MIN_LOOK_GAIN) continue;
    const score = gain / (1 + 0.3 * dist[k]! * VOXEL);
    if (score > bestScore) {
      bestScore = score;
      best = [px, py];
    }
  }
  return best;
}

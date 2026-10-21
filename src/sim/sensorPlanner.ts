import * as C from '../shared/constants';
import type { GainSource } from './mapPredict';
import { approachSpeed, CommandShaper, profileSpeed, uTurnSpeed, resolveTracker, type ResolvedTracker, type TrackerOptions } from './tracker';

// Module-local copies (imported bindings are getter calls under vite-node).
const LIDAR_H = C.LIDAR_H;
const LIDAR_MAX = C.LIDAR_MAX;
const VOXEL = C.VOXEL;
const MAX_SPEED_H = C.MAX_SPEED_H;
const MAX_SPEED_V = C.MAX_SPEED_V;
const MAX_YAW_RATE = C.MAX_YAW_RATE;
const DRONE_RADIUS = C.DRONE_RADIUS;
const POLICY_DT = C.POLICY_DT;

/**
 * What a robot actually has: its own pose, its lidar, and the maps it built
 * from them. Nothing here comes from the building's floor plan. `vw`, `vd`,
 * `floors` and `floorHeight` only size the grids (the building's outer
 * footprint, which a drone measures from outside before it enters).
 */
export interface SensorFrame {
  /** Seen-free voxels (SeenMap layout: f * vw * vd + vy * vw + vx). */
  seen: Uint8Array;
  /** Voxels the drone center has flown through, same layout. */
  visited: Uint8Array;
  /** Voxels a lidar return landed in (walls, furniture), same layout. Optional. */
  occupied?: Uint8Array;
  vw: number;
  vd: number;
  floors: number;
  floorHeight: number;
  pos: readonly [number, number, number];
  yaw: number;
  /** Horizontal lidar ranges in meters, ray i at yaw + i * 2π / LIDAR_H. LIDAR_MAX means no return. */
  lidar: ArrayLike<number>;
  /** Distance to whatever is below, meters. */
  alt: number;
  /** Measured horizontal velocity in the world frame, m/s (the IMU's estimate). The braking filter needs it. */
  vel?: readonly [number, number];
}

/** World-frame command. */
export interface SensorCommand {
  vx: number;
  vy: number;
  vz: number;
  yawRate: number;
  /** False when no reachable frontier is left on this floor: exploration is done. */
  exploring: boolean;
}

export interface SensorPlannerOptions {
  /** Cruise speed, m/s. */
  speed?: number;
  /** Flight height above the floor, meters. */
  flyHeight?: number;
  /** Replan every this many steps even when the goal is still valid. */
  replanEvery?: number;
  /** Turn to face the direction of travel (the camera looks forward). */
  faceTravel?: boolean;
  /** Path-cost credit per flown or blocked neighbour of a frontier voxel. */
  edgeBonus?: number;
  /** Safety-filter horizon, seconds. */
  horizon?: number;
  /**
   * Stopping model of the airframe. When set, the lidar filter keeps the
   * drone able to stop before every return given its measured velocity
   * (brakingFilter) instead of the kinematic closing-speed rule.
   */
  braking?: BrakingModel;
  /**
   * Meters of path a full reversal at 1 m/s costs when choosing the next
   * frontier: goals ahead of the current velocity win ties, so a drone that
   * can't turn on the spot sweeps instead of zig-zagging. 0 = off.
   */
  momentum?: number;
  /** Margin the braking filter keeps, meters (default BRAKING_MARGIN). */
  brakingMargin?: number;
  /**
   * Hidden-gain estimate for frontier candidates (mapPredict.ts). When set,
   * a candidate's cost is its path cost minus gainWeight × its gain, so a
   * frontier that likely opens into unseen space beats a nearer dead end.
   */
  gain?: GainSource;
  /** Path cost, in voxel steps, traded for one voxel of predicted gain. */
  gainWeight?: number;
  /**
   * Extra cost per frontier candidate, in voxel steps (team/allocate.ts: frontiers a
   * teammate has claimed cost more). Must be zero or positive.
   */
  penalty?: CandidatePenalty;
  /**
   * Quad-aware trajectory tracking (sim/tracker.ts): a speed profile from the
   * corners of the path ahead and an acceleration limit on the command. Off
   * by default; pass {} for the tuned defaults.
   */
  tracker?: TrackerOptions;
}

/** Non-negative extra cost of a frontier candidate, for coordinating with other drones. */
export interface CandidatePenalty {
  /** Called once per replan, before any cost() on that floor. */
  prepare(floor: number, vw: number): void;
  /** Cost of voxel k on the floor plane (k = vy * vw + vx), in voxel steps. */
  cost(k: number): number;
}

/** Free air the safety filter keeps between the airframe edge and a lidar return, meters. */
export const SAFETY_MARGIN = 0.2;
/** Seconds of travel the filter allows toward a return: v·u ≤ (r − radius − margin) / horizon. */
export const SAFETY_HORIZON = 0.3;

/**
 * How an airframe stops: after a command to stop it keeps going for about
 * `reaction` seconds (latency, then tilting back), then decelerates at
 * `decel` m/s². Stopping distance from speed v is v·reaction + v²/(2·decel).
 */
export interface BrakingModel {
  reaction: number;
  decel: number;
}

/**
 * Stopping model of the simulator's quadrotor (sim/dynamics/quad.ts), fitted by
 * scripts/study/quad-sysid.ts to the 97th percentile of stopping distance over
 * domain-randomized airframes with two control ticks of latency, the most the
 * noise model draws. It is the airframe's own spec, not knowledge of the
 * building.
 */
export const QUAD_BRAKING: BrakingModel = { reaction: 0.21, decel: 5 };

/**
 * Air the braking filter keeps past the airframe edge, meters: for the planner,
 * and for the policy guard. Picked on the planner-tuning buildings
 * (9101-9108) as the smallest margins that bring quad collisions under about
 * one per episode; larger margins cost coverage in doorways.
 */
export const BRAKING_MARGIN = 0.25;
export const GUARD_BRAKING_MARGIN = 0.15;

/** Fastest approach speed from which `model` stops within `d` meters. */
export function safeSpeed(d: number, model: BrakingModel): number {
  if (!(d > 0)) return 0;
  const a = model.decel;
  const t = model.reaction;
  return a * (Math.sqrt(t * t + (2 * d) / a) - t);
}

/**
 * Model-aware lidar filter, a discrete control barrier on stopping distance.
 * For each horizontal ray with a return at range r, with d = r − radius −
 * margin, the barrier is h = d − S(v), where S is the airframe's stopping
 * distance at the speed v toward the return. The commanded speed toward the
 * return is capped at safeSpeed(d). When the measured speed toward it is
 * already past that cap (the drone is still carrying momentum from a turn),
 * the cap drops below it by the excess, so the controller brakes hard.
 *
 * A NaN range is a ray whose reading is missing (a failed sector): it is
 * treated as a return at the margin, so the drone never moves toward a sector
 * it cannot see. A range at LIDAR_MAX or more, or Infinity, is open air.
 */
export function brakingFilter(
  vx: number,
  vy: number,
  yaw: number,
  lidar: ArrayLike<number>,
  vel: readonly [number, number] | null | undefined,
  model: BrakingModel,
  margin = BRAKING_MARGIN,
): [number, number] {
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const mx = vel ? vel[0] : vx;
  const my = vel ? vel[1] : vy;
  for (let pass = 0; pass < 3; pass++) {
    let changed = false;
    for (let i = 0; i < LIDAR_H; i++) {
      const raw = lidar[i]!;
      const r = Number.isNaN(raw) ? DRONE_RADIUS + margin : raw;
      if (r >= LIDAR_MAX - 1e-3) continue;
      const ux = cy * RAY_COS[i]! - sy * RAY_SIN[i]!;
      const uy = sy * RAY_COS[i]! + cy * RAY_SIN[i]!;
      const cap = safeSpeed(r - DRONE_RADIUS - margin, model);
      const measured = mx * ux + my * uy;
      const allowed = measured > cap ? cap - (measured - cap) : cap;
      const along = vx * ux + vy * uy;
      if (along > allowed) {
        const cut = along - allowed;
        vx -= cut * ux;
        vy -= cut * uy;
        changed = true;
      }
    }
    if (!changed) break;
  }
  return [vx, vy];
}

const DX = [1, -1, 0, 0, 1, 1, -1, -1];
const DY = [0, 0, 1, -1, 1, -1, 1, -1];
const STEP = [1, 1, 1, 1, Math.SQRT2, Math.SQRT2, Math.SQRT2, Math.SQRT2];

const RAY_COS = new Float64Array(LIDAR_H);
const RAY_SIN = new Float64Array(LIDAR_H);
for (let i = 0; i < LIDAR_H; i++) {
  RAY_COS[i] = Math.cos((i / LIDAR_H) * Math.PI * 2);
  RAY_SIN[i] = Math.sin((i / LIDAR_H) * Math.PI * 2);
}

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

function wrap(a: number): number {
  return a - Math.round(a / (Math.PI * 2)) * Math.PI * 2;
}

/**
 * Lidar velocity filter. Projects a world-frame horizontal velocity so that,
 * along every horizontal ray with a return, the drone closes at most
 * (range − radius − margin) / horizon m/s. Rays without a return (misses,
 * glass or night dropouts) constrain nothing. Uses only the lidar.
 */
export function filterVelocity(
  vx: number,
  vy: number,
  yaw: number,
  lidar: ArrayLike<number>,
  margin = SAFETY_MARGIN,
  horizon = SAFETY_HORIZON,
): [number, number] {
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  for (let pass = 0; pass < 3; pass++) {
    let changed = false;
    for (let i = 0; i < LIDAR_H; i++) {
      const r = lidar[i]!;
      if (r >= LIDAR_MAX - 1e-3) continue;
      const ux = cy * RAY_COS[i]! - sy * RAY_SIN[i]!;
      const uy = sy * RAY_COS[i]! + cy * RAY_SIN[i]!;
      const allowed = Math.max(0, (r - DRONE_RADIUS - margin) / horizon);
      const along = vx * ux + vy * uy;
      if (along > allowed) {
        const cut = along - allowed;
        vx -= cut * ux;
        vy -= cut * uy;
        changed = true;
      }
    }
    if (!changed) break;
  }
  return [vx, vy];
}

/**
 * Classical, sensor-only frontier explorer. It plans on the voxels its own
 * lidar has seen free or it has flown through, never on the floor plan:
 * Dijkstra from the drone to the cheapest frontier voxel (seen free, not yet
 * flown), with a cost that keeps paths off voxels next to unknown space,
 * pure-pursuit along the path, a lidar velocity filter, and an altitude hold.
 * A goal that stops getting closer is dropped and remembered.
 *
 * This is the honest counterpart of the privileged teacher in sim/frontier.ts,
 * which plans on the true floor plan: it is the baseline the learned policy
 * has to beat, and the mission's indoor fallback.
 */
export class SensorPlanner {
  readonly speed: number;
  readonly flyHeight: number;
  readonly replanEvery: number;
  readonly faceTravel: boolean;
  readonly edgeBonus: number;
  readonly horizon: number;
  readonly braking: BrakingModel | null;
  readonly brakingMargin: number;
  readonly momentum: number;
  readonly gain: GainSource | null;
  readonly gainWeight: number;
  /** Swappable so a team can attach its claim map after construction. */
  penalty: CandidatePenalty | null;
  readonly tracker: ResolvedTracker | null;
  private readonly shaper = new CommandShaper();
  private readonly corners: [number, number][] = [];
  /** The last string-pulled point is the path's end (the goal), not a lookahead cut. */
  private pulledToEnd = false;
  private n = 0;
  private dist = new Float64Array(0);
  private prev = new Int32Array(0);
  private clear = new Uint8Array(0);
  private heap = new Int32Array(0);
  private banned = new Uint8Array(0);
  private path: number[] = [];
  private goal = -1;
  private goalFloor = -1;
  private sinceReplan = 0;
  private bestGoalDist = Infinity;
  private noProgress = 0;
  private lastFloor = -1;
  /** Steps in a row without a reachable frontier. */
  private idle = 0;

  constructor(opts: SensorPlannerOptions = {}) {
    this.speed = opts.speed ?? 2;
    this.flyHeight = opts.flyHeight ?? 1.2;
    this.replanEvery = opts.replanEvery ?? 5;
    this.faceTravel = opts.faceTravel ?? true;
    this.edgeBonus = opts.edgeBonus ?? 0.3;
    this.horizon = opts.horizon ?? SAFETY_HORIZON;
    this.braking = opts.braking ?? null;
    this.brakingMargin = opts.brakingMargin ?? BRAKING_MARGIN;
    this.momentum = opts.momentum ?? 0;
    this.gain = opts.gain ?? null;
    this.gainWeight = opts.gain ? (opts.gainWeight ?? 0.05) : 0;
    this.penalty = opts.penalty ?? null;
    this.tracker = opts.tracker ? resolveTracker(opts.tracker) : null;
  }

  reset(): void {
    this.n = 0;
    this.path = [];
    this.goal = -1;
    this.goalFloor = -1;
    this.sinceReplan = 0;
    this.bestGoalDist = Infinity;
    this.noProgress = 0;
    this.lastFloor = -1;
    this.idle = 0;
    this.shaper.reset();
  }

  /** Goals dropped for lack of progress (diagnostics). */
  get bannedCount(): number {
    let n = 0;
    for (let i = 0; i < this.banned.length; i++) n += this.banned[i]!;
    return n;
  }

  /** The current goal voxel center in world xy, or null. */
  get goalPoint(): [number, number] | null {
    if (this.goal < 0 || this.n === 0) return null;
    const vw = this.vwCache;
    const k = this.goal % this.plane;
    return [((k % vw) + 0.5) * VOXEL, (Math.floor(k / vw) + 0.5) * VOXEL];
  }

  private vwCache = 1;
  private plane = 1;

  private ensure(f: SensorFrame): void {
    const n = f.floors * f.vw * f.vd;
    if (n === this.n) return;
    this.n = n;
    this.vwCache = f.vw;
    this.plane = f.vw * f.vd;
    this.dist = new Float64Array(this.plane);
    this.prev = new Int32Array(this.plane);
    this.clear = new Uint8Array(this.plane);
    this.heap = new Int32Array(this.plane * 8 + 16);
    this.banned = new Uint8Array(n);
  }

  private floorOf(f: SensorFrame): number {
    return clamp(Math.floor(f.pos[2] / f.floorHeight), 0, f.floors - 1);
  }

  private traversable(f: SensorFrame, base: number, k: number): boolean {
    return f.visited[base + k] === 1 || (f.seen[base + k] === 1 && !(f.occupied && f.occupied[base + k] === 1));
  }

  /** Voxel distance (0..3) from each traversable voxel to the nearest non-traversable one, on one floor. */
  private clearance(f: SensorFrame, base: number): void {
    const { vw, vd } = f;
    const c = this.clear;
    for (let k = 0; k < this.plane; k++) c[k] = this.traversable(f, base, k) ? 3 : 0;
    for (let level = 1; level <= 2; level++) {
      for (let y = 0; y < vd; y++) {
        for (let x = 0; x < vw; x++) {
          const k = y * vw + x;
          if (c[k]! <= level) continue;
          let near = x === 0 || y === 0 || x === vw - 1 || y === vd - 1;
          for (let d = 0; d < 8 && !near; d++) {
            const nx = x + DX[d]!;
            const ny = y + DY[d]!;
            if (c[ny * vw + nx]! === level - 1) near = true;
          }
          if (near) c[k] = level;
        }
      }
    }
  }

  /** Dijkstra from the drone voxel; returns the chosen frontier voxel index on this floor plane, or -1. */
  private plan(f: SensorFrame, floor: number): number {
    const { vw, vd } = f;
    const base = floor * this.plane;
    this.clearance(f, base);
    const dist = this.dist;
    const prev = this.prev;
    dist.fill(Infinity);
    prev.fill(-1);
    const sx = clamp(Math.floor(f.pos[0] / VOXEL), 0, vw - 1);
    const sy = clamp(Math.floor(f.pos[1] / VOXEL), 0, vd - 1);
    const start = sy * vw + sx;
    dist[start] = 0;
    // Binary heap of (dist, index) packed as indices; dist read from the array (lazy deletion).
    const heap = this.heap;
    let size = 0;
    const push = (k: number) => {
      let i = size++;
      heap[i] = k;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (dist[heap[p]!]! <= dist[heap[i]!]!) break;
        const t = heap[p]!;
        heap[p] = heap[i]!;
        heap[i] = t;
        i = p;
      }
    };
    const pop = (): number => {
      const top = heap[0]!;
      heap[0] = heap[--size]!;
      let i = 0;
      for (;;) {
        const l = i * 2 + 1;
        const r = l + 1;
        let m = i;
        if (l < size && dist[heap[l]!]! < dist[heap[m]!]!) m = l;
        if (r < size && dist[heap[r]!]! < dist[heap[m]!]!) m = r;
        if (m === i) break;
        const t = heap[m]!;
        heap[m] = heap[i]!;
        heap[i] = t;
        i = m;
      }
      return top;
    };
    push(start);
    const done = this.clear; // reuse: bit 7 marks settled
    let best = -1;
    let bestCost = Infinity;
    let keepCost = Infinity;
    const keep = this.goalFloor === floor ? this.goal % this.plane : -1;
    const gain = this.gain;
    const gw = this.gainWeight;
    const mom = this.momentum;
    const vel = f.vel ?? [0, 0];
    const vmag = Math.hypot(vel[0], vel[1]);
    // How far past the cheapest candidate a better one could still lie, once gains are counted.
    const reach = gain ? gw * gain.max : 0;
    if (gain) gain.prepare({ vw, vd, base, seen: f.seen, visited: f.visited, occupied: f.occupied });
    const claim = this.penalty;
    claim?.prepare(floor, vw);
    while (size > 0) {
      const k = pop();
      if (done[k]! & 128) continue;
      done[k] = done[k]! | 128;
      const d0 = dist[k]!;
      if (k !== start && f.visited[base + k] !== 1 && this.traversable(f, base, k) && !this.banned[base + k]) {
        // Prefer frontier voxels that border flown or blocked space: sweeping edges first
        // leaves fewer isolated stragglers to come back for.
        let score = d0 - this.edgeBonus * this.edgeCount(f, base, k);
        // Momentum: a goal behind the drone costs the distance it takes to stop and come back.
        if (mom > 0) {
          const ox = ((k % vw) + 0.5) * VOXEL - f.pos[0];
          const oy = (Math.floor(k / vw) + 0.5) * VOXEL - f.pos[1];
          const ol = Math.hypot(ox, oy);
          if (ol > 1e-6) score += mom * (1 - (ox * vel[0] + oy * vel[1]) / (ol * vmag + 1e-6)) * 0.5 * vmag;
        }
        // The gain is only worth computing where it could still change the choice.
        if (gain && score - reach < bestCost) score -= gw * gain.gain(k);
        // A penalty only raises the score, so the early-exit bound below still holds.
        if (claim) score += claim.cost(k);
        if (k === keep) keepCost = score;
        if (score < bestCost) {
          bestCost = score;
          best = k;
        }
      }
      if (d0 - 8 * this.edgeBonus - reach > bestCost + 1.5 && (keep < 0 || keepCost < Infinity)) break;
      const x = k % vw;
      const y = (k - x) / vw;
      for (let d = 0; d < 8; d++) {
        const nx = x + DX[d]!;
        const ny = y + DY[d]!;
        if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
        const nk = ny * vw + nx;
        if (!this.traversable(f, base, nk)) continue;
        if (d >= 4 && (!this.traversable(f, base, y * vw + nx) || !this.traversable(f, base, ny * vw + x))) continue;
        const cl = done[nk]! & 127;
        const penalty = cl <= 1 ? 2.5 : cl === 2 ? 0.6 : 0;
        const nd = d0 + STEP[d]! * (1 + penalty);
        if (nd < dist[nk]!) {
          dist[nk] = nd;
          prev[nk] = k;
          if (size < heap.length) push(nk);
        }
      }
    }
    for (let k = 0; k < this.plane; k++) done[k] = done[k]! & 127;
    // Hysteresis: keep the current goal unless another is clearly cheaper.
    if (keep >= 0 && keepCost < Infinity && keepCost <= bestCost + 1.5) best = keep;
    if (best < 0) return -1;
    const path: number[] = [];
    for (let k = best; k >= 0 && k !== start; k = prev[k]!) path.push(k);
    path.reverse();
    this.path = path;
    return best;
  }

  /** Neighbours of k (8-connected) that are flown or not traversable. */
  private edgeCount(f: SensorFrame, base: number, k: number): number {
    const x = k % f.vw;
    const y = (k - x) / f.vw;
    let n = 0;
    for (let d = 0; d < 8; d++) {
      const nx = x + DX[d]!;
      const ny = y + DY[d]!;
      if (nx < 0 || ny < 0 || nx >= f.vw || ny >= f.vd) {
        n += 1;
        continue;
      }
      const nk = base + ny * f.vw + nx;
      if (f.visited[nk] === 1 || !this.traversable(f, base, ny * f.vw + nx)) n += 1;
    }
    return n;
  }

  /** Straight segment between two world points stays on traversable voxels away from unknown space. */
  private lineClear(f: SensorFrame, base: number, ax: number, ay: number, bx: number, by: number): boolean {
    const l = Math.hypot(bx - ax, by - ay);
    const steps = Math.max(1, Math.ceil(l / (VOXEL * 0.4)));
    for (let i = 1; i <= steps; i++) {
      const t = i / steps;
      const x = Math.floor((ax + (bx - ax) * t) / VOXEL);
      const y = Math.floor((ay + (by - ay) * t) / VOXEL);
      if (x < 0 || y < 0 || x >= f.vw || y >= f.vd) return false;
      const k = y * f.vw + x;
      if (!this.traversable(f, base, k)) return false;
      if ((this.clear[k]! & 127) < 2 && i < steps) return false;
    }
    return true;
  }

  act(f: SensorFrame): SensorCommand {
    this.ensure(f);
    const floor = this.floorOf(f);
    const base = floor * this.plane;
    if (floor !== this.lastFloor) {
      this.goal = -1;
      this.lastFloor = floor;
    }
    const goalValid =
      this.goal >= 0 &&
      this.goalFloor === floor &&
      f.visited[this.goal] !== 1 &&
      this.traversable(f, 0, this.goal) &&
      !this.banned[this.goal];
    if (!goalValid || this.sinceReplan >= this.replanEvery) {
      const g = this.plan(f, floor);
      const next = g >= 0 ? base + g : -1;
      if (next !== this.goal) {
        this.bestGoalDist = Infinity;
        this.noProgress = 0;
      }
      this.goal = next;
      this.goalFloor = floor;
      this.sinceReplan = 0;
    } else {
      this.clearance(f, base);
    }
    this.sinceReplan += 1;

    const zTarget = floor * f.floorHeight + this.flyHeight;
    let vz = clamp((zTarget - f.pos[2]) * 2, -0.6, 0.6);
    // Something under the drone (a table, a bed): climb over it rather than settle onto it.
    if (vz < 0 && !descentClear(f.lidar)) vz = 0;
    if (vz < 0 && !descentClear(f.lidar)) vz = 0;
    if (f.alt < 0.55) vz = Math.max(vz, 0.4);

    if (this.goal < 0) {
      this.idle += 1;
      return { vx: 0, vy: 0, vz, yawRate: 0.6, exploring: this.idle < 20 };
    }
    this.idle = 0;

    // Progress watchdog: a goal that has not gotten closer in 60 steps is banned.
    const g = this.goal % this.plane;
    const gx = ((g % f.vw) + 0.5) * VOXEL;
    const gy = (Math.floor(g / f.vw) + 0.5) * VOXEL;
    const dGoal = Math.hypot(gx - f.pos[0], gy - f.pos[1]);
    if (dGoal < this.bestGoalDist - 0.05) {
      this.bestGoalDist = dGoal;
      this.noProgress = 0;
    } else if (++this.noProgress > 60) {
      this.banned[this.goal] = 1;
      this.goal = -1;
      this.noProgress = 0;
      this.bestGoalDist = Infinity;
    }

    // Pure pursuit: the farthest path voxel within 2.5 m that is in clear line of sight.
    let tx = gx;
    let ty = gy;
    const path = this.path;
    let picked = false;
    for (let i = Math.min(path.length - 1, 6); i >= 0; i--) {
      const k = path[i]!;
      const px = ((k % f.vw) + 0.5) * VOXEL;
      const py = (Math.floor(k / f.vw) + 0.5) * VOXEL;
      if (Math.hypot(px - f.pos[0], py - f.pos[1]) > 2.5) continue;
      if (this.lineClear(f, base, f.pos[0], f.pos[1], px, py)) {
        tx = px;
        ty = py;
        picked = true;
        break;
      }
    }
    if (!picked && path.length) {
      const k = path[0]!;
      tx = ((k % f.vw) + 0.5) * VOXEL;
      ty = (Math.floor(k / f.vw) + 0.5) * VOXEL;
    }
    const ex = tx - f.pos[0];
    const ey = ty - f.pos[1];
    const el = Math.hypot(ex, ey);
    const tr = this.tracker;
    let sp = this.speed;
    if (tr && el > 1e-6) {
      sp = profileSpeed(f.pos[0], f.pos[1], this.stringPull(f, base, tr.lookahead), sp, tr, this.pulledToEnd);
      // Lane end: the sweep turns where the run of unflown air ahead stops, so arrive there at turning speed.
      if (tr.lane) sp = Math.min(sp, approachSpeed(uTurnSpeed(tr, sp), this.laneAhead(f, base, ex / el, ey / el, tr.lookahead), tr));
    }
    let vx = el > 1e-6 ? (ex / el) * sp : 0;
    let vy = el > 1e-6 ? (ey / el) * sp : 0;
    if (tr) [vx, vy] = this.shaper.shape(vx, vy, tr.accel, POLICY_DT, f.vel);
    const sx = vx;
    const sy = vy;
    [vx, vy] = this.braking
      ? brakingFilter(vx, vy, f.yaw, f.lidar, f.vel, this.braking, this.brakingMargin)
      : filterVelocity(vx, vy, f.yaw, f.lidar, SAFETY_MARGIN, this.horizon);
    // A cut by the safety filter is where the next ramp starts from.
    if (tr && (vx !== sx || vy !== sy)) this.shaper.hold(vx, vy);
    let yawRate = 0;
    if (this.faceTravel && el > 0.2) {
      const cap = tr ? Math.min(tr.yawRate, MAX_YAW_RATE) : MAX_YAW_RATE;
      yawRate = clamp(wrap(Math.atan2(ey, ex) - f.yaw) * 2, -cap, cap);
    }
    return { vx, vy, vz, yawRate, exploring: true };
  }

  /**
   * Meters of unflown, traversable air straight ahead along (ux, uy) before the
   * first flown or blocked voxel, up to `max`: where the current sweep lane ends.
   */
  private laneAhead(f: SensorFrame, base: number, ux: number, uy: number, max: number): number {
    const stepM = VOXEL * 0.5;
    let lastK = -1;
    for (let d = stepM; d <= max; d += stepM) {
      const x = Math.floor((f.pos[0] + ux * d) / VOXEL);
      const y = Math.floor((f.pos[1] + uy * d) / VOXEL);
      if (x < 0 || y < 0 || x >= f.vw || y >= f.vd) return d - stepM;
      const k = y * f.vw + x;
      if (k === lastK) continue;
      lastK = k;
      if (f.visited[base + k] === 1 && d > VOXEL) return d - stepM;
      if (!this.traversable(f, base, k)) return d - stepM;
    }
    return max;
  }

  /**
   * The path ahead as straight legs: from the drone, repeatedly jump to the
   * farthest path voxel in clear line of sight, up to `lookahead` meters of
   * path. Returns the leg end points (the last one is the path's end or the
   * lookahead cut).
   */
  private stringPull(f: SensorFrame, base: number, lookahead: number): [number, number][] {
    const out = this.corners;
    out.length = 0;
    this.pulledToEnd = false;
    const path = this.path;
    if (!path.length) return out;
    const vw = f.vw;
    const px = (k: number) => ((k % vw) + 0.5) * VOXEL;
    const py = (k: number) => (Math.floor(k / vw) + 0.5) * VOXEL;
    // Path voxels within the lookahead, by path length from the first one.
    let last = 0;
    let along = Math.hypot(px(path[0]!) - f.pos[0], py(path[0]!) - f.pos[1]);
    while (last + 1 < path.length && along < lookahead) {
      along += Math.hypot(px(path[last + 1]!) - px(path[last]!), py(path[last + 1]!) - py(path[last]!));
      last += 1;
    }
    let ax = f.pos[0];
    let ay = f.pos[1];
    let i = -1;
    while (i < last && out.length < 8) {
      let j = i + 1;
      for (let c = last; c > i + 1; c--) {
        if (this.lineClear(f, base, ax, ay, px(path[c]!), py(path[c]!))) {
          j = c;
          break;
        }
      }
      ax = px(path[j]!);
      ay = py(path[j]!);
      out.push([ax, ay]);
      i = j;
    }
    this.pulledToEnd = last === path.length - 1;
    return out;
  }
}

/** World-frame command to the env's body-frame tanh action. */
export function commandToAction(
  cmd: { vx: number; vy: number; vz: number; yawRate: number },
  yaw: number,
  out = new Float32Array(4),
  speedH = MAX_SPEED_H,
): Float32Array {
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const fwd = cy * cmd.vx + sy * cmd.vy;
  const left = -sy * cmd.vx + cy * cmd.vy;
  out[0] = clamp(fwd / speedH, -1, 1);
  out[1] = clamp(left / speedH, -1, 1);
  out[2] = clamp(cmd.vz / MAX_SPEED_V, -1, 1);
  out[3] = clamp(cmd.yawRate / MAX_YAW_RATE, -1, 1);
  return out;
}

/** Height above the floor the policy guard holds the policy near, meters. */
export const GUARD_HEIGHT = 1.15;
/** Lidar filter settings for the policy guard: only imminent contacts are cut. */
export const GUARD_MARGIN = 0.05;
export const GUARD_HORIZON = 0.2;

/**
 * Horizontal distance to the nearest wall the lidar ring sees below which
 * the drone does not descend: a window sill or a ledge just under the ring is
 * invisible to it (a drone that came in through a window would sink onto the
 * sill otherwise).
 */
export const DESCENT_CLEARANCE = 0.6;

export function descentClear(lidar: ArrayLike<number>): boolean {
  for (let i = 0; i < lidar.length; i++) if (lidar[i]! < DESCENT_CLEARANCE) return false;
  return true;
}

/**
 * Sensor-only safety guard for the learned policy, applied to its body-frame
 * tanh action. It nudges altitude toward GUARD_HEIGHT above the current floor
 * (over the 0.9 m furniture tops, under the 1.4 m ones, so the lidar ring sees
 * those) and cuts horizontal velocity only when a lidar return is about to be
 * touched. Inputs are the drone's own height, yaw and horizontal lidar.
 */
export function guardAction(
  action: ArrayLike<number>,
  heightAboveFloor: number,
  yaw: number,
  lidar: ArrayLike<number>,
  out = new Float32Array(4),
  horizon = GUARD_HORIZON,
  braking?: { model: BrakingModel; vel: readonly [number, number] | null },
  speedH = MAX_SPEED_H,
): Float32Array {
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const f = clamp(action[0] ?? 0, -1, 1) * speedH;
  const l = clamp(action[1] ?? 0, -1, 1) * speedH;
  const [vx, vy] = braking
    ? brakingFilter(cy * f - sy * l, sy * f + cy * l, yaw, lidar, braking.vel, braking.model, GUARD_BRAKING_MARGIN)
    : filterVelocity(cy * f - sy * l, sy * f + cy * l, yaw, lidar, GUARD_MARGIN, horizon);
  out[0] = (cy * vx + sy * vy) / speedH;
  out[1] = (-sy * vx + cy * vy) / speedH;
  out[2] = clamp((action[2] ?? 0) + (GUARD_HEIGHT - heightAboveFloor) * 2, -1, 1);
  if (out[2] < 0 && heightAboveFloor > 0.55 && !descentClear(lidar)) out[2] = 0;
  out[3] = clamp(action[3] ?? 0, -1, 1);
  return out;
}

/** The maps a sensor-only path query reads. */
export interface SensorGrid {
  seen: Uint8Array;
  visited: Uint8Array;
  occupied?: Uint8Array;
  vw: number;
  vd: number;
}

function gridOpen(g: SensorGrid, base: number, x: number, y: number): boolean {
  if (x < 0 || y < 0 || x >= g.vw || y >= g.vd) return false;
  const k = base + y * g.vw + x;
  return g.visited[k] === 1 || (g.seen[k] === 1 && !(g.occupied && g.occupied[k] === 1));
}

/**
 * Lidar occupancy evidence per voxel: how many returns landed in it and how
 * many rays passed through it. A voxel is occupied when returns outweigh
 * pass-throughs (hits * 2 > passes). A wall voxel with a hole smaller than the
 * drone (a broken window) takes most of the returns of the rays around it and
 * a few pass-throughs, so it stays blocked; a doorway whose jamb pokes into
 * the voxel takes a few grazing returns and many pass-throughs, so it stays
 * open. The seen-free map alone cannot tell these apart.
 */
export class OccupancyMap {
  readonly occupied: Uint8Array;
  private readonly hits: Uint8Array;
  private readonly passes: Uint8Array;
  private readonly fixed: Uint8Array;
  /** Mean height of the lidar returns in each voxel (NaN before the first), for expected(). */
  private readonly hitZ: Float32Array;

  constructor(
    readonly vw: number,
    readonly vd: number,
    floors: number,
  ) {
    const n = vw * vd * floors;
    this.occupied = new Uint8Array(n);
    this.hits = new Uint8Array(n);
    this.passes = new Uint8Array(n);
    this.fixed = new Uint8Array(n);
    this.hitZ = new Float32Array(n).fill(NaN);
  }

  /** Mark voxel k occupied whatever the lidar says (structure known some other way). */
  block(k: number): void {
    this.fixed[k] = 1;
    this.occupied[k] = 1;
  }

  private update(k: number): void {
    this.occupied[k] = this.fixed[k] || this.hits[k]! * 2 > this.passes[k]! ? 1 : 0;
  }

  /** Add one horizontal lidar ring taken at (x, y) on `floor`, from height `z` when known. */
  mark(floor: number, x: number, y: number, yaw: number, lidar: ArrayLike<number>, z = NaN): void {
    const { vw, vd } = this;
    const base = floor * vw * vd;
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    for (let i = 0; i < LIDAR_H; i++) {
      const r = lidar[i]!;
      // A beam flagged invalid (NaN) says nothing about free or occupied space.
      if (Number.isNaN(r)) continue;
      const ux = cy * RAY_COS[i]! - sy * RAY_SIN[i]!;
      const uy = sy * RAY_COS[i]! + cy * RAY_SIN[i]!;
      const hit = r < LIDAR_MAX - 1e-3;
      // Pass-throughs up to just short of the return (or the whole range on a miss).
      const free = hit ? r - 0.15 : LIDAR_MAX;
      const n = Math.max(1, Math.ceil(free / (VOXEL * 0.5)));
      let last = -1;
      for (let s = 1; s <= n; s++) {
        const d = (s / n) * free;
        const vx = Math.floor((x + ux * d) / VOXEL);
        const vy = Math.floor((y + uy * d) / VOXEL);
        if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) break;
        const k = base + vy * vw + vx;
        if (k === last) continue;
        last = k;
        if (this.passes[k]! < 255) this.passes[k] = this.passes[k]! + 1;
        this.update(k);
      }
      if (!hit) continue;
      // Step just past the surface so the return lands in the voxel behind it.
      const hx = Math.floor((x + ux * (r + 0.05)) / VOXEL);
      const hy = Math.floor((y + uy * (r + 0.05)) / VOXEL);
      if (hx < 0 || hy < 0 || hx >= vw || hy >= vd) continue;
      const k = base + hy * vw + hx;
      if (this.hits[k]! < 255) this.hits[k] = this.hits[k]! + 1;
      if (!Number.isNaN(z)) {
        const h = this.hitZ[k]!;
        this.hitZ[k] = Number.isNaN(h) ? z : h + (z - h) / Math.min(this.hits[k]!, 16);
      }
      this.update(k);
    }
  }

  /**
   * Range to the first occupied voxel along each ring beam from (x, y, z), into `out`; Infinity when
   * none within range. Only voxels with at least `minHits` lidar returns taken within `dz` of `z` count:
   * the map is 2D per floor, and a beam at another height can pass an opening, a sill or a table top.
   */
  expected(floor: number, x: number, y: number, z: number, yaw: number, out: Float64Array, minHits = 3, dz = 0.25): Float64Array {
    const { vw, vd } = this;
    const base = floor * vw * vd;
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    const step = VOXEL * 0.5;
    for (let i = 0; i < LIDAR_H; i++) {
      const ux = cy * RAY_COS[i]! - sy * RAY_SIN[i]!;
      const uy = sy * RAY_COS[i]! + cy * RAY_SIN[i]!;
      out[i] = Infinity;
      for (let d = step; d <= LIDAR_MAX; d += step) {
        const vx = Math.floor((x + ux * d) / VOXEL);
        const vy = Math.floor((y + uy * d) / VOXEL);
        if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) break;
        const k = base + vy * vw + vx;
        // Only walls the lidar itself returned from at this height: the outer wall ring is blocked in and can hold window holes.
        if (this.occupied[k] && this.hits[k]! >= minHits && Math.abs(this.hitZ[k]! - z) <= dz) {
          out[i] = d;
          break;
        }
      }
    }
    return out;
  }
}

/** Voxels (0..2) between (x, y) and the nearest voxel that is neither seen free nor flown. */
function gridClearance(g: SensorGrid, base: number, x: number, y: number): number {
  if (!gridOpen(g, base, x, y)) return 0;
  for (let r = 1; r <= 2; r++) {
    for (let dy = -r; dy <= r; dy++) {
      for (let dx = -r; dx <= r; dx++) {
        if (Math.max(Math.abs(dx), Math.abs(dy)) !== r) continue;
        if (!gridOpen(g, base, x + dx, y + dy)) return r;
      }
    }
  }
  return 3;
}

/** Straight line between two xy points crosses only seen-free or flown voxels. */
export function sensorLos(g: SensorGrid, floor: number, ax: number, ay: number, bx: number, by: number, skipLast = 0): boolean {
  const base = floor * g.vw * g.vd;
  const l = Math.hypot(bx - ax, by - ay);
  const n = Math.max(1, Math.ceil(l / (VOXEL * 0.4)));
  const end = Math.max(0, l - skipLast);
  for (let i = 1; i <= n; i++) {
    if ((i / n) * l > end) break;
    const t = i / n;
    if (!gridOpen(g, base, Math.floor((ax + (bx - ax) * t) / VOXEL), Math.floor((ay + (by - ay) * t) / VOXEL))) return false;
  }
  return true;
}

/** Free air around an xy point in the sensor map, meters (0, 0.5, 1 or 1.5: voxels to unknown space). */
export function sensorClearance(g: SensorGrid, floor: number, x: number, y: number): number {
  return gridClearance(g, floor * g.vw * g.vd, Math.floor(x / VOXEL), Math.floor(y / VOXEL)) * VOXEL;
}

/**
 * Shortest path on one floor over voxels the drone has seen free or flown,
 * kept off voxels next to unknown space where it can be, then string-pulled.
 * Returns xy waypoints after `from`, ending at `to`, or null when the maps
 * hold no route. Reads only the sensor maps.
 */
export function sensorPath(
  g: SensorGrid,
  floor: number,
  from: readonly [number, number],
  to: readonly [number, number],
): [number, number][] | null {
  const { vw, vd } = g;
  const base = floor * vw * vd;
  const sx = clamp(Math.floor(from[0] / VOXEL), 0, vw - 1);
  const sy = clamp(Math.floor(from[1] / VOXEL), 0, vd - 1);
  let tx = clamp(Math.floor(to[0] / VOXEL), 0, vw - 1);
  let ty = clamp(Math.floor(to[1] / VOXEL), 0, vd - 1);
  if (!gridOpen(g, base, tx, ty)) {
    let best = Infinity;
    let bx = -1;
    let by = -1;
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        if (!gridOpen(g, base, tx + dx, ty + dy)) continue;
        const d = dx * dx + dy * dy;
        if (d < best) {
          best = d;
          bx = tx + dx;
          by = ty + dy;
        }
      }
    }
    if (bx < 0) return null;
    tx = bx;
    ty = by;
  }
  const n = vw * vd;
  const dist = new Float64Array(n).fill(Infinity);
  const prev = new Int32Array(n).fill(-1);
  const start = sy * vw + sx;
  const goal = ty * vw + tx;
  dist[start] = 0;
  const open: number[] = [start];
  const settled = new Uint8Array(n);
  while (open.length) {
    let bi = 0;
    for (let i = 1; i < open.length; i++) if (dist[open[i]!]! < dist[open[bi]!]!) bi = i;
    const k = open[bi]!;
    open[bi] = open[open.length - 1]!;
    open.pop();
    if (settled[k]) continue;
    settled[k] = 1;
    if (k === goal) break;
    const x = k % vw;
    const y = (k - x) / vw;
    for (let d = 0; d < 8; d++) {
      const nx = x + DX[d]!;
      const ny = y + DY[d]!;
      if (!gridOpen(g, base, nx, ny)) continue;
      if (d >= 4 && (!gridOpen(g, base, nx, y) || !gridOpen(g, base, x, ny))) continue;
      const nk = ny * vw + nx;
      if (settled[nk]) continue;
      const cl = gridClearance(g, base, nx, ny);
      const nd = dist[k]! + STEP[d]! * (1 + (cl <= 1 ? 2.5 : cl === 2 ? 0.6 : 0));
      if (nd < dist[nk]!) {
        dist[nk] = nd;
        prev[nk] = k;
        open.push(nk);
      }
    }
  }
  if (!Number.isFinite(dist[goal]!)) return null;
  const cells: [number, number][] = [];
  for (let k = goal; k >= 0; k = prev[k]!) cells.push([((k % vw) + 0.5) * VOXEL, (Math.floor(k / vw) + 0.5) * VOXEL]);
  cells.reverse();
  cells[cells.length - 1] = [to[0], to[1]];
  // String-pull: from each anchor, jump to the farthest point still in clear line of sight.
  const out: [number, number][] = [];
  let ax = from[0];
  let ay = from[1];
  let i = 0;
  while (i < cells.length) {
    let j = cells.length - 1;
    while (j > i && !(sensorLos(g, floor, ax, ay, cells[j]![0], cells[j]![1]) && pathClear(g, base, ax, ay, cells[j]![0], cells[j]![1])))
      j--;
    out.push(cells[j]!);
    ax = cells[j]![0];
    ay = cells[j]![1];
    i = j + 1;
  }
  return out;
}

/** Every interior sample of a segment has at least one voxel of known free space around it. */
function pathClear(g: SensorGrid, base: number, ax: number, ay: number, bx: number, by: number): boolean {
  const l = Math.hypot(bx - ax, by - ay);
  const n = Math.max(1, Math.ceil(l / (VOXEL * 0.4)));
  for (let i = 1; i < n; i++) {
    const t = i / n;
    if (gridClearance(g, base, Math.floor((ax + (bx - ax) * t) / VOXEL), Math.floor((ay + (by - ay) * t) / VOXEL)) < 2) return false;
  }
  return true;
}

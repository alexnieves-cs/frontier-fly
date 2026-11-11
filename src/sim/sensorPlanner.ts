import * as C from '../shared/constants';
import type { ExplorerEnv } from './env';
import type { GainSource } from './mapPredict';
import { approachSpeed, CommandShaper, cornerSpeed, profileSpeed, uTurnSpeed, resolveTracker, TRACKER_DEFAULTS, type ResolvedTracker, type TrackerOptions } from './tracker';
import { MAX_CANDIDATES, PICK_DIM, type GoalPicker, type PickCandidates } from './picker';
import { BRAKING_MARGIN, brakingFilter, descentClear, filterVelocity, SAFETY_HORIZON, SAFETY_MARGIN, type BrakingModel } from './lidarSafety';
import { voxelLayout } from '../shared/voxels';

// Module-local copies (imported bindings are getter calls under vite-node).
const LIDAR_H = C.LIDAR_H;
const LIDAR_MAX = C.LIDAR_MAX;
const VOXEL = C.VOXEL;
const MAX_SPEED_H = C.MAX_SPEED_H;
const MAX_SPEED_V = C.MAX_SPEED_V;
const MAX_YAW_RATE = C.MAX_YAW_RATE;
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
  /** Fraction of the step budget left, when the pilot knows it (a feature for SensorPlannerOptions.picker). */
  timeLeft?: number;
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
  /**
   * Learned goal choice (sim/picker.ts): replaces the rule that picks the next frontier goal among
   * one candidate per frontier cluster. Path search, filters and tracker are unchanged.
   */
  picker?: GoalPicker;
}

/** Non-negative extra cost of a frontier candidate, for coordinating with other drones. */
export interface CandidatePenalty {
  /** Called once per replan, before any cost() on that floor. */
  prepare(floor: number, vw: number): void;
  /** Cost of voxel k on the floor plane (k = vy * vw + vx), in voxel steps. */
  cost(k: number): number;
}

const DX = [1, -1, 0, 0, 1, 1, -1, -1];
const DY = [0, 0, 1, -1, 1, -1, 1, -1];
const STEP = [1, 1, 1, 1, Math.SQRT2, Math.SQRT2, Math.SQRT2, Math.SQRT2];

function clamp(v: number, lo: number, hi: number): number {
  return v < lo ? lo : v > hi ? hi : v;
}

function wrap(a: number): number {
  return a - Math.round(a / (Math.PI * 2)) * Math.PI * 2;
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
  readonly picker: GoalPicker | null;
  /** Goal choices the picker made (diagnostics). */
  picks = 0;
  private candScore = new Float64Array(0);
  private candOrder = new Int32Array(0);
  private candLabel = new Int32Array(0);
  private candQueue = new Int32Array(0);
  private readonly cands: PickCandidates = { n: 0, feats: new Float32Array(MAX_CANDIDATES * PICK_DIM), ruleScore: new Float64Array(MAX_CANDIDATES), current: -1 };
  private readonly candVox = new Int32Array(MAX_CANDIDATES);
  private readonly candSize = new Int32Array(MAX_CANDIDATES);
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
    this.picker = opts.picker ?? null;
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
    this.picks = 0;
    this.shaper.reset();
    this.picker?.reset?.();
  }

  /**
   * Tell the tracker which horizontal command the airframe was last given (world frame, m/s), when
   * it was not this planner's own: the acceleration limit then ramps from that command. Used when
   * the planner labels another pilot's states (train/ppoJob.ts, labels 'planner').
   */
  holdCommand(vx: number, vy: number): void {
    this.shaper.hold(vx, vy);
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
    this.candScore = new Float64Array(this.plane);
    this.candOrder = new Int32Array(this.plane);
    this.candLabel = new Int32Array(this.plane);
    this.candQueue = new Int32Array(this.plane);
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
    const picker = this.picker;
    const cs = this.candScore;
    if (picker) cs.fill(NaN);
    let popped = 0;
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
        if (gain && (picker || score - reach < bestCost)) score -= gw * gain.gain(k);
        // A penalty only raises the score, so the early-exit bound below still holds.
        if (claim) score += claim.cost(k);
        if (k === keep) keepCost = score;
        if (picker) {
          cs[k] = score;
          this.candOrder[k] = popped++;
        }
        if (score < bestCost) {
          bestCost = score;
          best = k;
        }
      }
      if (!picker && d0 - 8 * this.edgeBonus - reach > bestCost + 1.5 && (keep < 0 || keepCost < Infinity)) break;
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
    if (picker) {
      if (best < 0) return -1;
      const chosen = this.pickGoal(f, base, start, keep);
      return this.setPath(chosen, start);
    }
    // Hysteresis: keep the current goal unless another is clearly cheaper.
    if (keep >= 0 && keepCost < Infinity && keepCost <= bestCost + 1.5) best = keep;
    if (best < 0) return -1;
    return this.setPath(best, start);
  }

  private setPath(best: number, start: number): number {
    this.path = this.pathTo(best, start);
    return best;
  }

  private pathTo(k0: number, start: number): number[] {
    const path: number[] = [];
    for (let k = k0; k >= 0 && k !== start; k = this.prev[k]!) path.push(k);
    path.reverse();
    return path;
  }

  /**
   * One candidate per 8-connected cluster of scored frontier voxels (its lowest-score voxel, ties to
   * the earliest settled), the MAX_CANDIDATES − 1 best by score plus the current goal; features for
   * each; then the picker's choice. Returns the chosen voxel on the floor plane.
   */
  private pickGoal(f: SensorFrame, base: number, start: number, keep: number): number {
    const { vw, vd } = f;
    const cs = this.candScore;
    const ord = this.candOrder;
    const lab = this.candLabel;
    const q = this.candQueue;
    lab.fill(-1);
    // Cluster representatives, kept sorted by (score, order); at most MAX_CANDIDATES - 1.
    const reps: { k: number; s: number; o: number; size: number }[] = [];
    let nl = 0;
    for (let k0 = 0; k0 < this.plane; k0++) {
      if (!(cs[k0]! === cs[k0]!) || lab[k0]! >= 0) continue;
      let head = 0;
      let tail = 0;
      q[tail++] = k0;
      lab[k0] = nl;
      let bk = k0;
      let size = 0;
      while (head < tail) {
        const k = q[head++]!;
        size++;
        if (cs[k]! < cs[bk]! || (cs[k]! === cs[bk]! && ord[k]! < ord[bk]!)) bk = k;
        const x = k % vw;
        const y = (k - x) / vw;
        for (let d = 0; d < 8; d++) {
          const nx = x + DX[d]!;
          const ny = y + DY[d]!;
          if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
          const nk = ny * vw + nx;
          if (lab[nk]! >= 0 || !(cs[nk]! === cs[nk]!)) continue;
          lab[nk] = nl;
          q[tail++] = nk;
        }
      }
      nl++;
      reps.push({ k: bk, s: cs[bk]!, o: ord[bk]!, size });
    }
    reps.sort((a, b) => a.s - b.s || a.o - b.o);
    const c = this.cands;
    c.n = 0;
    c.current = -1;
    const sizeOf = (k: number) => reps.find((r) => lab[r.k] === lab[k])?.size ?? 1;
    for (let i = 0; i < reps.length && c.n < MAX_CANDIDATES - 1; i++) {
      this.candVox[c.n] = reps[i]!.k;
      this.candSize[c.n] = reps[i]!.size;
      c.ruleScore[c.n] = reps[i]!.s;
      if (reps[i]!.k === keep) c.current = c.n;
      c.n++;
    }
    if (keep >= 0 && c.current < 0 && cs[keep]! === cs[keep]!) {
      this.candVox[c.n] = keep;
      this.candSize[c.n] = sizeOf(keep);
      c.ruleScore[c.n] = cs[keep]!;
      c.current = c.n++;
    }
    for (let i = 0; i < c.n; i++) this.candidateFeatures(f, base, start, i);
    const pick = this.picker!.pick(c);
    this.picks++;
    return this.candVox[Math.max(0, Math.min(c.n - 1, pick))]!;
  }

  private candidateFeatures(f: SensorFrame, base: number, start: number, i: number): void {
    const c = this.cands;
    const k = this.candVox[i]!;
    const vw = f.vw;
    const o = i * PICK_DIM;
    const out = c.feats;
    const gx = ((k % vw) + 0.5) * VOXEL;
    const gy = (Math.floor(k / vw) + 0.5) * VOXEL;
    const vel = f.vel ?? [0, 0];
    const vmag = Math.hypot(vel[0], vel[1]);
    const tr = this.tracker ?? TRACKER_DEFAULTS;
    const legs = this.stringPullPath(f, base, this.pathTo(k, start), 40, 16, []);
    let meters = 0;
    let turns = 0;
    let time = 0;
    let px = f.pos[0];
    let py = f.pos[1];
    let lx = 0;
    let ly = 0;
    for (let j = 0; j < legs.length; j++) {
      const [qx, qy] = legs[j]!;
      const ax = qx - px;
      const ay = qy - py;
      const l = Math.hypot(ax, ay);
      meters += l;
      time += l / this.speed;
      if (j > 0 && l > 1e-6 && Math.hypot(lx, ly) > 1e-6) {
        const th = Math.acos(clamp((ax * lx + ay * ly) / (l * Math.hypot(lx, ly)), -1, 1));
        turns += th;
        const vc = cornerSpeed(th, tr, this.speed);
        time += ((this.speed - vc) * (this.speed - vc)) / (tr.accel * this.speed);
      }
      if (l > 1e-6) {
        lx = ax;
        ly = ay;
      }
      px = qx;
      py = qy;
    }
    let cosH = 0;
    if (legs.length) {
      const [qx, qy] = legs[0]!;
      const ax = qx - f.pos[0];
      const ay = qy - f.pos[1];
      const l = Math.hypot(ax, ay);
      if (l > 1e-6) {
        if (vmag > 1e-6) cosH = (ax * vel[0] + ay * vel[1]) / (l * vmag);
        // Turning the current velocity onto the first leg at cruise.
        time += Math.hypot((ax / l) * this.speed - vel[0], (ay / l) * this.speed - vel[1]) / (2 * tr.accel);
      }
    }
    const ll = Math.hypot(lx, ly);
    const lane = ll > 1e-6 ? this.laneFrom(f, base, gx, gy, lx / ll, ly / ll, 6) : 0;
    // Unknown space and frontier around the candidate.
    const cx = k % vw;
    const cy = (k - cx) / vw;
    let unk = 0;
    let tot = 0;
    for (let dy = -4; dy <= 4; dy++) {
      for (let dx = -4; dx <= 4; dx++) {
        if (dx * dx + dy * dy > 16) continue;
        const x = cx + dx;
        const y = cy + dy;
        if (x < 0 || y < 0 || x >= vw || y >= f.vd) continue;
        tot++;
        const kk = base + y * vw + x;
        if (f.seen[kk] !== 1 && f.visited[kk] !== 1) unk++;
      }
    }
    let fn = 0;
    for (let d = 0; d < 8; d++) {
      const x = cx + DX[d]!;
      const y = cy + DY[d]!;
      if (x < 0 || y < 0 || x >= vw || y >= f.vd) continue;
      if (this.candScore[y * vw + x]! === this.candScore[y * vw + x]!) fn++;
    }
    out[o + 0] = this.dist[k]! / 40;
    out[o + 1] = Math.hypot(gx - f.pos[0], gy - f.pos[1]) / 10;
    out[o + 2] = c.ruleScore[i]! / 40;
    out[o + 3] = this.edgeCount(f, base, k) / 8;
    out[o + 4] = Math.log1p(this.candSize[i]!) / 5;
    out[o + 5] = tot ? unk / tot : 0;
    out[o + 6] = (cosH * vmag) / 2;
    out[o + 7] = ((1 - cosH) * vmag) / 2;
    out[o + 8] = Math.min(4, turns / Math.PI);
    out[o + 9] = time / 20;
    out[o + 10] = lane / 6;
    out[o + 11] = i === c.current ? 1 : 0;
    out[o + 12] = meters / 10;
    out[o + 13] = fn / 8;
    out[o + 14] = f.timeLeft ?? 1;
    out[o + 15] = vmag / 2;
    out[o + 16] = c.n / MAX_CANDIDATES;
    out[o + 17] = 1;
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
    return this.laneFrom(f, base, f.pos[0], f.pos[1], ux, uy, max);
  }

  private laneFrom(f: SensorFrame, base: number, ox: number, oy: number, ux: number, uy: number, max: number): number {
    const stepM = VOXEL * 0.5;
    let lastK = -1;
    for (let d = stepM; d <= max; d += stepM) {
      const x = Math.floor((ox + ux * d) / VOXEL);
      const y = Math.floor((oy + uy * d) / VOXEL);
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
    this.corners.length = 0;
    this.pulledToEnd = false;
    if (!this.path.length) return this.corners;
    this.stringPullPath(f, base, this.path, lookahead, 8, this.corners);
    return this.corners;
  }

  /** String-pull `path` into at most `maxLegs` legs within `lookahead` meters; sets pulledToEnd. */
  private stringPullPath(f: SensorFrame, base: number, path: readonly number[], lookahead: number, maxLegs: number, out: [number, number][]): [number, number][] {
    out.length = 0;
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
    while (i < last && out.length < maxLegs) {
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

const lidarScratch = new Float64Array(LIDAR_H);

/** World-frame horizontal velocity from the observation's IMU estimate (body frame, normalized). */
export function measuredVelocity(obs: Float32Array, yaw: number): [number, number] {
  const f = obs[C.OBS.vel]! * C.MAX_SPEED_H;
  const l = obs[C.OBS.vel + 1]! * C.MAX_SPEED_H;
  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  return [cy * f - sy * l, sy * f + cy * l];
}

/** The sensor frame a pilot in the coverage env may use: its pose, lidar, and its own maps. */
export function sensorFrame(env: ExplorerEnv, obs: Float32Array): SensorFrame {
  const spec = env.building!;
  const s = env.snapshot();
  for (let i = 0; i < LIDAR_H; i++) lidarScratch[i] = obs[C.OBS.lidar + i]! * LIDAR_MAX;
  return {
    seen: env.seenGrid,
    visited: env.visitedGrid,
    ...voxelLayout(spec),
    floorHeight: spec.floorHeight,
    pos: s.pos,
    yaw: s.yaw,
    lidar: lidarScratch,
    alt: obs[C.OBS.alt]! * spec.floorHeight,
    vel: measuredVelocity(obs, s.yaw),
    timeLeft: obs[C.OBS.time]!,
  };
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

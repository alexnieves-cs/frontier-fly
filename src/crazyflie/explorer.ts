import * as C from '../shared/constants';
import { SensorPlanner, type SensorFrame } from '../sim/sensorPlanner';
import type { Arena } from './arena';
import { CF_BRAKING, CF_HEIGHT, CF_SPEED, CF_YAW_RATE, MULTIRANGER } from './profile';

const LIDAR_H = C.LIDAR_H;
/** Braking-filter margin past DRONE_RADIUS (0.15 m), meters. Picked on dev arenas 1-20 (docs/CRAZYFLIE.md). */
export const CF_MARGIN = 0.15;
/** Yaw sweep amplitude while cruising, radians, and its rate. Picked on dev arenas 1-20. */
export const CF_SWEEP = 0.5;
const SWEEP_HZ = 0.8;
const LIDAR_MAX = C.LIDAR_MAX;
const VOXEL = C.VOXEL;

/** Map margin around the arena, meters (two voxels): the planner's grid starts here. */
export const PAD = 1;

/** One reading of the Crazyflie's own estimate and decks, in the arena frame. */
export interface CfReading {
  /** Seconds since take-off. */
  t: number;
  /** Position estimate (flow deck + IMU), meters, arena frame. */
  x: number;
  y: number;
  /** Height above the floor (flow deck ToF), meters. */
  z: number;
  /** Heading, radians, counter-clockwise from the arena's +x. */
  yaw: number;
  /** Velocity estimate, world frame, m/s. */
  vx: number;
  vy: number;
  /**
   * Multi-ranger, meters. Infinity (or anything at or past 4 m) means no
   * return; NaN means the reading is missing and nothing is known.
   */
  front: number;
  back: number;
  left: number;
  right: number;
  up: number;
  /**
   * Bench only: a full 32-ray ring (meters, ray i at yaw + i·2π/32) that
   * replaces the four beams, to measure what the sparse rangers cost.
   */
  ring?: ArrayLike<number>;
}

/** What the explorer asks the drone to do: a world-frame velocity, a yaw rate and a height. */
export interface CfCommand {
  vx: number;
  vy: number;
  /** rad/s, counter-clockwise positive. */
  yawRate: number;
  z: number;
  /** 'spin' while taking the first look around, 'explore' while frontiers remain, 'done' when none are left. */
  phase: 'spin' | 'explore' | 'done';
}

export interface ExplorerOptions {
  speed?: number;
  height?: number;
  /** Seconds of the opening 360° look around. */
  spinSec?: number;
  /** Air the braking filter keeps past DRONE_RADIUS, meters. */
  margin?: number;
  /** Amplitude of the yaw sweep while cruising, radians (0 = off). */
  sweep?: number;
}

/** Beam angles relative to the nose and their ring bins (ray i at yaw + i·2π/LIDAR_H). */
const BEAMS = [
  { key: 'front', bin: 0 },
  { key: 'left', bin: LIDAR_H / 4 },
  { key: 'back', bin: LIDAR_H / 2 },
  { key: 'right', bin: (3 * LIDAR_H) / 4 },
] as const;

/**
 * Sensor-only frontier exploration for a Crazyflie with a Multi-ranger deck.
 *
 * frontier-fly's SensorPlanner expects a 32-ray lidar ring. The Multi-ranger
 * has four horizontal beams, so this class adapts it in two ways:
 *
 * - Mapping uses only what was measured: each step the four beams are walked
 *   into the seen-free and occupancy grids at the pose they were taken from.
 *   An opening spin sweeps them through a full circle.
 * - The braking filter gets a ring whose four beam bins are this step's
 *   readings and whose other 28 bins are ranges ray-cast through the drone's
 *   own occupancy map. To keep a measured beam on the way ahead, horizontal
 *   speed fades to zero while the nose is more than 30° off the direction of
 *   travel: the drone turns, then flies.
 *
 * The arena box is the geofence: voxels outside it are blocked in the map, so
 * no path leaves it. Inner walls and boxes are not given to the planner; it
 * has to find them with the rangers.
 */
export class CrazyflieExplorer {
  readonly vw: number;
  readonly vd: number;
  readonly seen: Uint8Array;
  readonly visited: Uint8Array;
  readonly occupied: Uint8Array;
  private readonly hits: Uint8Array;
  private readonly passes: Uint8Array;
  private readonly fixed: Uint8Array;
  private readonly ring = new Float64Array(LIDAR_H);
  private readonly planner: SensorPlanner;
  private readonly speed: number;
  private readonly height: number;
  private readonly spinSec: number;
  private spinStart = -1;
  private readonly sweep: number;
  private finished = false;

  constructor(
    readonly arena: Pick<Arena, 'w' | 'd'>,
    opts: ExplorerOptions = {},
  ) {
    this.vw = Math.ceil((arena.w + 2 * PAD) / VOXEL);
    this.vd = Math.ceil((arena.d + 2 * PAD) / VOXEL);
    const n = this.vw * this.vd;
    this.seen = new Uint8Array(n);
    this.visited = new Uint8Array(n);
    this.occupied = new Uint8Array(n);
    this.hits = new Uint8Array(n);
    this.passes = new Uint8Array(n);
    this.fixed = new Uint8Array(n);
    for (let vy = 0; vy < this.vd; vy++) {
      for (let vx = 0; vx < this.vw; vx++) {
        const cx = (vx + 0.5) * VOXEL - PAD;
        const cy = (vy + 0.5) * VOXEL - PAD;
        if (cx < 0 || cy < 0 || cx > arena.w || cy > arena.d) {
          const k = vy * this.vw + vx;
          this.fixed[k] = 1;
          this.occupied[k] = 1;
        }
      }
    }
    this.speed = opts.speed ?? CF_SPEED;
    this.sweep = opts.sweep ?? CF_SWEEP;
    this.height = opts.height ?? CF_HEIGHT;
    this.spinSec = opts.spinSec ?? 2 * Math.PI / CF_YAW_RATE;
    this.planner = new SensorPlanner({
      speed: this.speed,
      flyHeight: this.height,
      braking: CF_BRAKING,
      // The Crazyflie's prop disc is ~7 cm; DRONE_RADIUS (15 cm) plus 10 cm keeps the same 25 cm air as the sim quad.
      brakingMargin: opts.margin ?? CF_MARGIN,
      faceTravel: true,
      replanEvery: 5,
    });
  }

  /** Fraction of voxels inside the arena that the map knows are free or flown. */
  knownFree(): number {
    let n = 0;
    let free = 0;
    for (let k = 0; k < this.seen.length; k++) {
      if (this.fixed[k]) continue;
      n += 1;
      if (this.visited[k] || (this.seen[k] && !this.occupied[k])) free += 1;
    }
    return n ? free / n : 0;
  }

  private voxel(x: number, y: number): number {
    const vx = Math.floor((x + PAD) / VOXEL);
    const vy = Math.floor((y + PAD) / VOXEL);
    if (vx < 0 || vy < 0 || vx >= this.vw || vy >= this.vd) return -1;
    return vy * this.vw + vx;
  }

  private update(k: number): void {
    this.occupied[k] = this.fixed[k] || this.hits[k]! * 2 > this.passes[k]! ? 1 : 0;
  }

  /** Walk one beam into the maps. r: meters; Infinity for no return; NaN is skipped. */
  markBeam(x: number, y: number, ang: number, r: number): void {
    if (Number.isNaN(r)) return;
    const hit = r < MULTIRANGER.maxRange;
    const free = hit ? Math.max(0, r - 0.15) : MULTIRANGER.maxRange;
    const ux = Math.cos(ang);
    const uy = Math.sin(ang);
    const n = Math.max(1, Math.ceil(free / (VOXEL * 0.5)));
    let last = -1;
    for (let s = 1; s <= n && free > 0; s++) {
      const d = (s / n) * free;
      const k = this.voxel(x + ux * d, y + uy * d);
      if (k < 0) break;
      if (k === last) continue;
      last = k;
      if (this.fixed[k]) break;
      this.seen[k] = 1;
      if (this.passes[k]! < 255) this.passes[k] = this.passes[k]! + 1;
      this.update(k);
    }
    if (!hit) return;
    const k = this.voxel(x + ux * (r + 0.05), y + uy * (r + 0.05));
    if (k < 0 || this.fixed[k]) return;
    // A wall return counts double: four beams give far fewer looks than a 32-ray ring.
    this.hits[k] = Math.min(255, this.hits[k]! + 2);
    this.update(k);
  }

  /** Range from (x, y) along `ang` to the first occupied voxel of the own map, or LIDAR_MAX. */
  private mapRange(x: number, y: number, ang: number): number {
    const ux = Math.cos(ang);
    const uy = Math.sin(ang);
    for (let d = VOXEL * 0.25; d < LIDAR_MAX; d += VOXEL * 0.25) {
      const k = this.voxel(x + ux * d, y + uy * d);
      if (k < 0 || this.occupied[k]) return d;
    }
    return LIDAR_MAX;
  }

  step(r: CfReading): CfCommand {
    const vk = this.voxel(r.x, r.y);
    if (vk >= 0) this.visited[vk] = 1;
    if (r.ring) {
      for (let i = 0; i < LIDAR_H; i++) {
        const v = r.ring[i]!;
        this.markBeam(r.x, r.y, r.yaw + (i / LIDAR_H) * Math.PI * 2, v >= MULTIRANGER.maxRange ? Infinity : v);
      }
    } else {
      for (const b of BEAMS) {
        const v = r[b.key];
        this.markBeam(r.x, r.y, r.yaw + (b.bin / LIDAR_H) * Math.PI * 2, v >= MULTIRANGER.maxRange ? Infinity : v);
      }
    }
    if (this.spinStart < 0) this.spinStart = r.t;
    if (r.t - this.spinStart < this.spinSec) return { vx: 0, vy: 0, yawRate: CF_YAW_RATE, z: this.height, phase: 'spin' };
    if (this.finished) return { vx: 0, vy: 0, yawRate: 0, z: this.height, phase: 'done' };

    for (let i = 0; i < LIDAR_H; i++) this.ring[i] = this.mapRange(r.x, r.y, r.yaw + (i / LIDAR_H) * Math.PI * 2);
    if (r.ring) {
      for (let i = 0; i < LIDAR_H; i++) {
        const v = r.ring[i]!;
        this.ring[i] = Number.isNaN(v) ? NaN : v >= MULTIRANGER.maxRange ? LIDAR_MAX : v;
      }
    } else {
      for (const b of BEAMS) {
        const v = r[b.key];
        this.ring[b.bin] = Number.isNaN(v) ? NaN : v >= MULTIRANGER.maxRange ? LIDAR_MAX : v;
      }
    }
    const frame: SensorFrame = {
      seen: this.seen,
      visited: this.visited,
      occupied: this.occupied,
      vw: this.vw,
      vd: this.vd,
      floors: 1,
      floorHeight: C.FLOOR_HEIGHT,
      pos: [r.x + PAD, r.y + PAD, r.z],
      yaw: r.yaw,
      lidar: this.ring,
      alt: r.z,
      vel: [r.vx, r.vy],
    };
    const cmd = this.planner.act(frame);
    if (!cmd.exploring) {
      this.finished = true;
      return { vx: 0, vy: 0, yawRate: 0, z: this.height, phase: 'done' };
    }
    let { vx, vy } = cmd;
    const sp = Math.hypot(vx, vy);
    let yawRate = Math.max(-CF_YAW_RATE, Math.min(CF_YAW_RATE, cmd.yawRate));
    if (sp > 1e-3 && !r.ring) {
      const err = wrap(Math.atan2(vy, vx) - r.yaw);
      // Turn, then fly: full speed within 30° of the nose, none past 60°.
      const k = Math.max(0, Math.min(1, (Math.PI / 3 - Math.abs(err)) / (Math.PI / 6)));
      vx *= k;
      vy *= k;
      // Sweep the nose side to side while cruising so the front beam fans over the corners beside the path.
      const sweepErr = err + this.sweep * Math.sin(2 * Math.PI * SWEEP_HZ * r.t);
      yawRate = Math.max(-CF_YAW_RATE, Math.min(CF_YAW_RATE, 3 * sweepErr));
    }
    const cap = this.speed / Math.max(this.speed, Math.hypot(vx, vy));
    return { vx: vx * cap, vy: vy * cap, yawRate, z: this.height, phase: 'explore' };
  }
}

function wrap(a: number): number {
  return a - Math.round(a / (Math.PI * 2)) * Math.PI * 2;
}

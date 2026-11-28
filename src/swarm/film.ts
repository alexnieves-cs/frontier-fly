import type { BuildingSpec } from '../shared/types';

/** One study row: means over every held-out building where all arms and team sizes finished. */
export interface StudyRow {
  arm: 'px4' | 'quad' | 'kinematic';
  drones: number;
  buildings: number;
  coverage: number | null;
  reached80: number | null;
  t80Mean: number | null;
  contacts: number;
  collisions: number;
  crashes: number;
  failed: number;
  ekfRmsMean: number | null;
}

export interface EpisodeResult {
  coverage: number;
  t50: number | null;
  t80: number | null;
  baseKnown: number;
  collisions: number;
  crashes: number;
  droneContacts: number;
  steps: number;
  px4?: { ekfErrRmsM: number; ekfErrMaxM: number; offboardDrops: number; wallStops: number; lockstepTimeouts: number }[];
}

/** What scripts/px4/team-film.ts writes to public/swarm/<id>.json. */
export interface SwarmBundle {
  schema: 1;
  id: string;
  cond: string;
  condLabel: string;
  size: number;
  budgetS: number;
  seed: number;
  drones: number;
  /** Seconds between frames. */
  dt: number;
  autopilot: string;
  /** Frame row: t, coverage, baseKnown, then `perDrone` values for each drone in turn. */
  frames: number[][];
  /** Flat triples: frame index, voxel index, drone index, the first time a voxel was flown. */
  flown: number[];
  result: EpisodeResult;
  spec: BuildingSpec;
  base: [number, number, number];
  solo: { dt: number; coverage: number[]; result: EpisodeResult } | null;
  sameBuilding: { arm: string; drones: number; coverage: number; t50: number | null; t80: number | null; collisions: number; crashes: number; contacts: number }[];
  study: { split: string; buildings: number; rows: StudyRow[] };
}

export async function fetchSwarm(url: string): Promise<SwarmBundle> {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
  const b = (await res.json()) as SwarmBundle;
  // The packager writes typed arrays as plain arrays.
  for (const f of b.spec.floors) {
    f.cells = Uint8Array.from(f.cells as unknown as number[]);
    if (f.surfaces) f.surfaces = Uint8Array.from(f.surfaces as unknown as number[]);
  }
  return b;
}

const PER = 18;
const C = { x: 0, y: 1, z: 2, yaw: 3, roll: 4, pitch: 5, ekfX: 6, ekfY: 7, ekfZ: 8, m0: 9, launched: 13, done: 14, returning: 15, goalX: 16, goalY: 17 } as const;

export type DroneMode = 'waiting' | 'searching' | 'returning' | 'done';

export interface DroneSample {
  /** Sim frame: x, y horizontal, z up. */
  x: number;
  y: number;
  z: number;
  yaw: number;
  roll: number;
  pitch: number;
  /** Horizontal and vertical velocity from the frames, m/s. */
  vx: number;
  vy: number;
  vz: number;
  /** Where PX4's EKF2 thinks the drone is, and how far that is from the truth. */
  ekf: [number, number, number];
  ekfErr: number;
  /** Motor commands 0..1. */
  motors: [number, number, number, number];
  /** Accumulated rotor angle, rad. */
  rotor: number;
  mode: DroneMode;
  goal: [number, number] | null;
}

export interface FilmEvent {
  t: number;
  drone: number | null;
  text: string;
  /** A close pass between two drones: shown in red. */
  warn?: boolean;
  /** The two drones of a close pass. */
  pair?: [number, number];
}

/** Two drone centres closer than this would have touched (the study's contact rule, src/team/run.ts). */
export const CONTACT_M = 0.35;

function lerpAngle(a: number, b: number, w: number): number {
  let d = b - a;
  while (d > Math.PI) d -= 2 * Math.PI;
  while (d < -Math.PI) d += 2 * Math.PI;
  return a + d * w;
}

/** The film as functions of time: any moment can be sampled on its own. */
export class Track {
  readonly n: number;
  readonly dt: number;
  readonly end: number;
  /** Voxel size, m (half a floor-plan cell). */
  readonly voxel: number;
  readonly vw: number;
  readonly vd: number;
  readonly tileT: Float32Array;
  readonly tileX: Float32Array;
  readonly tileY: Float32Array;
  readonly tileZ: Float32Array;
  readonly tileDrone: Uint8Array;
  readonly events: FilmEvent[] = [];
  /** First time coverage reached 50% and 80%, s (null if never). */
  readonly t50: number | null;
  readonly t80: number | null;
  private readonly rotor: Float32Array[];
  /** Per drone, tiles flown up to each tile index (cumulative), for the share of the floor. */
  private readonly tileCum: Uint32Array[];

  constructor(readonly b: SwarmBundle) {
    this.n = b.drones;
    this.dt = b.dt;
    const F = b.frames;
    this.end = F[F.length - 1]![0]!;
    this.voxel = b.spec.cellSize / 2;
    const floor0 = b.spec.floors[0]!;
    this.vw = floor0.w * 2;
    this.vd = floor0.d * 2;

    // Rotor angle: the integral of motor command, so props spin up and slow down with thrust.
    this.rotor = Array.from({ length: this.n }, (_, d) => {
      const a = new Float32Array(F.length);
      for (let k = 1; k < F.length; k++) {
        const o = 3 + d * PER + C.m0;
        const m = (F[k]![o]! + F[k]![o + 1]! + F[k]![o + 2]! + F[k]![o + 3]!) / 4;
        a[k] = a[k - 1]! + m * this.dt * 2 * Math.PI * 9;
      }
      return a;
    });

    const fl = b.flown;
    const nt = fl.length / 3;
    this.tileT = new Float32Array(nt);
    this.tileX = new Float32Array(nt);
    this.tileY = new Float32Array(nt);
    this.tileZ = new Float32Array(nt);
    this.tileDrone = new Uint8Array(nt);
    const per = this.vw * this.vd;
    this.tileCum = Array.from({ length: this.n }, () => new Uint32Array(nt + 1));
    for (let k = 0; k < nt; k++) {
      const frame = fl[k * 3]!;
      const vox = fl[k * 3 + 1]!;
      const d = fl[k * 3 + 2]!;
      const f = Math.floor(vox / per);
      const rem = vox - f * per;
      const vy = Math.floor(rem / this.vw);
      const vx = rem - vy * this.vw;
      this.tileT[k] = F[frame]![0]!;
      this.tileX[k] = (vx + 0.5) * this.voxel;
      this.tileY[k] = (vy + 0.5) * this.voxel;
      this.tileZ[k] = f * b.spec.floorHeight;
      this.tileDrone[k] = d;
      for (let j = 0; j < this.n; j++) this.tileCum[j]![k + 1] = this.tileCum[j]![k]! + (j === d ? 1 : 0);
    }

    let t50: number | null = null;
    let t80: number | null = null;
    const prev = Array.from({ length: this.n }, () => ({ launched: 0, returning: 0, done: 0 }));
    for (const row of F) {
      const t = row[0]!;
      if (t50 === null && row[1]! >= 0.5) {
        t50 = t;
        this.events.push({ t, drone: null, text: 'Half the floor flown' });
      }
      if (t80 === null && row[1]! >= 0.8) {
        t80 = t;
        this.events.push({ t, drone: null, text: '80% of the floor flown' });
      }
      for (let d = 0; d < this.n; d++) {
        const o = 3 + d * PER;
        const p = prev[d]!;
        const launched = row[o + C.launched]!;
        const returning = row[o + C.returning]!;
        const done = row[o + C.done]!;
        if (launched && !p.launched) this.events.push({ t, drone: d, text: `Drone ${d + 1} launched` });
        if (returning && !p.returning) this.events.push({ t, drone: d, text: `Drone ${d + 1} heading back` });
        if (done && !p.done) this.events.push({ t, drone: d, text: `Drone ${d + 1} finished` });
        p.launched = launched;
        p.returning = returning;
        p.done = done;
      }
    }
    this.t50 = t50;
    this.t80 = t80;

    // Close passes, by the study's rule: two launched drones whose centres come within CONTACT_M.
    // One event per pass, at its closest point, so a near miss is never hidden by the replay.
    for (let a = 0; a < this.n; a++) {
      for (let c = a + 1; c < this.n; c++) {
        let best: { t: number; d: number } | null = null;
        for (const row of F) {
          const oa = 3 + a * PER;
          const oc = 3 + c * PER;
          const dist = row[oa + C.launched] && row[oc + C.launched] ? Math.hypot(row[oa + C.x]! - row[oc + C.x]!, row[oa + C.y]! - row[oc + C.y]!, row[oa + C.z]! - row[oc + C.z]!) : Infinity;
          if (dist < CONTACT_M) {
            if (!best || dist < best.d) best = { t: row[0]!, d: dist };
          } else if (best) {
            this.events.push({ t: best.t, drone: null, warn: true, pair: [a, c], text: `Drones ${a + 1} and ${c + 1} pass ${best.d.toFixed(2)} m apart` });
            best = null;
          }
        }
        if (best) this.events.push({ t: best.t, drone: null, warn: true, pair: [a, c], text: `Drones ${a + 1} and ${c + 1} pass ${best.d.toFixed(2)} m apart` });
      }
    }
    this.events.sort((x, y) => x.t - y.t);
  }

  /** Frame pair and blend for time t. */
  private at(t: number): [number, number, number] {
    const F = this.b.frames;
    const x = Math.max(0, Math.min(F.length - 1, t / this.dt));
    const i0 = Math.min(F.length - 1, Math.floor(x));
    const i1 = Math.min(F.length - 1, i0 + 1);
    return [i0, i1, x - i0];
  }

  private value(t: number, col: number): number {
    const [i0, i1, w] = this.at(t);
    const F = this.b.frames;
    return F[i0]![col]! * (1 - w) + F[i1]![col]! * w;
  }

  coverage(t: number): number {
    return this.value(t, 1);
  }

  baseKnown(t: number): number {
    return this.value(t, 2);
  }

  sample(t: number, d: number): DroneSample {
    const [i0, i1, w] = this.at(t);
    const F = this.b.frames;
    const a = F[i0]!;
    const b = F[i1]!;
    const o = 3 + d * PER;
    const v = (c: number) => a[o + c]! * (1 - w) + b[o + c]! * w;
    // Velocity over the surrounding frames, so it is smooth whichever frame t falls in.
    const k0 = Math.max(0, i0 - 1);
    const k1 = Math.min(F.length - 1, i0 + 2);
    const span = Math.max(1e-6, (k1 - k0) * this.dt);
    const pa = F[k0]!;
    const pb = F[k1]!;
    const x = v(C.x);
    const y = v(C.y);
    const z = v(C.z);
    const ekf: [number, number, number] = [v(C.ekfX), v(C.ekfY), v(C.ekfZ)];
    const near = w < 0.5 ? a : b;
    const launched = near[o + C.launched]! > 0;
    const done = near[o + C.done]! > 0;
    const returning = near[o + C.returning]! > 0;
    const gx = near[o + C.goalX]!;
    const gy = near[o + C.goalY]!;
    const rot = this.rotor[d]!;
    return {
      x,
      y,
      z,
      yaw: lerpAngle(a[o + C.yaw]!, b[o + C.yaw]!, w),
      roll: v(C.roll),
      pitch: v(C.pitch),
      vx: (pb[o + C.x]! - pa[o + C.x]!) / span,
      vy: (pb[o + C.y]! - pa[o + C.y]!) / span,
      vz: (pb[o + C.z]! - pa[o + C.z]!) / span,
      ekf,
      ekfErr: Math.hypot(ekf[0] - x, ekf[1] - y, ekf[2] - z),
      motors: [v(C.m0), v(C.m0 + 1), v(C.m0 + 2), v(C.m0 + 3)],
      rotor: rot[i0]! * (1 - w) + rot[i1]! * w,
      mode: done ? 'done' : returning ? 'returning' : launched ? 'searching' : 'waiting',
      goal: gx >= 0 && gy >= 0 && launched && !done ? [gx, gy] : null,
    };
  }

  /** Positions of drone d at every frame between t0 and t1, with exact end points. */
  path(d: number, t0: number, t1: number, out: number[] = []): number[] {
    out.length = 0;
    if (t1 <= t0) return out;
    const F = this.b.frames;
    const o = 3 + d * PER;
    const s0 = this.sample(Math.max(0, t0), d);
    out.push(s0.x, s0.y, s0.z);
    const k0 = Math.ceil(Math.max(0, t0) / this.dt + 1e-6);
    const k1 = Math.min(F.length - 1, Math.floor(t1 / this.dt - 1e-6));
    for (let k = k0; k <= k1; k++) out.push(F[k]![o + C.x]!, F[k]![o + C.y]!, F[k]![o + C.z]!);
    const s1 = this.sample(t1, d);
    out.push(s1.x, s1.y, s1.z);
    return out;
  }

  /** Number of tiles flown by time t (tiles are in time order). */
  tilesUpTo(t: number): number {
    let lo = 0;
    let hi = this.tileT.length;
    while (lo < hi) {
      const mid = (lo + hi) >> 1;
      if (this.tileT[mid]! <= t) lo = mid + 1;
      else hi = mid;
    }
    return lo;
  }

  /** Each drone's share of the tiles flown by time t. */
  shares(t: number): number[] {
    const k = this.tilesUpTo(t);
    return this.tileCum.map((c) => (k ? c[k]! / k : 0));
  }

  /** The launched drone positions' centroid at t, or the spawn before any launch. */
  centroid(t: number): [number, number, number] {
    let x = 0;
    let y = 0;
    let z = 0;
    let m = 0;
    for (let d = 0; d < this.n; d++) {
      const s = this.sample(t, d);
      if (s.mode === 'waiting') continue;
      x += s.x;
      y += s.y;
      z += s.z;
      m += 1;
    }
    if (!m) return [...this.b.spec.spawn];
    return [x / m, y / m, z / m];
  }
}

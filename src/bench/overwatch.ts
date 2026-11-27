/**
 * The overwatch track (FrontierBench Hard, round 5): a lost person walks a loop through several rooms
 * and the drone has to find them and keep them in its camera's view for the responders, from a
 * respectful standoff, until time runs out.
 *
 * It is a search-and-rescue task, not pursuit: the person is a civilian in need of help, who never
 * reacts to the drone; the drone never closes in. Nothing counts from closer than WATCH.standoff, and
 * every second spent inside it is reported as a standoff breach.
 *
 * The person (a 1.75 m walker, targets/catalog.ts) walks WATCH.speed along a loop through WATCH.rooms
 * rooms of the take-off floor (open floor and doorways), starting in a room far from take-off, so walls and doorways break line
 * of sight as they go. The analytic camera (forward 90°, out to WATCH.range, walls, glass and
 * furniture occlude, perception/occlusion.ts) both scores the flight and feeds the pilot a noisy
 * sighting (SensorFrame.person) whenever the person is in view. The person is not in the physics
 * world: the lidar does not see them and the drone cannot touch them.
 */
import { reachableMasks } from '../gen/validate';
import { pointOcclusion } from '../perception/occlusion';
import { CELL } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec } from '../shared/types';
import { followPath } from '../targets/motion';
import type { Aabb, Vec3 } from '../targets/types';

export const WATCH = {
  /** Closer than this, horizontal metres, nothing counts and the time is a standoff breach. */
  standoff: 2,
  /** Farthest the camera picks out a person, metres (a 1.75 m person spans about 12° at 8 m). */
  range: 8,
  /** Horizontal field of view of the camera, radians (FrontierBench Hard's forward camera). */
  fov: Math.PI / 2,
  /** Walking speed, m/s: a slow, disoriented walk. */
  speed: 0.6,
  /** Rooms on the loop. */
  rooms: 4,
  /** Standard deviation of the sighting's position, metres. */
  sigma: 0.15,
  /** Body heights sampled for line of sight, metres above the feet (hip, chest, head). */
  sample: [0.8, 1.2, 1.6],
} as const;

/** What the pilot's camera reports when the person is in view. */
export interface PersonSighting {
  /** Estimated feet position, world frame, metres (noisy). */
  pos: readonly [number, number, number];
  /** Horizontal range to that position, metres. */
  range: number;
  /** Bearing from the drone's heading, radians, positive to the left. */
  bearing: number;
}

/** One flight's overwatch columns. */
export interface WatchStats {
  /** Share of the flight with the person in camera view from at least the standoff: the track's score. */
  score: number;
  found: boolean;
  /** Seconds to the first time in view (from the standoff), or null. */
  findS: number | null;
  /** Share of the time after the first find with the person in view: track retention. */
  kept: number;
  /** Times the person left the view after being found. */
  losses: number;
  /** Median seconds from losing the person to having them in view again, over the regained losses, or null. */
  reacquireS: number | null;
  /** Seconds the drone spent closer than the standoff (horizontal, same floor). */
  breachS: number;
}

function wrap(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

function furnitureBoxes(spec: BuildingSpec, floor: number): Aabb[] {
  const out: Aabb[] = [];
  for (const part of spec.furniture?.boxes ?? []) {
    if (part.floor !== floor) continue;
    const c = Math.abs(Math.cos(part.yaw));
    const s = Math.abs(Math.sin(part.yaw));
    const ex = c * part.hx + s * part.hy;
    const ey = s * part.hx + c * part.hy;
    out.push({ min: [part.cx - ex, part.cy - ey, part.cz - part.hz], max: [part.cx + ex, part.cy + ey, part.cz + part.hz] });
  }
  return out;
}

/**
 * The person's loop: shortest walks between WATCH.rooms room anchors on the take-off floor, the first
 * room far from take-off. Cells are open floor the drone can reach, clear of furniture.
 */
export function walkerRoute(spec: BuildingSpec, seed: number, start: readonly [number, number, number]): Vec3[] {
  const f = Math.max(0, Math.min(spec.floors.length - 1, Math.floor(start[2] / spec.floorHeight)));
  const floor = spec.floors[f]!;
  const mask = reachableMasks(spec)[f]!;
  const cs = spec.cellSize;
  const { w, d } = floor;
  const boxes = furnitureBoxes(spec, f);
  const free = new Uint8Array(w * d);
  for (let iy = 0; iy < d; iy++) {
    for (let ix = 0; ix < w; ix++) {
      const i = iy * w + ix;
      if ((floor.cells[i] !== CELL.EMPTY && floor.cells[i] !== CELL.DOOR) || !mask[i]) continue;
      // A person's footprint (0.5 m) at the cell centre must miss the furniture below head height.
      const x0 = (ix + 0.25) * cs;
      const x1 = (ix + 0.75) * cs;
      const y0 = (iy + 0.25) * cs;
      const y1 = (iy + 0.75) * cs;
      if (boxes.some((b) => b.min[0] < x1 && b.max[0] > x0 && b.min[1] < y1 && b.max[1] > y0 && b.min[2] < f * spec.floorHeight + 1.75)) continue;
      free[i] = 1;
    }
  }
  const wallNext = (i: number): boolean => {
    const ix = i % w;
    const iy = Math.floor(i / w);
    for (let oy = -1; oy <= 1; oy++) for (let ox = -1; ox <= 1; ox++) {
      const x = ix + ox;
      const y = iy + oy;
      if (x < 0 || y < 0 || x >= w || y >= d || !free[y * w + x]) return true;
    }
    return false;
  };
  // Dijkstra on 4-connected free cells; cells beside a wall cost a little more, so walks keep to the middle.
  const walk = (from: number): { dist: Float64Array; prev: Int32Array } => {
    const dist = new Float64Array(w * d).fill(Infinity);
    const prev = new Int32Array(w * d).fill(-1);
    const done = new Uint8Array(w * d);
    dist[from] = 0;
    const open = [from];
    while (open.length) {
      let bi = 0;
      for (let k = 1; k < open.length; k++) if (dist[open[k]!]! < dist[open[bi]!]!) bi = k;
      const i = open[bi]!;
      open[bi] = open[open.length - 1]!;
      open.pop();
      if (done[i]) continue;
      done[i] = 1;
      const ix = i % w;
      const iy = Math.floor(i / w);
      for (const [ox, oy] of [[1, 0], [-1, 0], [0, 1], [0, -1]] as const) {
        const x = ix + ox;
        const y = iy + oy;
        if (x < 0 || y < 0 || x >= w || y >= d) continue;
        const j = y * w + x;
        if (!free[j] || done[j]) continue;
        const nd = dist[i]! + 1 + (wallNext(j) ? 0.5 : 0);
        if (nd < dist[j]!) {
          dist[j] = nd;
          prev[j] = i;
          open.push(j);
        }
      }
    }
    return { dist, prev };
  };
  const cellOf = (x: number, y: number) => Math.max(0, Math.min(d - 1, Math.floor(y / cs))) * w + Math.max(0, Math.min(w - 1, Math.floor(x / cs)));
  let home = cellOf(start[0], start[1]);
  if (!free[home]) {
    let best = Infinity;
    for (let i = 0; i < w * d; i++) {
      if (!free[i]) continue;
      const dd = Math.hypot((i % w) - (home % w), Math.floor(i / w) - Math.floor(home / w));
      if (dd < best) {
        best = dd;
        home = i;
      }
    }
  }
  const fromHome = walk(home).dist;
  // Each room's anchor: its reachable free cell nearest the room's centre.
  const anchors: number[] = [];
  for (const room of floor.rooms) {
    const cx = room.x + room.w / 2;
    const cy = room.z + room.d / 2;
    let best = -1;
    let bd = Infinity;
    for (let iy = room.z; iy < room.z + room.d; iy++) for (let ix = room.x; ix < room.x + room.w; ix++) {
      const i = iy * w + ix;
      if (!free[i] || fromHome[i] === Infinity) continue;
      const dd = Math.hypot(ix + 0.5 - cx, iy + 0.5 - cy);
      if (dd < bd) {
        bd = dd;
        best = i;
      }
    }
    if (best >= 0) anchors.push(best);
  }
  if (anchors.length < 2) throw new Error(`overwatch: floor ${f} of building ${spec.seed} has fewer than two reachable rooms`);
  const rng = new RNG(seed >>> 0, 'overwatch-route');
  // The first room: drawn from the half of the rooms farthest from take-off.
  const byFar = [...anchors].sort((a, b) => fromHome[b]! - fromHome[a]!);
  const chosen = [byFar[rng.int(Math.max(1, Math.ceil(byFar.length / 2)))]!];
  const left = anchors.filter((a) => a !== chosen[0]);
  while (chosen.length < Math.min(WATCH.rooms, anchors.length) && left.length) chosen.push(left.splice(rng.int(left.length), 1)[0]!);
  const cells: number[] = [];
  for (let k = 0; k < chosen.length; k++) {
    const a = chosen[k]!;
    const b = chosen[(k + 1) % chosen.length]!;
    const { prev } = walk(b);
    // prev points toward b, so following it from a walks a -> b.
    for (let i = a; i !== b && i >= 0; i = prev[i]!) cells.push(i);
  }
  // Keep the corners only.
  const pts: Vec3[] = [];
  const z = f * spec.floorHeight;
  for (let k = 0; k < cells.length; k++) {
    const p = cells[(k - 1 + cells.length) % cells.length]!;
    const c = cells[k]!;
    const n = cells[(k + 1) % cells.length]!;
    const turn = (c % w) - (p % w) !== (n % w) - (c % w) || Math.floor(c / w) - Math.floor(p / w) !== Math.floor(n / w) - Math.floor(c / w);
    if (turn || k === 0) pts.push([((c % w) + 0.5) * cs, (Math.floor(c / w) + 0.5) * cs, z]);
  }
  return pts.length >= 2 ? pts : [[((chosen[0]! % w) + 0.5) * cs, (Math.floor(chosen[0]! / w) + 0.5) * cs, z], [((chosen[1]! % w) + 0.5) * cs, (Math.floor(chosen[1]! / w) + 0.5) * cs, z]];
}

/** The camera's view of the person: in the field of view, in range, on the floor and not hidden. */
export function viewOfPerson(
  spec: BuildingSpec,
  boxes: Aabb[],
  pos: readonly [number, number, number],
  yaw: number,
  feet: readonly [number, number, number],
): { visible: boolean; range: number; bearing: number; sameFloor: boolean } {
  const dx = feet[0] - pos[0];
  const dy = feet[1] - pos[1];
  const range = Math.hypot(dx, dy);
  const bearing = wrap(Math.atan2(dy, dx) - yaw);
  const sameFloor = Math.floor(pos[2] / spec.floorHeight) === Math.floor((feet[2] + 0.1) / spec.floorHeight);
  if (!sameFloor || range > WATCH.range || Math.abs(bearing) > WATCH.fov / 2) return { visible: false, range, bearing, sameFloor };
  let seen = 0;
  for (const h of WATCH.sample) if (pointOcclusion(spec, [pos[0], pos[1], pos[2]], [feet[0], feet[1], feet[2] + h], boxes) < 0.5) seen += 1;
  return { visible: seen >= 2, range, bearing, sameFloor };
}

/** One flight's person and its scoring, stepped with the bench episode. */
export class Overwatch {
  readonly route: Vec3[];
  private readonly boxes: Aabb[];
  private readonly rng: RNG;
  private steps = 0;
  private watched = 0;
  private breach = 0;
  private firstStep: number | null = null;
  private inView = false;
  private gapStart: number | null = null;
  private readonly gaps: number[] = [];
  private losses = 0;
  private sighting: PersonSighting | null = null;
  feet: Vec3;

  constructor(
    private readonly spec: BuildingSpec,
    seed: number,
    start: readonly [number, number, number],
  ) {
    this.route = walkerRoute(spec, seed, start);
    const f = Math.max(0, Math.min(spec.floors.length - 1, Math.floor(start[2] / spec.floorHeight)));
    this.boxes = furnitureBoxes(spec, f);
    this.rng = new RNG(seed >>> 0, 'overwatch-sighting');
    this.feet = this.route[0]!;
  }

  /** Where the person is `t` seconds into the flight. */
  at(t: number): Vec3 {
    return followPath(this.route, WATCH.speed, t).feet;
  }

  /** Look from the drone's true pose at policy step `step` (0 before the first action). */
  observe(step: number, pos: readonly [number, number, number], yaw: number): void {
    this.feet = this.at(step / 20);
    const v = viewOfPerson(this.spec, this.boxes, pos, yaw, this.feet);
    if (v.visible) {
      const px = this.feet[0] + WATCH.sigma * this.rng.gaussian();
      const py = this.feet[1] + WATCH.sigma * this.rng.gaussian();
      const dx = px - pos[0];
      const dy = py - pos[1];
      this.sighting = { pos: [px, py, this.feet[2]], range: Math.hypot(dx, dy), bearing: wrap(Math.atan2(dy, dx) - yaw) };
    } else this.sighting = null;
    if (step === 0) return;
    this.steps = step;
    const watched = v.visible && v.range >= WATCH.standoff;
    if (v.sameFloor && v.range < WATCH.standoff) this.breach += 1;
    if (watched) {
      this.watched += 1;
      if (this.firstStep === null) this.firstStep = step;
      if (this.gapStart !== null) {
        this.gaps.push(step - this.gapStart);
        this.gapStart = null;
      }
    } else if (this.inView) {
      this.losses += 1;
      this.gapStart = step;
    }
    this.inView = watched;
  }

  /** Whether the last observed step counted: the person in view from the standoff or further. */
  get watching(): boolean {
    return this.inView;
  }

  /** Where the person stood at the last observed step. */
  get person(): Vec3 {
    return this.feet;
  }

  /** The pilot's sighting for the frame it is about to act on, or null. */
  seen(): PersonSighting | null {
    return this.sighting;
  }

  /**
   * The flight's columns. `budget` is the task's step budget: steps a crash cut off count as out
   * of view, so a pilot cannot bank a sighting and crash.
   */
  stats(budget = 0): WatchStats {
    const n = Math.max(1, this.steps, budget);
    const after = this.firstStep === null ? 0 : n - this.firstStep + 1;
    const g = [...this.gaps].sort((a, b) => a - b);
    const med = g.length ? (g.length % 2 ? g[(g.length - 1) / 2]! : (g[g.length / 2 - 1]! + g[g.length / 2]!) / 2) : null;
    return {
      score: this.watched / n,
      found: this.firstStep !== null,
      findS: this.firstStep === null ? null : this.firstStep / 20,
      kept: after > 0 ? this.watched / after : 0,
      losses: this.losses,
      reacquireS: med === null ? null : med / 20,
      breachS: this.breach / 20,
    };
  }
}

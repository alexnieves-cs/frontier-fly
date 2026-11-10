/**
 * The drone's own view of a building it is inside (IndoorSensing): lidar cast against the real collision
 * world, the seen-free and flown maps, the look estimate, and the lidar shield.
 */
import { LIDAR_H, LIDAR_MAX, OBS, OBS_DIM, VOXEL } from '../../../../shared/constants';
import { RNG } from '../../../../shared/rng';
import { type LightingState } from '../../../../shared/types';
import { makeDroneState } from '../../../../sim/drone';
import { zeroNoise } from '../../../../sim/dynamics/dr';
import { SensorNoise } from '../../../../sim/dynamics/noise';
import { FrontierNav } from '../../../../sim/frontierFeatures';
import { BRAKING_MARGIN, type BrakingModel, SAFETY_MARGIN, brakingFilter, filterVelocity } from '../../../../sim/lidarSafety';
import { EgoMaps, SeenMap } from '../../../../sim/seenMap';
import { OccupancyMap, type SensorGrid, sensorLos } from '../../../../sim/sensorMap';
import { type SensorFrame } from '../../../../sim/sensorPlanner';
import { lastRingSurfaces, sense } from '../../../../sim/sensors';
import { smokePath, smokeTransmittance } from '../../../../gen/hazards';
import { SURFACE } from '../../../../shared/constants';
import { type PhysicsWorld } from '../../../../sim/world';
import { type OutdoorPhysics } from '../../../../world/outdoor/physics';
import { type V3 } from '../../contracts';
import { clamp, wrapAngle } from '../geometry';
import { floorBase, inPlane, voxelCount, voxelIndex, voxelLayout } from '../../../../shared/voxels';
import { type IndoorGrid, LOOK_RANGE, SEE_HALF_FOV, floorOf } from './grid';

/**
 * Safety-filter horizon indoors, seconds. The mission vehicle brakes at
 * 1.5 m/s² with a jerk limit (INDOOR_LIMITS), so it needs longer to stop than
 * the coverage env's first-order lag.
 */
export const INDOOR_FILTER_HORIZON = 0.7;

/** The lidar's view of the world, in the building-local frame. */
export type LidarWorld = Pick<PhysicsWorld, 'cast'>;

/**
 * Cast the indoor lidar against the mission's real collision world (walls,
 * jambs, ajar doors, window holes), not the floor plan's idealized walls, so
 * the drone senses what it can actually hit. Glass comes from the building's
 * surface classes at the hit point.
 */
export function shellLidar(grid: IndoorGrid, cast: OutdoorPhysics['cast']): LidarWorld {
  const [ox, oy, oz] = grid.origin;
  const spec = grid.spec;
  // Exact surface lookup (just past the hit, in the cell entered) for buildings with hazards; see
  // PhysicsWorld.exactSurfaces for why the others keep the old lookup.
  const nudge = spec.hazards ? 1e-4 : 0;
  return {
    cast(x, y, z, dx, dy, dz, maxDist) {
      const hit = cast(x + ox, y + oy, z + oz, dx, dy, dz, maxDist);
      if (!hit) return null;
      const len = Math.hypot(dx, dy, dz) || 1;
      const at = hit.dist + nudge;
      const hx = x + (dx / len) * at;
      const hy = y + (dy / len) * at;
      const f = clamp(Math.floor((z + (dz / len) * at) / spec.floorHeight), 0, spec.floors.length - 1);
      const floor = spec.floors[f]!;
      const gx = clamp(Math.floor(hx / spec.cellSize), 0, floor.w - 1);
      const gy = clamp(Math.floor(hy / spec.cellSize), 0, floor.d - 1);
      return {
        dist: hit.dist,
        surface: floor.surfaces?.[gy * floor.w + gx] ?? 0,
      };
    },
  };
}

/**
 * The drone's own view of a building it is inside: the 1088-float
 * observation built by sense() on a PhysicsWorld of the building in the local
 * frame, the seen-free map its lidar paints, and the voxels it has flown.
 * One instance lives for a whole visit, so a fallback explorer inherits the
 * map the policy built. Nothing in here reads the floor plan for control:
 * spec is handed to sense() only as the world the simulated lidar hits.
 */
/** The camera recognises a shut door within this range, meters. */
export const DOOR_SEE_M = 4;
/** In smoke the look estimate trusts the camera only where at least this much contrast is left. */
export const LOOK_SMOKE_MIN_T = 0.6;

/** Height above the floor the dark look estimate checks: a small object lying there. */
const LOOK_POINT_Z = 0.25;

/** Angle between the camera axis (perception/camera.ts fpvCameraFromDrone) and a point. */
function coneAngle(local: V3, yaw: number, px: number, py: number, pz: number): number {
  const nx = Math.cos(yaw);
  const ny = Math.sin(yaw);
  const ex = local[0] + nx * 0.1;
  const ey = local[1] + ny * 0.1;
  const ez = local[2] + 0.02;
  const fx = nx * 2.9;
  const fy = ny * 2.9;
  const fz = -0.57;
  const dx = px - ex;
  const dy = py - ey;
  const dz = pz - ez;
  const c = (fx * dx + fy * dy + fz * dz) / (Math.hypot(fx, fy, fz) * Math.hypot(dx, dy, dz));
  return Math.acos(clamp(c, -1, 1));
}

export class IndoorSensing implements SensorGrid {
  readonly visited: Uint8Array;
  private readonly occ: OccupancyMap;
  /** Voxels the camera has looked at, by the drone's own estimate (see look()). */
  readonly looked: Uint8Array;
  /** Voxels where the up (down) range finder saw past the ceiling (floor): a shaft to the next floor. */
  readonly shaftUp: Uint8Array;
  readonly shaftDown: Uint8Array;
  /**
   * Per shaft voxel: summed x, y and count of the positions the range finder
   * saw through from. Their mean is inside the hole whatever the voxel grid's
   * alignment to the building, which a voxel centre is not.
   */
  private readonly shaftObsUp: Float64Array;
  private readonly shaftObsDown: Float64Array;
  /** Shaft voxels the drone tried and could not fly through; never marked again. */
  private readonly shaftRejected: Uint8Array;
  /** Camera half-angle and range the look estimate trusts; set from the lighting the drone knows. */
  lookHalf = SEE_HALF_FOV;
  lookRange = LOOK_RANGE;
  /**
   * In the dark: half-angle of the flashlight cone around the camera axis
   * (nose, tilted down as in perception/camera.ts). A voxel counts as looked
   * at only when a point just above its floor falls inside the cone, so the
   * floor right under the drone is not counted. Null in daylight.
   */
  lookCone: number | null = null;
  /** Closest distance at which a viewpoint can light a voxel on the floor (see lookCone). */
  lookMin = 0;
  readonly vw: number;
  readonly vd: number;
  visitedCount = 0;
  lastNewVoxelSec = 0;
  readonly obs = new Float32Array(OBS_DIM);
  readonly lidar = new Float64Array(LIDAR_H);
  private readonly noise: SensorNoise;
  readonly rng: RNG;
  private prevVel: [number, number, number] = [0, 0, 0];
  yawRate = 0;
  /** Stopping model of the airframe flying this building; when set, filter() is the braking filter. */
  braking: BrakingModel | null = null;
  /**
   * Hazard awareness (RunnerOptions.hazardAware). Doors: shut doors the camera recognised, by leaf
   * cell, with the cell on the far side of each. Smoke: the drone sees where the cloud is (it is
   * visible), not what is in it, and the look estimate only trusts lines of sight smoke leaves
   * LOOK_SMOKE_MIN_T of the contrast on.
   */
  hazardAware = true;
  /** Leaf cell key (floor, x, y) -> far-side cell key, for shut doors seen. */
  readonly shutDoors = new Map<number, number>();
  /**
   * Called on the lidar ranges after every reading and before anything uses
   * them, so a fault model can rewrite them in place. A NaN range is a
   * missing reading; the filters treat it as blocked.
   */
  rangeHook: ((lidar: Float64Array, tSec: number) => void) | null = null;
  private readonly velWorld: [number, number] = [0, 0];
  private steps = 0;
  readonly maxSteps = 2400;
  private readonly seenMap = new SeenMap();
  private readonly nav = new FrontierNav();
  private readonly maps = new EgoMaps();
  private stall = 0;

  constructor(
    readonly grid: IndoorGrid,
    readonly physics: LidarWorld,
    readonly lighting: LightingState,
    seed: number,
  ) {
    ({ vw: this.vw, vd: this.vd } = voxelLayout(grid.spec));
    const n = voxelCount({ vw: this.vw, vd: this.vd, floors: grid.floors });
    this.visited = new Uint8Array(n);
    this.occ = new OccupancyMap(this.vw, this.vd, grid.floors);
    // The outer wall ring: the envelope inspection measured the building from outside and the
    // fly-through crossed one cell of wall. Lidar through a window hole would paint it free.
    for (let f = 0; f < grid.floors; f++) {
      for (let y = 0; y < this.vd; y++) {
        for (let x = 0; x < this.vw; x++) {
          if (x < 2 || y < 2 || x >= this.vw - 2 || y >= this.vd - 2) this.occ.block(voxelIndex(this, f, x, y));
        }
      }
    }
    this.looked = new Uint8Array(n);
    this.shaftUp = new Uint8Array(n);
    this.shaftDown = new Uint8Array(n);
    this.shaftObsUp = new Float64Array(n * 3);
    this.shaftRejected = new Uint8Array(n);
    this.shaftObsDown = new Float64Array(n * 3);
    this.rng = new RNG(seed >>> 0, 'indoor-policy');
    this.noise = new SensorNoise(zeroNoise(), this.rng.fork('noise'));
    // The seen-free map may paint any voxel inside the outer wall ring, not just the floor plan's
    // reachable ones: the drone knows its footprint from outside, not which voxels it can fly.
    const inside = new Uint8Array(n);
    for (let i = 0; i < inside.length; i++) {
      const x = i % this.vw;
      const y = Math.floor(i / this.vw) % this.vd;
      inside[i] = x >= 2 && y >= 2 && x < this.vw - 2 && y < this.vd - 2 ? 1 : 0;
    }
    this.seenMap.reset(grid.spec, inside);
  }

  get seen(): Uint8Array {
    return this.seenMap.data;
  }

  /** Mark the voxel the drone is in, by its own pose, as flown. True when it is new. */
  private markFlown(local: V3): boolean {
    const vx = Math.floor(local[0] / VOXEL);
    const vy = Math.floor(local[1] / VOXEL);
    const f = Math.floor(local[2] / this.grid.H);
    if (!inPlane(this, vx, vy) || f < 0 || f >= this.grid.floors) return false;
    const k = voxelIndex(this, f, vx, vy);
    if (this.visited[k]) return false;
    this.visited[k] = 1;
    return true;
  }

  get occupied(): Uint8Array {
    return this.occ.occupied;
  }

  /**
   * Ranges the drone's own map predicts for the horizontal lidar ring from a local pose: the first
   * wall each beam would meet among voxels the lidar has returned from at about this height, or
   * Infinity. The fault failsafe compares them with the ring to catch beams that read open wrongly.
   */
  expectedRanges(local: V3, yaw: number, out: Float64Array): Float64Array {
    return this.occ.expected(floorOf(this.grid, local[2]), local[0], local[1], local[2], yaw, out);
  }

  /** Sense from a local pose: marks flown and seen-free voxels and writes obs and lidar (meters). */
  sense(local: V3, vel: V3, yaw: number, tSec: number): Float32Array {
    if (this.markFlown(local)) {
      this.visitedCount += 1;
      this.lastNewVoxelSec = tSec;
      this.stall = 0;
    } else {
      this.stall += 1;
    }
    const state = makeDroneState([local[0], local[1], local[2]], yaw);
    state.vel = [vel[0], vel[1], vel[2]];
    sense(
      {
        world: this.physics,
        state,
        spec: this.grid.spec,
        visited: this.visited,
        yawRate: this.yawRate,
        floor: floorOf(this.grid, local[2]),
        stepsLeft: Math.max(0, this.maxSteps - this.steps),
        maxSteps: this.maxSteps,
        lighting: this.lighting,
        noise: this.noise,
        prevVel: this.prevVel,
        seen: this.seenMap.data,
        markSeen: this.seenMap,
        nav: this.nav,
        maps: this.maps,
        stall: this.stall,
      },
      this.obs,
    );
    this.prevVel = [vel[0], vel[1], vel[2]];
    this.steps += 1;
    if (this.hazardAware && this.grid.spec.hazards?.closedDoors.length) this.noteDoors(local, yaw);
    for (let i = 0; i < LIDAR_H; i++) this.lidar[i] = this.obs[OBS.lidar + i]! * LIDAR_MAX;
    this.rangeHook?.(this.lidar, tSec);
    this.velWorld[0] = vel[0];
    this.velWorld[1] = vel[1];
    const f = floorOf(this.grid, local[2]);
    this.occ.mark(f, local[0], local[1], yaw, this.lidar, local[2]);
    this.look(local, yaw);
    // Range finders: open air well past the ceiling or the floor of this storey means a shaft.
    const zf = local[2] - f * this.grid.H;
    const vx = Math.floor(local[0] / VOXEL);
    const vy = Math.floor(local[1] / VOXEL);
    if (inPlane(this, vx, vy) && !this.shaftRejected[voxelIndex(this, f, vx, vy)]) {
      const k = voxelIndex(this, f, vx, vy);
      const note = (obs: Float64Array) => {
        obs[k * 3] += local[0];
        obs[k * 3 + 1] += local[1];
        obs[k * 3 + 2] += 1;
      };
      if (f < this.grid.floors - 1 && this.seesThrough('up', zf)) {
        this.shaftUp[k] = 1;
        note(this.shaftObsUp);
      }
      if (f > 0 && this.seesThrough('down', zf)) {
        this.shaftDown[k] = 1;
        note(this.shaftObsDown);
      }
    }
    return this.obs;
  }

  /** Record shut doors the horizontal ring returned from within DOOR_SEE_M (the camera tells a door from a wall). */
  private noteDoors(local: V3, yaw: number): void {
    const surfaces = lastRingSurfaces();
    const { w, d } = this.grid;
    const f = floorOf(this.grid, local[2]);
    for (let i = 0; i < LIDAR_H; i++) {
      if (surfaces[i] !== SURFACE.DOOR) continue;
      const r = this.obs[OBS.lidar + i]! * LIDAR_MAX;
      if (r > DOOR_SEE_M) continue;
      const ang = yaw + (i / LIDAR_H) * Math.PI * 2;
      const dx = Math.cos(ang);
      const dy = Math.sin(ang);
      const gx = Math.floor((local[0] + dx * (r + 0.05)) / this.grid.cs);
      const gy = Math.floor((local[1] + dy * (r + 0.05)) / this.grid.cs);
      if (gx < 0 || gy < 0 || gx >= w || gy >= d) continue;
      // The far side: across the leaf from the drone. The leaf's axis is the one whose near-side
      // neighbour the drone's own map knows as free; a beam meets the leaf from that side.
      const sx = Math.sign(dx) || 1;
      const sy = Math.sign(dy) || 1;
      const nearX = this.cellKnown(f, gx - sx, gy);
      const nearY = this.cellKnown(f, gx, gy - sy);
      const alongX = nearX && nearY ? Math.abs(dx) >= Math.abs(dy) : nearX || (!nearY && Math.abs(dx) >= Math.abs(dy));
      const fx = alongX ? gx + sx : gx;
      const fy = alongX ? gy : gy + sy;
      if (fx < 0 || fy < 0 || fx >= w || fy >= d) continue;
      const key = (f * d + gy) * w + gx;
      if (!this.shutDoors.has(key)) this.shutDoors.set(key, (f * d + fy) * w + fx);
    }
  }

  /** True when a voxel of grid cell (x, y) on floor f is flown or seen free in the drone's own map. */
  private cellKnown(f: number, gx: number, gy: number): boolean {
    const per = Math.round(this.grid.cs / VOXEL);
    for (let vy = gy * per; vy < (gy + 1) * per; vy++) {
      for (let vx = gx * per; vx < (gx + 1) * per; vx++) {
        if (!inPlane(this, vx, vy)) continue;
        const k = voxelIndex(this, f, vx, vy);
        if (this.visited[k] || (this.seen[k] && !this.occupied[k])) return true;
      }
    }
    return false;
  }

  /**
   * Shut doorways seen whose far side the drone never saw free or flew: space behind them nobody
   * searched. Leaf cells next to each other count as one doorway.
   */
  unsearchedDoors(): number {
    const { w, d } = this.grid;
    const open = new Set<number>();
    for (const [leaf, far] of this.shutDoors) {
      const f = Math.floor(far / (w * d));
      if (!this.cellKnown(f, far % w, Math.floor((far % (w * d)) / w))) open.add(leaf);
    }
    let n = 0;
    const done = new Set<number>();
    for (const leaf of open) {
      if (done.has(leaf)) continue;
      n += 1;
      const stack = [leaf];
      done.add(leaf);
      while (stack.length) {
        const k = stack.pop()!;
        for (const m of [k + 1, k - 1, k + w, k - w]) {
          if (open.has(m) && !done.has(m) && Math.floor(m / (w * d)) === Math.floor(k / (w * d))) {
            done.add(m);
            stack.push(m);
          }
        }
      }
    }
    return n;
  }

  /**
   * Mark known-free voxels inside the camera's trusted cone and range, with a
   * clear line of sight through the drone's own map, as looked at. This is
   * the sensor-map version of IndoorVisibility: it never reads the floor plan.
   */
  look(local: V3, yaw: number): number {
    const f = floorOf(this.grid, local[2]);
    const { vw, vd } = this;
    const base = floorBase(this, f);
    const r = this.lookRange;
    const x0 = Math.max(0, Math.floor((local[0] - r) / VOXEL));
    const x1 = Math.min(vw - 1, Math.floor((local[0] + r) / VOXEL));
    const y0 = Math.max(0, Math.floor((local[1] - r) / VOXEL));
    const y1 = Math.min(vd - 1, Math.floor((local[1] + r) / VOXEL));
    let fresh = 0;
    const smoke = this.hazardAware ? (this.grid.spec.hazards?.smoke ?? null) : null;
    for (let y = y0; y <= y1; y++) {
      for (let x = x0; x <= x1; x++) {
        const k = base + y * vw + x;
        if (this.looked[k]) continue;
        if (!(this.visited[k] || (this.seen[k] && !this.occupied[k]))) continue;
        const cx = (x + 0.5) * VOXEL;
        const cy = (y + 0.5) * VOXEL;
        const d = Math.hypot(cx - local[0], cy - local[1]);
        if (d > r) continue;
        if (this.lookCone !== null) {
          if (coneAngle(local, yaw, cx, cy, f * this.grid.H + LOOK_POINT_Z) > this.lookCone) continue;
        } else if (d > 0.7 && Math.abs(wrapAngle(Math.atan2(cy - local[1], cx - local[0]) - yaw)) > this.lookHalf) continue;
        if (!sensorLos(this, f, local[0], local[1], cx, cy)) continue;
        if (smoke && smokeTransmittance(smokePath(this.grid.spec, f, local[0], local[1], cx, cy), smoke.visibility) < LOOK_SMOKE_MIN_T) continue;
        this.looked[k] = 1;
        fresh += 1;
      }
    }
    return fresh;
  }

  /** Drop the shaft voxels within a metre of `at` on `floor`: the drone tried them and could not get through. */
  forgetShaft(floor: number, dir: 'up' | 'down', at: [number, number]): void {
    const map = dir === 'up' ? this.shaftUp : this.shaftDown;
    const base = floorBase(this, floor);
    for (let k = 0; k < this.vw * this.vd; k++) {
      if (Math.hypot(((k % this.vw) + 0.5) * VOXEL - at[0], (Math.floor(k / this.vw) + 0.5) * VOXEL - at[1]) >= 1) continue;
      map[base + k] = 0;
      this.shaftRejected[base + k] = 1;
    }
  }

  /**
   * True when the last reading of the up (down) range finder, taken `zf` above
   * the current floor, saw well past the ceiling (floor): open air to the next storey.
   */
  seesThrough(dir: 'up' | 'down', zf: number): boolean {
    return dir === 'up' ? this.obs[OBS.ceil]! * this.grid.H > this.grid.H - zf + 0.8 : this.obs[OBS.alt]! * this.grid.H > zf + 0.8;
  }

  /**
   * Centre of the shaft voxels (found by the range finders) nearest `from` on
   * `floor`, leading up or down, or null when none is known. The centre is
   * the mean of the positions the 8-connected group was seen through from, so
   * the climb stays inside the column.
   */
  shaftPoint(floor: number, dir: 'up' | 'down', from: V3): [number, number] | null {
    const map = dir === 'up' ? this.shaftUp : this.shaftDown;
    const obs = dir === 'up' ? this.shaftObsUp : this.shaftObsDown;
    const { vw, vd } = this;
    const base = floorBase(this, floor);
    let best = -1;
    let bestD = Infinity;
    for (let k = 0; k < vw * vd; k++) {
      if (!map[base + k]) continue;
      const d = Math.hypot(((k % vw) + 0.5) * VOXEL - from[0], (Math.floor(k / vw) + 0.5) * VOXEL - from[1]);
      if (d < bestD) {
        bestD = d;
        best = k;
      }
    }
    if (best < 0) return null;
    const group = [best];
    const seen = new Set(group);
    let sx = 0;
    let sy = 0;
    let n = 0;
    for (let i = 0; i < group.length; i++) {
      const k = group[i]!;
      const x = k % vw;
      const y = (k - x) / vw;
      sx += obs[(base + k) * 3]!;
      sy += obs[(base + k) * 3 + 1]!;
      n += obs[(base + k) * 3 + 2]!;
      for (let dy = -1; dy <= 1; dy++) {
        for (let dx = -1; dx <= 1; dx++) {
          const nx = x + dx;
          const ny = y + dy;
          if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
          const nk = ny * vw + nx;
          if (map[base + nk] && !seen.has(nk)) {
            seen.add(nk);
            group.push(nk);
          }
        }
      }
    }
    return [sx / n, sy / n];
  }

  /** Looked-at voxels over known-free voxels: the drone's own estimate of search coverage. */
  lookedCoverage(): number {
    let free = 0;
    let seen = 0;
    for (let i = 0; i < this.visited.length; i++) {
      if (!(this.visited[i] || (this.seen[i] && !this.occupied[i]))) continue;
      free += 1;
      if (this.looked[i]) seen += 1;
    }
    return free ? seen / free : 0;
  }

  /**
   * Per floor: the known-free voxels connected to where the drone flew, and how many of them the look
   * estimate covers. Confirm-clear reads it. Free voxels the drone can't get to through its own map
   * (slivers the lidar painted through a gap in a wall) are left out, like the space it never saw:
   * whether every reachable space was found is the explorer's job, which ends only with no frontier left.
   */
  lookStats(): { free: number; looked: number }[] {
    const { vw, vd } = this;
    const per = vw * vd;
    // A flown voxel the lidar later marked as wall (the pose estimate drifted) seeds nothing, or the fill would leak through that wall.
    const free = (k: number) => !this.occupied[k] && (this.visited[k] || this.seen[k]);
    const reach = new Uint8Array(this.visited.length);
    const queue: number[] = [];
    for (let k = 0; k < this.visited.length; k++) {
      if (this.visited[k] && !this.occupied[k]) {
        reach[k] = 1;
        queue.push(k);
      }
    }
    for (let i = 0; i < queue.length; i++) {
      const k = queue[i]!;
      const x = k % vw;
      const y = Math.floor(k / vw) % vd;
      for (const [dx, dy] of [[1, 0], [-1, 0], [0, 1], [0, -1]] as const) {
        const nx = x + dx;
        const ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
        const n = k + dy * vw + dx;
        if (reach[n] || !free(n)) continue;
        reach[n] = 1;
        queue.push(n);
      }
    }
    const out = Array.from({ length: this.grid.floors }, () => ({ free: 0, looked: 0 }));
    for (let k = 0; k < reach.length; k++) {
      if (!reach[k]) continue;
      const f = out[Math.floor(k / per)]!;
      f.free += 1;
      if (this.looked[k]) f.looked += 1;
    }
    return out;
  }

  /** True when the last observation's frontier search found nothing to fly to on this floor. */
  get frontierLeft(): boolean {
    return this.obs[OBS.hasFrontier]! > 0.5;
  }

  /** Flown voxels over the voxels this drone has mapped as free (its own estimate of how done it is). */
  mappedCoverage(): number {
    let free = 0;
    let flown = 0;
    for (let i = 0; i < this.visited.length; i++) {
      if (this.visited[i] || (this.seen[i] && !this.occupied[i])) free += 1;
      if (this.visited[i]) flown += 1;
    }
    return free ? flown / free : 0;
  }

  /** Visited voxels over reachable voxels (ground truth; for reports only, never for control). */
  coverage(): number {
    return this.grid.spec.reachableVoxels > 0 ? this.visitedCount / this.grid.spec.reachableVoxels : 0;
  }

  frame(local: V3, yaw: number): SensorFrame {
    return {
      seen: this.seen,
      visited: this.visited,
      occupied: this.occupied,
      vw: this.vw,
      vd: this.vd,
      floors: this.grid.floors,
      floorHeight: this.grid.H,
      pos: [local[0], local[1], local[2]],
      yaw,
      lidar: this.lidar,
      alt: this.obs[OBS.alt]! * this.grid.H,
      vel: this.velWorld,
    };
  }

  /**
   * The lidar velocity filter for this airframe on the latest reading: the
   * braking filter when the airframe has a stopping model (the quad), the
   * kinematic closing-speed rule otherwise.
   */
  filter(vx: number, vy: number, yaw: number, margin = SAFETY_MARGIN): [number, number] {
    return this.braking
      ? brakingFilter(vx, vy, yaw, this.lidar, this.velWorld, this.braking, Math.max(margin, BRAKING_MARGIN))
      : filterVelocity(vx, vy, yaw, this.lidar, margin, INDOOR_FILTER_HORIZON);
  }
}

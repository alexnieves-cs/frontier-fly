import {
  LIDAR_H,
  LIDAR_MAX,
  LIDAR_TILT,
  MAX_SPEED_H,
  MAX_SPEED_V,
  MAX_YAW_RATE,
  OBS,
  SURFACE,
  VOXEL,
} from '../shared/constants';
import type { BuildingSpec, DroneState, LightingState } from '../shared/types';
import type { SensorNoise } from './dynamics/noise';
import type { FrontierNav } from './frontierFeatures';
import { nightFactor } from './lighting';
import type { EgoMaps, SeenMap } from './seenMap';
import type { PhysicsWorld } from './world';
import { VOXELS_PER_CELL } from '../shared/voxels';

/** Observation v2 offsets; re-exported from shared/constants. Layout in sim/OBS_LAYOUT.md. */
export { OBS };

/** Measured horizontal ranges (meters) of the last sense() call. */
const ringMeters = new Float32Array(LIDAR_H);
/** Surface class each horizontal beam of the last sense() call returned from; -1 for no return or smoke. */
const ringSurface = new Int8Array(LIDAR_H);

/** Surface classes of the last sense() call's horizontal ring (see ringSurface). Read before the next call. */
export function lastRingSurfaces(): Int8Array {
  return ringSurface;
}

/**
 * Mean free path of a lidar pulse in smoke per meter of visibility. A 905 nm pulse sees a little
 * further than the eye, so the particles give an early return about once every two visibility ranges.
 */
export const LIDAR_SMOKE_MFP = 2;
/** Smoke march step, meters. */
const SMOKE_STEP = 0.25;

/**
 * A mirror returns the reflected path: the beam bounces off the panel's face and the range is the
 * total distance to whatever it meets next (one bounce). Faces are grid-aligned, so the face hit is
 * the one whose plane the hit point lies on.
 */
function mirrorPath(
  world: SenseContext['world'],
  cs: number,
  o: [number, number, number],
  d: [number, number, number],
  dist: number,
  maxRange: number,
): { dist: number; surface: number | null } {
  const px = o[0] + d[0] * dist;
  const py = o[1] + d[1] * dist;
  const pz = o[2] + d[2] * dist;
  const fx = Math.abs(px / cs - Math.round(px / cs));
  const fy = Math.abs(py / cs - Math.round(py / cs));
  let rx = d[0];
  let ry = d[1];
  if (fx < 1e-3 && Math.abs(d[0]) > 1e-6 && (fx <= fy || Math.abs(d[1]) < 1e-6)) rx = -rx;
  else if (fy < 1e-3 && Math.abs(d[1]) > 1e-6) ry = -ry;
  else return { dist, surface: SURFACE.MIRROR };
  const back = 1e-3;
  const left = maxRange - dist;
  if (left <= 0) return { dist: maxRange, surface: null };
  const hit = world.cast(px - d[0] * back, py - d[1] * back, pz - d[2] * back, rx, ry, d[2], left);
  return hit ? { dist: dist + hit.dist, surface: hit.surface } : { dist: maxRange, surface: null };
}

export interface SenseContext {
  /** Anything the lidar can be cast against: the building's PhysicsWorld, or a mission's real collision world. */
  world: Pick<PhysicsWorld, 'cast'>;
  state: DroneState;
  spec: BuildingSpec;
  visited: Uint8Array;
  yawRate: number;
  floor: number;
  stepsLeft: number;
  maxSteps: number;
  lighting: LightingState;
  noise: SensorNoise;
  prevVel: [number, number, number];
  /** Seen-free voxel grid (same layout as `visited`). */
  seen: Uint8Array;
  /** When set, this step's horizontal ring is marked into the map before the maps are read. */
  markSeen?: SeenMap | null;
  nav: FrontierNav;
  maps: EgoMaps;
  /** Steps since the last new visited voxel. */
  stall: number;
}

function voxelIndex(spec: BuildingSpec, x: number, y: number, z: number): number {
  const floor = spec.floors[0]!;
  const vw = floor.w * VOXELS_PER_CELL;
  const vd = floor.d * VOXELS_PER_CELL;
  const vx = Math.floor(x / VOXEL);
  const vy = Math.floor(y / VOXEL);
  const f = Math.floor(z / spec.floorHeight);
  if (vx < 0 || vy < 0 || vx >= vw || vy >= vd || f < 0 || f >= spec.floors.length) return -1;
  return f * vw * vd + vy * vw + vx;
}

export function markVisited(spec: BuildingSpec, visited: Uint8Array, x: number, y: number, z: number): boolean {
  const world = spec;
  const hit = Math.floor(z / world.floorHeight);
  if (hit < 0 || hit >= spec.floors.length) return false;
  const floor = spec.floors[hit]!;
  const gx = Math.floor(x / spec.cellSize);
  const gz = Math.floor(y / spec.cellSize);
  if (gx < 0 || gz < 0 || gx >= floor.w || gz >= floor.d) return false;
  const cell = floor.cells[gz * floor.w + gx]!;
  if (cell === 1) return false;
  const index = voxelIndex(spec, x, y, z);
  if (index < 0 || visited[index]) return false;
  visited[index] = 1;
  return true;
}

export function sense(ctx: SenseContext, out: Float32Array): Float32Array {
  out.fill(0);
  const { state, world, spec, noise, lighting } = ctx;
  const yaw = state.yaw;
  const night = nightFactor(lighting);
  const originZ = state.pos[2];
  const hazards = spec.hazards;
  const smoke = hazards?.smoke ?? null;
  const smokeFloor = smoke ? Math.max(0, Math.min(spec.floors.length - 1, Math.floor(originZ / spec.floorHeight))) : 0;
  let lastSurface = -1;
  const castNorm = (dx: number, dy: number, dz: number): number => {
    const hit = world.cast(state.pos[0], state.pos[1], originZ, dx, dy, dz, LIDAR_MAX);
    let meters = hit ? hit.dist : LIDAR_MAX;
    let surface: number | null = hit ? hit.surface : null;
    if (hit && surface === SURFACE.MIRROR) {
      const len = Math.hypot(dx, dy, dz) || 1;
      const r = mirrorPath(world, spec.cellSize, [state.pos[0], state.pos[1], originZ], [dx / len, dy / len, dz / len], hit.dist, LIDAR_MAX);
      meters = r.dist;
      surface = r.surface;
    }
    if (smoke && dz === 0) {
      // March the beam through the smoke cells: each step can return off the particles.
      const floor = spec.floors[smokeFloor]!;
      const mask = smoke.cells[smokeFloor]!;
      const p = 1 - Math.exp(-SMOKE_STEP / (LIDAR_SMOKE_MFP * smoke.visibility));
      for (let t = SMOKE_STEP / 2; t < meters; t += SMOKE_STEP) {
        const gx = Math.floor((state.pos[0] + dx * t) / spec.cellSize);
        const gy = Math.floor((state.pos[1] + dy * t) / spec.cellSize);
        if (gx < 0 || gy < 0 || gx >= floor.w || gy >= floor.d || !mask[gy * floor.w + gx]) continue;
        if (noise.uniform() < p) {
          meters = t;
          surface = null;
          break;
        }
      }
    }
    lastSurface = surface ?? -1;
    const glass = surface === SURFACE.GLASS;
    const noisy = noise.lidarMeters(meters, LIDAR_MAX, glass, night);
    const clipped = noisy < 0 ? 0 : noisy > LIDAR_MAX ? LIDAR_MAX : noisy;
    if (clipped >= LIDAR_MAX) lastSurface = -1;
    return clipped / LIDAR_MAX;
  };

  for (let i = 0; i < LIDAR_H; i++) {
    const ang = yaw + (i / LIDAR_H) * Math.PI * 2;
    const v = castNorm(Math.cos(ang), Math.sin(ang), 0);
    out[OBS.lidar + i] = v;
    ringMeters[i] = v * LIDAR_MAX;
    ringSurface[i] = lastSurface;
  }
  for (let i = 0; i < LIDAR_TILT; i++) {
    const ang = yaw + (i / LIDAR_TILT) * Math.PI * 2;
    const elev = i % 2 === 0 ? 0.45 : -0.45;
    const ch = Math.cos(elev);
    const sh = Math.sin(elev);
    out[OBS.tilt + i] = castNorm(Math.cos(ang) * ch, Math.sin(ang) * ch, sh);
  }

  const cy = Math.cos(yaw);
  const sy = Math.sin(yaw);
  const dt = 1 / 20;
  const ax = (state.vel[0] - ctx.prevVel[0]) / dt;
  const ay = (state.vel[1] - ctx.prevVel[1]) / dt;
  const az = (state.vel[2] - ctx.prevVel[2]) / dt;
  const truthAccel: [number, number, number] = [cy * ax + sy * ay, -sy * ax + cy * ay, az];
  const accel = noise.accel(truthAccel, dt);
  const gyro = noise.gyro([0, 0, ctx.yawRate], dt);
  out[OBS.vel] = (cy * state.vel[0] + sy * state.vel[1] + (accel[0] - truthAccel[0]) * dt) / MAX_SPEED_H;
  out[OBS.vel + 1] = (-sy * state.vel[0] + cy * state.vel[1] + (accel[1] - truthAccel[1]) * dt) / MAX_SPEED_H;
  out[OBS.vel + 2] = (state.vel[2] + (accel[2] - truthAccel[2]) * dt) / MAX_SPEED_V;
  out[OBS.yawRate] = gyro[2] / MAX_YAW_RATE;
  out[OBS.sinYaw] = Math.sin(yaw);
  out[OBS.cosYaw] = Math.cos(yaw);

  const down = world.cast(state.pos[0], state.pos[1], originZ, 0, 0, -1, spec.floorHeight * 2);
  const up = world.cast(state.pos[0], state.pos[1], originZ, 0, 0, 1, spec.floorHeight * 2);
  const alt = noise.baro(down ? down.dist : state.pos[2]);
  out[OBS.alt] = Math.max(0, Math.min(2, alt / spec.floorHeight));
  out[OBS.ceil] = Math.max(0, Math.min(2, (up ? up.dist : spec.floorHeight) / spec.floorHeight));

  const floor0 = spec.floors[0]!;
  const vw = floor0.w * VOXELS_PER_CELL;
  const vd = floor0.d * VOXELS_PER_CELL;
  const fCount = spec.floors.length;
  const x = state.pos[0];
  const y = state.pos[1];
  ctx.markSeen?.markRing(x, y, ctx.floor, yaw, ringMeters);
  ctx.maps.write(out, OBS, ctx.visited, ctx.seen, vw, vd, ctx.floor, fCount, x, y, yaw);
  ctx.nav.write(out, ctx.visited, ctx.seen, vw, vd, ctx.floor, fCount, x, y, yaw, ctx.stall);
  out[OBS.roll] = state.roll;
  out[OBS.pitch] = state.pitch;
  out[OBS.floor] = fCount <= 1 ? 0 : ctx.floor / (fCount - 1);
  out[OBS.time] = ctx.maxSteps <= 0 ? 0 : ctx.stepsLeft / ctx.maxSteps;
  return out;
}

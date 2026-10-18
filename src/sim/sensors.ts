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

/** Observation v2 offsets; re-exported from shared/constants. Layout in sim/OBS_LAYOUT.md. */
export { OBS };

/** Measured horizontal ranges (meters) of the last sense() call. */
const ringMeters = new Float32Array(LIDAR_H);

export interface SenseContext {
  world: PhysicsWorld;
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
  const vw = floor.w * 2;
  const vd = floor.d * 2;
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
  const castNorm = (dx: number, dy: number, dz: number): number => {
    const hit = world.cast(state.pos[0], state.pos[1], originZ, dx, dy, dz, LIDAR_MAX);
    const meters = hit ? hit.dist : LIDAR_MAX;
    const glass = hit?.surface === SURFACE.GLASS;
    const noisy = noise.lidarMeters(meters, LIDAR_MAX, glass, night);
    const clipped = noisy < 0 ? 0 : noisy > LIDAR_MAX ? LIDAR_MAX : noisy;
    return clipped / LIDAR_MAX;
  };

  for (let i = 0; i < LIDAR_H; i++) {
    const ang = yaw + (i / LIDAR_H) * Math.PI * 2;
    const v = castNorm(Math.cos(ang), Math.sin(ang), 0);
    out[OBS.lidar + i] = v;
    ringMeters[i] = v * LIDAR_MAX;
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
  const vw = floor0.w * 2;
  const vd = floor0.d * 2;
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

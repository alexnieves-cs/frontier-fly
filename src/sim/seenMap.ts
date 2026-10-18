import * as C from '../shared/constants';

// Module-local copies: under vite-node/vitest an imported binding is a getter
// call on every read, which dominated these loops.
const COARSE_VOXELS = C.COARSE_VOXELS;
const LIDAR_H = C.LIDAR_H;
const LIDAR_MAX = C.LIDAR_MAX;
const MAP_SIZE = C.MAP_SIZE;
const VOXEL = C.VOXEL;
import type { BuildingSpec } from '../shared/types';

/** Rays shorter than this many meters before the return are not trusted as free. */
const HIT_MARGIN = 0.15;
/** A reading at or above this is a miss or a dropout (glass, night): it marks nothing. */
const MISS_RANGE = LIDAR_MAX - 1e-3;

const RAY_COS = new Float64Array(LIDAR_H);
const RAY_SIN = new Float64Array(LIDAR_H);
for (let i = 0; i < LIDAR_H; i++) {
  RAY_COS[i] = Math.cos((i / LIDAR_H) * Math.PI * 2);
  RAY_SIN[i] = Math.sin((i / LIDAR_H) * Math.PI * 2);
}

/**
 * Sensor-only "seen free" voxel map. Each step the horizontal lidar rays are
 * walked (voxel DDA) from the drone to just short of their return, on the
 * drone's current floor. Same layout as the env's visited grid:
 * `f * vw * vd + vy * vw + vx`, 0.5 m voxels, vw = floor0.w * 2.
 *
 * Unlike the teacher's building cells this is knowledge a real robot has.
 * Readings with no return (misses and glass/night dropouts) mark nothing, so a
 * dropout through a glass wall can never paint the wall as free.
 */
export class SeenMap {
  data = new Uint8Array(0);
  vw = 0;
  vd = 0;
  floors = 0;
  count = 0;

  reset(spec: BuildingSpec): void {
    const floor0 = spec.floors[0]!;
    this.vw = floor0.w * 2;
    this.vd = floor0.d * 2;
    this.floors = spec.floors.length;
    const n = this.floors * this.vw * this.vd;
    if (this.data.length !== n) this.data = new Uint8Array(n);
    else this.data.fill(0);
    this.count = 0;
  }

  /**
   * Mark voxels observed free by the horizontal ring. `ranges` holds measured
   * meters for ray i at yaw + i * 2π / LIDAR_H. Returns newly marked voxels.
   */
  markRing(x: number, y: number, floor: number, yaw: number, ranges: ArrayLike<number>): number {
    if (floor < 0 || floor >= this.floors || this.vw === 0) return 0;
    const vw = this.vw;
    const vd = this.vd;
    const base = floor * vw * vd;
    const data = this.data;
    const ox = x / VOXEL;
    const oy = y / VOXEL;
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    let added = 0;
    for (let i = 0; i < LIDAR_H; i++) {
      const meters = ranges[i]!;
      if (!(meters < MISS_RANGE)) continue;
      const tEnd = (meters - HIT_MARGIN) / VOXEL;
      if (tEnd <= 0) continue;
      const dx = cy * RAY_COS[i]! - sy * RAY_SIN[i]!;
      const dy = sy * RAY_COS[i]! + cy * RAY_SIN[i]!;
      let vx = Math.floor(ox);
      let vy = Math.floor(oy);
      const stepX = dx > 0 ? 1 : -1;
      const stepY = dy > 0 ? 1 : -1;
      const tdx = dx !== 0 ? Math.abs(1 / dx) : Infinity;
      const tdy = dy !== 0 ? Math.abs(1 / dy) : Infinity;
      let tmx = dx > 0 ? (vx + 1 - ox) * tdx : dx < 0 ? (ox - vx) * tdx : Infinity;
      let tmy = dy > 0 ? (vy + 1 - oy) * tdy : dy < 0 ? (oy - vy) * tdy : Infinity;
      for (;;) {
        if (vx < 0 || vy < 0 || vx >= vw || vy >= vd) break;
        const k = base + vy * vw + vx;
        if (data[k] === 0) {
          data[k] = 1;
          added += 1;
        }
        if (tmx < tmy) {
          if (tmx > tEnd) break;
          vx += stepX;
          tmx += tdx;
        } else {
          if (tmy > tEnd) break;
          vy += stepY;
          tmy += tdy;
        }
      }
    }
    this.count += added;
    return added;
  }
}

/**
 * Egocentric maps, rotated with yaw. Body x is forward, body y is left; cell
 * (ix, iz) is written at `iz * MAP_SIZE + ix`.
 *  - local: 16x16 at 0.5 m (8 m). One sample per cell center.
 *  - coarse: 16x16 at 2 m (32 m). Each cell is the mean over the 4x4 voxel
 *    block nearest its rotated center (summed-area tables, exact when yaw is a
 *    multiple of 90° and the drone sits on a voxel corner).
 * Channels: visited (0/1, coarse = fraction) and frontier = seen-free and not
 * visited (0/1, coarse = fraction). Outside the grid reads 0.
 */
export class EgoMaps {
  private satVisited = new Int32Array(0);
  private satFrontier = new Int32Array(0);

  write(
    out: Float32Array,
    offsets: { localVisited: number; localFrontier: number; coarseVisited: number; coarseFrontier: number },
    visited: Uint8Array,
    seen: Uint8Array,
    vw: number,
    vd: number,
    floor: number,
    floors: number,
    x: number,
    y: number,
    yaw: number,
  ): void {
    const cy = Math.cos(yaw);
    const sy = Math.sin(yaw);
    if (floor < 0 || floor >= floors || vw === 0) {
      out.fill(0, offsets.localVisited, offsets.localVisited + MAP_SIZE * MAP_SIZE);
      out.fill(0, offsets.localFrontier, offsets.localFrontier + MAP_SIZE * MAP_SIZE);
      out.fill(0, offsets.coarseVisited, offsets.coarseVisited + MAP_SIZE * MAP_SIZE);
      out.fill(0, offsets.coarseFrontier, offsets.coarseFrontier + MAP_SIZE * MAP_SIZE);
      return;
    }
    const base = floor * vw * vd;
    const half = (MAP_SIZE - 1) / 2;
    for (let iz = 0; iz < MAP_SIZE; iz++) {
      for (let ix = 0; ix < MAP_SIZE; ix++) {
        const bx = (ix - half) * VOXEL;
        const by = (iz - half) * VOXEL;
        const vx = Math.floor((x + cy * bx - sy * by) / VOXEL);
        const vy = Math.floor((y + sy * bx + cy * by) / VOXEL);
        let v = 0;
        let fr = 0;
        if (vx >= 0 && vy >= 0 && vx < vw && vy < vd) {
          const k = base + vy * vw + vx;
          v = visited[k] ? 1 : 0;
          fr = v === 0 && seen[k] ? 1 : 0;
        }
        const cell = iz * MAP_SIZE + ix;
        out[offsets.localVisited + cell] = v;
        out[offsets.localFrontier + cell] = fr;
      }
    }

    // Summed-area tables over this floor, (vw + 1) x (vd + 1).
    const sw = vw + 1;
    const n = sw * (vd + 1);
    if (this.satVisited.length < n) {
      this.satVisited = new Int32Array(n);
      this.satFrontier = new Int32Array(n);
    }
    const sv = this.satVisited;
    const sf = this.satFrontier;
    for (let i = 0; i < sw; i++) {
      sv[i] = 0;
      sf[i] = 0;
    }
    for (let vy = 0; vy < vd; vy++) {
      const row = base + vy * vw;
      const r0 = vy * sw;
      const r1 = r0 + sw;
      sv[r1] = 0;
      sf[r1] = 0;
      let accV = 0;
      let accF = 0;
      for (let vx = 0; vx < vw; vx++) {
        const vis = visited[row + vx]!;
        if (vis) accV += 1;
        else if (seen[row + vx]) accF += 1;
        sv[r1 + vx + 1] = sv[r0 + vx + 1]! + accV;
        sf[r1 + vx + 1] = sf[r0 + vx + 1]! + accF;
      }
    }
    const sub = COARSE_VOXELS;
    const inv = 1 / (sub * sub);
    const coarse = sub * VOXEL;
    const hc = (MAP_SIZE - 1) / 2;
    for (let iz = 0; iz < MAP_SIZE; iz++) {
      for (let ix = 0; ix < MAP_SIZE; ix++) {
        const bx = (ix - hc) * coarse;
        const by = (iz - hc) * coarse;
        // Block of sub x sub voxels whose center is nearest the rotated cell center.
        const x0 = Math.round((x + cy * bx - sy * by) / VOXEL - sub / 2);
        const y0 = Math.round((y + sy * bx + cy * by) / VOXEL - sub / 2);
        const ax = x0 < 0 ? 0 : x0 > vw ? vw : x0;
        const ay = y0 < 0 ? 0 : y0 > vd ? vd : y0;
        const bxx = x0 + sub < 0 ? 0 : x0 + sub > vw ? vw : x0 + sub;
        const byy = y0 + sub < 0 ? 0 : y0 + sub > vd ? vd : y0 + sub;
        const cell = iz * MAP_SIZE + ix;
        if (bxx <= ax || byy <= ay) {
          out[offsets.coarseVisited + cell] = 0;
          out[offsets.coarseFrontier + cell] = 0;
          continue;
        }
        const i00 = ay * sw + ax;
        const i01 = ay * sw + bxx;
        const i10 = byy * sw + ax;
        const i11 = byy * sw + bxx;
        out[offsets.coarseVisited + cell] = (sv[i11]! - sv[i01]! - sv[i10]! + sv[i00]!) * inv;
        out[offsets.coarseFrontier + cell] = (sf[i11]! - sf[i01]! - sf[i10]! + sf[i00]!) * inv;
      }
    }
  }
}

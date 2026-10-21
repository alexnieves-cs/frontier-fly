import { distToSegment } from '../../../world/outdoor/math';
import { containsPoint } from '../../../world/outdoor/geofence';
import { sampleHeight } from '../../../world/outdoor/terrain';
import type { MissionWorld } from '../contracts';
import { CANOPY_CORE, plantingObstacles, type GroundBox } from '../world';

/**
 * The search area as a grid of ground cells, with the map layers a search
 * planner may read: terrain height, distance to the nearest trail and
 * structure, how much of each cell a person could stand on, and from which
 * directions the sky is hidden by tree crowns and buildings. Everything here
 * is map data a search team would have from aerial imagery and a site plan.
 * None of it says where the person is.
 */

/** Cell edge, meters. */
export const SAR_CELL_M = 2;
/** Azimuth sectors of the per-cell sky-view map. */
export const SKY_SECTORS = 32;
/** Blockers farther than this from a cell are ignored, meters. */
const SKY_REACH_M = 30;
/** Height of the sightline target above the ground: a person's body center. */
export const SIGHT_HEIGHT_M = 0.5;

export interface SearchGrid {
  cell: number;
  nx: number;
  ny: number;
  /** World x and y of the min corner of cell (0, 0). */
  x0: number;
  y0: number;
  /** 1 where some of the cell is ground a person could be on, inside the geofence. */
  mask: Uint8Array;
  /**
   * Fraction of the cell a person could be on: not a trunk, bush, crown
   * core, building margin or no-fly zone. Ground under a dense crown is a job
   * for ground teams, so the aerial search holds no probability there.
   */
  free: Float32Array;
  /** Terrain height at the cell center. */
  ground: Float32Array;
  /** Distance from the cell center to the nearest trail centerline, meters. */
  trailDist: Float32Array;
  /** Distance from the cell center to the nearest building wall, meters. */
  structureDist: Float32Array;
  /**
   * Sky-view map. For cell k and azimuth sector s, the blocked ranges of
   * tan(elevation) of a sightline from the cell's body-center point are
   * skyLo/skyHi[skyStart[k * SKY_SECTORS + s] .. skyStart[... + 1]).
   */
  skyStart: Int32Array;
  skyLo: Float32Array;
  skyHi: Float32Array;
  /** Indices of masked cells, row-major order. */
  cells: Int32Array;
}

export function cellIndex(grid: SearchGrid, x: number, y: number): number {
  const i = Math.floor((x - grid.x0) / grid.cell);
  const j = Math.floor((y - grid.y0) / grid.cell);
  if (i < 0 || j < 0 || i >= grid.nx || j >= grid.ny) return -1;
  return j * grid.nx + i;
}

export function cellCenter(grid: SearchGrid, index: number): [number, number] {
  const i = index % grid.nx;
  const j = Math.floor(index / grid.nx);
  return [grid.x0 + (i + 0.5) * grid.cell, grid.y0 + (j + 0.5) * grid.cell];
}

export function sectorOf(azimuth: number): number {
  const u = azimuth / (Math.PI * 2);
  return (((Math.floor(u * SKY_SECTORS + 0.5) % SKY_SECTORS) + SKY_SECTORS) % SKY_SECTORS);
}

/**
 * True when the sightline from cell k toward a camera in sector `sector`, at
 * tan(elevation) `tanE`, clears every crown and building on the map.
 */
export function skyVisible(grid: SearchGrid, k: number, sector: number, tanE: number): boolean {
  const s = k * SKY_SECTORS + sector;
  for (let i = grid.skyStart[s]!; i < grid.skyStart[s + 1]!; i++) {
    if (tanE >= grid.skyLo[i]! && tanE <= grid.skyHi[i]!) return false;
  }
  return true;
}

interface Blocker {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
  minZ: number;
  maxZ: number;
}

function inBox(x: number, y: number, box: GroundBox): boolean {
  return x >= box.minX && x < box.maxX && y >= box.minY && y < box.maxY;
}

/** Crown cores and building shells, the boxes that hide the ground from the gimbal camera in perception. */
function blockers(world: MissionWorld): Blocker[] {
  const out: Blocker[] = [];
  for (const tree of world.outdoor.trees) {
    if (tree.kind === 'undergrowth') continue;
    const r = tree.canopyRadius * CANOPY_CORE;
    const cz = tree.z + tree.canopyZ;
    out.push({ minX: tree.x - r, minY: tree.y - r, maxX: tree.x + r, maxY: tree.y + r, minZ: cz - r, maxZ: cz + r });
  }
  for (const b of world.buildings) {
    const xs = b.footprint.map((p) => p.x);
    const ys = b.footprint.map((p) => p.y);
    const z = b.shell.origin[2];
    out.push({ minX: Math.min(...xs), minY: Math.min(...ys), maxX: Math.max(...xs), maxY: Math.max(...ys), minZ: z, maxZ: z + b.heightM });
  }
  return out;
}

function boxDist(x: number, y: number, box: GroundBox): number {
  const dx = Math.max(box.minX - x, 0, x - box.maxX);
  const dy = Math.max(box.minY - y, 0, y - box.maxY);
  return Math.hypot(dx, dy);
}

/** Entry and exit distance of a 2D ray from (x, y) along (dx, dy) through a box, or null. */
function rayBox2d(x: number, y: number, dx: number, dy: number, b: Blocker): [number, number] | null {
  let t0 = 0;
  let t1 = SKY_REACH_M;
  const axes: [number, number, number, number][] = [
    [x, dx, b.minX, b.maxX],
    [y, dy, b.minY, b.maxY],
  ];
  for (const [o, d, lo, hi] of axes) {
    if (Math.abs(d) < 1e-9) {
      if (o < lo || o > hi) return null;
      continue;
    }
    let a = (lo - o) / d;
    let c = (hi - o) / d;
    if (a > c) [a, c] = [c, a];
    t0 = Math.max(t0, a);
    t1 = Math.min(t1, c);
    if (t0 > t1) return null;
  }
  return [t0, t1];
}

export function buildSearchGrid(world: MissionWorld, cell = SAR_CELL_M): SearchGrid {
  const outdoor = world.outdoor;
  const xs = outdoor.geofence.polygon.map(([x]) => x);
  const ys = outdoor.geofence.polygon.map(([, y]) => y);
  const x0 = Math.floor(Math.min(...xs) / cell) * cell;
  const y0 = Math.floor(Math.min(...ys) / cell) * cell;
  const nx = Math.ceil((Math.max(...xs) - x0) / cell);
  const ny = Math.ceil((Math.max(...ys) - y0) / cell);
  const n = nx * ny;
  const mask = new Uint8Array(n);
  const free = new Float32Array(n);
  const ground = new Float32Array(n);
  const trailDist = new Float32Array(n);
  const structureDist = new Float32Array(n);
  const blocked = plantingObstacles(outdoor);
  const walls: GroundBox[] = world.buildings.map((b) => {
    const px = b.footprint.map((p) => p.x);
    const py = b.footprint.map((p) => p.y);
    return { minX: Math.min(...px), minY: Math.min(...py), maxX: Math.max(...px), maxY: Math.max(...py) };
  });
  const boxes = blockers(world);
  const skyStart = new Int32Array(n * SKY_SECTORS + 1);
  const lo: number[] = [];
  const hi: number[] = [];
  const cells: number[] = [];
  for (let j = 0; j < ny; j++) {
    for (let i = 0; i < nx; i++) {
      const k = j * nx + i;
      const cx = x0 + (i + 0.5) * cell;
      const cy = y0 + (j + 0.5) * cell;
      ground[k] = sampleHeight(outdoor.terrain, cx, cy);
      let trail = Infinity;
      for (const c of outdoor.corridors) trail = Math.min(trail, distToSegment(cx, cy, c.ax, c.ay, c.bx, c.by));
      trailDist[k] = trail;
      let wall = Infinity;
      for (const box of walls) wall = Math.min(wall, boxDist(cx, cy, box));
      structureDist[k] = wall;
      let open = 0;
      for (let sy = 0; sy < 4; sy++) {
        for (let sx = 0; sx < 4; sx++) {
          const px = x0 + (i + (sx + 0.5) / 4) * cell;
          const py = y0 + (j + (sy + 0.5) / 4) * cell;
          if (containsPoint(outdoor.geofence, px, py) && !blocked.some((box) => inBox(px, py, box))) open += 1;
        }
      }
      free[k] = open / 16;
      if (open > 0) {
        mask[k] = 1;
        cells.push(k);
      }
      // Sky view: blocked elevation ranges per azimuth sector.
      const z0 = ground[k]! + SIGHT_HEIGHT_M;
      const near = mask[k] ? boxes.filter((b) => boxDist(cx, cy, b) < SKY_REACH_M) : [];
      for (let s = 0; s < SKY_SECTORS; s++) {
        skyStart[k * SKY_SECTORS + s] = lo.length;
        if (!near.length) continue;
        const a = (s / SKY_SECTORS) * Math.PI * 2;
        const dx = Math.cos(a);
        const dy = Math.sin(a);
        for (const b of near) {
          const hit = rayBox2d(cx, cy, dx, dy, b);
          if (!hit) continue;
          const [d0, d1] = hit;
          if (b.maxZ <= z0) continue;
          // A sightline at slope t is at height z0 + d t over the box: blocked when it meets [minZ, maxZ].
          const tLo = (b.minZ - z0) / Math.max(d1, 0.05);
          const tHi = d0 < 0.05 ? Infinity : (b.maxZ - z0) / d0;
          if (tHi < tLo) continue;
          lo.push(tLo);
          hi.push(Math.min(tHi, 1e6));
        }
      }
    }
  }
  skyStart[n * SKY_SECTORS] = lo.length;
  return {
    cell,
    nx,
    ny,
    x0,
    y0,
    mask,
    free,
    ground,
    trailDist,
    structureDist,
    skyStart,
    skyLo: Float32Array.from(lo),
    skyHi: Float32Array.from(hi),
    cells: Int32Array.from(cells),
  };
}

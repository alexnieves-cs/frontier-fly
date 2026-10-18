import { CELL, SURFACE } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec, FurnitureBox, FurniturePlan, PropKind, PropPlacement } from '../shared/types';

/**
 * Headless furniture layout. The plan is the single source of truth for the
 * meshes in src/viz/props.ts, the Rapier colliders and analytic contacts in
 * src/sim/world.ts, the coverage denominator (computeFlightMap in validate.ts)
 * and the frontier teacher.
 *
 * Collision parts are oriented boxes in each prop's local renderer frame
 * (x, y up, z depth). They are the tight bounds of the mesh parts in
 * src/viz/props.ts; keep the two in sync (tests/unit/furniture.test.ts checks).
 * Thin legs are folded into one solid block: the drone (radius 0.15 m) cannot
 * pass between them anyway.
 */
interface Part {
  /** Local center offset in the horizontal plane (renderer x, z). */
  ox: number;
  oz: number;
  /** Half extents along local x and local z. */
  hx: number;
  hz: number;
  /** Height range above the floor. */
  y0: number;
  y1: number;
}

export const PROP_PARTS: Record<PropKind, readonly Part[]> = {
  // Top 1.1 x 0.62 at 0.72-0.76, legs below.
  table: [{ ox: 0, oz: 0, hx: 0.55, hz: 0.31, y0: 0, y1: 0.76 }],
  // Seat and legs up to 0.48, backrest slab 0.47-0.93 at local z = -0.19.
  chair: [
    { ox: 0, oz: 0, hx: 0.21, hz: 0.21, y0: 0, y1: 0.48 },
    { ox: 0, oz: -0.19, hx: 0.21, hz: 0.02, y0: 0.47, y1: 0.93 },
  ],
  // Carcass 0.7 x 1.3 x 0.28 centred at 0.7.
  shelf: [{ ox: 0, oz: 0, hx: 0.35, hz: 0.14, y0: 0.05, y1: 1.35 }],
  // Seat plus arms up to 0.62, backrest 0.42-0.82 at local z = -0.22.
  sofa: [
    { ox: 0, oz: 0, hx: 0.65, hz: 0.275, y0: 0.16, y1: 0.62 },
    { ox: 0, oz: -0.22, hx: 0.65, hz: 0.05, y0: 0.42, y1: 0.82 },
  ],
  // Pot and foliage sphere (r 0.2 at 0.36).
  plant: [{ ox: 0, oz: 0, hx: 0.2, hz: 0.2, y0: 0.01, y1: 0.56 }],
};

/**
 * Keep-out margin around each prop footprint. Wider than the drone diameter
 * (0.3 m) so a prop never narrows a door, a shaft, a hallway mouth or the
 * spawn cell below what the drone can pass.
 */
export const FURNITURE_CLEARANCE = 0.35;

/** Sim-space boxes for one placement. `prop` is the placement's index. */
export function propBoxes(spec: BuildingSpec, prop: PropPlacement, index: number): FurnitureBox[] {
  const yaw = 0 - prop.rot; // not -rot: keeps +0 so plans survive JSON round trips unchanged
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  return PROP_PARTS[prop.kind].map((part) => ({
    floor: prop.floor,
    prop: index,
    cx: prop.x + part.ox * c - part.oz * s,
    cy: prop.z + part.ox * s + part.oz * c,
    cz: prop.floor * spec.floorHeight + (part.y0 + part.y1) / 2,
    hx: part.hx,
    hy: part.hz,
    hz: (part.y1 - part.y0) / 2,
    yaw,
    surface: SURFACE.WOOD,
  }));
}

/** Horizontal half extents of the box's world-axis bounding rectangle. */
export function boxExtent(box: FurnitureBox): [number, number] {
  const c = Math.abs(Math.cos(box.yaw));
  const s = Math.abs(Math.sin(box.yaw));
  return [c * box.hx + s * box.hy, s * box.hx + c * box.hy];
}

function footprintOk(spec: BuildingSpec, boxes: FurnitureBox[], roomRect: { x: number; z: number; w: number; d: number }): boolean {
  const f = boxes[0]!.floor;
  const grid = spec.floors[f]!;
  const cs = spec.cellSize;
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const box of boxes) {
    const [ex, ey] = boxExtent(box);
    minX = Math.min(minX, box.cx - ex);
    maxX = Math.max(maxX, box.cx + ex);
    minY = Math.min(minY, box.cy - ey);
    maxY = Math.max(maxY, box.cy + ey);
  }
  const inRoom = (gx: number, gz: number) =>
    gx >= roomRect.x && gx < roomRect.x + roomRect.w && gz >= roomRect.z && gz < roomRect.z + roomRect.d;
  const sx = Math.floor(spec.spawn[0] / cs);
  const sz = Math.floor(spec.spawn[1] / cs);
  const sf = Math.floor(spec.spawn[2] / spec.floorHeight);
  // The footprint itself sits on empty cells of its own room.
  for (let gz = Math.floor(minY / cs); gz <= Math.floor((maxY - 1e-6) / cs); gz++) {
    for (let gx = Math.floor(minX / cs); gx <= Math.floor((maxX - 1e-6) / cs); gx++) {
      if (gx < 0 || gz < 0 || gx >= grid.w || gz >= grid.d) return false;
      if (grid.cells[gz * grid.w + gx] !== CELL.EMPTY || !inRoom(gx, gz)) return false;
    }
  }
  // The keep-out margin may touch walls but no door, shaft, spawn or any walkable cell outside the room.
  const m = FURNITURE_CLEARANCE;
  for (let gz = Math.floor((minY - m) / cs); gz <= Math.floor((maxY + m - 1e-6) / cs); gz++) {
    for (let gx = Math.floor((minX - m) / cs); gx <= Math.floor((maxX + m - 1e-6) / cs); gx++) {
      if (gx < 0 || gz < 0 || gx >= grid.w || gz >= grid.d) continue;
      const cell = grid.cells[gz * grid.w + gx];
      if (cell === CELL.WALL) continue;
      if (cell !== CELL.EMPTY || !inRoom(gx, gz)) return false;
      if (f === sf && gx === sx && gz === sz) return false;
    }
  }
  return true;
}

/**
 * Seeded furniture layout. The candidate list and RNG draws match the original
 * planProps, so layouts stay recognisable; a candidate is kept only when its
 * whole footprint (not just its centre) is valid.
 */
export function planFurniture(spec: BuildingSpec): FurniturePlan {
  const rng = new RNG(spec.seed, 'props');
  const props: PropPlacement[] = [];
  const boxes: FurnitureBox[] = [];
  spec.floors.forEach((floor, f) => {
    for (const room of floor.rooms) {
      if (room.w < 3 || room.d < 3) continue;
      const y = f * spec.floorHeight;
      const cx = (room.x + room.w / 2) * spec.cellSize;
      const cz = (room.z + room.d / 2) * spec.cellSize;
      const candidates: PropPlacement[] = [
        { kind: 'table', x: cx, y, z: cz, rot: rng.range(-0.2, 0.2), floor: f },
        { kind: 'chair', x: cx + 0.62, y, z: cz + 0.05, rot: Math.PI / 2, floor: f },
        { kind: 'chair', x: cx - 0.15, y, z: cz + 0.62, rot: Math.PI, floor: f },
        {
          kind: 'shelf',
          x: (room.x + 0.55) * spec.cellSize,
          y,
          z: (room.z + 1.1) * spec.cellSize,
          rot: 0,
          floor: f,
        },
      ];
      if (room.w >= 4 && room.d >= 4) {
        candidates.push({
          kind: rng.bool(0.5) ? 'sofa' : 'plant',
          x: (room.x + room.w - 0.8) * spec.cellSize,
          y,
          z: (room.z + room.d - 0.8) * spec.cellSize,
          rot: rng.range(0, Math.PI),
          floor: f,
        });
      }
      for (const prop of candidates) {
        const parts = propBoxes(spec, prop, props.length);
        if (!footprintOk(spec, parts, room)) continue;
        props.push(prop);
        boxes.push(...parts);
      }
    }
  });
  return { props, boxes };
}

/** Plan without the props whose index is in `drop` (indices re-packed). */
export function withoutProps(spec: BuildingSpec, plan: FurniturePlan, drop: Set<number>): FurniturePlan {
  const props: PropPlacement[] = [];
  const boxes: FurnitureBox[] = [];
  plan.props.forEach((prop, i) => {
    if (drop.has(i)) return;
    boxes.push(...propBoxes(spec, prop, props.length));
    props.push(prop);
  });
  return { props, boxes };
}

/** Exact sphere vs oriented-box overlap (strict, like the grid contact test). */
export function sphereHitsBox(box: FurnitureBox, x: number, y: number, z: number, r: number): boolean {
  const dz = Math.abs(z - box.cz) - box.hz;
  if (dz >= r) return false;
  const c = Math.cos(box.yaw);
  const s = Math.sin(box.yaw);
  const dx = x - box.cx;
  const dy = y - box.cy;
  const lx = Math.abs(dx * c + dy * s) - box.hx;
  const ly = Math.abs(-dx * s + dy * c) - box.hy;
  const ex = lx > 0 ? lx : 0;
  const ey = ly > 0 ? ly : 0;
  const ez = dz > 0 ? dz : 0;
  return ex * ex + ey * ey + ez * ez < r * r;
}

/**
 * Cell-bucketed furniture lookup. Each box is registered in every grid cell its
 * world-axis rectangle touches, so a query only tests boxes near the point.
 * Immutable after construction.
 */
export class FurnitureIndex {
  readonly boxes: readonly FurnitureBox[];
  private readonly cs: number;
  private readonly H: number;
  private readonly w: number;
  private readonly d: number;
  private readonly nFloors: number;
  /** CSR buckets over floors * w * d cells. */
  private readonly start: Int32Array;
  private readonly items: Int32Array;
  /** Per box: cx, cy, cz, hx, hy, hz, cos, sin. */
  private readonly data: Float64Array;

  constructor(spec: BuildingSpec, boxes: readonly FurnitureBox[] = spec.furniture?.boxes ?? []) {
    this.boxes = boxes;
    this.cs = spec.cellSize;
    this.H = spec.floorHeight;
    const floor0 = spec.floors[0]!;
    this.w = floor0.w;
    this.d = floor0.d;
    this.nFloors = spec.floors.length;
    const nCells = this.nFloors * this.w * this.d;
    const counts = new Int32Array(nCells + 1);
    const ranges: number[] = [];
    this.data = new Float64Array(boxes.length * 8);
    boxes.forEach((box, i) => {
      const [ex, ey] = boxExtent(box);
      const x0 = Math.max(0, Math.floor((box.cx - ex) / this.cs));
      const x1 = Math.min(this.w - 1, Math.floor((box.cx + ex) / this.cs));
      const y0 = Math.max(0, Math.floor((box.cy - ey) / this.cs));
      const y1 = Math.min(this.d - 1, Math.floor((box.cy + ey) / this.cs));
      ranges.push(x0, x1, y0, y1);
      for (let gy = y0; gy <= y1; gy++) {
        for (let gx = x0; gx <= x1; gx++) counts[this.cellIndex(box.floor, gx, gy) + 1]! += 1;
      }
      this.data.set([box.cx, box.cy, box.cz, box.hx, box.hy, box.hz, Math.cos(box.yaw), Math.sin(box.yaw)], i * 8);
    });
    for (let i = 0; i < nCells; i++) counts[i + 1]! += counts[i]!;
    this.start = counts;
    this.items = new Int32Array(counts[nCells]!);
    const fill = counts.slice(0, nCells);
    boxes.forEach((box, i) => {
      const [x0, x1, y0, y1] = ranges.slice(i * 4, i * 4 + 4) as [number, number, number, number];
      for (let gy = y0; gy <= y1; gy++) {
        for (let gx = x0; gx <= x1; gx++) {
          const cell = this.cellIndex(box.floor, gx, gy);
          this.items[fill[cell]!++] = i;
        }
      }
    });
  }

  get size(): number {
    return this.boxes.length;
  }

  private cellIndex(f: number, gx: number, gy: number): number {
    return (f * this.d + gy) * this.w + gx;
  }

  private hitsBox(i: number, x: number, y: number, z: number, r: number, margin: number): boolean {
    const o = i * 8;
    const data = this.data;
    const dz = Math.abs(z - data[o + 2]!) - data[o + 5]!;
    if (dz >= r) return false;
    if (margin > 0) {
      // Clearance query: the sphere reaches the box's height band; test the footprint inflated by r + margin.
      const c = data[o + 6]!;
      const s = data[o + 7]!;
      const dx = x - data[o]!;
      const dy = y - data[o + 1]!;
      const lx = Math.max(0, Math.abs(dx * c + dy * s) - data[o + 3]!);
      const ly = Math.max(0, Math.abs(-dx * s + dy * c) - data[o + 4]!);
      const rr = r + margin;
      return lx * lx + ly * ly < rr * rr;
    }
    const c = data[o + 6]!;
    const s = data[o + 7]!;
    const dx = x - data[o]!;
    const dy = y - data[o + 1]!;
    const lx = Math.abs(dx * c + dy * s) - data[o + 3]!;
    const ly = Math.abs(-dx * s + dy * c) - data[o + 4]!;
    const ex = lx > 0 ? lx : 0;
    const ey = ly > 0 ? ly : 0;
    const ez = dz > 0 ? dz : 0;
    return ex * ex + ey * ey + ez * ez < r * r;
  }

  /**
   * True when a sphere at (x, y, z) overlaps any furniture box. With `margin`,
   * true when a box reaching the sphere's height band comes within r + margin
   * horizontally (a clearance query for planners; physics uses margin 0).
   */
  hits(x: number, y: number, z: number, r: number, margin = 0): boolean {
    if (this.boxes.length === 0) return false;
    const f = Math.floor(z / this.H);
    if (f < 0 || f >= this.nFloors) return false;
    const reach = r + margin;
    const x0 = Math.max(0, Math.floor((x - reach) / this.cs));
    const x1 = Math.min(this.w - 1, Math.floor((x + reach) / this.cs));
    const y0 = Math.max(0, Math.floor((y - reach) / this.cs));
    const y1 = Math.min(this.d - 1, Math.floor((y + reach) / this.cs));
    for (let gy = y0; gy <= y1; gy++) {
      for (let gx = x0; gx <= x1; gx++) {
        const cell = this.cellIndex(f, gx, gy);
        for (let k = this.start[cell]!; k < this.start[cell + 1]!; k++) {
          if (this.hitsBox(this.items[k]!, x, y, z, r, margin)) return true;
        }
      }
    }
    return false;
  }

  /**
   * Horizontal push away from boxes that reach the band z +- r and lie within
   * `reach` of (x, y). Returns [px, py] summed over boxes; zero when clear.
   */
  repulsion(x: number, y: number, z: number, r: number, reach: number, gain: number): [number, number] {
    let px = 0;
    let py = 0;
    if (this.boxes.length === 0) return [px, py];
    const f = Math.floor(z / this.H);
    if (f < 0 || f >= this.nFloors) return [px, py];
    const x0 = Math.max(0, Math.floor((x - reach) / this.cs));
    const x1 = Math.min(this.w - 1, Math.floor((x + reach) / this.cs));
    const y0 = Math.max(0, Math.floor((y - reach) / this.cs));
    const y1 = Math.min(this.d - 1, Math.floor((y + reach) / this.cs));
    const seen = new Set<number>();
    for (let gy = y0; gy <= y1; gy++) {
      for (let gx = x0; gx <= x1; gx++) {
        const cell = this.cellIndex(f, gx, gy);
        for (let k = this.start[cell]!; k < this.start[cell + 1]!; k++) {
          const i = this.items[k]!;
          if (seen.has(i)) continue;
          seen.add(i);
          const o = i * 8;
          const data = this.data;
          if (Math.abs(z - data[o + 2]!) - data[o + 5]! >= r) continue;
          const c = data[o + 6]!;
          const s = data[o + 7]!;
          const dx = x - data[o]!;
          const dy = y - data[o + 1]!;
          const lx = dx * c + dy * s;
          const ly = -dx * s + dy * c;
          const hx = data[o + 3]!;
          const hy = data[o + 4]!;
          // Offset from the nearest point on the footprint, in the local frame.
          let ox = lx - Math.max(-hx, Math.min(hx, lx));
          let oy = ly - Math.max(-hy, Math.min(hy, ly));
          let dist = Math.hypot(ox, oy);
          if (dist < 1e-6) {
            // Inside the footprint: push out through the nearest face.
            const fx = hx - Math.abs(lx);
            const fy = hy - Math.abs(ly);
            if (fx < fy) {
              ox = Math.sign(lx) || 1;
              oy = 0;
            } else {
              ox = 0;
              oy = Math.sign(ly) || 1;
            }
            dist = 0;
          } else {
            ox /= dist;
            oy /= dist;
          }
          if (dist >= reach) continue;
          const mag = (reach - dist) * gain;
          px += (ox * c - oy * s) * mag;
          py += (ox * s + oy * c) * mag;
        }
      }
    }
    return [px, py];
  }
}

const indexCache = new WeakMap<BuildingSpec, FurnitureIndex>();

/** Cached index of spec.furniture. Specs are treated as immutable once generated. */
export function furnitureIndex(spec: BuildingSpec): FurnitureIndex {
  let index = indexCache.get(spec);
  if (!index || index.boxes !== (spec.furniture?.boxes ?? index.boxes)) {
    index = new FurnitureIndex(spec);
    indexCache.set(spec, index);
  }
  return index;
}

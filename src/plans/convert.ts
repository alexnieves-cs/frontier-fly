import { furnish } from '../gen/building';
import { computeFlightMap, isFullyReachable } from '../gen/validate';
import { CELL, CELL_SIZE, FLOOR_HEIGHT, SURFACE } from '../shared/constants';
import type { BuildingSpec, FloorGrid, Room } from '../shared/types';
import { inBox, inRing, ringArea, ringBox, type Box } from './geometry';
import type { PlanRecord } from './types';

/**
 * Real plan to sim building.
 *
 * The sim's walls are whole 1 m cells, and real interior walls are 10-30 cm
 * thick, so a plain 1 m raster either closes 1.2 m corridors and doors or loses
 * thin walls. The converter instead cuts each axis into cells of any length
 * (dynamic programming over a 5 cm lattice, cost Σ (length - 1)²) under three
 * rules: every wall band (the thin extent of an axis-aligned wall) lies inside
 * one cell, two wall bands never share a cell and always have a band-free cell
 * between them, and a cell centre inside a door's width earns a credit. Every
 * cut cell becomes one 1 m sim cell. Where walls are far apart the cut is close
 * to metric; where they crowd (a bathroom between two closets) cells shrink, so
 * the sim stretches that part of the plan rather than dropping a wall or a room.
 * Cells are classified by sample points mapped back into the plan: the cell
 * centre and, where the cell holds a wall band, the band's centre line.
 */

/** A wall polygon whose box is at most this thin, and at least this long, is a wall band for the cut. */
const WALL_MAX_THICKNESS = 0.6;
const WALL_MIN_LENGTH = 0.8;
/** Facing walls closer than this share a cell: the gap is a duct or a lining, not a room. */
const BAND_MERGE_GAP = 0.45;
/** Two walls face each other when their long extents overlap by more than this (m). */
const OVERLAP_MIN = 0.2;
/** Door and window polygons are tested with this much slack (m). */
const OPENING_PAD = 0.05;
/** Rooms whose largest inscribed rectangle is smaller than this (cells) are not listed as rooms. */
const MIN_ROOM_CELLS = 2;
/** Cut lattice and the longest cell (m). */
const STEP = 0.05;
const CELL_MAX = 2;
/** Length a cell holding walls aims for: the walls' own extent plus this much room on each side. */
const WALL_CELL_SLACK = 0.1;
/** Cost credit for a cell centre inside a door's width (cells of at least DOOR_CELL_MIN, so one per door). */
const DOOR_BONUS = 1.5;
const DOOR_CELL_MIN = 0.8;
/** Cost of a wall cell whose span holds a door's middle. */
const DOOR_BLOCK = 10;

/** A wall's thin extent [a, b] on the cut axis and its long extent [c0, c1] on the other. */
export interface Band {
  a: number;
  b: number;
  c0: number;
  c1: number;
}

interface Group {
  a: number;
  b: number;
  members: Band[];
}

interface Axis {
  /** Plan coordinate of each cell's centre (without the 1-cell wall border). */
  centers: number[];
  /** Centre lines of the walls inside each cell (empty when the cell holds no band). */
  walls: number[][];
  /** Plan length the axis spans. */
  span: number;
}

const overlaps = (p: Band, q: Band) => Math.min(p.c1, q.c1) - Math.max(p.c0, q.c0) > OVERLAP_MIN;

/**
 * Group bands that may share a cell: consecutive bands closer than
 * BAND_MERGE_GAP, as long as every pair inside a group that faces each other
 * (overlapping long extents) is that close too, so no room between facing
 * walls is swallowed.
 */
function groupBands(bands: Band[]): Group[] {
  const sorted = bands.filter((band) => band.b > band.a).sort((p, q) => p.a - q.a);
  const groups: Group[] = [];
  for (const band of sorted) {
    const last = groups[groups.length - 1];
    // Touching bands always join: the lattice could not put a cell between them.
    const fits =
      last &&
      (band.a < last.b + 2 * STEP ||
        (band.a < last.b + BAND_MERGE_GAP && last.members.every((m) => band.a - m.b < BAND_MERGE_GAP || !overlaps(m, band))));
    if (fits) {
      last.b = Math.max(last.b, band.b);
      last.members.push(band);
    } else {
      groups.push({ a: band.a, b: band.b, members: [band] });
    }
  }
  return groups;
}

/** Facing walls in two groups: the space between them must keep a cell. */
function conflict(p: Group, q: Group): boolean {
  return p.members.some((m) => q.members.some((n) => overlaps(m, n)));
}

/**
 * Cut [lo, hi] into cells. Cuts sit on a 5 cm lattice and never inside a wall
 * group. A cell may hold several consecutive groups, and two band cells may
 * touch, unless walls in neighbouring groups face each other; then a band-free
 * cell must separate them. Minimizes Σ (length - 1)² minus a credit per door
 * whose width holds a band-free cell's centre.
 */
export function cutAxis(lo: number, hi: number, wallBands: Band[], doorBands: { a: number; b: number }[]): Axis {
  const groups = groupBands(wallBands);
  const clash = groups.map((g, k) => k + 1 < groups.length && conflict(g, groups[k + 1]!));
  const origin = lo - 0.5;
  const n = Math.ceil((hi + 0.5 - origin) / STEP) + 1;
  const pos = (i: number) => origin + i * STEP;
  // inside[i]: point i lies inside a group. ended[i]: groups ending at or before i; started[i]: groups starting before i.
  const inside = new Uint8Array(n);
  const ended = new Int32Array(n);
  const started = new Int32Array(n);
  for (let i = 0; i < n; i++) {
    const p = pos(i);
    for (const g of groups) {
      if (p > g.a + 1e-9 && p < g.b - 1e-9) inside[i] = 1;
      if (g.b <= p + 1e-9) ended[i] += 1;
      if (g.a < p - 1e-9) started[i] += 1;
    }
  }
  const maxSpan = Math.ceil((Math.max(CELL_MAX, ...groups.map((g) => g.b - g.a + 0.4)) + 1e-9) / STEP);
  // State (i, kind): a cut at point i whose last cell held a band (kind 1) or not (kind 0).
  const cost = [new Float64Array(n).fill(Infinity), new Float64Array(n).fill(Infinity)];
  const prev = [new Int32Array(n).fill(-1), new Int32Array(n).fill(-1)];
  const prevKind = [new Uint8Array(n), new Uint8Array(n)];
  cost[0]![0] = 0;
  const end = Math.ceil((hi - origin) / STEP);
  for (let i = 0; i < n; i++) {
    if (inside[i]) continue;
    for (let kind = 0; kind < 2; kind++) {
      const base = cost[kind]![i]!;
      if (!Number.isFinite(base)) continue;
      const a = pos(i);
      const first = ended[i]!;
      for (let j = i + 1; j < n && j - i <= maxSpan; j++) {
        const count = started[j]! - first;
        // Groups first..first+count-1 lie in [a, b]; facing walls inside one cell end the scan.
        if (count > 1 && clash[first + count - 2]) break;
        if (inside[j]) continue;
        if (count >= 1 && kind === 1 && clash[first - 1]) continue;
        const b = pos(j);
        const len = b - a;
        const target = count > 0 ? Math.min(1, groups[first + count - 1]!.b - groups[first]!.a + 2 * WALL_CELL_SLACK) : 1;
        let c = base + (len - target) * (len - target);
        if (count === 0 && len >= DOOR_CELL_MIN - 1e-9) {
          const mid = (a + b) / 2;
          for (const d of doorBands) if (mid > d.a + 0.05 && mid < d.b - 0.05) c -= DOOR_BONUS;
        } else if (count > 0) {
          // A wall cell across a door's width would wall the door in.
          for (const d of doorBands) if ((d.a + d.b) / 2 > a && (d.a + d.b) / 2 < b) c += DOOR_BLOCK;
        }
        const k = count > 0 ? 1 : 0;
        if (c < cost[k]![j]!) {
          cost[k]![j] = c;
          prev[k]![j] = i;
          prevKind[k]![j] = kind;
        }
      }
    }
  }
  let last = -1;
  let lastKind = 0;
  for (let j = end; j < n; j++) {
    for (let kind = 0; kind < 2; kind++) {
      if (Number.isFinite(cost[kind]![j]!) && (last < 0 || cost[kind]![j]! < cost[lastKind]![last]!)) {
        last = j;
        lastKind = kind;
      }
    }
  }
  if (last < 0) throw new Error('cutAxis: no feasible cut');
  const cuts: number[] = [];
  for (let j = last, kind = lastKind; ; ) {
    cuts.push(pos(j));
    if (j === 0) break;
    const pj = prev[kind]![j]!;
    kind = prevKind[kind]![j]!;
    j = pj;
  }
  cuts.reverse();
  const centers: number[] = [];
  const walls: number[][] = [];
  const lines = [...new Set(wallBands.map((w) => Math.round(((w.a + w.b) / 2) * 1000) / 1000))].sort((p, q) => p - q);
  for (let k = 0; k + 1 < cuts.length; k++) {
    const a = cuts[k]!;
    const b = cuts[k + 1]!;
    centers.push((a + b) / 2);
    walls.push(lines.filter((x) => x > a && x < b));
  }
  return { centers, walls, span: hi - lo };
}

/** Largest axis-aligned rectangle of `mask` cells (histogram method). */
function largestRect(mask: Uint8Array, w: number, d: number): Room | null {
  const heights = new Int32Array(w);
  let best: Room | null = null;
  let bestArea = 0;
  for (let z = 0; z < d; z++) {
    for (let x = 0; x < w; x++) heights[x] = mask[z * w + x] ? heights[x]! + 1 : 0;
    const stack: number[] = [];
    for (let x = 0; x <= w; x++) {
      const h = x < w ? heights[x]! : 0;
      while (stack.length && heights[stack[stack.length - 1]!]! >= h) {
        const top = stack.pop()!;
        const height = heights[top]!;
        const left = stack.length ? stack[stack.length - 1]! + 1 : 0;
        const area = height * (x - left);
        if (area > bestArea) {
          bestArea = area;
          best = { id: 0, x: left, z: z - height + 1, w: x - left, d: height };
        }
      }
      stack.push(x);
    }
  }
  return best;
}

/** Stable 32-bit FNV-1a hash of the plan id, used as the building seed. */
export function planSeed(id: string): number {
  let h = 0x811c9dc5;
  for (let i = 0; i < id.length; i++) {
    h ^= id.charCodeAt(i);
    h = Math.imul(h, 0x01000193);
  }
  return h >>> 0;
}

/**
 * Convert a plan. Returns null when the result is unusable (no room, or a room
 * the drone cannot reach at flight altitude even without furniture).
 */
export interface ConvertedPlan {
  spec: BuildingSpec;
  /** Room type per spec room id (floor 0). */
  roomTypes: string[];
  /** Index into plan.areas per spec room id. */
  roomAreas: number[];
  /** Areas that produced no listed room (closets under 2 cells, or too narrow). */
  droppedAreas: number;
  /** Door cells that join two rooms, and whether the entrance was found (one side outside). */
  doors: number;
  entrance: boolean;
  /** Plan footprint (m) and sim footprint (cells, border included). */
  planSize: [number, number];
  gridSize: [number, number];
  /** Indoor plan area (m²) and walkable cells. */
  planArea: number;
  walkableCells: number;
}

export function planToBuilding(plan: PlanRecord, opts: { furniture?: boolean } = {}): ConvertedPlan | null {
  const wallBoxes = plan.walls.map(ringBox);
  const doorBoxes = plan.doors.map(ringBox);
  const windowBoxes = plan.windows.map(ringBox);
  const areaBoxes = plan.areas.map((a) => ringBox(a.poly));
  const all: Box[] = [...wallBoxes, ...areaBoxes];
  if (areaBoxes.length === 0) return null;
  const x0 = Math.min(...all.map((b) => b.x0));
  const x1 = Math.max(...all.map((b) => b.x1));
  const y0 = Math.min(...all.map((b) => b.y0));
  const y1 = Math.max(...all.map((b) => b.y1));

  const xWalls: Band[] = [];
  const yWalls: Band[] = [];
  for (const b of wallBoxes) {
    const bw = b.x1 - b.x0;
    const bh = b.y1 - b.y0;
    if (bw <= WALL_MAX_THICKNESS && bh >= WALL_MIN_LENGTH) xWalls.push({ a: b.x0, b: b.x1, c0: b.y0, c1: b.y1 });
    else if (bh <= WALL_MAX_THICKNESS && bw >= WALL_MIN_LENGTH) yWalls.push({ a: b.y0, b: b.y1, c0: b.x0, c1: b.x1 });
  }
  // A door's width (its long side) should hold a cell centre; its thin side sits in a wall band.
  const xDoors: { a: number; b: number }[] = [];
  const yDoors: { a: number; b: number }[] = [];
  for (const b of doorBoxes) {
    if (b.x1 - b.x0 >= b.y1 - b.y0) xDoors.push({ a: b.x0, b: b.x1 });
    else yDoors.push({ a: b.y0, b: b.y1 });
  }
  const ax = cutAxis(x0, x1, xWalls, xDoors);
  const ay = cutAxis(y0, y1, yWalls, yDoors);
  const w = ax.centers.length + 2;
  const d = ay.centers.length + 2;

  const OUT = -1;
  const WALL = -2;
  const DOOR = -3;
  // label: area index, or OUT / WALL / DOOR.
  const label = new Int32Array(w * d).fill(OUT);
  const glass = new Uint8Array(w * d);
  const px: number[] = [];
  const py: number[] = [];
  const inWall = (x: number, y: number) => {
    for (let k = 0; k < plan.walls.length; k++) if (inBox(wallBoxes[k]!, x, y) && inRing(plan.walls[k]!, x, y)) return true;
    return false;
  };
  const inDoor = (x: number, y: number) => doorBoxes.some((b) => inBox(b, x, y, OPENING_PAD));
  const areaAt = (x: number, y: number) => {
    for (let k = 0; k < plan.areas.length; k++) if (inBox(areaBoxes[k]!, x, y) && inRing(plan.areas[k]!.poly, x, y)) return k;
    return OUT;
  };
  for (let gz = 1; gz < d - 1; gz++) {
    for (let gx = 1; gx < w - 1; gx++) {
      const i = gz * w + gx;
      const cx = ax.centers[gx - 1]!;
      const cy = ay.centers[gz - 1]!;
      const wxs = ax.walls[gx - 1]!;
      const wys = ay.walls[gz - 1]!;
      // Centre first, then the wall centre lines this cell holds.
      px.length = 0;
      py.length = 0;
      px.push(cx);
      py.push(cy);
      for (const wx of wxs) {
        px.push(wx);
        py.push(cy);
      }
      for (const wy of wys) {
        px.push(cx);
        py.push(wy);
        for (const wx of wxs) {
          px.push(wx);
          py.push(wy);
        }
      }
      let l = OUT;
      for (let k = 1; k < px.length && l === OUT; k++) if (inDoor(px[k]!, py[k]!)) l = DOOR;
      for (let k = 0; k < px.length && l === OUT; k++) if (inWall(px[k]!, py[k]!)) l = WALL;
      for (let k = 0; k < px.length && l === OUT; k++) l = areaAt(px[k]!, py[k]!);
      label[i] = l;
      if (l === WALL || l === OUT) {
        for (let k = 0; k < px.length; k++) if (windowBoxes.some((b) => inBox(b, px[k]!, py[k]!, OPENING_PAD))) glass[i] = 1;
      }
    }
  }

  // A door cell is kept when it joins indoor cells on two opposite sides (or
  // joins another door cell that does, for doors two cells deep). One that
  // leads outside on one side is the entrance; the drone starts next to it.
  const indoor = (i: number) => label[i]! >= 0;
  let entranceCell = -1;
  let doors = 0;
  const keep = new Uint8Array(w * d);
  for (let gz = 1; gz < d - 1; gz++) {
    for (let gx = 1; gx < w - 1; gx++) {
      const i = gz * w + gx;
      if (label[i] !== DOOR) continue;
      const pass = (a: number, b: number) =>
        (indoor(a) || label[a] === DOOR) && (indoor(b) || label[b] === DOOR) && (indoor(a) || indoor(b));
      if (pass(i - 1, i + 1) || pass(i - w, i + w)) {
        keep[i] = 1;
        doors += 1;
      } else if (entranceCell < 0) {
        for (const n of [i - 1, i + 1, i - w, i + w]) if (indoor(n)) entranceCell = n;
      }
    }
  }

  const cells = new Uint8Array(w * d).fill(CELL.WALL);
  for (let i = 0; i < w * d; i++) {
    if (indoor(i) || keep[i]) cells[i] = label[i] === DOOR ? CELL.DOOR : CELL.EMPTY;
  }

  // Keep the component the drone starts in; unreachable pockets (shafts
  // misread as rooms, closets behind a missing door) become solid.
  let start = entranceCell;
  if (start < 0) {
    let bestCount = -1;
    for (let k = 0; k < plan.areas.length; k++) {
      let count = 0;
      let first = -1;
      for (let i = 0; i < w * d; i++) {
        if (label[i] === k) {
          count += 1;
          if (first < 0) first = i;
        }
      }
      if (count > bestCount) {
        bestCount = count;
        start = first;
      }
    }
  }
  if (start < 0) return null;
  const seen = new Uint8Array(w * d);
  const queue = [start];
  seen[start] = 1;
  for (let h = 0; h < queue.length; h++) {
    const i = queue[h]!;
    for (const n of [i - 1, i + 1, i - w, i + w]) {
      if (n < 0 || n >= w * d || seen[n] || cells[n] === CELL.WALL) continue;
      seen[n] = 1;
      queue.push(n);
    }
  }
  for (let i = 0; i < w * d; i++) if (!seen[i]) cells[i] = CELL.WALL;

  const rooms: Room[] = [];
  const roomTypes: string[] = [];
  const roomAreas: number[] = [];
  let droppedAreas = 0;
  const mask = new Uint8Array(w * d);
  for (let k = 0; k < plan.areas.length; k++) {
    mask.fill(0);
    for (let i = 0; i < w * d; i++) if (label[i] === k && cells[i] === CELL.EMPTY) mask[i] = 1;
    const rect = largestRect(mask, w, d);
    if (!rect || rect.w * rect.d < MIN_ROOM_CELLS) {
      droppedAreas += 1;
      continue;
    }
    rect.id = rooms.length;
    rooms.push(rect);
    roomTypes.push(plan.areas[k]!.type);
    roomAreas.push(k);
  }
  if (rooms.length === 0) return null;

  const surfaces = new Uint8Array(w * d);
  let walkableCells = 0;
  for (let i = 0; i < w * d; i++) {
    if (cells[i] !== CELL.WALL) {
      surfaces[i] = SURFACE.WOOD;
      walkableCells += 1;
    } else if (glass[i]) surfaces[i] = SURFACE.GLASS;
    else surfaces[i] = label[i] === WALL ? SURFACE.DRYWALL : SURFACE.CONCRETE;
  }
  let spawnCell = entranceCell >= 0 && cells[entranceCell] === CELL.EMPTY ? entranceCell : -1;
  if (spawnCell < 0) {
    let best = rooms[0]!;
    for (const room of rooms) if (room.w * room.d > best.w * best.d) best = room;
    spawnCell = (best.z + Math.floor(best.d / 2)) * w + best.x + Math.floor(best.w / 2);
  }

  // Crop to the walkable cells plus a one-cell wall border.
  let minX = w;
  let minZ = d;
  let maxX = 0;
  let maxZ = 0;
  for (let i = 0; i < w * d; i++) {
    if (cells[i] === CELL.WALL) continue;
    const x = i % w;
    const z = Math.floor(i / w);
    minX = Math.min(minX, x);
    maxX = Math.max(maxX, x);
    minZ = Math.min(minZ, z);
    maxZ = Math.max(maxZ, z);
  }
  const ox = minX - 1;
  const oz = minZ - 1;
  const cw = maxX - minX + 3;
  const cd = maxZ - minZ + 3;
  const crop = (src: Uint8Array) => {
    const out = new Uint8Array(cw * cd);
    for (let z = 0; z < cd; z++) for (let x = 0; x < cw; x++) out[z * cw + x] = src[(z + oz) * w + x + ox]!;
    return out;
  };
  for (const room of rooms) {
    room.x -= ox;
    room.z -= oz;
  }
  const floor: FloorGrid = { w: cw, d: cd, cells: crop(cells), rooms, surfaces: crop(surfaces) };
  const sx = (spawnCell % w) - ox;
  const sz = Math.floor(spawnCell / w) - oz;
  const spec: BuildingSpec = {
    seed: planSeed(plan.id),
    cellSize: CELL_SIZE,
    floorHeight: FLOOR_HEIGHT,
    floors: [floor],
    spawn: [(sx + 0.5) * CELL_SIZE, (sz + 0.5) * CELL_SIZE, 1],
    reachableVoxels: 0,
  };
  const empty = computeFlightMap(spec);
  if (!isFullyReachable(spec, empty)) return null;
  if (opts.furniture === false) {
    spec.furniture = { props: [], boxes: [] };
    spec.reachableVoxels = empty.count;
  } else {
    furnish(spec);
  }

  const planArea = plan.areas.reduce((sum, a) => sum + ringArea(a.poly), 0);
  return {
    spec,
    roomTypes,
    roomAreas,
    droppedAreas,
    doors,
    entrance: entranceCell >= 0,
    planSize: [ax.span, ay.span],
    gridSize: [cw, cd],
    planArea,
    walkableCells,
  };
}

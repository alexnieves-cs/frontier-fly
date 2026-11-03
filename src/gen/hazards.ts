/**
 * Indoor hazards a reviewer asks about: shut and ajar interior doors, glass rooms, mirrors and smoke.
 *
 * applyHazards() takes a generated building and returns a copy with the hazards built in, seeded, so
 * every consumer reads the same layout from the spec:
 *
 * - A shut door turns one layer of a doorway into wall cells with SURFACE.DOOR. It is as solid as a
 *   wall: the drone cannot open it, the lidar and camera stop at it, and the rooms behind it leave
 *   the coverage denominator unless another route reaches them. Doors are only shut while at least
 *   HAZARD_MIN_REACHABLE of the building's reachable cells stay reachable from the spawn.
 * - An ajar door is a 0.9 m leaf (furniture prop kind 'door') hinged at a jamb and swung 30 to 75
 *   degrees into the passage, so the gap left is 0.2 to 0.8 m: some a drone 0.3 m across can thread,
 *   some it cannot. The leaf is thin, so a 32-beam lidar ring can miss it edge-on.
 * - A glass room has the inner face of its walls in SURFACE.GLASS. The existing lidar model drops
 *   35% of glass returns (sim/dynamics/noise.ts), and a dropped return reads as open air.
 * - A mirror is a wall cell in SURFACE.MIRROR. The lidar measures the reflected path, so the wall
 *   reads as a room that is not there (sim/sensors.ts).
 * - Smoke fills chosen rooms. It cuts the camera's contrast with distance (perception/detector.ts)
 *   and gives the lidar early returns off the particles (sim/sensors.ts).
 *
 * The coverage denominator (spec.reachableVoxels) is recomputed with the shut doors and leaves in place.
 */
import { CELL, SURFACE } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec, ClosedDoor, FloorGrid, FurnitureBox, HazardLayout, HazardParams, PropPlacement } from '../shared/types';
import { propBoxes } from './furniture';
import { computeFlightMap, isWalkable, reachableMasks } from './validate';

/** Shut doors never cut the building below this share of its reachable cells. */
export const HAZARD_MIN_REACHABLE = 0.5;
/** Swing of an ajar leaf from shut, radians. */
export const AJAR_ANGLE: [number, number] = [(30 * Math.PI) / 180, (75 * Math.PI) / 180];
export const DEFAULT_SMOKE_VISIBILITY: [number, number] = [1.5, 4];

/** One interior doorway: its leaf layer (the middle row of its door cells across the passage). */
export interface Doorway {
  floor: number;
  /** Passage axis: 'x' when the drone crosses it moving along x. */
  axis: 'x' | 'y';
  /** Cell coordinate of the leaf layer along the passage axis. */
  at: number;
  /** First and last cell of the leaf along the other axis. */
  from: number;
  to: number;
}

function cloneSpec(spec: BuildingSpec): BuildingSpec {
  return {
    ...spec,
    spawn: [spec.spawn[0], spec.spawn[1], spec.spawn[2]],
    floors: spec.floors.map((floor) => ({
      ...floor,
      cells: floor.cells.slice(),
      surfaces: floor.surfaces ? floor.surfaces.slice() : new Uint8Array(floor.w * floor.d),
      rooms: floor.rooms.map((room) => ({ ...room })),
    })),
    furniture: spec.furniture
      ? { props: spec.furniture.props.map((p) => ({ ...p })), boxes: spec.furniture.boxes.map((b) => ({ ...b })) }
      : { props: [], boxes: [] },
  };
}

/**
 * Interior doorways of every floor: 4-connected groups of CELL.DOOR cells whose open neighbours lie
 * along one axis only. A group that opens on both axes (a corner mouth) is left alone.
 */
export function findDoorways(spec: BuildingSpec): Doorway[] {
  const out: Doorway[] = [];
  spec.floors.forEach((floor, f) => {
    const { w, d, cells } = floor;
    const seen = new Uint8Array(w * d);
    for (let i = 0; i < cells.length; i++) {
      if (cells[i] !== CELL.DOOR || seen[i]) continue;
      const group: number[] = [];
      const stack = [i];
      seen[i] = 1;
      while (stack.length) {
        const k = stack.pop()!;
        group.push(k);
        const x = k % w;
        const z = Math.floor(k / w);
        for (const [dx, dz] of [[1, 0], [-1, 0], [0, 1], [0, -1]] as const) {
          const nx = x + dx;
          const nz = z + dz;
          if (nx < 0 || nz < 0 || nx >= w || nz >= d) continue;
          const n = nz * w + nx;
          if (cells[n] === CELL.DOOR && !seen[n]) {
            seen[n] = 1;
            stack.push(n);
          }
        }
      }
      const inGroup = new Set(group);
      let openX = false;
      let openY = false;
      for (const k of group) {
        const x = k % w;
        const z = Math.floor(k / w);
        const open = (nx: number, nz: number) =>
          nx >= 0 && nz >= 0 && nx < w && nz < d && !inGroup.has(nz * w + nx) && isWalkable(cells[nz * w + nx]!);
        if (open(x + 1, z) || open(x - 1, z)) openX = true;
        if (open(x, z + 1) || open(x, z - 1)) openY = true;
      }
      if (openX === openY) continue;
      const axis = openX ? 'x' : 'y';
      // Leaf layer: the middle row across the passage.
      const along = group.map((k) => (axis === 'x' ? k % w : Math.floor(k / w)));
      const lo = Math.min(...along);
      const hi = Math.max(...along);
      const at = lo + Math.floor((hi - lo) / 2);
      const across = group.filter((_, j) => along[j] === at).map((k) => (axis === 'x' ? Math.floor(k / w) : k % w));
      const from = Math.min(...across);
      const to = Math.max(...across);
      if (to - from + 1 !== across.length) continue;
      out.push({ floor: f, axis, at, from, to });
    }
  });
  return out;
}

function doorwayCells(floor: FloorGrid, door: Doorway): number[] {
  const out: number[] = [];
  for (let p = door.from; p <= door.to; p++) out.push(door.axis === 'x' ? p * floor.w + door.at : door.at * floor.w + p);
  return out;
}

function reachableCells(spec: BuildingSpec): number {
  return reachableMasks(spec).reduce((n, mask) => n + mask.reduce((a, v) => a + v, 0), 0);
}

/** Leaves for an ajar doorway: one per jamb for a two-cell doorway, one for a single cell. */
function ajarLeaves(spec: BuildingSpec, door: Doorway, rng: RNG): PropPlacement[] | null {
  const width = door.to - door.from + 1;
  if (width > 2) return null;
  const floor = spec.floors[door.floor]!;
  const cs = spec.cellSize;
  // Swing into the side whose neighbour cells are all open (both usually are).
  const sideOpen = (s: number) =>
    doorwayCells(floor, { ...door, at: door.at + s }).every((k) => k >= 0 && k < floor.cells.length && isWalkable(floor.cells[k]!));
  const sides = [-1, 1].filter(sideOpen);
  if (!sides.length) return null;
  const s = sides[rng.int(sides.length)]!;
  const theta = rng.range(AJAR_ANGLE[0], AJAR_ANGLE[1]);
  // Hinge on the face of the leaf layer toward the swing side, at a jamb.
  const hingeAlong = (door.at + (s > 0 ? 1 : 0)) * cs;
  const hinges: { across: number; dir: number }[] =
    width === 2
      ? [
          { across: door.from * cs, dir: 1 },
          { across: (door.to + 1) * cs, dir: -1 },
        ]
      : [rng.bool(0.5) ? { across: door.from * cs, dir: 1 } : { across: (door.to + 1) * cs, dir: -1 }];
  return hinges.map((h) => {
    // Leaf direction: shut lies along the doorway (dir), opening swings toward the side s.
    const da = s * Math.sin(theta);
    const dp = h.dir * Math.cos(theta);
    const [ux, uy] = door.axis === 'x' ? [da, dp] : [dp, da];
    const [hx, hy] = door.axis === 'x' ? [hingeAlong, h.across] : [h.across, hingeAlong];
    // propBoxes maps local +x to (cos(-rot), sin(-rot)).
    return { kind: 'door', x: hx, y: door.floor * spec.floorHeight, z: hy, rot: -Math.atan2(uy, ux), floor: door.floor };
  });
}

/** Building with the requested hazards, seeded by the spec's seed and `salt`. The input is not changed. */
export function applyHazards(input: BuildingSpec, params: HazardParams, salt = 0): BuildingSpec {
  const spec = cloneSpec(input);
  const rng = new RNG((input.seed + Math.imul(salt, 0x9e3779b1)) >>> 0, 'hazards');
  const layout: HazardLayout = { params: { ...params }, closedDoors: [], ajarDoors: 0, glassRooms: 0, mirrors: 0, smoke: null };
  const furniture = spec.furniture!;

  // Doors.
  const doorways = findDoorways(spec);
  for (let i = doorways.length - 1; i > 0; i--) {
    const j = rng.int(i + 1);
    [doorways[i], doorways[j]] = [doorways[j]!, doorways[i]!];
  }
  const fullCells = reachableCells(spec);
  const leafProps: number[] = [];
  for (const door of doorways) {
    const floor = spec.floors[door.floor]!;
    const cells = doorwayCells(floor, door);
    const roll = rng.next();
    if (roll < (params.closedDoors ?? 0)) {
      for (const k of cells) floor.cells[k] = CELL.WALL;
      if (reachableCells(spec) < HAZARD_MIN_REACHABLE * fullCells) {
        for (const k of cells) floor.cells[k] = CELL.DOOR;
        continue;
      }
      for (const k of cells) floor.surfaces![k] = SURFACE.DOOR;
      const shut: ClosedDoor = { floor: door.floor, cells };
      layout.closedDoors.push(shut);
    } else if (rng.next() < (params.ajarDoors ?? 0)) {
      const leaves = ajarLeaves(spec, door, rng);
      if (!leaves) continue;
      for (const prop of leaves) {
        leafProps.push(furniture.props.length);
        furniture.boxes.push(...propBoxes(spec, prop, furniture.props.length));
        furniture.props.push(prop);
      }
      layout.ajarDoors += 1;
    }
  }

  // Glass rooms and mirrors: interior wall faces only, the outer wall keeps its windows.
  const interiorWall = (floor: FloorGrid, x: number, z: number) =>
    x > 0 && z > 0 && x < floor.w - 1 && z < floor.d - 1 && floor.cells[z * floor.w + x] === CELL.WALL;
  spec.floors.forEach((floor) => {
    for (const room of floor.rooms) {
      if (!(rng.next() < (params.glassRooms ?? 0))) continue;
      let any = false;
      for (let z = room.z - 1; z <= room.z + room.d; z++) {
        for (let x = room.x - 1; x <= room.x + room.w; x++) {
          const edge = z === room.z - 1 || z === room.z + room.d || x === room.x - 1 || x === room.x + room.w;
          if (!edge || !interiorWall(floor, x, z)) continue;
          const k = z * floor.w + x;
          if (floor.surfaces![k] === SURFACE.DOOR) continue;
          floor.surfaces![k] = SURFACE.GLASS;
          any = true;
        }
      }
      if (any) layout.glassRooms += 1;
    }
  });
  spec.floors.forEach((floor) => {
    const want = Math.max(0, Math.floor(params.mirrors ?? 0));
    if (!want) return;
    const options: number[] = [];
    for (let z = 1; z < floor.d - 1; z++) {
      for (let x = 1; x < floor.w - 1; x++) {
        if (!interiorWall(floor, x, z)) continue;
        const k = z * floor.w + x;
        const s = floor.surfaces![k];
        if (s !== SURFACE.DRYWALL && s !== SURFACE.CONCRETE) continue;
        const faces = [[1, 0], [-1, 0], [0, 1], [0, -1]].some(([dx, dz]) => isWalkable(floor.cells[(z + dz!) * floor.w + x + dx!]!));
        if (faces) options.push(k);
      }
    }
    for (let n = 0; n < want && options.length; n++) {
      const k = options.splice(rng.int(options.length), 1)[0]!;
      floor.surfaces![k] = SURFACE.MIRROR;
      layout.mirrors += 1;
    }
  });

  // Smoke.
  if ((params.smokeRooms ?? 0) > 0) {
    const [vMin, vMax] = params.smokeVisibility ?? DEFAULT_SMOKE_VISIBILITY;
    const masks = spec.floors.map((floor) => new Uint8Array(floor.w * floor.d));
    let any = false;
    spec.floors.forEach((floor, f) => {
      for (const room of floor.rooms) {
        if (!(rng.next() < params.smokeRooms!)) continue;
        any = true;
        for (let z = room.z; z < room.z + room.d; z++) {
          for (let x = room.x; x < room.x + room.w; x++) {
            if (x > 0 && z > 0 && x < floor.w - 1 && z < floor.d - 1) masks[f]![z * floor.w + x] = 1;
          }
        }
      }
    });
    const visibility = rng.range(vMin, vMax);
    if (any) layout.smoke = { visibility, cells: masks };
  }

  spec.hazards = layout;
  // The denominator with shut doors and leaves in place. If the leaves cut it below the floor, take
  // them out again, last first.
  let map = computeFlightMap(spec);
  const floorCount = HAZARD_MIN_REACHABLE * input.reachableVoxels;
  while (map.count < floorCount && leafProps.length) {
    const prop = leafProps.pop()!;
    furniture.boxes = furniture.boxes.filter((b: FurnitureBox) => b.prop !== prop);
    layout.ajarDoors = new Set(furniture.boxes.filter((b) => furniture.props[b.prop]?.kind === 'door').map((b) => b.prop)).size;
    map = computeFlightMap(spec);
  }
  spec.reachableVoxels = map.count;
  return spec;
}

/** Smoke path length, meters, along the horizontal segment (x0, y0) -> (x1, y1) on floor f. */
export function smokePath(spec: BuildingSpec, f: number, x0: number, y0: number, x1: number, y1: number, step = 0.25): number {
  const smoke = spec.hazards?.smoke;
  const mask = smoke?.cells[f];
  if (!mask) return 0;
  const floor = spec.floors[f]!;
  const len = Math.hypot(x1 - x0, y1 - y0);
  const n = Math.max(1, Math.ceil(len / step));
  const ds = len / n;
  let s = 0;
  for (let i = 0; i < n; i++) {
    const t = (i + 0.5) / n;
    const gx = Math.floor((x0 + (x1 - x0) * t) / spec.cellSize);
    const gy = Math.floor((y0 + (y1 - y0) * t) / spec.cellSize);
    if (gx < 0 || gy < 0 || gx >= floor.w || gy >= floor.d) continue;
    if (mask[gy * floor.w + gx]) s += ds;
  }
  return s;
}

/** Camera contrast left after `meters` of smoke at this visibility (Koschmieder: 2% at the visibility range). */
export function smokeTransmittance(meters: number, visibility: number): number {
  return meters <= 0 ? 1 : Math.exp((-3.912 * meters) / visibility);
}

/** True when the cell under (x, y) on floor f is in smoke. */
export function inSmoke(spec: BuildingSpec, f: number, x: number, y: number): boolean {
  const mask = spec.hazards?.smoke?.cells[f];
  if (!mask) return false;
  const floor = spec.floors[f]!;
  const gx = Math.floor(x / spec.cellSize);
  const gy = Math.floor(y / spec.cellSize);
  return gx >= 0 && gy >= 0 && gx < floor.w && gy < floor.d && mask[gy * floor.w + gx] === 1;
}

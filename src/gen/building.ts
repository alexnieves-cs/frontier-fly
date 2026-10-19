import { CELL, CELL_SIZE, FLOOR_HEIGHT, SURFACE } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec, FloorGrid, GenParams, Room } from '../shared/types';
import { splitRooms, type Rect } from './bsp';
import { applyLinks, planLinks } from './connect';
import { carveShafts, cloneFloor, sharedWalkable } from './shafts';
import { planFurniture, withoutProps } from './furniture';
import { computeFlightMap, isFullyReachable } from './validate';

function carveRoom(cells: Uint8Array, w: number, room: Room): void {
  for (let z = room.z; z < room.z + room.d; z++) {
    for (let x = room.x; x < room.x + room.w; x++) {
      if (x <= 0 || z <= 0 || x >= w - 1 || z >= cells.length / w - 1) continue;
      cells[z * w + x] = CELL.EMPTY;
    }
  }
}

function roomFromLeaf(leaf: Rect, id: number, stage: number): Room {
  const margin = stage >= 3 && leaf.w >= 7 && leaf.d >= 7 ? 2 : 1;
  let x = leaf.x + margin;
  let z = leaf.z + margin;
  let w = leaf.w - margin * 2;
  let d = leaf.d - margin * 2;
  if (w < 2 || d < 2) {
    x = leaf.x;
    z = leaf.z;
    w = leaf.w;
    d = leaf.d;
    if (x <= 0) {
      x = 1;
      w -= 1;
    }
    if (z <= 0) {
      z = 1;
      d -= 1;
    }
  }
  return { id, x, z, w: Math.max(2, w), d: Math.max(2, d) };
}

function placePillars(cells: Uint8Array, w: number, room: Room, rng: RNG): void {
  const n = 2 + rng.int(3);
  const cx = room.x + Math.floor(room.w / 2);
  const cz = room.z + Math.floor(room.d / 2);
  for (let k = 0; k < n; k++) {
    if (room.w < 4 || room.d < 4) return;
    const x = room.x + 1 + rng.int(room.w - 2);
    const z = room.z + 1 + rng.int(room.d - 2);
    if (Math.abs(x - cx) + Math.abs(z - cz) <= 1) continue;
    cells[z * w + x] = CELL.WALL;
  }
}

function paint(cells: Uint8Array, w: number, d: number, rng: RNG): Uint8Array {
  const surfaces = new Uint8Array(w * d);
  for (let z = 0; z < d; z++) {
    for (let x = 0; x < w; x++) {
      const i = z * w + x;
      const border = x === 0 || z === 0 || x === w - 1 || z === d - 1;
      const corner = (x === 0 || x === w - 1) && (z === 0 || z === d - 1);
      if (cells[i] === CELL.WALL && border && !corner && (x + z) % 2 === 0) surfaces[i] = SURFACE.GLASS;
      else if (cells[i] === CELL.WALL) surfaces[i] = rng.bool(0.12) ? SURFACE.CONCRETE : SURFACE.DRYWALL;
      else if (cells[i] === CELL.SHAFT) surfaces[i] = SURFACE.METAL;
      else surfaces[i] = SURFACE.WOOD;
    }
  }
  return surfaces;
}

function makeFloor(seed: number, w: number, d: number, stage: number): FloorGrid {
  const cells = new Uint8Array(w * d);
  cells.fill(CELL.WALL);
  let rooms: Room[];
  if (stage <= 1) {
    const room: Room = { id: 0, x: 1, z: 1, w: w - 2, d: d - 2 };
    carveRoom(cells, w, room);
    rooms = [room];
    if (stage === 1) placePillars(cells, w, room, new RNG(seed, 'pillars'));
  } else {
    const leaves = splitRooms(new RNG(seed, 'bsp'), { x: 1, z: 1, w: w - 2, d: d - 2 }, stage);
    rooms = leaves.map((leaf, id) => roomFromLeaf(leaf, id, stage));
    for (const room of rooms) carveRoom(cells, w, room);
    applyLinks(cells, w, d, rooms, planLinks(rooms, stage));
  }
  return { w, d, cells, rooms, surfaces: paint(cells, w, d, new RNG(seed, 'surf')) };
}

function spawnOf(floor: FloorGrid, cellSize: number): { spawn: [number, number, number]; cell: [number, number] } {
  let best = floor.rooms[0]!;
  for (const room of floor.rooms) {
    if (room.w * room.d > best.w * best.d) best = room;
  }
  let x = best.x + Math.floor(best.w / 2);
  let z = best.z + Math.floor(best.d / 2);
  if (floor.cells[z * floor.w + x] === CELL.WALL) {
    for (let dz = -2; dz <= 2 && floor.cells[z * floor.w + x] === CELL.WALL; dz++) {
      for (let dx = -2; dx <= 2; dx++) {
        const nx = x + dx;
        const nz = z + dz;
        if (nx <= 0 || nz <= 0 || nx >= floor.w - 1 || nz >= floor.d - 1) continue;
        if (floor.cells[nz * floor.w + nx] !== CELL.WALL) {
          x = nx;
          z = nz;
          break;
        }
      }
    }
  }
  return { spawn: [(x + 0.5) * cellSize, (z + 0.5) * cellSize, 1], cell: [x, z] };
}

function tryGenerate(p: GenParams): BuildingSpec | null {
  const w = Math.max(6, p.w | 0);
  const d = Math.max(6, p.d | 0);
  const floorsN = Math.max(1, Math.min(4, p.floors | 0));
  const stage = Math.max(0, Math.min(5, p.stage | 0));
  const floors: FloorGrid[] = [];
  for (let f = 0; f < floorsN; f++) {
    floors.push(makeFloor((p.seed + f * 101) >>> 0, w, d, stage));
  }
  if (floorsN > 1) {
    for (let f = 0; f < floorsN - 1; f++) {
      if (sharedWalkable(floors[f]!, floors[f + 1]!) < 4) {
        for (let i = 1; i < floorsN; i++) floors[i] = cloneFloor(floors[0]!);
        break;
      }
    }
  }
  const { spawn, cell } = spawnOf(floors[0]!, CELL_SIZE);
  if (floorsN > 1) {
    if (!carveShafts(floors, new RNG(p.seed, 'shaft'), cell)) return null;
  }
  const spec: BuildingSpec = {
    seed: p.seed >>> 0,
    cellSize: CELL_SIZE,
    floorHeight: FLOOR_HEIGHT,
    floors,
    spawn,
    reachableVoxels: 0,
  };
  if (!isFullyReachable(spec, computeFlightMap(spec))) return null;
  // Furniture is solid. Its keep-out margin keeps doors and shafts clear, but if
  // a layout still cuts a room off at flight altitude, drop props from the end
  // (deterministically) until every room is reachable again.
  let furniture = planFurniture(spec);
  for (;;) {
    spec.furniture = furniture;
    const map = computeFlightMap(spec);
    if (isFullyReachable(spec, map)) {
      spec.reachableVoxels = map.count;
      break;
    }
    furniture = withoutProps(spec, furniture, new Set([furniture.props.length - 1]));
  }
  return spec;
}

/** Seeded building. Retries with seed+1 until every room is reachable. */
export function generateBuilding(p: GenParams): BuildingSpec {
  let seed = p.seed >>> 0;
  for (let attempt = 0; attempt < 32; attempt++) {
    const spec = tryGenerate({ ...p, seed });
    if (spec) return spec;
    seed = (seed + 1) >>> 0;
  }
  throw new Error(`generateBuilding failed for seed ${p.seed}`);
}

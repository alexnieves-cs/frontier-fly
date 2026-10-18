import { CELL } from '../shared/constants';
import { reachableMasks } from '../gen/validate';
import { RNG } from '../shared/rng';
import type { BuildingSpec, Room } from '../shared/types';
import { DECOY_IDS, DECOY_LOOKS_LIKE, DECOY_SIZE, OBJECT_IDS, OBJECT_SIZE, PERSON_SIZE } from './catalog';
import { bodyAabb } from './motion';
import {
  PERSON_STATES,
  type Aabb,
  type OutdoorArea,
  type PersonState,
  type PlaceOptions,
  type PlacedTarget,
  type TargetCategory,
  type TargetClass,
  type TargetScene,
  type Vec3,
} from './types';

interface CellRef {
  f: number;
  ix: number;
  iy: number;
  roomIndex: number;
  room: Room;
}

interface Size {
  hx: number;
  hy: number;
  height: number;
  speed: number;
}

interface Draft {
  classId: TargetClass;
  state: PersonState | 'static';
  feet: Vec3;
  yaw: number;
  size: Size;
  floor: number;
  roomIndex: number;
  path?: Vec3[];
  occluders: Aabb[];
}

function categoryOf(classId: TargetClass): TargetCategory {
  if (classId === 'person') return 'person';
  if (classId in DECOY_LOOKS_LIKE) return 'decoy';
  return 'object';
}

function sizeOfClass(classId: TargetClass, state: PersonState | 'static'): Size {
  if (classId === 'person') return PERSON_SIZE[state === 'static' ? 'standing' : state];
  if (classId in OBJECT_SIZE) return OBJECT_SIZE[classId as keyof typeof OBJECT_SIZE];
  return DECOY_SIZE[classId as keyof typeof DECOY_SIZE];
}

function cellPos(spec: BuildingSpec, f: number, ix: number, iy: number): Vec3 {
  const cs = spec.cellSize;
  return [(ix + 0.5) * cs, (iy + 0.5) * cs, f * spec.floorHeight];
}

function candidates(spec: BuildingSpec): { cells: CellRef[]; masks: Uint8Array[] } {
  const masks = reachableMasks(spec);
  const cells: CellRef[] = [];
  let roomIndex = 0;
  for (let f = 0; f < spec.floors.length; f++) {
    const floor = spec.floors[f]!;
    const mask = masks[f]!;
    for (const room of floor.rooms) {
      for (let iy = room.z; iy < room.z + room.d; iy++) {
        for (let ix = room.x; ix < room.x + room.w; ix++) {
          const i = iy * floor.w + ix;
          if (floor.cells[i] !== CELL.EMPTY || !mask[i]) continue;
          cells.push({ f, ix, iy, roomIndex, room });
        }
      }
      roomIndex += 1;
    }
  }
  return { cells, masks };
}

function coversFit(spec: BuildingSpec, masks: Uint8Array[], f: number, feet: Vec3, yaw: number, size: Size): boolean {
  const floor = spec.floors[f];
  if (!floor) return false;
  const z0 = f * spec.floorHeight;
  const z1 = z0 + spec.floorHeight;
  if (feet[2] < z0 - 0.02 || feet[2] + size.height > z1 - 0.05) return false;
  const box = bodyAabb(feet, yaw, size.hx, size.hy, size.height);
  const cs = spec.cellSize;
  const ix0 = Math.floor(box.min[0] / cs);
  const ix1 = Math.floor((box.max[0] - 1e-4) / cs);
  const iy0 = Math.floor(box.min[1] / cs);
  const iy1 = Math.floor((box.max[1] - 1e-4) / cs);
  const mask = masks[f]!;
  for (let iy = iy0; iy <= iy1; iy++) {
    for (let ix = ix0; ix <= ix1; ix++) {
      if (ix < 0 || iy < 0 || ix >= floor.w || iy >= floor.d) return false;
      const i = iy * floor.w + ix;
      if (floor.cells[i] !== CELL.EMPTY || !mask[i]) return false;
    }
  }
  return true;
}

function boxesOverlap(a: Aabb, b: Aabb): boolean {
  return a.min[0] < b.max[0] && a.max[0] > b.min[0] && a.min[1] < b.max[1] && a.max[1] > b.min[1] && a.min[2] < b.max[2] && a.max[2] > b.min[2];
}

function draftBox(item: Draft): Aabb {
  return bodyAabb(item.feet, item.yaw, item.size.hx, item.size.hy, item.size.height);
}

function hitsPlaced(placed: Draft[], feet: Vec3, yaw: number, size: Size): boolean {
  const box = bodyAabb(feet, yaw, size.hx, size.hy, size.height);
  return placed.some((item) => boxesOverlap(box, draftBox(item)));
}

function wallNeighbor(spec: BuildingSpec, cell: CellRef): boolean {
  const floor = spec.floors[cell.f]!;
  for (const [dx, dy] of [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ] as const) {
    const ix = cell.ix + dx;
    const iy = cell.iy + dy;
    if (ix < 0 || iy < 0 || ix >= floor.w || iy >= floor.d) return true;
    if (floor.cells[iy * floor.w + ix] === CELL.WALL) return true;
  }
  return false;
}

function roomCenter(spec: BuildingSpec, room: Room, f: number): Vec3 {
  const cs = spec.cellSize;
  return [(room.x + room.w / 2) * cs, (room.z + room.d / 2) * cs, f * spec.floorHeight];
}

function hideCrate(feet: Vec3, toward: Vec3): Aabb {
  const dx = toward[0] - feet[0];
  const dy = toward[1] - feet[1];
  const len = Math.hypot(dx, dy) || 1;
  const cx = feet[0] + (dx / len) * 0.5;
  const cy = feet[1] + (dy / len) * 0.5;
  return { min: [cx - 0.32, cy - 0.26, feet[2]], max: [cx + 0.32, cy + 0.26, feet[2] + 0.78] };
}

function crateFits(spec: BuildingSpec, masks: Uint8Array[], f: number, crate: Aabb): boolean {
  const floor = spec.floors[f]!;
  const cs = spec.cellSize;
  const ix0 = Math.floor(crate.min[0] / cs);
  const ix1 = Math.floor((crate.max[0] - 1e-4) / cs);
  const iy0 = Math.floor(crate.min[1] / cs);
  const iy1 = Math.floor((crate.max[1] - 1e-4) / cs);
  for (let iy = iy0; iy <= iy1; iy++) {
    for (let ix = ix0; ix <= ix1; ix++) {
      if (ix < 0 || iy < 0 || ix >= floor.w || iy >= floor.d) return false;
      const i = iy * floor.w + ix;
      if (floor.cells[i] !== CELL.EMPTY || !masks[f]![i]) return false;
    }
  }
  return true;
}

function toPlaced(item: Draft, index: number): PlacedTarget {
  const category = categoryOf(item.classId);
  const kind = item.classId === 'person' ? item.state : item.classId;
  return {
    id: `${category}-${kind}-${index}`,
    classId: item.classId,
    category,
    state: item.state,
    feet: item.feet,
    yaw: item.yaw,
    hx: item.size.hx,
    hy: item.size.hy,
    height: item.size.height,
    path: item.path,
    speed: item.size.speed,
    floor: item.floor,
    roomIndex: item.roomIndex,
    occluders: item.occluders,
    looksLike: category === 'decoy' ? DECOY_LOOKS_LIKE[item.classId as keyof typeof DECOY_LOOKS_LIKE] : undefined,
  };
}

function commit(
  spec: BuildingSpec,
  masks: Uint8Array[],
  placed: Draft[],
  classId: TargetClass,
  state: PersonState | 'static',
  feet: Vec3,
  yaw: number,
  floor: number,
  roomIndex: number,
  path?: Vec3[],
  occluders: Aabb[] = [],
): boolean {
  const size = sizeOfClass(classId, state);
  const poseFeet = path?.[0] ?? feet;
  const poseYaw = path && path.length > 1 ? Math.atan2(path[1]![1] - path[0]![1], path[1]![0] - path[0]![0]) : yaw;
  if (!coversFit(spec, masks, floor, poseFeet, poseYaw, size)) return false;
  if (hitsPlaced(placed, poseFeet, poseYaw, size)) return false;
  if (path) {
    for (let i = 0; i < path.length; i++) {
      const a = path[i]!;
      const b = path[(i + 1) % path.length]!;
      const face = Math.atan2(b[1] - a[1], b[0] - a[0]);
      const mid: Vec3 = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[2]];
      if (!coversFit(spec, masks, floor, a, face, size) || !coversFit(spec, masks, floor, mid, face, size)) return false;
    }
  }
  placed.push({ classId, state, feet: poseFeet, yaw: poseYaw, size, floor, roomIndex, path, occluders });
  return true;
}

function walkPath(rng: RNG, spec: BuildingSpec, masks: Uint8Array[], cells: CellRef[], start: CellRef, size: Size): Vec3[] | null {
  const same = cells.filter((cell) => cell.f === start.f && cell.roomIndex === start.roomIndex);
  for (let attempt = 0; attempt < 16; attempt++) {
    const next = same[rng.int(same.length)]!;
    if (Math.abs(next.ix - start.ix) + Math.abs(next.iy - start.iy) < 2) continue;
    const a = cellPos(spec, start.f, start.ix, start.iy);
    const b = cellPos(spec, next.f, next.ix, next.iy);
    const yaw = Math.atan2(b[1] - a[1], b[0] - a[0]);
    const mid: Vec3 = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, a[2]];
    if (!coversFit(spec, masks, start.f, a, yaw, size)) continue;
    if (!coversFit(spec, masks, start.f, b, yaw, size)) continue;
    if (!coversFit(spec, masks, start.f, mid, yaw, size)) continue;
    return [a, b];
  }
  return null;
}

function placePeople(
  rng: RNG,
  spec: BuildingSpec,
  masks: Uint8Array[],
  cells: CellRef[],
  placed: Draft[],
  count: number,
): void {
  for (let i = 0; i < count; i++) {
    const state = PERSON_STATES[i % PERSON_STATES.length]!;
    const size = PERSON_SIZE[state];
    for (let attempt = 0; attempt < 48; attempt++) {
      const cell = cells[rng.int(cells.length)]!;
      if (state === 'hiding' && !wallNeighbor(spec, cell)) continue;
      const yaw = state === 'lying' ? (rng.bool() ? 0 : Math.PI / 2) : rng.range(-Math.PI, Math.PI);
      const feet = cellPos(spec, cell.f, cell.ix, cell.iy);
      let path: Vec3[] | undefined;
      if (state === 'walking') {
        const built = walkPath(rng, spec, masks, cells, cell, size);
        if (!built) continue;
        path = built;
      }
      let occluders: Aabb[] = [];
      if (state === 'hiding') {
        const crate = hideCrate(feet, roomCenter(spec, cell.room, cell.f));
        if (!crateFits(spec, masks, cell.f, crate)) continue;
        occluders = [crate];
      }
      if (commit(spec, masks, placed, 'person', state, feet, yaw, cell.f, cell.roomIndex, path, occluders)) break;
    }
  }
}

function placeProps(
  rng: RNG,
  spec: BuildingSpec,
  masks: Uint8Array[],
  cells: CellRef[],
  placed: Draft[],
  ids: readonly TargetClass[],
  count: number,
): void {
  for (let i = 0; i < count; i++) {
    const classId = ids[i % ids.length]!;
    const size = sizeOfClass(classId, 'static');
    for (let attempt = 0; attempt < 40; attempt++) {
      const cell = cells[rng.int(cells.length)]!;
      if (classId === 'coat' && !wallNeighbor(spec, cell)) continue;
      const feet = cellPos(spec, cell.f, cell.ix, cell.iy);
      if (classId === 'coat') feet[2] += 0.45;
      const yaw = rng.range(-Math.PI, Math.PI);
      if (!coversFit(spec, masks, cell.f, feet, yaw, size)) continue;
      if (commit(spec, masks, placed, classId, 'static', feet, yaw, cell.f, cell.roomIndex)) break;
    }
  }
}

function placeShowcase(spec: BuildingSpec, masks: Uint8Array[], cells: CellRef[], placed: Draft[]): void {
  let best = cells[0]!;
  let bestArea = -1;
  const seen = new Set<number>();
  for (const cell of cells) {
    if (seen.has(cell.roomIndex)) continue;
    seen.add(cell.roomIndex);
    const area = cell.room.w * cell.room.d;
    if (area > bestArea) {
      best = cell;
      bestArea = area;
    }
  }
  const spots: Array<{ classId: TargetClass; state: PersonState | 'static'; dx: number; dy: number; yaw: number; walk?: number }> = [
    { classId: 'person', state: 'standing', dx: 2, dy: 4, yaw: -Math.PI / 2 },
    { classId: 'person', state: 'sitting', dx: 4, dy: 4, yaw: -Math.PI / 2 },
    { classId: 'person', state: 'lying', dx: 6, dy: 4, yaw: 0 },
    { classId: 'mannequin', state: 'static', dx: 8.4, dy: 4, yaw: -Math.PI / 2 },
    { classId: 'coat', state: 'static', dx: 10, dy: 4.2, yaw: Math.PI / 2 },
    { classId: 'person', state: 'walking', dx: 2, dy: 6.4, yaw: 0, walk: 3.2 },
    { classId: 'person', state: 'hiding', dx: 8.4, dy: 7.6, yaw: -Math.PI / 2 },
    { classId: 'backpack', state: 'static', dx: 2, dy: 2, yaw: 0.2 },
    { classId: 'phone', state: 'static', dx: 3.3, dy: 2, yaw: 0.5 },
    { classId: 'toolbox', state: 'static', dx: 4.6, dy: 2, yaw: 0 },
    { classId: 'medkit', state: 'static', dx: 5.8, dy: 2.1, yaw: 0.2 },
    { classId: 'bottle', state: 'static', dx: 7, dy: 2, yaw: 0 },
    { classId: 'radio', state: 'static', dx: 8.1, dy: 2, yaw: 0.4 },
    { classId: 'helmet', state: 'static', dx: 9.2, dy: 2.1, yaw: 0 },
    { classId: 'blanket', state: 'static', dx: 10.3, dy: 2.3, yaw: 0.2 },
    { classId: 'decoy_pack', state: 'static', dx: 3.3, dy: 3.05, yaw: 0.3 },
    { classId: 'decoy_case', state: 'static', dx: 6.9, dy: 3.05, yaw: 0.1 },
  ];
  const room = best.room;
  for (const spot of spots) {
    const feet: Vec3 = [
      (room.x + spot.dx + 0.5) * spec.cellSize,
      (room.z + spot.dy + 0.5) * spec.cellSize,
      best.f * spec.floorHeight,
    ];
    if (spot.classId === 'coat') feet[2] += 0.45;
    let path: Vec3[] | undefined;
    if (spot.walk) {
      path = [feet, [feet[0] + spot.walk * spec.cellSize, feet[1], feet[2]]];
    }
    let occluders: Aabb[] = [];
    if (spot.state === 'hiding') {
      const crate = hideCrate(feet, roomCenter(spec, room, best.f));
      if (!crateFits(spec, masks, best.f, crate)) continue;
      occluders = [crate];
    }
    commit(spec, masks, placed, spot.classId, spot.state, feet, spot.yaw, best.f, best.roomIndex, path, occluders);
  }
}

function roomCount(cells: CellRef[]): number {
  return new Set(cells.map((cell) => cell.roomIndex)).size;
}

/** Seeded people, objects, and decoys on reachable floor cells. */
export function placeIndoor(spec: BuildingSpec, options: PlaceOptions): TargetScene {
  const rng = new RNG(options.seed >>> 0, 'targets');
  const { cells, masks } = candidates(spec);
  const placed: Draft[] = [];
  if (cells.length === 0) return { seed: options.seed >>> 0, space: 'indoor', targets: [] };
  if (options.showcase) {
    placeShowcase(spec, masks, cells, placed);
  } else {
    const rooms = roomCount(cells);
    const people = options.people ?? Math.min(5, Math.max(2, Math.ceil(rooms / 2)));
    const objects = options.objects ?? Math.min(8, Math.max(3, rooms));
    const decoys = options.decoys ?? Math.min(3, Math.max(1, Math.floor(people / 2)));
    placePeople(rng, spec, masks, cells, placed, people);
    placeProps(rng, spec, masks, cells, placed, OBJECT_IDS, objects);
    placeProps(rng, spec, masks, cells, placed, DECOY_IDS, decoys);
  }
  return { seed: options.seed >>> 0, space: 'indoor', targets: placed.map(toPlaced) };
}

function outdoorBlocked(area: OutdoorArea, box: Aabb): boolean {
  if (box.min[0] < area.minX || box.max[0] > area.maxX || box.min[1] < area.minY || box.max[1] > area.maxY) return true;
  if (box.min[2] < area.groundZ - 0.05 || box.max[2] > area.groundZ + 3) return true;
  for (const obstacle of area.obstacles ?? []) {
    if (box.min[0] < obstacle.maxX && box.max[0] > obstacle.minX && box.min[1] < obstacle.maxY && box.max[1] > obstacle.minY) return true;
  }
  return false;
}

/** Seeded placement on an outdoor rectangle, avoiding obstacle footprints. */
export function placeOutdoor(area: OutdoorArea, options: PlaceOptions): TargetScene {
  const rng = new RNG(options.seed >>> 0, 'targets-outdoor');
  const people = options.people ?? 3;
  const objects = options.objects ?? 4;
  const decoys = options.decoys ?? 2;
  const placed: Draft[] = [];

  const accept = (classId: TargetClass, state: PersonState | 'static', feet: Vec3, yaw: number, path?: Vec3[]): boolean => {
    const size = sizeOfClass(classId, state);
    const samples = path && path.length > 1 ? path : [feet];
    for (let i = 0; i < samples.length; i++) {
      const point = samples[i]!;
      const nxt = samples[(i + 1) % samples.length]!;
      const face = samples.length > 1 ? Math.atan2(nxt[1] - point[1], nxt[0] - point[0]) : yaw;
      const box = bodyAabb(point, face, size.hx, size.hy, size.height);
      if (outdoorBlocked(area, box)) return false;
      if (hitsPlaced(placed, point, face, size)) return false;
    }
    const pose = samples[0]!;
    const poseYaw = samples.length > 1 ? Math.atan2(samples[1]![1] - pose[1], samples[1]![0] - pose[0]) : yaw;
    placed.push({
      classId,
      state,
      feet: [pose[0], pose[1], classId === 'coat' ? area.groundZ + 0.45 : pose[2]],
      yaw: poseYaw,
      size,
      floor: 0,
      roomIndex: -1,
      path,
      occluders: [],
    });
    return true;
  };

  for (let i = 0; i < people; i++) {
    const state = PERSON_STATES[i % PERSON_STATES.length]!;
    for (let attempt = 0; attempt < 40; attempt++) {
      const yaw = state === 'lying' ? (rng.bool() ? 0 : Math.PI / 2) : rng.range(-Math.PI, Math.PI);
      let path: Vec3[] | undefined;
      let feet: Vec3 = [rng.range(area.minX, area.maxX), rng.range(area.minY, area.maxY), area.groundZ];
      if (state === 'walking') {
        const route = area.paths?.[i % Math.max(1, area.paths?.length ?? 1)];
        if (route && route.length >= 2) path = route.map(([x, y]) => [x, y, area.groundZ]);
        else {
          const b: Vec3 = [rng.range(area.minX, area.maxX), rng.range(area.minY, area.maxY), area.groundZ];
          path = [feet, b];
        }
        feet = path[0]!;
      }
      if (accept('person', state, feet, yaw, path)) break;
    }
  }
  const props = [...OBJECT_IDS.slice(0, objects), ...DECOY_IDS.slice(0, decoys)];
  for (const classId of props) {
    for (let attempt = 0; attempt < 40; attempt++) {
      const feet: Vec3 = [rng.range(area.minX, area.maxX), rng.range(area.minY, area.maxY), area.groundZ];
      if (classId === 'coat') feet[2] += 0.45;
      if (accept(classId, 'static', feet, rng.range(-Math.PI, Math.PI))) break;
    }
  }
  return {
    seed: options.seed >>> 0,
    space: 'outdoor',
    outdoor: area,
    targets: placed.map(toPlaced),
  };
}

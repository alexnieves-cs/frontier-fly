import { CELL, CELL_SIZE, FLOOR_HEIGHT } from '../../shared/constants';
import { RNG } from '../../shared/rng';
import type { BuildingSpec, FloorGrid } from '../../shared/types';
import { measureClearance } from './clearance';
import { add, scale, type Vec3 } from './math';
import type { BuildingShell, GridCell, Opening, OpeningKind } from './types';

interface Recipe {
  kind: OpeningKind;
  width: number;
  height: number;
  sill: number;
  angle: number;
  span: number;
}

const PASSABLE: Recipe[] = [
  { kind: 'garage_gap', width: 1.55, height: 1.65, sill: 0.24, angle: 0, span: 2 },
  { kind: 'ajar_door', width: 0.9, height: 2.02, sill: 0.04, angle: 1.4, span: 1 },
  { kind: 'open_window', width: 0.8, height: 0.95, sill: 0.9, angle: 0, span: 1 },
  { kind: 'broken_window', width: 0.7, height: 0.66, sill: 1.05, angle: 0, span: 1 },
  { kind: 'roof_hatch', width: 0.78, height: 0.78, sill: 0, angle: 0, span: 1 },
];

const DECOYS: Recipe[] = [
  { kind: 'vent', width: 0.18, height: 0.12, sill: 1.7, angle: 0, span: 1 },
  { kind: 'closed_door', width: 0.88, height: 2.0, sill: 0.04, angle: 0, span: 1 },
  { kind: 'ajar_door', width: 0.88, height: 2.0, sill: 0.04, angle: 0.16, span: 1 },
  { kind: 'open_window', width: 0.22, height: 0.26, sill: 1.35, angle: 0, span: 1 },
  { kind: 'broken_window', width: 0.2, height: 0.18, sill: 1.15, angle: 0, span: 1 },
  { kind: 'garage_gap', width: 0.28, height: 0.22, sill: 0.35, angle: 0, span: 1 },
  { kind: 'roof_hatch', width: 0.22, height: 0.22, sill: 0, angle: 0, span: 1 },
];

type Face = 'n' | 's' | 'e' | 'w';

interface FaceInfo {
  face: Face;
  normal: Vec3;
  tangent: Vec3;
  inward: [number, number];
}

const FACES: FaceInfo[] = [
  { face: 's', normal: [0, -1, 0], tangent: [1, 0, 0], inward: [0, 1] },
  { face: 'n', normal: [0, 1, 0], tangent: [1, 0, 0], inward: [0, -1] },
  { face: 'w', normal: [-1, 0, 0], tangent: [0, 1, 0], inward: [1, 0] },
  { face: 'e', normal: [1, 0, 0], tangent: [0, 1, 0], inward: [-1, 0] },
];

export function furnishShell(shell: BuildingShell, spawnX: number, spawnY: number, rng: RNG, ids: { next: number }): boolean {
  const floor = shell.spec.floors[0]!;
  const centerX = shell.origin[0] + (floor.w * shell.spec.cellSize) / 2;
  const centerY = shell.origin[1] + (floor.d * shell.spec.cellSize) / 2;
  const toward = faceToward(spawnX - centerX, spawnY - centerY);
  const order = [toward, ...FACES.map((face) => face.face).filter((face) => face !== toward)];
  const used = new Set<string>();
  const recipe = PASSABLE[(shell.id + shell.spec.seed) % PASSABLE.length]!;
  const passable = placeRecipe(shell, recipe, true, order, used, rng, ids);
  if (!passable) return false;
  let hatches = recipe.kind === 'roof_hatch' ? 1 : 0;
  const decoyCount = 3;
  for (let n = 0; n < decoyCount; n++) {
    const decoy = DECOYS[(rng.int(DECOYS.length) + n) % DECOYS.length]!;
    if (decoy.kind === 'roof_hatch') {
      if (hatches > 0) continue;
      hatches += 1;
    }
    placeRecipe(shell, decoy, false, rotate(order, n + 1), used, rng, ids);
  }
  const entrance = shell.openings.find((opening) => opening.passable);
  if (!entrance) return false;
  shell.approach = approachPoint(shell, entrance, spawnX, spawnY);
  return connectedToSpawn(shell.spec, entrance);
}

function placeRecipe(
  shell: BuildingShell,
  recipe: Recipe,
  wantPassable: boolean,
  faceOrder: Face[],
  used: Set<string>,
  rng: RNG,
  ids: { next: number },
): Opening | null {
  const clearance = measureClearance(recipe);
  if (clearance.passable !== wantPassable) return null;
  if (recipe.kind === 'roof_hatch') return placeHatch(shell, recipe, clearance, wantPassable, used, ids);
  for (const faceName of faceOrder) {
    const face = FACES.find((item) => item.face === faceName)!;
    const opening = placeOnFace(shell, recipe, clearance, face, used, rng, ids);
    if (opening) return opening;
  }
  return null;
}

function placeOnFace(
  shell: BuildingShell,
  recipe: Recipe,
  clearance: ReturnType<typeof measureClearance>,
  face: FaceInfo,
  used: Set<string>,
  rng: RNG,
  ids: { next: number },
): Opening | null {
  const grid = shell.spec.floors[0]!;
  const cells = faceCells(face.face, grid.w, grid.d);
  const bias = (cells.length - 1) / 2 + (recipe.span > 1 ? 0 : rng.range(-1.5, 1.5));
  const ordered = takeRuns(cells, recipe.span, used, bias);
  for (const run of ordered) {
    const opening = commitRun(shell, grid, run, face, recipe, clearance, used, ids);
    if (opening) return opening;
  }
  return null;
}

function commitRun(
  shell: BuildingShell,
  grid: FloorGrid,
  run: GridCell[],
  face: FaceInfo,
  recipe: Recipe,
  clearance: ReturnType<typeof measureClearance>,
  used: Set<string>,
  ids: { next: number },
): Opening | null {
  const backup = grid.cells.slice();
  const carved: GridCell[] = [];
  for (const cell of run) {
    const added = carveTunnel(grid, cell, face.inward);
    if (added.length === 0) {
      grid.cells.set(backup);
      return null;
    }
    carved.push(...added);
  }
  if (!reachesSpawn(shell.spec, 0, carved)) {
    grid.cells.set(backup);
    return null;
  }
  for (const cell of carved) used.add(`${cell.x},${cell.z}`);
  const center = apertureCenter(shell, run, 0, recipe.sill, recipe.height);
  const opening: Opening = {
    id: ids.next++,
    buildingId: shell.id,
    kind: recipe.kind,
    center,
    normal: face.normal,
    tangent: face.tangent,
    width: recipe.width,
    height: recipe.height,
    angle: recipe.angle,
    sill: recipe.sill,
    floor: 0,
    passable: clearance.passable,
    decoy: !clearance.passable,
    clearWidth: clearance.clearWidth,
    clearHeight: clearance.clearHeight,
    apertureCells: run,
    carvedCells: carved,
  };
  shell.openings.push(opening);
  return opening;
}

function placeHatch(
  shell: BuildingShell,
  recipe: Recipe,
  clearance: ReturnType<typeof measureClearance>,
  wantPassable: boolean,
  used: Set<string>,
  ids: { next: number },
): Opening | null {
  const floorIndex = shell.spec.floors.length - 1;
  const grid = shell.spec.floors[floorIndex]!;
  const cell = findDeckCell(grid, used, wantPassable);
  if (!cell) return null;
  if (!reachesSpawn(shell.spec, floorIndex, [cell])) return null;
  used.add(`hatch:${cell.x},${cell.z}`);
  const center: Vec3 = [
    shell.origin[0] + (cell.x + 0.5) * CELL_SIZE,
    shell.origin[1] + (cell.z + 0.5) * CELL_SIZE,
    shell.origin[2] + shell.spec.floors.length * FLOOR_HEIGHT,
  ];
  const opening: Opening = {
    id: ids.next++,
    buildingId: shell.id,
    kind: 'roof_hatch',
    center,
    normal: [0, 0, 1],
    tangent: [1, 0, 0],
    width: recipe.width,
    height: recipe.height,
    angle: 0,
    sill: 0,
    floor: floorIndex,
    passable: clearance.passable,
    decoy: !clearance.passable,
    clearWidth: clearance.clearWidth,
    clearHeight: clearance.clearHeight,
    apertureCells: [cell],
    carvedCells: [cell],
  };
  shell.openings.push(opening);
  return opening;
}

function findDeckCell(grid: FloorGrid, used: Set<string>, preferCenter: boolean): GridCell | null {
  const cx = Math.floor(grid.w / 2);
  const cz = Math.floor(grid.d / 2);
  let best: GridCell | null = null;
  let bestScore = preferCenter ? Infinity : -1;
  for (let z = 1; z < grid.d - 1; z++) {
    for (let x = 1; x < grid.w - 1; x++) {
      if (used.has(`hatch:${x},${z}`)) continue;
      const value = grid.cells[z * grid.w + x]!;
      if (value !== CELL.EMPTY && value !== CELL.DOOR) continue;
      const score = Math.abs(x - cx) + Math.abs(z - cz);
      if (preferCenter ? score < bestScore : score > bestScore) {
        bestScore = score;
        best = { x, z };
      }
    }
  }
  return best;
}

function carveTunnel(grid: FloorGrid, start: GridCell, inward: [number, number]): GridCell[] {
  const carved: GridCell[] = [];
  let x = start.x;
  let z = start.z;
  for (let step = 0; step < 5; step++) {
    if (x <= 0 && inward[0] < 0) break;
    if (z <= 0 && inward[1] < 0) break;
    if (x >= grid.w - 1 && inward[0] > 0) break;
    if (z >= grid.d - 1 && inward[1] > 0) break;
    if (x < 0 || z < 0 || x >= grid.w || z >= grid.d) break;
    const index = z * grid.w + x;
    const value = grid.cells[index]!;
    if (value === CELL.EMPTY || value === CELL.DOOR || value === CELL.SHAFT) break;
    grid.cells[index] = CELL.EMPTY;
    carved.push({ x, z });
    x += inward[0];
    z += inward[1];
  }
  return carved;
}

function reachesSpawn(spec: BuildingSpec, floorIndex: number, seeds: readonly GridCell[]): boolean {
  const sx = Math.floor(spec.spawn[0] / spec.cellSize);
  const sz = Math.floor(spec.spawn[1] / spec.cellSize);
  const masks = spec.floors.map((floor) => new Uint8Array(floor.w * floor.d));
  const queue: number[] = [];
  for (const seed of seeds) {
    const floor = spec.floors[floorIndex];
    if (!floor) return false;
    if (seed.x < 0 || seed.z < 0 || seed.x >= floor.w || seed.z >= floor.d) continue;
    const index = seed.z * floor.w + seed.x;
    if (!walkable(floor.cells[index]!)) continue;
    if (masks[floorIndex]![index]) continue;
    masks[floorIndex]![index] = 1;
    queue.push(floorIndex, seed.x, seed.z);
  }
  const dirs = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ];
  for (let head = 0; head < queue.length; head += 3) {
    const f = queue[head]!;
    const x = queue[head + 1]!;
    const z = queue[head + 2]!;
    if (f === 0 && x === sx && z === sz) return true;
    const floor = spec.floors[f]!;
    for (const [dx, dz] of dirs) {
      const nx = x + dx;
      const nz = z + dz;
      if (nx < 0 || nz < 0 || nx >= floor.w || nz >= floor.d) continue;
      const ni = nz * floor.w + nx;
      if (masks[f]![ni] || !walkable(floor.cells[ni]!)) continue;
      masks[f]![ni] = 1;
      queue.push(f, nx, nz);
    }
    if (floor.cells[z * floor.w + x] === CELL.SHAFT) {
      for (const nf of [f + 1, f - 1]) {
        if (nf < 0 || nf >= spec.floors.length) continue;
        const other = spec.floors[nf]!;
        const oi = z * other.w + x;
        if (masks[nf]![oi] || other.cells[oi] !== CELL.SHAFT) continue;
        masks[nf]![oi] = 1;
        queue.push(nf, x, z);
      }
    }
  }
  return false;
}

function connectedToSpawn(spec: BuildingSpec, opening: Opening): boolean {
  return reachesSpawn(spec, opening.floor, opening.carvedCells);
}

function approachPoint(shell: BuildingShell, opening: Opening, spawnX: number, spawnY: number): Vec3 {
  if (opening.kind !== 'roof_hatch') return add(opening.center, scale(opening.normal, 1.2));
  const floor = shell.spec.floors[0]!;
  const cx = shell.origin[0] + (floor.w * shell.spec.cellSize) / 2;
  const cy = shell.origin[1] + (floor.d * shell.spec.cellSize) / 2;
  const dx = spawnX - cx;
  const dy = spawnY - cy;
  const len = Math.hypot(dx, dy) || 1;
  const half = Math.max(floor.w, floor.d) * shell.spec.cellSize * 0.5 + 1.4;
  return [cx + (dx / len) * half, cy + (dy / len) * half, shell.origin[2] + 1.4];
}

function apertureCenter(shell: BuildingShell, cells: readonly GridCell[], floor: number, sill: number, height: number): Vec3 {
  let x = 0;
  let y = 0;
  for (const cell of cells) {
    x += shell.origin[0] + (cell.x + 0.5) * CELL_SIZE;
    y += shell.origin[1] + (cell.z + 0.5) * CELL_SIZE;
  }
  x /= cells.length;
  y /= cells.length;
  return [x, y, shell.origin[2] + floor * FLOOR_HEIGHT + sill + height / 2];
}

function faceCells(face: Face, w: number, d: number): GridCell[] {
  const cells: GridCell[] = [];
  if (face === 's' || face === 'n') {
    const z = face === 's' ? 0 : d - 1;
    for (let x = 1; x < w - 1; x++) cells.push({ x, z });
  } else {
    const x = face === 'w' ? 0 : w - 1;
    for (let z = 1; z < d - 1; z++) cells.push({ x, z });
  }
  return cells;
}

function takeRuns(cells: readonly GridCell[], span: number, used: Set<string>, bias: number): GridCell[][] {
  const runs: { run: GridCell[]; dist: number }[] = [];
  for (let i = 0; i + span <= cells.length; i++) {
    const run = cells.slice(i, i + span);
    if (run.some((cell) => used.has(`${cell.x},${cell.z}`))) continue;
    runs.push({ run, dist: Math.abs(i + (span - 1) / 2 - bias) });
  }
  runs.sort((a, b) => a.dist - b.dist);
  return runs.map((item) => item.run);
}

function faceToward(dx: number, dy: number): Face {
  if (Math.abs(dx) > Math.abs(dy)) return dx >= 0 ? 'e' : 'w';
  return dy >= 0 ? 'n' : 's';
}

function rotate(order: Face[], shift: number): Face[] {
  const n = order.length;
  return order.map((_, index) => order[(index + shift) % n]!);
}

function walkable(cell: number): boolean {
  return cell === CELL.EMPTY || cell === CELL.DOOR || cell === CELL.SHAFT;
}

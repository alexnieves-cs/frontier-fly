import { generateBuilding } from '../gen/building';
import { planFurniture, withoutProps } from '../gen/furniture';
import { computeFlightMap, isFullyReachable, isWalkable } from '../gen/validate';
import { CELL, CELL_SIZE, FLOOR_HEIGHT, SURFACE } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec, FloorGrid, Room } from '../shared/types';

/** Quadrant side, meters: each quadrant is one stage-3 building (rooms, hallways, loops). */
const Q = 20;

/**
 * A 40 x 40 m single-floor building made of four stage-3 floor plans in a
 * 2 x 2 grid, joined by doorways through the walls between them (three joins
 * keep it connected, and a fourth closes a loop on half the seeds).
 *
 * The stage-3 generator itself stops at eight rooms, so asking it for 40 m
 * gives a few huge halls wider than the lidar's 8 m range; tiling keeps the
 * room scale of the hard bench and makes the building four times larger.
 */
export function generateLargeBuilding(seed: number): BuildingSpec {
  const rng = new RNG(seed, 'large');
  for (let attempt = 0; attempt < 16; attempt++) {
    const spec = tryLarge(rng.fork(`attempt-${attempt}`));
    if (spec) return spec;
  }
  throw new Error(`generateLargeBuilding failed for seed ${seed}`);
}

function tryLarge(rng: RNG): BuildingSpec | null {
  const W = Q * 2;
  const cells = new Uint8Array(W * W).fill(CELL.WALL);
  const surfaces = new Uint8Array(W * W);
  const rooms: Room[] = [];
  let spawn: [number, number, number] = [0, 0, 1];
  for (let q = 0; q < 4; q++) {
    const part = generateBuilding({ seed: rng.int(1_000_000_000), floors: 1, w: Q, d: Q, stage: 3 });
    const f = part.floors[0]!;
    const ox = (q % 2) * Q;
    const oz = Math.floor(q / 2) * Q;
    for (let z = 0; z < Q; z++) {
      for (let x = 0; x < Q; x++) {
        const i = (oz + z) * W + ox + x;
        cells[i] = f.cells[z * Q + x]!;
        surfaces[i] = f.surfaces?.[z * Q + x] ?? SURFACE.DRYWALL;
      }
    }
    for (const r of f.rooms) rooms.push({ id: rooms.length, x: r.x + ox, z: r.z + oz, w: r.w, d: r.d });
    // The team enters through quadrant 0, at its own spawn.
    if (q === 0) spawn = [part.spawn[0], part.spawn[1], part.spawn[2]];
  }
  // Joins: 0-1 and 2-3 across x = 19|20, 0-2 and 1-3 across z = 19|20.
  const joins: [boolean, number][] = [
    [true, 0],
    [true, Q],
    [false, 0],
    [false, Q],
  ];
  const skip = rng.int(4);
  const loop = rng.bool(0.5);
  for (let j = 0; j < 4; j++) {
    if (j === skip && !loop) continue;
    const [acrossX, offset] = joins[j]!;
    if (!carveJoin(cells, surfaces, W, acrossX, offset, rng)) return null;
  }
  const floor: FloorGrid = { w: W, d: W, cells, rooms, surfaces };
  const spec: BuildingSpec = { seed: rng.int(1_000_000_000), cellSize: CELL_SIZE, floorHeight: FLOOR_HEIGHT, floors: [floor], spawn, reachableVoxels: 0 };
  if (!isFullyReachable(spec, computeFlightMap(spec))) return null;
  let furniture = planFurniture(spec);
  for (;;) {
    spec.furniture = furniture;
    const map = computeFlightMap(spec);
    if (isFullyReachable(spec, map)) {
      spec.reachableVoxels = map.count;
      return spec;
    }
    furniture = withoutProps(spec, furniture, new Set([furniture.props.length - 1]));
  }
}

/**
 * Cut a two-cell-wide doorway through the walls between two quadrants, along
 * two adjacent rows where both sides reach a walkable cell within a few cells
 * of the seam. `acrossX`: the seam runs along z between x = 19 and 20, over
 * rows offset..offset+19; otherwise it runs along x between z = 19 and 20.
 */
function carveJoin(cells: Uint8Array, surfaces: Uint8Array, W: number, acrossX: boolean, offset: number, rng: RNG): boolean {
  const at = (a: number, b: number) => (acrossX ? b * W + a : a * W + b);
  /** [first, last] wall cells to cut in row r, or null when a side has no walkable cell within reach. */
  const span = (r: number): [number, number] | null => {
    let lo = -1;
    for (let a = Q - 1; a >= Q - 4; a--) {
      if (isWalkable(cells[at(a, r)]!)) {
        lo = a + 1;
        break;
      }
    }
    let hi = -1;
    for (let a = Q; a <= Q + 3; a++) {
      if (isWalkable(cells[at(a, r)]!)) {
        hi = a - 1;
        break;
      }
    }
    return lo < 0 || hi < 0 ? null : [lo, hi];
  };
  const candidates: number[] = [];
  for (let r = offset + 2; r < offset + Q - 3; r++) if (span(r) && span(r + 1)) candidates.push(r);
  if (!candidates.length) return false;
  const r0 = candidates[rng.int(candidates.length)]!;
  for (const r of [r0, r0 + 1]) {
    const [lo, hi] = span(r)!;
    for (let a = lo; a <= hi; a++) {
      cells[at(a, r)] = CELL.DOOR;
      surfaces[at(a, r)] = SURFACE.WOOD;
    }
  }
  return true;
}

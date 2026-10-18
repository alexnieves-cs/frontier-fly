import { CELL, SURFACE } from '../shared/constants';
import type { RNG } from '../shared/rng';
import type { FloorGrid } from '../shared/types';

function walkable(cell: number): boolean {
  return cell === CELL.EMPTY || cell === CELL.DOOR || cell === CELL.SHAFT;
}

export function sharedWalkable(a: FloorGrid, b: FloorGrid): number {
  let n = 0;
  const w = Math.min(a.w, b.w);
  const d = Math.min(a.d, b.d);
  for (let z = 1; z < d - 1; z++) {
    for (let x = 1; x < w - 1; x++) {
      if (walkable(a.cells[z * a.w + x]!) && walkable(b.cells[z * b.w + x]!)) n += 1;
    }
  }
  return n;
}

export function cloneFloor(floor: FloorGrid): FloorGrid {
  return {
    w: floor.w,
    d: floor.d,
    cells: new Uint8Array(floor.cells),
    rooms: floor.rooms.map((room) => ({ ...room })),
    surfaces: floor.surfaces ? new Uint8Array(floor.surfaces) : undefined,
  };
}

/** Cut 1–2 shaft columns through cells that are open on both floors of each pair. */
export function carveShafts(floors: FloorGrid[], rng: RNG, avoid: [number, number] | null): boolean {
  if (floors.length < 2) return true;
  let ok = true;
  for (let f = 0; f < floors.length - 1; f++) {
    const a = floors[f]!;
    const b = floors[f + 1]!;
    const cands: [number, number][] = [];
    for (let z = 2; z < a.d - 2; z++) {
      for (let x = 2; x < a.w - 2; x++) {
        if (!walkable(a.cells[z * a.w + x]!) || !walkable(b.cells[z * b.w + x]!)) continue;
        if (avoid && f === 0 && x === avoid[0] && z === avoid[1]) continue;
        cands.push([x, z]);
      }
    }
    for (let i = cands.length - 1; i > 0; i--) {
      const j = rng.int(i + 1);
      const tmp = cands[i]!;
      cands[i] = cands[j]!;
      cands[j] = tmp;
    }
    const want = 1 + (rng.bool() ? 1 : 0);
    const placed: [number, number][] = [];
    for (const cand of cands) {
      if (placed.length >= want) break;
      if (placed.some((p) => Math.abs(p[0] - cand[0]) + Math.abs(p[1] - cand[1]) < 4)) continue;
      placed.push(cand);
      for (const floor of [a, b]) {
        const i = cand[1] * floor.w + cand[0];
        floor.cells[i] = CELL.SHAFT;
        if (floor.surfaces) floor.surfaces[i] = SURFACE.METAL;
      }
    }
    if (placed.length === 0) ok = false;
  }
  return ok;
}

import { CELL, SURFACE } from '../shared/constants';
import type { BuildingSpec } from '../shared/types';

export interface Aabb {
  cx: number;
  cy: number;
  cz: number;
  hx: number;
  hy: number;
  hz: number;
  surface: number;
  kind: 'wall' | 'window' | 'slab' | 'door';
}

interface Rect {
  x: number;
  z: number;
  w: number;
  d: number;
}

function merge(mask: Uint8Array, w: number, d: number): Rect[] {
  const used = new Uint8Array(w * d);
  const rects: Rect[] = [];
  for (let z = 0; z < d; z++) {
    for (let x = 0; x < w; x++) {
      const i = z * w + x;
      if (!mask[i] || used[i]) continue;
      let rw = 1;
      while (x + rw < w && mask[z * w + x + rw] && !used[z * w + x + rw]) rw += 1;
      let rd = 1;
      outer: while (z + rd < d) {
        for (let k = 0; k < rw; k++) {
          if (!mask[(z + rd) * w + x + k] || used[(z + rd) * w + x + k]) break outer;
        }
        rd += 1;
      }
      for (let zz = 0; zz < rd; zz++) {
        for (let xx = 0; xx < rw; xx++) used[(z + zz) * w + x + xx] = 1;
      }
      rects.push({ x, z, w: rw, d: rd });
    }
  }
  return rects;
}

/** Merged cuboids for colliders and the renderer. Windows stay blocking. */
export function buildingAabbs(spec: BuildingSpec): Aabb[] {
  const out: Aabb[] = [];
  const cs = spec.cellSize;
  const H = spec.floorHeight;
  for (let f = 0; f < spec.floors.length; f++) {
    const floor = spec.floors[f]!;
    for (const surface of [SURFACE.DRYWALL, SURFACE.CONCRETE, SURFACE.GLASS, SURFACE.METAL]) {
      const mask = new Uint8Array(floor.w * floor.d);
      for (let i = 0; i < mask.length; i++) {
        if (floor.cells[i] !== CELL.WALL) continue;
        const s = floor.surfaces?.[i] ?? SURFACE.DRYWALL;
        if (s === surface) mask[i] = 1;
      }
      for (const rect of merge(mask, floor.w, floor.d)) {
        out.push({
          cx: (rect.x + rect.w / 2) * cs,
          cy: (rect.z + rect.d / 2) * cs,
          cz: f * H + H / 2,
          hx: (rect.w * cs) / 2,
          hy: (rect.d * cs) / 2,
          hz: H / 2,
          surface,
          kind: surface === SURFACE.GLASS ? 'window' : 'wall',
        });
      }
    }
    const slabMask = new Uint8Array(floor.w * floor.d);
    for (let i = 0; i < slabMask.length; i++) {
      if (floor.cells[i] !== CELL.SHAFT) slabMask[i] = 1;
    }
    const slabZ = f === 0 ? 0.05 : f * H;
    const slabHz = 0.05;
    for (const rect of merge(slabMask, floor.w, floor.d)) {
      out.push({
        cx: (rect.x + rect.w / 2) * cs,
        cy: (rect.z + rect.d / 2) * cs,
        cz: f === 0 ? slabHz : slabZ,
        hx: (rect.w * cs) / 2,
        hy: (rect.d * cs) / 2,
        hz: slabHz,
        surface: SURFACE.WOOD,
        kind: 'slab',
      });
    }
  }
  const top = spec.floors[spec.floors.length - 1]!;
  const ceil = new Uint8Array(top.w * top.d).fill(1);
  for (const rect of merge(ceil, top.w, top.d)) {
    out.push({
      cx: (rect.x + rect.w / 2) * cs,
      cy: (rect.z + rect.d / 2) * cs,
      cz: spec.floors.length * H + 0.05,
      hx: (rect.w * cs) / 2,
      hy: (rect.d * cs) / 2,
      hz: 0.05,
      surface: SURFACE.CONCRETE,
      kind: 'slab',
    });
  }
  return out;
}

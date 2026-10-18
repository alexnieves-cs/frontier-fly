import { CELL } from '../shared/constants';
import type { Room } from '../shared/types';

export interface Link {
  a: number;
  b: number;
  hallway: boolean;
}

function center(room: Room): [number, number] {
  return [room.x + Math.floor(room.w / 2), room.z + Math.floor(room.d / 2)];
}

function manhattan(a: Room, b: Room): number {
  const [ax, az] = center(a);
  const [bx, bz] = center(b);
  return Math.abs(ax - bx) + Math.abs(az - bz);
}

function kruskal(n: number, edges: { a: number; b: number; w: number }[]): Link[] {
  const parent = Array.from({ length: n }, (_, i) => i);
  const find = (x: number): number => {
    let root = x;
    while (parent[root] !== root) root = parent[root]!;
    while (parent[x] !== root) {
      const next = parent[x]!;
      parent[x] = root;
      x = next;
    }
    return root;
  };
  const out: Link[] = [];
  const sorted = [...edges].sort((p, q) => p.w - q.w || p.a - q.a || p.b - q.b);
  for (const edge of sorted) {
    const pa = find(edge.a);
    const pb = find(edge.b);
    if (pa === pb) continue;
    parent[pa] = pb;
    out.push({ a: edge.a, b: edge.b, hallway: false });
  }
  return out;
}

/** MST over rooms plus extra loop edges. Hallways turn on at stage 3+. */
export function planLinks(rooms: Room[], stage: number): Link[] {
  const n = rooms.length;
  if (n <= 1) return [];
  const edges: { a: number; b: number; w: number }[] = [];
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      edges.push({ a: i, b: j, w: manhattan(rooms[i]!, rooms[j]!) });
    }
  }
  const mst = kruskal(n, edges);
  const used = new Set(mst.map((link) => `${link.a}:${link.b}`));
  const extras = stage >= 3 ? Math.max(1, Math.floor(n * 0.4)) : n >= 4 ? 1 : 0;
  const leftover = edges
    .filter((edge) => !used.has(`${edge.a}:${edge.b}`))
    .sort((p, q) => p.w - q.w || p.a - q.a || p.b - q.b);
  for (let i = 0; i < extras && i < leftover.length; i++) {
    const edge = leftover[i]!;
    mst.push({ a: edge.a, b: edge.b, hallway: false });
  }
  if (stage >= 3) {
    for (const link of mst) link.hallway = true;
  }
  return mst;
}

function stamp(cells: Uint8Array, w: number, d: number, x: number, z: number, width: number): void {
  for (let dx = 0; dx < width; dx++) {
    const xx = x + dx;
    if (xx <= 0 || z <= 0 || xx >= w - 1 || z >= d - 1) continue;
    const i = z * w + xx;
    if (cells[i] === CELL.WALL) cells[i] = CELL.EMPTY;
  }
}

function carveL(cells: Uint8Array, w: number, d: number, a: Room, b: Room, width: number): void {
  const [x0, z0] = center(a);
  const [x1, z1] = center(b);
  const stepX = Math.sign(x1 - x0);
  if (stepX !== 0) {
    for (let x = x0; x !== x1; x += stepX) stamp(cells, w, d, x, z0, width);
  }
  stamp(cells, w, d, x1, z0, width);
  const stepZ = Math.sign(z1 - z0);
  if (stepZ !== 0) {
    for (let z = z0; z !== z1; z += stepZ) stamp(cells, w, d, x1, z, width);
  }
  stamp(cells, w, d, x1, z1, width);
}

function fillGap(
  cells: Uint8Array,
  w: number,
  d: number,
  alongX: boolean,
  gap0: number,
  gap1: number,
  span0: number,
  span1: number,
  width: number,
  hallway: boolean,
): boolean {
  if (gap1 < gap0 || gap1 - gap0 > 6) return false;
  if (span1 - span0 < 1) return false;
  if (gap1 === gap0) return true;
  const wid = Math.max(1, Math.min(width, span1 - span0));
  const start = span0 + Math.floor((span1 - span0 - wid) / 2);
  const value = hallway ? CELL.EMPTY : CELL.DOOR;
  for (let s = start; s < start + wid; s++) {
    for (let g = gap0; g < gap1; g++) {
      const x = alongX ? g : s;
      const z = alongX ? s : g;
      if (x <= 0 || z <= 0 || x >= w - 1 || z >= d - 1) continue;
      cells[z * w + x] = value;
    }
  }
  return true;
}

function tryAdjacent(cells: Uint8Array, w: number, d: number, a: Room, b: Room, width: number, hallway: boolean): boolean {
  if (a.x + a.w <= b.x) {
    const z0 = Math.max(a.z, b.z);
    const z1 = Math.min(a.z + a.d, b.z + b.d);
    if (fillGap(cells, w, d, true, a.x + a.w, b.x, z0, z1, width, hallway)) return true;
  }
  if (a.z + a.d <= b.z) {
    const x0 = Math.max(a.x, b.x);
    const x1 = Math.min(a.x + a.w, b.x + b.w);
    if (fillGap(cells, w, d, false, a.z + a.d, b.z, x0, x1, width, hallway)) return true;
  }
  return false;
}

function markDoors(cells: Uint8Array, w: number, d: number, rooms: Room[]): void {
  const inRoom = new Uint8Array(w * d);
  for (const room of rooms) {
    for (let z = room.z; z < room.z + room.d; z++) {
      for (let x = room.x; x < room.x + room.w; x++) {
        if (x <= 0 || z <= 0 || x >= w - 1 || z >= d - 1) continue;
        inRoom[z * w + x] = 1;
      }
    }
  }
  for (let z = 1; z < d - 1; z++) {
    for (let x = 1; x < w - 1; x++) {
      const i = z * w + x;
      if (inRoom[i] || cells[i] === CELL.WALL) continue;
      if (inRoom[i - 1] || inRoom[i + 1] || inRoom[i - w] || inRoom[i + w]) cells[i] = CELL.DOOR;
    }
  }
}

export function applyLinks(cells: Uint8Array, w: number, d: number, rooms: Room[], links: Link[]): void {
  for (const link of links) {
    const a = rooms[link.a]!;
    const b = rooms[link.b]!;
    const width = link.hallway ? 2 : 1 + (link.a + link.b) % 2;
    if (tryAdjacent(cells, w, d, a, b, width, link.hallway)) continue;
    if (tryAdjacent(cells, w, d, b, a, width, link.hallway)) continue;
    carveL(cells, w, d, a, b, width);
  }
  markDoors(cells, w, d, rooms);
}

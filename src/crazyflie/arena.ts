import { RNG } from '../shared/rng';

/**
 * A room-scale flying arena for a Crazyflie: a rectangle of walls, optional
 * inner walls with doorways, and boxes on the floor taller than the flight
 * height. The same JSON describes a generated bench arena and the real room
 * Alex measures with a tape (docs/CRAZYFLIE.md), so the flight script and the
 * bench read one format.
 *
 * Frame: x along the first wall measured (width w), y along the second (depth
 * d), origin in that corner, meters. Yaw is counter-clockwise from +x.
 */
export interface ArenaBox {
  /** Lower-left corner and size, meters. */
  x: number;
  y: number;
  w: number;
  d: number;
}

export interface ArenaWall {
  ax: number;
  ay: number;
  bx: number;
  by: number;
}

export interface Arena {
  name: string;
  w: number;
  d: number;
  /** Inner walls (segments). The four outer walls are implied by w and d. */
  walls: ArenaWall[];
  boxes: ArenaBox[];
  /** Take-off point and heading. On the real drone this is where the flow estimate's origin is. */
  start: { x: number; y: number; yaw: number };
}

export type Segment = [number, number, number, number];

/** Every wall and box edge as a segment. */
export function arenaSegments(a: Arena): Segment[] {
  const s: Segment[] = [
    [0, 0, a.w, 0],
    [a.w, 0, a.w, a.d],
    [a.w, a.d, 0, a.d],
    [0, a.d, 0, 0],
  ];
  for (const w of a.walls) s.push([w.ax, w.ay, w.bx, w.by]);
  for (const b of a.boxes) {
    s.push([b.x, b.y, b.x + b.w, b.y]);
    s.push([b.x + b.w, b.y, b.x + b.w, b.y + b.d]);
    s.push([b.x + b.w, b.y + b.d, b.x, b.y + b.d]);
    s.push([b.x, b.y + b.d, b.x, b.y]);
  }
  return s;
}

/** Distance along the ray from (x, y) at angle `ang` to the first segment, or Infinity. */
export function castRay(segs: readonly Segment[], x: number, y: number, ang: number): number {
  const dx = Math.cos(ang);
  const dy = Math.sin(ang);
  let best = Infinity;
  for (const [ax, ay, bx, by] of segs) {
    const ex = bx - ax;
    const ey = by - ay;
    const den = dx * ey - dy * ex;
    if (Math.abs(den) < 1e-12) continue;
    const t = ((ax - x) * ey - (ay - y) * ex) / den;
    const u = ((ax - x) * dy - (ay - y) * dx) / den;
    if (t > 0 && u >= 0 && u <= 1 && t < best) best = t;
  }
  return best;
}

/** Shortest distance from (x, y) to any segment. */
export function clearance(segs: readonly Segment[], x: number, y: number): number {
  let best = Infinity;
  for (const [ax, ay, bx, by] of segs) {
    const ex = bx - ax;
    const ey = by - ay;
    const l2 = ex * ex + ey * ey;
    const t = l2 > 0 ? Math.max(0, Math.min(1, ((x - ax) * ex + (y - ay) * ey) / l2)) : 0;
    best = Math.min(best, Math.hypot(x - ax - t * ex, y - ay - t * ey));
  }
  return best;
}

export function insideBox(a: Arena, x: number, y: number): boolean {
  return a.boxes.some((b) => x >= b.x && x <= b.x + b.w && y >= b.y && y <= b.y + b.d);
}

function segmentsCross(p: Segment, q: Segment): boolean {
  const o = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number) => Math.sign((bx - ax) * (cy - ay) - (by - ay) * (cx - ax));
  return (
    o(p[0], p[1], p[2], p[3], q[0], q[1]) !== o(p[0], p[1], p[2], p[3], q[2], q[3]) &&
    o(q[0], q[1], q[2], q[3], p[0], p[1]) !== o(q[0], q[1], q[2], q[3], p[2], p[3])
  );
}

/**
 * Ground truth for coverage: centres of a `cell` grid over the arena where a
 * Crazyflie fits (`radius` clear of every wall and box), flood-filled from the
 * start. A centre is reachable when a straight hop from a reachable
 * neighbour crosses no wall.
 */
export function reachableCells(a: Arena, cell = 0.25, radius = 0.12): { nx: number; ny: number; cell: number; mask: Uint8Array; count: number } {
  const segs = arenaSegments(a);
  const nx = Math.ceil(a.w / cell);
  const ny = Math.ceil(a.d / cell);
  const fits = new Uint8Array(nx * ny);
  for (let j = 0; j < ny; j++) {
    for (let i = 0; i < nx; i++) {
      const x = (i + 0.5) * cell;
      const y = (j + 0.5) * cell;
      if (x < a.w && y < a.d && !insideBox(a, x, y) && clearance(segs, x, y) >= radius) fits[j * nx + i] = 1;
    }
  }
  const mask = new Uint8Array(nx * ny);
  let bestStart = -1;
  let bestD = Infinity;
  for (let k = 0; k < fits.length; k++) {
    if (!fits[k]) continue;
    const d = Math.hypot(((k % nx) + 0.5) * cell - a.start.x, (Math.floor(k / nx) + 0.5) * cell - a.start.y);
    if (d < bestD) {
      bestD = d;
      bestStart = k;
    }
  }
  if (bestStart < 0) return { nx, ny, cell, mask, count: 0 };
  const stack = [bestStart];
  mask[bestStart] = 1;
  let count = 1;
  while (stack.length) {
    const k = stack.pop()!;
    const i = k % nx;
    const j = Math.floor(k / nx);
    for (const [di, dj] of [
      [1, 0],
      [-1, 0],
      [0, 1],
      [0, -1],
    ] as const) {
      const ni = i + di;
      const nj = j + dj;
      if (ni < 0 || nj < 0 || ni >= nx || nj >= ny) continue;
      const nk = nj * nx + ni;
      if (mask[nk] || !fits[nk]) continue;
      const hop: Segment = [(i + 0.5) * cell, (j + 0.5) * cell, (ni + 0.5) * cell, (nj + 0.5) * cell];
      if (segs.some((s) => segmentsCross(hop, s))) continue;
      mask[nk] = 1;
      count += 1;
      stack.push(nk);
    }
  }
  return { nx, ny, cell, mask, count };
}

/**
 * A random living-room-sized arena: 3 to 6 m on a side, an inner wall with a
 * doorway half the time, and one to four boxes. Boxes keep clear of the
 * take-off spot and the doorway so every arena can be fully explored.
 */
export function generateArena(seed: number): Arena {
  const rng = new RNG(seed, 'crazyflie-arena');
  const w = Math.round(rng.range(3, 6) * 10) / 10;
  const d = Math.round(rng.range(3, 5) * 10) / 10;
  const walls: ArenaWall[] = [];
  let door: { x: number; y: number } | null = null;
  const start = { x: 0.7, y: 0.7, yaw: 0 };
  if (w >= 4 && rng.bool(0.5)) {
    // Divider across the width, a doorway (0.9-1.2 m) somewhere along it.
    const wx = Math.round(rng.range(w * 0.45, w * 0.6) * 10) / 10;
    const gap = rng.range(0.9, 1.2);
    const g0 = rng.range(0.4, d - gap - 0.4);
    if (g0 > 0) walls.push({ ax: wx, ay: 0, bx: wx, by: g0 });
    if (g0 + gap < d) walls.push({ ax: wx, ay: g0 + gap, bx: wx, by: d });
    door = { x: wx, y: g0 + gap / 2 };
  }
  const boxes: ArenaBox[] = [];
  const nBoxes = 1 + rng.int(4);
  for (let tries = 0; boxes.length < nBoxes && tries < 200; tries++) {
    const bw = rng.range(0.3, 0.8);
    const bd = rng.range(0.3, 0.8);
    const bx = rng.range(0.1, w - bw - 0.1);
    const by = rng.range(0.1, d - bd - 0.1);
    const cx = bx + bw / 2;
    const cy = by + bd / 2;
    if (Math.hypot(cx - start.x, cy - start.y) < 1.1) continue;
    if (door && Math.hypot(cx - door.x, cy - door.y) < 1.4) continue;
    if (walls.some((wl) => cx + bw / 2 + 0.6 > wl.ax && cx - bw / 2 - 0.6 < wl.ax)) continue;
    if (boxes.some((o) => bx < o.x + o.w + 0.6 && bx + bw + 0.6 > o.x && by < o.y + o.d + 0.6 && by + bd + 0.6 > o.y)) continue;
    boxes.push({ x: round2(bx), y: round2(by), w: round2(bw), d: round2(bd) });
  }
  return { name: `arena-${seed}`, w, d, walls, boxes, start };
}

function round2(v: number): number {
  return Math.round(v * 100) / 100;
}

/** Check a hand-written arena (the real room) before anything flies in it. Returns problems, empty when fine. */
export function validateArena(a: unknown): string[] {
  const errs: string[] = [];
  const o = a as Partial<Arena> | null;
  if (!o || typeof o !== 'object') return ['arena is not an object'];
  const num = (v: unknown) => typeof v === 'number' && Number.isFinite(v);
  if (!num(o.w) || o.w! < 1.5 || o.w! > 20) errs.push('w must be 1.5 to 20 m');
  if (!num(o.d) || o.d! < 1.5 || o.d! > 20) errs.push('d must be 1.5 to 20 m');
  if (!Array.isArray(o.walls)) errs.push('walls must be a list');
  if (!Array.isArray(o.boxes)) errs.push('boxes must be a list');
  const s = o.start;
  if (!s || !num(s.x) || !num(s.y) || !num(s.yaw)) errs.push('start needs x, y and yaw');
  if (errs.length) return errs;
  const arena = o as Arena;
  const segs = arenaSegments(arena);
  if (s!.x <= 0 || s!.y <= 0 || s!.x >= arena.w || s!.y >= arena.d) errs.push('start is outside the arena');
  else if (clearance(segs, s!.x, s!.y) < 0.4) errs.push('start must be at least 0.4 m from every wall and box');
  else if (insideBox(arena, s!.x, s!.y)) errs.push('start is inside a box');
  return errs;
}

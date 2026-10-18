import type { Point } from './types';

const EPS = 1e-8;

export function roundTo(n: number, digits: number): number {
  const p = 10 ** digits;
  const v = Math.round(n * p) / p;
  return Object.is(v, -0) ? 0 : v;
}

export function snapToGrid(point: Point, gridM: number): Point {
  const grid = gridM > 0 ? gridM : 1;
  return {
    x: roundTo(Math.round(point.x / grid) * grid, 3),
    y: roundTo(Math.round(point.y / grid) * grid, 3),
  };
}

export function bounds(points: readonly Point[]): { minX: number; minY: number; maxX: number; maxY: number } {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const point of points) {
    minX = Math.min(minX, point.x);
    minY = Math.min(minY, point.y);
    maxX = Math.max(maxX, point.x);
    maxY = Math.max(maxY, point.y);
  }
  if (!Number.isFinite(minX)) return { minX: 0, minY: 0, maxX: 0, maxY: 0 };
  return { minX, minY, maxX, maxY };
}

export function signedArea(points: readonly Point[]): number {
  let sum = 0;
  for (let i = 0; i < points.length; i++) {
    const a = points[i]!;
    const b = points[(i + 1) % points.length]!;
    sum += a.x * b.y - b.x * a.y;
  }
  return sum / 2;
}

export function centroid(points: readonly Point[]): Point {
  if (points.length === 0) return { x: 0, y: 0 };
  const area = signedArea(points);
  if (Math.abs(area) < EPS) {
    const box = bounds(points);
    return { x: (box.minX + box.maxX) / 2, y: (box.minY + box.maxY) / 2 };
  }
  let x = 0;
  let y = 0;
  for (let i = 0; i < points.length; i++) {
    const a = points[i]!;
    const b = points[(i + 1) % points.length]!;
    const cross = a.x * b.y - b.x * a.y;
    x += (a.x + b.x) * cross;
    y += (a.y + b.y) * cross;
  }
  const scale = 1 / (6 * area);
  return { x: x * scale, y: y * scale };
}

export function rect(x: number, y: number, w: number, h: number): Point[] {
  return [
    { x, y },
    { x: x + w, y },
    { x: x + w, y: y + h },
    { x, y: y + h },
  ];
}

function sgn(value: number): number {
  if (value > EPS) return 1;
  if (value < -EPS) return -1;
  return 0;
}

export function orient(a: Point, b: Point, c: Point): number {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/** True when the open segments cross. Endpoint touches and collinear overlaps are not crossings. */
export function properCrossing(a: Point, b: Point, c: Point, d: Point): boolean {
  const o1 = sgn(orient(a, b, c));
  const o2 = sgn(orient(a, b, d));
  const o3 = sgn(orient(c, d, a));
  const o4 = sgn(orient(c, d, b));
  return o1 * o2 < 0 && o3 * o4 < 0;
}

function overlapInterior(a: Point, b: Point, c: Point, d: Point): boolean {
  if (sgn(orient(a, b, c)) !== 0 || sgn(orient(a, b, d)) !== 0) return false;
  const useX = Math.abs(b.x - a.x) >= Math.abs(b.y - a.y);
  const a0 = useX ? a.x : a.y;
  const a1 = useX ? b.x : b.y;
  const b0 = useX ? c.x : c.y;
  const b1 = useX ? d.x : d.y;
  const lo = Math.max(Math.min(a0, a1), Math.min(b0, b1));
  const hi = Math.min(Math.max(a0, a1), Math.max(b0, b1));
  return hi - lo > 1e-6;
}

export function isSelfIntersecting(points: readonly Point[]): boolean {
  const n = points.length;
  if (n < 4) return false;
  for (let i = 0; i < n; i++) {
    const a = points[i]!;
    const b = points[(i + 1) % n]!;
    for (let j = i + 1; j < n; j++) {
      const adjacent = j === i + 1 || (i === 0 && j === n - 1);
      if (adjacent) continue;
      const c = points[j]!;
      const d = points[(j + 1) % n]!;
      if (properCrossing(a, b, c, d) || overlapInterior(a, b, c, d)) return true;
    }
  }
  return false;
}

export function distToSegment(p: Point, a: Point, b: Point): number {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const len2 = dx * dx + dy * dy;
  if (len2 < EPS) return Math.hypot(p.x - a.x, p.y - a.y);
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2));
  return Math.hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

export function pointOnRingBoundary(p: Point, ring: readonly Point[], eps = 1e-6): boolean {
  for (let i = 0; i < ring.length; i++) {
    if (distToSegment(p, ring[i]!, ring[(i + 1) % ring.length]!) <= eps) return true;
  }
  return false;
}

/** Boundary counts as inside. */
export function pointInRing(p: Point, ring: readonly Point[]): boolean {
  if (ring.length < 3) return false;
  if (pointOnRingBoundary(p, ring)) return true;
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
    const a = ring[i]!;
    const b = ring[j]!;
    const crosses = a.y > p.y !== b.y > p.y && p.x < ((b.x - a.x) * (p.y - a.y)) / (b.y - a.y) + a.x;
    if (crosses) inside = !inside;
  }
  return inside;
}

export function ringInsideRing(inner: readonly Point[], outer: readonly Point[]): boolean {
  if (inner.length < 3 || outer.length < 3) return false;
  for (const point of inner) {
    if (!pointInRing(point, outer)) return false;
  }
  for (let i = 0; i < inner.length; i++) {
    const a = inner[i]!;
    const b = inner[(i + 1) % inner.length]!;
    for (let j = 0; j < outer.length; j++) {
      const c = outer[j]!;
      const d = outer[(j + 1) % outer.length]!;
      if (properCrossing(a, b, c, d)) return false;
    }
  }
  return true;
}

export function ringsOverlap(a: readonly Point[], b: readonly Point[]): boolean {
  if (a.length < 3 || b.length < 3) return false;
  if (a.some((point) => pointInRing(point, b))) return true;
  if (b.some((point) => pointInRing(point, a))) return true;
  for (let i = 0; i < a.length; i++) {
    const a0 = a[i]!;
    const a1 = a[(i + 1) % a.length]!;
    for (let j = 0; j < b.length; j++) {
      if (properCrossing(a0, a1, b[j]!, b[(j + 1) % b.length]!)) return true;
    }
  }
  return false;
}

export function hasDuplicateVertex(points: readonly Point[], minSeparation = 0.05): boolean {
  for (let i = 0; i < points.length; i++) {
    for (let j = i + 1; j < points.length; j++) {
      if (Math.hypot(points[i]!.x - points[j]!.x, points[i]!.y - points[j]!.y) < minSeparation) return true;
    }
  }
  return false;
}

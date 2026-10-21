/** Small polygon helpers for flat rings [x0, y0, x1, y1, ...]. */

export interface Box {
  x0: number;
  y0: number;
  x1: number;
  y1: number;
}

export function ringBox(ring: number[]): Box {
  let x0 = Infinity;
  let y0 = Infinity;
  let x1 = -Infinity;
  let y1 = -Infinity;
  for (let i = 0; i < ring.length; i += 2) {
    const x = ring[i]!;
    const y = ring[i + 1]!;
    if (x < x0) x0 = x;
    if (x > x1) x1 = x;
    if (y < y0) y0 = y;
    if (y > y1) y1 = y;
  }
  return { x0, y0, x1, y1 };
}

export function inBox(b: Box, x: number, y: number, pad = 0): boolean {
  return x >= b.x0 - pad && x <= b.x1 + pad && y >= b.y0 - pad && y <= b.y1 + pad;
}

/** Even-odd point in polygon. */
export function inRing(ring: number[], x: number, y: number): boolean {
  let inside = false;
  const n = ring.length / 2;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    const xi = ring[i * 2]!;
    const yi = ring[i * 2 + 1]!;
    const xj = ring[j * 2]!;
    const yj = ring[j * 2 + 1]!;
    if (yi > y !== yj > y && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) inside = !inside;
  }
  return inside;
}

/** Shoelace area (absolute). */
export function ringArea(ring: number[]): number {
  let a = 0;
  const n = ring.length / 2;
  for (let i = 0, j = n - 1; i < n; j = i++) {
    a += ring[j * 2]! * ring[i * 2 + 1]! - ring[i * 2]! * ring[j * 2 + 1]!;
  }
  return Math.abs(a) / 2;
}

export function rotateRing(ring: number[], angle: number): number[] {
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const out = new Array<number>(ring.length);
  for (let i = 0; i < ring.length; i += 2) {
    const x = ring[i]!;
    const y = ring[i + 1]!;
    out[i] = x * c - y * s;
    out[i + 1] = x * s + y * c;
  }
  return out;
}

/**
 * Rotation (radians) that best aligns the rings' edges with the axes: the
 * length-weighted mode of edge directions folded into [0, π/2), in 0.5° bins,
 * refined by the weighted mean inside the winning bin's neighbourhood.
 */
export function dominantRotation(rings: number[][]): number {
  const BINS = 180;
  const hist = new Float64Array(BINS);
  const edges: { a: number; w: number }[] = [];
  const quarter = Math.PI / 2;
  for (const ring of rings) {
    const n = ring.length / 2;
    for (let i = 0, j = n - 1; i < n; j = i++) {
      const dx = ring[i * 2]! - ring[j * 2]!;
      const dy = ring[i * 2 + 1]! - ring[j * 2 + 1]!;
      const w = Math.hypot(dx, dy);
      if (w < 1e-6) continue;
      let a = Math.atan2(dy, dx) % quarter;
      if (a < 0) a += quarter;
      edges.push({ a, w });
      hist[Math.min(BINS - 1, Math.floor((a / quarter) * BINS))]! += w;
    }
  }
  let best = 0;
  for (let b = 1; b < BINS; b++) if (hist[b]! > hist[best]!) best = b;
  const center = ((best + 0.5) / BINS) * quarter;
  let sum = 0;
  let wsum = 0;
  for (const { a, w } of edges) {
    // Wrap-aware offset from the bin center, within ±1°.
    let d = a - center;
    if (d > quarter / 2) d -= quarter;
    if (d < -quarter / 2) d += quarter;
    if (Math.abs(d) > Math.PI / 180) continue;
    sum += d * w;
    wsum += w;
  }
  const angle = center + (wsum > 0 ? sum / wsum : 0);
  return -angle;
}

/** Share of edge length within `tol` radians of an axis. */
export function axisAlignedShare(rings: number[][], tol = (2 * Math.PI) / 180): number {
  let on = 0;
  let total = 0;
  const quarter = Math.PI / 2;
  for (const ring of rings) {
    const n = ring.length / 2;
    for (let i = 0, j = n - 1; i < n; j = i++) {
      const dx = ring[i * 2]! - ring[j * 2]!;
      const dy = ring[i * 2 + 1]! - ring[j * 2 + 1]!;
      const w = Math.hypot(dx, dy);
      let a = Math.atan2(dy, dx) % quarter;
      if (a < 0) a += quarter;
      if (a < tol || quarter - a < tol) on += w;
      total += w;
    }
  }
  return total > 0 ? on / total : 1;
}

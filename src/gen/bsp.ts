import type { RNG } from '../shared/rng';

export interface Rect {
  x: number;
  z: number;
  w: number;
  d: number;
}

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v));
}

function splitOne(rng: RNG, rect: Rect, minSize: number): [Rect, Rect] | null {
  const canW = rect.w >= minSize * 2;
  const canD = rect.d >= minSize * 2;
  if (!canW && !canD) return null;
  const vertical = canW && (!canD || rng.bool());
  if (vertical) {
    const cut = clamp(Math.round(rect.w * rng.range(0.3, 0.7)), minSize, rect.w - minSize);
    return [
      { x: rect.x, z: rect.z, w: cut, d: rect.d },
      { x: rect.x + cut, z: rect.z, w: rect.w - cut, d: rect.d },
    ];
  }
  const cut = clamp(Math.round(rect.d * rng.range(0.3, 0.7)), minSize, rect.d - minSize);
  return [
    { x: rect.x, z: rect.z, w: rect.w, d: cut },
    { x: rect.x, z: rect.z + cut, w: rect.w, d: rect.d - cut },
  ];
}

/** BSP-split `root` into room leaves. Split ratio is always in 0.3–0.7. */
export function splitRooms(rng: RNG, root: Rect, stage: number): Rect[] {
  const minSize = stage >= 3 ? 6 : 5;
  const maxRooms = stage === 2 ? 4 : 8;
  const minRooms = 2;
  const rects: Rect[] = [{ ...root }];
  let guard = 0;
  while (rects.length < maxRooms && guard < 40) {
    guard += 1;
    let best = -1;
    let bestArea = -1;
    for (let i = 0; i < rects.length; i++) {
      const rect = rects[i]!;
      if ((rect.w >= minSize * 2 || rect.d >= minSize * 2) && rect.w * rect.d > bestArea) {
        best = i;
        bestArea = rect.w * rect.d;
      }
    }
    if (best < 0) break;
    const parts = splitOne(rng, rects[best]!, minSize);
    if (!parts) break;
    rects.splice(best, 1, parts[0], parts[1]);
    if (rects.length >= minRooms && rng.bool(0.28)) break;
  }
  return rects;
}

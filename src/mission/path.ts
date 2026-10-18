import { bounds, pointInRing, roundTo } from './polygon';
import type { Mission, Point } from './types';

const SWATH_M = 8;
const STEP_M = 2;

export function isOperable(point: Point, mission: Mission): boolean {
  if (!pointInRing(point, mission.geofence.points)) return false;
  return !mission.noFlyZones.some((zone) => pointInRing(point, zone.polygon.points));
}

function segmentOperable(a: Point, b: Point, mission: Mission): boolean {
  const steps = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y)));
  for (let i = 0; i <= steps; i++) {
    const t = i / steps;
    if (!isOperable({ x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t }, mission)) return false;
  }
  return true;
}

function dedupe(points: Point[]): Point[] {
  const out: Point[] = [];
  for (const point of points) {
    const prev = out[out.length - 1];
    if (!prev || Math.hypot(prev.x - point.x, prev.y - point.y) > 0.05) out.push(point);
  }
  return out;
}

function chainOperable(chain: Point[], mission: Mission): boolean {
  for (let i = 0; i < chain.length - 1; i++) {
    if (!segmentOperable(chain[i]!, chain[i + 1]!, mission)) return false;
  }
  return true;
}

function appendConnector(points: Point[], to: Point, mission: Mission): void {
  const from = points[points.length - 1];
  if (!from) {
    points.push(to);
    return;
  }
  if (Math.hypot(from.x - to.x, from.y - to.y) < 0.4) return;
  const vias: Point[][] = [
    [],
    [{ x: from.x, y: to.y }],
    [{ x: to.x, y: from.y }],
  ];
  const pad = 2;
  for (const zone of mission.noFlyZones) {
    const box = bounds(zone.polygon.points);
    const south = roundTo(box.minY - pad, 3);
    const north = roundTo(box.maxY + pad, 3);
    const west = roundTo(box.minX - pad, 3);
    const east = roundTo(box.maxX + pad, 3);
    vias.push(
      [
        { x: from.x, y: south },
        { x: to.x, y: south },
      ],
      [
        { x: from.x, y: north },
        { x: to.x, y: north },
      ],
      [
        { x: west, y: from.y },
        { x: west, y: to.y },
      ],
      [
        { x: east, y: from.y },
        { x: east, y: to.y },
      ],
    );
  }
  const site = bounds(mission.geofence.points);
  const margin = roundTo(site.minX + 3, 3);
  vias.push([
    { x: margin, y: from.y },
    { x: margin, y: to.y },
  ]);
  for (const via of vias) {
    if (chainOperable([from, ...via, to], mission)) {
      points.push(...via, to);
      return;
    }
  }
  points.push(to);
}

/** Lawnmower preview inside the geofence and outside no-fly zones. Starts at the start pose. */
export function planPath(mission: Mission): Point[] {
  const start = { x: mission.startPose.x, y: mission.startPose.y };
  if (mission.geofence.points.length < 3) return [start];
  const box = bounds(mission.geofence.points);
  const lanes: Point[][] = [];
  for (let y = box.minY + SWATH_M / 2; y < box.maxY; y += SWATH_M) {
    let row: Point[] = [];
    const flush = (): void => {
      if (row.length >= 2) lanes.push(row);
      row = [];
    };
    for (let x = box.minX; x <= box.maxX + 0.01; x += STEP_M) {
      const point = { x: roundTo(x, 3), y: roundTo(y, 3) };
      if (isOperable(point, mission)) row.push(point);
      else flush();
    }
    flush();
  }
  const points: Point[] = [start];
  for (const row of lanes) {
    const last = points[points.length - 1] ?? start;
    const head = row[0]!;
    const tail = row[row.length - 1]!;
    const forward = Math.hypot(last.x - head.x, last.y - head.y) <= Math.hypot(last.x - tail.x, last.y - tail.y);
    const from = forward ? head : tail;
    const to = forward ? tail : head;
    appendConnector(points, from, mission);
    points.push(to);
  }
  return dedupe(points);
}

export function pathLength(path: readonly Point[]): number {
  let total = 0;
  for (let i = 0; i < path.length - 1; i++) {
    total += Math.hypot(path[i + 1]!.x - path[i]!.x, path[i + 1]!.y - path[i]!.y);
  }
  return total;
}

export function pointAtDistance(path: readonly Point[], distance: number): { x: number; y: number; yawRad: number } {
  const first = path[0];
  if (!first) return { x: 0, y: 0, yawRad: 0 };
  if (path.length === 1) return { x: first.x, y: first.y, yawRad: 0 };
  let remaining = Math.max(0, distance);
  for (let i = 0; i < path.length - 1; i++) {
    const a = path[i]!;
    const b = path[i + 1]!;
    const len = Math.hypot(b.x - a.x, b.y - a.y);
    const yawRad = Math.atan2(b.y - a.y, b.x - a.x);
    if (remaining <= len || i === path.length - 2) {
      const t = len < 1e-6 ? 0 : Math.min(1, remaining / Math.max(len, 1e-6));
      return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, yawRad };
    }
    remaining -= len;
  }
  const last = path[path.length - 1]!;
  return { x: last.x, y: last.y, yawRad: 0 };
}

export function closestAlong(path: readonly Point[], target: Point): { along: number; gap: number } {
  if (path.length === 0) return { along: 0, gap: Infinity };
  if (path.length === 1) {
    return { along: 0, gap: Math.hypot(path[0]!.x - target.x, path[0]!.y - target.y) };
  }
  let bestGap = Infinity;
  let bestAlong = 0;
  let traveled = 0;
  for (let i = 0; i < path.length - 1; i++) {
    const a = path[i]!;
    const b = path[i + 1]!;
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const len2 = dx * dx + dy * dy;
    const len = Math.sqrt(len2);
    let t = 0;
    if (len2 > 1e-9) t = Math.max(0, Math.min(1, ((target.x - a.x) * dx + (target.y - a.y) * dy) / len2));
    const x = a.x + dx * t;
    const y = a.y + dy * t;
    const gap = Math.hypot(target.x - x, target.y - y);
    if (gap < bestGap) {
      bestGap = gap;
      bestAlong = traveled + len * t;
    }
    traveled += len;
  }
  return { along: bestAlong, gap: bestGap };
}

export function pathIsOperable(mission: Mission, path: readonly Point[]): boolean {
  if (path.length === 0) return false;
  for (let i = 0; i < path.length - 1; i++) {
    if (!segmentOperable(path[i]!, path[i + 1]!, mission)) return false;
  }
  return path.every((point) => isOperable(point, mission));
}

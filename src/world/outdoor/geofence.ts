import { RNG } from '../../shared/rng';
import { distToSegment, pointInPolygon } from './math';
import type { Corridor, Geofence, NoFlyZone } from './types';

export function makeGeofence(cx: number, cy: number, radius: number, rng: RNG): Geofence {
  const count = 14;
  const polygon: [number, number][] = [];
  for (let i = 0; i < count; i++) {
    const angle = (i / count) * Math.PI * 2;
    const wobble = 1 + rng.next() * 0.07;
    polygon.push([cx + Math.cos(angle) * radius * wobble, cy + Math.sin(angle) * radius * wobble]);
  }
  return { polygon };
}

export function containsPoint(fence: Geofence, x: number, y: number): boolean {
  return pointInPolygon(x, y, fence.polygon);
}

export interface Block {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

/** Push the ring outward until every footprint corner sits inside. */
export function containBlocks(fence: Geofence, blocks: readonly Block[], cx: number, cy: number): void {
  for (let pass = 0; pass < 6; pass++) {
    let missed = false;
    for (const block of blocks) {
      const corners: [number, number][] = [
        [block.minX, block.minY],
        [block.maxX, block.minY],
        [block.maxX, block.maxY],
        [block.minX, block.maxY],
      ];
      for (const [x, y] of corners) {
        if (pointInPolygon(x, y, fence.polygon)) continue;
        missed = true;
        for (const vertex of fence.polygon) {
          vertex[0] = cx + (vertex[0] - cx) * 1.08;
          vertex[1] = cy + (vertex[1] - cy) * 1.08;
        }
        break;
      }
      if (missed) break;
    }
    if (!missed) return;
  }
}

export function makeNoFlyZones(
  cx: number,
  cy: number,
  blocks: readonly Block[],
  corridors: readonly Corridor[],
  rng: RNG,
): NoFlyZone[] {
  const zones: NoFlyZone[] = [];
  const angles = [0.4, 2.2, 3.6, 5.1].map((a) => a + rng.range(-0.15, 0.15));
  for (const angle of angles) {
    if (zones.length >= 1) break;
    const dist = 30 + rng.range(0, 2);
    const qx = cx + Math.cos(angle) * dist;
    const qy = cy + Math.sin(angle) * dist;
    const polygon = axisQuad(qx, qy, 5.5, 4.5);
    if (zoneHitsWork(polygon, blocks, corridors)) continue;
    zones.push({
      id: zones.length,
      name: 'ravine',
      polygon,
      minZ: 0,
      maxZ: 28,
    });
  }
  const high = axisQuad(cx + 24, cy - 22, 6, 5);
  zones.push({
    id: zones.length,
    name: 'ceiling',
    polygon: high,
    minZ: 14,
    maxZ: 48,
  });
  return zones;
}

export function inNoFly(zones: readonly NoFlyZone[], x: number, y: number, z: number): boolean {
  for (const zone of zones) {
    if (z < zone.minZ || z > zone.maxZ) continue;
    if (pointInPolygon(x, y, zone.polygon)) return true;
  }
  return false;
}

function axisQuad(cx: number, cy: number, hx: number, hy: number): [number, number][] {
  return [
    [cx - hx, cy - hy],
    [cx + hx, cy - hy],
    [cx + hx, cy + hy],
    [cx - hx, cy + hy],
  ];
}

function zoneHitsWork(polygon: readonly [number, number][], blocks: readonly Block[], corridors: readonly Corridor[]): boolean {
  for (const block of blocks) {
    const corners: [number, number][] = [
      [block.minX - 2, block.minY - 2],
      [block.maxX + 2, block.minY - 2],
      [block.maxX + 2, block.maxY + 2],
      [block.minX - 2, block.maxY + 2],
    ];
    for (const [x, y] of corners) if (pointInPolygon(x, y, polygon)) return true;
    for (const [x, y] of polygon) {
      if (x >= block.minX - 2 && x <= block.maxX + 2 && y >= block.minY - 2 && y <= block.maxY + 2) return true;
    }
  }
  for (const corridor of corridors) {
    for (let s = 0; s <= 8; s++) {
      const t = s / 8;
      const x = corridor.ax + (corridor.bx - corridor.ax) * t;
      const y = corridor.ay + (corridor.by - corridor.ay) * t;
      if (pointInPolygon(x, y, polygon)) return true;
    }
    for (const [x, y] of polygon) {
      if (distToSegment(x, y, corridor.ax, corridor.ay, corridor.bx, corridor.by) < corridor.halfWidth + 1.5) return true;
    }
  }
  return false;
}

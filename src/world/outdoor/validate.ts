import { measureClearance } from './clearance';
import { COLLIDER_BUDGET } from './constants';
import { shellAabb, treeHitsAabb } from './forest';
import { containsPoint, inNoFly, type Block } from './geofence';
import { pointInPolygon } from './math';
import { sampleHeight, slopeAt } from './terrain';
import type { Opening, OutdoorWorld, TreeInstance } from './types';

export interface EntrancePath {
  buildingId: number;
  openingId: number;
  steps: number;
}

export function validateOutdoor(world: OutdoorWorld): string[] {
  const errors: string[] = [];
  if (world.buildings.length < 1) errors.push('no buildings');
  if (world.colliders.length < 8) errors.push('too few colliders');
  if (world.colliders.length > COLLIDER_BUDGET) errors.push(`collider budget ${world.colliders.length}`);
  if (!containsPoint(world.geofence, world.spawn[0], world.spawn[1])) errors.push('spawn outside geofence');
  if (inNoFly(world.noFly, world.spawn[0], world.spawn[1], world.spawn[2])) errors.push('spawn in no-fly');
  const ground = sampleHeight(world.terrain, world.spawn[0], world.spawn[1]);
  if (world.spawn[2] < ground + 0.4) errors.push('spawn inside terrain');

  let minH = Infinity;
  let maxH = -Infinity;
  for (const height of world.terrain.heights) {
    if (height < minH) minH = height;
    if (height > maxH) maxH = height;
  }
  if (maxH - minH < 2) errors.push('terrain is flat');

  const decoys = world.buildings.reduce((sum, shell) => sum + shell.openings.filter((opening) => opening.decoy).length, 0);
  if (decoys < 1) errors.push('no decoy openings');

  for (const shell of world.buildings) {
    const floor = shell.spec.floors[0]!;
    const block: Block = {
      minX: shell.origin[0],
      minY: shell.origin[1],
      maxX: shell.origin[0] + floor.w * shell.spec.cellSize,
      maxY: shell.origin[1] + floor.d * shell.spec.cellSize,
    };
    const corners: [number, number][] = [
      [block.minX, block.minY],
      [block.maxX, block.minY],
      [block.maxX, block.maxY],
      [block.minX, block.maxY],
    ];
    for (const [x, y] of corners) {
      if (!pointInPolygon(x, y, world.geofence.polygon)) errors.push(`building ${shell.id} leaves the geofence`);
    }
    const passable = shell.openings.filter((opening) => opening.passable);
    if (passable.length < 1) errors.push(`building ${shell.id} has no passable opening`);
    for (const opening of shell.openings) {
      const measured = measureClearance(opening);
      if (measured.passable !== opening.passable) errors.push(`opening ${opening.id} passable flag`);
      if (Math.abs(measured.clearWidth - opening.clearWidth) > 1e-3) errors.push(`opening ${opening.id} clearance`);
      if (opening.decoy === opening.passable) errors.push(`opening ${opening.id} decoy flag`);
      if (opening.passable && (opening.clearWidth < measured.clearWidth - 1e-3 || opening.clearHeight + 1e-6 < 0)) {
        errors.push(`opening ${opening.id} size`);
      }
    }
    for (const tree of world.trees) {
      if (treeHitsAabb(tree, shellAabb(shell, 0))) errors.push(`tree intersects building ${shell.id}`);
    }
  }

  if (world.clearings.length > 1) {
    const pad = world.clearings[1]!;
    const padSlope = slopeAt(world.terrain, pad.x, pad.y);
    let wild = 0;
    let n = 0;
    for (let i = 0; i < 12; i++) {
      const ang = (i / 12) * Math.PI * 2;
      const x = world.size / 2 + Math.cos(ang) * (world.size * 0.34);
      const y = world.size / 2 + Math.sin(ang) * (world.size * 0.34);
      wild += slopeAt(world.terrain, x, y);
      n += 1;
    }
    if (padSlope > (wild / n) * 0.55 && padSlope > 0.05) errors.push('clearing is not flatter than the hills');
  }

  const paths = entrancePaths(world);
  for (const shell of world.buildings) {
    if (!paths.some((path) => path.buildingId === shell.id)) errors.push(`building ${shell.id} unreachable from spawn`);
  }
  return errors;
}

/** Outdoor grid path from the spawn to a passable opening, then the interior link. */
export function entrancePaths(world: OutdoorWorld): EntrancePath[] {
  const res = 1;
  const n = Math.ceil(world.size / res);
  const blocked = new Uint8Array(n * n);
  const index = (ix: number, iy: number) => iy * n + ix;
  const markDisk = (x: number, y: number, radius: number) => {
    const r = Math.ceil(radius);
    const ix0 = Math.max(0, Math.floor(x - r));
    const iy0 = Math.max(0, Math.floor(y - r));
    const ix1 = Math.min(n - 1, Math.ceil(x + r));
    const iy1 = Math.min(n - 1, Math.ceil(y + r));
    for (let iy = iy0; iy <= iy1; iy++) {
      for (let ix = ix0; ix <= ix1; ix++) {
        const cx = ix + 0.5;
        const cy = iy + 0.5;
        if ((cx - x) * (cx - x) + (cy - y) * (cy - y) <= radius * radius) blocked[index(ix, iy)] = 1;
      }
    }
  };
  for (const tree of world.trees) if (blocksCruise(tree)) markDisk(tree.x, tree.y, Math.max(tree.trunkRadius, tree.canopyRadius));
  for (const shell of world.buildings) {
    const box = shellAabb(shell, 0);
    const ix0 = Math.max(0, Math.floor(box.minX));
    const iy0 = Math.max(0, Math.floor(box.minY));
    const ix1 = Math.min(n - 1, Math.floor(box.maxX));
    const iy1 = Math.min(n - 1, Math.floor(box.maxY));
    for (let iy = iy0; iy <= iy1; iy++) {
      for (let ix = ix0; ix <= ix1; ix++) {
        const cx = ix + 0.5;
        const cy = iy + 0.5;
        if (cx >= box.minX && cx <= box.maxX && cy >= box.minY && cy <= box.maxY) blocked[index(ix, iy)] = 1;
      }
    }
  }
  for (let iy = 0; iy < n; iy++) {
    for (let ix = 0; ix < n; ix++) {
      const x = ix + 0.5;
      const y = iy + 0.5;
      if (!containsPoint(world.geofence, x, y)) blocked[index(ix, iy)] = 1;
      const z = sampleHeight(world.terrain, x, y) + 2;
      if (inNoFly(world.noFly, x, y, z)) blocked[index(ix, iy)] = 1;
    }
  }
  for (const shell of world.buildings) {
    for (const opening of shell.openings) {
      if (!opening.passable) continue;
      const goal = opening.kind === 'roof_hatch' ? shell.approach : mouth(opening);
      clearDisk(blocked, n, goal[0], goal[1], 1.8);
    }
  }
  clearDisk(blocked, n, world.spawn[0], world.spawn[1], 2);

  const startX = Math.max(0, Math.min(n - 1, Math.floor(world.spawn[0])));
  const startY = Math.max(0, Math.min(n - 1, Math.floor(world.spawn[1])));
  const dist = new Int16Array(n * n);
  dist.fill(-1);
  const queue: number[] = [startX, startY];
  dist[index(startX, startY)] = 0;
  blocked[index(startX, startY)] = 0;
  const dirs = [
    [1, 0],
    [-1, 0],
    [0, 1],
    [0, -1],
  ];
  for (let head = 0; head < queue.length; head += 2) {
    const x = queue[head]!;
    const y = queue[head + 1]!;
    const base = dist[index(x, y)]!;
    for (const [dx, dy] of dirs) {
      const nx = x + dx;
      const ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= n || ny >= n) continue;
      const ni = index(nx, ny);
      if (blocked[ni] || dist[ni] !== -1) continue;
      dist[ni] = base + 1;
      queue.push(nx, ny);
    }
  }

  const paths: EntrancePath[] = [];
  for (const shell of world.buildings) {
    let best: EntrancePath | null = null;
    for (const opening of shell.openings) {
      if (!opening.passable) continue;
      const goal = opening.kind === 'roof_hatch' ? shell.approach : mouth(opening);
      const gx = Math.max(0, Math.min(n - 1, Math.floor(goal[0])));
      const gy = Math.max(0, Math.min(n - 1, Math.floor(goal[1])));
      const steps = nearestDist(dist, blocked, n, gx, gy);
      if (steps < 0) continue;
      if (!best || steps < best.steps) best = { buildingId: shell.id, openingId: opening.id, steps };
    }
    if (best) paths.push(best);
  }
  return paths;
}

function mouth(opening: Opening): [number, number, number] {
  return [
    opening.center[0] + opening.normal[0] * 1.2,
    opening.center[1] + opening.normal[1] * 1.2,
    opening.center[2],
  ];
}

function blocksCruise(tree: TreeInstance): boolean {
  if (tree.kind === 'undergrowth') return false;
  const low = tree.canopyZ - tree.canopyRadius;
  const high = tree.canopyZ + tree.canopyRadius;
  return tree.trunkHeight > 0.2 || (low < 3.4 && high > 1.1);
}

function clearDisk(blocked: Uint8Array, n: number, x: number, y: number, radius: number): void {
  const r = Math.ceil(radius);
  const ix0 = Math.max(0, Math.floor(x - r));
  const iy0 = Math.max(0, Math.floor(y - r));
  const ix1 = Math.min(n - 1, Math.ceil(x + r));
  const iy1 = Math.min(n - 1, Math.ceil(y + r));
  for (let iy = iy0; iy <= iy1; iy++) {
    for (let ix = ix0; ix <= ix1; ix++) {
      const cx = ix + 0.5;
      const cy = iy + 0.5;
      if ((cx - x) * (cx - x) + (cy - y) * (cy - y) <= radius * radius) blocked[iy * n + ix] = 0;
    }
  }
}

function nearestDist(dist: Int16Array, blocked: Uint8Array, n: number, gx: number, gy: number): number {
  let best = -1;
  for (let iy = Math.max(0, gy - 2); iy <= Math.min(n - 1, gy + 2); iy++) {
    for (let ix = Math.max(0, gx - 2); ix <= Math.min(n - 1, gx + 2); ix++) {
      const value = dist[iy * n + ix]!;
      if (blocked[iy * n + ix] || value < 0) continue;
      if (best < 0 || value < best) best = value;
    }
  }
  return best;
}

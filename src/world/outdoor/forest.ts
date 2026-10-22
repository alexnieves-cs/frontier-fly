import { RNG } from '../../shared/rng';
import { distToSegment } from './math';
import type { BuildingShell, Corridor, Geofence, Terrain, TreeInstance, TreeKind } from './types';
import { containsPoint } from './geofence';
import { sampleHeight } from './terrain';

/** Radius around home kept free of every plant, so the drone takes off and lands on open ground. */
const PAD_CLEAR_M = 2;

export interface Aabb {
  minX: number;
  minY: number;
  minZ: number;
  maxX: number;
  maxY: number;
  maxZ: number;
}

export function shellAabb(shell: BuildingShell, margin = 0): Aabb {
  const floor = shell.spec.floors[0]!;
  const w = floor.w * shell.spec.cellSize;
  const d = floor.d * shell.spec.cellSize;
  const h = shell.spec.floors.length * shell.spec.floorHeight;
  return {
    minX: shell.origin[0] - margin,
    minY: shell.origin[1] - margin,
    minZ: shell.origin[2] - margin,
    maxX: shell.origin[0] + w + margin,
    maxY: shell.origin[1] + d + margin,
    maxZ: shell.origin[2] + h + margin,
  };
}

export function treeHitsAabb(tree: TreeInstance, box: Aabb): boolean {
  if (tree.trunkHeight > 0.05 && tree.trunkRadius > 0) {
    const z0 = tree.z;
    const z1 = tree.z + tree.trunkHeight;
    if (cylinderHitsAabb(tree.x, tree.y, z0, z1, tree.trunkRadius, box)) return true;
  }
  return sphereHitsAabb(tree.x, tree.y, tree.z + tree.canopyZ, tree.canopyRadius, box);
}

export function scatterForest(
  terrain: Terrain,
  buildings: readonly BuildingShell[],
  corridors: readonly Corridor[],
  fence: Geofence,
  spawnX: number,
  spawnY: number,
  rng: RNG,
): TreeInstance[] {
  const trees: TreeInstance[] = [];
  const step = 4.1;
  const boxes = buildings.map((shell) => shellAabb(shell, 0.4));
  for (let y = step * 0.5; y < terrain.size - 2; y += step) {
    for (let x = step * 0.5; x < terrain.size - 2; x += step) {
      if (trees.length >= 260) return trees;
      const px = x + rng.range(-1.35, 1.35);
      const py = y + rng.range(-1.35, 1.35);
      if (!containsPoint(fence, px, py)) continue;
      const kind = pickKind(rng);
      const tree = makeTree(terrain, px, py, kind, rng);
      if (lodOf(px, py, spawnX, spawnY) === 2 && rng.bool(0.25)) continue;
      if (blocked(tree, boxes, corridors)) continue;
      // Corridors allow undergrowth, but not on the landing pad itself.
      if (Math.hypot(px - spawnX, py - spawnY) < PAD_CLEAR_M + tree.canopyRadius) continue;
      tree.lod = lodOf(px, py, spawnX, spawnY);
      trees.push(tree);
    }
  }
  return trees;
}

function blocked(tree: TreeInstance, boxes: readonly Aabb[], corridors: readonly Corridor[]): boolean {
  for (const box of boxes) if (treeHitsAabb(tree, box)) return true;
  if (tree.kind === 'undergrowth') return false;
  const reach = Math.max(tree.trunkRadius, tree.canopyRadius);
  for (const corridor of corridors) {
    if (distToSegment(tree.x, tree.y, corridor.ax, corridor.ay, corridor.bx, corridor.by) < corridor.halfWidth + reach) {
      return true;
    }
  }
  return false;
}

function pickKind(rng: RNG): TreeKind {
  const roll = rng.next();
  if (roll < 0.46) return 'pine';
  if (roll < 0.78) return 'broadleaf';
  return 'undergrowth';
}

function makeTree(terrain: Terrain, x: number, y: number, kind: TreeKind, rng: RNG): TreeInstance {
  const z = sampleHeight(terrain, x, y);
  const tint = rng.next();
  const yaw = rng.range(0, Math.PI * 2);
  if (kind === 'undergrowth') {
    const canopyRadius = rng.range(0.28, 0.55);
    return {
      x,
      y,
      z,
      yaw,
      kind,
      trunkRadius: 0,
      trunkHeight: 0,
      canopyRadius,
      canopyZ: canopyRadius * 0.75,
      lod: 0,
      tint,
    };
  }
  if (kind === 'pine') {
    const trunkHeight = rng.range(3.6, 6.4);
    const canopyRadius = rng.range(1.25, 2.15);
    return {
      x,
      y,
      z,
      yaw,
      kind,
      trunkRadius: rng.range(0.11, 0.2),
      trunkHeight,
      canopyRadius,
      canopyZ: trunkHeight * 0.7,
      lod: 0,
      tint,
    };
  }
  const trunkHeight = rng.range(2.1, 3.8);
  const canopyRadius = rng.range(1.45, 2.45);
  return {
    x,
    y,
    z,
    yaw,
    kind,
    trunkRadius: rng.range(0.14, 0.26),
    trunkHeight,
    canopyRadius,
    canopyZ: trunkHeight + canopyRadius * 0.45,
    lod: 0,
    tint,
  };
}

export function lodOf(x: number, y: number, spawnX: number, spawnY: number): 0 | 1 | 2 {
  const dist = Math.hypot(x - spawnX, y - spawnY);
  if (dist < 16) return 0;
  if (dist < 30) return 1;
  return 2;
}

function sphereHitsAabb(x: number, y: number, z: number, r: number, box: Aabb): boolean {
  const cx = x < box.minX ? box.minX : x > box.maxX ? box.maxX : x;
  const cy = y < box.minY ? box.minY : y > box.maxY ? box.maxY : y;
  const cz = z < box.minZ ? box.minZ : z > box.maxZ ? box.maxZ : z;
  const dx = x - cx;
  const dy = y - cy;
  const dz = z - cz;
  return dx * dx + dy * dy + dz * dz < r * r;
}

/** Vertical flat-capped cylinder. */
function cylinderHitsAabb(x: number, y: number, z0: number, z1: number, r: number, box: Aabb): boolean {
  if (box.maxZ < z0 || box.minZ > z1) return false;
  const cx = x < box.minX ? box.minX : x > box.maxX ? box.maxX : x;
  const cy = y < box.minY ? box.minY : y > box.maxY ? box.maxY : y;
  const dx = x - cx;
  const dy = y - cy;
  return dx * dx + dy * dy < r * r;
}

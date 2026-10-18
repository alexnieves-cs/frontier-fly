import { CELL } from '../shared/constants';
import { reachableMasks } from '../gen/validate';
import type { BuildingSpec } from '../shared/types';
import { bodyAabb } from './motion';
import { resolveTarget } from './motion';
import type { Aabb, PlacedTarget, TargetScene, Vec3 } from './types';

function cellsOf(spec: BuildingSpec, floor: number, box: Aabb): Array<{ ix: number; iy: number; cell: number; reachable: number }> {
  const grid = spec.floors[floor];
  if (!grid) return [];
  const masks = reachableMasks(spec);
  const cs = spec.cellSize;
  const out = [];
  const ix0 = Math.floor(box.min[0] / cs);
  const ix1 = Math.floor((box.max[0] - 1e-4) / cs);
  const iy0 = Math.floor(box.min[1] / cs);
  const iy1 = Math.floor((box.max[1] - 1e-4) / cs);
  for (let iy = iy0; iy <= iy1; iy++) {
    for (let ix = ix0; ix <= ix1; ix++) {
      if (ix < 0 || iy < 0 || ix >= grid.w || iy >= grid.d) {
        out.push({ ix, iy, cell: -1, reachable: 0 });
        continue;
      }
      const i = iy * grid.w + ix;
      out.push({ ix, iy, cell: grid.cells[i] ?? -1, reachable: masks[floor]?.[i] ?? 0 });
    }
  }
  return out;
}

function samples(target: PlacedTarget): Vec3[] {
  if (!target.path || target.path.length < 2 || target.state !== 'walking') return [target.feet];
  const points = [...target.path];
  for (let i = 0; i < target.path.length; i++) {
    const a = target.path[i]!;
    const b = target.path[(i + 1) % target.path.length]!;
    points.push([(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2]);
  }
  return points;
}

function outdoorHit(scene: TargetScene, box: Aabb): string | null {
  const area = scene.outdoor;
  if (!area) return 'missing outdoor area';
  if (box.min[0] < area.minX - 1e-3 || box.max[0] > area.maxX + 1e-3) return 'outside east-west bounds';
  if (box.min[1] < area.minY - 1e-3 || box.max[1] > area.maxY + 1e-3) return 'outside north-south bounds';
  if (box.min[2] < area.groundZ - 0.08) return 'below the ground';
  for (const obstacle of area.obstacles ?? []) {
    const hit = box.min[0] < obstacle.maxX && box.max[0] > obstacle.minX && box.min[1] < obstacle.maxY && box.max[1] > obstacle.minY;
    if (hit) return 'inside an obstacle';
  }
  return null;
}

/** Human-readable placement problems. An empty list means the scene is valid. */
export function placementIssues(scene: TargetScene, spec?: BuildingSpec): string[] {
  const issues: string[] = [];
  const boxes: Aabb[] = [];
  for (const target of scene.targets) {
    for (const point of samples(target)) {
      const yaw = target.path && target.path.length > 1
        ? Math.atan2(target.path[1]![1] - target.path[0]![1], target.path[1]![0] - target.path[0]![0])
        : target.yaw;
      const box = bodyAabb(point, yaw, target.hx, target.hy, target.height);
      if (scene.space === 'outdoor') {
        const problem = outdoorHit(scene, box);
        if (problem) issues.push(`${target.id} ${problem}`);
        continue;
      }
      if (!spec) {
        issues.push(`${target.id} indoor scene is missing a building`);
        continue;
      }
      const floor = spec.floors[target.floor];
      if (!floor) {
        issues.push(`${target.id} has no floor`);
        continue;
      }
      const z0 = target.floor * spec.floorHeight;
      const z1 = z0 + spec.floorHeight;
      if (box.min[2] < z0 - 0.05 || box.max[2] > z1 - 0.02) issues.push(`${target.id} is outside the floor slab`);
      for (const cell of cellsOf(spec, target.floor, box)) {
        if (cell.cell !== CELL.EMPTY) issues.push(`${target.id} overlaps a non-floor cell at ${cell.ix},${cell.iy}`);
        else if (!cell.reachable) issues.push(`${target.id} is not reachable at ${cell.ix},${cell.iy}`);
      }
    }
    const resolved = resolveTarget(target, 0);
    for (const other of boxes) {
      const a = resolved.aabb;
      const overlap = a.min[0] < other.max[0] - 0.05 && a.max[0] > other.min[0] + 0.05
        && a.min[1] < other.max[1] - 0.05 && a.max[1] > other.min[1] + 0.05
        && a.min[2] < other.max[2] && a.max[2] > other.min[2];
      if (overlap) issues.push(`${target.id} overlaps another target`);
    }
    boxes.push(resolved.aabb);
  }
  return issues;
}

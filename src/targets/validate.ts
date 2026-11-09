import { CELL } from '../shared/constants';
import { reachableMasks } from '../gen/validate';
import type { BuildingSpec } from '../shared/types';
import { bodyAabb } from './motion';
import { resolveTarget } from './motion';
import { sweptPoses } from './placement';
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

function samples(target: PlacedTarget): Array<{ at: Vec3; yaw: number }> {
  const walking = target.state === 'walking' && target.path && target.path.length >= 2;
  return sweptPoses(target.feet, target.yaw, walking ? target.path : undefined);
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
  const boxes: Aabb[][] = [];
  for (const target of scene.targets) {
    const poses = samples(target);
    const swept: Aabb[] = [];
    for (const { at, yaw } of poses) {
      const box = bodyAabb(at, yaw, target.hx, target.hy, target.height);
      swept.push(box);
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
    // A walker is checked at every pose of its walk; anything else where it stands.
    const mine = target.state === 'walking' && poses.length > 1 ? swept : [resolveTarget(target, 0).aabb];
    if (scene.space === 'indoor' && spec) {
      for (const part of spec.furniture?.boxes ?? []) {
        if (part.floor !== target.floor) continue;
        const c = Math.abs(Math.cos(part.yaw));
        const sn = Math.abs(Math.sin(part.yaw));
        const ex = c * part.hx + sn * part.hy;
        const ey = sn * part.hx + c * part.hy;
        const solid: Aabb = { min: [part.cx - ex, part.cy - ey, part.cz - part.hz], max: [part.cx + ex, part.cy + ey, part.cz + part.hz] };
        if (mine.some((a) => overlaps(a, solid, 0.02))) issues.push(`${target.id} passes through furniture`);
      }
    }
    for (const other of boxes) if (other.some((b) => mine.some((a) => overlaps(a, b, 0.05)))) issues.push(`${target.id} overlaps another target`);
    boxes.push(mine);
  }
  return [...new Set(issues)];
}

function overlaps(a: Aabb, b: Aabb, slack: number): boolean {
  return a.min[0] < b.max[0] - slack && a.max[0] > b.min[0] + slack
    && a.min[1] < b.max[1] - slack && a.max[1] > b.min[1] + slack
    && a.min[2] < b.max[2] && a.max[2] > b.min[2];
}

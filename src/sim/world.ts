import RAPIER from '@dimforge/rapier3d-compat';
import { CELL, DRONE_RADIUS } from '../shared/constants';
import { furnitureIndex, type FurnitureIndex } from '../gen/furniture';
import { buildingAabbs } from '../gen/parts';
import { isWalkable } from '../gen/validate';
import type { BuildingSpec } from '../shared/types';

let ready: Promise<void> | null = null;

export function initPhysics(): Promise<void> {
  if (!ready) ready = RAPIER.init();
  return ready;
}

function sphereAabb(
  x: number,
  y: number,
  z: number,
  r: number,
  minX: number,
  minY: number,
  minZ: number,
  maxX: number,
  maxY: number,
  maxZ: number,
): boolean {
  const cx = x < minX ? minX : x > maxX ? maxX : x;
  const cy = y < minY ? minY : y > maxY ? maxY : y;
  const cz = z < minZ ? minZ : z > maxZ ? maxZ : z;
  const dx = x - cx;
  const dy = y - cy;
  const dz = z - cz;
  return dx * dx + dy * dy + dz * dz < r * r;
}

export interface MoveResult {
  pos: [number, number, number];
  hit: boolean;
  blocked: [boolean, boolean, boolean];
}

export interface RayHit {
  dist: number;
  surface: number;
}

/**
 * Rapier owns the raycast colliders. Contact response uses the same grid
 * AABBs analytically so a step stays deterministic and cheap.
 *
 * Furniture (spec.furniture, planned in src/gen/furniture.ts) is solid: each
 * part is an oriented Rapier cuboid the lidar sees, and `penetrates` tests the
 * drone sphere against the same boxes through a per-cell bucket index, so only
 * boxes in the cells under the sphere are checked.
 */
export class PhysicsWorld {
  readonly spec: BuildingSpec;
  readonly furniture: FurnitureIndex;
  private readonly world: RAPIER.World;
  private readonly ray: RAPIER.Ray;
  /** Collider handle -> SURFACE class for furniture parts. */
  private readonly furnitureSurface = new Map<number, number>();

  constructor(spec: BuildingSpec) {
    this.spec = spec;
    this.world = new RAPIER.World({ x: 0, y: 0, z: 0 });
    const body = this.world.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    for (const box of buildingAabbs(spec)) {
      const desc = RAPIER.ColliderDesc.cuboid(box.hx, box.hy, box.hz).setTranslation(box.cx, box.cy, box.cz);
      this.world.createCollider(desc, body);
    }
    this.furniture = furnitureIndex(spec);
    this.addFurniture(body);
    this.world.timestep = 1 / 60;
    this.world.step();
    this.ray = new RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 1, y: 0, z: 0 });
  }

  private addFurniture(body: RAPIER.RigidBody): void {
    for (const box of this.furniture.boxes) {
      const half = box.yaw / 2;
      const desc = RAPIER.ColliderDesc.cuboid(box.hx, box.hy, box.hz)
        .setTranslation(box.cx, box.cy, box.cz)
        .setRotation({ x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) });
      const collider = this.world.createCollider(desc, body);
      this.furnitureSurface.set(collider.handle, box.surface);
    }
  }

  free(): void {
    this.world.free();
  }

  surfaceAt(x: number, y: number, z: number): number {
    const floors = this.spec.floors;
    const f = Math.max(0, Math.min(floors.length - 1, Math.floor(z / this.spec.floorHeight)));
    const floor = floors[f]!;
    const gx = Math.max(0, Math.min(floor.w - 1, Math.floor(x / this.spec.cellSize)));
    const gz = Math.max(0, Math.min(floor.d - 1, Math.floor(y / this.spec.cellSize)));
    return floor.surfaces?.[gz * floor.w + gx] ?? 0;
  }

  cast(ox: number, oy: number, oz: number, dx: number, dy: number, dz: number, maxDist: number): RayHit | null {
    const len = Math.hypot(dx, dy, dz);
    if (len < 1e-8 || maxDist <= 0) return null;
    const inv = 1 / len;
    this.ray.origin.x = ox;
    this.ray.origin.y = oy;
    this.ray.origin.z = oz;
    this.ray.dir.x = dx * inv;
    this.ray.dir.y = dy * inv;
    this.ray.dir.z = dz * inv;
    const hit = this.world.castRay(this.ray, maxDist, true);
    if (!hit) return null;
    const dist = hit.timeOfImpact;
    if (this.furnitureSurface.size > 0) {
      const surface = this.furnitureSurface.get(hit.collider.handle);
      if (surface != null) return { dist, surface };
    }
    return {
      dist,
      surface: this.surfaceAt(ox + this.ray.dir.x * dist, oy + this.ray.dir.y * dist, oz + this.ray.dir.z * dist),
    };
  }

  penetrates(p: [number, number, number], radius = DRONE_RADIUS): boolean {
    const [x, y, z] = p;
    const { floors, cellSize: cs, floorHeight: H } = this.spec;
    const floor0 = floors[0]!;
    if (x - radius < 0 || y - radius < 0 || x + radius > floor0.w * cs || y + radius > floor0.d * cs) return true;
    if (z - radius < -1e-4) return true;
    if (z + radius > floors.length * H + 1e-4) return true;
    for (let f = 0; f < floors.length; f++) {
      const z0 = f * H;
      const z1 = z0 + H;
      if (z + radius < z0 || z - radius > z1) continue;
      const floor = floors[f]!;
      const x0 = Math.max(0, Math.floor((x - radius) / cs));
      const x1 = Math.min(floor.w - 1, Math.floor((x + radius) / cs));
      const y0 = Math.max(0, Math.floor((y - radius) / cs));
      const y1 = Math.min(floor.d - 1, Math.floor((y + radius) / cs));
      for (let gz = y0; gz <= y1; gz++) {
        for (let gx = x0; gx <= x1; gx++) {
          if (floor.cells[gz * floor.w + gx] !== CELL.WALL) continue;
          if (sphereAabb(x, y, z, radius, gx * cs, gz * cs, z0, (gx + 1) * cs, (gz + 1) * cs, z1)) return true;
        }
      }
    }
    for (let k = 1; k < floors.length; k++) {
      const slab = k * H;
      if (Math.abs(z - slab) > radius) continue;
      const gx = Math.floor(x / cs);
      const gz = Math.floor(y / cs);
      const lower = floors[k - 1]!;
      if (gx < 0 || gz < 0 || gx >= lower.w || gz >= lower.d) return true;
      if (lower.cells[gz * lower.w + gx] !== CELL.SHAFT) return true;
    }
    return this.furniture.hits(x, y, z, radius);
  }

  resolve(pos: [number, number, number], delta: [number, number, number], radius = DRONE_RADIUS): MoveResult {
    const next: [number, number, number] = [pos[0], pos[1], pos[2]];
    const blocked: [boolean, boolean, boolean] = [false, false, false];
    for (let axis = 0; axis < 3; axis++) {
      next[axis] += delta[axis];
      if (this.penetrates(next, radius)) {
        next[axis] -= delta[axis];
        blocked[axis] = true;
      }
    }
    return { pos: next, hit: blocked[0] || blocked[1] || blocked[2], blocked };
  }

  cellAt(x: number, y: number, z: number): { f: number; gx: number; gz: number; cell: number } | null {
    const f = Math.floor(z / this.spec.floorHeight);
    if (f < 0 || f >= this.spec.floors.length) return null;
    const floor = this.spec.floors[f]!;
    const gx = Math.floor(x / this.spec.cellSize);
    const gz = Math.floor(y / this.spec.cellSize);
    if (gx < 0 || gz < 0 || gx >= floor.w || gz >= floor.d) return null;
    return { f, gx, gz, cell: floor.cells[gz * floor.w + gx]! };
  }

  walkable(x: number, y: number, z: number): boolean {
    const hit = this.cellAt(x, y, z);
    return hit != null && isWalkable(hit.cell);
  }
}

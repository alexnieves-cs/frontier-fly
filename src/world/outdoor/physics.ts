import RAPIER from '@dimforge/rapier3d-compat';
import { DRONE_RADIUS, LIDAR_H, LIDAR_TILT } from '../../shared/constants';
import { OUTDOOR_LIDAR_MAX } from './constants';
import { inNoFly } from './geofence';
import { containsPoint } from './geofence';
import { sampleHeight } from './terrain';
import type { OutdoorWorld, Solid } from './types';

let ready: Promise<void> | null = null;

export function initOutdoorPhysics(): Promise<void> {
  if (!ready) ready = RAPIER.init();
  return ready;
}

export interface OutdoorHit {
  dist: number;
  tag: string;
}

const CYLINDER_UP = { x: Math.SQRT1_2, y: 0, z: 0, w: Math.SQRT1_2 };

/**
 * Rapier colliders for the outdoor world. Rays are the outdoor lidar.
 * Movement is a kinematic ball with slide, plus the geofence and no-fly data.
 */
export class OutdoorPhysics {
  readonly colliderCount: number;
  private readonly rapier: RAPIER.World;
  private readonly ray: RAPIER.Ray;
  private readonly body: RAPIER.RigidBody;
  private readonly drone: RAPIER.Collider;
  private readonly controller: RAPIER.KinematicCharacterController;
  private readonly tags = new Map<number, string>();

  constructor(private readonly world: OutdoorWorld) {
    this.rapier = new RAPIER.World({ x: 0, y: 0, z: 0 });
    const fixed = this.rapier.createRigidBody(RAPIER.RigidBodyDesc.fixed());
    for (const solid of world.colliders) {
      const desc = describe(solid);
      if (!desc) continue;
      const collider = this.rapier.createCollider(desc, fixed);
      this.tags.set(collider.handle, solid.tag);
    }
    this.body = this.rapier.createRigidBody(
      RAPIER.RigidBodyDesc.kinematicPositionBased().setTranslation(world.spawn[0], world.spawn[1], world.spawn[2]),
    );
    this.drone = this.rapier.createCollider(RAPIER.ColliderDesc.ball(DRONE_RADIUS), this.body);
    this.controller = this.rapier.createCharacterController(0.02);
    this.controller.setUp({ x: 0, y: 0, z: 1 });
    this.controller.setSlideEnabled(true);
    this.controller.setMaxSlopeClimbAngle(Math.PI / 2);
    this.controller.setMinSlopeSlideAngle(Math.PI / 2);
    this.controller.disableSnapToGround();
    this.rapier.timestep = 1 / 60;
    this.rapier.step();
    this.ray = new RAPIER.Ray({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: -1 });
    this.colliderCount = world.colliders.length;
  }

  free(): void {
    this.rapier.free();
  }

  cast(ox: number, oy: number, oz: number, dx: number, dy: number, dz: number, maxDist: number): OutdoorHit | null {
    const len = Math.hypot(dx, dy, dz);
    if (len < 1e-8 || maxDist <= 0) return null;
    const inv = 1 / len;
    this.ray.origin.x = ox;
    this.ray.origin.y = oy;
    this.ray.origin.z = oz;
    this.ray.dir.x = dx * inv;
    this.ray.dir.y = dy * inv;
    this.ray.dir.z = dz * inv;
    const hit = this.rapier.castRay(this.ray, maxDist, true, undefined, undefined, this.drone);
    if (!hit) return null;
    return { dist: hit.timeOfImpact, tag: this.tags.get(hit.collider.handle) ?? 'unknown' };
  }

  /**
   * Gap between the drone's sphere at `pos` and the nearest solid other than the terrain, m; negative
   * when they overlap. The ground is left out because landing on it is the point of the flight.
   */
  clearance(pos: [number, number, number]): { gapM: number; tag: string } | null {
    const notTerrain = (c: RAPIER.Collider) => this.tags.get(c.handle) !== 'terrain';
    const hit = this.rapier.projectPoint({ x: pos[0], y: pos[1], z: pos[2] }, true, undefined, undefined, this.drone, undefined, notTerrain);
    if (!hit) return null;
    const d = Math.hypot(hit.point.x - pos[0], hit.point.y - pos[1], hit.point.z - pos[2]);
    return { gapM: (hit.isInside ? -d : d) - DRONE_RADIUS, tag: this.tags.get(hit.collider.handle) ?? 'unknown' };
  }

  /** 32 horizontal rays plus 8 tilted rays, normalized to [0, 1] by the outdoor range. */
  lidar(pos: [number, number, number], yaw: number, out = new Float32Array(LIDAR_H + LIDAR_TILT)): Float32Array {
    const normRay = (dx: number, dy: number, dz: number) => {
      const hit = this.cast(pos[0], pos[1], pos[2], dx, dy, dz, OUTDOOR_LIDAR_MAX);
      const meters = hit ? hit.dist : OUTDOOR_LIDAR_MAX;
      return Math.max(0, Math.min(1, meters / OUTDOOR_LIDAR_MAX));
    };
    for (let i = 0; i < LIDAR_H; i++) {
      const angle = yaw + (i / LIDAR_H) * Math.PI * 2;
      out[i] = normRay(Math.cos(angle), Math.sin(angle), 0);
    }
    for (let i = 0; i < LIDAR_TILT; i++) {
      const angle = yaw + (i / LIDAR_TILT) * Math.PI * 2;
      const elev = i % 2 === 0 ? 0.45 : -0.45;
      const ch = Math.cos(elev);
      out[LIDAR_H + i] = normRay(Math.cos(angle) * ch, Math.sin(angle) * ch, Math.sin(elev));
    }
    return out;
  }

  move(pos: [number, number, number], delta: [number, number, number]): { pos: [number, number, number]; hit: boolean } {
    this.body.setNextKinematicTranslation({ x: pos[0], y: pos[1], z: pos[2] });
    this.rapier.step();
    this.controller.computeColliderMovement(this.drone, { x: delta[0], y: delta[1], z: delta[2] });
    const motion = this.controller.computedMovement();
    const hit = this.controller.numComputedCollisions() > 0;
    const candidates: [number, number, number][] = [
      [pos[0] + motion.x, pos[1] + motion.y, pos[2] + motion.z],
      [pos[0] + motion.x, pos[1], pos[2]],
      [pos[0], pos[1] + motion.y, pos[2]],
      [pos[0], pos[1], pos[2] + motion.z],
    ];
    let next: [number, number, number] = [pos[0], pos[1], pos[2]];
    for (const candidate of candidates) {
      if (this.allowed(candidate[0], candidate[1], candidate[2])) {
        next = candidate;
        break;
      }
    }
    const floor = sampleHeight(this.world.terrain, next[0], next[1]) + DRONE_RADIUS + 0.04;
    if (next[2] < floor) next[2] = floor;
    this.body.setNextKinematicTranslation({ x: next[0], y: next[1], z: next[2] });
    this.rapier.step();
    return { pos: next, hit };
  }

  private allowed(x: number, y: number, z: number): boolean {
    if (!containsPoint(this.world.geofence, x, y)) return false;
    if (inNoFly(this.world.noFly, x, y, z)) return false;
    return true;
  }
}

function describe(solid: Solid): RAPIER.ColliderDesc | null {
  if (solid.shape === 'trimesh') {
    return RAPIER.ColliderDesc.trimesh(solid.vertices, solid.indices, RAPIER.TriMeshFlags.FIX_INTERNAL_EDGES);
  }
  if (solid.shape === 'cuboid') {
    return RAPIER.ColliderDesc.cuboid(solid.hx, solid.hy, solid.hz)
      .setTranslation(solid.cx, solid.cy, solid.cz)
      .setRotation({ x: solid.qx, y: solid.qy, z: solid.qz, w: solid.qw });
  }
  if (solid.shape === 'cylinder') {
    return RAPIER.ColliderDesc.cylinder(solid.halfHeight, solid.radius)
      .setTranslation(solid.cx, solid.cy, solid.cz)
      .setRotation(CYLINDER_UP);
  }
  return RAPIER.ColliderDesc.ball(solid.radius).setTranslation(solid.cx, solid.cy, solid.cz);
}

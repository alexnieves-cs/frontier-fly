import * as THREE from 'three';
import { describe, expect, it } from 'vitest';
import raw from '../fixtures/simpleRoom.json';
import { generateBuilding } from '../../src/gen/building';
import { FURNITURE_CLEARANCE, boxExtent, planFurniture, propBoxes } from '../../src/gen/furniture';
import { computeFlightMap, isFullyReachable } from '../../src/gen/validate';
import { Curriculum } from '../../src/rl/curriculum';
import { CELL, DRONE_RADIUS, SURFACE } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import { buildingFromJson, buildingToJson } from '../../src/shared/serialize';
import type { BuildingSpec, PropPlacement } from '../../src/shared/types';
import { createEnv, defaultEpisodeConfig } from '../../src/sim/env';
import { frontierAction } from '../../src/sim/frontier';
import { initPhysics, PhysicsWorld } from '../../src/sim/world';
import { buildProps, planProps } from '../../src/viz/props';
import type { MaterialSet } from '../../src/viz/three/materials';

/** The 8x8 fixture room with hand-placed furniture. */
function furnished(props: PropPlacement[]): BuildingSpec {
  const spec = buildingFromJson(raw);
  spec.furniture = { props, boxes: props.flatMap((prop, i) => propBoxes(spec, prop, i)) };
  spec.reachableVoxels = computeFlightMap(spec).count;
  return spec;
}

// A shelf across the +x path from the spawn (5, 5, 1): its 0.28 m depth spans x 6.36-6.64.
const SHELF: PropPlacement = { kind: 'shelf', x: 6.5, y: 0, z: 5, rot: Math.PI / 2, floor: 0 };

function sweep(count: number, fn: (spec: BuildingSpec) => void): void {
  for (let i = 0; i < count; i++) {
    const rng = new RNG(i + 77, 'furniture-sweep');
    const stage = rng.int(6);
    const floors = stage >= 4 ? 2 + rng.int(2) : 1;
    fn(generateBuilding({ seed: 20_000 + i, floors, w: 8 + rng.int(12), d: 8 + rng.int(12), stage }));
  }
}

describe('solid furniture', () => {
  it('blocks the drone at a shelf and counts the impact as a collision', async () => {
    const spec = furnished([SHELF]);
    const env = await createEnv();
    const cfg = defaultEpisodeConfig();
    cfg.maxSteps = 10_000;
    cfg.stallSteps = 10_000;
    cfg.maxCollisions = 100;
    env.reset(spec, 3, cfg);
    let collisions = 0;
    for (let i = 0; i < 80; i++) {
      const step = env.step(new Float32Array([0.6, 0, 0, 0]));
      collisions = step.info.collisions;
    }
    const x = env.snapshot().pos[0];
    expect(collisions).toBeGreaterThanOrEqual(1);
    expect(x).toBeLessThan(6.36 - DRONE_RADIUS + 1e-6);
    expect(x).toBeGreaterThan(6.36 - DRONE_RADIUS - 0.1);
    env.dispose();

    // Without the shelf the same flight reaches the far wall.
    const bare = buildingFromJson(raw);
    env.reset(bare, 3, cfg);
    for (let i = 0; i < 80; i++) env.step(new Float32Array([0.6, 0, 0, 0]));
    expect(env.snapshot().pos[0]).toBeGreaterThan(8.5);
    env.dispose();
  });

  it('shows furniture to the lidar at the expected distance, as wood', async () => {
    await initPhysics();
    const world = new PhysicsWorld(furnished([SHELF]));
    const hit = world.cast(5, 5, 1, 1, 0, 0, 8);
    expect(hit).not.toBeNull();
    expect(hit!.dist).toBeCloseTo(1.36, 3);
    expect(hit!.surface).toBe(SURFACE.WOOD);
    // Above the shelf top (1.35 m) the ray reaches the wall at x = 9.
    expect(world.cast(5, 5, 1.6, 1, 0, 0, 8)!.dist).toBeCloseTo(4, 3);
    // A table top (0.76 m) is below flight altitude: flying over it is free.
    const table = new PhysicsWorld(furnished([{ kind: 'table', x: 6.5, y: 0, z: 5, rot: 0, floor: 0 }]));
    expect(table.penetrates([6.5, 5, 1])).toBe(false);
    expect(table.penetrates([6.5, 5, 0.85])).toBe(true);
    world.free();
    table.free();
  });

  it('never puts furniture on doors, shafts, outside its room, or near the spawn', () => {
    let props = 0;
    sweep(300, (spec) => {
      const plan = spec.furniture!;
      props += plan.props.length;
      const cs = spec.cellSize;
      const sx = Math.floor(spec.spawn[0] / cs);
      const sy = Math.floor(spec.spawn[1] / cs);
      for (const box of plan.boxes) {
        const grid = spec.floors[box.floor]!;
        const [ex, ey] = boxExtent(box);
        const m = FURNITURE_CLEARANCE;
        for (let gz = Math.floor((box.cy - ey - m) / cs); gz <= Math.floor((box.cy + ey + m - 1e-6) / cs); gz++) {
          for (let gx = Math.floor((box.cx - ex - m) / cs); gx <= Math.floor((box.cx + ex + m - 1e-6) / cs); gx++) {
            const cell = grid.cells[gz * grid.w + gx];
            expect(cell === CELL.DOOR || cell === CELL.SHAFT).toBe(false);
            if (box.floor === 0) expect(gx === sx && gz === sy).toBe(false);
          }
        }
        for (let gz = Math.floor((box.cy - ey) / cs); gz <= Math.floor((box.cy + ey - 1e-6) / cs); gz++) {
          for (let gx = Math.floor((box.cx - ex) / cs); gx <= Math.floor((box.cx + ex - 1e-6) / cs); gx++) {
            expect(grid.cells[gz * grid.w + gx]).toBe(CELL.EMPTY);
          }
        }
      }
      expect(isFullyReachable(spec)).toBe(true);
    });
    expect(props).toBeGreaterThan(300);
  });

  it('meshes the renderer props on the physics boxes of the same plan', () => {
    const spec = generateBuilding({ seed: 9, floors: 2, w: 18, d: 16, stage: 3 });
    const plan = spec.furniture!;
    expect(planProps(spec)).toBe(plan.props);
    const plain = new THREE.MeshStandardMaterial();
    const mats = { wall: plain, concrete: plain, floor: plain, door: plain } as unknown as MaterialSet;
    const root = buildProps(spec, mats);
    expect(root.children).toHaveLength(plan.props.length);
    root.updateMatrixWorld(true);
    root.children.forEach((group, i) => {
      // Renderer space is (x, up, sim y). Compare the mesh bounds with the union of its boxes.
      const mesh = new THREE.Box3().setFromObject(group);
      const boxes = plan.boxes.filter((box) => box.prop === i);
      expect(boxes.length).toBeGreaterThan(0);
      let minX = Infinity;
      let maxX = -Infinity;
      let minY = Infinity;
      let maxY = -Infinity;
      let minZ = Infinity;
      let maxZ = -Infinity;
      for (const box of boxes) {
        // Exact corners of the oriented box.
        const c = Math.cos(box.yaw);
        const s = Math.sin(box.yaw);
        for (const [a, b] of [
          [1, 1],
          [1, -1],
          [-1, 1],
          [-1, -1],
        ] as const) {
          const x = box.cx + a * box.hx * c - b * box.hy * s;
          const y = box.cy + a * box.hx * s + b * box.hy * c;
          minX = Math.min(minX, x);
          maxX = Math.max(maxX, x);
          minY = Math.min(minY, y);
          maxY = Math.max(maxY, y);
        }
        minZ = Math.min(minZ, box.cz - box.hz);
        maxZ = Math.max(maxZ, box.cz + box.hz);
      }
      const prop = plan.props[i]!;
      // Round meshes (plant pot, foliage) are inside their box; boxy props match it.
      const tol = prop.kind === 'plant' ? 0.06 : 0.011;
      expect(mesh.min.x).toBeGreaterThanOrEqual(minX - 0.011);
      expect(mesh.max.x).toBeLessThanOrEqual(maxX + 0.011);
      expect(mesh.min.z).toBeGreaterThanOrEqual(minY - 0.011);
      expect(mesh.max.z).toBeLessThanOrEqual(maxY + 0.011);
      expect(Math.abs(mesh.max.x - maxX)).toBeLessThan(tol);
      expect(Math.abs(mesh.min.z - minY)).toBeLessThan(tol);
      expect(mesh.max.y).toBeCloseTo(maxZ, 2);
      expect(mesh.min.y).toBeGreaterThanOrEqual(minZ - 0.011);
      expect(group.position.x).toBe(prop.x);
      expect(group.position.z).toBe(prop.z);
    });
  });

  it('drops furniture-blocked voxels from reachableVoxels', () => {
    const bare = buildingFromJson(raw);
    const bareCount = computeFlightMap(bare).count;
    // A row of shelves against a wall leaves a strip the drone center cannot occupy.
    const row: PropPlacement[] = [];
    for (let k = 0; k < 5; k++) row.push({ kind: 'shelf', x: 1.7 + k * 0.7, y: 0, z: 1.2, rot: 0, floor: 0 });
    const spec = furnished(row);
    const map = computeFlightMap(spec);
    expect(map.count).toBeLessThan(bareCount);
    expect(spec.reachableVoxels).toBe(map.count);
    // Voxels between the shelves and the wall (y 1.0-1.5) are gone; the room is still reachable.
    const vw = map.vw;
    for (let vx = 4; vx < 10; vx++) expect(map.reachable[2 * vw + vx]).toBe(0);
    expect(isFullyReachable(spec, map)).toBe(true);
    // Low furniture does not change the count.
    expect(computeFlightMap(furnished([{ kind: 'sofa', x: 5, y: 0, z: 3, rot: 0.4, floor: 0 }])).count).toBe(bareCount);
  });

  it('lets the teacher reach 95% coverage of a furnished stage-0 room without collisions', async () => {
    const env = await createEnv();
    const params = new Curriculum().genParams(0, new RNG(101, 'eval-0'));
    const spec = generateBuilding(params);
    expect(spec.furniture!.props.some((p) => p.kind === 'shelf')).toBe(true);
    const cfg = defaultEpisodeConfig(0, spec.floors[0]!.rooms.length);
    cfg.maxSteps = 1000;
    cfg.stallSteps = 500;
    cfg.maxCollisions = 16;
    env.reset(spec, params.seed, cfg);
    let info = { coverage: 0, collisions: 0, success: false };
    for (let t = 0; t < cfg.maxSteps; t++) {
      const pose = env.snapshot();
      const step = env.step(frontierAction(spec, env.visitedGrid, pose.pos, pose.yaw));
      info = step.info;
      if (step.done) break;
    }
    expect(info.coverage).toBeGreaterThanOrEqual(0.95);
    expect(info.collisions).toBe(0);
    env.dispose();
  });

  it('is deterministic per seed and survives JSON round trips', () => {
    const params = { seed: 4242, floors: 2, w: 18, d: 16, stage: 4 };
    const a = generateBuilding(params);
    const b = generateBuilding(params);
    expect(a.furniture).toEqual(b.furniture);
    expect(planFurniture(a)).toEqual(planFurniture(b));
    const back = buildingFromJson(JSON.parse(JSON.stringify(buildingToJson(a))));
    expect(back.furniture).toEqual(a.furniture);
    expect(back.reachableVoxels).toBe(computeFlightMap(back).count);
    expect(generateBuilding({ ...params, seed: 4243 }).furniture).not.toEqual(a.furniture);
  });
});

import { describe, expect, it } from 'vitest';
import raw from '../fixtures/simpleRoom.json';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { applyHazards, findDoorways, inSmoke, smokePath, smokeTransmittance } from '../../src/gen/hazards';
import { buildingAabbs } from '../../src/gen/parts';
import { computeFlightMap, reachableMasks } from '../../src/gen/validate';
import { CELL, LIDAR_MAX, OBS, SURFACE } from '../../src/shared/constants';
import { buildingFromJson, buildingToJson } from '../../src/shared/serialize';
import type { BuildingSpec } from '../../src/shared/types';
import { createEnv, defaultEpisodeConfig } from '../../src/sim/env';

const ALL = { closedDoors: 0.35, ajarDoors: 0.6, glassRooms: 0.35, mirrors: 2, smokeRooms: 0.4 } as const;

function hard(seed: number): BuildingSpec {
  return evalEpisodeSetup(3, seed, 'day').spec;
}

const cellsOf = (spec: BuildingSpec) => reachableMasks(spec).reduce((n, m) => n + m.reduce((a, v) => a + v, 0), 0);

/** The 8x8 fixture room narrowed to a 4 m wide corridor (x 1-5), with an empty hazard record. */
function corridor(): BuildingSpec {
  const spec = buildingFromJson(raw);
  const floor = spec.floors[0]!;
  for (let z = 1; z < floor.d - 1; z++) for (let x = 5; x < floor.w - 1; x++) {
      floor.cells[z * floor.w + x] = CELL.WALL;
      floor.surfaces![z * floor.w + x] = SURFACE.DRYWALL;
    }
  spec.spawn = [2.5, 5.5, 1];
  spec.hazards = { params: {}, closedDoors: [], ajarDoors: 0, glassRooms: 0, mirrors: 0, smoke: null };
  spec.reachableVoxels = computeFlightMap(spec).count;
  return spec;
}

describe('indoor hazards', () => {
  it('is seeded, leaves the input alone, and keeps half the building reachable', () => {
    for (let seed = 81901; seed < 81911; seed++) {
      const spec = hard(seed);
      const before = buildingToJson(spec);
      const a = applyHazards(spec, ALL);
      const b = applyHazards(spec, ALL);
      expect(buildingToJson(a)).toEqual(buildingToJson(b));
      expect(buildingToJson(spec)).toEqual(before);
      expect(cellsOf(a)).toBeGreaterThanOrEqual(0.5 * cellsOf(spec));
      expect(a.reachableVoxels).toBe(computeFlightMap(a).count);
      expect(a.reachableVoxels).toBeLessThanOrEqual(spec.reachableVoxels);
    }
  });

  it('shuts doors as solid door cells and adds ajar leaves as thin door props', () => {
    let shut = 0;
    let ajar = 0;
    for (let seed = 81901; seed < 81911; seed++) {
      const spec = hard(seed);
      const h = applyHazards(spec, { closedDoors: 0.5, ajarDoors: 1 });
      for (const door of h.hazards!.closedDoors) {
        for (const k of door.cells) {
          expect(h.floors[door.floor]!.cells[k]).toBe(CELL.WALL);
          expect(h.floors[door.floor]!.surfaces![k]).toBe(SURFACE.DOOR);
          expect(spec.floors[door.floor]!.cells[k]).toBe(CELL.DOOR);
        }
      }
      shut += h.hazards!.closedDoors.length;
      ajar += h.hazards!.ajarDoors;
      const leaves = h.furniture!.boxes.filter((box) => h.furniture!.props[box.prop]!.kind === 'door');
      for (const leaf of leaves) {
        expect(leaf.hy).toBeLessThan(0.05);
        expect(leaf.hz * 2).toBeGreaterThan(2.5);
      }
      if (h.hazards!.closedDoors.length) expect(buildingAabbs(h).some((box) => box.kind === 'door' && box.surface === SURFACE.DOOR)).toBe(true);
    }
    expect(shut).toBeGreaterThan(3);
    expect(ajar).toBeGreaterThan(3);
  });

  it('finds doorways with a single passage axis', () => {
    const spec = hard(81904);
    const doors = findDoorways(spec);
    expect(doors.length).toBeGreaterThan(0);
    for (const door of doors) expect(door.to).toBeGreaterThanOrEqual(door.from);
  });

  it('round-trips the hazard record through JSON', () => {
    const h = applyHazards(hard(81902), ALL);
    const back = buildingFromJson(JSON.parse(JSON.stringify(buildingToJson(h))));
    expect(buildingToJson(back)).toEqual(buildingToJson(h));
    expect(back.hazards!.smoke?.cells[0]).toBeInstanceOf(Uint8Array);
  });

  it('a mirror returns the reflected path to the lidar', async () => {
    const env = await createEnv();
    const plain = corridor();
    env.reset(plain, 1, defaultEpisodeConfig());
    const wall = env.observeAt(2.5, 5.5, 1, 0)[OBS.lidar]! * LIDAR_MAX;
    expect(wall).toBeCloseTo(2.5, 2);
    const mirrored = corridor();
    mirrored.floors[0]!.surfaces![5 * mirrored.floors[0]!.w + 5] = SURFACE.MIRROR;
    env.reset(mirrored, 1, defaultEpisodeConfig());
    // 2.5 m to the mirror at x = 5, then 4 m back to the wall face at x = 1.
    expect(env.observeAt(2.5, 5.5, 1, 0)[OBS.lidar]! * LIDAR_MAX).toBeCloseTo(6.5, 2);
    env.dispose();
  });

  it('smoke gives the lidar early returns and takes contrast off the camera', async () => {
    const spec = corridor();
    const floor = spec.floors[0]!;
    const mask = new Uint8Array(floor.w * floor.d);
    for (let z = 1; z < 9; z++) for (let x = 1; x < 5; x++) mask[z * floor.w + x] = 1;
    spec.hazards!.smoke = { visibility: 1.5, cells: [mask] };
    expect(inSmoke(spec, 0, 2.5, 5.5)).toBe(true);
    expect(smokePath(spec, 0, 2.5, 1.5, 2.5, 7.5)).toBeCloseTo(6, 5);
    expect(smokeTransmittance(1.5, 1.5)).toBeCloseTo(0.02, 2);
    const env = await createEnv();
    const clear = await createEnv();
    clear.reset(corridor(), 1, defaultEpisodeConfig());
    let obs = env.reset(spec, 1, defaultEpisodeConfig());
    // The same pose without smoke gives the true ranges; in 1.5 m smoke many beams come back short.
    let short = 0;
    let beams = 0;
    for (let i = 0; i < 20; i++) {
      const s = env.snapshot();
      const truth = clear.observeAt(s.pos[0], s.pos[1], s.pos[2], s.yaw);
      for (let b = 0; b < 32; b++) {
        beams += 1;
        if (obs[OBS.lidar + b]! < truth[OBS.lidar + b]! - 0.3 / LIDAR_MAX) short += 1;
      }
      obs = env.step(new Float32Array([0, 0, 0, 0.3])).obs;
    }
    env.dispose();
    clear.dispose();
    expect(short / beams).toBeGreaterThan(0.2);
    expect(short / beams).toBeLessThan(0.95);
  });
});

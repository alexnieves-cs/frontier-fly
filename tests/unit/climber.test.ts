import { beforeAll, describe, expect, it } from 'vitest';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { climbingPilot } from '../../src/eval/pilots';
import { buildingAabbs } from '../../src/gen/parts';
import { CELL } from '../../src/shared/constants';
import { shaftCentre } from '../../src/sim/climber';
import { createEnv } from '../../src/sim/env';
import { initPhysics, PhysicsWorld } from '../../src/sim/world';

beforeAll(async () => {
  await initPhysics();
});

describe('shafts', () => {
  it('open a slab exactly where the physics lets the drone through', () => {
    // Seed 9103 at stage 4 has three floors with shafts at different places between each pair.
    const { spec } = evalEpisodeSetup(4, 9103, 'day');
    expect(spec.floors.length).toBe(3);
    const world = new PhysicsWorld(spec);
    const slabs = buildingAabbs(spec).filter((b) => b.kind === 'slab');
    const cs = spec.cellSize;
    const H = spec.floorHeight;
    let holes = 0;
    for (let k = 1; k < spec.floors.length; k++) {
      const f = spec.floors[k]!;
      for (let gz = 0; gz < f.d; gz++) {
        for (let gx = 0; gx < f.w; gx++) {
          const x = (gx + 0.5) * cs;
          const y = (gz + 0.5) * cs;
          const slabbed = slabs.some((b) => Math.abs(b.cz - k * H) < 0.2 && Math.abs(x - b.cx) < b.hx && Math.abs(y - b.cy) < b.hy);
          const blocked = world.penetrates([x, y, k * H], 0.05);
          const shaft = f.cells[gz * f.w + gx] === CELL.SHAFT && spec.floors[k - 1]!.cells[gz * f.w + gx] === CELL.SHAFT;
          expect(slabbed, `slab ${k} cell ${gx},${gz}`).toBe(!shaft);
          if (f.cells[gz * f.w + gx] !== CELL.WALL) expect(blocked, `physics ${k} cell ${gx},${gz}`).toBe(!shaft);
          if (shaft) holes += 1;
        }
      }
    }
    expect(holes).toBeGreaterThan(1);
  });

  it('finds the centre of a marked column', () => {
    const vw = 10;
    const map = new Uint8Array(vw * vw);
    for (const [x, y] of [
      [6, 6],
      [7, 6],
      [6, 7],
      [7, 7],
    ])
      map[y * vw + x] = 1;
    expect(shaftCentre(map, 0, vw, vw, [0, 0])).toEqual([3.5, 3.5]);
    expect(shaftCentre(new Uint8Array(vw * vw), 0, vw, vw, [0, 0])).toBeNull();
  });
});

describe('climbing pilot', () => {
  it('reaches the second floor of a two-floor building without touching anything', async () => {
    const { spec, envSeed, cfg } = evalEpisodeSetup(4, 9101, 'day');
    const pilot = climbingPilot({}, { floorSteps: (0.8 * cfg.maxSteps) / spec.floors.length });
    pilot.reset();
    const env = await createEnv();
    let obs = env.reset(spec, envSeed, cfg);
    const floors = new Set<number>();
    let collisions = 0;
    for (let t = 0; t < cfg.maxSteps; t++) {
      const step = env.step(pilot.act(obs, env));
      obs = step.obs;
      floors.add(step.info.floor);
      collisions = step.info.collisions;
      if (step.done) break;
    }
    env.dispose();
    expect(floors.size).toBe(spec.floors.length);
    expect(pilot.climber.floorChanges).toBeGreaterThan(0);
    expect(collisions).toBe(0);
  }, 60_000);

  it('started on the middle floor, goes up and then back down past it to the bottom floor', async () => {
    const { spec, envSeed, cfg } = evalEpisodeSetup(4, 9103, 'day');
    expect(spec.floors.length).toBe(3);
    const room = spec.floors[1]!.rooms[0]!;
    spec.spawn = [(room.x + room.w / 2) * spec.cellSize, (room.z + room.d / 2) * spec.cellSize, spec.floorHeight + 1.2];
    cfg.maxSteps *= spec.floors.length;
    const pilot = climbingPilot({}, { floorSteps: (0.8 * cfg.maxSteps) / spec.floors.length });
    pilot.reset();
    const env = await createEnv();
    let obs = env.reset(spec, envSeed, cfg);
    const order: number[] = [];
    let info = { coverage: 0, collisions: 0 };
    for (let t = 0; t < cfg.maxSteps; t++) {
      const step = env.step(pilot.act(obs, env));
      obs = step.obs;
      info = step.info;
      if (order[order.length - 1] !== step.info.floor) order.push(step.info.floor);
      if (step.done) break;
    }
    env.dispose();
    // The bottom floor is two floors from the top one: the climber used to look one floor up and down only.
    expect(order.slice(0, 2)).toEqual([1, 2]);
    expect(order).toContain(0);
    expect(info.collisions).toBe(0);
    expect(info.coverage).toBeGreaterThan(0.7);
  }, 120_000);
});

import { describe, expect, it } from 'vitest';
import { DRONE_RADIUS } from '../../src/shared/constants';
import { measureClearance } from '../../src/world/outdoor/clearance';
import { COLLIDER_BUDGET, REQUIRED_CLEARANCE } from '../../src/world/outdoor/constants';
import { fingerprint } from '../../src/world/outdoor/fingerprint';
import { generateOutdoor } from '../../src/world/outdoor/generate';
import { containsPoint } from '../../src/world/outdoor/geofence';
import { initOutdoorPhysics, OutdoorPhysics } from '../../src/world/outdoor/physics';
import { shellAabb, treeHitsAabb } from '../../src/world/outdoor/forest';
import { entrancePaths, validateOutdoor } from '../../src/world/outdoor/validate';

describe('opening clearance', () => {
  it('accepts apertures the airframe can pass and rejects decoys', () => {
    const vent = measureClearance({ kind: 'vent', width: 0.18, height: 0.12, angle: 0 });
    expect(vent.passable).toBe(false);
    expect(vent.clearWidth).toBeLessThan(REQUIRED_CLEARANCE);

    const window = measureClearance({ kind: 'open_window', width: 0.8, height: 1, angle: 0 });
    expect(window.passable).toBe(true);
    expect(window.clearWidth).toBeGreaterThanOrEqual(REQUIRED_CLEARANCE);
    expect(window.clearHeight).toBeGreaterThanOrEqual(REQUIRED_CLEARANCE);

    const tight = measureClearance({ kind: 'open_window', width: REQUIRED_CLEARANCE - 0.04, height: 1.2, angle: 0 });
    expect(tight.passable).toBe(false);

    const closed = measureClearance({ kind: 'closed_door', width: 0.9, height: 2, angle: 0 });
    expect(closed.passable).toBe(false);
    expect(closed.clearWidth).toBe(0);

    const ajar = measureClearance({ kind: 'ajar_door', width: 0.9, height: 2, angle: (10 * Math.PI) / 180 });
    expect(ajar.passable).toBe(false);

    const open = measureClearance({ kind: 'ajar_door', width: 0.9, height: 2, angle: (80 * Math.PI) / 180 });
    expect(open.passable).toBe(true);
    expect(open.clearWidth).toBeGreaterThanOrEqual(REQUIRED_CLEARANCE);

    const hatch = measureClearance({ kind: 'roof_hatch', width: 0.75, height: 0.75, angle: 0 });
    expect(hatch.passable).toBe(true);
    const smallHatch = measureClearance({ kind: 'roof_hatch', width: 0.22, height: 0.7, angle: 0 });
    expect(smallHatch.passable).toBe(false);

    const broken = measureClearance({ kind: 'broken_window', width: 0.2, height: 0.18, angle: 0 });
    expect(broken.passable).toBe(false);
    const garage = measureClearance({ kind: 'garage_gap', width: 1.6, height: 1.7, angle: 0 });
    expect(garage.passable).toBe(true);
    expect(REQUIRED_CLEARANCE).toBeGreaterThan(DRONE_RADIUS * 2);
  });
});

describe('outdoor world', () => {
  it('is deterministic per seed and changes with the seed', () => {
    const a = fingerprint(generateOutdoor({ seed: 4 }));
    const b = fingerprint(generateOutdoor({ seed: 4 }));
    const c = fingerprint(generateOutdoor({ seed: 5 }));
    expect(a).toBe(b);
    expect(a).not.toBe(c);
  });

  it('keeps every building reachable, clear of trees, and inside the collider budget', () => {
    for (const seed of [1, 4, 7, 11]) {
      const world = generateOutdoor({ seed });
      expect(validateOutdoor(world)).toEqual([]);
      expect(world.colliders.length).toBeLessThanOrEqual(COLLIDER_BUDGET);
      expect(world.buildings.length).toBeGreaterThanOrEqual(1);
      expect(world.noFly.length).toBeGreaterThanOrEqual(1);
      expect(world.geofence.polygon.length).toBeGreaterThanOrEqual(8);
      const paths = entrancePaths(world);
      expect(paths).toHaveLength(world.buildings.length);
      for (const shell of world.buildings) {
        expect(shell.openings.some((opening) => opening.passable)).toBe(true);
        expect(shell.spec.floors.length).toBeGreaterThanOrEqual(1);
        for (const tree of world.trees) expect(treeHitsAabb(tree, shellAabb(shell, 0))).toBe(false);
      }
      const kinds = new Set(world.buildings.flatMap((shell) => shell.openings.map((opening) => opening.kind)));
      expect(kinds.size).toBeGreaterThanOrEqual(2);
      const decoy = world.buildings.some((shell) => shell.openings.some((opening) => opening.decoy));
      expect(decoy).toBe(true);
    }
  });
});

describe('outdoor physics', () => {
  it('casts lidar, fits a passable opening, and stops on a decoy', async () => {
    await initOutdoorPhysics();
    const world = generateOutdoor({ seed: 4 });
    const physics = new OutdoorPhysics(world);
    const again = new OutdoorPhysics(world);
    try {
      const down = physics.cast(world.spawn[0], world.spawn[1], world.spawn[2], 0, 0, -1, 8);
      expect(down).not.toBeNull();
      expect(down!.tag).toBe('terrain');
      expect(down!.dist).toBeGreaterThan(0.5);
      expect(down!.dist).toBeLessThan(3);
      const other = again.cast(world.spawn[0], world.spawn[1], world.spawn[2], 0, 0, -1, 8);
      expect(other!.dist).toBeCloseTo(down!.dist, 4);

      const ranges = physics.lidar(world.spawn, world.spawnYaw);
      expect(ranges).toHaveLength(40);
      for (const value of ranges) {
        expect(Number.isFinite(value)).toBe(true);
        expect(value).toBeGreaterThanOrEqual(0);
        expect(value).toBeLessThanOrEqual(1);
      }
      expect(ranges.some((value) => value < 0.98)).toBe(true);

      const passable = world.buildings
        .flatMap((shell) => shell.openings)
        .filter((opening) => opening.passable && opening.kind !== 'roof_hatch')
        .sort((a, b) => b.clearWidth - a.clearWidth)[0];
      expect(passable).toBeTruthy();
      const outside = stepThrough(physics, passable!.center, passable!.normal, 1);
      expect(outside).toBeLessThan(-0.15);

      const decoy = world.buildings
        .flatMap((shell) => shell.openings)
        .find((opening) => opening.decoy && opening.kind !== 'roof_hatch');
      expect(decoy).toBeTruthy();
      const stuck = stepThrough(physics, decoy!.center, decoy!.normal, 1);
      expect(stuck).toBeGreaterThan(0.02);

      const fled = physics.move(world.spawn, [400, 0, 0]);
      expect(containsPoint(world.geofence, fled.pos[0], fled.pos[1])).toBe(true);
      expect(Math.hypot(fled.pos[0] - world.spawn[0], fled.pos[1] - world.spawn[1])).toBeLessThan(world.size);
    } finally {
      physics.free();
      again.free();
    }
  });
});

function stepThrough(
  physics: OutdoorPhysics,
  center: [number, number, number],
  normal: [number, number, number],
  start: number,
): number {
  let pos: [number, number, number] = [
    center[0] + normal[0] * start,
    center[1] + normal[1] * start,
    center[2],
  ];
  const step: [number, number, number] = [-normal[0] * 0.12, -normal[1] * 0.12, 0];
  for (let i = 0; i < 24; i++) pos = physics.move(pos, step).pos;
  const dx = pos[0] - center[0];
  const dy = pos[1] - center[1];
  const dz = pos[2] - center[2];
  return dx * normal[0] + dy * normal[1] + dz * normal[2];
}

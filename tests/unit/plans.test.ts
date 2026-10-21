import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { isFullyReachable } from '../../src/gen/validate';
import { parsePlanSet, PLAN_SET_PATH, planEpisodeSetup, planToBuilding, type PlanRecord } from '../../src/plans';
import { cutAxis } from '../../src/plans/convert';
import { axisAlignedShare, dominantRotation, rotateRing } from '../../src/plans/geometry';
import { CELL } from '../../src/shared/constants';
import { buildingToJson } from '../../src/shared/serialize';
import { createEnv } from '../../src/sim/env';

const rect = (x0: number, y0: number, x1: number, y1: number) => [x0, y0, x1, y0, x1, y1, x0, y1];

/**
 * Two 4 × 3 m rooms side by side, 15 cm walls, a 0.9 m door in the shared wall
 * and a 1.2 m corridor along the bottom joined to the left room by another door.
 */
function twoRooms(): PlanRecord {
  const t = 0.15;
  return {
    id: 'test-two-rooms',
    split: 'dev',
    source: { dataset: 'test', site: 0, building: 0, floor: 0, apartment: 'x' },
    rotation: 0,
    areas: [
      { type: 'ROOM', poly: rect(t, 1.2 + 2 * t, 4 + t, 4.2 + 2 * t) },
      { type: 'ROOM', poly: rect(4 + 2 * t, 1.2 + 2 * t, 8 + 2 * t, 4.2 + 2 * t) },
      { type: 'CORRIDOR', poly: rect(t, t, 8 + 2 * t, 1.2 + t) },
    ],
    walls: [
      rect(0, 0, 8 + 3 * t, t),
      rect(0, 4.2 + 2 * t, 8 + 3 * t, 4.2 + 3 * t),
      rect(0, 0, t, 4.2 + 3 * t),
      rect(8 + 2 * t, 0, 8 + 3 * t, 4.2 + 3 * t),
      rect(4 + t, 1.2 + t, 4 + 2 * t, 4.2 + 2 * t),
      rect(t, 1.2 + t, 8 + 2 * t, 1.2 + 2 * t),
    ],
    doors: [rect(4 + t, 2.4, 4 + 2 * t, 3.3), rect(1.5, 1.2 + t, 2.4, 1.2 + 2 * t)],
    windows: [rect(1, 4.2 + 2 * t, 3, 4.2 + 3 * t)],
  };
}

describe('axis cut', () => {
  it('gives each thin wall its own cell and keeps a cell for a 1.2 m corridor', () => {
    const band = (a: number, b: number) => ({ a, b, c0: 0, c1: 10 });
    const axis = cutAxis(0, 1.5, [band(0, 0.15), band(1.35, 1.5)], []);
    const kinds = axis.walls.map((w) => (w.length ? 'W' : '.')).join('');
    expect(kinds).toMatch(/W\.+W/);
    expect(axis.centers.filter((_, k) => axis.walls[k]!.length === 0).every((c) => c > 0.15 && c < 1.35)).toBe(true);
  });

  it('stays close to metric for a 4 m room', () => {
    const band = (a: number, b: number) => ({ a, b, c0: 0, c1: 10 });
    const axis = cutAxis(0, 4.3, [band(0, 0.15), band(4.15, 4.3)], []);
    expect(axis.centers.length).toBeGreaterThanOrEqual(4);
    expect(axis.centers.length).toBeLessThanOrEqual(6);
  });

  it('lets walls that do not face each other share a cell', () => {
    const axis = cutAxis(0, 2, [{ a: 1, b: 1.1, c0: 0, c1: 3 }, { a: 1.3, b: 1.4, c0: 4, c1: 8 }], []);
    expect(axis.walls.some((w) => w.length === 2)).toBe(true);
  });
});

describe('plan to building', () => {
  it('converts a synthetic plan into reachable rooms joined by doors', () => {
    const conv = planToBuilding(twoRooms())!;
    expect(conv).not.toBeNull();
    const floor = conv.spec.floors[0]!;
    expect(floor.rooms).toHaveLength(3);
    expect(conv.roomTypes.sort()).toEqual(['CORRIDOR', 'ROOM', 'ROOM']);
    expect(floor.cells.filter((c) => c === CELL.DOOR).length).toBeGreaterThanOrEqual(2);
    expect(isFullyReachable(conv.spec)).toBe(true);
    // 8.3 × 4.65 m: within a few cells of metric.
    expect(Math.abs(conv.walkableCells - conv.planArea)).toBeLessThan(0.35 * conv.planArea);
    expect(floor.surfaces!.some((s) => s === 3)).toBe(true);
  });

  it('walls the door off when the plan has none', () => {
    const plan = { ...twoRooms(), doors: [] };
    const conv = planToBuilding(plan, { furniture: false })!;
    // Only the room the drone starts in (the largest component) survives.
    expect(conv.spec.floors[0]!.rooms.length).toBe(1);
  });

  it('is deterministic', () => {
    const a = buildingToJson(planToBuilding(twoRooms())!.spec);
    const b = buildingToJson(planToBuilding(twoRooms())!.spec);
    expect(a).toEqual(b);
  });
});

describe('geometry', () => {
  it('recovers the rotation of a rotated plan', () => {
    const rings = twoRooms().walls.map((r) => rotateRing(r, 0.4));
    const back = dominantRotation(rings);
    const fixed = rings.map((r) => rotateRing(r, back));
    expect(axisAlignedShare(fixed, 0.01)).toBeGreaterThan(0.99);
  });
});

describe('shipped plan set', () => {
  const set = parsePlanSet(JSON.parse(readFileSync(PLAN_SET_PATH, 'utf8')));

  it('carries its license and attribution', () => {
    expect(set.dataset.license).toBe('CC BY 4.0');
    expect(set.dataset.doi).toMatch(/^10\.5281\/zenodo\./);
    expect(set.dataset.attribution.length).toBeGreaterThan(20);
  });

  it('has 20 dev and 60 eval plans from distinct buildings', () => {
    expect(set.plans.filter((p) => p.split === 'dev')).toHaveLength(20);
    expect(set.plans.filter((p) => p.split === 'eval')).toHaveLength(60);
    expect(new Set(set.plans.map((p) => p.id)).size).toBe(80);
    expect(new Set(set.plans.map((p) => p.source.building)).size).toBe(80);
  });

  it('converts every plan into a reachable stage-3-sized building', () => {
    for (const plan of set.plans) {
      const conv = planToBuilding(plan);
      expect(conv, plan.id).not.toBeNull();
      const floor = conv!.spec.floors[0]!;
      expect(floor.w).toBeLessThanOrEqual(28);
      expect(floor.d).toBeLessThanOrEqual(28);
      expect(floor.rooms.length).toBeGreaterThanOrEqual(Math.max(3, plan.areas.length - 1));
      expect(isFullyReachable(conv!.spec)).toBe(true);
      expect(conv!.spec.reachableVoxels).toBeGreaterThan(0);
    }
  });

  it('runs an episode on a real plan', async () => {
    const plan = set.plans.find((p) => p.split === 'eval')!;
    const { spec, envSeed, cfg } = planEpisodeSetup(set, plan, 'day');
    expect(envSeed).toBe(72001);
    const env = await createEnv();
    env.reset(spec, envSeed, cfg);
    let coverage = 0;
    for (let t = 0; t < 50; t++) coverage = env.step(new Float32Array([0.5, 0, 0, 0.2])).info.coverage;
    expect(coverage).toBeGreaterThan(0);
    env.dispose();
  });
});

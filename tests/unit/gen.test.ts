import { readFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { isFullyReachable } from '../../src/gen/validate';
import { CELL } from '../../src/shared/constants';
import { buildingToJson } from '../../src/shared/serialize';
import { RNG } from '../../src/shared/rng';

describe('building generator', () => {
  it('is byte-identical for the same seed', () => {
    const params = { seed: 42, floors: 2, w: 18, d: 16, stage: 4 };
    const a = buildingToJson(generateBuilding(params));
    const b = buildingToJson(generateBuilding(params));
    expect(a).toEqual(b);
  });

  it('keeps 1000 random seeds valid and fully reachable', () => {
    for (let i = 0; i < 1000; i++) {
      const rng = new RNG(i + 3, 'sweep');
      const stage = rng.int(6);
      const floors = stage >= 4 ? 2 + rng.int(2) : 1;
      const w = 8 + rng.int(10);
      const d = 8 + rng.int(10);
      const spec = generateBuilding({ seed: 10_000 + i, floors, w, d, stage });
      expect(isFullyReachable(spec)).toBe(true);
      expect(spec.reachableVoxels).toBeGreaterThan(0);
      expect(spec.floors).toHaveLength(floors);
      if (stage >= 2 && w >= 14 && d >= 14) {
        expect(spec.floors[0]!.rooms.length).toBeGreaterThanOrEqual(2);
      }
      if (floors > 1) {
        const shafts = spec.floors[0]!.cells.some((cell) => cell === CELL.SHAFT);
        expect(shafts).toBe(true);
      }
    }
  });

  it('generates a 3×40×40 building in under 50 ms', () => {
    generateBuilding({ seed: 7, floors: 3, w: 40, d: 40, stage: 4 });
    const t0 = performance.now();
    generateBuilding({ seed: 8, floors: 3, w: 40, d: 40, stage: 4 });
    expect(performance.now() - t0).toBeLessThan(50);
  });

  it('does not call Math.random', () => {
    const files = ['building.ts', 'bsp.ts', 'connect.ts', 'shafts.ts', 'validate.ts', 'parts.ts'];
    for (const file of files) {
      const src = readFileSync(new URL(`../../src/gen/${file}`, import.meta.url), 'utf8');
      expect(src.includes('Math.random')).toBe(false);
    }
  });
});

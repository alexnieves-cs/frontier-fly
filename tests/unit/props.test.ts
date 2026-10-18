import { describe, expect, it } from 'vitest';
import { generateBuilding } from '../../src/gen/building';
import { CELL } from '../../src/shared/constants';
import { planProps } from '../../src/viz/props';

describe('furniture layout', () => {
  it('keeps props inside empty room cells', () => {
    const spec = generateBuilding({ seed: 9, floors: 2, w: 18, d: 16, stage: 3 });
    const props = planProps(spec);
    expect(props.length).toBeGreaterThan(2);
    for (const prop of props) {
      const floor = spec.floors[prop.floor]!;
      const gx = Math.floor(prop.x / spec.cellSize);
      const gz = Math.floor(prop.z / spec.cellSize);
      expect(gx).toBeGreaterThanOrEqual(0);
      expect(gz).toBeGreaterThanOrEqual(0);
      expect(floor.cells[gz * floor.w + gx]).toBe(CELL.EMPTY);
      expect(prop.y).toBeCloseTo(prop.floor * spec.floorHeight);
    }
  });
});

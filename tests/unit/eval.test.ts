import { describe, expect, it } from 'vitest';
import { LIGHTING_PRESETS } from '../../src/sim/lighting';
import { runEvalMatrix } from '../../src/eval/matrix';

describe('eval matrix', () => {
  it('runs headless and reports every lighting cell', async () => {
    const report = await runEvalMatrix({ seeds: [4], steps: 30, dynamics: ['kinematic'] });
    expect(report.cells).toHaveLength(LIGHTING_PRESETS.length);
    for (const name of LIGHTING_PRESETS) {
      expect(report.cells.some((cell) => cell.lighting === name && cell.dynamics === 'kinematic')).toBe(true);
    }
    for (const cell of report.cells) {
      expect(cell.coverage).toBeGreaterThanOrEqual(0);
      expect(cell.coverage).toBeLessThanOrEqual(1);
      expect(Number.isFinite(cell.collisions)).toBe(true);
    }
    expect(report.nightToDay === null || Number.isFinite(report.nightToDay)).toBe(true);
  });
});

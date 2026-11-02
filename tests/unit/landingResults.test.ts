import { describe, expect, it } from 'vitest';
import study from '../../docs/study.json';
import { pct, results } from '../../src/landing/results';

describe('landing results', () => {
  it('computes finite headline numbers from the coverage study', () => {
    const r = results();
    expect(r.pilots).toHaveLength(6);
    for (const p of r.pilots) {
      expect(Number.isFinite(p.value)).toBe(true);
      expect(Number.isFinite(p.contacts)).toBe(true);
      expect(p.ci[0]).toBeLessThanOrEqual(p.value);
      expect(p.ci[1]).toBeGreaterThanOrEqual(p.value);
    }
    expect(r.seeds).toBe(study.seedsPerCondition);
    expect(r.nightOverDay).toBeGreaterThan(0);
    expect(pct(0.8081)).toBe('80.8%');
  });

  it('leads with the 30-fresh-building guarded-policy cell, the number the README headline quotes', () => {
    const cell = study.cells.find((c) => c.condition === 'hard' && c.lighting === 'day' && c.pilot === 'guarded')!;
    expect(results().pilots[0]!.value).toBe(cell.coverage);
    expect(study.seedsPerCondition).toBeGreaterThanOrEqual(30);
  });

  it('shows the zero-contact sensor-only planner beside the policy', () => {
    const names = results().pilots.map((p) => p.name);
    expect(names.some((n) => /sensor-only planner/i.test(n))).toBe(true);
  });
});

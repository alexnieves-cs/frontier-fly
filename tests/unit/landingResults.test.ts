import { describe, expect, it } from 'vitest';
import { pct, results } from '../../src/landing/results';

describe('landing results', () => {
  it('computes finite headline numbers from the benchmark file', () => {
    const r = results();
    expect(r.pilots).toHaveLength(4);
    for (const p of r.pilots) expect(Number.isFinite(p.value)).toBe(true);
    expect(r.hardSeeds).toBeGreaterThan(0);
    expect(r.nightOverDay).toBeGreaterThan(0);
    expect(pct(0.8081)).toBe('80.8%');
  });
});

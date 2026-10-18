import { describe, expect, it } from 'vitest';
import { resolveQuality } from '../../src/viz/quality';

describe('quality preset', () => {
  it('auto-picks from the adapter and honors an explicit preset', () => {
    expect(resolveQuality('auto', true)).toBe('high');
    expect(resolveQuality('auto', false)).toBe('low');
    expect(resolveQuality('low', true)).toBe('low');
    expect(resolveQuality('high', false)).toBe('high');
  });
});
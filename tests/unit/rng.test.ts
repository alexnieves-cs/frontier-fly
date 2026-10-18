import { describe, expect, it } from 'vitest';
import { RNG } from '../../src/shared/rng';
import { COOP_COEP_HEADERS } from '../../coopHeaders';

describe('sfc32 RNG', () => {
  it('reproduces the same stream for a seed', () => {
    const a = new RNG(42);
    const b = new RNG(42);
    const seqA = Array.from({ length: 20 }, () => a.next());
    const seqB = Array.from({ length: 20 }, () => b.next());
    expect(seqA).toEqual(seqB);
    expect(new Set(seqA).size).toBeGreaterThan(10);
  });

  it('fork(label) is deterministic and independent of parent consumption', () => {
    const a = new RNG(7, 'root');
    a.next();
    a.next();
    const childA = a.fork('lidar');
    const b = new RNG(7, 'root');
    const childB = b.fork('lidar');
    expect(Array.from({ length: 8 }, () => childA.next())).toEqual(
      Array.from({ length: 8 }, () => childB.next()),
    );
    const other = new RNG(7, 'root').fork('doors');
    expect(other.next()).not.toBe(childB.next());
  });

  it('does not use Math.random', async () => {
    const src = await import('node:fs/promises').then((fs) =>
      fs.readFile(new URL('../../src/shared/rng.ts', import.meta.url), 'utf8'),
    );
    expect(src.includes('Math.random')).toBe(false);
  });
});

describe('COOP/COEP', () => {
  it('sets same-origin opener and require-corp embedder policies', () => {
    expect(COOP_COEP_HEADERS['Cross-Origin-Opener-Policy']).toBe('same-origin');
    expect(COOP_COEP_HEADERS['Cross-Origin-Embedder-Policy']).toBe('require-corp');
  });
});

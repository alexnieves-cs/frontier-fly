import { describe, expect, it } from 'vitest';
import { Curriculum } from '../../src/rl/curriculum';
import { RNG } from '../../src/shared/rng';

function push(c: Curriculum, n: number, coverage: number, reachedNewFloor: boolean): void {
  for (let i = 0; i < n; i++) c.record({ coverage, reachedNewFloor });
}

describe('curriculum promotion', () => {
  it('promotes on a full window and stays put below the gate', () => {
    const c = new Curriculum();
    expect(c.stage).toBe(0);
    push(c, 49, 0.9, false);
    expect(c.stage).toBe(0);
    c.record({ coverage: 0.9, reachedNewFloor: false });
    expect(c.stage).toBe(1);

    push(c, 50, 0.8, false);
    expect(c.stage).toBe(2);

    const below = new Curriculum();
    push(below, 50, 0.5, false);
    expect(below.stage).toBe(0);

    const atGate = new Curriculum();
    push(atGate, 50, 0.8, true);
    expect(atGate.stage).toBe(0);

    const short = new Curriculum({ window: 2 });
    short.record({ coverage: 0.99, reachedNewFloor: true });
    expect(short.stage).toBe(0);
    short.record({ coverage: 0.99, reachedNewFloor: true });
    expect(short.stage).toBe(1);
  });

  it('promotes one stage at a time and requires floor reach at stage 4', () => {
    const one = new Curriculum({ window: 1 });
    one.record({ coverage: 1, reachedNewFloor: true });
    expect(one.stage).toBe(1);
    one.record({ coverage: 1, reachedNewFloor: true });
    expect(one.stage).toBe(2);

    const c = new Curriculum();
    push(c, 50, 0.9, false);
    expect(c.stage).toBe(1);
    push(c, 50, 0.8, false);
    expect(c.stage).toBe(2);
    push(c, 50, 0.71, false);
    expect(c.stage).toBe(3);
    push(c, 50, 0.66, false);
    expect(c.stage).toBe(4);

    push(c, 50, 0.9, false);
    expect(c.stage).toBe(4);

    for (let i = 0; i < 25; i++) c.record({ coverage: 0.9, reachedNewFloor: false });
    for (let i = 0; i < 25; i++) c.record({ coverage: 0.9, reachedNewFloor: true });
    expect(c.stage).toBe(4);
    c.record({ coverage: 0.9, reachedNewFloor: true });
    expect(c.stage).toBe(5);

    push(c, 50, 0.99, true);
    expect(c.stage).toBe(5);
  });

  it('mixes about 20% earlier stages after stage 0', () => {
    const c = new Curriculum();
    push(c, 50, 0.9, false);
    push(c, 50, 0.8, false);
    expect(c.stage).toBe(2);
    const rng = new RNG(12345);
    const n = 4000;
    let earlier = 0;
    for (let i = 0; i < n; i++) {
      const stage = c.sampleStage(rng);
      expect(stage).toBeGreaterThanOrEqual(0);
      expect(stage).toBeLessThanOrEqual(2);
      if (stage < 2) earlier += 1;
    }
    const frac = earlier / n;
    expect(frac).toBeGreaterThan(0.15);
    expect(frac).toBeLessThan(0.25);

    const root = new Curriculum();
    const rng0 = new RNG(1);
    for (let i = 0; i < 200; i++) expect(root.sampleStage(rng0)).toBe(0);
  });

  it('builds generation params and dynamics per stage', () => {
    const c = new Curriculum();
    const rng = new RNG(7);
    expect(c.genParams(0, rng)).toMatchObject({ floors: 1, w: 8, d: 8, stage: 0 });
    expect(c.genParams(1, rng)).toMatchObject({ floors: 1, w: 10, d: 10, stage: 1 });
    expect(c.genParams(2, rng)).toMatchObject({ floors: 1, w: 16, d: 16, stage: 2 });
    expect(c.genParams(3, rng)).toMatchObject({ floors: 1, w: 20, d: 20, stage: 3 });
    const floors = new Set<number>();
    for (let i = 0; i < 40; i++) {
      const params = c.genParams(4, new RNG(100 + i));
      floors.add(params.floors);
      expect(params.w).toBe(16);
      expect(params.d).toBe(16);
      expect(params.stage).toBe(4);
      expect(params.seed).toBeGreaterThanOrEqual(0);
      expect(params.seed).toBeLessThan(1_000_000_000);
    }
    expect([...floors].sort()).toEqual([2, 3]);
    expect(c.genParams(5, rng)).toMatchObject({ floors: 2, w: 18, d: 18, stage: 5 });

    expect(c.dynamicsFor(0, rng)).toBe('kinematic');
    expect(c.dynamicsFor(2, rng)).toBe('kinematic');
    expect(c.dynamicsFor(5, rng)).toBe('quad');
    const n = 4000;
    let quad3 = 0;
    const rng3 = new RNG(99);
    for (let i = 0; i < n; i++) if (c.dynamicsFor(3, rng3) === 'quad') quad3 += 1;
    expect(quad3 / n).toBeGreaterThan(0.25);
    expect(quad3 / n).toBeLessThan(0.41);
    let quad4 = 0;
    const rng4 = new RNG(99);
    for (let i = 0; i < n; i++) if (c.dynamicsFor(4, rng4) === 'quad') quad4 += 1;
    expect(quad4 / n).toBeGreaterThan(0.58);
    expect(quad4 / n).toBeLessThan(0.74);
  });
});

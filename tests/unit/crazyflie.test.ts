import { describe, expect, it } from 'vitest';
import { castRay, arenaSegments, generateArena, reachableCells, validateArena, type Arena } from '../../src/crazyflie/arena';
import { CrazyflieExplorer, PAD } from '../../src/crazyflie/explorer';
import { flyArena } from '../../src/crazyflie/sim';
import * as C from '../../src/shared/constants';

const box: Arena = { name: 'box', w: 4, d: 3, walls: [], boxes: [], start: { x: 1, y: 1, yaw: 0 } };

describe('crazyflie arena', () => {
  it('casts rays to the walls', () => {
    const s = arenaSegments(box);
    expect(castRay(s, 1, 1, 0)).toBeCloseTo(3, 6);
    expect(castRay(s, 1, 1, Math.PI / 2)).toBeCloseTo(2, 6);
  });

  it('generates connected arenas deterministically', () => {
    for (let seed = 1; seed <= 10; seed++) {
      const a = generateArena(seed);
      expect(generateArena(seed)).toEqual(a);
      expect(validateArena(a)).toEqual([]);
      expect(reachableCells(a).count).toBeGreaterThan(50);
    }
  });

  it('rejects a take-off spot next to a wall', () => {
    expect(validateArena({ ...box, start: { x: 0.2, y: 1, yaw: 0 } })).not.toEqual([]);
    expect(validateArena({ w: 'x' })).not.toEqual([]);
  });
});

describe('crazyflie explorer', () => {
  it('maps only along the four beams and blocks the outside of the arena', () => {
    const e = new CrazyflieExplorer(box, { spinSec: 0 });
    e.step({ t: 0, x: 1, y: 1, z: 0.5, yaw: 0, vx: 0, vy: 0, front: 3, back: 1, left: 2, right: 1, up: Infinity });
    const k = (x: number, y: number) => Math.floor((y + PAD) / C.VOXEL) * e.vw + Math.floor((x + PAD) / C.VOXEL);
    expect(e.seen[k(2.5, 1.2)]).toBe(1); // ahead, along the front beam
    expect(e.seen[k(2.5, 2.5)]).toBe(0); // off every beam
    expect(e.occupied[k(-0.5, 1)]).toBe(1); // outside the box
  });

  it('takes its first look around before it moves', () => {
    const e = new CrazyflieExplorer(box);
    const first = e.step({ t: 0, x: 1, y: 1, z: 0.5, yaw: 0, vx: 0, vy: 0, front: 3, back: 1, left: 2, right: 1, up: Infinity });
    expect(first.phase).toBe('spin');
    expect(first.vx).toBe(0);
  });
});

describe('crazyflie sim', () => {
  it('explores generated arenas without touching anything', () => {
    for (const seed of [1, 2, 3]) {
      const f = flyArena(generateArena(seed), { seed });
      expect(f.outcome).toBe('done');
      expect(f.minClearance).toBeGreaterThan(0);
      expect(f.coverage).toBeGreaterThan(0.6);
    }
  });
});

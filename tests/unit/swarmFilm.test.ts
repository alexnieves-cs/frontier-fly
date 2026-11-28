import { readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { CONTACT_M, Track, type SwarmBundle } from '../../src/swarm/film';

const PER = 18;

/** A two-drone film on a 4 x 4 cell floor. Drone 1 sits still; drone 2 flies past it along x. */
function bundle(closest: number): SwarmBundle {
  const frames: number[][] = [];
  for (let k = 0; k <= 40; k++) {
    const t = k * 0.1;
    const row = [t, Math.min(1, k / 40), 0];
    const drone = (x: number, y: number): number[] => {
      const v = new Array<number>(PER).fill(0);
      v[0] = x;
      v[1] = y;
      v[2] = 1;
      v[13] = 1; // launched
      return v;
    };
    row.push(...drone(2, 2), ...drone(-2 + k * 0.1, 2 + closest));
    frames.push(row);
  }
  return {
    schema: 1,
    id: 'test',
    cond: 'hard',
    condLabel: 'test',
    size: 4,
    budgetS: 4,
    seed: 1,
    drones: 2,
    dt: 0.1,
    autopilot: 'test',
    frames,
    flown: [],
    result: { coverage: 1, t50: 2, t80: 3.2, baseKnown: 0, collisions: 0, crashes: 0, droneContacts: 0, steps: 80 },
    spec: { cellSize: 1, floorHeight: 3, floors: [{ w: 4, d: 4, cells: new Uint8Array(16) }] } as unknown as SwarmBundle['spec'],
    base: [0, 0, 0],
    solo: null,
    sameBuilding: [],
    study: { split: 'test', buildings: 0, rows: [] },
  };
}

describe('swarm film', () => {
  it('marks one close pass at its closest point', () => {
    const track = new Track(bundle(0.2));
    const passes = track.events.filter((e) => e.warn);
    expect(passes).toHaveLength(1);
    expect(passes[0]!.pair).toEqual([0, 1]);
    expect(passes[0]!.t).toBeCloseTo(4, 5);
    expect(passes[0]!.text).toContain('0.20 m');
  });

  it('marks no pass when the drones stay apart', () => {
    const track = new Track(bundle(CONTACT_M + 0.05));
    expect(track.events.some((e) => e.warn)).toBe(false);
  });

  it('finds the half and 80% marks from the coverage column', () => {
    const track = new Track(bundle(1));
    expect(track.t50).toBeCloseTo(2, 5);
    expect(track.t80).toBeCloseTo(3.2, 5);
  });

  it('agrees with the study on which shipped films had a close pass', () => {
    const dir = join(__dirname, '../../public/swarm');
    const files = readdirSync(dir).filter((f) => f.endsWith('.json'));
    expect(files.length).toBeGreaterThan(0);
    for (const f of files) {
      const b = JSON.parse(readFileSync(join(dir, f), 'utf8')) as SwarmBundle;
      const passes = new Track(b).events.filter((e) => e.warn).length;
      // The film keeps every other 50 ms step, so a pass of one step can fall between frames;
      // a pass the film shows must still be one the study counted.
      if (passes > 0) expect(b.result.droneContacts, f).toBeGreaterThan(0);
      if (b.result.droneContacts >= 4) expect(passes, f).toBeGreaterThan(0);
    }
  });
});

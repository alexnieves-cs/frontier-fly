import { describe, expect, it } from 'vitest';
import { runDuo, TeamBoard, TWO_WAYS_IN } from '../../src/mission/autonomy/duo';
import type { MeasuredOpening } from '../../src/mission/autonomy/openingSensor';
import { fingerprint } from '../../src/world/outdoor/fingerprint';
import { generateOutdoor } from '../../src/world/outdoor/generate';
import type { Opening } from '../../src/world/outdoor/types';
import { validateOutdoor } from '../../src/world/outdoor/validate';

const SEEDS = [3, 20, 41, 107, 139, 158];

function face(opening: Opening): string {
  const [x, y, z] = opening.normal;
  if (z !== 0) return 'roof';
  if (Math.abs(x) > Math.abs(y)) return x > 0 ? 'e' : 'w';
  return y > 0 ? 'n' : 's';
}

function segmentDistance(px: number, py: number, ax: number, ay: number, bx: number, by: number): number {
  const dx = bx - ax;
  const dy = by - ay;
  const t = Math.max(0, Math.min(1, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)));
  return Math.hypot(px - (ax + dx * t), py - (ay + dy * t));
}

describe('a second way in', () => {
  it('leaves the default world exactly as it was', () => {
    for (const seed of SEEDS) {
      expect(fingerprint(generateOutdoor({ seed, entrances: 1 }))).toBe(fingerprint(generateOutdoor({ seed })));
    }
  });

  it('adds a passable opening of another kind on another face, after the first opening and decoys', () => {
    for (const seed of SEEDS) {
      const one = generateOutdoor({ seed, buildings: 1 });
      const two = generateOutdoor({ seed, buildings: 1, entrances: 2 });
      expect(validateOutdoor(two)).toEqual([]);
      for (const [index, shell] of two.buildings.entries()) {
        const before = one.buildings[index]!.openings;
        // The first opening and the decoys keep their ids, kinds and places.
        for (const old of before) {
          const same = shell.openings.find((o) => o.id === old.id)!;
          expect([same.kind, same.passable, same.center]).toEqual([old.kind, old.passable, old.center]);
        }
        const passable = shell.openings.filter((o) => o.passable);
        expect(passable.length).toBe(2);
        const [first, second] = passable as [Opening, Opening];
        expect(second.kind).not.toBe(first.kind);
        expect(face(second)).not.toBe('roof');
        expect(face(second)).not.toBe(face(first));
      }
    }
  });

  it('keeps trees off the lane in front of the second way in, also when the first is a roof hatch', () => {
    let hatchFirst = 0;
    for (const seed of [...SEEDS, 120, 131, 144]) {
      const world = generateOutdoor({ seed, buildings: 1, entrances: 2 });
      for (const shell of world.buildings) {
        const [first, second] = shell.openings.filter((o) => o.passable) as [Opening, Opening];
        if (face(first) === 'roof') hatchFirst++;
        const ax = second.center[0] + second.normal[0] * 0.5;
        const ay = second.center[1] + second.normal[1] * 0.5;
        const bx = second.center[0] + second.normal[0] * 7;
        const by = second.center[1] + second.normal[1] * 7;
        for (const tree of world.trees) {
          if (tree.kind === 'undergrowth') continue;
          const reach = Math.max(tree.trunkRadius, tree.canopyRadius);
          expect(segmentDistance(tree.x, tree.y, ax, ay, bx, by)).toBeGreaterThanOrEqual(1.8 + reach - 1e-9);
        }
      }
    }
    // Seed 139 puts a roof hatch first; the lane must not depend on the first opening being in a wall.
    expect(hatchFirst).toBeGreaterThan(0);
  });
});

describe('team board', () => {
  const opening = (id: number): MeasuredOpening => ({ id }) as unknown as MeasuredOpening;

  it('shows each drone only its teammate’s claims, standoffs and openings', () => {
    const board = new TeamBoard();
    const a = board.link('A', true);
    const b = board.link('B', false);
    a.claim('barn', 4);
    expect(b.claimedBy('barn', 4)).toBe('A');
    expect(a.claimedBy('barn', 4)).toBeNull();
    a.claim('barn', null);
    expect(b.claimedBy('barn', 4)).toBeNull();

    a.publishStandoff('barn', [1, 2, 3]);
    expect(b.standoffs('barn')).toEqual([{ name: 'A', pos: [1, 2, 3] }]);
    expect(a.standoffs('barn')).toEqual([]);

    a.shareOpening('barn', opening(3));
    b.shareOpening('barn', opening(3));
    expect(b.openings('barn').map((o) => [o.name, o.opening.id])).toEqual([['A', 3]]);
    expect(a.openings('barn')).toEqual([]);
  });

  it('calls the survey done only when every drone at the building has finished its side', () => {
    const board = new TeamBoard();
    const a = board.link('A', true);
    const b = board.link('B', false);
    a.publishStandoff('barn', [0, 0, 0]);
    b.publishStandoff('barn', [10, 0, 0]);
    a.finishSurvey('barn');
    expect(a.surveyDone('barn')).toBe(false);
    b.finishSurvey('barn');
    expect(a.surveyDone('barn')).toBe(true);
    expect(b.surveyDone('barn')).toBe(true);
  });

  it('tells a drone who else is re-viewing a contact', () => {
    const board = new TeamBoard();
    const a = board.link('A', true);
    const b = board.link('B', false);
    a.setConfirming([5, 5, 1]);
    expect(b.confirming()).toEqual([{ name: 'A', estimate: [5, 5, 1] }]);
    expect(a.confirming()).toEqual([]);
    a.setConfirming(null);
    expect(b.confirming()).toEqual([]);
  });
});

describe('two drones, two ways in', () => {
  it(
    'finds the person on the demo seed with each drone through its own opening, and no contacts',
    async () => {
      const result = await runDuo(TWO_WAYS_IN.world, TWO_WAYS_IN.mission, TWO_WAYS_IN.runner);
      expect(result.found).toBe(true);
      expect(result.entries.every((e) => e !== null)).toBe(true);
      expect(result.distinctEntries).toBe(true);
      expect(result.collisions).toBe(0);
      expect(result.landedHome).toBe(true);
      expect(result.minSeparationM).toBeGreaterThan(0.3);
    },
    300_000,
  );
});

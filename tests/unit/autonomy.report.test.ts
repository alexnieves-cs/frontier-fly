import { describe, expect, it } from 'vitest';
import type {
  ConfirmedContact,
  LiveDetectionSource,
  MissionWorld,
  RunRecord,
  TruthTarget,
  V3,
} from '../../src/mission/autonomy/contracts';
import {
  buildReport,
  cameraFootprintRadius,
  FOOTPRINT_MAX_RADIUS_M,
  operableCells,
  reportMarkdown,
  reportRows,
} from '../../src/mission/autonomy/report';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import type { ClearReport } from '../../src/perception/seenEmpty';
import { bounds } from '../../src/mission/polygon';
import type { Mission } from '../../src/mission/types';
import { sampleHeight } from '../../src/world/outdoor/terrain';

const world = buildMissionWorld({ seed: 7, buildings: 2 });
const b0 = world.buildings[0]!.id;
const b1 = world.buildings[1]!.id;

const PERSON: TruthTarget = { id: 'person-0', classId: 'person', category: 'person', buildingId: b0, world: [55, 40, 6] };
const DECOY: TruthTarget = { id: 'decoy-0', classId: 'mannequin', category: 'decoy', buildingId: b0, world: [57, 41, 6] };
const PACK: TruthTarget = { id: 'pack-0', classId: 'backpack', category: 'object', buildingId: null, world: [30, 30, 5] };

function fakeSource(truth: TruthTarget[], status: Record<string, ClearReport> = {}): LiveDetectionSource {
  return {
    id: 'fake-source',
    observe: (_pose, tSec) => ({ tSec, contacts: [] }),
    clearStatus: (scope) => status[scope] ?? { status: 'incomplete', seenEmpty: 0, occupied: 0, unseen: 10, total: 10 },
    truthTargets: () => truth,
  };
}

function contact(over: Partial<ConfirmedContact> & { truth: ConfirmedContact['truth'] }): ConfirmedContact {
  return { contactId: 'c', classId: 'person', confidence: 0.9, views: 3, estimate: [0, 0, 0], tSec: 0, buildingId: b0, ...over };
}

function run(mission: Mission, over: Partial<RunRecord> = {}): RunRecord {
  return {
    mission,
    seed: world.seed,
    outcome: 'found',
    tSec: 300,
    track: [{ t: 0, pos: world.home, phase: 'preflight' }],
    phases: [],
    confirmed: [],
    declaredClear: [],
    collisions: [],
    entries: [],
    safetyEvents: [],
    batteryStartPct: 100,
    batteryEndPct: 62.5,
    landedHome: true,
    indoorExplorer: null,
    log: [],
    ...over,
  };
}

function lawnmower(w: MissionWorld, mission: Mission, agl: number, spacing: number): RunRecord['track'] {
  const b = bounds(mission.geofence.points);
  const track: RunRecord['track'] = [];
  let t = 0;
  let dir = 1;
  for (let y = b.minY + spacing / 2; y < b.maxY; y += spacing) {
    for (let i = 0; i <= 40; i++) {
      const x = dir > 0 ? b.minX + ((b.maxX - b.minX) * i) / 40 : b.maxX - ((b.maxX - b.minX) * i) / 40;
      const pos: V3 = [x, y, sampleHeight(w.outdoor.terrain, x, y) + agl];
      track.push({ t: t++, pos, phase: 'coverage' });
    }
    dir = -dir;
  }
  return track;
}

describe('report metrics', () => {
  const find = missionForWorld(world, { missionType: 'find_person' });

  it('computes time to find from the first confirmed true positive', () => {
    const r = buildReport(
      run(find, {
        confirmed: [
          contact({ contactId: 'late', tSec: 140, truth: { targetId: PERSON.id, trueClass: 'person', kind: 'tp' } }),
          contact({ contactId: 'decoy', tSec: 60, truth: { targetId: DECOY.id, trueClass: 'mannequin', kind: 'fp' } }),
          contact({ contactId: 'first', tSec: 95.5, truth: { targetId: PERSON.id, trueClass: 'person', kind: 'tp' } }),
        ],
      }),
      fakeSource([PERSON, DECOY]),
      world,
    );
    expect(r.timeToFindSec).toBe(95.5);
    expect(r.falseAlarms).toBe(1);
    expect(r.missed).toBe(0);
  });

  it('counts a missed person and no find when only a decoy was confirmed', () => {
    const r = buildReport(
      run(find, {
        outcome: 'not_found',
        confirmed: [contact({ tSec: 60, truth: { targetId: DECOY.id, trueClass: 'mannequin', kind: 'fp' } })],
      }),
      fakeSource([PERSON, DECOY]),
      world,
    );
    expect(r.timeToFindSec).toBeNull();
    expect(r.falseAlarms).toBe(1);
    expect(r.missed).toBe(1);
  });

  it('counts a confused class as a false alarm', () => {
    const objects = missionForWorld(world, { missionType: 'find_objects' });
    const r = buildReport(
      run(objects, {
        confirmed: [
          contact({ classId: 'medkit', buildingId: null, tSec: 20, truth: { targetId: PACK.id, trueClass: 'backpack', kind: 'confused' } }),
          contact({ classId: 'backpack', buildingId: null, tSec: 30, truth: { targetId: PACK.id, trueClass: 'backpack', kind: 'tp' } }),
        ],
      }),
      fakeSource([PACK, PERSON]),
      world,
    );
    expect(r.falseAlarms).toBe(1);
    expect(r.timeToFindSec).toBe(30);
    expect(r.missed).toBe(0);
  });

  it('flags a clear declared over a building that held a person', () => {
    const sweep = missionForWorld(world, { missionType: 'safety_sweep' });
    const r = buildReport(
      run(sweep, {
        outcome: 'clear',
        declaredClear: [
          { scope: b0, tSec: 200 },
          { scope: b1, tSec: 250 },
          { scope: 'outdoor', tSec: 280 },
        ],
      }),
      fakeSource([PERSON], {
        [b0]: { status: 'clear', seenEmpty: 40, occupied: 0, unseen: 0, total: 40 },
        [b1]: { status: 'incomplete', seenEmpty: 15, occupied: 5, unseen: 20, total: 40 },
      }),
      world,
    );
    expect(r.falseClears).toBe(1);
    expect(r.missed).toBe(0);
    // Safety sweeps cover every building.
    expect(r.perBuildingCoverage[b0]).toBe(1);
    expect(r.perBuildingCoverage[b1]).toBe(0.5);
    expect(r.indoorCoverage).toBe(0.75);
    expect(r.notes.join(' ')).toContain(b1);
  });

  it('only covers entered buildings on a find mission', () => {
    const r = buildReport(
      run(find, { entries: [{ tSec: 50, buildingId: b1, openingId: 0, safety: 0.8, decision: 'auto' }] }),
      fakeSource([], { [b1]: { status: 'incomplete', seenEmpty: 3, occupied: 1, unseen: 4, total: 8 } }),
      world,
    );
    expect(Object.keys(r.perBuildingCoverage)).toEqual([b1]);
    expect(r.indoorCoverage).toBe(0.5);
  });

  it('gives zero outdoor coverage at home and more for a lawnmower sweep', () => {
    expect(cameraFootprintRadius(0)).toBe(0);
    expect(cameraFootprintRadius(10)).toBeCloseTo(10, 6);
    expect(cameraFootprintRadius(100)).toBe(FOOTPRINT_MAX_RADIUS_M);
    const ground = sampleHeight(world.outdoor.terrain, world.home[0], world.home[1]);
    const parked: V3 = [world.home[0], world.home[1], ground];
    const atHome = buildReport(run(find, { track: [{ t: 0, pos: parked, phase: 'preflight' }] }), fakeSource([]), world);
    expect(atHome.outdoorCoverage).toBe(0);
    const sparse = buildReport(run(find, { track: lawnmower(world, find, 6, 30) }), fakeSource([]), world);
    const dense = buildReport(run(find, { track: lawnmower(world, find, 12, 16) }), fakeSource([]), world);
    expect(sparse.outdoorCoverage).toBeGreaterThan(0.2);
    expect(dense.outdoorCoverage).toBeGreaterThan(sparse.outdoorCoverage);
    expect(dense.outdoorCoverage).toBeGreaterThan(0.9);
    expect(dense.outdoorCoverage).toBeLessThanOrEqual(1);
    // Building footprints and blocking no-fly zones are not operable.
    const cells = operableCells(find, world).cells;
    const fp = world.buildings[0]!.footprint;
    const mid = { x: (fp[0]!.x + fp[2]!.x) / 2, y: (fp[0]!.y + fp[2]!.y) / 2 };
    expect(cells.some((c) => Math.hypot(c.x - mid.x, c.y - mid.y) < 1.5)).toBe(false);
  });

  it('reports collisions, battery, and renders rows and markdown', () => {
    const r = buildReport(
      run(find, {
        collisions: [
          { tSec: 10, pos: [0, 0, 0], inside: b0 },
          { tSec: 20, pos: [0, 0, 0], inside: null },
        ],
        safetyEvents: [{ tSec: 10, kind: 'collision', message: 'Collision 1', action: 'hold' }],
        landedHome: false,
      }),
      fakeSource([PERSON]),
      world,
    );
    expect(r.collisions).toBe(2);
    expect(r.batteryUsedPct).toBe(37.5);
    expect(r.safetyEvents).toBe(1);
    expect(r.notes.some((n) => n.includes('momentum-theory'))).toBe(true);
    expect(r.notes.some((n) => n.includes('simulated'))).toBe(true);
    const rows = reportRows(r);
    expect(rows.find((row) => row.label === 'Collisions')!.value).toBe('2');
    expect(rows.find((row) => row.label === 'Landed home')!.value).toBe('no');
    const md = reportMarkdown(r);
    expect(md).toContain('| Battery used | 37.5% |');
    expect(md).toContain('## Notes');
  });
});

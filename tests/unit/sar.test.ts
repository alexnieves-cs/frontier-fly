import { describe, expect, it } from 'vitest';
import { airspaceFor } from '../../src/mission/autonomy/planner/geometry';
import { ConfirmTracker } from '../../src/mission/autonomy/planner/confirm';
import { runMission } from '../../src/mission/autonomy/runner';
import { createPerceptionSource } from '../../src/mission/autonomy/perception';
import { SearchBelief } from '../../src/mission/autonomy/sar/belief';
import { buildSearchGrid, cellCenter, cellIndex, sectorOf, SKY_SECTORS, skyVisible } from '../../src/mission/autonomy/sar/grid';
import { lostPersonPrior, simulateLostPerson, STRATEGIES, STRATEGY_WEIGHT, plantPersonAt } from '../../src/mission/autonomy/sar/lostPerson';
import { ProbabilityPlanner } from '../../src/mission/autonomy/sar/planner';
import { framePod, sweepWidth } from '../../src/mission/autonomy/sar/pod';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { containsPoint } from '../../src/world/outdoor/geofence';
import { findCurve, pairedBootstrap, restrictedMean, sarScene, type SarFlight } from '../../src/eval/sar';

const DAY = { timeOfDay: 13, sensorMode: 'rgb' as const, flashlightOn: false };
const world = buildMissionWorld({ seed: 7101, timeOfDay: 13, targets: [] });
const grid = buildSearchGrid(world);
const ipp: [number, number] = [world.home[0] + 10, world.home[1] - 6];

function total(prior: ReturnType<typeof lostPersonPrior>): number {
  let s = prior.rest;
  for (const k of grid.cells) s += prior.cell[k]!;
  for (const m of prior.structure.values()) s += m;
  return s;
}

describe('search grid', () => {
  it('masks cells a person could stand on, inside the fence', () => {
    expect(grid.cells.length).toBeGreaterThan(200);
    for (const k of grid.cells) {
      expect(grid.free[k]).toBeGreaterThan(0);
      expect(grid.free[k]).toBeLessThanOrEqual(1);
    }
    const [x, y] = cellCenter(grid, grid.cells[0]!);
    expect(cellIndex(grid, x, y)).toBe(grid.cells[0]);
  });

  it('hides ground straight under a building roof from a camera above it', () => {
    const b = world.buildings[0]!;
    const xs = b.footprint.map((p) => p.x);
    const ys = b.footprint.map((p) => p.y);
    // A cell just outside the wall sees the sky straight up but not a low camera across the roof.
    const x = Math.max(...xs) + 2.5;
    const y = (Math.min(...ys) + Math.max(...ys)) / 2;
    const k = cellIndex(grid, x, y);
    expect(skyVisible(grid, k, sectorOf(0), 1e5)).toBe(true);
    expect(skyVisible(grid, k, sectorOf(Math.PI), 0.3)).toBe(false);
    expect(sectorOf(Math.PI * 2)).toBe(0);
    expect(sectorOf(-0.001)).toBe(0);
    expect(sectorOf(Math.PI)).toBe(SKY_SECTORS / 2);
  });
});

describe('lost-person prior', () => {
  for (const shape of ['behavior', 'ring', 'uniform'] as const) {
    it(`${shape}: cells, buildings and rest of world sum to one`, () => {
      const prior = lostPersonPrior(world, grid, ipp, { shape });
      expect(total(prior)).toBeCloseTo(1, 6);
      expect(prior.rest).toBeGreaterThanOrEqual(0);
      for (const k of grid.cells) expect(prior.cell[k]).toBeGreaterThanOrEqual(0);
    });
  }

  it('puts more mass near the last known point than far from it', () => {
    const prior = lostPersonPrior(world, grid, ipp);
    let near = 0;
    let far = 0;
    for (const k of grid.cells) {
      const [x, y] = cellCenter(grid, k);
      const d = Math.hypot(x - ipp[0], y - ipp[1]);
      if (d < 12) near += prior.cell[k]!;
      else if (d > 24) far += prior.cell[k]!;
    }
    expect(near).toBeGreaterThan(2 * far);
  });

  it('gives buildings their floor area under every prior, and more under the behavior prior', () => {
    const sum = (m: Map<string, number>) => [...m.values()].reduce((a, b) => a + b, 0);
    const behavior = sum(lostPersonPrior(world, grid, ipp).structure);
    const ring = sum(lostPersonPrior(world, grid, ipp, { shape: 'ring' }).structure);
    const uniform = lostPersonPrior(world, grid, ipp, { shape: 'uniform' });
    expect(uniform.structure.size).toBe(world.buildings.length);
    expect(sum(uniform.structure)).toBeGreaterThan(0);
    expect(ring).toBeGreaterThan(0);
    expect(behavior).toBeGreaterThan(ring);
    expect(behavior).toBeLessThan(STRATEGY_WEIGHT.shelter + ring);
  });
});

describe('lost-person simulator', () => {
  it('is deterministic and stays inside the geofence', () => {
    for (let s = 0; s < 40; s++) {
      const a = simulateLostPerson(world, ipp, 500 + s);
      expect(simulateLostPerson(world, ipp, 500 + s)).toEqual(a);
      expect(containsPoint(world.outdoor.geofence, a.xy[0], a.xy[1])).toBe(true);
      expect(STRATEGIES).toContain(a.strategy);
    }
  });

  it('draws strategies near their planning weights', () => {
    const counts = Object.fromEntries(STRATEGIES.map((s) => [s, 0])) as Record<string, number>;
    const n = 400;
    for (let s = 0; s < n; s++) counts[simulateLostPerson(world, ipp, 9000 + s).strategy]! += 1;
    for (const s of STRATEGIES) expect(Math.abs(counts[s]! / n - STRATEGY_WEIGHT[s])).toBeLessThan(0.07);
  });

  it('a stay-put person stays near the last known point; a shelter seeker reaches a building', () => {
    const still = simulateLostPerson(world, ipp, 1, 'stay_put');
    expect(Math.hypot(still.xy[0] - ipp[0], still.xy[1] - ipp[1])).toBeLessThan(7);
    const shelter = simulateLostPerson(world, ipp, 2, 'shelter');
    const b = world.buildings.map((x) => Math.min(...x.footprint.map((p) => Math.hypot(p.x - shelter.xy[0], p.y - shelter.xy[1]))));
    expect(Math.min(...b)).toBeLessThan(12);
  });

  it('plants a person clear of trees and buildings', () => {
    const p = plantPersonAt(world, ipp, 'lying', 3);
    expect(p).not.toBeNull();
    expect(p!.classId).toBe('person');
    expect(Math.hypot(p!.feet[0] - ipp[0], p!.feet[1] - ipp[1])).toBeLessThanOrEqual(5);
  });
});

describe('probability of detection', () => {
  it('is zero outside the frame and highest near nadir', () => {
    expect(framePod(12, 20, 0, 0, 0, DAY)).toBe(0);
    expect(framePod(12, 0, 0, 0, 0, DAY)).toBeGreaterThan(framePod(12, 7, 9, 0, 0, DAY));
    expect(framePod(12, 0, 0, 0, 0, DAY)).toBeGreaterThan(framePod(20, 0, 0, 0, 0, DAY));
  });

  it('drops with occlusion, speed and darkness, and thermal recovers the night', () => {
    const base = framePod(12, 0, 0, 0, 0, DAY);
    expect(framePod(12, 0, 0, 0, 1, DAY)).toBe(0);
    expect(framePod(12, 0, 0, 5, 0, DAY)).toBeLessThan(base);
    const night = { timeOfDay: 23, sensorMode: 'rgb' as const, flashlightOn: false };
    expect(framePod(12, 0, 0, 0, 0, night)).toBeLessThan(base * 0.2);
    const thermal = { timeOfDay: 23, sensorMode: 'ir' as const, flashlightOn: false };
    expect(framePod(12, 0, 0, 0, 0, thermal)).toBeGreaterThan(base * 0.7);
  });

  it('gives a sweep width no wider than the image, and narrower at night on RGB', () => {
    const w = sweepWidth(12.5, 5.5, 0.2, DAY);
    expect(w).toBeGreaterThan(8);
    expect(w).toBeLessThanOrEqual(2 * 12.5 * Math.tan((35 * Math.PI) / 180) * (4 / 3) + 1e-9);
    expect(sweepWidth(12.5, 5.5, 0.2, { timeOfDay: 23, sensorMode: 'rgb', flashlightOn: true })).toBeLessThan(w * 0.75);
  });
});

describe('search belief', () => {
  it('removes mass only under the camera and keeps the posterior normalized', () => {
    const prior = lostPersonPrior(world, grid, ipp);
    const belief = new SearchBelief(grid, prior, DAY);
    const before = Float64Array.from(belief.mass);
    const z = grid.ground[cellIndex(grid, ipp[0], ipp[1])]! + 13;
    const removed = belief.observeFrame(ipp[0], ipp[1], z, 0, 0);
    expect(removed).toBeGreaterThan(0);
    expect(belief.pos()).toBeCloseTo(removed, 9);
    let changedFar = 0;
    let post = 0;
    for (const k of grid.cells) {
      const [x, y] = cellCenter(grid, k);
      if (Math.hypot(x - ipp[0], y - ipp[1]) > 20 && belief.mass[k] !== before[k]) changedFar += 1;
      expect(belief.mass[k]).toBeLessThanOrEqual(before[k]!);
      post += belief.poa(k);
    }
    for (const m of belief.structure.values()) post += m / belief.remaining();
    post += belief.rest / belief.remaining();
    expect(changedFar).toBe(0);
    expect(post).toBeCloseTo(1, 9);
  });

  it('leaves cells near a contact alone', () => {
    const belief = new SearchBelief(grid, lostPersonPrior(world, grid, ipp), DAY);
    const k = cellIndex(grid, ipp[0], ipp[1]);
    const [x, y] = cellCenter(grid, k);
    const m = belief.mass[k]!;
    belief.observeFrame(x, y, grid.ground[k]! + 13, 0, 0, [[x, y]]);
    expect(belief.mass[k]).toBe(m);
  });

  it('searches a building by its indoor POD', () => {
    const belief = new SearchBelief(grid, lostPersonPrior(world, grid, ipp), DAY);
    const id = world.buildings[0]!.id;
    const m = belief.structure.get(id)!;
    belief.observeStructure(id, 0.85);
    expect(belief.structure.get(id)).toBeCloseTo(m * 0.15, 12);
  });
});

describe('probability planner', () => {
  const mission = missionForWorld(world, { missionType: 'find_person' });
  const air = airspaceFor(mission, world);

  it('plans legs inside the airspace toward unsearched mass, deterministically', () => {
    const make = () => new ProbabilityPlanner(new SearchBelief(grid, lostPersonPrior(world, grid, ipp), DAY), air, DAY);
    const a = make().plan([world.home[0], world.home[1]], 0);
    const b = make().plan([world.home[0], world.home[1]], 0);
    expect(a).toEqual(b);
    expect(a.kind).toBe('fly');
    if (a.kind !== 'fly') return;
    expect(a.path.length).toBeGreaterThan(0);
    expect(a.gain).toBeGreaterThan(0);
  });

  it('goes to a building once the outdoor mass is spent, and stops when nothing is left', () => {
    const belief = new SearchBelief(grid, lostPersonPrior(world, grid, ipp), DAY);
    const planner = new ProbabilityPlanner(belief, air, DAY);
    for (const k of grid.cells) belief.mass[k] = 0;
    const segments = world.buildings.map((b) => ({ id: b.id, center: [b.footprint[0]!.x, b.footprint[0]!.y] as [number, number] }));
    const go = planner.plan([world.home[0], world.home[1]], 0, segments);
    expect(go.kind).toBe('building');
    const done = planner.plan([world.home[0], world.home[1]], 0, segments, new Set(world.buildings.map((b) => b.id)));
    expect(done.kind).toBe('done');
  });
});

describe('outdoor confirmation range', () => {
  it('counts gimbal views of outdoor tracks out to the scaled range only', () => {
    const mission = missionForWorld(world, { missionType: 'find_person' });
    const tracker = new ConfirmTracker(mission, { outdoorRangeScale: 2 });
    const contact = (range: number, buildingId: string | null) => ({
      id: `c${range}${buildingId}`,
      tSec: 0,
      classId: 'person' as const,
      confidence: 0.9,
      estimate: [10, 10, 0] as [number, number, number],
      range,
      bearing: [1, 0, -1] as [number, number, number],
      buildingId,
      truth: { targetId: 't', trueClass: 'person' as const, kind: 'tp' as const },
    });
    expect(tracker.ingest(contact(14, null))!.views.length).toBe(1);
    const indoor = new ConfirmTracker(mission, { outdoorRangeScale: 2 });
    expect(indoor.ingest(contact(14, 'bldg-0'))!.views.length).toBe(0);
  });
});

describe('benchmark statistics', () => {
  const flight = (seed: number, tFind: number | null): SarFlight => ({
    seed,
    condition: 'day',
    method: 'lanes',
    strategy: 'wander',
    where: 'outdoor',
    dIpp: 0,
    found: tFind !== null,
    tFind,
    outcome: tFind !== null ? 'found' : 'not_found',
    durationSec: 0,
    batteryUsedPct: 0,
    pathM: 0,
    buildingsEntered: 0,
    falseAlarms: 0,
    collisions: 0,
    landedHome: true,
    pos: null,
    reachable: null,
  });

  it('computes find curves and censored means', () => {
    const fs = [flight(1, 10), flight(2, 100), flight(3, null)];
    expect(findCurve(fs, [0, 50, 200])).toEqual([0, 1 / 3, 2 / 3]);
    expect(restrictedMean(fs, 900)).toBeCloseTo((10 + 100 + 900) / 3, 9);
  });

  it('pairs flights by seed in the bootstrap', () => {
    const a = [flight(1, 100), flight(2, 200), flight(3, null)];
    const b = [flight(1, 50), flight(2, 150), flight(3, 300)];
    const r = pairedBootstrap(a, b, 500);
    expect(r.dMean[0]).toBeCloseTo((-50 - 50 - 600) / 3, 9);
    expect(r.dFound[0]).toBeCloseTo(1 / 3, 9);
    expect(r.dMean[1]).toBeLessThanOrEqual(r.dMean[0]);
    expect(r.dMean[2]).toBeGreaterThanOrEqual(r.dMean[0]);
  });
});

describe('probability search end to end', () => {
  it('finds a lost hiker on a development seed and lands home, deterministically', async () => {
    const scene = sarScene(7105, 'day');
    const fly = async () => {
      const source = createPerceptionSource(scene.world);
      return runMission(scene.mission, scene.world, source, { dt: 0.05, indoor: 'frontier', operator: () => 'approve', search: { mode: 'probability', lastKnown: scene.ipp } });
    };
    const run = await fly();
    expect(run.outcome).toBe('found');
    expect(run.landedHome).toBe(true);
    expect(run.collisions).toEqual([]);
    expect(run.search!.pos.length).toBeGreaterThan(0);
    const again = await fly();
    expect(again.tSec).toBe(run.tSec);
    expect(again.confirmed).toEqual(run.confirmed);
  }, 60_000);
});

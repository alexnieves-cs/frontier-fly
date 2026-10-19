import { beforeAll, describe, expect, it } from 'vitest';
import { createBattery } from '../../src/mission/autonomy/battery';
import type { Contact, MissionWorld, V3 } from '../../src/mission/autonomy/contracts';
import { planApproach } from '../../src/mission/autonomy/planner/approach';
import { ConfirmTracker, confirmViewpoints } from '../../src/mission/autonomy/planner/confirm';
import { planCoverage } from '../../src/mission/autonomy/planner/coverage';
import { entryDecision, passageAxis, rankCandidates, SAFETY_ASK, SAFETY_AUTO, scoreOpening } from '../../src/mission/autonomy/planner/entry';
import { FlyThrough } from '../../src/mission/autonomy/planner/flythrough';
import { airspaceFor, buildingBox, inRing, insideBox, operable, segmentClear } from '../../src/mission/autonomy/planner/geometry';
import { landedAtHome, landingCommand, planReturn } from '../../src/mission/autonomy/planner/home';
import {
  cellAt,
  FrontierExplorer,
  INDOOR_FLY_Z,
  IndoorVisibility,
  indoorGrid,
  nearestNav,
  toLocal,
  wallClearance,
} from '../../src/mission/autonomy/planner/indoor';
import { observeOpenings, planInspection } from '../../src/mission/autonomy/planner/inspect';
import { INDOOR_LIMITS, OUTDOOR_LIMITS, Vehicle, type VehicleLimits } from '../../src/mission/autonomy/vehicle';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import type { Mission } from '../../src/mission/types';
import { DRONE_RADIUS } from '../../src/shared/constants';
import { initOutdoorPhysics, OutdoorPhysics } from '../../src/world/outdoor/physics';
import { sampleHeight } from '../../src/world/outdoor/terrain';

/*
 * One test or more per autonomy stage. Worlds: seed 20 with one building
 * (garage gap entry, person inside) and seed 37 with two buildings.
 */

interface Site {
  world: MissionWorld;
  mission: Mission;
  physics: OutdoorPhysics;
}

function site(seed: number, buildings: number): Site {
  const world = buildMissionWorld({
    seed,
    buildings,
    timeOfDay: 13,
    interiorLights: 'on',
    targets: [{ classId: 'person', state: 'lying', where: 'indoor', building: 0 }],
  });
  const mission = missionForWorld(world, {
    missionType: 'find_person',
    targets: [{ id: 'tgt-person', kind: 'person', label: 'Person', minCount: 1, minConfidence: null }],
    requiredConfidence: 0.8,
    entryPolicy: 'auto',
  });
  return { world, mission, physics: new OutdoorPhysics(world.outdoor) };
}

/** Flies a polyline with a simple speed-limited pursuit. Returns contact events. */
function fly(vehicle: Vehicle, route: readonly V3[], limits: VehicleLimits = OUTDOOR_LIMITS, tol = 0.4): number {
  let contacts = 0;
  for (const wp of route) {
    for (let i = 0; i < 4000; i++) {
      const d: V3 = [wp[0] - vehicle.pos[0], wp[1] - vehicle.pos[1], wp[2] - vehicle.pos[2]];
      const l = Math.hypot(d[0], d[1], d[2]);
      if (l < tol) break;
      const speed = Math.min(limits.speedH, Math.sqrt(2 * limits.accel * 0.5 * l));
      const step = vehicle.step({ vel: [(d[0] / l) * speed, (d[1] / l) * speed, (d[2] / l) * speed], yaw: null }, 0.05, limits);
      if (step.collided) contacts += 1;
    }
  }
  return contacts;
}

let a: Site;
let b: Site;

beforeAll(async () => {
  await initOutdoorPhysics();
  a = site(20, 1);
  b = site(37, 2);
});

describe('coverage', () => {
  it('stays in the fence, out of no-fly, in the altitude band, and covers the area', () => {
    for (const s of [a, b]) {
      const air = airspaceFor(s.mission, s.world);
      const plan = planCoverage(s.mission, s.world);
      expect(plan.waypoints.length).toBeGreaterThan(4);
      expect(plan.laneSpacing).toBeLessThan(plan.footprintM * 2);
      for (const p of plan.waypoints) {
        expect(inRing(p[0], p[1], air.fence)).toBe(true);
        expect(operable(air, p[0], p[1], 0)).toBe(true);
        const agl = p[2] - sampleHeight(air.terrain, p[0], p[1]);
        expect(agl).toBeGreaterThanOrEqual(air.minAgl - 0.5);
        expect(agl).toBeLessThanOrEqual(air.maxAgl + 0.5);
      }
      // Operable ground within the margin is inside some lane's footprint.
      let total = 0;
      let covered = 0;
      const pts = plan.waypoints;
      const xs = air.fence.map((q) => q.x);
      const ys = air.fence.map((q) => q.y);
      for (let x = Math.min(...xs); x <= Math.max(...xs); x += 4) {
        for (let y = Math.min(...ys); y <= Math.max(...ys); y += 4) {
          if (!operable(air, x, y, 4)) continue;
          total += 1;
          let best = Infinity;
          for (let i = 1; i < pts.length; i++) {
            const p = pts[i - 1]!;
            const q = pts[i]!;
            const dx = q[0] - p[0];
            const dy = q[1] - p[1];
            const t = Math.max(0, Math.min(1, ((x - p[0]) * dx + (y - p[1]) * dy) / (dx * dx + dy * dy || 1)));
            best = Math.min(best, Math.hypot(p[0] + dx * t - x, p[1] + dy * t - y));
          }
          if (best <= plan.footprintM) covered += 1;
        }
      }
      expect(covered / total).toBeGreaterThan(0.95);
    }
  });
});

describe('approach and inspection', () => {
  it('reaches a standoff without touching canopy and sees the passable opening', () => {
    for (const s of [a, b]) {
      const air = airspaceFor(s.mission, s.world);
      for (const building of s.world.buildings) {
        const home = s.world.home;
        const from: V3 = [home[0], home[1], sampleHeight(air.terrain, home[0], home[1]) + 13];
        const plan = planApproach(building, from, s.physics, air);
        expect(plan.ok, plan.reason).toBe(true);
        const route = [from, ...plan.cruise, ...plan.descent];
        for (let i = 1; i < route.length; i++) expect(segmentClear(s.physics, route[i - 1]!, route[i]!), `${s.world.seed} ${building.id} leg ${i}`).toBe(true);
        const vehicle = new Vehicle(s.physics, createBattery(), from, 0, s.world.outdoor);
        expect(fly(vehicle, route.slice(1))).toBe(0);

        const steps = planInspection(building, vehicle.pos, s.physics, air);
        expect(steps.length).toBeGreaterThan(2);
        const seen = new Set<number>();
        let contacts = 0;
        for (const step of steps) {
          contacts += fly(vehicle, [step.pos], OUTDOOR_LIMITS, 0.25);
          const yaw = step.yaw ?? vehicle.yaw;
          for (const o of observeOpenings(step.pos, yaw, building, s.physics)) seen.add(o.id);
        }
        expect(contacts).toBe(0);
        const passable = building.shell.openings.filter((o) => o.passable && !o.decoy);
        expect(passable.length).toBeGreaterThan(0);
        expect(passable.some((o) => seen.has(o.id)), `${s.world.seed} ${building.id}`).toBe(true);
      }
    }
  });
});

describe('entry selection', () => {
  it('maps safety to decisions and caps by policy', () => {
    expect(entryDecision(SAFETY_AUTO + 0.01, 'auto')).toBe('auto');
    expect(entryDecision(SAFETY_ASK + 0.01, 'auto')).toBe('ask');
    expect(entryDecision(SAFETY_ASK - 0.01, 'auto')).toBe('never');
    expect(entryDecision(0.95, 'ask')).toBe('ask');
    expect(entryDecision(0.95, 'never')).toBe('never');
  });

  it('scores the real entrance as enterable and a too-small vent as never', () => {
    const building = a.world.buildings[0]!;
    const opening = building.shell.openings.find((o) => o.passable && !o.decoy)!;
    const good = scoreOpening(building, opening, a.physics, 'auto');
    expect(good.clearanceM).toBeGreaterThan(0);
    expect(good.decision).toBe('auto');
    expect(scoreOpening(building, opening, a.physics, 'ask').decision).toBe('ask');
    expect(scoreOpening(building, opening, a.physics, 'never').decision).toBe('never');
    const vent = scoreOpening(building, { ...opening, id: 99, kind: 'vent', width: 0.3, height: 0.3, angle: 0 }, a.physics, 'auto');
    expect(vent.safety).toBe(0);
    expect(vent.decision).toBe('never');
    const ranked = rankCandidates([vent, good]);
    expect(ranked[0]!.openingId).toBe(opening.id);
    expect(ranked.some((c) => c.openingId === 99)).toBe(false);
  });
});

describe('fly-through', () => {
  it('crosses on the centerline without contact and ends inside', () => {
    for (const s of [a, b]) {
      for (const building of s.world.buildings) {
        const opening = building.shell.openings.find((o) => o.passable && !o.decoy)!;
        const axis = passageAxis(opening);
        const ft = new FlyThrough(axis, 'in');
        const vehicle = new Vehicle(s.physics, createBattery(), ft.start, 0, s.world.outdoor);
        let collided = false;
        let contacts = 0;
        for (let i = 0; i < 1200 && ft.state !== 'done' && ft.state !== 'aborted'; i++) {
          const out = ft.step(vehicle.pos, vehicle.vel, collided);
          const step = vehicle.step(out.cmd, 0.05, INDOOR_LIMITS);
          collided = step.collided;
          if (collided) contacts += 1;
        }
        expect(ft.state, ft.abortReason).toBe('done');
        expect(contacts).toBe(0);
        expect(ft.maxErrU).toBeLessThan(Math.max(0.05, axis.marginU));
        expect(ft.maxErrV).toBeLessThan(Math.max(0.05, axis.marginV));
        expect(insideBox(buildingBox(building), vehicle.pos, -0.3)).toBe(true);
      }
    }
  });
});

describe('indoor frontier exploration', () => {
  it('sees most cells on paths that keep wall clearance', () => {
    for (const s of [a, b]) {
      for (const building of s.world.buildings) {
        const grid = indoorGrid(building);
        const vis = new IndoorVisibility(grid);
        const explorer = new FrontierExplorer(grid, vis);
        const opening = building.shell.openings.find((o) => o.passable && !o.decoy)!;
        const entry = toLocal(grid, passageAxis(opening).center);
        const start = nearestNav(grid, [entry[0], entry[1], INDOOR_FLY_Z])!;
        let cur: V3 = [(start.x + 0.5) * grid.cs, (start.z + 0.5) * grid.cs, start.f * grid.H + INDOOR_FLY_Z];
        vis.observe(cur, 0);
        let minClear = Infinity;
        for (let k = 0; k < 200; k++) {
          const goal = explorer.next(cur);
          if (!goal) break;
          let prev = cur;
          for (const p of goal.path) {
            const n = Math.max(1, Math.ceil(Math.hypot(p[0] - prev[0], p[1] - prev[1]) / 0.1));
            for (let i = 1; i <= n; i++) {
              const x = prev[0] + ((p[0] - prev[0]) * i) / n;
              const y = prev[1] + ((p[1] - prev[1]) * i) / n;
              minClear = Math.min(minClear, wallClearance(grid, cellAt(grid, p).f, x, y));
            }
            vis.observe(p, Math.atan2(p[1] - prev[1], p[0] - prev[0]));
            prev = p;
          }
          if (goal.spin) for (let q = 0; q < 8; q++) vis.observe(prev, (q / 8) * Math.PI * 2);
          cur = prev;
        }
        expect(vis.coverage()).toBeGreaterThan(0.9);
        expect(minClear).toBeGreaterThan(DRONE_RADIUS);
      }
    }
  });
});

function contact(estimate: V3, angleDeg: number, confidence: number, t: number): Contact {
  const a = (angleDeg * Math.PI) / 180;
  return {
    id: `c-${t}`,
    tSec: t,
    classId: 'person',
    confidence,
    estimate,
    range: 3,
    bearing: [Math.cos(a), Math.sin(a), 0],
    buildingId: 'bldg-0',
    truth: { targetId: 'x', trueClass: 'person', kind: 'tp' },
  };
}

describe('confirmation', () => {
  const at: V3 = [10, 10, 1];

  it('never confirms from one high-confidence frame or one bearing', () => {
    const tracker = new ConfirmTracker(a.mission);
    const track = tracker.ingest(contact(at, 0, 0.99, 0))!;
    expect(tracker.evaluate(track)).toBe('open');
    for (let i = 1; i < 20; i++) tracker.ingest(contact(at, 5, 0.99, i * 0.2));
    expect(track.views.length).toBe(1);
    expect(tracker.evaluate(track)).toBe('open');
    tracker.reject(track);
    expect(track.status).toBe('rejected');
  });

  it('confirms from three separated views at the threshold, and not below it', () => {
    const tracker = new ConfirmTracker(a.mission);
    let track = tracker.ingest(contact(at, 0, 0.84, 0))!;
    tracker.ingest(contact(at, 60, 0.82, 1));
    tracker.ingest(contact(at, 130, 0.86, 2));
    expect(tracker.evaluate(track)).toBe('confirmed');

    const low = new ConfirmTracker(a.mission);
    track = low.ingest(contact(at, 0, 0.62, 0))!;
    low.ingest(contact(at, 60, 0.6, 1));
    low.ingest(contact(at, 130, 0.64, 2));
    expect(low.evaluate(track)).toBe('open');
  });

  it('plans re-view poses at fresh bearings first', () => {
    const tracker = new ConfirmTracker(a.mission);
    const track = tracker.ingest(contact(at, 0, 0.7, 0))!;
    const views = confirmViewpoints(track, at, [7, 10, 1], 2.5, 1.2, 2, () => true);
    expect(views.length).toBe(2);
    for (const v of views) {
      expect(Math.hypot(v[0] - at[0], v[1] - at[1])).toBeCloseTo(2.5, 5);
      const bearing = Math.atan2(at[1] - v[1], at[0] - v[0]);
      expect(Math.abs(Math.atan2(Math.sin(bearing), Math.cos(bearing)))).toBeGreaterThan((39 * Math.PI) / 180);
    }
  });
});

describe('return home', () => {
  it('climbs, cruises, and lands on the pad without contact', () => {
    for (const s of [a, b]) {
      const air = airspaceFor(s.mission, s.world);
      const building = s.world.buildings[0]!;
      const plan = planApproach(building, s.world.home, s.physics, air);
      expect(plan.ok).toBe(true);
      const from = plan.standoff!.pos;
      const ret = planReturn(s.physics, air, from, s.world.home, null);
      expect(ret.lengthM).toBeGreaterThan(0);
      const vehicle = new Vehicle(s.physics, createBattery(), from, 0, s.world.outdoor);
      expect(fly(vehicle, ret.waypoints)).toBe(0);
      const ground = sampleHeight(air.terrain, s.world.home[0], s.world.home[1]);
      let touched = false;
      for (let i = 0; i < 2000 && !touched; i++) {
        const cmd = landingCommand(vehicle.pos, s.world.home, ground);
        touched = cmd.touched;
        vehicle.step({ vel: cmd.vel }, 0.05, OUTDOOR_LIMITS);
      }
      expect(touched).toBe(true);
      expect(landedAtHome(vehicle.pos, s.world.home)).toBe(true);
    }
  });
});

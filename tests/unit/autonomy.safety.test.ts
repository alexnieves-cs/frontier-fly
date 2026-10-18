import { describe, expect, it } from 'vitest';
import { createBattery, DEFAULT_BATTERY, hoverEnduranceMin, hoverPowerW } from '../../src/mission/autonomy/battery';
import type { BatteryModel, Phase, SafetyInput, V3 } from '../../src/mission/autonomy/contracts';
import { createSafetyMonitor } from '../../src/mission/autonomy/safety';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { sampleHeight } from '../../src/world/outdoor/terrain';

const world = buildMissionWorld({ seed: 7, buildings: 2 });
const mission = missionForWorld(world, { missionType: 'find_person' });
const terrain = world.outdoor.terrain;

/** A world point `agl` meters above the terrain at x, y. */
function at(x: number, y: number, agl: number): V3 {
  return [x, y, sampleHeight(terrain, x, y) + agl];
}

/** Open ground well away from home and buildings, inside the fence. */
const OPEN: V3 = at(40, 22, 12);

function fakeBattery(remaining: number, costPerM = 0): BatteryModel {
  return {
    capacityWh: 74,
    powerFor: () => 0,
    draw: () => undefined,
    remainingPct: () => remaining,
    usedPct: () => 100 - remaining,
    costPct: (d) => d * costPerM,
  };
}

function input(over: Partial<SafetyInput> & { pos?: V3 }): SafetyInput {
  const pos = over.pos ?? OPEN;
  return {
    tSec: 0,
    pose: { pos, yaw: 0, vel: [0, 0, 0] },
    predicted: pos,
    battery: fakeBattery(90),
    distanceHomeM: 20,
    collided: false,
    inside: null,
    phase: 'coverage' as Phase,
    ...over,
  };
}

describe('battery model', () => {
  it('orders cruise below hover below climb, and fast flight above cruise', () => {
    const b = createBattery();
    const hover = b.powerFor([0, 0, 0], [0, 0, 0]);
    const cruise = b.powerFor([8, 0, 0], [0, 0, 0]);
    const fast = b.powerFor([18, 0, 0], [0, 0, 0]);
    const climb = b.powerFor([0, 0, 2], [0, 0, 0]);
    const accel = b.powerFor([4, 0, 0], [3, 0, 0]);
    expect(cruise).toBeLessThan(hover);
    expect(hover).toBeLessThan(climb);
    expect(fast).toBeGreaterThan(cruise);
    expect(accel).toBeGreaterThan(b.powerFor([4, 0, 0], [0, 0, 0]));
    expect(hover).toBeGreaterThan(100);
    expect(hover).toBeLessThan(250);
  });

  it('gives a plausible hover endurance and drains deterministically', () => {
    const minutes = hoverEnduranceMin();
    expect(minutes).toBeGreaterThan(20);
    expect(minutes).toBeLessThan(35);
    const b = createBattery();
    const p = hoverPowerW(DEFAULT_BATTERY);
    for (let i = 0; i < 600; i++) b.draw(p, 0.1);
    const expected = ((p * 60) / 3600 / (DEFAULT_BATTERY.capacityWh * DEFAULT_BATTERY.usableFraction)) * 100;
    expect(b.usedPct()).toBeCloseTo(expected, 6);
    expect(b.remainingPct()).toBeCloseTo(100 - expected, 6);
  });

  it('costPct grows with distance and climb', () => {
    const b = createBattery();
    expect(b.costPct(0, 0)).toBe(0);
    expect(b.costPct(100, 0)).toBeLessThan(b.costPct(200, 0));
    expect(b.costPct(100, 0)).toBeLessThan(b.costPct(100, 10));
    // 1 km at cruise is a handful of percent, not most of the pack.
    expect(b.costPct(1000, 0)).toBeGreaterThan(2);
    expect(b.costPct(1000, 0)).toBeLessThan(15);
    expect(createBattery(60).capacityWh).toBe(60);
  });
});

describe('safety monitor', () => {
  it('is quiet in nominal flight', () => {
    const s = createSafetyMonitor(mission, world);
    for (let t = 0; t < 5; t += 0.1) expect(s.check(input({ tSec: t })).kind).toBe('none');
    expect(s.events()).toEqual([]);
  });

  it('holds on a predicted geofence breach, then returns home if it persists, with two events', () => {
    const s = createSafetyMonitor(mission, world);
    const predicted: V3 = [world.home[0], 0, OPEN[2]];
    const kinds: string[] = [];
    for (let t = 0; t <= 4; t += 0.05) kinds.push(s.check(input({ tSec: t, predicted })).kind);
    expect(kinds[0]).toBe('hold');
    expect(kinds.at(-1)).toBe('return_home');
    const ev = s.events().filter((e) => e.kind === 'geofence');
    expect(ev.map((e) => e.action)).toEqual(['hold', 'return_home']);
  });

  it('returns home on an actual breach and does not loop while homing', () => {
    const s = createSafetyMonitor(mission, world);
    const out = at(40, 2, 12);
    expect(s.check(input({ tSec: 0, pos: out })).kind).toBe('return_home');
    for (let t = 0.05; t < 3; t += 0.05) s.check(input({ tSec: t, pos: out }));
    expect(s.check(input({ tSec: 3, pos: out, phase: 'return_home' })).kind).toBe('none');
    expect(s.events().filter((e) => e.kind === 'geofence')).toHaveLength(1);
  });

  it('honors site no-fly zones and their altitude bands', () => {
    const ceiling = world.outdoor.noFly.find((z) => z.name === 'ceiling')!;
    const [x0, y0] = ceiling.polygon[0]!;
    const [x2, y2] = ceiling.polygon[2]!;
    // A corner of the zone that lies inside the geofence.
    const cx = Math.min(x0, x2) + 2;
    const cy = Math.max(y0, y2) - 2;
    const s = createSafetyMonitor(mission, world);
    // Under the ceiling band is fine.
    expect(s.check(input({ tSec: 0, pos: [cx, cy, ceiling.minZ - 4] })).kind).toBe('none');
    // Inside the band is a breach.
    expect(s.check(input({ tSec: 1, pos: [cx, cy, ceiling.minZ + 1] })).kind).toBe('return_home');
    expect(s.events().some((e) => e.kind === 'no_fly')).toBe(true);
    // Lookahead into the ravine zone holds.
    const ravine = world.outdoor.noFly.find((z) => z.name === 'ravine')!;
    const rx = Math.max(...ravine.polygon.map((p) => p[0])) - 4;
    const ry = Math.min(...ravine.polygon.map((p) => p[1])) + 2;
    const s2 = createSafetyMonitor(mission, world);
    const r = s2.check(input({ tSec: 0, predicted: [rx, ry, OPEN[2]] }));
    expect(r.kind).toBe('hold');
    expect(s2.events()[0]!.kind).toBe('no_fly');
  });

  it('enforces the altitude band with the near-building exception', () => {
    const s = createSafetyMonitor(mission, world);
    const r = s.check(input({ tSec: 0, pos: at(40, 22, mission.altitude.maxM + 5) }));
    expect(r.kind).toBe('hold');
    expect(s.events()[0]!.kind).toBe('altitude');
    // Low over open ground in coverage: hold.
    const s2 = createSafetyMonitor(mission, world);
    expect(s2.check(input({ tSec: 0, pos: at(40, 22, 1) })).kind).toBe('hold');
    // Low beside a building while approaching: allowed.
    const fp = world.buildings[0]!.footprint;
    const beside = at(fp[1]!.x + 2, fp[1]!.y + 2, 1.5);
    const s3 = createSafetyMonitor(mission, world);
    expect(s3.check(input({ tSec: 0, pos: beside, phase: 'approach' })).kind).toBe('none');
    expect(s3.check(input({ tSec: 0.1, pos: beside, phase: 'coverage' })).kind).toBe('hold');
    // Low at home (takeoff) is fine in any phase.
    const s4 = createSafetyMonitor(mission, world);
    expect(s4.check(input({ tSec: 0, pos: world.home, phase: 'preflight' })).kind).toBe('none');
  });

  it('returns home on the battery reserve, exits first when inside, and lands when critical', () => {
    // Budget 80% means a 20% reserve. 30% left, 15% to get home: margin 15 < 20.
    const s = createSafetyMonitor(mission, world);
    const battery = fakeBattery(30, 0.75);
    expect(s.check(input({ tSec: 0, battery, distanceHomeM: 20 })).kind).toBe('return_home');
    expect(s.check(input({ tSec: 0.1, battery, distanceHomeM: 20 })).kind).toBe('return_home');
    expect(s.check(input({ tSec: 0.2, battery, phase: 'return_home' })).kind).toBe('none');
    expect(s.events().filter((e) => e.kind === 'battery_reserve')).toHaveLength(1);

    const inside = createSafetyMonitor(mission, world);
    expect(inside.check(input({ tSec: 0, battery, inside: world.buildings[0]!.id, phase: 'indoor_explore' })).kind).toBe(
      'exit_building',
    );

    const tight = createSafetyMonitor(missionForWorld(world, { missionType: 'find_person', budgets: { batteryPct: 50, timeSec: 900 } }), world);
    // 52% left, 5% home cost: under a 50% reserve from the 50% budget.
    expect(tight.check(input({ tSec: 0, battery: fakeBattery(52, 0.25) })).kind).toBe('return_home');

    const crit = createSafetyMonitor(mission, world);
    const low = fakeBattery(5);
    expect(crit.check(input({ tSec: 0, battery: low })).kind).toBe('land');
    expect(crit.check(input({ tSec: 0.1, battery: low, phase: 'return_home' })).kind).toBe('land');
    expect(crit.events().filter((e) => e.kind === 'battery_critical')).toHaveLength(1);
  });

  it('escalates collisions within the window and records one event per collision', () => {
    const s = createSafetyMonitor(mission, world);
    const inside = world.buildings[0]!.id;
    const kinds: string[] = [];
    let t = 0;
    for (let n = 0; n < 6; n++) {
      // A contact lasting several ticks is one collision.
      const first = s.check(input({ tSec: t, collided: true, inside, phase: 'indoor_explore' }));
      kinds.push(first.kind);
      if (first.kind === 'backtrack') expect(first.meters).toBeGreaterThan(0);
      if (first.kind === 'hold') expect(first.seconds).toBeGreaterThan(0);
      for (let k = 1; k < 4; k++) {
        expect(s.check(input({ tSec: t + k * 0.05, collided: true, inside, phase: 'indoor_explore' })).kind).toBe('none');
      }
      t += 2;
    }
    expect(kinds).toEqual(['hold', 'backtrack', 'exit_building', 'return_home', 'land', 'land']);
    expect(s.events().filter((e) => e.kind === 'collision')).toHaveLength(6);

    const outdoor = createSafetyMonitor(mission, world);
    outdoor.check(input({ tSec: 0, collided: true }));
    outdoor.check(input({ tSec: 2, collided: true }));
    expect(outdoor.check(input({ tSec: 4, collided: true })).kind).toBe('return_home');
    // Outside the window the ladder resets.
    expect(outdoor.check(input({ tSec: 100, collided: true })).kind).toBe('hold');
  });

  it('does not spam events while a condition persists', () => {
    const s = createSafetyMonitor(mission, world);
    const high = at(40, 22, mission.altitude.maxM + 5);
    for (let t = 0; t < 20; t += 0.05) s.check(input({ tSec: t, pos: high }));
    expect(s.events()).toHaveLength(1);
  });
});

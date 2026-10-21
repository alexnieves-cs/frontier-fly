import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import Ajv2020 from 'ajv/dist/2020';
import { describe, expect, it } from 'vitest';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import {
  buildFlightPlan,
  exportStem,
  type ExportOptions,
  type FlightPlan,
} from '../../src/mission/export/flightPlan';
import { mavlinkJson, qgcPlanJson, toMavlink, toQgcPlan, toWpl } from '../../src/mission/export/formats';
import { GEO_ORIGINS, haversineM, localFrame } from '../../src/mission/export/geo';
import { importQgcPlan, planStructureIssues } from '../../src/mission/export/importPlan';
import { MAV_CMD, MAV_CMD_NAMES, MAV_FRAME } from '../../src/mission/export/mavlink';
import planSchema from '../../src/mission/export/qgc-plan.schema.json';
import {
  routeFromCoverage,
  routeFromPreview,
  routeFromWaypoints,
  simplify,
  type MissionRoute,
  type RoutePoint,
} from '../../src/mission/export/route';
import { isOperable, pathIsOperable } from '../../src/mission/path';
import { sampleMission } from '../../src/mission/sample';
import type { Mission } from '../../src/mission/types';
import { validateMission } from '../../src/mission/validate';

// Mutation tests poke at arbitrary depths of parsed JSON.
// eslint-disable-next-line @typescript-eslint/no-explicit-any
type Loose = Record<string, any>;

const FIXTURES = join(__dirname, '../fixtures/qgc');
const QGC_PLANS = ['PX4SITL-PX4-MultiRotor.plan', 'ArduPilotSITL-ArduCopter.plan', 'TerrainTest-ArduPlane.plan', 'SectionTest.plan'];

const ajv = new Ajv2020({ strict: true, strictRequired: false, allErrors: true });
const validatePlan = ajv.compile(planSchema);

function schemaErrors(data: unknown): string[] {
  return validatePlan(data) ? [] : (validatePlan.errors ?? []).map((e) => `${e.instancePath || '/'} ${e.message ?? ''}`);
}

const PX4: ExportOptions = {
  home: GEO_ORIGINS['px4-sitl'],
  firmware: 'px4',
  altitudeRef: 'relative',
  finish: 'rtl',
  video: false,
};

const VARIANTS: ExportOptions[] = [
  PX4,
  { ...PX4, video: true, finish: 'land' },
  { ...PX4, altitudeRef: 'terrain' },
  { ...PX4, home: GEO_ORIGINS['ardupilot-sitl'], firmware: 'ardupilot' },
  { ...PX4, home: GEO_ORIGINS['ardupilot-sitl'], firmware: 'ardupilot', finish: 'land', video: true, altitudeRef: 'terrain' },
  // Far north and across the antimeridian: the geodetic math has no flat-earth shortcuts.
  { ...PX4, home: { latDeg: 69.6492, lonDeg: 179.9995, altM: 12 } },
];

interface Case {
  name: string;
  mission: Mission;
  route: MissionRoute;
}

const cases: Case[] = [];
for (const seed of [11, 4242, 5150]) {
  const world = buildMissionWorld({ seed });
  const mission = missionForWorld(world, { missionType: 'find_person', name: `Woods ${seed}` });
  cases.push({ name: `woods seed ${seed}`, mission, route: routeFromCoverage(mission, world) });
}
{
  const mission = sampleMission(11);
  cases.push({ name: 'mock practice site', mission, route: routeFromPreview(mission) });
}

function allPoints(route: MissionRoute): RoutePoint[] {
  const out: RoutePoint[] = [];
  for (const leg of route.legs) {
    for (const p of leg.points) {
      const last = out[out.length - 1];
      if (!last || Math.hypot(p.x - last.x, p.y - last.y, p.z - last.z) > 1e-6) out.push(p);
    }
  }
  return out;
}

describe('geodetic frame', () => {
  it('round-trips local meters through WGS84 to well under a millimeter', () => {
    for (const origin of [GEO_ORIGINS['px4-sitl'], GEO_ORIGINS['ardupilot-sitl'], { latDeg: 69.6492, lonDeg: 179.9995, altM: 12 }]) {
      const frame = localFrame(origin);
      for (const [e, n, u] of [
        [0, 0, 0],
        [1500, -800, 40],
        [-2000, 2000, 0],
        [12.345, 678.9, 13],
      ] as const) {
        const back = frame.toLocal(frame.toGeo(e, n, u));
        expect(Math.abs(back.east - e)).toBeLessThan(1e-4);
        expect(Math.abs(back.north - n)).toBeLessThan(1e-4);
        expect(Math.abs(back.up - u)).toBeLessThan(1e-4);
      }
    }
  });

  it('puts 1 km east and north at the right great-circle distance and bearing', () => {
    const origin = GEO_ORIGINS['px4-sitl'];
    const frame = localFrame(origin);
    const east = frame.toGeo(1000, 0);
    const north = frame.toGeo(0, 1000);
    // The mean-sphere distance differs from the ellipsoid by a few tenths of a percent.
    expect(Math.abs(haversineM(origin, east) - 1000)).toBeLessThan(5);
    expect(Math.abs(haversineM(origin, north) - 1000)).toBeLessThan(5);
    // Exact to first order: east steps follow the prime-vertical radius N, north steps the meridian radius M.
    const lat = (origin.latDeg * Math.PI) / 180;
    const e2 = (1 / 298.257223563) * (2 - 1 / 298.257223563);
    const w = Math.sqrt(1 - e2 * Math.sin(lat) ** 2);
    const n = 6378137 / w + origin.altM;
    const m = (6378137 * (1 - e2)) / w ** 3 + origin.altM;
    const tiny = localFrame(origin).toGeo(10, 10);
    expect(((tiny.lonDeg - origin.lonDeg) * Math.PI) / 180 * n * Math.cos(lat)).toBeCloseTo(10, 4);
    expect(((tiny.latDeg - origin.latDeg) * Math.PI) / 180 * m).toBeCloseTo(10, 4);
    expect(east.lonDeg).toBeGreaterThan(origin.lonDeg);
    expect(Math.abs(east.latDeg - origin.latDeg)).toBeLessThan(1e-3);
    expect(north.latDeg).toBeGreaterThan(origin.latDeg);
    // One meter of latitude near 47.4 N is about 8.99e-6 degrees.
    expect(frame.toGeo(0, 1).latDeg - origin.latDeg).toBeCloseTo(8.99e-6, 8);
  });

  it('refuses an origin off the globe', () => {
    expect(() => localFrame({ latDeg: 91, lonDeg: 0, altM: 0 })).toThrow(RangeError);
    expect(() => localFrame({ latDeg: 0, lonDeg: Number.NaN, altM: 0 })).toThrow(RangeError);
  });
});

describe('route', () => {
  it('simplify keeps the ends and stays within tolerance of every dropped point', () => {
    const pts: RoutePoint[] = [];
    for (let i = 0; i <= 60; i++) pts.push({ x: i, y: Math.sin(i / 6) * 3, z: 10 + 0.05 * i, groundZ: 0 });
    const kept = simplify(pts, 0.25);
    expect(kept[0]).toEqual(pts[0]);
    expect(kept[kept.length - 1]).toEqual(pts[pts.length - 1]);
    expect(kept.length).toBeLessThan(pts.length);
    for (const p of pts) {
      let best = Infinity;
      for (let k = 1; k < kept.length; k++) {
        const a = kept[k - 1]!;
        const b = kept[k]!;
        const ab = [b.x - a.x, b.y - a.y, b.z - a.z];
        const ap = [p.x - a.x, p.y - a.y, p.z - a.z];
        const t = Math.max(0, Math.min(1, (ap[0]! * ab[0]! + ap[1]! * ab[1]! + ap[2]! * ab[2]!) / (ab[0]! ** 2 + ab[1]! ** 2 + ab[2]! ** 2)));
        best = Math.min(best, Math.hypot(ap[0]! - ab[0]! * t, ap[1]! - ab[1]! * t, ap[2]! - ab[2]! * t));
      }
      expect(best).toBeLessThanOrEqual(0.25 + 1e-9);
    }
  });

  it('labels waypoints from any planner, with a fallback leg kind', () => {
    const route = routeFromWaypoints(
      [
        [0, 0, 10],
        [10, 0, 10],
        [20, 0, 10],
      ],
      { source: 'coverage', home: { x: 0, y: 0, z: 0, groundZ: 0 }, cruiseAglM: 10, speedMps: 5, ground: () => 0, leg: 'search' },
    );
    expect(route.legs).toHaveLength(1);
    expect(route.legs[0]!.kind).toBe('search');
    // The middle point is collinear and drops out.
    expect(route.legs[0]!.points.map((p) => p.x)).toEqual([0, 20]);
  });

  for (const c of cases) {
    it(`${c.name}: stays in the geofence, out of no-fly zones and inside the altitude band`, () => {
      const pts = allPoints(c.route);
      expect(pts.length).toBeGreaterThan(4);
      expect(c.route.legs.some((leg) => leg.kind === 'sweep')).toBe(true);
      for (const p of pts) {
        expect(isOperable(p, c.mission), `(${p.x}, ${p.y})`).toBe(true);
        const agl = p.z - p.groundZ;
        expect(agl).toBeGreaterThanOrEqual(c.mission.altitude.minM - 1e-6);
        expect(agl).toBeLessThanOrEqual(c.mission.altitude.maxM + 1e-6);
      }
      // Simplification never cuts a corner through a no-fly zone.
      expect(pathIsOperable(c.mission, pts)).toBe(true);
    });
  }
});

describe('QGroundControl .plan', () => {
  it('the schema accepts every one of QGroundControl\'s own test plans', () => {
    for (const file of QGC_PLANS) {
      const data: unknown = JSON.parse(readFileSync(join(FIXTURES, file), 'utf8'));
      expect(schemaErrors(data), file).toEqual([]);
      expect(planStructureIssues(data), file).toEqual([]);
    }
  });

  for (const c of cases) {
    it(`${c.name}: every export variant validates against the schema`, () => {
      for (const options of VARIANTS) {
        const plan = buildFlightPlan(c.mission, c.route, options);
        const json: unknown = JSON.parse(qgcPlanJson(plan));
        expect(schemaErrors(json), `${options.firmware} ${options.altitudeRef} ${options.finish}`).toEqual([]);
        expect(planStructureIssues(json)).toEqual([]);
      }
    });
  }

  it('rejects the mistakes QGroundControl rejects, in both the schema and the loader checks', () => {
    const plan = toQgcPlan(buildFlightPlan(cases[0]!.mission, cases[0]!.route, PX4));
    const mutants: [string, (p: Loose) => void][] = [
      ['wrong file type', (p) => (p.fileType = 'Mission')],
      ['newer plan version', (p) => (p.version = 2)],
      ['missing groundStation', (p) => delete p.groundStation],
      ['missing geoFence', (p) => delete p.geoFence],
      ['missing rallyPoints', (p) => delete p.rallyPoints],
      ['mission version 3', (p) => (p.mission.version = 3)],
      ['missing firmwareType', (p) => delete p.mission.firmwareType],
      ['home without altitude', (p) => (p.mission.plannedHomePosition = [47, 8])],
      ['six params', (p) => p.mission.items[3].params.pop()],
      ['coordinate with seven params', (p) => (p.mission.items[3].coordinate = [47, 8, 10])],
      ['legacy param keys without coordinate', (p) => {
        const item = p.mission.items[3];
        [item.param1, item.param2, item.param3, item.param4] = item.params.slice(0, 4);
        delete item.params;
      }],
      ['string param', (p) => (p.mission.items[3].params[0] = '0')],
      ['missing autoContinue', (p) => delete p.mission.items[3].autoContinue],
      ['Altitude without AltitudeMode', (p) => delete p.mission.items[3].AltitudeMode],
      ['unknown item type', (p) => (p.mission.items[3].type = 'MissionItem')],
      ['complex item without complexItemType', (p) => (p.mission.items[3] = { type: 'ComplexItem' })],
      ['fence version 3', (p) => (p.geoFence.version = 3)],
      ['fence polygon version 2', (p) => (p.geoFence.polygons[0].version = 2)],
      ['fence polygon without inclusion', (p) => delete p.geoFence.polygons[0].inclusion],
      ['fence vertex with altitude', (p) => p.geoFence.polygons[0].polygon[0].push(10)],
      ['circle without radius', (p) => p.geoFence.circles.push({ version: 1, inclusion: true, circle: { center: [47, 8] } })],
      ['rally point without altitude', (p) => p.rallyPoints.points.push([47, 8])],
      ['rally version 3', (p) => (p.rallyPoints.version = 3)],
    ];
    for (const [name, mutate] of mutants) {
      const copy = JSON.parse(JSON.stringify(plan)) as Loose;
      mutate(copy);
      expect(schemaErrors(copy).length, `schema: ${name}`).toBeGreaterThan(0);
      expect(planStructureIssues(copy).length, `loader: ${name}`).toBeGreaterThan(0);
    }
    // QGC ignores version-1 fence and rally sections, so both checks accept them.
    const legacy = JSON.parse(JSON.stringify(plan)) as Loose;
    legacy.geoFence = { version: 1, polygon: [] };
    legacy.rallyPoints = { version: 1, points: [] };
    expect(schemaErrors(legacy)).toEqual([]);
    expect(planStructureIssues(legacy)).toEqual([]);
  });

  it('flies takeoff, speed, the route in order, then return to launch', () => {
    const c = cases[1]!;
    const plan = buildFlightPlan(c.mission, c.route, { ...PX4, video: true });
    const qgc = toQgcPlan(plan);
    const items = qgc.mission.items;
    expect(items[0]!.command).toBe(MAV_CMD.NAV_TAKEOFF);
    expect(items[1]!.command).toBe(MAV_CMD.DO_CHANGE_SPEED);
    expect(items[1]!.params[1]).toBeCloseTo(c.route.speedMps);
    expect(items[2]!.command).toBe(MAV_CMD.VIDEO_START_CAPTURE);
    expect(items[items.length - 2]!.command).toBe(MAV_CMD.VIDEO_STOP_CAPTURE);
    expect(items[items.length - 1]!.command).toBe(MAV_CMD.NAV_RETURN_TO_LAUNCH);
    expect(items.map((item) => item.doJumpId)).toEqual(items.map((_, i) => i + 1));
    expect(qgc.mission.plannedHomePosition).toEqual([GEO_ORIGINS['px4-sitl'].latDeg, GEO_ORIGINS['px4-sitl'].lonDeg, 488]);
    for (const item of items) expect(MAV_CMD_NAMES[item.command], String(item.command)).toBeDefined();

    // Every waypoint converts back to its route point within MAVLink's 1e-7 degree resolution.
    const frame = localFrame(GEO_ORIGINS['px4-sitl']);
    const wps = items.filter((item) => item.command === MAV_CMD.NAV_WAYPOINT);
    const pts = allPoints(c.route);
    expect(wps).toHaveLength(pts.length);
    wps.forEach((item, i) => {
      const p = pts[i]!;
      const local = frame.toLocal({ latDeg: item.params[4]!, lonDeg: item.params[5]!, altM: 488 });
      expect(Math.hypot(local.east - (p.x - c.route.home.x), local.north - (p.y - c.route.home.y))).toBeLessThan(0.02);
      expect(item.frame).toBe(MAV_FRAME.GLOBAL_RELATIVE_ALT);
      expect(item.params[6]).toBeCloseTo(p.z - c.route.home.groundZ, 2);
      expect(item.Altitude).toBe(item.params[6]);
    });
  });

  it('terrain mode flies heights above the ground under each point', () => {
    const c = cases[2]!;
    const plan = buildFlightPlan(c.mission, c.route, { ...PX4, altitudeRef: 'terrain' });
    const wps = plan.items.filter((item) => item.command === MAV_CMD.NAV_WAYPOINT);
    const pts = allPoints(c.route);
    wps.forEach((item, i) => {
      expect(item.frame).toBe(MAV_FRAME.GLOBAL_TERRAIN_ALT);
      expect(item.altM).toBeCloseTo(pts[i]!.z - pts[i]!.groundZ, 2);
      expect(item.altM).toBeGreaterThanOrEqual(c.mission.altitude.minM - 0.01);
    });
    expect(toQgcPlan(plan).mission.items.find((item) => item.command === MAV_CMD.NAV_WAYPOINT)!.AltitudeMode).toBe(4);
    expect(plan.warnings.some((w) => w.includes('terrain data'))).toBe(true);
  });

  it('lands at the pad when asked instead of returning to launch', () => {
    const c = cases[3]!;
    const plan = buildFlightPlan(c.mission, c.route, { ...PX4, finish: 'land' });
    const last = plan.items[plan.items.length - 1]!;
    const approach = plan.items[plan.items.length - 2]!;
    expect(last.command).toBe(MAV_CMD.NAV_LAND);
    expect([last.latDeg, last.lonDeg]).toEqual([plan.home.latDeg, plan.home.lonDeg]);
    expect([approach.latDeg, approach.lonDeg]).toEqual([plan.home.latDeg, plan.home.lonDeg]);
    expect(approach.altM).toBeGreaterThanOrEqual(c.route.cruiseAglM);
  });

  it('writes the geofence as an inclusion polygon and each no-fly zone as an exclusion', () => {
    for (const c of cases) {
      const qgc = toQgcPlan(buildFlightPlan(c.mission, c.route, PX4));
      expect(qgc.geoFence.polygons[0]!.inclusion).toBe(true);
      expect(qgc.geoFence.polygons[0]!.polygon).toHaveLength(c.mission.geofence.points.length);
      expect(qgc.geoFence.polygons.slice(1).every((p) => !p.inclusion)).toBe(true);
      expect(qgc.geoFence.polygons).toHaveLength(1 + c.mission.noFlyZones.length);
    }
  });

  it('is deterministic and keeps keys in QGroundControl\'s byte order', () => {
    const c = cases[0]!;
    const a = qgcPlanJson(buildFlightPlan(c.mission, c.route, PX4));
    const b = qgcPlanJson(buildFlightPlan(c.mission, routeFromCoverage(c.mission, buildMissionWorld({ seed: 11 })), PX4));
    expect(a).toBe(b);
    expect(Object.keys(JSON.parse(a) as object)).toEqual(['fileType', 'geoFence', 'groundStation', 'mission', 'rallyPoints', 'version']);
    expect(a.endsWith('\n')).toBe(true);
    expect(exportStem({ missionName: 'Woods 11 / Ravine search', seed: 11 })).toBe('woods-11-ravine-search-seed11');
  });

  it('estimates the flight and warns when it overruns the time budget', () => {
    const c = cases[0]!;
    const plan = buildFlightPlan(c.mission, c.route, PX4);
    expect(plan.summary.sweepM).toBeGreaterThan(100);
    expect(plan.summary.estimatedSec).toBeGreaterThan(plan.summary.sweepM / c.route.speedMps);
    const tight = buildFlightPlan({ ...c.mission, budgets: { ...c.mission.budgets, timeSec: 30 } }, c.route, PX4);
    expect(tight.warnings.some((w) => w.includes('budget'))).toBe(true);
  });
});

describe('.plan import', () => {
  for (const c of cases) {
    it(`${c.name}: an export re-imports to the same geofence and no-fly zones`, () => {
      for (const options of [PX4, VARIANTS[3]!, VARIANTS[5]!]) {
        const text = qgcPlanJson(buildFlightPlan(c.mission, c.route, options));
        const imported = importQgcPlan(text, c.mission);
        expect(imported.ok, imported.issues.map((i) => i.message).join('; ')).toBe(true);
        const m = imported.mission!;
        const close = (a: { x: number; y: number }, b: { x: number; y: number }) => Math.hypot(a.x - b.x, a.y - b.y) < 0.02;
        expect(m.geofence.points.every((p, i) => close(p, c.mission.geofence.points[i]!))).toBe(true);
        expect(m.noFlyZones).toHaveLength(c.mission.noFlyZones.length);
        m.noFlyZones.forEach((zone, z) => {
          expect(zone.polygon.points.every((p, i) => close(p, c.mission.noFlyZones[z]!.polygon.points[i]!))).toBe(true);
        });
        expect(imported.counts.missionItems).toBeGreaterThan(3);
      }
    });
  }

  it('imports a circle fence as a polygon around the start', () => {
    const base = sampleMission(11);
    const home = GEO_ORIGINS['px4-sitl'];
    const frame = localFrame(home);
    const c = frame.toGeo(0, 0);
    const text = JSON.stringify({
      fileType: 'Plan',
      version: 1,
      groundStation: 'QGroundControl',
      mission: { version: 2, firmwareType: 12, plannedHomePosition: [home.latDeg, home.lonDeg, home.altM], items: [] },
      geoFence: { version: 2, polygons: [], circles: [{ version: 1, inclusion: true, circle: { center: [c.latDeg, c.lonDeg], radius: 40 } }] },
      rallyPoints: { version: 2, points: [] },
    });
    const imported = importQgcPlan(text, base);
    expect(imported.ok, imported.issues.map((i) => i.message).join('; ')).toBe(true);
    for (const p of imported.mission!.geofence.points) {
      expect(Math.hypot(p.x - base.startPose.x, p.y - base.startPose.y)).toBeCloseTo(40, 1);
    }
  });

  it('refuses plans it cannot use, with a reason', () => {
    const base = sampleMission(11);
    expect(importQgcPlan('{nope', base).issues[0]!.message).toMatch(/JSON/);
    const fixture = readFileSync(join(FIXTURES, 'PX4SITL-PX4-MultiRotor.plan'), 'utf8');
    expect(importQgcPlan(fixture, base).issues[0]!.message).toMatch(/no inclusion fence/);
    const broken = JSON.parse(fixture) as Loose;
    broken.mission.version = 9;
    expect(importQgcPlan(JSON.stringify(broken), base).ok).toBe(false);
    // A fence on another continent is not a mission over this site.
    const far = toQgcPlan(buildFlightPlan(cases[3]!.mission, cases[3]!.route, PX4));
    far.geoFence.polygons[0]!.polygon[0] = [10, 10];
    expect(importQgcPlan(JSON.stringify(far), base).issues[0]!.message).toMatch(/km from home/);
    // A fence the mission rules reject comes back with the validator's issues.
    const tiny = toQgcPlan(buildFlightPlan(cases[3]!.mission, cases[3]!.route, PX4));
    tiny.geoFence.polygons = [tiny.geoFence.polygons[0]!];
    tiny.geoFence.polygons[0]!.polygon = tiny.geoFence.polygons[0]!.polygon.slice(0, 2);
    const res = importQgcPlan(JSON.stringify(tiny), base);
    expect(res.ok).toBe(false);
    expect(res.issues.some((i) => i.path.startsWith('geofence'))).toBe(true);
  });
});

/** QGC's text loader (MissionController::_loadTextMissionFile + MissionItem::load), as a strict parser. */
function parseWpl(text: string): { seq: number; current: number; frame: number; command: number; values: number[]; autoContinue: number }[] {
  const lines = text.split(/\r?\n/u).filter((line) => line.length > 0);
  const [head, ...rows] = lines;
  const version = head!.split(' ');
  if (version.length !== 3 || version[0] !== 'QGC' || version[1] !== 'WPL' || !['110', '120'].includes(version[2]!)) {
    throw new Error(`bad header ${head}`);
  }
  return rows.map((row, index) => {
    const f = row.split('\t');
    if (f.length !== 12) throw new Error(`line ${index + 1} has ${f.length} fields`);
    const nums = f.map(Number);
    if (nums.some((n) => !Number.isFinite(n))) throw new Error(`line ${index + 1} has a non-number`);
    return { seq: nums[0]!, current: nums[1]!, frame: nums[2]!, command: nums[3]!, values: nums.slice(4, 11), autoContinue: nums[11]! };
  });
}

describe('QGC WPL 110 and MAVLink mission items', () => {
  it('the WPL parser reads Mission Planner\'s own file', () => {
    const rows = parseWpl(readFileSync(join(FIXTURES, 'MissionPlanner.waypoints'), 'utf8'));
    expect(rows.length).toBeGreaterThan(3);
    expect(rows[0]!.seq).toBe(0);
  });

  for (const options of VARIANTS) {
    it(`WPL and MAVLink match the plan item for item (${options.firmware}, ${options.altitudeRef}, ${options.finish}${options.video ? ', video' : ''})`, () => {
      const c = cases[2]!;
      const plan: FlightPlan = buildFlightPlan(c.mission, c.route, options);
      const rows = parseWpl(toWpl(plan));
      expect(rows).toHaveLength(plan.items.length + 1);
      expect(rows.map((r) => r.seq)).toEqual(rows.map((_, i) => i));
      expect(rows[0]!.frame).toBe(MAV_FRAME.GLOBAL);
      expect(rows[0]!.values[6]).toBeCloseTo(plan.home.altM, 2);
      plan.items.forEach((item, i) => {
        const r = rows[i + 1]!;
        expect(r.command).toBe(item.command);
        expect(r.frame).toBe(item.frame);
        expect(r.current).toBe(0);
        expect(r.values[4]).toBeCloseTo(item.latDeg, 7);
        expect(r.values[5]).toBeCloseTo(item.lonDeg, 7);
        expect(r.values[6]).toBeCloseTo(item.altM, 2);
      });

      const mav = toMavlink(plan);
      expect(JSON.parse(mavlinkJson(plan))).toEqual(mav);
      const offset = options.firmware === 'ardupilot' ? 1 : 0;
      expect(mav.homeAtSeq0).toBe(options.firmware === 'ardupilot');
      expect(mav.mission).toHaveLength(plan.items.length + offset);
      for (const list of [mav.mission, mav.fence, mav.rally]) {
        expect(list.map((item) => item.seq)).toEqual(list.map((_, i) => i));
        for (const item of list) {
          expect(Number.isInteger(item.x) && Math.abs(item.x) <= 2 ** 31 - 1).toBe(true);
          expect(Number.isInteger(item.y) && Math.abs(item.y) <= 2 ** 31 - 1).toBe(true);
        }
      }
      expect(mav.mission.filter((item) => item.current === 1)).toHaveLength(1);
      plan.items.forEach((item, i) => {
        const m = mav.mission[i + offset]!;
        expect(m.command).toBe(item.command);
        if (item.frame !== MAV_FRAME.MISSION) expect(m.x).toBe(Math.round(item.latDeg * 1e7));
      });
      // Fence: one item per vertex; param1 carries the vertex count of its polygon.
      expect(mav.fence).toHaveLength(plan.fence.reduce((n, p) => n + p.vertices.length, 0));
      let at = 0;
      for (const polygon of plan.fence) {
        for (let v = 0; v < polygon.vertices.length; v++) {
          const f = mav.fence[at++]!;
          expect(f.param1).toBe(polygon.vertices.length);
          expect(f.command).toBe(
            polygon.inclusion ? MAV_CMD.NAV_FENCE_POLYGON_VERTEX_INCLUSION : MAV_CMD.NAV_FENCE_POLYGON_VERTEX_EXCLUSION,
          );
        }
      }
    });
  }
});

describe('exported missions are valid missions', () => {
  it('every test mission passes the mission validator', () => {
    for (const c of cases) expect(validateMission(c.mission).ok, c.name).toBe(true);
  });
});

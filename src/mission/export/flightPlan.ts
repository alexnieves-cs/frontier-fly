import type { Mission, Point } from '../types';
import { localFrame, type GeoPoint, type LocalFrame } from './geo';
import { MAV_CMD, MAV_FRAME, NAV_COMMANDS, SPEED_TYPE_GROUND, type MavFrame } from './mavlink';
import { routeLength, type LegKind, type MissionRoute, type RoutePoint } from './route';

/**
 * A mission as an autopilot sees it: geodetic mission items, fence polygons
 * and rally points. Every file writer (QGroundControl .plan, QGC WPL 110,
 * MAVLink MISSION_ITEM_INT) renders this one structure, so they cannot drift.
 */

export type Firmware = 'px4' | 'ardupilot';
export type AltitudeRef = 'relative' | 'terrain';
export type Finish = 'rtl' | 'land';

export interface ExportOptions {
  /** Where the takeoff pad lands on the globe. Altitude is the pad's height above sea level. */
  home: GeoPoint;
  firmware: Firmware;
  /** `relative`: heights above the pad, so the sim's 3D profile is flown as is. `terrain`: heights above the ground under each point. */
  altitudeRef: AltitudeRef;
  /** End with return-to-launch, or with an explicit approach and landing at the pad. */
  finish: Finish;
  /** Start video recording after takeoff and stop it before the return. */
  video: boolean;
  /** Waypoint acceptance radius, meters. Default 2. */
  acceptanceRadiusM?: number;
}

export interface FlightItem {
  command: number;
  frame: MavFrame;
  /** param1 to param4. NaN means "unset, keep the autopilot's default". */
  params: [number, number, number, number];
  /** Latitude, longitude, altitude for navigation items; zeros for commands. */
  latDeg: number;
  lonDeg: number;
  altM: number;
  autoContinue: boolean;
  /** Which part of the flight emitted the item. */
  leg: LegKind | 'takeoff' | 'command' | 'return';
}

export interface FencePolygon {
  inclusion: boolean;
  /** Mission no-fly zone id, or 'geofence'. */
  id: string;
  name: string;
  vertices: { latDeg: number; lonDeg: number }[];
}

export interface FlightSummary {
  waypoints: number;
  items: number;
  sweepM: number;
  transitM: number;
  searchM: number;
  /** Rough flight time: horizontal legs at the cruise speed plus the climb and descent. */
  estimatedSec: number;
  maxAltM: number;
}

export interface FlightPlan {
  missionId: string;
  missionName: string;
  seed: number;
  firmware: Firmware;
  altitudeRef: AltitudeRef;
  source: MissionRoute['source'];
  /** Pad position, AMSL altitude. */
  home: GeoPoint;
  cruiseSpeedMps: number;
  items: FlightItem[];
  fence: FencePolygon[];
  rally: GeoPoint[];
  summary: FlightSummary;
  warnings: string[];
}

/** ArduPilot boards with 4 KB of mission storage hold about 700 items; flag well before that. */
export const ITEM_WARN_LIMIT = 500;
/** Climb and descent rate the sim vehicle uses, m/s (OUTDOOR_LIMITS.speedV). */
const VERTICAL_SPEED_MPS = 1.5;

function round(value: number, digits: number): number {
  const k = 10 ** digits;
  const out = Math.round(value * k) / k;
  return Object.is(out, -0) ? 0 : out;
}

/** 1e-7 degrees is MAVLink's integer resolution (about 1.1 cm). */
function geo(frame: LocalFrame, home: RoutePoint, p: { x: number; y: number }): { latDeg: number; lonDeg: number } {
  const g = frame.toGeo(p.x - home.x, p.y - home.y, 0);
  return { latDeg: round(g.latDeg, 7), lonDeg: round(g.lonDeg, 7) };
}

function ringVertices(frame: LocalFrame, home: RoutePoint, points: readonly Point[]): FencePolygon['vertices'] {
  return points.map((p) => geo(frame, home, p));
}

function nav(
  command: number,
  frame: MavFrame,
  at: { latDeg: number; lonDeg: number },
  altM: number,
  params: FlightItem['params'],
  leg: FlightItem['leg'],
): FlightItem {
  return { command, frame, params, latDeg: at.latDeg, lonDeg: at.lonDeg, altM: round(altM, 2), autoContinue: true, leg };
}

function cmd(command: number, params: FlightItem['params']): FlightItem {
  return { command, frame: MAV_FRAME.MISSION, params, latDeg: 0, lonDeg: 0, altM: 0, autoContinue: true, leg: 'command' };
}

export function buildFlightPlan(mission: Mission, route: MissionRoute, options: ExportOptions): FlightPlan {
  const warnings: string[] = [];
  const frame = localFrame(options.home);
  const home = route.home;
  const radius = options.acceptanceRadiusM ?? 2;
  const terrain = options.altitudeRef === 'terrain';
  const wpFrame = terrain ? MAV_FRAME.GLOBAL_TERRAIN_ALT : MAV_FRAME.GLOBAL_RELATIVE_ALT;
  const altOf = (p: RoutePoint): number => (terrain ? p.z - p.groundZ : p.z - home.groundZ);
  const homeGeo = { latDeg: round(options.home.latDeg, 7), lonDeg: round(options.home.lonDeg, 7) };

  const items: FlightItem[] = [];
  const takeoffAlt = route.cruiseAglM;
  items.push(nav(MAV_CMD.NAV_TAKEOFF, MAV_FRAME.GLOBAL_RELATIVE_ALT, homeGeo, takeoffAlt, [0, 0, 0, NaN], 'takeoff'));
  items.push(cmd(MAV_CMD.DO_CHANGE_SPEED, [SPEED_TYPE_GROUND, round(route.speedMps, 2), -1, 0]));
  if (options.video) items.push(cmd(MAV_CMD.VIDEO_START_CAPTURE, [0, 0, 0, 0]));

  let last: RoutePoint | null = null;
  let waypoints = 0;
  let maxAlt = takeoffAlt;
  for (const leg of route.legs) {
    for (const p of leg.points) {
      if (!Number.isFinite(p.x) || !Number.isFinite(p.y) || !Number.isFinite(p.z) || !Number.isFinite(p.groundZ)) {
        throw new RangeError('The route has a non-finite point.');
      }
      // Legs share their corner point; emit it once.
      if (last && Math.hypot(p.x - last.x, p.y - last.y, p.z - last.z) < 1e-6) continue;
      const alt = altOf(p);
      maxAlt = Math.max(maxAlt, alt);
      items.push(nav(MAV_CMD.NAV_WAYPOINT, wpFrame, geo(frame, home, p), alt, [0, radius, 0, NaN], leg.kind));
      waypoints += 1;
      last = p;
    }
  }
  if (waypoints === 0) warnings.push('The route has no waypoints: the geofence leaves no operable sweep lanes.');

  if (options.video) items.push(cmd(MAV_CMD.VIDEO_STOP_CAPTURE, [0, 0, 0, 0]));
  if (options.finish === 'rtl') {
    items.push({ ...cmd(MAV_CMD.NAV_RETURN_TO_LAUNCH, [0, 0, 0, 0]), leg: 'return' });
  } else {
    const approach = last ? Math.max(route.cruiseAglM, altOf(last)) : route.cruiseAglM;
    items.push(nav(MAV_CMD.NAV_WAYPOINT, MAV_FRAME.GLOBAL_RELATIVE_ALT, homeGeo, approach, [0, radius, 0, NaN], 'return'));
    items.push(nav(MAV_CMD.NAV_LAND, MAV_FRAME.GLOBAL_RELATIVE_ALT, homeGeo, 0, [0, 0, 0, NaN], 'return'));
  }

  const fence: FencePolygon[] = [
    { inclusion: true, id: 'geofence', name: 'Geofence', vertices: ringVertices(frame, home, mission.geofence.points) },
    ...mission.noFlyZones.map((zone) => ({
      inclusion: false,
      id: zone.id,
      name: zone.name,
      vertices: ringVertices(frame, home, zone.polygon.points),
    })),
  ];

  const sweepM = routeLength(route, 'sweep');
  const transitM = routeLength(route, 'transit');
  const searchM = routeLength(route, 'search');
  const first = route.legs[0]?.points[0];
  const out = first ? Math.hypot(first.x - home.x, first.y - home.y) : 0;
  const back = last ? Math.hypot(last.x - home.x, last.y - home.y) : 0;
  const horizontal = sweepM + transitM + searchM + out + back;
  const estimatedSec = horizontal / Math.max(0.1, route.speedMps) + (2 * takeoffAlt) / VERTICAL_SPEED_MPS;
  if (estimatedSec > mission.budgets.timeSec) {
    warnings.push(
      `The planned flight takes about ${Math.round(estimatedSec)} s, longer than the mission's ${mission.budgets.timeSec} s budget.`,
    );
  }
  if (items.length > ITEM_WARN_LIMIT) {
    warnings.push(`${items.length} mission items. Some ArduPilot boards hold about 700; raise the simplify tolerance if an upload fails.`);
  }
  if (terrain) {
    warnings.push('Terrain-relative heights need terrain data on the vehicle, and real terrain at the chosen home differs from the sim.');
  }

  return {
    missionId: mission.id,
    missionName: mission.name,
    seed: mission.seed,
    firmware: options.firmware,
    altitudeRef: options.altitudeRef,
    source: route.source,
    home: { latDeg: homeGeo.latDeg, lonDeg: homeGeo.lonDeg, altM: round(options.home.altM, 2) },
    cruiseSpeedMps: route.speedMps,
    items,
    fence,
    rally: [],
    summary: {
      waypoints,
      items: items.length,
      sweepM: round(sweepM, 1),
      transitM: round(transitM, 1),
      searchM: round(searchM, 1),
      estimatedSec: Math.round(estimatedSec),
      maxAltM: round(maxAlt, 2),
    },
    warnings,
  };
}

export function isNavItem(item: FlightItem): boolean {
  return NAV_COMMANDS.has(item.command);
}

/** File-name stem: mission name and seed, lowercase, safe on every OS. */
export function exportStem(plan: Pick<FlightPlan, 'missionName' | 'seed'>): string {
  const slug = plan.missionName
    .toLowerCase()
    .normalize('NFKD')
    .replace(/[^a-z0-9]+/gu, '-')
    .replace(/^-+|-+$/gu, '')
    .slice(0, 48);
  return `${slug || 'mission'}-seed${plan.seed >>> 0}`;
}

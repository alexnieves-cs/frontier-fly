import { isNavItem, type FlightItem, type FlightPlan } from './flightPlan';
import { MAV_AUTOPILOT, MAV_CMD, MAV_FRAME, MAV_MISSION_TYPE, MAV_TYPE } from './mavlink';

/**
 * File writers over one FlightPlan.
 *
 * - QGroundControl `.plan` (JSON), the format QGC saves and loads. Field rules
 *   follow QGC's loaders (PlanMasterController, MissionController,
 *   SimpleMissionItem, GeoFenceController, RallyPointController).
 * - `QGC WPL 110` text, read by QGC, Mission Planner and MAVProxy. Line 0 is
 *   the home position, as ArduPilot expects.
 * - MAVLink MISSION_ITEM_INT field lists for the mission, fence and rally
 *   mission types, for scripted uploads with pymavlink or MAVSDK.
 */

// ------------------------------------------------------------------ .plan

export interface QgcSimpleItem {
  type: 'SimpleItem';
  autoContinue: boolean;
  command: number;
  doJumpId: number;
  frame: number;
  params: (number | null)[];
  AMSLAltAboveTerrain?: null;
  Altitude?: number;
  AltitudeMode?: number;
}

export interface QgcPlan {
  fileType: 'Plan';
  version: 1;
  groundStation: string;
  mission: {
    version: 2;
    firmwareType: number;
    vehicleType: number;
    cruiseSpeed: number;
    hoverSpeed: number;
    globalPlanAltitudeMode: number;
    plannedHomePosition: [number, number, number];
    items: QgcSimpleItem[];
  };
  geoFence: {
    version: 2;
    circles: { version: 1; inclusion: boolean; circle: { center: [number, number]; radius: number } }[];
    polygons: { version: 1; inclusion: boolean; polygon: [number, number][] }[];
  };
  rallyPoints: { version: 2; points: [number, number, number][] };
}

/** QGroundControlQmlGlobal::AltitudeFrame. */
export const QGC_ALTITUDE_MODE = { MIXED: 0, RELATIVE: 1, ABSOLUTE: 2, CALC_ABOVE_TERRAIN: 3, TERRAIN: 4, NONE: 5 } as const;

/** QGC uses this for the fixed-wing cruise default; a multirotor flies hoverSpeed. */
const QGC_DEFAULT_CRUISE = 15;

export const PLAN_GROUND_STATION = 'frontier-fly';

function nullable(value: number): number | null {
  return Number.isFinite(value) ? value : null;
}

function altitudeMode(item: FlightItem): number {
  if (item.frame === MAV_FRAME.GLOBAL_TERRAIN_ALT) return QGC_ALTITUDE_MODE.TERRAIN;
  if (item.frame === MAV_FRAME.GLOBAL) return QGC_ALTITUDE_MODE.ABSOLUTE;
  return QGC_ALTITUDE_MODE.RELATIVE;
}

function qgcItem(item: FlightItem, doJumpId: number): QgcSimpleItem {
  const params = [...item.params.map(nullable), item.latDeg, item.lonDeg, item.altM];
  const base: QgcSimpleItem = {
    type: 'SimpleItem',
    autoContinue: item.autoContinue,
    command: item.command,
    doJumpId,
    frame: item.frame,
    params,
  };
  // QGC stores its own altitude fields only on items that carry an altitude; when present, all three are required.
  if (isNavItem(item)) {
    return { AMSLAltAboveTerrain: null, Altitude: item.altM, AltitudeMode: altitudeMode(item), ...base };
  }
  return base;
}

export function toQgcPlan(plan: FlightPlan): QgcPlan {
  return {
    fileType: 'Plan',
    version: 1,
    groundStation: PLAN_GROUND_STATION,
    mission: {
      version: 2,
      firmwareType: plan.firmware === 'px4' ? MAV_AUTOPILOT.PX4 : MAV_AUTOPILOT.ARDUPILOTMEGA,
      vehicleType: MAV_TYPE.QUADROTOR,
      cruiseSpeed: QGC_DEFAULT_CRUISE,
      hoverSpeed: plan.cruiseSpeedMps,
      globalPlanAltitudeMode: plan.altitudeRef === 'terrain' ? QGC_ALTITUDE_MODE.TERRAIN : QGC_ALTITUDE_MODE.RELATIVE,
      plannedHomePosition: [plan.home.latDeg, plan.home.lonDeg, plan.home.altM],
      items: plan.items.map((item, index) => qgcItem(item, index + 1)),
    },
    geoFence: {
      version: 2,
      circles: [],
      polygons: plan.fence.map((polygon) => ({
        version: 1,
        inclusion: polygon.inclusion,
        polygon: polygon.vertices.map((v) => [v.latDeg, v.lonDeg] as [number, number]),
      })),
    },
    rallyPoints: { version: 2, points: plan.rally.map((p) => [p.latDeg, p.lonDeg, p.altM] as [number, number, number]) },
  };
}

function sortKeys(value: unknown): unknown {
  if (Array.isArray(value)) return value.map(sortKeys);
  if (value && typeof value === 'object') {
    const entries = Object.entries(value as Record<string, unknown>).sort(([a], [b]) => (a < b ? -1 : a > b ? 1 : 0));
    return Object.fromEntries(entries.map(([key, inner]) => [key, sortKeys(inner)]));
  }
  return value;
}

/** QGC's own layout: keys in byte order (as QJsonDocument writes them), four-space indent, trailing newline. */
export function qgcPlanJson(plan: FlightPlan): string {
  return `${JSON.stringify(sortKeys(toQgcPlan(plan)), null, 4)}\n`;
}

// ------------------------------------------------------------------- WPL

function wplNumber(value: number, digits: number): string {
  // NaN params are written as 0: older Mission Planner builds cannot parse "nan".
  return (Number.isFinite(value) ? value : 0).toFixed(digits);
}

/**
 * `QGC WPL 110`: one tab-separated line per item,
 * seq, current, frame, command, p1, p2, p3, p4, lat, lon, alt, autocontinue.
 * Line 0 is home, AMSL; QGC and ArduPilot both read it as the planned home.
 */
export function toWpl(plan: FlightPlan): string {
  const lines = ['QGC WPL 110'];
  const row = (seq: number, current: number, item: Pick<FlightItem, 'frame' | 'command' | 'params' | 'latDeg' | 'lonDeg' | 'altM' | 'autoContinue'>) =>
    [
      seq,
      current,
      item.frame,
      item.command,
      ...item.params.map((p) => wplNumber(p, 6)),
      wplNumber(item.latDeg, 7),
      wplNumber(item.lonDeg, 7),
      wplNumber(item.altM, 2),
      item.autoContinue ? 1 : 0,
    ].join('\t');
  lines.push(
    row(0, 1, {
      frame: MAV_FRAME.GLOBAL,
      command: MAV_CMD.NAV_WAYPOINT,
      params: [0, 0, 0, 0],
      latDeg: plan.home.latDeg,
      lonDeg: plan.home.lonDeg,
      altM: plan.home.altM,
      autoContinue: true,
    }),
  );
  plan.items.forEach((item, index) => lines.push(row(index + 1, 0, item)));
  return `${lines.join('\n')}\n`;
}

// --------------------------------------------------------------- MAVLink

/** MISSION_ITEM_INT fields. `null` params mean NaN (JSON has no NaN). */
export interface MavlinkMissionItemInt {
  seq: number;
  frame: number;
  command: number;
  current: 0 | 1;
  autocontinue: 0 | 1;
  param1: number | null;
  param2: number | null;
  param3: number | null;
  param4: number | null;
  /** Latitude in 1e-7 degrees. */
  x: number;
  /** Longitude in 1e-7 degrees. */
  y: number;
  z: number;
  mission_type: number;
}

export interface MavlinkExport {
  format: 'frontier-fly.mavlink-mission';
  version: 1;
  autopilot: number;
  /** ArduPilot keeps home at seq 0 of the mission list; PX4 does not. */
  homeAtSeq0: boolean;
  home: { x: number; y: number; z: number };
  mission: MavlinkMissionItemInt[];
  fence: MavlinkMissionItemInt[];
  rally: MavlinkMissionItemInt[];
}

export function degE7(deg: number): number {
  return Math.round(deg * 1e7);
}

function mavItem(
  seq: number,
  missionType: number,
  item: Pick<FlightItem, 'frame' | 'command' | 'params' | 'latDeg' | 'lonDeg' | 'altM' | 'autoContinue'>,
): MavlinkMissionItemInt {
  const global = item.frame !== MAV_FRAME.MISSION;
  return {
    seq,
    frame: item.frame,
    command: item.command,
    current: 0,
    autocontinue: item.autoContinue ? 1 : 0,
    param1: nullable(item.params[0]),
    param2: nullable(item.params[1]),
    param3: nullable(item.params[2]),
    param4: nullable(item.params[3]),
    x: global ? degE7(item.latDeg) : 0,
    y: global ? degE7(item.lonDeg) : 0,
    z: global ? item.altM : 0,
    mission_type: missionType,
  };
}

export function toMavlink(plan: FlightPlan): MavlinkExport {
  const ardu = plan.firmware === 'ardupilot';
  const mission: MavlinkMissionItemInt[] = [];
  if (ardu) {
    mission.push(
      mavItem(0, MAV_MISSION_TYPE.MISSION, {
        frame: MAV_FRAME.GLOBAL,
        command: MAV_CMD.NAV_WAYPOINT,
        params: [0, 0, 0, 0],
        latDeg: plan.home.latDeg,
        lonDeg: plan.home.lonDeg,
        altM: plan.home.altM,
        autoContinue: true,
      }),
    );
  }
  for (const item of plan.items) mission.push(mavItem(mission.length, MAV_MISSION_TYPE.MISSION, item));
  if (mission[0]) mission[0].current = 1;

  const fence: MavlinkMissionItemInt[] = [];
  for (const polygon of plan.fence) {
    const command = polygon.inclusion ? MAV_CMD.NAV_FENCE_POLYGON_VERTEX_INCLUSION : MAV_CMD.NAV_FENCE_POLYGON_VERTEX_EXCLUSION;
    for (const v of polygon.vertices) {
      fence.push(
        mavItem(fence.length, MAV_MISSION_TYPE.FENCE, {
          frame: MAV_FRAME.GLOBAL,
          command,
          // param1: vertex count of this polygon; param2: inclusion group.
          params: [polygon.vertices.length, 0, 0, 0],
          latDeg: v.latDeg,
          lonDeg: v.lonDeg,
          altM: 0,
          autoContinue: true,
        }),
      );
    }
  }
  const rally = plan.rally.map((p, seq) =>
    mavItem(seq, MAV_MISSION_TYPE.RALLY, {
      frame: MAV_FRAME.GLOBAL_RELATIVE_ALT,
      command: MAV_CMD.NAV_RALLY_POINT,
      params: [0, 0, 0, 0],
      latDeg: p.latDeg,
      lonDeg: p.lonDeg,
      altM: p.altM,
      autoContinue: true,
    }),
  );
  return {
    format: 'frontier-fly.mavlink-mission',
    version: 1,
    autopilot: ardu ? MAV_AUTOPILOT.ARDUPILOTMEGA : MAV_AUTOPILOT.PX4,
    homeAtSeq0: ardu,
    home: { x: degE7(plan.home.latDeg), y: degE7(plan.home.lonDeg), z: plan.home.altM },
    mission,
    fence,
    rally,
  };
}

export function mavlinkJson(plan: FlightPlan): string {
  return `${JSON.stringify(toMavlink(plan), null, 2)}\n`;
}

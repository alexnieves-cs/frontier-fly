import { roundTo } from '../polygon';
import type { Mission, MissionIssue, NoFlyZone, Point } from '../types';
import { validateMission } from '../validate';
import { haversineM, localFrame, type GeoPoint } from './geo';

/**
 * Read the geofence and no-fly zones back from a QGroundControl `.plan`.
 *
 * The structural checks mirror what QGC's own loaders require, so a file QGC
 * would refuse is refused here too, with the same reason. The plan's home
 * lands on the mission's current start pose, so an export re-imports onto the
 * same local coordinates.
 */

export interface PlanImport {
  ok: boolean;
  /** The mission with the imported geometry, already validated. Null when the file was refused. */
  mission: Mission | null;
  home: GeoPoint | null;
  issues: MissionIssue[];
  /** What was read, for the UI. */
  counts: { inclusion: number; exclusion: number; circles: number; missionItems: number };
}

/** Circles become polygons with this many corners. */
const CIRCLE_SIDES = 16;
/** A plan this far from home cannot be a mission over the sim site. */
const MAX_EXTENT_M = 20_000;

type Json = Record<string, unknown>;

function isObject(value: unknown): value is Json {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function isNum(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value);
}

/** QGC accepts a coordinate as an array of doubles (null reads as NaN). */
function coordinate(value: unknown, withAlt: boolean): number[] | null {
  if (!Array.isArray(value) || value.length !== (withAlt ? 3 : 2)) return null;
  if (!value.every((v) => v === null || isNum(v))) return null;
  return value.map((v) => (v === null ? NaN : (v as number)));
}

/** Structural issues QGroundControl itself would report on load. Empty means QGC can open the file. */
export function planStructureIssues(data: unknown): string[] {
  const issues: string[] = [];
  if (!isObject(data)) return ['The file is not a JSON object.'];
  if (typeof data.groundStation !== 'string') issues.push('groundStation must be a string.');
  if (data.fileType !== 'Plan') issues.push('fileType must be "Plan".');
  if (data.version !== 1) issues.push('Only plan file version 1 is supported.');
  for (const key of ['mission', 'geoFence', 'rallyPoints']) {
    if (!isObject(data[key])) issues.push(`${key} must be an object.`);
  }
  if (issues.length) return issues;

  const mission = data.mission as Json;
  if (mission.version !== 2) issues.push('mission.version must be 2.');
  if (!isNum(mission.firmwareType)) issues.push('mission.firmwareType must be a number.');
  for (const key of ['vehicleType', 'cruiseSpeed', 'hoverSpeed', 'globalPlanAltitudeMode']) {
    if (key in mission && !isNum(mission[key])) issues.push(`mission.${key} must be a number.`);
  }
  if (!coordinate(mission.plannedHomePosition, true)) issues.push('mission.plannedHomePosition must be [lat, lon, alt].');
  if (!Array.isArray(mission.items)) {
    issues.push('mission.items must be an array.');
  } else {
    mission.items.forEach((item, index) => {
      const at = `mission.items[${index}]`;
      if (!isObject(item) || typeof item.type !== 'string') {
        issues.push(`${at} needs a type.`);
        return;
      }
      if (item.type === 'ComplexItem') {
        if (typeof item.complexItemType !== 'string') issues.push(`${at} needs a complexItemType.`);
        return;
      }
      if (item.type !== 'SimpleItem') {
        issues.push(`${at} has an unknown type.`);
        return;
      }
      if (!isNum(item.frame) || !isNum(item.command) || typeof item.autoContinue !== 'boolean') {
        issues.push(`${at} needs frame, command and autoContinue.`);
      }
      // Older files keep param1..param4 as keys, and the position in `coordinate`; QGC converts both on load.
      const legacyKeys = !('params' in item);
      if (legacyKeys && !['param1', 'param2', 'param3', 'param4'].every((key) => isNum(item[key]))) {
        issues.push(`${at} needs params, or param1 to param4.`);
      }
      const hasCoordinate = 'coordinate' in item;
      if (hasCoordinate && !coordinate(item.coordinate, true)) issues.push(`${at}.coordinate must be [lat, lon, alt].`);
      if (legacyKeys && !hasCoordinate) issues.push(`${at} needs a coordinate.`);
      if (!legacyKeys) {
        const want = hasCoordinate ? 4 : 7;
        if (!Array.isArray(item.params) || item.params.length !== want) {
          issues.push(`${at}.params must hold ${want} values.`);
        } else if (!item.params.every((p) => p === null || isNum(p))) {
          issues.push(`${at}.params must be numbers or null.`);
        }
      }
      const alt = ['AltitudeMode', 'Altitude', 'AMSLAltAboveTerrain'].filter((key) => key in item);
      if (alt.length && alt.length !== 3) issues.push(`${at} must have all of AltitudeMode, Altitude and AMSLAltAboveTerrain, or none.`);
      if ('Altitude' in item && !isNum(item.Altitude)) issues.push(`${at}.Altitude must be a number.`);
      if ('AltitudeMode' in item && !isNum(item.AltitudeMode)) issues.push(`${at}.AltitudeMode must be a number.`);
    });
  }

  const fence = data.geoFence as Json;
  // QGC silently skips a version-1 (or unversioned) fence.
  if (fence.version !== undefined && fence.version !== 1) {
    if (fence.version !== 2) issues.push('geoFence.version must be 2.');
    if (!Array.isArray(fence.polygons) || !Array.isArray(fence.circles)) {
      issues.push('geoFence needs polygons and circles arrays.');
    } else {
      fence.polygons.forEach((polygon, index) => {
        const at = `geoFence.polygons[${index}]`;
        if (!isObject(polygon) || polygon.version !== 1 || typeof polygon.inclusion !== 'boolean') {
          issues.push(`${at} needs version 1 and an inclusion flag.`);
          return;
        }
        if (!Array.isArray(polygon.polygon) || !polygon.polygon.every((v) => coordinate(v, false))) {
          issues.push(`${at}.polygon must be a list of [lat, lon].`);
        }
      });
      fence.circles.forEach((circle, index) => {
        const at = `geoFence.circles[${index}]`;
        if (!isObject(circle) || circle.version !== 1 || typeof circle.inclusion !== 'boolean' || !isObject(circle.circle)) {
          issues.push(`${at} needs version 1, an inclusion flag and a circle.`);
          return;
        }
        if (!coordinate(circle.circle.center, false) || !isNum(circle.circle.radius)) {
          issues.push(`${at}.circle needs a [lat, lon] center and a radius.`);
        }
      });
    }
  }

  const rally = data.rallyPoints as Json;
  if (rally.version !== undefined && rally.version !== 1) {
    if (rally.version !== 2) issues.push('rallyPoints.version must be 2.');
    if (!Array.isArray(rally.points) || !rally.points.every((p) => coordinate(p, true))) {
      issues.push('rallyPoints.points must be a list of [lat, lon, alt].');
    }
  }
  return issues;
}

function err(message: string, path = 'plan'): MissionIssue {
  return { path, message, severity: 'error' };
}

function circleRing(center: Point, radius: number): Point[] {
  const ring: Point[] = [];
  for (let k = 0; k < CIRCLE_SIDES; k++) {
    const a = (2 * Math.PI * k) / CIRCLE_SIDES;
    ring.push({ x: center.x + radius * Math.cos(a), y: center.y + radius * Math.sin(a) });
  }
  return ring;
}

/** Drop the closing vertex some tools repeat at the end of a ring. */
function openRing(points: Point[]): Point[] {
  const first = points[0];
  const last = points[points.length - 1];
  if (points.length > 3 && first && last && Math.hypot(first.x - last.x, first.y - last.y) < 0.05) return points.slice(0, -1);
  return points;
}

export function importQgcPlan(text: string, base: Mission): PlanImport {
  const counts = { inclusion: 0, exclusion: 0, circles: 0, missionItems: 0 };
  const refuse = (issues: MissionIssue[]): PlanImport => ({ ok: false, mission: null, home: null, issues, counts });
  let data: unknown;
  try {
    data = JSON.parse(text);
  } catch {
    return refuse([err('The file is not valid JSON.')]);
  }
  const structure = planStructureIssues(data);
  if (structure.length) return refuse(structure.map((message) => err(message)));

  const plan = data as Json;
  const missionJson = plan.mission as Json;
  const fenceJson = plan.geoFence as Json;
  const [lat, lon, alt] = coordinate(missionJson.plannedHomePosition, true)!;
  if (!isNum(lat) || !isNum(lon) || Math.abs(lat) > 85 || Math.abs(lon) > 180) {
    return refuse([err('The planned home position is not a valid latitude and longitude.')]);
  }
  const home: GeoPoint = { latDeg: lat, lonDeg: lon, altM: isNum(alt) ? alt : 0 };
  const frame = localFrame(home);
  const toLocal = ([la, lo]: number[]): Point => {
    const local = frame.toLocal({ latDeg: la!, lonDeg: lo!, altM: home.altM });
    return { x: roundTo(base.startPose.x + local.east, 3), y: roundTo(base.startPose.y + local.north, 3) };
  };
  const far = (latDeg: number, lonDeg: number): boolean => haversineM(home, { latDeg, lonDeg, altM: 0 }) > MAX_EXTENT_M;
  counts.missionItems = Array.isArray(missionJson.items) ? missionJson.items.length : 0;

  const inclusion: Point[][] = [];
  const exclusion: Point[][] = [];
  const issues: MissionIssue[] = [];
  const polygons = Array.isArray(fenceJson.polygons) ? (fenceJson.polygons as Json[]) : [];
  const circles = Array.isArray(fenceJson.circles) ? (fenceJson.circles as Json[]) : [];
  for (const polygon of polygons) {
    const vertices = (polygon.polygon as number[][]).map((v) => coordinate(v, false)!);
    if (vertices.some(([la, lo]) => !isNum(la) || !isNum(lo) || far(la, lo))) {
      return refuse([err(`A fence polygon lies more than ${MAX_EXTENT_M / 1000} km from home or has an empty vertex.`)]);
    }
    const ring = openRing(vertices.map(toLocal));
    if (polygon.inclusion) inclusion.push(ring);
    else exclusion.push(ring);
  }
  for (const circle of circles) {
    const body = circle.circle as Json;
    const [la, lo] = coordinate(body.center, false)!;
    const radius = body.radius as number;
    if (!isNum(la) || !isNum(lo) || far(la, lo) || !(radius > 0)) return refuse([err('A fence circle has an invalid center or radius.')]);
    counts.circles += 1;
    const ring = circleRing(toLocal([la, lo]), radius);
    if (circle.inclusion) inclusion.push(ring);
    else exclusion.push(ring);
  }
  counts.inclusion = inclusion.length;
  counts.exclusion = exclusion.length;
  if (inclusion.length === 0) return refuse([err('The plan has no inclusion fence, so there is no search area to import.')]);
  if (inclusion.length > 1) {
    issues.push({ path: 'geofence', message: `The plan has ${inclusion.length} inclusion fences; only the first is used.`, severity: 'warning' });
  }

  const noFlyZones: NoFlyZone[] = exclusion.map((points, index) => ({
    id: `nf-import-${index + 1}`,
    name: `Imported no-fly ${index + 1}`,
    polygon: { points },
    reason: 'Exclusion fence from a QGroundControl plan.',
  }));
  const next: Mission = { ...base, geofence: { points: inclusion[0]! }, noFlyZones };
  const validated = validateMission(next);
  return {
    ok: validated.ok,
    mission: validated.ok ? validated.mission : null,
    home,
    issues: [...issues, ...validated.issues],
    counts,
  };
}

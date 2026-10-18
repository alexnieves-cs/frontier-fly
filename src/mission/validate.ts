import { hasDuplicateVertex, isSelfIntersecting, pointInRing, ringInsideRing, ringsOverlap, signedArea } from './polygon';
import { missionSchema, validateJsonSchema, type SchemaIssue } from './schemaValidate';
import {
  MISSION_SCHEMA_VERSION,
  MISSION_TYPE_LABELS,
  type Mission,
  type MissionIssue,
  type MissionType,
  type MissionValidation,
  type Point,
  type TargetKind,
} from './types';

const TYPE_NEEDS: Record<Exclude<MissionType, 'composite'>, TargetKind> = {
  find_objects: 'object',
  find_person: 'person',
  safety_sweep: 'clearance',
  building_inspection: 'structure',
};

function isRecord(data: unknown): data is Record<string, unknown> {
  return typeof data === 'object' && data !== null && !Array.isArray(data);
}

function readPoints(value: unknown): Point[] | null {
  if (!isRecord(value) || !Array.isArray(value.points)) return null;
  const points: Point[] = [];
  for (const item of value.points) {
    if (!isRecord(item) || typeof item.x !== 'number' || typeof item.y !== 'number') return null;
    if (!Number.isFinite(item.x) || !Number.isFinite(item.y)) return null;
    points.push({ x: item.x, y: item.y });
  }
  return points;
}

function ringIssues(path: string, label: string, points: Point[]): MissionIssue[] {
  const issues: MissionIssue[] = [];
  if (points.length < 3) {
    issues.push({ path, message: `${label} needs at least 3 corners.`, severity: 'error' });
    return issues;
  }
  if (hasDuplicateVertex(points)) {
    issues.push({ path, message: `${label} has corners stacked on each other.`, severity: 'error' });
  }
  if (isSelfIntersecting(points)) {
    issues.push({ path, message: `${label} crosses itself.`, severity: 'error' });
  }
  if (Math.abs(signedArea(points)) < 0.5) {
    issues.push({ path, message: `${label} area is too small.`, severity: 'error' });
  }
  return issues;
}

function ringIsSimple(points: Point[]): boolean {
  return points.length >= 3 && !hasDuplicateVertex(points) && !isSelfIntersecting(points) && Math.abs(signedArea(points)) >= 0.5;
}

function schemaToIssue(issue: SchemaIssue): MissionIssue {
  const path = issue.path || 'mission';
  return { path, message: issue.message, severity: 'error' };
}

function skipSchemaIssue(issue: SchemaIssue): boolean {
  if (issue.keyword !== 'minItems') return false;
  return issue.path === 'geofence.points' || /noFlyZones\[\d+\]\.polygon\.points$/.test(issue.path);
}

/**
 * Structural check against the v1 JSON Schema, then geometric and cross-field checks
 * the schema cannot express.
 */
export function validateMission(input: unknown): MissionValidation {
  const schemaIssues = validateJsonSchema(input, missionSchema).filter((issue) => !skipSchemaIssue(issue));
  const issues: MissionIssue[] = schemaIssues.map(schemaToIssue);

  if (!isRecord(input)) {
    return { ok: false, mission: null, issues };
  }

  const geofence = readPoints(input.geofence);
  if (geofence) issues.push(...ringIssues('geofence', 'Geofence', geofence));
  const geofenceOk = geofence != null && ringIsSimple(geofence);

  const noFlyRings: { name: string; points: Point[]; ok: boolean }[] = [];
  if (Array.isArray(input.noFlyZones)) {
    input.noFlyZones.forEach((zone, index) => {
      const name = isRecord(zone) && typeof zone.name === 'string' && zone.name.trim() ? zone.name : `No-fly ${index + 1}`;
      const points = isRecord(zone) ? readPoints(zone.polygon) : null;
      if (!points) return;
      const path = `noFlyZones[${index}]`;
      issues.push(...ringIssues(path, name, points));
      const ok = ringIsSimple(points);
      noFlyRings.push({ name, points, ok });
      if (ok && geofenceOk && geofence && !ringInsideRing(points, geofence)) {
        issues.push({ path, message: `${name} must sit inside the geofence.`, severity: 'error' });
      }
    });
  }

  for (let i = 0; i < noFlyRings.length; i++) {
    for (let j = i + 1; j < noFlyRings.length; j++) {
      const a = noFlyRings[i]!;
      const b = noFlyRings[j]!;
      if (a.ok && b.ok && ringsOverlap(a.points, b.points)) {
        issues.push({
          path: `noFlyZones[${j}]`,
          message: `${a.name} and ${b.name} overlap.`,
          severity: 'error',
        });
      }
    }
  }

  const start = isRecord(input.startPose) ? input.startPose : null;
  const startX = start && typeof start.x === 'number' ? start.x : null;
  const startY = start && typeof start.y === 'number' ? start.y : null;
  if (geofenceOk && geofence && startX != null && startY != null && Number.isFinite(startX) && Number.isFinite(startY)) {
    if (!pointInRing({ x: startX, y: startY }, geofence)) {
      issues.push({ path: 'startPose', message: 'Start is outside the geofence.', severity: 'error' });
    }
    for (const zone of noFlyRings) {
      if (zone.ok && pointInRing({ x: startX, y: startY }, zone.points)) {
        issues.push({ path: 'startPose', message: `Start is inside ${zone.name}.`, severity: 'error' });
      }
    }
  }

  const altitude = isRecord(input.altitude) ? input.altitude : null;
  const minM = altitude && typeof altitude.minM === 'number' ? altitude.minM : null;
  const maxM = altitude && typeof altitude.maxM === 'number' ? altitude.maxM : null;
  if (minM != null && maxM != null && !(minM < maxM)) {
    issues.push({ path: 'altitude', message: 'Minimum altitude must be below the maximum.', severity: 'error' });
  }
  const startZ = start && typeof start.z === 'number' ? start.z : null;
  if (minM != null && maxM != null && minM < maxM && startZ != null && (startZ < minM || startZ > maxM)) {
    issues.push({ path: 'startPose.z', message: 'Start altitude is outside the altitude band.', severity: 'error' });
  }

  const missionType = typeof input.missionType === 'string' ? input.missionType : '';
  if (Array.isArray(input.targets) && missionType in MISSION_TYPE_LABELS) {
    const kinds = input.targets
      .filter(isRecord)
      .map((target) => target.kind)
      .filter((kind): kind is TargetKind => typeof kind === 'string');
    if (missionType === 'composite') {
      if (new Set(kinds).size < 2) {
        issues.push({
          path: 'targets',
          message: 'A composite mission needs at least two kinds of target.',
          severity: 'error',
        });
      }
    } else {
      const need = TYPE_NEEDS[missionType as Exclude<MissionType, 'composite'>];
      if (!kinds.includes(need)) {
        issues.push({
          path: 'targets',
          message: `${MISSION_TYPE_LABELS[missionType as MissionType]} needs a ${need.replace('_', ' ')} target.`,
          severity: 'error',
        });
      }
    }
  }

  const ids: string[] = [];
  if (Array.isArray(input.noFlyZones)) {
    for (const zone of input.noFlyZones) {
      if (isRecord(zone) && typeof zone.id === 'string') ids.push(zone.id);
    }
  }
  if (Array.isArray(input.targets)) {
    for (const target of input.targets) {
      if (isRecord(target) && typeof target.id === 'string') ids.push(target.id);
    }
  }
  if (new Set(ids).size !== ids.length) {
    issues.push({ path: 'id', message: 'No-fly zones and targets need distinct ids.', severity: 'error' });
  }

  const confidence = typeof input.requiredConfidence === 'number' ? input.requiredConfidence : null;
  if (confidence != null && confidence < 0.5) {
    issues.push({
      path: 'requiredConfidence',
      message: 'Confidence below 50% will count weak contacts.',
      severity: 'warning',
    });
  }

  const budgets = isRecord(input.budgets) ? input.budgets : null;
  const timeSec = budgets && typeof budgets.timeSec === 'number' ? budgets.timeSec : null;
  if (geofenceOk && geofence && timeSec != null && timeSec >= 30) {
    const area = Math.abs(signedArea(geofence));
    const capacity = timeSec * 3 * 8;
    if (area > capacity) {
      issues.push({
        path: 'budgets.timeSec',
        message: 'Time budget is short for this geofence at a 3 m/s sweep with an 8 m swath.',
        severity: 'warning',
      });
    }
    const noFlyArea = noFlyRings.filter((zone) => zone.ok).reduce((sum, zone) => sum + Math.abs(signedArea(zone.points)), 0);
    if (area > 0 && noFlyArea / area > 0.45) {
      issues.push({
        path: 'noFlyZones',
        message: 'No-fly zones cover most of the geofence.',
        severity: 'warning',
      });
    }
  }

  const errors = issues.filter((issue) => issue.severity === 'error');
  if (errors.length > 0 || input.schemaVersion !== MISSION_SCHEMA_VERSION) {
    return { ok: false, mission: null, issues };
  }
  return { ok: true, mission: input as unknown as Mission, issues };
}

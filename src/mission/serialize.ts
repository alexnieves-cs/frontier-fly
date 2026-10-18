import { roundTo } from './polygon';
import type { Mission, MissionValidation, Point } from './types';
import { validateMission } from './validate';

const HASH_LIMIT = 20_000;

function roundPoint(point: Point): Point {
  return { x: roundTo(point.x, 3), y: roundTo(point.y, 3) };
}

function wrapPi(yaw: number): number {
  const tau = Math.PI * 2;
  let value = yaw % tau;
  if (value > Math.PI) value -= tau;
  if (value < -Math.PI) value += tau;
  return Object.is(value, -0) ? 0 : value;
}

/** Stable key order and rounded numbers. The same mission always produces the same text. */
export function canonicalMission(mission: Mission): Mission {
  return {
    schemaVersion: 1,
    id: mission.id,
    name: mission.name,
    seed: mission.seed >>> 0,
    notes: mission.notes,
    geofence: { points: mission.geofence.points.map(roundPoint) },
    noFlyZones: mission.noFlyZones.map((zone) => ({
      id: zone.id,
      name: zone.name,
      polygon: { points: zone.polygon.points.map(roundPoint) },
      reason: zone.reason,
    })),
    altitude: {
      minM: roundTo(mission.altitude.minM, 2),
      maxM: roundTo(mission.altitude.maxM, 2),
    },
    missionType: mission.missionType,
    targets: mission.targets.map((target) => ({
      id: target.id,
      kind: target.kind,
      label: target.label,
      minCount: Math.round(target.minCount),
      minConfidence: target.minConfidence == null ? null : roundTo(target.minConfidence, 3),
    })),
    requiredConfidence: roundTo(mission.requiredConfidence, 3),
    entryPolicy: mission.entryPolicy,
    budgets: {
      batteryPct: roundTo(mission.budgets.batteryPct, 1),
      timeSec: Math.round(mission.budgets.timeSec),
    },
    startPose: {
      x: roundTo(mission.startPose.x, 3),
      y: roundTo(mission.startPose.y, 3),
      z: roundTo(mission.startPose.z, 2),
      yawRad: roundTo(wrapPi(mission.startPose.yawRad), 5),
    },
  };
}

export function canonicalMissionJson(mission: Mission): string {
  return JSON.stringify(canonicalMission(mission));
}

export function toBase64Url(text: string): string {
  const bytes = new TextEncoder().encode(text);
  let binary = '';
  for (const byte of bytes) binary += String.fromCharCode(byte);
  return btoa(binary).replaceAll('+', '-').replaceAll('/', '_').replace(/=+$/u, '');
}

export function fromBase64Url(value: string): string {
  const pad = value.length % 4 === 0 ? '' : '='.repeat(4 - (value.length % 4));
  const b64 = value.replaceAll('-', '+').replaceAll('_', '/') + pad;
  const binary = atob(b64);
  const bytes = Uint8Array.from(binary, (char) => char.charCodeAt(0));
  return new TextDecoder().decode(bytes);
}

/** Hash fragment without the leading `#`, shaped `m=<base64url>`. */
export function missionToHash(mission: Mission): string {
  return `m=${toBase64Url(canonicalMissionJson(mission))}`;
}

export function decodeMissionHash(hash: string): MissionValidation {
  const trimmed = hash.startsWith('#') ? hash.slice(1) : hash;
  const payload = trimmed.startsWith('m=') ? trimmed.slice(2) : '';
  if (!payload) {
    return { ok: false, mission: null, issues: [{ path: 'url', message: 'The link has no mission payload.', severity: 'error' }] };
  }
  if (payload.length > HASH_LIMIT) {
    return { ok: false, mission: null, issues: [{ path: 'url', message: 'The mission link is too long.', severity: 'error' }] };
  }
  let json = '';
  try {
    json = fromBase64Url(payload);
  } catch {
    return { ok: false, mission: null, issues: [{ path: 'url', message: 'The mission link is not valid base64.', severity: 'error' }] };
  }
  let data: unknown;
  try {
    data = JSON.parse(json);
  } catch {
    return { ok: false, mission: null, issues: [{ path: 'url', message: 'The mission link is not valid JSON.', severity: 'error' }] };
  }
  const validated = validateMission(data);
  if (validated.ok && validated.mission) {
    return { ...validated, mission: canonicalMission(validated.mission) };
  }
  return validated;
}

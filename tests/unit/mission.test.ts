import { readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { pathIsOperable, planPath } from '../../src/mission/path';
import { isSelfIntersecting, ringInsideRing, snapToGrid } from '../../src/mission/polygon';
import { blankMission, sampleMission } from '../../src/mission/sample';
import { canonicalMission, decodeMissionHash, missionToHash } from '../../src/mission/serialize';
import { validateMission } from '../../src/mission/validate';
import type { Mission } from '../../src/mission/types';

function square(x: number, y: number, w: number, h: number): Mission['geofence'] {
  return {
    points: [
      { x, y },
      { x: x + w, y },
      { x: x + w, y: y + h },
      { x, y: y + h },
    ],
  };
}

function withGeofence(mission: Mission, points: Mission['geofence']['points'], noFly: Mission['noFlyZones'] = []): Mission {
  return {
    ...mission,
    geofence: { points },
    noFlyZones: noFly,
    startPose: { ...mission.startPose, x: points[0] ? points[0].x + 1 : 0, y: points[0] ? points[0].y + 1 : 0 },
  };
}

describe('mission schema and geometry', () => {
  it('accepts the seeded sample and rejects an unknown field', () => {
    const mission = sampleMission(11);
    const validated = validateMission(mission);
    expect(validated.ok, validated.issues.map((issue) => issue.message).join('; ')).toBe(true);
    const extra = { ...mission, pilot: 'keys' };
    const rejected = validateMission(extra);
    expect(rejected.ok).toBe(false);
    expect(rejected.issues.some((issue) => issue.path === 'pilot')).toBe(true);
  });

  it('rejects a bad schema version, a wrong enum, and a non-integer seed', () => {
    const mission = sampleMission(3);
    expect(validateMission({ ...mission, schemaVersion: 2 }).issues.some((issue) => issue.path === 'schemaVersion')).toBe(true);
    expect(validateMission({ ...mission, entryPolicy: 'maybe' }).ok).toBe(false);
    expect(validateMission({ ...mission, seed: 1.5 }).ok).toBe(false);
  });

  it('flags a self-intersecting geofence', () => {
    const bowtie = [
      { x: 0, y: 0 },
      { x: 10, y: 10 },
      { x: 0, y: 10 },
      { x: 10, y: 0 },
    ];
    expect(isSelfIntersecting(bowtie)).toBe(true);
    expect(isSelfIntersecting(square(0, 0, 10, 10).points)).toBe(false);
    const mission = withGeofence(sampleMission(1), bowtie);
    const validated = validateMission(mission);
    expect(validated.ok).toBe(false);
    expect(validated.issues.some((issue) => issue.path === 'geofence' && /crosses itself/.test(issue.message))).toBe(true);
  });

  it('requires no-fly zones to sit inside the geofence and not on the start', () => {
    const outer = square(0, 0, 100, 100).points;
    const inside = square(10, 10, 10, 10).points;
    const outside = square(110, 10, 10, 10).points;
    const crossing = square(90, 10, 20, 10).points;
    expect(ringInsideRing(inside, outer)).toBe(true);
    expect(ringInsideRing(outside, outer)).toBe(false);
    expect(ringInsideRing(crossing, outer)).toBe(false);

    const base = sampleMission(4);
    const zone = { ...base.noFlyZones[0]!, polygon: { points: outside } };
    const outsideMission = withGeofence(base, outer, [zone]);
    expect(validateMission(outsideMission).issues.some((issue) => /inside the geofence/.test(issue.message))).toBe(true);

    const blocked = withGeofence(base, outer, [{ ...zone, polygon: { points: inside } }]);
    blocked.startPose = { ...blocked.startPose, x: 15, y: 15 };
    expect(validateMission(blocked).issues.some((issue) => /inside/.test(issue.message) && issue.path === 'startPose')).toBe(true);

    const overlapping = withGeofence(base, outer, [
      { ...zone, id: 'nf-a', name: 'A', polygon: { points: inside } },
      { ...zone, id: 'nf-b', name: 'B', polygon: { points: square(15, 15, 10, 10).points } },
    ]);
    overlapping.startPose = { ...overlapping.startPose, x: 2, y: 2 };
    expect(validateMission(overlapping).issues.some((issue) => /overlap/.test(issue.message))).toBe(true);
  });

  it('checks altitude order, start altitude, and target kind', () => {
    const mission = sampleMission(8);
    expect(validateMission({ ...mission, altitude: { minM: 20, maxM: 10 } }).issues.some((issue) => issue.path === 'altitude')).toBe(true);
    expect(
      validateMission({ ...mission, startPose: { ...mission.startPose, z: 40 } }).issues.some((issue) => issue.path === 'startPose.z'),
    ).toBe(true);
    const sweep = {
      ...mission,
      missionType: 'safety_sweep' as const,
      targets: mission.targets,
    };
    expect(validateMission(sweep).issues.some((issue) => /clearance/.test(issue.message))).toBe(true);
  });

  it('snaps to the grid', () => {
    expect(snapToGrid({ x: 1.4, y: 2.6 }, 1)).toEqual({ x: 1, y: 3 });
  });
});

describe('mission URL round-trip', () => {
  it('encodes a canonical hash and reads it back', () => {
    const mission = canonicalMission(sampleMission(11));
    const hash = missionToHash(mission);
    const decoded = decodeMissionHash(hash);
    expect(decoded.ok).toBe(true);
    expect(decoded.mission).toEqual(mission);
    expect(missionToHash(decoded.mission!)).toBe(hash);
  });

  it('round-trips a name with spaces and keeps the seed', () => {
    const mission = canonicalMission({ ...sampleMission(42), name: 'Lot sweep, west gate' });
    const decoded = decodeMissionHash(`#${missionToHash(mission)}`);
    expect(decoded.mission?.name).toBe('Lot sweep, west gate');
    expect(decoded.mission?.seed).toBe(42);
    expect(missionToHash(decoded.mission!)).toBe(missionToHash(mission));
  });

  it('rejects a tampered payload and a bowtie hidden in a hash', () => {
    expect(decodeMissionHash('#m=not-valid').ok).toBe(false);
    const bowtie = canonicalMission(
      withGeofence(sampleMission(9), [
        { x: 0, y: 0 },
        { x: 10, y: 10 },
        { x: 0, y: 10 },
        { x: 10, y: 0 },
      ]),
    );
    const decoded = decodeMissionHash(missionToHash(bowtie));
    expect(decoded.ok).toBe(false);
    expect(decoded.issues.some((issue) => /crosses itself/.test(issue.message))).toBe(true);
  });

  it('builds the same sample for a seed', () => {
    expect(canonicalMission(sampleMission(11))).toEqual(canonicalMission(sampleMission(11)));
    expect(sampleMission(11).name).not.toBe(sampleMission(12).name);
    expect(blankMission(11).geofence.points).toEqual([]);
  });
});

describe('mission sweep preview', () => {
  it('keeps the sample sweep inside the operating area', () => {
    const mission = sampleMission(11);
    const path = planPath(mission);
    expect(path.length).toBeGreaterThan(4);
    expect(pathIsOperable(mission, path)).toBe(true);
  });
});

describe('mission core boundary', () => {
  it('does not import the renderer or touch the DOM', () => {
    const dir = join(process.cwd(), 'src/mission');
    const files = readdirSync(dir).filter((name) => name.endsWith('.ts'));
    expect(files.length).toBeGreaterThan(5);
    for (const name of files) {
      const text = readFileSync(join(dir, name), 'utf8');
      expect(text, name).not.toMatch(/from ['"]three['"]/);
      expect(text, name).not.toMatch(/\bdocument\./);
      expect(text, name).not.toMatch(/\bwindow\./);
    }
  });
});

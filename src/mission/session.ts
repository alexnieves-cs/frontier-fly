import type { Detection, DetectionSource } from './detections';
import { closestAlong, planPath, pointAtDistance } from './path';
import { bounds, pointInRing } from './polygon';
import type { Mission, MissionType } from './types';
import type { WorldSnapshot } from './world';

/** Placeholder pack drain used until a real energy model is connected. */
export const PLACEHOLDER_BATTERY_DRAIN_PCT_PER_SEC = 0.15;

const SPEED_M_PER_S = 3;
const HOLD_SEC = 1.5;
const SENSE_M = 6;
const CELL_M = 2;

export type SessionStatus = 'ready' | 'running' | 'awaiting_approval' | 'complete' | 'aborted';
export type DetectionDisposition = 'pending' | 'logged' | 'entered' | 'denied' | 'skipped';

export interface ScheduledDetection extends Detection {
  alongM: number;
  disposition: DetectionDisposition;
  revealed: boolean;
}

export interface ApprovalRequest {
  detectionId: string;
  label: string;
  kind: Detection['kind'];
  confidence: number;
  structureId: string | null;
  structureLabel: string | null;
  x: number;
  y: number;
}

export interface LogEntry {
  tSec: number;
  tone: 'info' | 'warn' | 'action';
  message: string;
}

export interface CoverageGrid {
  originX: number;
  originY: number;
  cellM: number;
  cols: number;
  rows: number;
  visited: Uint8Array;
  operable: number;
  seen: number;
  ratio: number;
}

export interface TargetResult {
  id: string;
  label: string;
  found: number;
  minCount: number;
  met: boolean;
}

export interface MissionReport {
  missionName: string;
  missionType: MissionType;
  outcome: 'complete' | 'budget' | 'aborted';
  budgetCause: 'battery' | 'time' | null;
  elapsedSec: number;
  batteryUsedPct: number;
  coverageRatio: number;
  detectionCount: number;
  entriesRequested: number;
  entriesApproved: number;
  entriesDenied: number;
  targets: TargetResult[];
  notes: string[];
  placeholders: {
    pathEfficiency: null;
    sensorHealth: null;
    commsDropouts: null;
    falsePositiveRate: null;
  };
}

export interface SessionSnapshot {
  status: SessionStatus;
  tSec: number;
  durationSec: number;
  pose: { x: number; y: number; z: number; yawRad: number };
  plan: { x: number; y: number }[];
  track: { x: number; y: number }[];
  coverage: CoverageGrid;
  detections: ScheduledDetection[];
  log: LogEntry[];
  pendingApproval: ApprovalRequest | null;
  report: MissionReport | null;
  batteryUsedPct: number;
  batteryRemainingPct: number;
}

interface FlySegment {
  kind: 'fly';
  fromM: number;
  toM: number;
  seconds: number;
}

interface EventSegment {
  kind: 'event';
  index: number;
  seconds: number;
  fired: boolean;
}

type Segment = FlySegment | EventSegment;

function cloneDetections(list: Detection[], path: { x: number; y: number }[]): ScheduledDetection[] {
  return list.map((detection) => ({
    ...detection,
    alongM: closestAlong(path, detection).along,
    disposition: 'pending',
    revealed: false,
  }));
}

function buildCoverage(mission: Mission): CoverageGrid {
  const box = bounds(mission.geofence.points);
  const originX = Math.floor(box.minX / CELL_M) * CELL_M;
  const originY = Math.floor(box.minY / CELL_M) * CELL_M;
  const cols = Math.max(1, Math.ceil((box.maxX - originX) / CELL_M));
  const rows = Math.max(1, Math.ceil((box.maxY - originY) / CELL_M));
  const visited = new Uint8Array(cols * rows);
  let operable = 0;
  if (mission.geofence.points.length >= 3) {
    for (let row = 0; row < rows; row++) {
      for (let col = 0; col < cols; col++) {
        const point = { x: originX + (col + 0.5) * CELL_M, y: originY + (row + 0.5) * CELL_M };
        const inside = pointInRing(point, mission.geofence.points);
        const blocked = mission.noFlyZones.some((zone) => pointInRing(point, zone.polygon.points));
        if (inside && !blocked) {
          visited[row * cols + col] = 0;
          operable += 1;
        } else {
          visited[row * cols + col] = 2;
        }
      }
    }
  }
  return { originX, originY, cellM: CELL_M, cols, rows, visited, operable, seen: 0, ratio: 0 };
}

function buildSegments(detections: ScheduledDetection[], lengthM: number): Segment[] {
  const marks = detections
    .map((detection, index) => ({ index, alongM: Math.max(0, Math.min(lengthM, detection.alongM)) }))
    .sort((a, b) => a.alongM - b.alongM || a.index - b.index);
  const segments: Segment[] = [];
  let cursor = 0;
  for (const mark of marks) {
    if (mark.alongM > cursor + 0.05) {
      const meters = mark.alongM - cursor;
      segments.push({ kind: 'fly', fromM: cursor, toM: mark.alongM, seconds: meters / SPEED_M_PER_S });
    }
    segments.push({ kind: 'event', index: mark.index, seconds: HOLD_SEC, fired: false });
    cursor = mark.alongM;
  }
  if (lengthM > cursor + 0.05) {
    segments.push({ kind: 'fly', fromM: cursor, toM: lengthM, seconds: (lengthM - cursor) / SPEED_M_PER_S });
  }
  return segments;
}

function structureLabel(world: WorldSnapshot, id: string | null): string | null {
  if (!id) return null;
  return world.footprints.find((footprint) => footprint.id === id)?.label ?? null;
}

export interface MissionSession {
  start(): void;
  step(dt: number): void;
  approve(): void;
  deny(): void;
  abort(): void;
  snapshot(): SessionSnapshot;
}

export function createSession(mission: Mission, world: WorldSnapshot, source: DetectionSource): MissionSession {
  const plan = planPath(mission);
  const detections = cloneDetections(source.catalog(mission, world), plan);
  const lengthM = plan.reduce((sum, point, index) => {
    if (index === 0) return 0;
    const prev = plan[index - 1]!;
    return sum + Math.hypot(point.x - prev.x, point.y - prev.y);
  }, 0);
  const segments = buildSegments(detections, lengthM);
  const coverage = buildCoverage(mission);
  const log: LogEntry[] = [];
  let status: SessionStatus = 'ready';
  let tSec = 0;
  let segIndex = 0;
  let segT = 0;
  let pending: ApprovalRequest | null = null;
  let report: MissionReport | null = null;
  let entriesRequested = 0;
  let entriesApproved = 0;
  let entriesDenied = 0;
  const track: { x: number; y: number }[] = [{ x: mission.startPose.x, y: mission.startPose.y }];
  let pose = { x: mission.startPose.x, y: mission.startPose.y, z: mission.startPose.z, yawRad: mission.startPose.yawRad };

  const pushLog = (tone: LogEntry['tone'], message: string): void => {
    log.push({ tSec, tone, message });
  };

  const markCoverage = (): void => {
    const { originX, originY, cols, rows, cellM, visited } = coverage;
    for (let row = 0; row < rows; row++) {
      for (let col = 0; col < cols; col++) {
        const index = row * cols + col;
        if (visited[index] !== 0) continue;
        const x = originX + (col + 0.5) * cellM;
        const y = originY + (row + 0.5) * cellM;
        if (Math.hypot(x - pose.x, y - pose.y) <= SENSE_M) {
          visited[index] = 1;
          coverage.seen += 1;
        }
      }
    }
    coverage.ratio = coverage.operable === 0 ? 0 : coverage.seen / coverage.operable;
  };

  const rememberTrack = (): void => {
    const last = track[track.length - 1];
    if (!last || Math.hypot(last.x - pose.x, last.y - pose.y) >= 0.4) {
      track.push({ x: pose.x, y: pose.y });
    }
  };

  const batteryUsed = (): number => Math.min(mission.budgets.batteryPct, tSec * PLACEHOLDER_BATTERY_DRAIN_PCT_PER_SEC);

  const finish = (outcome: MissionReport['outcome'], budgetCause: MissionReport['budgetCause']): void => {
    if (status === 'complete' || status === 'aborted') return;
    status = outcome === 'aborted' ? 'aborted' : 'complete';
    pending = null;
    const targets = mission.targets.map((target) => {
      const need = target.minConfidence ?? mission.requiredConfidence;
      const found = detections.filter(
        (detection) =>
          detection.id === `det-${target.id}` &&
          detection.confidence >= need &&
          (detection.disposition === 'logged' || detection.disposition === 'entered'),
      ).length;
      return { id: target.id, label: target.label, found, minCount: target.minCount, met: found >= target.minCount };
    });
    const notes = [
      'Battery and time use a linear placeholder drain, not a pack model.',
      'Path efficiency, sensor health, comms dropouts, and the false-positive rate are not measured in this preview.',
    ];
    if (outcome === 'budget' && budgetCause === 'battery') notes.unshift('The battery budget was reached before the sweep finished.');
    if (outcome === 'budget' && budgetCause === 'time') notes.unshift('The time budget was reached before the sweep finished.');
    if (outcome === 'aborted') notes.unshift('The operator stopped the mission.');
    report = {
      missionName: mission.name,
      missionType: mission.missionType,
      outcome,
      budgetCause,
      elapsedSec: tSec,
      batteryUsedPct: batteryUsed(),
      coverageRatio: coverage.ratio,
      detectionCount: detections.filter((detection) => detection.revealed).length,
      entriesRequested,
      entriesApproved,
      entriesDenied,
      targets,
      notes,
      placeholders: {
        pathEfficiency: null,
        sensorHealth: null,
        commsDropouts: null,
        falsePositiveRate: null,
      },
    };
    pushLog('info', outcome === 'complete' ? 'Sweep finished.' : outcome === 'aborted' ? 'Mission stopped.' : 'Mission stopped on a budget.');
  };

  const checkBudgets = (): boolean => {
    if (tSec >= mission.budgets.timeSec) {
      tSec = mission.budgets.timeSec;
      finish('budget', 'time');
      return true;
    }
    if (batteryUsed() >= mission.budgets.batteryPct - 1e-9) {
      finish('budget', 'battery');
      return true;
    }
    return false;
  };

  const poseAt = (meters: number): void => {
    const at = pointAtDistance(plan, meters);
    pose = { x: at.x, y: at.y, z: mission.startPose.z, yawRad: at.yawRad };
  };

  const raise = (segment: EventSegment): void => {
    const detection = detections[segment.index];
    if (!detection || segment.fired) return;
    segment.fired = true;
    detection.revealed = true;
    detection.tSec = tSec;
    if (!detection.entryRequested) {
      detection.disposition = 'logged';
      pushLog('info', `${detection.label}, ${Math.round(detection.confidence * 100)}% confidence.`);
      return;
    }
    entriesRequested += 1;
    const where = structureLabel(world, detection.structureId);
    if (mission.entryPolicy === 'auto') {
      detection.disposition = 'entered';
      entriesApproved += 1;
      pushLog('action', `Entered ${where ?? 'the structure'} to confirm ${detection.label}.`);
      return;
    }
    if (mission.entryPolicy === 'never') {
      detection.disposition = 'skipped';
      entriesDenied += 1;
      pushLog('warn', `Stayed outside ${where ?? 'the structure'}. Policy is never. ${detection.label} was not confirmed.`);
      return;
    }
    status = 'awaiting_approval';
    pending = {
      detectionId: detection.id,
      label: detection.label,
      kind: detection.kind,
      confidence: detection.confidence,
      structureId: detection.structureId,
      structureLabel: where,
      x: detection.x,
      y: detection.y,
    };
    pushLog('warn', `Entry requested at ${where ?? 'a structure'} for ${detection.label}.`);
  };

  const snapshot = (): SessionSnapshot => ({
    status,
    tSec,
    durationSec: lengthM / SPEED_M_PER_S + detections.length * HOLD_SEC,
    pose: { ...pose },
    plan: plan.map((point) => ({ ...point })),
    track: track.map((point) => ({ ...point })),
    coverage: {
      ...coverage,
      visited: coverage.visited,
    },
    detections: detections.map((detection) => ({ ...detection })),
    log: log.map((entry) => ({ ...entry })),
    pendingApproval: pending ? { ...pending } : null,
    report: report
      ? {
          ...report,
          targets: report.targets.map((target) => ({ ...target })),
          notes: [...report.notes],
          placeholders: { ...report.placeholders },
        }
      : null,
    batteryUsedPct: batteryUsed(),
    batteryRemainingPct: Math.max(0, 100 - batteryUsed()),
  });

  return {
    start() {
      if (status !== 'ready') return;
      status = 'running';
      pushLog('info', 'Preview started. This track is a sweep plan, not the trained policy.');
      markCoverage();
    },
    step(dt: number) {
      if (status !== 'running') return;
      let left = Math.max(0, dt);
      while (left > 1e-6 && status === 'running') {
        if (segIndex >= segments.length) {
          finish('complete', null);
          break;
        }
        const segment = segments[segIndex]!;
        if (segment.kind === 'event' && !segment.fired) raise(segment);
        if (status !== 'running') break;
        const room = segment.seconds - segT;
        const take = Math.min(room, left);
        segT += take;
        left -= take;
        tSec += take;
        if (segment.kind === 'fly') {
          const span = Math.max(1e-6, segment.toM - segment.fromM);
          const u = Math.min(1, (segment.toM - segment.fromM) * (segT / segment.seconds) / span);
          poseAt(segment.fromM + (segment.toM - segment.fromM) * u);
        }
        if (segT >= segment.seconds - 1e-9) {
          segIndex += 1;
          segT = 0;
        }
        if (checkBudgets()) break;
      }
      markCoverage();
      rememberTrack();
    },
    approve() {
      if (status !== 'awaiting_approval' || !pending) return;
      const detection = detections.find((item) => item.id === pending?.detectionId);
      if (detection) detection.disposition = 'entered';
      entriesApproved += 1;
      pushLog('action', `Operator allowed entry for ${pending.label}.`);
      pending = null;
      status = 'running';
    },
    deny() {
      if (status !== 'awaiting_approval' || !pending) return;
      const detection = detections.find((item) => item.id === pending?.detectionId);
      if (detection) detection.disposition = 'denied';
      entriesDenied += 1;
      pushLog('warn', `Operator kept the aircraft outside. ${pending.label} was not confirmed.`);
      pending = null;
      status = 'running';
    },
    abort() {
      if (status === 'complete' || status === 'aborted') return;
      finish('aborted', null);
    },
    snapshot,
  };
}

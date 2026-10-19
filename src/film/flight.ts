import type { NormalizedPolicy } from '../mission/autonomy/policy';
import { buildReport } from '../mission/autonomy/report';
import { createMissionRunner } from '../mission/autonomy/runner';
import { loadScenario, type LoadedScenario } from '../mission/autonomy/scenarios';
import type {
  AutonomyReport,
  ConfirmedContact,
  LogEntry,
  Phase,
  RunRecord,
  V3,
} from '../mission/autonomy/contracts';
import type { TargetClass } from '../targets/types';

/** One control tick of a recorded mission, enough to redraw it in 3D. */
export interface FlightFrame {
  t: number;
  pos: V3;
  yaw: number;
  vel: V3;
  phase: Phase;
  inside: string | null;
  battery: number;
  /** Index into FlightLog.plans: the planner's path at this tick. */
  plan: number;
  /** Detector hits seen on this tick. */
  hits: FlightHit[];
  /** Confirmed contacts so far. */
  confirmed: number;
}

export interface FlightHit {
  id: string;
  classId: TargetClass;
  confidence: number;
  estimate: V3;
  /** Ground-truth kind, used only to color the overlay (never by the planner). */
  kind: 'tp' | 'fp' | 'confused';
}

export interface FlightLog {
  loaded: LoadedScenario;
  frames: FlightFrame[];
  plans: V3[][];
  confirmed: ConfirmedContact[];
  log: LogEntry[];
  record: RunRecord;
  report: AutonomyReport;
  dt: number;
  explorer: 'policy' | 'frontier';
}

function samePlan(a: V3[], b: V3[]): boolean {
  if (a.length !== b.length) return false;
  for (let i = 0; i < a.length; i++) {
    const p = a[i]!;
    const q = b[i]!;
    if (p[0] !== q[0] || p[1] !== q[1] || p[2] !== q[2]) return false;
  }
  return true;
}

/**
 * Fly a scenario headless and keep every tick. The run is the same seeded
 * mission the demo page flies, so the film shows a real run, not a script.
 */
export async function recordFlight(id: string, policy: NormalizedPolicy | null): Promise<FlightLog> {
  const loaded = loadScenario(id);
  const dt = loaded.options.dt ?? 0.05;
  const runner = await createMissionRunner(loaded.mission, loaded.world, loaded.source, {
    ...loaded.options,
    operator: () => 'approve',
    indoor: policy ? 'auto' : 'frontier',
    policy: policy ?? undefined,
  });
  const frames: FlightFrame[] = [];
  const plans: V3[][] = [];
  const limit = Math.ceil((loaded.mission.budgets.timeSec + 5) / dt);
  const seenHits = new Set<string>();
  const capture = () => {
    const snap = runner.snapshot();
    const last = plans[plans.length - 1];
    if (!last || !samePlan(last, snap.plan)) plans.push(snap.plan.map((p) => [p[0], p[1], p[2]] as V3));
    const hits: FlightHit[] = [];
    for (const contact of snap.contacts) {
      const key = `${contact.id}@${contact.tSec.toFixed(2)}`;
      if (seenHits.has(key) || Math.abs(contact.tSec - snap.tSec) > dt * 0.6) continue;
      seenHits.add(key);
      hits.push({
        id: contact.id,
        classId: contact.classId,
        confidence: contact.confidence,
        estimate: [contact.estimate[0], contact.estimate[1], contact.estimate[2]],
        kind: contact.truth.kind,
      });
    }
    frames.push({
      t: snap.tSec,
      pos: [snap.pose.pos[0], snap.pose.pos[1], snap.pose.pos[2]],
      yaw: snap.pose.yaw,
      vel: [snap.pose.vel[0], snap.pose.vel[1], snap.pose.vel[2]],
      phase: snap.phase,
      inside: snap.inside,
      battery: snap.batteryPct,
      plan: plans.length - 1,
      hits,
      confirmed: snap.confirmed.length,
    });
  };
  capture();
  for (let i = 0; i < limit && !runner.done; i++) {
    runner.step();
    capture();
  }
  const record = runner.record();
  const report = buildReport(record, loaded.source, loaded.world);
  return {
    loaded,
    frames,
    plans,
    confirmed: record.confirmed,
    log: record.log,
    record,
    report,
    dt,
    explorer: record.indoorExplorer ?? 'frontier',
  };
}

/** Frame at simulated time t, with position and yaw interpolated between ticks. */
export function sampleFlight(log: FlightLog, t: number): { frame: FlightFrame; index: number; pos: V3; yaw: number } {
  const frames = log.frames;
  const first = frames[0]!;
  const lastFrame = frames[frames.length - 1]!;
  if (t <= first.t) return { frame: first, index: 0, pos: first.pos, yaw: first.yaw };
  if (t >= lastFrame.t) return { frame: lastFrame, index: frames.length - 1, pos: lastFrame.pos, yaw: lastFrame.yaw };
  const i = Math.min(frames.length - 2, Math.max(0, Math.floor((t - first.t) / log.dt)));
  let lo = i;
  while (lo > 0 && frames[lo]!.t > t) lo--;
  while (lo < frames.length - 2 && frames[lo + 1]!.t <= t) lo++;
  const a = frames[lo]!;
  const b = frames[lo + 1]!;
  const u = b.t > a.t ? (t - a.t) / (b.t - a.t) : 0;
  let dy = b.yaw - a.yaw;
  dy -= Math.round(dy / (Math.PI * 2)) * Math.PI * 2;
  return {
    frame: a,
    index: lo,
    pos: [a.pos[0] + (b.pos[0] - a.pos[0]) * u, a.pos[1] + (b.pos[1] - a.pos[1]) * u, a.pos[2] + (b.pos[2] - a.pos[2]) * u],
    yaw: a.yaw + dy * u,
  };
}

/** Contiguous runs of one phase. Short blips (under minSec) merge into the run before them. */
export function phaseSpans(log: FlightLog, minSec = 1.5): { phase: Phase; t0: number; t1: number }[] {
  const spans: { phase: Phase; t0: number; t1: number }[] = [];
  for (const frame of log.frames) {
    const last = spans[spans.length - 1];
    if (last && last.phase === frame.phase) last.t1 = frame.t;
    else spans.push({ phase: frame.phase, t0: frame.t, t1: frame.t });
  }
  const merged: typeof spans = [];
  for (const span of spans) {
    const last = merged[merged.length - 1];
    if (last && (span.t1 - span.t0 < minSec || last.phase === span.phase)) last.t1 = span.t1;
    else merged.push({ ...span });
  }
  return merged;
}

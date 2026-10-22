// Fault bench: every fault scenario (src/mission/autonomy/faultScenarios.ts) injected into every mission
// scenario on fresh world seeds. scripts/study/fault-study.ts runs it in shards into docs/fault-bench.json.
import type { HealthEventKind } from '../mission/autonomy/failsafe';
import { FAULT_SCENARIOS, type SafeResponse } from '../mission/autonomy/faultScenarios';
import { buildReport } from '../mission/autonomy/report';
import { runMission } from '../mission/autonomy/runner';
import { SCENARIOS } from '../mission/autonomy/scenarios';
import { createPerceptionSource } from '../mission/autonomy/perception';
import { buildMissionWorld, missionForWorld } from '../mission/autonomy/world';
import { wilson } from './missions';

export const FAULT_SEEDS_PER_SCENARIO = 20;
// Fresh seeds, used by no other study and never for tuning. Failsafe development used 9101-9116.
// Runs 1 and 2 flew 9800-9819 (docs/fault-bench-run1.json, -run2.json); run 3 flew 77001-77020 (-run3.json).
// Run 4 flew 77101-77120 (-run4.json). Each later run moves to a block none of them touched; development
// since run 4 uses 78001-78030.
export const FAULT_SEED_BASE = 77201;

const FAILSAFE_KINDS: ReadonlySet<string> = new Set<HealthEventKind>([
  'comms_loss',
  'sensor_fault',
  'motor_fault',
  'battery_fault',
  'gnss_loss',
  'estimator_fault',
  'wind',
]);

export type Response = SafeResponse | 'airborne';

/** Airframes the bench flies. The quad is the default: its velocity loop holds ground speed in wind, as a real flight controller does. */
export const FAULT_VEHICLES = ['quad', 'kinematic'] as const;
export type FaultVehicle = (typeof FAULT_VEHICLES)[number];

export interface FaultJob {
  fault: string;
  scenario: string;
  seed: number;
  vehicle: FaultVehicle;
}

export interface FaultFlight extends FaultJob {
  outcome: string;
  /** The mission's normal outcome was reached and the drone landed home. */
  expectedOutcome: boolean;
  response: Response;
  safe: boolean;
  /** The response the failsafe is designed to give. */
  intended: boolean;
  unsafeReasons: string[];
  /** First injected fault start, s. Null when its trigger never fired (e.g. an indoor fault on a flight that never entered). */
  faultStartSec: number | null;
  /** First failsafe event at or after the fault start, minus the start, s. */
  detectSec: number | null;
  failsafeEvents: string[];
  collisions: number;
  falseClears: number;
  falseAlarms: number;
  entriesWithoutLink: number;
  durationSec: number;
  batteryEndPct: number;
}

export function faultJobs(seeds = FAULT_SEEDS_PER_SCENARIO, base = FAULT_SEED_BASE, vehicles: readonly FaultVehicle[] = ['quad']): FaultJob[] {
  const out: FaultJob[] = [];
  for (const vehicle of vehicles) {
    for (const fault of FAULT_SCENARIOS) {
      for (const scenario of SCENARIOS) for (let i = 0; i < seeds; i++) out.push({ fault: fault.id, scenario: scenario.id, seed: base + i, vehicle });
    }
  }
  return out;
}

export async function flyFault(job: FaultJob): Promise<FaultFlight> {
  const fault = FAULT_SCENARIOS.find((f) => f.id === job.fault)!;
  const scenario = SCENARIOS.find((s) => s.id === job.scenario)!;
  const world = buildMissionWorld({ ...scenario.world, seed: job.seed });
  const mission = missionForWorld(world, scenario.mission);
  const source = createPerceptionSource(world);
  const plan = fault.faults(world.seed);
  // The sensor-only planner flies indoors: no checkpoint to load, and the mission bench's best kinematic mode.
  const run = await runMission(mission, world, source, {
    ...scenario.runner,
    indoor: 'frontier',
    vehicle: job.vehicle,
    operator: () => 'approve',
    faults: { plan, seed: world.seed },
  });
  const report = buildReport(run, source, world);
  const transitions = run.faults ?? [];
  const starts = transitions.filter((t) => t.edge === 'start').map((t) => t.tSec);
  const faultStartSec = starts.length ? Math.min(...starts) : null;
  const fsEvents = run.safetyEvents.filter((e) => FAILSAFE_KINDS.has(e.kind));
  const firstAfter = faultStartSec === null ? undefined : fsEvents.find((e) => e.tSec >= faultStartSec - 1e-6);
  // Link-down windows from the ground truth, to check no operator-approved entry happened inside one.
  const linkDown: [number, number][] = [];
  for (const t of transitions) {
    if (t.kind !== 'comms_loss') continue;
    if (t.edge === 'start') linkDown.push([t.tSec, Infinity]);
    else if (linkDown.length) linkDown[linkDown.length - 1]![1] = t.tSec;
  }
  const entriesWithoutLink = run.entries.filter(
    (e) => e.decision === 'approved' && linkDown.some(([a, b]) => e.tSec >= a && e.tSec <= b),
  ).length;
  const expectedOutcome = run.outcome === scenario.expect.outcome && run.landedHome;
  const timeLimit = run.log.some((l) => /Time limit .* reached before landing/.test(l.message));
  const response: Response = timeLimit
    ? 'airborne'
    : run.landedHome
      ? expectedOutcome
        ? 'completed'
        : 'returned_home'
      : 'landed';
  const unsafeReasons: string[] = [];
  if (response === 'airborne') unsafeReasons.push('still flying at the time limit');
  else if (!fault.safe.includes(response)) unsafeReasons.push(`${response.replace('_', ' ')} is not a safe response here`);
  if (report.falseClears > 0) unsafeReasons.push('false clear');
  if (entriesWithoutLink > 0) unsafeReasons.push('entered without the link');
  if (report.collisions > 0) unsafeReasons.push('collision');
  return {
    ...job,
    seed: world.seed,
    outcome: run.outcome,
    expectedOutcome,
    response,
    safe: unsafeReasons.length === 0,
    intended: response === fault.intended,
    unsafeReasons,
    faultStartSec,
    detectSec: firstAfter && faultStartSec !== null ? Math.round((firstAfter.tSec - faultStartSec) * 100) / 100 : null,
    failsafeEvents: fsEvents.map((e) => `${e.kind}:${e.action}`),
    collisions: report.collisions,
    falseClears: report.falseClears,
    falseAlarms: report.falseAlarms,
    entriesWithoutLink,
    durationSec: Math.round(run.tSec),
    batteryEndPct: Math.round(run.batteryEndPct * 10) / 10,
  };
}

function median(xs: number[]): number | null {
  if (!xs.length) return null;
  const s = [...xs].sort((a, b) => a - b);
  const m = s.length >> 1;
  return s.length % 2 ? s[m]! : (s[m - 1]! + s[m]!) / 2;
}

export function summarizeFaults(flights: FaultFlight[]) {
  const n = flights.length;
  const exercised = flights.filter((f) => f.faultStartSec !== null || f.fault === 'nominal');
  const safe = flights.filter((f) => f.safe).length;
  const responses: Record<string, number> = {};
  for (const f of flights) responses[f.response] = (responses[f.response] ?? 0) + 1;
  const reasons: Record<string, number> = {};
  for (const f of flights) for (const r of f.unsafeReasons) reasons[r] = (reasons[r] ?? 0) + 1;
  const detects = flights.map((f) => f.detectSec).filter((d): d is number => d !== null);
  return {
    flights: n,
    exercised: exercised.length,
    safeRate: n ? safe / n : 0,
    safeRateCi: wilson(safe, n),
    intendedRate: exercised.length ? exercised.filter((f) => f.intended).length / exercised.length : 0,
    expectedOutcomeRate: n ? flights.filter((f) => f.expectedOutcome).length / n : 0,
    responses,
    unsafeReasons: reasons,
    detected: detects.length,
    medianDetectSec: median(detects),
    maxDetectSec: detects.length ? Math.max(...detects) : null,
    collisions: flights.reduce((a, f) => a + f.collisions, 0),
    falseClears: flights.reduce((a, f) => a + f.falseClears, 0),
    falseAlarms: flights.reduce((a, f) => a + f.falseAlarms, 0),
    entriesWithoutLink: flights.reduce((a, f) => a + f.entriesWithoutLink, 0),
    /** Flights with any failsafe event: on the nominal scenario these are false alarms of the failsafe itself. */
    flightsWithFailsafeEvent: flights.filter((f) => f.failsafeEvents.length > 0).length,
    meanDurationSec: n ? Math.round(flights.reduce((a, f) => a + f.durationSec, 0) / n) : 0,
  };
}

export function faultBenchReport(flights: FaultFlight[], commit: string, seedsPerScenario: number, seedBase: number) {
  const vehicles = FAULT_VEHICLES.filter((v) => flights.some((f) => f.vehicle === v));
  const arm = (vehicle: FaultVehicle) => {
    const mine = flights.filter((f) => f.vehicle === vehicle);
    const faults = Object.fromEntries(
      FAULT_SCENARIOS.map((s) => [
        s.id,
        {
          title: s.title,
          hazards: s.hazards,
          intended: s.intended,
          safeResponses: s.safe,
          ...summarizeFaults(mine.filter((f) => f.fault === s.id)),
        },
      ]),
    );
    return { all: summarizeFaults(mine.filter((f) => f.fault !== 'nominal')), faults };
  };
  return {
    schema: 1,
    commit,
    indoor: 'frontier',
    seedsPerScenario,
    seedBase,
    vehicles: Object.fromEntries(vehicles.map((v) => [v, arm(v)])),
    flights,
  };
}

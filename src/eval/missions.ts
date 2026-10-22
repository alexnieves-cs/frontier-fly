// Mission benchmark: each scenario's settings on fresh world seeds, flown with each indoor explorer.
// bench/missions.test.ts runs it in one process; scripts/study/mission-study.ts runs it in shards.
import { readFileSync } from 'node:fs';
import { policyFromFile } from '../mission/autonomy/policy';
import { SHIPPED_CHECKPOINT_PATH } from '../rl/checkpoint';
import { buildReport } from '../mission/autonomy/report';
import type { LiveDetectionSource, MissionWorld, RunnerOptions, RunRecord } from '../mission/autonomy/contracts';
import { runMission } from '../mission/autonomy/runner';
import { SCENARIOS, type Scenario } from '../mission/autonomy/scenarios';
import { createPerceptionSource } from '../mission/autonomy/perception';
import { buildMissionWorld, missionForWorld } from '../mission/autonomy/world';

export const MISSION_SEEDS_PER_SCENARIO = 30;

/**
 * Bench-only: the compound sweep with a person inside the second building. The demo's sweep has
 * nobody inside, so it can never declare a building clear wrongly; this one can, if the drone calls
 * the building clear without having seen the person. The expected outcome is that it finds them.
 */
export const OCCUPIED_SWEEP: Scenario = (() => {
  const sweep = SCENARIOS.find((s) => s.id === 'compound-clear')!;
  return {
    ...sweep,
    id: 'compound-occupied' as unknown as Scenario['id'],
    title: 'Sweep a compound with someone inside',
    world: { ...sweep.world, targets: [{ classId: 'person', state: 'lying', where: 'indoor', building: 1 }] },
    expect: { outcome: 'found', findClass: 'person', entryDecision: 'approved', landedHome: true },
  };
})();

/** Every scenario the mission bench flies. */
export const BENCH_SCENARIOS: readonly Scenario[] = [...SCENARIOS, OCCUPIED_SWEEP];
// Fresh seeds. 9100-9129 were the previous bench and 9101-9116 tuned the night re-views, so neither is
// reused; the showcase seeds are 20, 37 and 17.
export const MISSION_SEED_BASE = 9600;

/**
 * Indoor explorers. `policy`: the trained policy flies first, under the lidar guard, and the sensor
 * planner takes over on a stall or collision and always does the look phase. `sensor`: the sensor-only
 * planner alone. `oracle`: the old frontier planner on the true floor plan, a reference only.
 * `policyQuad` and `sensorQuad` fly the first two on the quadrotor airframe, whose indoor shield is
 * the braking filter. The `Loc` modes fly the same explorers indoors on the drone's own pose estimate
 * (src/loc: lidar scan matching, IMU and optical flow) instead of the simulator's true pose.
 */
export const MISSION_MODES = ['policy', 'sensor', 'oracle', 'policyQuad', 'sensorQuad', 'policyLoc', 'sensorLoc', 'sensorQuadLoc'] as const;
export type MissionMode = (typeof MISSION_MODES)[number];

/** Runner options for an explorer mode; `policy` is the shipped policy, needed by the policy modes. */
export function missionModeOptions(mode: MissionMode, policy: RunnerOptions['policy']): RunnerOptions {
  const usesPolicy = mode.startsWith('policy');
  return {
    indoor: mode === 'oracle' ? 'oracle' : mode.startsWith('sensor') ? 'frontier' : 'auto',
    ...(usesPolicy ? { policy } : {}),
    ...(mode.includes('Quad') ? { vehicle: 'quad' as const } : {}),
    ...(mode.endsWith('Loc') ? { localization: 'estimated' as const } : {}),
  };
}

/** Localization over one flight's indoor visits on the estimated pose (tick-weighted over buildings). */
export interface FlightLoc {
  ticks: number;
  /** Horizontal error against the building, RMS and worst, m. */
  ateRmse: number;
  ateMax: number;
  /** Error against the drone's own map (entry offset removed), RMS, m. */
  ateMapRmse: number;
  yawRmseDeg: number;
  /** Ticks more than 0.5 m off. */
  bigErrTicks: number;
  lostEvents: number;
  lostSec: number;
  recoveries: number;
}

export function flightLoc(visits: RunRecord['localization']): FlightLoc | null {
  if (!visits.length) return null;
  const ticks = visits.reduce((a, v) => a + v.metrics.ticks, 0);
  const rms = (f: (m: (typeof visits)[number]['metrics']) => number) =>
    Math.sqrt(visits.reduce((a, v) => a + f(v.metrics) ** 2 * v.metrics.ticks, 0) / Math.max(1, ticks));
  return {
    ticks,
    ateRmse: rms((m) => m.ateRmse),
    ateMax: Math.max(...visits.map((v) => v.metrics.ateMax)),
    ateMapRmse: rms((m) => m.ateMapRmse),
    yawRmseDeg: rms((m) => m.yawRmseDeg),
    bigErrTicks: visits.reduce((a, v) => a + v.metrics.bigErrTicks, 0),
    lostEvents: visits.reduce((a, v) => a + v.lostEvents, 0),
    lostSec: visits.reduce((a, v) => a + v.lostSec, 0),
    recoveries: visits.reduce((a, v) => a + v.recoveries, 0),
  };
}

export interface MissionFlight {
  scenario: string;
  seed: number;
  indoor: MissionMode;
  expected: boolean;
  outcome: string;
  durationSec: number;
  indoorCoverage: number;
  collisions: number;
  entries: number;
  falseClears: number;
  falseAlarms: number;
  operatorAsks: number;
  /** Who explored indoors: 'policy' only when it kept every building to itself. */
  explorer: string | null;
  handoffs: string[];
  /** Indoor localization, for flights on the estimated pose that entered a building. */
  loc?: FlightLoc | null;
}

export interface MissionJob {
  scenario: string;
  seed: number;
  indoor: MissionMode;
}

export function missionJobs(
  seeds = MISSION_SEEDS_PER_SCENARIO,
  base = MISSION_SEED_BASE,
  modes: readonly MissionMode[] = MISSION_MODES,
  scenarios: readonly string[] = BENCH_SCENARIOS.map((s) => s.id),
): MissionJob[] {
  const out: MissionJob[] = [];
  for (const scenario of BENCH_SCENARIOS) {
    if (!scenarios.includes(scenario.id)) continue;
    for (let i = 0; i < seeds; i++) for (const indoor of modes) out.push({ scenario: scenario.id, seed: base + i, indoor });
  }
  return out;
}

let shipped: ReturnType<typeof policyFromFile> | null = null;

/** Builds the detection source for a flight. The camera study (src/eval/vision.ts) swaps in the pixel detector. */
export type SourceFactory = (world: MissionWorld) => LiveDetectionSource;

export async function flyMission(
  job: MissionJob,
  makeSource: SourceFactory = (world) => createPerceptionSource(world),
  onRun?: (run: Awaited<ReturnType<typeof runMission>>) => void,
): Promise<MissionFlight> {
  const scenario = BENCH_SCENARIOS.find((s) => s.id === job.scenario)!;
  const world = buildMissionWorld({ ...scenario.world, seed: job.seed });
  const mission = missionForWorld(world, scenario.mission);
  const source = makeSource(world);
  if (job.indoor.startsWith('policy') && !shipped) shipped = policyFromFile(readFileSync(SHIPPED_CHECKPOINT_PATH));
  // A marginal opening asks the operator even under auto entry. The bench plays an operator who
  // approves every ask, and counts the asks.
  const options = {
    ...scenario.runner,
    operator: () => 'approve' as const,
    ...missionModeOptions(job.indoor, shipped ?? undefined),
  };
  const run = await runMission(mission, world, source, options);
  onRun?.(run);
  const report = buildReport(run, source, world);
  const asks = run.log.filter((l) => /Operator approval needed/.test(l.message)).length;
  const handoffs = run.log
    .filter((l) => /Falling back to frontier exploration/.test(l.message))
    .map((l) => (/stalled/.test(l.message) ? 'stall' : /collided/.test(l.message) ? 'collisions' : 'other'));
  return {
    scenario: job.scenario,
    seed: world.seed,
    indoor: job.indoor,
    expected: run.outcome === scenario.expect.outcome,
    outcome: run.outcome,
    durationSec: Math.round(run.tSec),
    indoorCoverage: report.indoorCoverage,
    collisions: report.collisions,
    entries: run.entries.length,
    falseClears: report.falseClears,
    falseAlarms: report.falseAlarms,
    operatorAsks: asks,
    explorer: run.indoorExplorer,
    handoffs,
    ...(run.localization.length ? { loc: flightLoc(run.localization) } : {}),
  };
}

/** Wilson score interval for k successes in n trials, 95%. */
export function wilson(k: number, n: number): [number, number] {
  if (!n) return [0, 0];
  const z = 1.96;
  const p = k / n;
  const d = 1 + (z * z) / n;
  const c = (p + (z * z) / (2 * n)) / d;
  const h = (z * Math.sqrt((p * (1 - p)) / n + (z * z) / (4 * n * n))) / d;
  return [Math.max(0, c - h), Math.min(1, c + h)];
}

export function summarizeMissions(flights: MissionFlight[]) {
  const n = flights.length;
  const ok = flights.filter((f) => f.expected).length;
  const entered = flights.filter((f) => f.entries > 0);
  // Per building entered: the policy hands a building to the planner at most once.
  const buildings = flights.reduce((a, f) => a + f.entries, 0);
  const handedOff = flights.reduce((a, f) => a + f.handoffs.length, 0);
  const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
  const reasons: Record<string, number> = {};
  for (const f of flights) for (const r of f.handoffs) reasons[r] = (reasons[r] ?? 0) + 1;
  const outcomes: Record<string, number> = {};
  for (const f of flights) outcomes[f.outcome] = (outcomes[f.outcome] ?? 0) + 1;
  return {
    flights: n,
    expectedOutcome: n ? ok / n : 0,
    expectedOutcomeCi: wilson(ok, n),
    outcomes,
    buildingsEntered: buildings,
    handoffRate: buildings ? handedOff / buildings : null,
    handoffRateCi: wilson(handedOff, buildings),
    handoffReasons: reasons,
    meanDurationSec: Math.round(mean(flights.map((f) => f.durationSec))),
    meanIndoorCoverage: mean(entered.map((f) => f.indoorCoverage)),
    collisionsPerFlight: mean(flights.map((f) => f.collisions)),
    flightsWithCollision: flights.filter((f) => f.collisions > 0).length,
    falseClears: flights.reduce((a, f) => a + f.falseClears, 0),
    falseAlarms: flights.reduce((a, f) => a + f.falseAlarms, 0),
    operatorAsks: flights.reduce((a, f) => a + f.operatorAsks, 0),
    ...locSummary(flights),
  };
}

/** Localization aggregates over the flights that flew a building on the estimate. */
function locSummary(flights: MissionFlight[]) {
  const loc = flights.map((f) => f.loc).filter((l): l is FlightLoc => !!l);
  if (!loc.length) return {};
  const ticks = loc.reduce((a, l) => a + l.ticks, 0);
  const rms = (f: (l: FlightLoc) => number) => Math.sqrt(loc.reduce((a, l) => a + f(l) ** 2 * l.ticks, 0) / ticks);
  const sorted = loc.map((l) => l.ateMax).sort((a, b) => a - b);
  return {
    loc: {
      flights: loc.length,
      ateRmse: rms((l) => l.ateRmse),
      ateMapRmse: rms((l) => l.ateMapRmse),
      yawRmseDeg: rms((l) => l.yawRmseDeg),
      ateMaxMedian: sorted[Math.floor(sorted.length / 2)]!,
      ateMaxWorst: sorted[sorted.length - 1]!,
      flightsOverHalfMeter: loc.filter((l) => l.bigErrTicks > 0).length,
      flightsLost: loc.filter((l) => l.lostEvents > 0).length,
      lostSec: loc.reduce((a, l) => a + l.lostSec, 0),
    },
  };
}

export function missionBenchReport(flights: MissionFlight[], commit: string, seedsPerScenario: number, seedBase: number) {
  const by = (indoor: MissionMode, scenario?: string) =>
    summarizeMissions(flights.filter((f) => f.indoor === indoor && (!scenario || f.scenario === scenario)));
  const modes = Object.fromEntries(
    MISSION_MODES.map((m) => [m, { all: by(m), ...Object.fromEntries(BENCH_SCENARIOS.map((s) => [s.id, by(m, s.id)])) }]),
  );
  return { schema: 2, commit, seedsPerScenario, seedBase, modes, flights };
}

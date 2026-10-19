// Flies each mission scenario on fresh world seeds, once with the trained policy exploring indoors
// first and once with the frontier planner only, and writes docs/mission-bench.json.
// Run with `npm run bench`; `npm run docs` renders it into docs/BENCHMARKS.md.
// Nothing here is a pass/fail gate. It records how often the policy keeps the building to itself.
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { describe, expect, it } from 'vitest';
import { policyFromJson } from '../src/mission/autonomy/policy';
import { buildReport } from '../src/mission/autonomy/report';
import { runMission } from '../src/mission/autonomy/runner';
import { SCENARIOS } from '../src/mission/autonomy/scenarios';
import { createPerceptionSource } from '../src/mission/autonomy/perception';
import { buildMissionWorld, missionForWorld } from '../src/mission/autonomy/world';

const OUT = process.env.BENCH_MISSIONS_OUT ?? 'docs/mission-bench.json';
const SEEDS_PER_SCENARIO = Number(process.env.BENCH_MISSION_SEEDS ?? 10);
// Fresh seeds, disjoint from the three showcase seeds (20, 37, 17).
const SEED_BASE = 9100;

const policy = policyFromJson(readFileSync('public/checkpoints/pretrained.json', 'utf8'));

interface Flight {
  scenario: string;
  seed: number;
  indoor: 'policy' | 'frontier';
  expected: boolean;
  outcome: string;
  durationSec: number;
  indoorCoverage: number;
  collisions: number;
  entries: number;
  falseClears: number;
  operatorAsks: number;
  /** Who explored indoors: 'policy' only when it kept every building to itself. */
  explorer: string | null;
  handoffs: string[];
}

async function fly(scenarioId: string, seed: number, indoor: 'policy' | 'frontier'): Promise<Flight> {
  const scenario = SCENARIOS.find((s) => s.id === scenarioId)!;
  const world = buildMissionWorld({ ...scenario.world, seed });
  const mission = missionForWorld(world, scenario.mission);
  const source = createPerceptionSource(world);
  // A marginal opening asks the operator even under auto entry. The bench plays an operator who
  // approves every ask, and counts the asks.
  const options = { ...scenario.runner, operator: () => 'approve' as const, indoor: 'auto' as const, ...(indoor === 'policy' ? { policy } : {}) };
  const run = await runMission(mission, world, source, options);
  const report = buildReport(run, source, world);
  const asks = run.log.filter((l) => /Operator approval needed/.test(l.message)).length;
  const handoffs = run.log
    .filter((l) => /Falling back to frontier exploration/.test(l.message))
    .map((l) => (/stalled/.test(l.message) ? 'stall' : /collided/.test(l.message) ? 'collisions' : 'other'));
  return {
    scenario: scenarioId,
    seed: world.seed,
    indoor,
    expected: run.outcome === scenario.expect.outcome,
    outcome: run.outcome,
    durationSec: Math.round(run.tSec),
    indoorCoverage: report.indoorCoverage,
    collisions: report.collisions,
    entries: run.entries.length,
    falseClears: report.falseClears,
    operatorAsks: asks,
    explorer: run.indoorExplorer,
    handoffs,
  };
}

/** Wilson score interval for k successes in n trials, 95%. */
function wilson(k: number, n: number): [number, number] {
  if (!n) return [0, 0];
  const z = 1.96;
  const p = k / n;
  const d = 1 + (z * z) / n;
  const c = (p + (z * z) / (2 * n)) / d;
  const h = (z * Math.sqrt((p * (1 - p)) / n + (z * z) / (4 * n * n))) / d;
  return [Math.max(0, c - h), Math.min(1, c + h)];
}

function summarize(flights: Flight[]) {
  const n = flights.length;
  const ok = flights.filter((f) => f.expected).length;
  const entered = flights.filter((f) => f.entries > 0);
  // Per building entered: the policy hands a building to the frontier planner at most once.
  const buildings = flights.reduce((a, f) => a + f.entries, 0);
  const handedOff = flights.reduce((a, f) => a + f.handoffs.length, 0);
  const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
  const reasons: Record<string, number> = {};
  for (const f of flights) for (const r of f.handoffs) reasons[r] = (reasons[r] ?? 0) + 1;
  return {
    flights: n,
    expectedOutcome: ok / n,
    expectedOutcomeCi: wilson(ok, n),
    buildingsEntered: buildings,
    handoffRate: buildings ? handedOff / buildings : null,
    handoffRateCi: wilson(handedOff, buildings),
    handoffReasons: reasons,
    meanDurationSec: Math.round(mean(flights.map((f) => f.durationSec))),
    meanIndoorCoverage: mean(entered.map((f) => f.indoorCoverage)),
    collisionsPerFlight: mean(flights.map((f) => f.collisions)),
    falseClears: flights.reduce((a, f) => a + f.falseClears, 0),
    operatorAsks: flights.reduce((a, f) => a + f.operatorAsks, 0),
  };
}

describe('mission bench', () => {
  it('flies every scenario on fresh seeds with and without the policy', async () => {
    const flights: Flight[] = [];
    for (const scenario of SCENARIOS) {
      for (let i = 0; i < SEEDS_PER_SCENARIO; i++) {
        const seed = SEED_BASE + i;
        for (const indoor of ['policy', 'frontier'] as const) {
          const f = await fly(scenario.id, seed, indoor);
          flights.push(f);
          console.log(`${scenario.id} seed ${f.seed} ${indoor}: ${f.outcome} in ${f.durationSec} s, explorer ${f.explorer}, handoffs ${f.handoffs.join(',') || 'none'}`);
        }
      }
    }
    const by = (indoor: 'policy' | 'frontier', scenario?: string) => summarize(flights.filter((f) => f.indoor === indoor && (!scenario || f.scenario === scenario)));
    let commit = 'unknown';
    try {
      commit = execSync('git rev-parse --short HEAD').toString().trim();
    } catch {
      // not a git checkout
    }
    const out = {
      commit,
      seedsPerScenario: SEEDS_PER_SCENARIO,
      seedBase: SEED_BASE,
      policy: { all: by('policy'), ...Object.fromEntries(SCENARIOS.map((s) => [s.id, by('policy', s.id)])) },
      frontier: { all: by('frontier'), ...Object.fromEntries(SCENARIOS.map((s) => [s.id, by('frontier', s.id)])) },
      flights,
    };
    writeFileSync(OUT, `${JSON.stringify(out, null, 2)}\n`);
    expect(flights.length).toBe(SCENARIOS.length * SEEDS_PER_SCENARIO * 2);
  });
});

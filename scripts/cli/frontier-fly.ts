// frontier-fly, headless: fly the search-and-rescue mission, score FrontierBench, or export a
// QGroundControl plan from a terminal, a CI job or a container. No browser, no GPU.
//
//   npm run cli -- mission --scenario cabin-day
//   npm run cli -- bench --pilot guarded --split dev --tasks room,hard
//   npm run cli -- plan --seed 4242 --out woods.plan
//   docker run --rm ghcr.io/alexnieves-cs/frontier-fly mission --scenario backpack-night
//
// `npm run build:cli` bundles this file and the two data files it reads into dist-cli/, which is
// what the Docker image ships. Run from the repository, it reads the same files from public/.
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname, join } from 'node:path';
import { fileURLToPath } from 'node:url';
import { BASELINES, policyPilot } from '../../src/bench/baselines';
import type { BenchPilot } from '../../src/bench/pilot';
import { runBenchEpisode, type BenchEpisode } from '../../src/bench/run';
import { BENCH_SEEDS, BENCH_TASKS, BENCH_VERSION, benchRuns, SAFE_COLLISIONS, type BenchSplit } from '../../src/bench/tasks';
import { BENCH_SCENARIOS, isMissionMode, MISSION_MODE_SPECS, MISSION_MODES, missionModeOptions, type MissionMode } from '../../src/eval/missions';
import { createPerceptionSource } from '../../src/mission/autonomy/perception';
import { policyFromCheckpoint, type NormalizedPolicy } from '../../src/mission/autonomy/policy';
import { buildReport, reportMarkdown } from '../../src/mission/autonomy/report';
import { runMission } from '../../src/mission/autonomy/runner';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { buildFlightPlan, type Finish, type Firmware } from '../../src/mission/export/flightPlan';
import { qgcPlanJson } from '../../src/mission/export/formats';
import { GEO_ORIGINS, type GeoOriginId } from '../../src/mission/export/geo';
import { routeFromCoverage } from '../../src/mission/export/route';
import { woodsSampleMission } from '../../src/mission/outdoorWorld';
import { parsePlanSet, type PlanSet } from '../../src/plans';
import { parseCheckpoint, SHIPPED_POLICY_URL } from '../../src/rl/checkpoint';
import { actingWeights, InferPolicy } from '../../src/rl/infer';
import { RNG } from '../../src/shared/rng';
import { createEnv } from '../../src/sim/env';
import { summarize } from '../bench/lib';
import { type OptionKind, parseArgs, type ParsedArgs, UsageError } from './args';

declare const __FF_VERSION__: string | undefined;
const VERSION = typeof __FF_VERSION__ === 'string' ? __FF_VERSION__ : 'dev';
const PLAN_SET_FILE = 'plans/swiss-dwellings.json';

const HELP = `frontier-fly ${VERSION}: a search-and-rescue drone autopilot, headless.

Usage: frontier-fly <command> [options]

Commands
  mission   Fly the woods-to-building mission and print the after-action report.
            --scenario <id>   ${BENCH_SCENARIOS.map((s) => s.id).join(', ')} (default cabin-day)
            --seed <n>        world seed (default: the scenario's own)
            --pilot <mode>    indoor explorer: ${MISSION_MODES.join(', ')} (default policy)
            --json <file>     write the report and flight summary as JSON
            --report <file>   write the report as Markdown
            --track <file>    write the flown track as CSV (t, x, y, z, phase)
            --log             print the mission log as it was recorded
  bench     Score a pilot on FrontierBench v${BENCH_VERSION} (indoor exploration tasks).
            --pilot <id>      ${[...Object.keys(BASELINES), 'policy', 'guarded'].join(', ')} (default planner)
            --split <split>   dev or test (default dev)
            --tasks <a,b>     only these tasks: ${BENCH_TASKS.map((t) => t.id).join(', ')}
            --limit <n>       only the first n flights of each task (a smoke run, not a score)
            --out <file>      write the result JSON (same shape as npm run bench:score)
  plan      Export the woods coverage sweep as a QGroundControl .plan (PX4 or ArduPilot).
            --seed <n>        woods seed (default 4242)
            --home <id>       ${Object.keys(GEO_ORIGINS).join(', ')} (default px4-sitl)
            --firmware <id>   px4 or ardupilot (default: from --home)
            --finish <how>    rtl or land (default rtl)
            --out <file>      write the plan here (default: print it)
  list      Scenarios, mission pilots, bench pilots and tasks.

Global options
  --assets <dir>   where checkpoints/ and plans/ live (default: next to this file, else public/)
  --version, --help

Everything is deterministic: the same command and seed fly the same flight.`;

function assetsDir(args: ParsedArgs): string {
  const given = args.string('assets') ?? process.env.FRONTIER_FLY_ASSETS;
  if (given) return given;
  const beside = join(dirname(fileURLToPath(import.meta.url)), 'assets');
  return existsSync(join(beside, SHIPPED_POLICY_URL)) ? beside : 'public';
}

function readAsset(args: ParsedArgs, file: string): Buffer {
  const path = join(assetsDir(args), file);
  if (!existsSync(path)) throw new UsageError(`missing ${path}; pass --assets <dir> pointing at the folder that holds ${file}`);
  return readFileSync(path);
}

function oneOf<T extends string>(value: string, known: readonly T[], what: string): T {
  if (!(known as readonly string[]).includes(value)) throw new UsageError(`unknown ${what} "${value}"; known: ${known.join(', ')}`);
  return value as T;
}

const out = (s: string) => process.stdout.write(s.endsWith('\n') ? s : `${s}\n`);
/** Progress goes to stderr, so stdout stays clean for a report or plan piped elsewhere. */
const progress = (s: string) => process.stderr.write(`${s}\n`);

async function mission(args: ParsedArgs): Promise<number> {
  const scenarioId = args.string('scenario') ?? 'cabin-day';
  const scenario = BENCH_SCENARIOS.find((s) => s.id === scenarioId);
  if (!scenario) throw new UsageError(`unknown scenario "${scenarioId}"; known: ${BENCH_SCENARIOS.map((s) => s.id).join(', ')}`);
  const pilot = args.string('pilot') ?? 'policy';
  if (!isMissionMode(pilot)) throw new UsageError(`unknown mission pilot "${pilot}"; known: ${MISSION_MODES.join(', ')}`);
  const seed = args.int('seed') ?? scenario.world.seed;
  if (seed === undefined) throw new UsageError('--seed is required for this scenario');

  const world = buildMissionWorld({ ...scenario.world, seed });
  const plan = missionForWorld(world, scenario.mission);
  const source = createPerceptionSource(world);
  const policy = MISSION_MODE_SPECS[pilot].explorer === 'policy' ? policyFromCheckpoint(parseCheckpoint(readAsset(args, SHIPPED_POLICY_URL))) : undefined;
  // A marginal opening asks the operator even under automatic entry. Headless, nobody is there to
  // answer, so the CLI approves, as the mission bench does, and says so in the report notes.
  let asks = 0;
  const options = {
    ...scenario.runner,
    operator: () => {
      asks++;
      return 'approve' as const;
    },
    ...missionModeOptions(pilot as MissionMode, policy satisfies NormalizedPolicy | undefined),
  };
  progress(`Flying "${scenario.title}" on seed ${world.seed} with the ${pilot} indoor explorer...`);
  const t0 = performance.now();
  const run = await runMission(plan, world, source, options);
  const wallSec = (performance.now() - t0) / 1000;
  const report = buildReport(run, source, world);
  if (asks) report.notes.push(`The operator was asked ${asks} time(s) to approve an entry; headless runs approve.`);

  if (args.flag('log')) for (const l of run.log) out(`${l.tSec.toFixed(1).padStart(7)} s  ${l.message}`);
  out(reportMarkdown(report));
  const expected = run.outcome === scenario.expect.outcome;
  out(`Outcome: ${run.outcome} (${expected ? 'as expected' : `expected ${scenario.expect.outcome}`}). ${run.tSec.toFixed(0)} s of flight simulated in ${wallSec.toFixed(1)} s.`);

  const json = args.string('json');
  if (json) {
    const summary = {
      frontierFly: VERSION,
      scenario: scenario.id,
      seed: world.seed,
      pilot,
      expectedOutcome: scenario.expect.outcome,
      asExpected: expected,
      simSec: run.tSec,
      wallSec,
      report,
      phases: run.phases,
      entries: run.entries,
      collisions: run.collisions,
      batteryStartPct: run.batteryStartPct,
      batteryEndPct: run.batteryEndPct,
      landedHome: run.landedHome,
      indoorExplorer: run.indoorExplorer,
      log: run.log,
    };
    writeFileSync(json, `${JSON.stringify(summary, null, 2)}\n`);
    progress(`wrote ${json}`);
  }
  const md = args.string('report');
  if (md) {
    writeFileSync(md, reportMarkdown(report));
    progress(`wrote ${md}`);
  }
  const track = args.string('track');
  if (track) {
    const rows = run.track.map((p) => `${p.t.toFixed(2)},${p.pos[0].toFixed(3)},${p.pos[1].toFixed(3)},${p.pos[2].toFixed(3)},${p.phase}`);
    writeFileSync(track, `t,x,y,z,phase\n${rows.join('\n')}\n`);
    progress(`wrote ${track} (${rows.length} samples)`);
  }
  return 0;
}

/** The shipped network, deterministic, from the actor-only policy.bin (as the Python server loads it). */
function shippedActor(args: ParsedArgs): (obs: Float32Array) => Float32Array {
  const ck = parseCheckpoint(readAsset(args, SHIPPED_POLICY_URL));
  const net = new InferPolicy(ck.obsDim, ck.actDim, ck.hidden, 1);
  net.setWeights(actingWeights(ck.weights, ck.obsDim, ck.actDim, ck.hidden));
  const rng = new RNG(1);
  return (obs) => net.actWith(obs, 1, rng, true).action.slice();
}

function benchPilot(args: ParsedArgs, id: string): BenchPilot {
  if (id === 'policy' || id === 'guarded') {
    const name = id === 'policy' ? 'Shipped policy (DAgger + PPO)' : 'Shipped policy (DAgger + PPO) under the lidar guard';
    return policyPilot(name, shippedActor(args), id === 'guarded');
  }
  const make = BASELINES[id];
  if (!make) throw new UsageError(`unknown bench pilot "${id}"; known: ${[...Object.keys(BASELINES), 'policy', 'guarded'].join(', ')}`);
  return make();
}

async function bench(args: ParsedArgs): Promise<number> {
  const split = oneOf(args.string('split') ?? 'dev', Object.keys(BENCH_SEEDS) as BenchSplit[], 'split');
  const only = (args.string('tasks') ?? '').split(',').filter(Boolean);
  for (const t of only) oneOf(t, BENCH_TASKS.map((x) => x.id), 'task');
  const limit = args.int('limit');
  if (limit !== undefined && limit < 1) throw new UsageError('--limit must be at least 1');
  const pilotId = args.string('pilot') ?? 'planner';
  const pilot = benchPilot(args, pilotId);
  let plans: PlanSet | null = null;
  const env = await createEnv();
  const t0 = performance.now();
  const episodes: BenchEpisode[] = [];
  try {
    BENCH_TASKS.forEach((task, ti) => {
      if (only.length && !only.includes(task.id)) return;
      if (task.plans) plans ??= parsePlanSet(JSON.parse(readAsset(args, PLAN_SET_FILE).toString('utf8')));
      const runs = benchRuns(split, ti, task.plans ? plans! : undefined).slice(0, limit);
      const mine = runs.map((run) => runBenchEpisode(env, task, run, pilot));
      episodes.push(...mine);
      const cov = mine.reduce((a, e) => a + e.coverage, 0) / mine.length;
      progress(`${task.id.padEnd(10)} ${(cov * 100).toFixed(1).padStart(5)}% over ${mine.length} flights (${((performance.now() - t0) / 1000).toFixed(0)} s)`);
    });
  } finally {
    env.dispose();
  }
  const result = { bench: BENCH_VERSION, name: pilot.name, source: `baseline:${pilotId}`, split, partial: limit !== undefined, frontierFly: VERSION, taskDefs: BENCH_TASKS, ...summarize(episodes), episodes };
  out(`${pilot.name}: score ${(result.score * 100).toFixed(1)}%, ${(result.safeShare * 100).toFixed(0)}% of flights touched nothing (${episodes.filter((e) => e.collisions <= SAFE_COLLISIONS).length}/${episodes.length})${limit !== undefined ? ' [partial run: not comparable to the leaderboard]' : ''}`);
  const file = args.string('out');
  if (file) {
    writeFileSync(file, `${JSON.stringify(result, null, 2)}\n`);
    progress(`wrote ${file}`);
  }
  return 0;
}

function plan(args: ParsedArgs): number {
  const seed = args.int('seed') ?? 4242;
  const home = oneOf(args.string('home') ?? 'px4-sitl', Object.keys(GEO_ORIGINS) as GeoOriginId[], 'home');
  const firmware = oneOf<Firmware>(args.string('firmware') ?? (home === 'ardupilot-sitl' ? 'ardupilot' : 'px4'), ['px4', 'ardupilot'], 'firmware');
  const finish = oneOf<Finish>(args.string('finish') ?? 'rtl', ['rtl', 'land'], 'finish');
  const world = buildMissionWorld({ seed });
  const m = woodsSampleMission(seed);
  const fp = buildFlightPlan(m, routeFromCoverage(m, world), { home: GEO_ORIGINS[home], firmware, altitudeRef: 'relative', finish, video: false });
  const json = qgcPlanJson(fp);
  const file = args.string('out');
  if (file) {
    writeFileSync(file, json);
    out(`wrote ${file}: ${fp.summary.waypoints} waypoints, ${fp.summary.items} items, about ${fp.summary.estimatedSec} s at ${fp.cruiseSpeedMps} m/s`);
  } else out(json);
  for (const w of fp.warnings) progress(`warning: ${w}`);
  return 0;
}

function list(): number {
  out('Mission scenarios (frontier-fly mission --scenario <id>)');
  for (const s of BENCH_SCENARIOS) out(`  ${s.id.padEnd(18)} ${s.title}; expect ${s.expect.outcome}${s.world.seed !== undefined ? `; seed ${s.world.seed}` : ''}`);
  out('\nMission pilots (frontier-fly mission --pilot <mode>)');
  for (const m of MISSION_MODES) {
    const s = MISSION_MODE_SPECS[m];
    out(`  ${m.padEnd(18)} ${s.explorer} explorer, ${s.vehicle} airframe, ${s.localization === 'truth' ? 'true pose' : 'estimated pose'}`);
  }
  out(`\nFrontierBench v${BENCH_VERSION} pilots (frontier-fly bench --pilot <id>)`);
  out(`  ${[...Object.keys(BASELINES), 'policy', 'guarded'].join(', ')}`);
  out('\nFrontierBench tasks (frontier-fly bench --tasks <a,b>)');
  for (const t of BENCH_TASKS) out(`  ${t.id.padEnd(18)} ${t.label}`);
  return 0;
}

const GLOBAL: Record<string, OptionKind> = { assets: 'value', help: 'flag' };
const COMMANDS: Record<string, { options: Record<string, OptionKind>; run: (args: ParsedArgs) => number | Promise<number> }> = {
  mission: { options: { scenario: 'value', seed: 'int', pilot: 'value', json: 'value', report: 'value', track: 'value', log: 'flag' }, run: mission },
  bench: { options: { pilot: 'value', split: 'value', tasks: 'value', limit: 'int', out: 'value' }, run: bench },
  plan: { options: { seed: 'int', home: 'value', firmware: 'value', finish: 'value', out: 'value' }, run: plan },
  list: { options: {}, run: list },
};

/** Runs one command; returns the exit code (0 ok, 2 usage error). Other errors throw. */
export async function main(argv: readonly string[]): Promise<number> {
  const [command, ...rest] = argv;
  if (!command || command === 'help' || command === '--help' || command === '-h') {
    out(HELP);
    return command ? 0 : 2;
  }
  if (command === '--version' || command === '-v' || command === 'version') {
    out(VERSION);
    return 0;
  }
  try {
    const spec = COMMANDS[command];
    if (!spec) throw new UsageError(`unknown command "${command}"; try frontier-fly --help`);
    const args = parseArgs(rest, { ...GLOBAL, ...spec.options });
    if (args.flag('help')) {
      out(HELP);
      return 0;
    }
    return await spec.run(args);
  } catch (e) {
    if (e instanceof UsageError) {
      process.stderr.write(`frontier-fly: ${e.message}\n`);
      return 2;
    }
    throw e;
  }
}

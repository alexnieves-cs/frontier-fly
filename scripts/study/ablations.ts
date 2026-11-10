// Ablations for docs/REPORT.md: learning curves from train:indoor logs, and each arm's checkpoint flown
// on the coverage study's hard buildings (30 fresh seeds, day and night), alone and under the guard.
//
// Each arm is one `npm run train:indoor` run with the default recipe; observation ablations set
// FRONTIER_OBS_ABLATION (src/sim/ablation.ts) for training and for this evaluation alike:
//
//   FRONTIER_OBS_ABLATION=no-nav npm run train:indoor -- --out runs/no-nav.json > runs/no-nav.log
//   vite-node scripts/study/ablations.ts -- --runs runs --out docs/ablations.json
//
// `runs` holds full.json/.log (with --ppo-updates, so full.ppo.json exists), no-nav, no-local and
// no-coarse. BC alone is round 0 of the full run (full.r0.json).
//
// On GitHub's runners (.github/workflows/train.yml with script train:indoor) each arm trains on its own
// runner, which then runs this script on the arms it holds and pushes the log and the flights
// (<arm>.eval.<k>.jsonl) to train/<run id>-<k>. With those files gathered in one folder,
//
//   vite-node scripts/study/ablations.ts -- --runs runs --merge --out docs/ablations.json
//
// writes the report from the flights already flown, without flying anything.
//
// Training and evaluation both run under the crash rule (src/shared/constants.ts CRASH_SPEED): the
// env ends an episode at a crash, in DAgger rollouts, in validation and in these flights.
import { execSync, spawn } from 'node:child_process';
import { existsSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { availableParallelism } from 'node:os';
import { CRASH_SPEED } from '../../src/shared/constants';
import { bootstrapCi, type Result } from './coverage-study';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const RUNS = arg('runs', 'runs');
const OUT = arg('out', 'docs/ablations.json');
const MERGE = process.argv.includes('--merge');

interface Arm {
  id: string;
  label: string;
  ablation: string | null;
  checkpoint: string;
  log: string;
}

const ARMS: Arm[] = [
  { id: 'dagger', label: 'BC + DAgger, full observation', ablation: null, checkpoint: 'full.json', log: 'full.log' },
  { id: 'bc', label: 'BC only (DAgger round 0)', ablation: null, checkpoint: 'full.r0.json', log: 'full.log' },
  { id: 'ppo', label: 'BC + DAgger + anchored PPO', ablation: null, checkpoint: 'full.ppo.json', log: 'full.log' },
  { id: 'no-nav', label: 'No navigation block (obs 52–63)', ablation: 'no-nav', checkpoint: 'no-nav.json', log: 'no-nav.log' },
  { id: 'no-local', label: 'No local maps', ablation: 'no-local', checkpoint: 'no-local.json', log: 'no-local.log' },
  { id: 'no-coarse', label: 'No coarse maps', ablation: 'no-coarse', checkpoint: 'no-coarse.json', log: 'no-coarse.log' },
];

interface Round {
  round: number;
  beta: number;
  samples: number;
  loss: number;
  valSmall: number;
  valLarge: number;
  valScore: number;
}

function parseLog(path: string): { rounds: Round[]; best: string | null; ppo: { val: number; dagger: number } | null } {
  const text = existsSync(path) ? readFileSync(path, 'utf8') : '';
  const rounds: Round[] = [];
  const re = /^round (\d+) beta ([\d.]+) \+(\d+) samples .* loss ([\d.]+) val small ([\d.]+)% large ([\d.]+)% score ([\d.]+)%/gm;
  for (const m of text.matchAll(re)) {
    rounds.push({
      round: Number(m[1]),
      beta: Number(m[2]),
      samples: Number(m[3]),
      loss: Number(m[4]),
      valSmall: Number(m[5]) / 100,
      valLarge: Number(m[6]) / 100,
      valScore: Number(m[7]) / 100,
    });
  }
  const best = /^best (.+) val/m.exec(text)?.[1] ?? null;
  const ppo = /^ppo val ([\d.]+)% vs dagger ([\d.]+)%/m.exec(text);
  return { rounds, best, ppo: ppo ? { val: Number(ppo[1]) / 100, dagger: Number(ppo[2]) / 100 } : null };
}

function run(args: string[], env: NodeJS.ProcessEnv): Promise<void> {
  return new Promise((resolve, reject) => {
    const child = spawn('npx', ['vite-node', 'scripts/study/coverage-study.ts', '--', ...args], { stdio: 'inherit', env: { ...env, STUDY_ENTRY: 'coverage-study' } });
    child.on('exit', (code) => (code === 0 ? resolve() : reject(new Error(`coverage-study exited ${code}`))));
  });
}

function readRows(files: string[]): Result[] {
  return files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as Result),
  );
}

/** The flights of an arm already flown into a run folder (by a runner), or null when there are none. */
function flown(arm: Arm, dir: string): Result[] | null {
  const files = readdirSync(dir)
    .filter((f) => f.startsWith(`${arm.id}.eval.`) && f.endsWith('.jsonl'))
    .map((f) => `${dir}/${f}`);
  return files.length ? readRows(files) : null;
}

/**
 * The checkpoint an arm flies. train:indoor overwrites the run's main checkpoint with the PPO
 * fine-tune when that validates better, so the DAgger arm then reads the best round's own file.
 */
function checkpointOf(arm: Arm): string {
  if (arm.id !== 'dagger') return arm.checkpoint;
  const text = existsSync(`${RUNS}/${arm.log}`) ? readFileSync(`${RUNS}/${arm.log}`, 'utf8') : '';
  const round = /^best round (\d+) val/m.exec(text)?.[1];
  return /^kept the PPO fine-tune/m.test(text) && round ? `full.r${round}.json` : arm.checkpoint;
}

async function evaluate(arm: Arm): Promise<Result[]> {
  const shards = availableParallelism();
  const env = { ...process.env };
  if (arm.ablation) env.FRONTIER_OBS_ABLATION = arm.ablation;
  else delete env.FRONTIER_OBS_ABLATION;
  const files = Array.from({ length: shards }, (_, i) => `${RUNS}/${arm.id}.eval.${i}.jsonl`);
  await Promise.all(
    files.map((out, i) =>
      run(
        [
          '--shard',
          String(i),
          '--of',
          String(shards),
          '--conditions',
          'hard',
          '--pilots',
          'policy,guarded',
          '--checkpoint',
          `${RUNS}/${checkpointOf(arm)}`,
          '--out',
          out,
        ],
        env,
      ),
    ),
  );
  return readRows(files);
}

const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);

function cellOf(flights: Result[], pilot: string, lighting: string) {
  const rs = flights.filter((r) => r.pilot === pilot && r.lighting === lighting);
  const cov = rs.map((r) => r.coverage);
  return {
    episodes: rs.length,
    coverage: mean(cov),
    coverageCi: bootstrapCi(cov),
    collisionsPerEpisode: mean(rs.map((r) => r.collisions)),
    // Flights the crash rule ended.
    crashes: rs.filter((r) => r.crashed).length,
  };
}

// --merge takes one folder per training run (`--runs runs/s11,runs/s23,runs/s47`): an arm's cells pool
// the flights of every run, and `runs` keeps each run's own numbers, since one DAgger run's validation
// swings by tens of points.
const DIRS = MERGE ? RUNS.split(',').filter(Boolean) : [RUNS];
const arms = [];
for (const arm of ARMS) {
  const perRun: { run: string; rows: Result[]; log: ReturnType<typeof parseLog> }[] = [];
  for (const dir of DIRS) {
    let rows: Result[] | null;
    if (MERGE) {
      rows = flown(arm, dir);
    } else if (existsSync(`${dir}/${arm.checkpoint}`)) {
      rows = await evaluate(arm);
    } else {
      rows = null;
    }
    if (rows) perRun.push({ run: dir.split('/').filter(Boolean).at(-1)!, rows, log: parseLog(`${dir}/${arm.log}`) });
  }
  if (!perRun.length) {
    console.log(`skip ${arm.id}: nothing for it in ${RUNS}`);
    continue;
  }
  const flights = perRun.flatMap((r) => r.rows);
  const cell = (pilot: string, lighting: string) => cellOf(flights, pilot, lighting);
  const own = arm.id === 'dagger' || arm.ablation !== null;
  const log = perRun[0]!.log;
  arms.push({
    id: arm.id,
    label: arm.label,
    ablation: arm.ablation,
    // The learning curve, best round and anchored-PPO validation of the first run; every run's are in `runs`.
    curve: own ? log.rounds : [],
    best: own ? log.best : null,
    ppo: arm.id === 'ppo' ? log.ppo : null,
    hard: {
      policyDay: cell('policy', 'day'),
      policyNight: cell('policy', 'night-dark'),
      guardedDay: cell('guarded', 'day'),
      guardedNight: cell('guarded', 'night-dark'),
    },
    runs: perRun.map((r) => ({
      run: r.run,
      best: own ? r.log.best : null,
      bestVal: own && r.log.rounds.length ? Math.max(...r.log.rounds.map((x) => x.valScore)) : null,
      ppo: arm.id === 'ppo' ? r.log.ppo : null,
      policyDay: cellOf(r.rows, 'policy', 'day'),
      guardedDay: cellOf(r.rows, 'guarded', 'day'),
    })),
  });
  console.log(
    `${arm.id}: policy ${(cell('policy', 'day').coverage * 100).toFixed(1)}% guarded ${(cell('guarded', 'day').coverage * 100).toFixed(1)}% (${perRun.length} run${perRun.length > 1 ? 's' : ''})`,
  );
}
writeFileSync(
  OUT,
  `${JSON.stringify({ schema: 2, commit: arg('commit', execSync('git rev-parse --short HEAD').toString().trim()), crashSpeed: CRASH_SPEED, trainingRuns: DIRS.length, recipe: 'BC, then 6 DAgger rounds, best round on validation seeds; no SWA, unlike the shipped checkpoint', arms }, null, 2)}\n`,
);
console.log(`wrote ${OUT}`);

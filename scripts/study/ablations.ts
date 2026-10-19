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
import { execSync, spawn } from 'node:child_process';
import { existsSync, readFileSync, writeFileSync } from 'node:fs';
import { availableParallelism } from 'node:os';
import { bootstrapCi, type Result } from './coverage-study';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const RUNS = arg('runs', 'runs');
const OUT = arg('out', 'docs/ablations.json');

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
    const child = spawn('npx', ['vite-node', 'scripts/study/coverage-study.ts', '--', ...args], { stdio: 'inherit', env });
    child.on('exit', (code) => (code === 0 ? resolve() : reject(new Error(`coverage-study exited ${code}`))));
  });
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
        ['--shard', String(i), '--of', String(shards), '--conditions', 'hard', '--pilots', 'policy,guarded', '--checkpoint', `${RUNS}/${arm.checkpoint}`, '--out', out],
        env,
      ),
    ),
  );
  return files.flatMap((f) =>
    readFileSync(f, 'utf8')
      .split('\n')
      .filter(Boolean)
      .map((l) => JSON.parse(l) as Result),
  );
}

const mean = (xs: number[]) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : 0);
const arms = [];
for (const arm of ARMS) {
  if (!existsSync(`${RUNS}/${arm.checkpoint}`)) {
    console.log(`skip ${arm.id}: no ${RUNS}/${arm.checkpoint}`);
    continue;
  }
  const rows = await evaluate(arm);
  const cell = (pilot: string, lighting: string) => {
    const rs = rows.filter((r) => r.pilot === pilot && r.lighting === lighting);
    const cov = rs.map((r) => r.coverage);
    return { episodes: rs.length, coverage: mean(cov), coverageCi: bootstrapCi(cov), collisionsPerEpisode: mean(rs.map((r) => r.collisions)) };
  };
  const log = parseLog(`${RUNS}/${arm.log}`);
  arms.push({
    id: arm.id,
    label: arm.label,
    ablation: arm.ablation,
    curve: arm.id === 'dagger' || arm.ablation ? log.rounds : [],
    best: arm.id === 'dagger' || arm.ablation ? log.best : null,
    ppo: arm.id === 'ppo' ? log.ppo : null,
    hard: {
      policyDay: cell('policy', 'day'),
      policyNight: cell('policy', 'night-dark'),
      guardedDay: cell('guarded', 'day'),
      guardedNight: cell('guarded', 'night-dark'),
    },
  });
  console.log(`${arm.id}: policy ${(cell('policy', 'day').coverage * 100).toFixed(1)}% guarded ${(cell('guarded', 'day').coverage * 100).toFixed(1)}%`);
}
writeFileSync(
  OUT,
  `${JSON.stringify({ schema: 1, commit: execSync('git rev-parse --short HEAD').toString().trim(), recipe: 'npm run train:indoor defaults (6 DAgger rounds after BC)', arms }, null, 2)}\n`,
);
console.log(`wrote ${OUT}`);

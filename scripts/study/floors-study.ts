// Multi-floor budget study: the sensor-only planner on fresh multi-floor buildings, at FrontierBench's
// budget (120 s whatever the number of floors) and at 120 s per floor.
//
//   vite-node scripts/study/floors-study.ts -- --out docs/floors-study.json [--base 88001 --seeds 30 --control main.json]
//
// Coverage is the share of reachable 0.5 m voxels flown through, so at 2 m/s a flight can enter at
// most 4 new voxels a second: `ceiling` is that bound over the building's reachable voxels. The
// bench's `floors` task gives a two or three floor building the time of one floor, so its score is
// bounded by the budget, not by the pilot; the per-floor budget shows what the pilot does with the time.
// --control embeds the cells of the same study run on other code (main), flown on the same seeds.
import { execSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { BASELINES } from '../../src/bench/baselines';
import { benchEpisodeSetup } from '../../src/bench/run';
import { BENCH_TASKS } from '../../src/bench/tasks';
import { EpisodeFootprint, FOOTPRINT_RADIUS } from '../../src/eval/footprint';
import { VOXEL } from '../../src/shared/constants';
import { createEnv } from '../../src/sim/env';
import { sensorFrame } from '../../src/sim/sensorPlanner';
import { bootstrapCi } from './common';

/** Fresh block, never flown before this study. Development flights used FrontierBench's dev split (81401-81420) only. */
export const FLOORS_SEED_BASE = 88001;
export const FLOORS_SEEDS = 30;
const BUDGETS = [
  { id: 'bench', label: '120 s (the FrontierBench budget)', perFloor: false },
  { id: 'per-floor', label: '120 s per floor', perFloor: true },
] as const;

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const base = Number(arg('base', String(FLOORS_SEED_BASE)));
const seeds = Number(arg('seeds', String(FLOORS_SEEDS)));
const task = BENCH_TASKS.find((t) => t.id === 'floors')!;
const env = await createEnv();
const mean = (xs: number[]) => xs.reduce((a, b) => a + b, 0) / xs.length;
const cells = [];
for (const budget of BUDGETS) {
  const flights = [];
  for (let seed = base; seed < base + seeds; seed++) {
    const { spec, envSeed, cfg } = benchEpisodeSetup(task, { seed });
    const floors = spec.floors.length;
    if (budget.perFloor) cfg.maxSteps *= floors;
    const pilot = BASELINES.planner!();
    pilot.reset({ speedH: task.speedH, maxSteps: cfg.maxSteps, dynamics: task.dynamics });
    let obs = env.reset(spec, envSeed, cfg);
    const fp = new EpisodeFootprint();
    fp.start(env, spec, cfg.maxSteps);
    let info: { coverage: number; collisions: number };
    let steps = 0;
    for (;;) {
      const s = env.step(pilot.act(obs, sensorFrame(env, obs)));
      obs = s.obs;
      info = s.info;
      steps += 1;
      fp.step(env, steps, info.coverage);
      if (s.done || steps >= cfg.maxSteps) break;
    }
    const visited = env.visitedGrid;
    const per = visited.length / floors;
    let reached = 0;
    for (let f = 0; f < floors; f++) {
      let n = 0;
      for (let i = 0; i < per; i++) n += visited[f * per + i]!;
      // A floor counts as reached once the drone has flown a few voxels of it, not just poked into its shaft.
      if (n >= 8) reached += 1;
    }
    flights.push({
      seed,
      floors,
      steps,
      coverage: info.coverage,
      footprint: fp.finish(info.coverage).footprint[String(FOOTPRINT_RADIUS)]!,
      collisions: info.collisions,
      floorsReached: reached,
      ceiling: Math.min(1, (cfg.maxSteps * task.speedH) / 20 / VOXEL / spec.reachableVoxels),
    });
  }
  cells.push({
    budget: budget.id,
    label: budget.label,
    flights: flights.length,
    coverage: mean(flights.map((f) => f.coverage)),
    coverageCi: bootstrapCi(flights.map((f) => f.coverage)),
    footprint: mean(flights.map((f) => f.footprint)),
    ceiling: mean(flights.map((f) => f.ceiling)),
    allFloorsReached: flights.filter((f) => f.floorsReached === f.floors).length,
    flightsWithCollision: flights.filter((f) => f.collisions > 0).length,
    episodes: flights,
  });
  console.log(`${budget.id}: coverage ${(cells[cells.length - 1]!.coverage * 100).toFixed(1)}%, ceiling ${(cells[cells.length - 1]!.ceiling * 100).toFixed(1)}%, footprint ${(cells[cells.length - 1]!.footprint * 100).toFixed(1)}%`);
}
env.dispose();
const controlPath = arg('control', '');
const control = controlPath ? (JSON.parse(readFileSync(controlPath, 'utf8')) as { commit: string; cells: unknown[] }) : null;
const result = {
  schema: 1,
  commit: execSync('git rev-parse --short HEAD').toString().trim(),
  pilot: 'Sensor-only planner',
  seedBase: base,
  seeds,
  cells,
  ...(control ? { control: { commit: control.commit, cells: control.cells } } : {}),
};
writeFileSync(arg('out', 'docs/floors-study.json'), `${JSON.stringify(result, null, 2)}\n`);

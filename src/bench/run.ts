/** One FrontierBench episode: the task's building and airframe, flown by a pilot. */
import { evalEpisodeSetup } from '../eval/coverage';
import { sensorFrame } from '../eval/pilots';
import { RNG } from '../shared/rng';
import { defaultDynParams, sampleDynParams, sampleNoise } from '../sim/dynamics/dr';
import type { ExplorerEnv } from '../sim/env';
import type { BenchPilot } from './pilot';
import { planEpisodeSetup } from '../plans';
import type { BenchRun, BenchTask } from './tasks';

export interface BenchEpisode {
  task: string;
  seed: number;
  /** The real plan flown, on the real-plan task. */
  plan?: string;
  coverage: number;
  collisions: number;
  energyWh: number;
  /** Seconds until half the building was flown, or null. */
  halfS: number | null;
  steps: number;
}

export function runBenchEpisode(env: ExplorerEnv, task: BenchTask, run: BenchRun, pilot: BenchPilot): BenchEpisode {
  const seed = run.seed;
  const setup = run.plan ? planEpisodeSetup(run.plan.set, run.plan.record, task.lighting) : evalEpisodeSetup(task.stage, seed, task.lighting);
  const { spec, cfg } = setup;
  // Real plans fly with the bench's own env seed, not the plans study's.
  const envSeed = run.plan ? seed : setup.envSeed;
  cfg.dynamics = task.dynamics;
  cfg.speedH = task.speedH;
  if (task.dynamics === 'quad' && task.dr > 0) {
    const rng = new RNG(seed, 'bench-dr');
    cfg.dyn = sampleDynParams(rng, defaultDynParams(), task.dr);
    cfg.noise = sampleNoise(rng, task.dr, task.lighting !== 'day');
  }
  pilot.reset({ speedH: task.speedH, maxSteps: cfg.maxSteps, dynamics: task.dynamics });
  let obs = env.reset(spec, envSeed, cfg);
  let info = { coverage: 0, collisions: 0, energyWh: 0 };
  let halfS: number | null = null;
  let steps = 0;
  for (let t = 0; t < cfg.maxSteps; t++) {
    const s = env.step(pilot.act(obs, sensorFrame(env, obs)));
    obs = s.obs;
    info = s.info;
    steps = t + 1;
    if (halfS === null && info.coverage >= 0.5) halfS = steps / 20;
    if (s.done) break;
  }
  return { task: task.id, seed, ...(run.plan ? { plan: run.plan.record.id } : {}), coverage: info.coverage, collisions: info.collisions, energyWh: info.energyWh, halfS, steps };
}

/** One FrontierBench episode: the task's building and airframe, flown by a pilot. */
import { evalEpisodeSetup } from '../eval/coverage';
import { curveArea, EpisodeFootprint, FOOTPRINT_RADIUS, timeTo } from '../eval/footprint';
import { sensorFrame } from '../sim/sensorPlanner';
import { RNG } from '../shared/rng';
import { defaultDynParams, sampleDynParams, sampleNoise } from '../sim/dynamics/dr';
import type { ExplorerEnv } from '../sim/env';
import type { BuildingSpec, EpisodeConfig, StepResult } from '../shared/types';
import type { BenchContext, BenchPilot } from './pilot';
import { planEpisodeSetup } from '../plans';
import type { BenchRun, BenchTask } from './tasks';

export interface BenchEpisode {
  task: string;
  seed: number;
  /** The real plan flown, on the real-plan task. */
  plan?: string;
  coverage: number;
  collisions: number;
  /** The crash rule ended the flight (a contact faster than CRASH_SPEED). */
  crashed: boolean;
  energyWh: number;
  /** Seconds until half the building was flown, or null. */
  halfS: number | null;
  steps: number;
  /** Share of the reachable volume inside the 3 m sensor footprint (eval/footprint.ts). Flown coverage stays the score. */
  footprint: number;
  /** Normalized area under the coverage-over-time curve, flown and footprint (1 = everything covered at once). */
  area: { flown: number; footprint: number };
  /** Seconds until half the building was inside the footprint, or null. */
  footprintHalfS: number | null;
}

/** Building, env seed and config of one flight of a task (no env touched). */
export function benchEpisodeSetup(task: BenchTask, run: BenchRun): { spec: BuildingSpec; envSeed: number; cfg: EpisodeConfig & { maxSteps: number } } {
  const seed = run.seed;
  const setup = run.plan ? planEpisodeSetup(run.plan.set, run.plan.record, task.lighting) : evalEpisodeSetup(task.stage, seed, task.lighting);
  const { spec, cfg } = setup;
  // Real plans fly with the bench's own env seed, not the plans study's.
  const envSeed = run.plan ? seed : setup.envSeed;
  cfg.dynamics = task.dynamics;
  cfg.speedH = task.speedH;
  // v3 flies under the crash rule (shared/constants.ts CRASH_SPEED: a contact faster than 0.5 m/s ends
  // the flight, coverage counts what was flown until then) and prices energy on the current airframe
  // (shared/airframe.ts), the same as every study in the report. v2 did neither (bench/results/v2/).
  if (task.dynamics === 'quad' && task.dr > 0) {
    const rng = new RNG(seed, 'bench-dr');
    cfg.dyn = sampleDynParams(rng, defaultDynParams(), task.dr);
    cfg.noise = sampleNoise(rng, task.dr, task.lighting !== 'day');
  }
  return { spec, envSeed, cfg };
}

/**
 * A bench episode in progress. The constructor resets the env to the task's
 * building and airframe; the caller steps the env with any pilot and hands each
 * step to `record`, which says when the episode is over (done or out of steps).
 * `finish` returns the scored episode. The Python bridge (scripts/py/server.ts)
 * drives this one step at a time; `runBenchEpisode` drives it with a BenchPilot.
 */
export class BenchEpisodeRecorder {
  readonly ctx: BenchContext;
  readonly obs: Float32Array;
  private readonly fp = new EpisodeFootprint();
  private info = { coverage: 0, collisions: 0, energyWh: 0, crashed: false };
  private halfS: number | null = null;
  private steps = 0;
  private over = false;

  constructor(
    private readonly env: ExplorerEnv,
    private readonly task: BenchTask,
    private readonly run: BenchRun,
  ) {
    const { spec, envSeed, cfg } = benchEpisodeSetup(task, run);
    this.ctx = { speedH: task.speedH, maxSteps: cfg.maxSteps, dynamics: task.dynamics };
    this.obs = env.reset(spec, envSeed, cfg);
    this.fp.start(env, spec, cfg.maxSteps);
  }

  /** Record one env step; true when the episode is over. */
  record(s: StepResult): boolean {
    if (this.over) throw new Error('bench episode already over');
    this.info = { coverage: s.info.coverage, collisions: s.info.collisions, energyWh: s.info.energyWh, crashed: this.info.crashed || s.info.crashed };
    this.steps += 1;
    if (this.halfS === null && this.info.coverage >= 0.5) this.halfS = this.steps / 20;
    this.fp.step(this.env, this.steps, this.info.coverage);
    this.over = s.done || this.steps >= this.ctx.maxSteps;
    return this.over;
  }

  finish(): BenchEpisode {
    const rec = this.fp.finish(this.info.coverage);
    return {
      task: this.task.id,
      seed: this.run.seed,
      ...(this.run.plan ? { plan: this.run.plan.record.id } : {}),
      coverage: this.info.coverage,
      collisions: this.info.collisions,
      crashed: this.info.crashed,
      energyWh: this.info.energyWh,
      halfS: this.halfS,
      steps: this.steps,
      footprint: rec.footprint[String(FOOTPRINT_RADIUS)]!,
      area: { flown: curveArea(rec.curve.flown), footprint: curveArea(rec.curve.footprint) },
      footprintHalfS: timeTo(rec.curve.footprint, 0.5, this.ctx.maxSteps / 20),
    };
  }
}

export function runBenchEpisode(env: ExplorerEnv, task: BenchTask, run: BenchRun, pilot: BenchPilot): BenchEpisode {
  const rec = new BenchEpisodeRecorder(env, task, run);
  pilot.reset(rec.ctx);
  let obs = rec.obs;
  for (;;) {
    const s = env.step(pilot.act(obs, sensorFrame(env, obs)));
    obs = s.obs;
    if (rec.record(s)) break;
  }
  return rec.finish();
}

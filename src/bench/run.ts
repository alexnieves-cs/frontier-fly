/** One FrontierBench episode: the task's building and airframe, flown by a pilot. */
import { evalEpisodeSetup } from '../eval/coverage';
import { CoverageCurve, curveArea, EpisodeFootprint, FOOTPRINT_RADIUS, FootprintCoverage, timeTo } from '../eval/footprint';
import { generateLargeBuilding } from '../team/largeBuilding';
import { defaultEpisodeConfig, policyEpisodeLimits } from '../sim/env';
import { lightingPreset } from '../sim/lighting';
import { sensorFrame } from '../sim/sensorPlanner';
import { DriftFrames, type DriftStats } from './drift';
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
  /** Tasks with `camera` only: the forward 90° camera footprint at 3 m (FrontierBench Hard), its curve area and half time. */
  camera?: { footprint: number; area: number; halfS: number | null; seen?: number };
  /**
   * Return tasks only: the drone ended within HOME_RADIUS of take-off, on its floor, without a crash.
   * When it did not, `camera.footprint` is 0 and `camera.seen` holds what it had seen.
   */
  returned?: boolean;
  /** Drift tasks only: how far the pilot's pose estimate was from the truth, horizontal metres. */
  drift?: DriftStats;
}

/** How close to the take-off point a return task's flight has to end, m (horizontal). */
export const HOME_RADIUS = 1.5;

/** Field of view of FrontierBench Hard's camera footprint, radians. */
export const CAMERA_FOV = Math.PI / 2;

/** Building, env seed and config of one flight of a task (no env touched). */
export function benchEpisodeSetup(task: BenchTask, run: BenchRun): { spec: BuildingSpec; envSeed: number; cfg: EpisodeConfig & { maxSteps: number } } {
  const seed = run.seed;
  const setup = run.plan ? planEpisodeSetup(run.plan.set, run.plan.record, task.lighting) : task.large ? largeEpisodeSetup(seed, task.lighting) : evalEpisodeSetup(task.stage, seed, task.lighting);
  const { spec, cfg } = setup;
  // Real plans fly with the bench's own env seed, not the plans study's.
  const envSeed = run.plan ? seed : setup.envSeed;
  cfg.dynamics = task.dynamics;
  if (task.maxSteps) cfg.maxSteps = task.maxSteps;
  cfg.speedH = task.speedH;
  // v3 flies under the crash rule (shared/constants.ts CRASH_SPEED: a contact faster than 0.5 m/s ends
  // the flight, coverage counts what was flown until then) and prices energy on the current airframe
  // (shared/airframe.ts), the same as every study in the report. v2 did neither (bench/results/v2/).
  // A return flight crosses air it has already flown, so the no-new-voxels stall cut is off.
  if (task.home) cfg.stallSteps = cfg.maxSteps + 1;
  if (task.wind) {
    const a = new RNG(seed, 'bench-wind').next() * Math.PI * 2;
    cfg.wind = [Math.cos(a) * task.wind, Math.sin(a) * task.wind];
  }
  if (task.dynamics === 'quad' && task.dr > 0) {
    const rng = new RNG(seed, 'bench-dr');
    cfg.dyn = sampleDynParams(rng, defaultDynParams(), task.dr);
    cfg.noise = sampleNoise(rng, task.dr, task.lighting !== 'day');
  }
  return { spec, envSeed, cfg };
}

/** The 40 x 40 m tiled building (four stage-3 plans, team/largeBuilding.ts) as a single-drone episode. */
function largeEpisodeSetup(seed: number, lighting: BenchTask['lighting']): { spec: BuildingSpec; envSeed: number; cfg: EpisodeConfig & { maxSteps: number } } {
  const spec = generateLargeBuilding(seed);
  const nRooms = spec.floors.reduce((n, f) => n + f.rooms.length, 0);
  const cfg = defaultEpisodeConfig(3, nRooms);
  Object.assign(cfg, policyEpisodeLimits(3));
  cfg.lighting = lightingPreset(lighting, nRooms);
  return { spec, envSeed: spec.seed, cfg: cfg as EpisodeConfig & { maxSteps: number } };
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
  private readonly cam: { fp: FootprintCoverage; curve: CoverageCurve } | null;
  private info = { coverage: 0, collisions: 0, energyWh: 0, crashed: false };
  private halfS: number | null = null;
  private steps = 0;
  private over = false;
  private start: [number, number, number] = [0, 0, 0];

  constructor(
    private readonly env: ExplorerEnv,
    private readonly task: BenchTask,
    private readonly run: BenchRun,
  ) {
    const { spec, envSeed, cfg } = benchEpisodeSetup(task, run);
    this.ctx = { speedH: task.speedH, maxSteps: cfg.maxSteps, dynamics: task.dynamics, ...(task.home ? { home: true } : {}) };
    this.obs = env.reset(spec, envSeed, cfg);
    const p0 = env.snapshot().pos;
    this.start = [p0[0], p0[1], p0[2]];
    this.fp.start(env, spec, cfg.maxSteps);
    this.cam = task.camera ? { fp: new FootprintCoverage([FOOTPRINT_RADIUS], CAMERA_FOV), curve: new CoverageCurve(cfg.maxSteps) } : null;
    if (this.cam) {
      this.cam.fp.reset(spec);
      this.observeCamera();
    }
  }

  private observeCamera(): void {
    const world = this.env.world;
    if (!this.cam || !world) return;
    const s = this.env.snapshot();
    this.cam.fp.observe(world, s.pos[0], s.pos[1], s.pos[2], s.yaw);
  }

  /** Record one env step; true when the episode is over. */
  record(s: StepResult): boolean {
    if (this.over) throw new Error('bench episode already over');
    this.info = { coverage: s.info.coverage, collisions: s.info.collisions, energyWh: s.info.energyWh, crashed: this.info.crashed || s.info.crashed };
    this.steps += 1;
    if (this.halfS === null && this.info.coverage >= 0.5) this.halfS = this.steps / 20;
    this.fp.step(this.env, this.steps, this.info.coverage);
    if (this.cam) {
      this.observeCamera();
      this.cam.curve.step(this.steps, [this.cam.fp.coverage(0)]);
    }
    this.over = s.done || this.steps >= this.ctx.maxSteps;
    return this.over;
  }

  finish(): BenchEpisode {
    const rec = this.fp.finish(this.info.coverage);
    let camera: BenchEpisode['camera'];
    let returned: boolean | undefined;
    if (this.task.home) {
      const p = this.env.snapshot().pos;
      returned = !this.info.crashed && Math.hypot(p[0] - this.start[0], p[1] - this.start[1]) <= HOME_RADIUS && Math.abs(p[2] - this.start[2]) < 1.5;
    }
    if (this.cam) {
      const v = this.cam.fp.coverage(0);
      this.cam.curve.finish([v]);
      const series = this.cam.curve.series(0);
      camera = { footprint: returned === false ? 0 : v, area: curveArea(series), halfS: timeTo(series, 0.5, this.ctx.maxSteps / 20), ...(returned === undefined ? {} : { seen: v }) };
    }
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
      ...(camera ? { camera } : {}),
      ...(returned === undefined ? {} : { returned }),
    };
  }
}

export function runBenchEpisode(env: ExplorerEnv, task: BenchTask, run: BenchRun, pilot: BenchPilot): BenchEpisode {
  const rec = new BenchEpisodeRecorder(env, task, run);
  pilot.reset(rec.ctx);
  let obs = rec.obs;
  const drift = task.drift ? new DriftFrames(env, run.seed, task.lighting !== 'day') : null;
  let first = true;
  for (;;) {
    const s = env.step(pilot.act(obs, drift ? drift.frame(obs, first) : sensorFrame(env, obs)));
    first = false;
    obs = s.obs;
    if (rec.record(s)) break;
  }
  const ep = rec.finish();
  return drift ? { ...ep, drift: drift.stats() } : ep;
}

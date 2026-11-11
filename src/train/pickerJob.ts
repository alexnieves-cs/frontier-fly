/**
 * Worker job for the quad goal-picker study (scripts/train-picker.ts, docs/QUAD_PICKER.md):
 * fly the sensor planner with braking filter and trajectory tracker (the coverage study's
 * classicalBrakeTrack pilot) on fixed quad episodes, with its goal choice made by a picker.
 * Registered through NodeRolloutPool `jobModules: ['src/train/pickerJob.ts']`.
 */
import { evalEpisodeSetup } from '../eval/coverage';
import { classicalPilot } from '../eval/pilots';
import { RNG } from '../shared/rng';
import { defaultDynParams, sampleDynParams, sampleNoise } from '../sim/dynamics/dr';
import { ExplorerEnv } from '../sim/env';
import { QUAD_BRAKING } from '../sim/lidarSafety';
import type { LightingPreset } from '../sim/lighting';
import { LinearPicker, MlpPicker, RulePicker, type GoalPicker, type MlpWeights, type PickRecorder } from '../sim/picker';
import type { SensorPlannerOptions } from '../sim/sensorPlanner';
import type { JobFn } from './rolloutCore';

export interface PickerEpisode {
  stage: number;
  seed: number;
  lighting: LightingPreset;
  /** Domain-randomization scale of the quadrotor dynamics; 0 = kinematic airframe. */
  quad: number;
}

export type PickerSpec =
  | { kind: 'rule'; planner?: Pick<SensorPlannerOptions, 'edgeBonus' | 'momentum' | 'replanEvery' | 'speed'> }
  | { kind: 'linear'; w: number[]; temperature?: number; planner?: Pick<SensorPlannerOptions, 'replanEvery' | 'speed'> }
  | { kind: 'mlp'; weights: MlpWeights; temperature?: number; record?: boolean; planner?: Pick<SensorPlannerOptions, 'replanEvery' | 'speed'> };

export interface PickerEpisodesPayload {
  /** Episodes to fly, each with its own picker. */
  items: { spec: PickerSpec; episode: PickerEpisode }[];
  /** Seed of the sampling stream when temperature > 0. */
  seed?: number;
}

export interface PickerEpisodeResult extends PickerEpisode {
  coverage: number;
  crashed: boolean;
  collisions: number;
  steps: number;
  picks: number;
  /** With spec.record: per decision, the step it was made at, the features and the chosen index (see PickRecorder). */
  trace?: { steps: Int32Array; coverage: Float32Array; n: Int32Array; feats: Float32Array; chosen: Int32Array; logp: Float32Array };
}

export function episodeSetup(e: PickerEpisode): ReturnType<typeof evalEpisodeSetup> {
  const setup = evalEpisodeSetup(e.stage, e.seed, e.lighting);
  if (e.quad > 0) {
    // As the coverage study's quad condition (scripts/study/coverage-study.ts startEpisode).
    const dr = new RNG(e.seed, 'study-dr');
    setup.cfg.dynamics = 'quad';
    setup.cfg.dyn = sampleDynParams(dr, defaultDynParams(), e.quad);
    setup.cfg.noise = sampleNoise(dr, e.quad, e.lighting !== 'day');
  }
  return setup;
}

export function makePicker(spec: PickerSpec, rng: RNG): GoalPicker | undefined {
  const rand = () => rng.next();
  if (spec.kind === 'rule') return undefined;
  if (spec.kind === 'linear') return new LinearPicker(spec.w, spec.temperature ?? 0, rand);
  return new MlpPicker(spec.weights, spec.temperature ?? 0, rand);
}

let env: ExplorerEnv | null = null;

/** Fly one episode; `picker` undefined = the planner's own rule. */
export function flyEpisode(e: PickerEpisode, spec: PickerSpec, picker: GoalPicker | undefined, recorder?: PickRecorder): PickerEpisodeResult {
  env ??= new ExplorerEnv();
  const setup = episodeSetup(e);
  const p = classicalPilot({ braking: QUAD_BRAKING, tracker: {}, ...(spec.planner ?? {}), ...(picker ? { picker: spec.kind === 'rule' ? new RulePicker() : picker } : {}) });
  p.reset();
  let obs = env.reset(setup.spec, setup.envSeed, setup.cfg);
  let coverage = 0;
  let crashed = false;
  let collisions = 0;
  let steps = 0;
  for (let t = 0; t < setup.cfg.maxSteps; t++) {
    if (recorder) recorder.now(t, coverage);
    const step = env.step(p.act(obs, env));
    obs = step.obs;
    coverage = step.info.coverage;
    collisions = step.info.collisions;
    steps = t + 1;
    if (step.done) {
      crashed = step.info.crashed;
      break;
    }
  }
  const out: PickerEpisodeResult = { ...e, coverage, crashed, collisions, steps, picks: p.planner.picks };
  if (recorder) out.trace = recorder.take();
  return out;
}

const pickerEpisodes: JobFn = (_ctx, raw) => {
  const p = raw as PickerEpisodesPayload;
  const rng = new RNG(p.seed ?? 1, 'picker');
  return p.items.map(({ spec, episode }) => {
    const picker = makePicker(spec, rng);
    const recorder = spec.kind === 'mlp' && spec.record && picker instanceof MlpPicker ? picker.recorder() : undefined;
    return flyEpisode(episode, spec, picker, recorder);
  });
};

export const jobs: Record<string, JobFn> = { pickerEpisodes };

import { generateBuilding } from '../gen/building';
import { ACT_DIM, OBS_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { EpisodeConfig, Policy } from '../shared/types';
import { defaultDynParams, sampleDynParams, zeroNoise } from '../sim/dynamics/dr';
import { createEnv } from '../sim/env';
import { heuristicAction } from '../sim/heuristic';
import { LIGHTING_PRESETS, lightingPreset, type LightingPreset } from '../sim/lighting';

export interface MatrixCell {
  seed: number;
  lighting: LightingPreset;
  dynamics: 'kinematic' | 'quad';
  coverage: number;
  collisions: number;
  steps: number;
}

export interface MatrixReport {
  cells: MatrixCell[];
  nightToDay: number | null;
}

export interface MatrixPolicy {
  act(obs: Float32Array): Float32Array;
}

export function heuristicPolicy(): MatrixPolicy {
  return { act: (obs) => heuristicAction(obs) };
}

export function neuralPolicy(policy: Policy): MatrixPolicy {
  return {
    act(obs: Float32Array) {
      const out = policy.act(obs, 1);
      return out.action;
    },
  };
}

function config(lighting: LightingPreset, dynamics: 'kinematic' | 'quad', nRooms: number, seed: number): EpisodeConfig {
  const rng = new RNG(seed, `${lighting}:${dynamics}`);
  const scale = dynamics === 'quad' ? 0.25 : 0;
  return {
    stage: dynamics === 'quad' ? 5 : 2,
    dynamics,
    dyn: scale === 0 ? defaultDynParams() : sampleDynParams(rng, defaultDynParams(), scale),
    noise: zeroNoise(),
    lighting: lightingPreset(lighting, nRooms),
    wind: [0, 0],
    maxSteps: 100000,
    stallSteps: 100000,
  };
}

/** Headless eval. No renderer is constructed. */
export async function runEvalMatrix(opts: {
  seeds: number[];
  steps?: number;
  policy?: MatrixPolicy;
  dynamics?: Array<'kinematic' | 'quad'>;
}): Promise<MatrixReport> {
  const steps = opts.steps ?? 250;
  const policy = opts.policy ?? heuristicPolicy();
  const dynamicsList = opts.dynamics ?? ['kinematic', 'quad'];
  const env = await createEnv();
  const cells: MatrixCell[] = [];
  for (const seed of opts.seeds) {
    const spec = generateBuilding({ seed, floors: 1, w: 14, d: 14, stage: 2 });
    const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
    for (const dynamics of dynamicsList) {
      for (const lighting of LIGHTING_PRESETS) {
        let obs = env.reset(spec, seed, config(lighting, dynamics, nRooms, seed));
        if (obs.length !== OBS_DIM) throw new Error('obs dim');
        let collisions = 0;
        let coverage = 0;
        for (let t = 0; t < steps; t++) {
          const action = policy.act(obs);
          if (action.length < ACT_DIM) throw new Error('action dim');
          const step = env.step(action);
          obs = step.obs;
          collisions = step.info.collisions;
          coverage = step.info.coverage;
          if (step.done) break;
        }
        cells.push({ seed, lighting, dynamics, coverage, collisions, steps });
      }
    }
  }
  const day = cells.filter((cell) => cell.lighting === 'day' && cell.dynamics === 'kinematic');
  const night = cells.filter((cell) => cell.lighting === 'night-dark' && cell.dynamics === 'kinematic');
  const dayMean = day.reduce((s, c) => s + c.coverage, 0) / Math.max(1, day.length);
  const nightMean = night.reduce((s, c) => s + c.coverage, 0) / Math.max(1, night.length);
  return { cells, nightToDay: dayMean > 0 ? nightMean / dayMean : null };
}

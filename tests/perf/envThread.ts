import { parentPort, workerData } from 'node:worker_threads';
import { generateBuilding } from '../../src/gen/building';
import { OBS_DIM } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import type { BuildingSpec } from '../../src/shared/types';
import { Curriculum } from '../../src/rl/curriculum';
import { InferPolicy } from '../../src/rl/infer';
import { RunningNorm } from '../../src/rl/normalize';
import { createEnv, type ExplorerEnv } from '../../src/sim/env';
import { heuristicAction } from '../../src/sim/heuristic';
import { WorldPool } from '../../src/sim/worldPool';
import { collectRollout } from '../../src/workers/rollout';

const data = workerData as {
  envs: number;
  steps: number;
  seed: number;
  mode?: 'policy' | 'heuristic';
  /** Policy mode: reset onto a WorldPool (what the browser env worker does) instead of rebuilding. */
  pool?: boolean;
  /** Policy mode: untimed collect steps first, so the JIT is warm. */
  warmup?: number;
};
const mode = data.mode ?? 'policy';
const curriculum = new Curriculum();
const envs: ExplorerEnv[] = [];
const specs: BuildingSpec[] = [];
const obs: Float32Array[] = [];
for (let i = 0; i < data.envs; i++) {
  const env = await createEnv();
  const params = curriculum.genParams(0, new RNG(data.seed + i, 'boot'));
  const spec = generateBuilding(params);
  obs.push(env.reset(spec, params.seed));
  envs.push(env);
  specs.push(spec);
}

if (mode === 'heuristic') {
  for (let t = 0; t < 200; t++) {
    for (let i = 0; i < envs.length; i++) {
      const step = envs[i]!.step(heuristicAction(obs[i]!));
      obs[i] = step.done ? envs[i]!.reset(specs[i]!, data.seed + t) : step.obs;
    }
  }
  const t0 = performance.now();
  let steps = 0;
  const target = data.steps * envs.length;
  while (steps < target) {
    for (let i = 0; i < envs.length; i++) {
      const step = envs[i]!.step(heuristicAction(obs[i]!));
      obs[i] = step.obs;
      steps += 1;
      if (step.done) obs[i] = envs[i]!.reset(specs[i]!, data.seed + steps);
    }
  }
  const elapsed = Math.max(1, performance.now() - t0);
  parentPort?.postMessage({ steps, stepsPerSec: (steps * 1000) / elapsed });
} else {
  const policy = new InferPolicy(OBS_DIM, 4, [256, 128], data.seed);
  const norm = new RunningNorm(OBS_DIM);
  const worlds = data.pool ? { pool: await WorldPool.create({ stages: [0], perStage: 8, seed: data.seed }), held: envs.map(() => null) } : undefined;
  const run = (steps: number, baseSeed: number) =>
    collectRollout({ envs, specs, obs, policy, norm, curriculum, steps, baseSeed, stageOverride: 0, worlds });
  if (data.warmup) await run(data.warmup, data.seed + 7);
  const outcome = await run(data.steps, data.seed);
  parentPort?.postMessage({ steps: outcome.buffer.size, stepsPerSec: outcome.stepsPerSec });
}

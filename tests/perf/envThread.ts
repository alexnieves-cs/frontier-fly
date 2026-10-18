import { parentPort, workerData } from 'node:worker_threads';
import { generateBuilding } from '../../src/gen/building';
import { OBS_DIM } from '../../src/shared/constants';
import { RNG } from '../../src/shared/rng';
import { Curriculum } from '../../src/rl/curriculum';
import { InferPolicy } from '../../src/rl/infer';
import { RunningNorm } from '../../src/rl/normalize';
import { createEnv, type ExplorerEnv } from '../../src/sim/env';
import { heuristicAction } from '../../src/sim/heuristic';
import { collectRollout } from '../../src/workers/rollout';

const data = workerData as { envs: number; steps: number; seed: number; mode?: 'policy' | 'heuristic' };
const mode = data.mode ?? 'policy';
const curriculum = new Curriculum();
const envs: ExplorerEnv[] = [];
const specs = [];
const obs = [];
for (let i = 0; i < data.envs; i++) {
  const env = await createEnv();
  const params = curriculum.genParams(0, new RNG(data.seed + i, 'boot'));
  const spec = generateBuilding(params);
  obs.push(env.reset(spec, params.seed));
  envs.push(env);
  specs.push(spec);
}

if (mode === 'heuristic') {
  for (let t = 0; t < 24; t++) {
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
  const outcome = await collectRollout({
    envs,
    specs,
    obs,
    policy,
    norm,
    curriculum,
    steps: data.steps,
    baseSeed: data.seed,
    stageOverride: 0,
  });
  parentPort?.postMessage({ steps: outcome.buffer.size, stepsPerSec: outcome.stepsPerSec });
}

import { generateBuilding } from '../gen/building';
import { ACT_DIM, OBS_DIM } from '../shared/constants';
import type { MetricsPayload } from '../shared/messages';
import { RNG } from '../shared/rng';
import type { BuildingSpec, EpisodeConfig } from '../shared/types';
import { defaultDynParams, sampleDynParams, sampleNoise, zeroNoise } from '../sim/dynamics/dr';
import { ExplorerEnv } from '../sim/env';
import { isLowLight, sampleLighting } from '../sim/lighting';
import { RolloutBuffer } from '../rl/buffer';
import { Curriculum } from '../rl/curriculum';
import { InferPolicy } from '../rl/infer';
import { RunningNorm } from '../rl/normalize';

export interface ShowcaseSnap {
  spec: BuildingSpec;
  pos: [number, number, number];
  vel: [number, number, number];
  yaw: number;
  roll: number;
  pitch: number;
  quat: [number, number, number, number];
  motors: number[];
  coverage: number;
  collisions: number;
  floor: number;
  reward: number;
  step: number;
  lidar: number[];
  visited: Uint8Array;
  lighting: EpisodeConfig['lighting'];
}

export interface RolloutOutcome {
  buffer: RolloutBuffer;
  episodes: { coverage: number; reward: number; length: number; reachedNewFloor: boolean }[];
  stepsPerSec: number;
  showcase: ShowcaseSnap | null;
}

function makeConfig(stage: number, seed: number, nRooms: number, dynamics: 'kinematic' | 'quad'): EpisodeConfig {
  const rng = new RNG(seed, 'cfg');
  const scale = stage <= 2 ? 0 : stage === 3 ? 0.33 : stage === 4 ? 0.66 : 1;
  const lighting = stage >= 5 ? sampleLighting(seed ^ 0x9e37, nRooms, stage) : sampleLighting(seed, nRooms, stage);
  const low = isLowLight(lighting);
  return {
    stage,
    dynamics,
    dyn: sampleDynParams(rng, defaultDynParams(), scale),
    noise: scale === 0 ? zeroNoise() : sampleNoise(rng, scale, low),
    lighting,
    wind: [rng.range(-0.15, 0.15) * scale, rng.range(-0.15, 0.15) * scale],
    maxSteps: stage <= 1 ? 500 : 900,
    stallSteps: 200,
    maxCollisions: stage <= 1 ? 8 : 3,
  };
}

export async function collectRollout(opts: {
  envs: ExplorerEnv[];
  specs: BuildingSpec[];
  obs: Float32Array[];
  policy: InferPolicy;
  norm: RunningNorm;
  curriculum: Curriculum;
  steps: number;
  baseSeed: number;
  stageOverride: number | null;
  onShowcase?: (snap: ShowcaseSnap) => void;
  yieldEvery?: number;
}): Promise<RolloutOutcome> {
  const { envs, policy, norm, curriculum } = opts;
  const n = envs.length;
  const buffer = new RolloutBuffer(opts.steps * n, OBS_DIM, ACT_DIM);
  const episodes: RolloutOutcome['episodes'] = [];
  const returns = new Float64Array(n);
  const lengths = new Int32Array(n);
  const t0 = performance.now();
  let showcase: ShowcaseSnap | null = null;
  const packed = new Float32Array(n * OBS_DIM);

  for (let t = 0; t < opts.steps; t++) {
    for (let i = 0; i < n; i++) packed.set(opts.obs[i]!, i * OBS_DIM);
    norm.update(packed, n);
    const normalized = norm.apply(packed, n);
    const act = policy.act(normalized, n);
    for (let i = 0; i < n; i++) {
      const action = act.action.subarray(i * ACT_DIM, (i + 1) * ACT_DIM);
      const step = envs[i]!.step(action);
      const rawReward = step.reward;
      norm.updateReward(rawReward);
      const reward = norm.normReward(rawReward);
      buffer.add(normalized.subarray(i * OBS_DIM, (i + 1) * OBS_DIM), action, act.logp[i]!, act.value[i]!, reward, step.done && !step.truncated, step.truncated);
      returns[i] += rawReward;
      lengths[i] += 1;
      opts.obs[i] = step.obs;
      const liveShowcase = Boolean(opts.onShowcase) && i === 0 && (t % 2 === 0 || step.done);
      const finalShowcase = i === 0 && t === opts.steps - 1;
      if (liveShowcase || finalShowcase) {
        const snap = envs[0]!.snapshot();
        showcase = {
          spec: opts.specs[0]!,
          pos: [snap.pos[0], snap.pos[1], snap.pos[2]],
          vel: [snap.vel[0], snap.vel[1], snap.vel[2]],
          yaw: snap.yaw,
          roll: snap.roll,
          pitch: snap.pitch,
          quat: [snap.quat[0], snap.quat[1], snap.quat[2], snap.quat[3]],
          motors: Array.from(snap.motors),
          coverage: step.info.coverage,
          collisions: step.info.collisions,
          floor: step.info.floor,
          reward: returns[0] ?? 0,
          step: lengths[0] ?? 0,
          lidar: Array.from(step.obs.slice(0, 40)),
          visited: new Uint8Array(envs[0]!.visitedGrid),
          lighting: {
            timeOfDay: envs[0]!.episodeLighting.timeOfDay,
            interiorLightsOn: new Uint8Array(envs[0]!.episodeLighting.interiorLightsOn),
            flashlightOn: envs[0]!.episodeLighting.flashlightOn,
            sensorMode: envs[0]!.episodeLighting.sensorMode,
          },
        };
        opts.onShowcase?.(showcase);
      }
      if (opts.yieldEvery && i === n - 1 && t % opts.yieldEvery === 0) {
        await new Promise((resolve) => setTimeout(resolve, 0));
      }
      if (step.done) {
        const reached = step.info.floor > 0 || step.info.newFloor;
        episodes.push({
          coverage: step.info.coverage,
          reward: returns[i] ?? 0,
          length: lengths[i] ?? 0,
          reachedNewFloor: reached,
        });
        curriculum.record({ coverage: step.info.coverage, reachedNewFloor: reached });
        const stage = opts.stageOverride ?? curriculum.sampleStage(new RNG(opts.baseSeed + t * 17 + i, 'stage'));
        const params = curriculum.genParams(stage, new RNG(opts.baseSeed + t * 13 + i, 'gen'));
        const spec = generateBuilding(params);
        const dynamics = curriculum.dynamicsFor(stage, new RNG(params.seed, 'dyn'));
        const nRooms = spec.floors.reduce((acc, floor) => acc + floor.rooms.length, 0);
        opts.specs[i] = spec;
        opts.obs[i] = envs[i]!.reset(spec, params.seed, makeConfig(stage, params.seed, nRooms, dynamics));
        returns[i] = 0;
        lengths[i] = 0;
      }
    }
  }
  const elapsed = Math.max(1, performance.now() - t0);
  return { buffer, episodes, stepsPerSec: (opts.steps * n * 1000) / elapsed, showcase };
}

export function metricsFrom(opts: {
  update: number;
  episodes: RolloutOutcome['episodes'];
  stepsPerSec: number;
  stage: number;
  totalSteps: number;
  losses: { policyLoss: number; valueLoss: number; entropy: number; kl: number };
  best: number;
}): MetricsPayload {
  const ep = opts.episodes;
  const mean = (pick: (e: (typeof ep)[number]) => number) =>
    ep.length === 0 ? 0 : ep.reduce((s, e) => s + pick(e), 0) / ep.length;
  const rewardMean = mean((e) => e.reward);
  return {
    update: opts.update,
    episode: opts.totalSteps,
    episodeReturn: rewardMean,
    episodeLength: mean((e) => e.length),
    coverage: mean((e) => e.coverage),
    entropy: opts.losses.entropy,
    kl: opts.losses.kl,
    valueLoss: opts.losses.valueLoss,
    policyLoss: opts.losses.policyLoss,
    stepsPerSec: opts.stepsPerSec,
    stage: opts.stage,
    successRate: ep.length === 0 ? 0 : ep.filter((e) => e.coverage >= 0.95).length / ep.length,
    rewardMean,
    bestReward: Math.max(opts.best, ...ep.map((e) => e.reward)),
    totalSteps: opts.totalSteps,
  };
}

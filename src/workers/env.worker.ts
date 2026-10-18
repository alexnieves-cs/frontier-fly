/// <reference lib="webworker" />
import { generateBuilding } from '../gen/building';
import { OBS_DIM } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { RunningNormState } from '../rl/normalize';
import { Curriculum } from '../rl/curriculum';
import { InferPolicy } from '../rl/infer';
import { RunningNorm } from '../rl/normalize';
import { defaultDynParams, zeroNoise } from '../sim/dynamics/dr';
import { createEnv, type ExplorerEnv } from '../sim/env';
import { collectRollout, type ShowcaseSnap } from './rollout';
import { rolloutViews, writeRollout, type RolloutViews } from './sab';

interface StartMsg {
  type: 'start';
  nEnvs: number;
  seed: number;
  steps: number;
  sab?: SharedArrayBuffer;
  capacity?: number;
  yieldEvery?: number;
}
interface WeightsMsg {
  type: 'weights';
  weights: ArrayBuffer[];
  norm: RunningNormState;
}
interface StageMsg { type: 'setStage'; stage: number | null }
interface HeatMsg { type: 'heatmap'; id: number; floor: number }
interface LightMsg {
  type: 'lighting';
  timeOfDay?: number;
  flashlightOn?: boolean;
  sensorMode?: 'rgb' | 'ir' | 'depth';
  lightsOn?: boolean;
}
type InMsg = StartMsg | WeightsMsg | StageMsg | HeatMsg | LightMsg | { type: 'pause' } | { type: 'resume' } | { type: 'stop' };

let envs: ExplorerEnv[] = [];
const specs = [] as ReturnType<typeof generateBuilding>[];
const obs: Float32Array[] = [];
let policy = new InferPolicy(OBS_DIM, 4, [256, 128], 1);
let norm = new RunningNorm(OBS_DIM);
let curriculum = new Curriculum();
let running = false;
let seed = 1;
let steps = 256;
let stageOverride: number | null = null;
let tick = 0;
let waitWeights: (() => void) | null = null;
let sabView: RolloutViews | null = null;
let yieldEvery = 0;
let booted = false;
let started = false;
let pendingWeights: WeightsMsg | null = null;

async function setup(nEnvs: number, baseSeed: number): Promise<void> {
  envs = [];
  specs.length = 0;
  obs.length = 0;
  curriculum = new Curriculum();
  policy = new InferPolicy(OBS_DIM, 4, [256, 128], baseSeed);
  norm = new RunningNorm(OBS_DIM);
  for (let i = 0; i < nEnvs; i++) {
    const env = await createEnv();
    const stage = 0;
    const params = curriculum.genParams(stage, new RNG(baseSeed + i * 17, 'boot'));
    const spec = generateBuilding(params);
    const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
    const lighting = { timeOfDay: 12, interiorLightsOn: new Uint8Array(nRooms).fill(1), flashlightOn: false, sensorMode: 'rgb' as const };
    const first = env.reset(spec, params.seed, {
      stage,
      dynamics: 'kinematic',
      dyn: defaultDynParams(),
      noise: zeroNoise(),
      lighting,
      wind: [0, 0],
      maxSteps: 500,
      stallSteps: 200,
    });
    envs.push(env);
    specs.push(spec);
    obs.push(first);
  }
}

async function loop(): Promise<void> {
  while (running) {
    const outcome = await collectRollout({
      envs,
      specs,
      obs,
      policy,
      norm,
      curriculum,
      steps,
      baseSeed: seed + tick * 1009,
      stageOverride,
      yieldEvery,
      onShowcase: yieldEvery
        ? (snap) => {
            (self as DedicatedWorkerGlobalScope).postMessage({ type: 'showcase', showcase: snap });
          }
        : undefined,
    });
    tick += 1;
    const arrays = outcome.buffer.toArrays();
    const snap: ShowcaseSnap | null = outcome.showcase;
    const scope = self as DedicatedWorkerGlobalScope;
    if (sabView) {
      writeRollout(sabView, arrays);
      scope.postMessage({
        type: 'rollout',
        shared: true,
        episodes: outcome.episodes,
        stepsPerSec: outcome.stepsPerSec,
        stage: curriculum.stage,
        showcase: snap,
        norm: norm.toJSON(),
      });
    } else {
      scope.postMessage(
        {
          type: 'rollout',
          arrays,
          episodes: outcome.episodes,
          stepsPerSec: outcome.stepsPerSec,
          stage: curriculum.stage,
          showcase: snap,
          norm: norm.toJSON(),
        },
        { transfer: [arrays.obs.buffer, arrays.actions.buffer, arrays.logp.buffer, arrays.values.buffer, arrays.rewards.buffer, arrays.dones.buffer, arrays.truncated.buffer] },
      );
    }
    await new Promise<void>((resolve) => {
      waitWeights = resolve;
    });
  }
}

function applyWeights(msg: WeightsMsg): void {
  policy.setWeights(msg.weights);
  norm.load(msg.norm);
  if (!started) {
    started = true;
    running = true;
    void loop();
    return;
  }
  waitWeights?.();
  waitWeights = null;
}

self.onmessage = (event: MessageEvent<InMsg>) => {
  const msg = event.data;
  if (msg.type === 'start') {
    seed = msg.seed;
    steps = msg.steps;
    yieldEvery = msg.yieldEvery ?? 0;
    if (msg.sab && msg.capacity) sabView = rolloutViews(msg.sab, msg.capacity);
    void setup(msg.nEnvs, msg.seed).then(() => {
      booted = true;
      self.postMessage({ type: 'ready', weights: policy.getWeights(), norm: norm.toJSON() });
      if (pendingWeights) {
        const queued = pendingWeights;
        pendingWeights = null;
        applyWeights(queued);
      }
    });
  } else if (msg.type === 'weights') {
    if (!booted) {
      pendingWeights = msg;
      return;
    }
    applyWeights(msg);
  } else if (msg.type === 'pause') {
    running = false;
  } else if (msg.type === 'resume') {
    if (!running) {
      running = true;
      void loop();
    }
  } else if (msg.type === 'stop') {
    running = false;
  } else if (msg.type === 'setStage') {
    stageOverride = msg.stage;
  } else if (msg.type === 'lighting') {
    envs[0]?.setLighting(msg);
  } else if (msg.type === 'heatmap') {
    const env = envs[0];
    const spec = env?.building;
    if (!env || !spec) return;
    const floor = spec.floors[Math.max(0, Math.min(spec.floors.length - 1, msg.floor))]!;
    const values = new Float32Array(floor.w * floor.d);
    const arrows = new Float32Array(floor.w * floor.d * 2);
    values.fill(Number.NaN);
    for (let z = 0; z < floor.d; z++) {
      for (let x = 0; x < floor.w; x++) {
        const cell = floor.cells[z * floor.w + x]!;
        const i = z * floor.w + x;
        if (cell === 1) continue;
        const o = env.observeAt((x + 0.5) * spec.cellSize, (z + 0.5) * spec.cellSize, msg.floor * spec.floorHeight + 1, 0);
        const nobs = norm.apply(o, 1);
        const act = policy.actWith(nobs, 1, new RNG(1, 'heat'), true);
        values[i] = act.value[0] ?? 0;
        arrows[i * 2] = act.action[0] ?? 0;
        arrows[i * 2 + 1] = act.action[1] ?? 0;
      }
    }
    self.postMessage({ type: 'heatmap', id: msg.id, floor: msg.floor, w: floor.w, d: floor.d, values, arrows }, { transfer: [values.buffer, arrows.buffer] });
  }
};

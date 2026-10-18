import type { BuildingSpec, CameraMode, DroneState, EpisodeConfig, LightingState } from './types';

export interface MetricsPayload {
  update: number;
  episode: number;
  episodeReturn: number;
  episodeLength: number;
  coverage: number;
  entropy: number;
  kl: number;
  valueLoss: number;
  policyLoss: number;
  stepsPerSec: number;
  stage: number;
  successRate: number;
  rewardMean: number;
  bestReward: number;
  totalSteps: number;
}

export interface ShowcasePayload {
  state: DroneState;
  lighting: LightingState;
  spec: BuildingSpec;
  coverage: number;
  collisions: number;
  lidar: number[];
  trail: Array<[number, number, number]>;
  visited: Uint8Array;
  floor: number;
  reward: number;
  step: number;
}

export interface HeatmapResult {
  id: number;
  floor: number;
  w: number;
  d: number;
  values: Float32Array;
  actions: Float32Array;
}

export type WorkerMessage =
  | { type: 'reset'; spec: BuildingSpec; seed: number; cfg: EpisodeConfig }
  | { type: 'step'; action: Float32Array }
  | { type: 'weights'; weights: ArrayBuffer[]; obsMean: Float32Array; obsVar: Float32Array }
  | { type: 'metrics'; metrics: MetricsPayload }
  | { type: 'showcase'; showcase: ShowcasePayload }
  | { type: 'checkpoint'; data: ArrayBuffer }
  | { type: 'heatmapReq'; id: number; floor: number; resolution: number }
  | { type: 'heatmapRes'; heatmap: HeatmapResult }
  | { type: 'rollout'; steps: number; stage: number; seed: number; dynamics: EpisodeConfig['dynamics'] }
  | { type: 'rolloutResult'; payload: ArrayBuffer; meta: RolloutMeta }
  | { type: 'ready' }
  | { type: 'pause' }
  | { type: 'resume' }
  | { type: 'setStage'; stage: number }
  | { type: 'setSeed'; seed: number }
  | { type: 'error'; message: string };

export interface RolloutMeta {
  steps: number;
  nEnvs: number;
  obsDim: number;
  actDim: number;
  episodes: number;
  meanCoverage: number;
  meanReturn: number;
  stage: number;
  elapsedMs: number;
}

export interface UiCommand {
  camera?: CameraMode;
  paused?: boolean;
  seed?: number;
  stage?: number | 'auto';
  speed?: 'realtime' | 'max';
}

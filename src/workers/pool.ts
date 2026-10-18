import { OBS_DIM } from '../shared/constants';
import type { MetricsPayload } from '../shared/messages';
import type { RunningNormState } from '../rl/normalize';
import type { ShowcaseSnap } from './rollout';

export interface PoolHandlers {
  onReady: () => void;
  onShowcase: (snap: ShowcaseSnap) => void;
  onMetrics: (metrics: MetricsPayload) => void;
  onHeatmap: (msg: { id: number; floor: number; w: number; d: number; values: Float32Array; arrows: Float32Array }) => void;
}

export class TrainPool {
  private env: Worker | null = null;
  private learner: Worker | null = null;
  private learnerReady = false;
  private pending: { arrays: unknown; episodes: { coverage: number; reward: number; length: number }[]; stepsPerSec: number; stage: number; norm: RunningNormState; sent: boolean } | null = null;
  private totalSteps = 0;
  private best = -Infinity;
  private update = 0;
  readonly stepsPerSec = { value: 0 };

  constructor(private readonly handlers: PoolHandlers) {}

  start(seed: number, nEnvs: number): void {
    this.stop();
    this.env = new Worker(new URL('./env.worker.ts', import.meta.url), { type: 'module' });
    this.learner = new Worker(new URL('./learner.worker.ts', import.meta.url), { type: 'module' });
    this.env.onmessage = (event: MessageEvent) => {
      const msg = event.data as {
        type: string;
        weights?: ArrayBuffer[];
        norm?: RunningNormState;
        arrays?: {
          obs: Float32Array;
          actions: Float32Array;
          logp: Float32Array;
          values: Float32Array;
          rewards: Float32Array;
          dones: Uint8Array;
          truncated: Uint8Array;
          n: number;
        };
        episodes?: { coverage: number; reward: number; length: number; reachedNewFloor: boolean }[];
        stepsPerSec?: number;
        stage?: number;
        showcase?: ShowcaseSnap;
        id?: number;
        floor?: number;
        w?: number;
        d?: number;
        values?: Float32Array;
        arrows?: Float32Array;
      };
      if (msg.type === 'ready' && msg.weights && msg.norm) {
        this.learner?.postMessage({ type: 'boot', weights: msg.weights }, msg.weights);
        this.handlers.onReady();
      } else if (msg.type === 'showcase' && msg.showcase) {
        this.handlers.onShowcase(msg.showcase);
      } else if (msg.type === 'rollout' && msg.arrays && msg.episodes && msg.norm) {
        this.stepsPerSec.value = msg.stepsPerSec ?? 0;
        this.totalSteps += msg.arrays.n;
        if (msg.showcase) this.handlers.onShowcase(msg.showcase);
        if (this.pending && !this.pending.sent) return;
        this.pending = {
          arrays: msg.arrays,
          episodes: msg.episodes,
          stepsPerSec: msg.stepsPerSec ?? 0,
          stage: msg.stage ?? 0,
          norm: msg.norm,
          sent: false,
        };
        this.flush();
      } else if (msg.type === 'heatmap' && msg.values && msg.arrows) {
        this.handlers.onHeatmap({
          id: msg.id ?? 0,
          floor: msg.floor ?? 0,
          w: msg.w ?? 0,
          d: msg.d ?? 0,
          values: msg.values,
          arrows: msg.arrows,
        });
      }
    };
    this.learner.onmessage = (event: MessageEvent) => {
      const msg = event.data as {
        type: string;
        weights?: ArrayBuffer[];
        losses?: { policyLoss: number; valueLoss: number; entropy: number; kl: number };
        norm?: RunningNormState;
      };
      if (msg.type === 'learner-ready') {
        this.learnerReady = true;
        this.flush();
      } else if (msg.type === 'updated' && msg.weights && msg.losses && msg.norm && this.pending) {
        const episodes = this.pending.episodes;
        const rewardMean = episodes.length ? episodes.reduce((s, e) => s + e.reward, 0) / episodes.length : 0;
        this.best = Math.max(this.best, ...episodes.map((e) => e.reward));
        this.update += 1;
        const metrics: MetricsPayload = {
          update: this.update,
          episode: this.totalSteps,
          episodeReturn: rewardMean,
          episodeLength: episodes.length ? episodes.reduce((s, e) => s + e.length, 0) / episodes.length : 0,
          coverage: episodes.length ? episodes.reduce((s, e) => s + e.coverage, 0) / episodes.length : 0,
          entropy: msg.losses.entropy,
          kl: msg.losses.kl,
          valueLoss: msg.losses.valueLoss,
          policyLoss: msg.losses.policyLoss,
          stepsPerSec: this.pending.stepsPerSec,
          stage: this.pending.stage,
          successRate: episodes.length ? episodes.filter((e) => e.coverage >= 0.95).length / episodes.length : 0,
          rewardMean,
          bestReward: this.best,
          totalSteps: this.totalSteps,
        };
        this.handlers.onMetrics(metrics);
        this.env?.postMessage({ type: 'weights', weights: msg.weights, norm: msg.norm }, msg.weights);
        this.pending = null;
      }
    };
    this.env.postMessage({ type: 'start', nEnvs, seed, steps: 128 });
    void OBS_DIM;
  }

  pause(): void {
    this.env?.postMessage({ type: 'pause' });
  }

  resume(): void {
    this.env?.postMessage({ type: 'resume' });
  }

  setStage(stage: number | null): void {
    this.env?.postMessage({ type: 'setStage', stage });
  }

  requestHeatmap(floor: number): void {
    this.env?.postMessage({ type: 'heatmap', id: Date.now(), floor });
  }

  setLighting(partial: { timeOfDay?: number; flashlightOn?: boolean; sensorMode?: 'rgb' | 'ir' | 'depth'; lightsOn?: boolean }): void {
    this.env?.postMessage({ type: 'lighting', ...partial });
  }

  stop(): void {
    this.env?.postMessage({ type: 'stop' });
    this.env?.terminate();
    this.learner?.terminate();
    this.env = null;
    this.learner = null;
    this.learnerReady = false;
    this.pending = null;
  }

  private flush(): void {
    if (!this.learnerReady || !this.pending || this.pending.sent || !this.learner) return;
    this.pending.sent = true;
    const arrays = this.pending.arrays as {
      obs: Float32Array;
      actions: Float32Array;
      logp: Float32Array;
      values: Float32Array;
      rewards: Float32Array;
      dones: Uint8Array;
      truncated: Uint8Array;
    };
    const transfer = [arrays.obs.buffer, arrays.actions.buffer, arrays.logp.buffer, arrays.values.buffer, arrays.rewards.buffer, arrays.dones.buffer, arrays.truncated.buffer];
    this.learner.postMessage({ type: 'rollout', arrays: this.pending.arrays, norm: this.pending.norm }, transfer);
  }
}

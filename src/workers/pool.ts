import type { MetricsPayload } from '../shared/messages';
import type { RunningNormState } from '../rl/normalize';
import { createRolloutSab, mergeRollouts, readRollout, rolloutViews, type RolloutArraysLike, type RolloutViews } from './sab';
import type { ShowcaseSnap } from './rollout';

export interface PoolHandlers {
  onReady: () => void;
  onShowcase: (snap: ShowcaseSnap) => void;
  onMetrics: (metrics: MetricsPayload) => void;
  onHeatmap: (msg: { id: number; floor: number; w: number; d: number; values: Float32Array; arrows: Float32Array }) => void;
}

interface PendingRollout {
  arrays: RolloutArraysLike;
  episodes: { coverage: number; reward: number; length: number; reachedNewFloor?: boolean }[];
  stepsPerSec: number;
  stage: number;
  norm: RunningNormState;
  sent: boolean;
}

/**
 * One learner and up to 4 env workers. Each worker writes its rollout into a
 * SharedArrayBuffer the main thread reads without cloning the payload.
 */
export class TrainPool {
  private envs: Worker[] = [];
  private slots: RolloutViews[] = [];
  private learner: Worker | null = null;
  private learnerReady = false;
  private pending: PendingRollout | null = null;
  private inbox: Array<PendingRollout | null> = [];
  private bootWeights: ArrayBuffer[] | null = null;
  private bootNorm: RunningNormState | null = null;
  private readyCount = 0;
  private totalSteps = 0;
  private best = -Infinity;
  private update = 0;
  readonly stepsPerSec = { value: 0 };
  readonly workers = { value: 0 };
  /** Envs per worker and rows per worker block, for per-env GAE in the learner. */
  private layout = { envs: 1, block: 1 };

  constructor(private readonly handlers: PoolHandlers) {}

  /** Eight envs per worker so each policy forward pass is a batch of eight. */
  start(seed: number, envsPerWorker = 8): void {
    this.stop();
    const cores = typeof navigator === 'undefined' ? 4 : navigator.hardwareConcurrency || 4;
    const nWorkers = Math.max(1, Math.min(4, cores));
    // 8 envs x 32 steps keeps the learner's batch at 256 rows per worker, as with the old 2 x 128.
    const steps = 32;
    const capacity = Math.max(1, envsPerWorker) * steps;
    this.workers.value = nWorkers;
    this.layout = { envs: Math.max(1, envsPerWorker), block: capacity };
    this.learner = new Worker(new URL('./learner.worker.ts', import.meta.url), { type: 'module' });
    this.learner.onmessage = (event: MessageEvent) => this.onLearner(event);
    for (let i = 0; i < nWorkers; i++) {
      const sab = createRolloutSab(capacity);
      const view = rolloutViews(sab, capacity);
      const worker = new Worker(new URL('./env.worker.ts', import.meta.url), { type: 'module' });
      this.slots.push(view);
      this.envs.push(worker);
      this.inbox.push(null);
      worker.onmessage = (event: MessageEvent) => this.onEnv(i, event);
      worker.postMessage({
        type: 'start',
        nEnvs: envsPerWorker,
        seed: seed + i * 10007,
        steps,
        sab,
        capacity,
        yieldEvery: i === 0 ? 8 : 0,
      });
    }
  }

  pause(): void {
    for (const worker of this.envs) worker.postMessage({ type: 'pause' });
  }

  resume(): void {
    for (const worker of this.envs) worker.postMessage({ type: 'resume' });
  }

  setStage(stage: number | null): void {
    for (const worker of this.envs) worker.postMessage({ type: 'setStage', stage });
  }

  requestHeatmap(floor: number): void {
    this.envs[0]?.postMessage({ type: 'heatmap', id: Date.now(), floor });
  }

  setLighting(partial: { timeOfDay?: number; flashlightOn?: boolean; sensorMode?: 'rgb' | 'ir' | 'depth'; lightsOn?: boolean }): void {
    this.envs[0]?.postMessage({ type: 'lighting', ...partial });
  }

  stop(): void {
    for (const worker of this.envs) {
      worker.postMessage({ type: 'stop' });
      worker.terminate();
    }
    this.learner?.terminate();
    this.envs = [];
    this.slots = [];
    this.inbox = [];
    this.learner = null;
    this.learnerReady = false;
    this.pending = null;
    this.bootWeights = null;
    this.bootNorm = null;
    this.readyCount = 0;
  }

  private onEnv(index: number, event: MessageEvent): void {
    const msg = event.data as {
      type: string;
      shared?: boolean;
      weights?: ArrayBuffer[];
      norm?: RunningNormState;
      arrays?: RolloutArraysLike;
      episodes?: PendingRollout['episodes'];
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
      this.readyCount += 1;
      if (!this.bootWeights) {
        this.bootWeights = msg.weights.map((buf) => buf.slice(0));
        this.bootNorm = msg.norm;
        const boot = this.bootWeights.map((buf) => buf.slice(0));
        this.learner?.postMessage({ type: 'boot', weights: boot }, boot);
        this.handlers.onReady();
      }
      return;
    }
    if (msg.type === 'showcase' && msg.showcase) {
      this.handlers.onShowcase(msg.showcase);
      return;
    }
    if (msg.type === 'rollout' && msg.episodes && msg.norm) {
      const arrays = msg.shared ? readRollout(this.slots[index]!) : msg.arrays;
      if (!arrays) return;
      this.inbox[index] = {
        arrays,
        episodes: msg.episodes,
        stepsPerSec: msg.stepsPerSec ?? 0,
        stage: msg.stage ?? 0,
        norm: msg.norm,
        sent: false,
      };
      if (msg.showcase) this.handlers.onShowcase(msg.showcase);
      if (this.inbox.every((slot) => slot)) this.assemble();
      return;
    }
    if (msg.type === 'heatmap' && msg.values && msg.arrows) {
      this.handlers.onHeatmap({
        id: msg.id ?? 0,
        floor: msg.floor ?? 0,
        w: msg.w ?? 0,
        d: msg.d ?? 0,
        values: msg.values,
        arrows: msg.arrows,
      });
    }
  }

  private assemble(): void {
    const parts = this.inbox.map((slot) => slot!);
    this.inbox = this.inbox.map(() => null);
    const stepsPerSec = parts.reduce((sum, part) => sum + part.stepsPerSec, 0);
    this.stepsPerSec.value = stepsPerSec;
    const episodes = parts.flatMap((part) => part.episodes);
    const arrays = mergeRollouts(parts.map((part) => part.arrays));
    this.totalSteps += arrays.n;
    if (this.pending && !this.pending.sent) return;
    this.pending = {
      arrays,
      episodes,
      stepsPerSec,
      stage: parts[0]?.stage ?? 0,
      norm: parts[0]?.norm ?? this.bootNorm!,
      sent: false,
    };
    this.flush();
  }

  private onLearner(event: MessageEvent): void {
    const msg = event.data as {
      type: string;
      weights?: ArrayBuffer[];
      losses?: { policyLoss: number; valueLoss: number; entropy: number; kl: number };
      norm?: RunningNormState;
    };
    if (msg.type === 'learner-ready' && this.bootWeights && this.bootNorm) {
      this.learnerReady = true;
      this.broadcast(this.bootWeights, this.bootNorm);
      this.flush();
      return;
    }
    if (msg.type === 'updated' && msg.weights && msg.losses && msg.norm && this.pending) {
      const episodes = this.pending.episodes;
      const rewardMean = episodes.length ? episodes.reduce((sum, episode) => sum + episode.reward, 0) / episodes.length : 0;
      this.best = Math.max(this.best, ...episodes.map((episode) => episode.reward), 0);
      this.update += 1;
      const metrics: MetricsPayload = {
        update: this.update,
        episode: this.totalSteps,
        episodeReturn: rewardMean,
        episodeLength: episodes.length ? episodes.reduce((sum, episode) => sum + episode.length, 0) / episodes.length : 0,
        coverage: episodes.length ? episodes.reduce((sum, episode) => sum + episode.coverage, 0) / episodes.length : 0,
        entropy: msg.losses.entropy,
        kl: msg.losses.kl,
        valueLoss: msg.losses.valueLoss,
        policyLoss: msg.losses.policyLoss,
        stepsPerSec: this.pending.stepsPerSec,
        stage: this.pending.stage,
        successRate: episodes.length ? episodes.filter((episode) => episode.coverage >= 0.95).length / episodes.length : 0,
        rewardMean,
        bestReward: this.best,
        totalSteps: this.totalSteps,
      };
      this.handlers.onMetrics(metrics);
      this.broadcast(msg.weights, msg.norm);
      this.pending = null;
    }
  }

  private broadcast(weights: ArrayBuffer[], norm: RunningNormState): void {
    for (const worker of this.envs) {
      const copies = weights.map((buf) => buf.slice(0));
      worker.postMessage({ type: 'weights', weights: copies, norm }, copies);
    }
  }

  private flush(): void {
    if (!this.learnerReady || !this.pending || this.pending.sent || !this.learner) return;
    this.pending.sent = true;
    const arrays = this.pending.arrays;
    const transfer = [
      arrays.obs.buffer,
      arrays.actions.buffer,
      arrays.logp.buffer,
      arrays.values.buffer,
      arrays.rewards.buffer,
      arrays.dones.buffer,
      arrays.truncated.buffer,
    ];
    this.learner.postMessage({ type: 'rollout', arrays, norm: this.pending.norm, ...this.layout }, transfer);
  }
}

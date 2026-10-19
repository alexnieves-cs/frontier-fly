/**
 * Adapter that runs a DAgger collector (workstream (a): src/rl/dagger.ts
 * `collectDagger`) inside NodeRolloutPool workers. NodeRolloutPool registers
 * it automatically as job "dagger" when src/rl/dagger.ts exports a
 * `collectDagger` function; otherwise register it from a job module:
 *
 *   // my-jobs.ts, passed as NodeRolloutPool({ jobModules: ['my-jobs.ts'] })
 *   import { collectDagger } from '../rl/dagger';
 *   import { makeDaggerJob } from './daggerJob';
 *   export const jobs = { dagger: makeDaggerJob(collectDagger) };
 */
import { Curriculum } from '../rl/curriculum';
import type { InferPolicy } from '../rl/infer';
import { RNG } from '../shared/rng';
import type { BuildingSpec, EpisodeConfig } from '../shared/types';
import { ExplorerEnv } from '../sim/env';
import { lightingPreset, type LightingPreset } from '../sim/lighting';
import type { PooledWorld } from '../sim/worldPool';
import { makeConfig } from '../workers/rollout';
import type { JobContext, JobFn } from './rolloutCore';

/** The collector contract the adapter calls. Extra fields are passed through. */
export interface CollectDaggerArgs {
  env: ExplorerEnv;
  spec: BuildingSpec;
  cfg: EpisodeConfig;
  policy: InferPolicy;
  beta: number;
  steps: number;
  rng: RNG;
  vzDisturbance?: number;
}

/** Returns Float32Arrays (obs, teacher actions, weights, ...) plus any scalars. */
export type CollectDaggerFn = (args: CollectDaggerArgs) => object | Promise<object>;

export interface DaggerEpisode {
  stage: number;
  /** Mixing probability passed through to the collector. */
  beta: number;
  steps: number;
  /** Fixed building (GenParams drawn from RNG(seed, 'dagger-<stage>')); otherwise a pooled training building. */
  seed?: number;
  /** A named preset, or the stage-sampled training lighting (default). */
  lighting?: LightingPreset | 'sampled';
  dynamics?: 'kinematic' | 'quad';
  episode?: { maxSteps?: number; stallSteps?: number; maxCollisions?: number };
  /** See DaggerCollectOpts.vzDisturbance. */
  vzDisturbance?: number;
}

export interface DaggerPayload {
  episodes: DaggerEpisode[];
  /** Seeds this call's RNG (worker index mixed in). */
  seed?: number;
}

export interface DaggerJobResult {
  /** Float32Array fields of every episode, concatenated in episode order (e.g. obs, actions, weights). */
  arrays: Record<string, Float32Array>;
  /** Per-episode length of each concatenated field, in elements. */
  lengths: Record<string, number[]>;
  /** Non-array fields of each episode's result (coverage, n, ...). */
  extras: Record<string, unknown>[];
}

let daggerEnv: ExplorerEnv | null = null;

function concat(parts: Float32Array[]): Float32Array {
  const out = new Float32Array(parts.reduce((n, p) => n + p.length, 0));
  let at = 0;
  for (const p of parts) {
    out.set(p, at);
    at += p.length;
  }
  return out;
}

export function makeDaggerJob(collect: CollectDaggerFn): JobFn {
  return async (ctx: JobContext, raw: unknown): Promise<DaggerJobResult> => {
    const payload = raw as DaggerPayload;
    const rng = new RNG(((payload.seed ?? 1) >>> 0) ^ Math.imul(ctx.workerIndex + 1, 0x85ebca6b), `dagger-${ctx.workerIndex}`);
    // A dedicated env, so a DAgger call never disturbs the PPO envs mid-episode.
    daggerEnv ??= new ExplorerEnv();
    const env = daggerEnv;
    const curriculum = new Curriculum();
    const parts = new Map<string, Float32Array[]>();
    const extras: Record<string, unknown>[] = [];
    for (const ep of payload.episodes) {
      const world: PooledWorld =
        ep.seed != null
          ? ctx.worlds.bySeed(curriculum.genParams(ep.stage, new RNG(ep.seed, `dagger-${ep.stage}`)))
          : ctx.worlds.acquire(ep.stage, rng);
      const spec = world.spec;
      const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
      const envSeed = rng.int(1_000_000_000);
      const dynamics = ep.dynamics ?? 'kinematic';
      const cfg = makeConfig(ep.stage, envSeed, nRooms, dynamics);
      if (ep.lighting && ep.lighting !== 'sampled') cfg.lighting = lightingPreset(ep.lighting, nRooms);
      if (ep.episode?.maxSteps != null) cfg.maxSteps = ep.episode.maxSteps;
      if (ep.episode?.stallSteps != null) cfg.stallSteps = ep.episode.stallSteps;
      if (ep.episode?.maxCollisions != null) cfg.maxCollisions = ep.episode.maxCollisions;
      // The collector resets the env itself; hand it the pooled physics for this spec.
      const proto = ExplorerEnv.prototype.reset;
      env.reset = (s, seed, c, physics) => proto.call(env, s, seed, c, physics ?? (s === spec ? world.physics : undefined));
      let result: object;
      try {
        result = await collect({ env, spec, cfg, policy: ctx.policy, beta: ep.beta, steps: ep.steps, rng: rng.fork('episode'), vzDisturbance: ep.vzDisturbance });
      } finally {
        delete (env as { reset?: unknown }).reset;
        // Detach before the world may be freed (churn / LRU), then let the pool reclaim it.
        env.dispose();
        ctx.worlds.release(world);
      }
      const extra: Record<string, unknown> = {};
      for (const [key, value] of Object.entries(result)) {
        if (value instanceof Float32Array) {
          if (!parts.has(key)) parts.set(key, []);
          parts.get(key)!.push(value);
        } else {
          extra[key] = value;
        }
      }
      extras.push(extra);
    }
    const arrays: Record<string, Float32Array> = {};
    const lengths: Record<string, number[]> = {};
    for (const [key, list] of parts) {
      arrays[key] = concat(list);
      lengths[key] = list.map((p) => p.length);
    }
    return { arrays, lengths, extras };
  };
}

/** Merge DaggerJobResults from several workers (runJobAll) into one set of arrays. */
export function mergeDaggerResults(results: DaggerJobResult[]): DaggerJobResult {
  const arrays: Record<string, Float32Array> = {};
  const lengths: Record<string, number[]> = {};
  const keys = new Set(results.flatMap((r) => Object.keys(r.arrays)));
  for (const key of keys) {
    arrays[key] = concat(results.map((r) => r.arrays[key] ?? new Float32Array(0)));
    lengths[key] = results.flatMap((r) => r.lengths[key] ?? []);
  }
  return { arrays, lengths, extras: results.flatMap((r) => r.extras) };
}

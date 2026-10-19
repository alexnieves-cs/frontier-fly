/**
 * Deterministic env traces for the bit-identity tests. Only public env API is
 * used so the same file can be run against an older checkout of src/ to
 * record golden hashes (see tests/unit/determinism.test.ts).
 */
import { generateBuilding } from '../../src/gen/building';
import { Curriculum } from '../../src/rl/curriculum';
import { RNG } from '../../src/shared/rng';
import type { BuildingSpec, EpisodeConfig } from '../../src/shared/types';
import { defaultDynParams, sampleDynParams, sampleNoise } from '../../src/sim/dynamics/dr';
import { defaultEpisodeConfig, type ExplorerEnv } from '../../src/sim/env';
import { sampleLighting } from '../../src/sim/lighting';
import type { PhysicsWorld } from '../../src/sim/world';

export interface TraceCase {
  name: string;
  stage: number;
  seed: number;
  dynamics: 'kinematic' | 'quad';
  noiseScale: number;
  latencyTicks?: number;
  steps: number;
}

export const TRACE_CASES: TraceCase[] = [
  { name: 'stage0-kinematic', stage: 0, seed: 11, dynamics: 'kinematic', noiseScale: 0, steps: 700 },
  { name: 'stage2-kinematic', stage: 2, seed: 23, dynamics: 'kinematic', noiseScale: 0, steps: 700 },
  { name: 'stage3-noise-latency', stage: 3, seed: 37, dynamics: 'kinematic', noiseScale: 0.66, latencyTicks: 2, steps: 600 },
  { name: 'stage4-multifloor', stage: 4, seed: 41, dynamics: 'kinematic', noiseScale: 0.33, steps: 600 },
  { name: 'stage4-quad', stage: 4, seed: 53, dynamics: 'quad', noiseScale: 0.66, steps: 200 },
];

export function traceSpec(c: TraceCase, episode: number): BuildingSpec {
  return generateBuilding(new Curriculum().genParams(c.stage, new RNG(c.seed + episode * 7919, 'trace')));
}

export function traceConfig(c: TraceCase, spec: BuildingSpec, episode: number): EpisodeConfig {
  const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
  const rng = new RNG(c.seed + episode, 'trace-cfg');
  const cfg = defaultEpisodeConfig(c.stage, nRooms);
  cfg.dynamics = c.dynamics;
  cfg.dyn = sampleDynParams(rng, defaultDynParams(), c.noiseScale);
  const noise = sampleNoise(rng, c.noiseScale, false);
  if (c.latencyTicks != null) noise.latencyTicks = c.latencyTicks;
  cfg.noise = noise;
  cfg.lighting = sampleLighting(c.seed + episode, nRooms, c.stage);
  cfg.wind = [0.05 * c.noiseScale, -0.03 * c.noiseScale];
  cfg.maxSteps = 260;
  cfg.stallSteps = 120;
  cfg.maxCollisions = 4;
  return cfg;
}

/** FNV-1a over raw bytes. */
export class Fnv {
  private h = 0x811c9dc5;
  bytes(u8: Uint8Array): void {
    let h = this.h;
    for (let i = 0; i < u8.length; i++) {
      h ^= u8[i]!;
      h = Math.imul(h, 0x01000193);
    }
    this.h = h;
  }
  f32(a: Float32Array): void {
    this.bytes(new Uint8Array(a.buffer, a.byteOffset, a.byteLength));
  }
  f64(v: number): void {
    this.bytes(new Uint8Array(Float64Array.of(v).buffer));
  }
  hex(): string {
    return (this.h >>> 0).toString(16).padStart(8, '0');
  }
}

export interface TraceRecord {
  obs: Float32Array[];
  rewards: number[];
  dones: boolean[];
  hash: string;
}

export interface TraceOptions {
  /** Building for an episode (default traceSpec). Lets a caller reuse spec objects across runs. */
  specFor?: (episode: number) => BuildingSpec;
  /** A pooled physics world for the episode's spec, or undefined for the env to build its own. */
  physicsFor?: (spec: BuildingSpec, episode: number) => PhysicsWorld | undefined;
  /** Keep every obs / reward / done in the record. */
  keep?: boolean;
}

/**
 * Runs `c.steps` steps with a seeded action stream, resetting into the next
 * episode's building on done.
 */
export function runTrace(env: ExplorerEnv, c: TraceCase, opts: TraceOptions = {}): TraceRecord {
  const keep = opts.keep ?? false;
  const physicsFor = opts.physicsFor;
  const specFor = opts.specFor ?? ((episode: number) => traceSpec(c, episode));
  const fnv = new Fnv();
  const actions = new RNG(c.seed, 'trace-actions');
  const record: TraceRecord = { obs: [], rewards: [], dones: [], hash: '' };
  let episode = 0;
  let spec = specFor(episode);
  const reset = () => {
    const physics = physicsFor?.(spec, episode);
    const cfg = traceConfig(c, spec, episode);
    return physics ? env.reset(spec, c.seed + episode, cfg, physics) : env.reset(spec, c.seed + episode, cfg);
  };
  let obs = reset();
  fnv.f32(obs);
  if (keep) record.obs.push(obs);
  const action = new Float32Array(4);
  for (let t = 0; t < c.steps; t++) {
    // Biased forward so episodes cover ground, hit walls and terminate.
    action[0] = actions.range(-0.3, 1);
    action[1] = actions.range(-0.6, 0.6);
    action[2] = actions.range(-0.4, 0.4);
    action[3] = actions.range(-0.7, 0.7);
    const step = env.step(action);
    fnv.f32(step.obs);
    fnv.f64(step.reward);
    fnv.bytes(Uint8Array.of(step.done ? 1 : 0, step.truncated ? 1 : 0));
    fnv.f64(step.info.coverage);
    if (keep) {
      record.obs.push(step.obs);
      record.rewards.push(step.reward);
      record.dones.push(step.done);
    }
    obs = step.obs;
    if (step.done) {
      episode += 1;
      spec = specFor(episode);
      obs = reset();
      fnv.f32(obs);
      if (keep) record.obs.push(obs);
    }
  }
  record.hash = fnv.hex();
  return record;
}

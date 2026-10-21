import { LIDAR_H, LIDAR_MAX } from '../shared/constants';
import type { BuildingSpec } from '../shared/types';
import type { LightingPreset } from '../sim/lighting';
import { nightFactor } from '../sim/lighting';
import { createEnv } from '../sim/env';
import { SeenMap } from '../sim/seenMap';
import { PhysicsWorld } from '../sim/world';
import { locNoise, type LocNoise } from '../loc/sensors';
import type { EstimatorOptions } from '../loc/estimator';
import type { LocMetrics } from '../loc/metrics';
import { LOG_STRIDE, replay, type TrajectoryLog } from '../loc/replay';
import { evalEpisodeSetup } from './coverage';
import { classicalPilot } from './pilots';

/**
 * Offline localization benchmark. A trajectory is flown once in the coverage
 * env by the sensor-only planner with the true pose (it is collision free, so
 * paths look like a mission's), logged, and then every estimator arm is
 * replayed open loop over the same log with its own simulated sensors. The
 * estimate never steers, so the arms are compared on identical paths; the
 * closed-loop cost (the planner steering on the estimate) is the mission
 * bench's job once the estimator is wired into the mission vehicle.
 */

export interface LocCondition {
  id: string;
  label: string;
  stage: number;
  seedBase: number;
  lighting: LightingPreset[];
}

/** Report seeds. Never used to choose a parameter: tuning runs on LOC_DEV_SEEDS. */
export const LOC_CONDITIONS: LocCondition[] = [
  { id: 'easy', label: 'Easy: one room (stage 0)', stage: 0, seedBase: 62001, lighting: ['day', 'night-dark'] },
  { id: 'hard', label: 'Hard: rooms, hallways, loops (stage 3)', stage: 3, seedBase: 60001, lighting: ['day', 'night-dark'] },
];

/** Seeds the estimator's parameters were tuned on (stage 3, both lightings). */
export const LOC_DEV_SEED_BASE = 7001;
export const LOC_DEV_SEEDS = 12;
export const LOC_SEEDS_PER_CONDITION = 30;

export interface LocArm {
  id: string;
  label: string;
  estimator?: Partial<EstimatorOptions>;
  noise?: Partial<LocNoise>;
  /** Teleport the true drone mid-flight (kidnapped robot): tests failure detection and recovery. */
  kidnap?: boolean;
}

export const LOC_ARMS: LocArm[] = [
  { id: 'fused', label: 'Shipped: IMU + flow + 90-beam lidar scan matching' },
  { id: 'odometry', label: 'Dead reckoning: IMU + flow only', estimator: { mode: 'odometry' } },
  { id: 'scan', label: 'Scan matching only (no IMU or flow)', estimator: { mode: 'scan' } },
  { id: 'beams32', label: 'Fused with the policy\'s 32-beam ring', noise: { beams: 32 } },
  { id: 'kidnap', label: 'Fused, drone displaced 1 m and 15° mid-flight', kidnap: true },
];

export interface LocJob {
  condition: string;
  stage: number;
  seed: number;
  lighting: LightingPreset;
}

export interface LocRow extends LocJob {
  arm: string;
  metrics: LocMetrics;
  /** Seen-free map painted from the estimated pose vs from the truth: intersection over union. */
  seenIoU: number;
  /** Share of voxels the estimate-built map marks free that the truth-built map does not. */
  seenFalseFree: number;
  /** Ticks from the kidnap until the error was back under 0.3 m (kidnap arm), or -1 if never. */
  recoverTicks: number;
  matched: number;
  sweeps: number;
  recoveries: number;
  ms: number;
}

/** Fly the sensor-only planner on the job's building with the true pose and log the trajectory. */
export async function recordTrajectory(job: LocJob): Promise<{ spec: BuildingSpec; log: TrajectoryLog; dark: boolean }> {
  const env = await createEnv();
  const { spec, envSeed, cfg } = evalEpisodeSetup(job.stage, job.seed, job.lighting);
  const pilot = classicalPilot();
  pilot.reset();
  let obs = env.reset(spec, envSeed, cfg);
  const poses: number[] = [];
  const push = () => {
    const s = env.snapshot();
    poses.push(s.pos[0], s.pos[1], s.pos[2], s.yaw, s.vel[0], s.vel[1], s.vel[2]);
  };
  push();
  for (let t = 0; t < cfg.maxSteps; t++) {
    const step = env.step(pilot.act(obs, env));
    obs = step.obs;
    push();
    if (step.done) break;
  }
  const dark = nightFactor(cfg.lighting) > 0.5;
  env.dispose();
  return { spec, log: { dt: 0.05, poses: Float32Array.from(poses) }, dark };
}

const ring = new Float64Array(LIDAR_H);

function castRing(world: PhysicsWorld, pos: [number, number, number], yaw: number): Float64Array {
  for (let i = 0; i < LIDAR_H; i++) {
    const a = yaw + (i / LIDAR_H) * Math.PI * 2;
    const hit = world.cast(pos[0], pos[1], pos[2], Math.cos(a), Math.sin(a), 0, LIDAR_MAX);
    ring[i] = hit ? hit.dist : LIDAR_MAX;
  }
  return ring;
}

export function locBounds(spec: BuildingSpec): EstimatorOptions['bounds'] {
  const w = spec.floors[0]!.w * spec.cellSize;
  const d = spec.floors[0]!.d * spec.cellSize;
  return { x0: -2, y0: -2, x1: w + 2, y1: d + 2 };
}

/** Replay every arm over one recorded trajectory. */
export function evaluateTrajectory(
  job: LocJob,
  spec: BuildingSpec,
  log: TrajectoryLog,
  dark: boolean,
  arms: LocArm[] = LOC_ARMS,
  overrides: Partial<EstimatorOptions> = {},
): LocRow[] {
  const world = new PhysicsWorld(spec);
  const rows: LocRow[] = [];
  const n = Math.floor(log.poses.length / LOG_STRIDE);
  for (const arm of arms) {
    const t0 = Date.now();
    const truthMap = new SeenMap();
    const estMap = new SeenMap();
    truthMap.reset(spec);
    estMap.reset(spec);
    const kidnapTick = Math.floor(n / 2);
    let recoverTicks = arm.kidnap ? -1 : 0;
    const H = spec.floorHeight;
    const floors = spec.floors.length;
    const floorOf = (z: number) => Math.max(0, Math.min(floors - 1, Math.floor(z / H)));
    const r = replay(log, world, locBounds(spec), floors, H, job.seed, {
      noise: { ...locNoise(dark), ...arm.noise },
      estimator: { ...overrides, ...arm.estimator },
      kidnap: arm.kidnap ? { tick: kidnapTick, offset: [0.7, -0.7, 0], yaw: 0.26 } : undefined,
      onTick: (truth, est, i) => {
        const rg = castRing(world, truth.pos, truth.yaw);
        truthMap.markRing(truth.pos[0], truth.pos[1], floorOf(truth.pos[2]), truth.yaw, rg);
        estMap.markRing(est.pos[0], est.pos[1], floorOf(est.pos[2]), est.yaw, rg);
        if (arm.kidnap && i >= kidnapTick && recoverTicks < 0) {
          if (Math.hypot(est.pos[0] - truth.pos[0], est.pos[1] - truth.pos[1]) < 0.3) recoverTicks = i - kidnapTick;
        }
      },
    });
    let inter = 0;
    let uni = 0;
    let falseFree = 0;
    let estCount = 0;
    for (let k = 0; k < truthMap.data.length; k++) {
      const a = truthMap.data[k]!;
      const b = estMap.data[k]!;
      if (a && b) inter += 1;
      if (a || b) uni += 1;
      if (b) estCount += 1;
      if (b && !a) falseFree += 1;
    }
    rows.push({
      ...job,
      arm: arm.id,
      metrics: r.metrics,
      seenIoU: uni ? inter / uni : 1,
      seenFalseFree: estCount ? falseFree / estCount : 0,
      recoverTicks,
      matched: r.stats.matched,
      sweeps: r.stats.sweeps,
      recoveries: r.stats.recoveries,
      ms: Date.now() - t0,
    });
  }
  world.free();
  return rows;
}

export function locJobs(seedsPerCondition = LOC_SEEDS_PER_CONDITION): LocJob[] {
  const out: LocJob[] = [];
  for (const c of LOC_CONDITIONS) {
    for (let i = 0; i < seedsPerCondition; i++) {
      for (const lighting of c.lighting) out.push({ condition: c.id, stage: c.stage, seed: c.seedBase + i, lighting });
    }
  }
  return out;
}

export function locDevJobs(): LocJob[] {
  const out: LocJob[] = [];
  for (let i = 0; i < LOC_DEV_SEEDS; i++) {
    for (const lighting of ['day', 'night-dark'] as const) out.push({ condition: 'dev', stage: 3, seed: LOC_DEV_SEED_BASE + i, lighting });
  }
  return out;
}

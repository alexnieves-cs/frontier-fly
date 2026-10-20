import { applyObsAblation } from './ablation';
import {
  COVERAGE_SUCCESS,
  MAX_HARD_COLLISIONS,
  OBS_DIM,
  PHYSICS_DT,
  PHYSICS_SUBSTEPS,
  POLICY_DT,
  STALL_STEPS,
} from '../shared/constants';
import { RNG } from '../shared/rng';
import type { BuildingSpec, DroneState, Dynamics, EpisodeConfig, Env, StepResult } from '../shared/types';
import { defaultDynParams, zeroNoise } from './dynamics/dr';
import { KinematicDynamics } from './dynamics/kinematic';
import { SensorNoise } from './dynamics/noise';
import { QuadrotorDynamics } from './dynamics/quad';
import { cloneDroneState, makeDroneState, scaleAction } from './drone';
import { defaultLighting, isLowLight } from './lighting';
import { ProgressTracker, rawReward, type ProgressPenalty } from './reward';
import { markVisited, sense } from './sensors';
import { initPhysics, PhysicsWorld } from './world';
import { isWalkable } from '../gen/validate';
import { frontierDistance } from './frontier';
import { FrontierNav } from './frontierFeatures';
import { EgoMaps, SeenMap } from './seenMap';

export function defaultEpisodeConfig(stage = 0, nRooms = 1): EpisodeConfig {
  return {
    stage,
    dynamics: stage >= 5 ? 'quad' : 'kinematic',
    dyn: defaultDynParams(),
    noise: zeroNoise(),
    lighting: defaultLighting(nRooms),
    wind: [0, 0],
  };
}

/**
 * Episode limits the shipped policy is trained and evaluated with. The time
 * feature (OBS.time) is steps-left / maxSteps, so every place that flies the
 * policy uses these to keep its observations in distribution.
 */
export function policyEpisodeLimits(stage: number): { maxSteps: number; stallSteps: number; maxCollisions: number } {
  return { maxSteps: stage <= 1 ? 1000 : 2400, stallSteps: 500, maxCollisions: 16 };
}

export class ExplorerEnv implements Env {
  private spec: BuildingSpec | null = null;
  private cfg: EpisodeConfig = defaultEpisodeConfig();
  private physics: PhysicsWorld | null = null;
  /** False when reset() was handed a pooled PhysicsWorld; the env then never frees it. */
  private ownsPhysics = true;
  private latencyTicks = 0;
  private readonly before: [number, number, number] = [0, 0, 0];
  private readonly delta: [number, number, number] = [0, 0, 0];
  private dyn: Dynamics = new KinematicDynamics();
  private noise = new SensorNoise(zeroNoise(), new RNG(1));
  private state = makeDroneState([0, 0, 1]);
  private visited = new Uint8Array(0);
  private visitedCount = 0;
  private roomsSeen = new Set<number>();
  private floorsSeen = new Set<number>();
  private prevAction = new Float32Array(4);
  private held = new Float32Array(4);
  private scaled = new Float32Array(4);
  private obs = new Float32Array(OBS_DIM);
  /** Sensor-only seen-free voxels, per floor, reset per episode. */
  private seen = new SeenMap();
  private nav = new FrontierNav();
  private maps = new EgoMaps();
  private collisions = 0;
  private hard = 0;
  private inContact = false;
  private steps = 0;
  private stall = 0;
  private maxSteps = 1000;
  private stallLimit = STALL_STEPS;
  private yawRate = 0;
  private prevYaw = 0;
  private readonly prevVel: [number, number, number] = [0, 0, 0];
  private frontierDist = 0;
  private done = false;
  private rng = new RNG(1);
  private progress = new ProgressTracker();
  private penalty: ProgressPenalty | undefined;

  /**
   * Start an episode. Pass `physics` (a PhysicsWorld built for this exact
   * `spec`, e.g. from a WorldPool) to skip the Rapier rebuild; the env then
   * borrows it and never frees it. Without it the env builds and owns one.
   * The physics world holds no per-episode state, so reuse is bit-identical.
   */
  reset(spec: BuildingSpec, seed: number, cfg?: EpisodeConfig, physics?: PhysicsWorld): Float32Array {
    if (physics && physics.spec !== spec) throw new Error('env.reset: physics world was built for a different spec');
    if (this.physics && this.ownsPhysics && this.physics !== physics) this.physics.free();
    this.spec = spec;
    const nRooms = spec.floors.reduce((n, floor) => n + floor.rooms.length, 0);
    this.cfg = cfg ?? defaultEpisodeConfig(0, nRooms);
    if (this.cfg.lighting.interiorLightsOn.length < nRooms) {
      const lights = new Uint8Array(nRooms);
      lights.set(this.cfg.lighting.interiorLightsOn);
      if (this.cfg.lighting.interiorLightsOn.length === 0) lights.fill(1);
      this.cfg = { ...this.cfg, lighting: { ...this.cfg.lighting, interiorLightsOn: lights } };
    }
    if (physics) {
      // Re-passing the world this env already holds keeps its ownership as is.
      if (physics !== this.physics) this.ownsPhysics = false;
      this.physics = physics;
    } else {
      this.physics = new PhysicsWorld(spec);
      this.ownsPhysics = true;
    }
    this.rng = new RNG(seed >>> 0, 'env');
    const noise = { ...this.cfg.noise };
    if (isLowLight(this.cfg.lighting)) noise.lowLightImu = Math.max(noise.lowLightImu, 1.5);
    this.noise = new SensorNoise(noise, this.rng.fork('noise'));
    this.latencyTicks = noise.latencyTicks | 0;
    this.state = makeDroneState([spec.spawn[0], spec.spawn[1], spec.spawn[2]], 0);
    this.dyn = this.cfg.dynamics === 'quad' ? new QuadrotorDynamics() : new KinematicDynamics();
    this.dyn.reset(this.state, this.cfg.dyn);
    const floor0 = spec.floors[0]!;
    const vw = floor0.w * 2;
    const vd = floor0.d * 2;
    this.visited = new Uint8Array(spec.floors.length * vw * vd);
    this.visitedCount = 0;
    this.seen.reset(spec);
    if (markVisited(spec, this.visited, this.state.pos[0], this.state.pos[1], this.state.pos[2])) {
      this.visitedCount = 1;
    }
    this.roomsSeen = new Set();
    this.floorsSeen = new Set();
    const room = this.roomAt(this.state.pos[0], this.state.pos[1], this.state.pos[2]);
    if (room >= 0) this.roomsSeen.add(room);
    const floor = this.floorAt(this.state.pos[2]);
    this.floorsSeen.add(floor);
    this.prevAction.fill(0);
    this.held.fill(0);
    this.collisions = 0;
    this.hard = 0;
    this.inContact = false;
    this.steps = 0;
    this.stall = 0;
    this.done = false;
    this.yawRate = 0;
    this.prevYaw = 0;
    this.prevVel[0] = 0;
    this.prevVel[1] = 0;
    this.prevVel[2] = 0;
    this.frontierDist = frontierDistance(spec, this.visited, this.state.pos);
    this.progress.reset(this.state.pos[0], this.state.pos[1]);
    this.penalty = undefined;
    const base = Math.max(400, Math.min(2500, Math.round(spec.reachableVoxels * 1.5)));
    this.maxSteps = this.cfg.maxSteps ?? base;
    this.stallLimit = this.cfg.stallSteps ?? STALL_STEPS;
    return this.observe();
  }

  step(action: Float32Array): StepResult {
    if (!this.spec || !this.physics) throw new Error('env.step before reset');
    if (this.done) {
      return {
        obs: this.obs.slice(),
        reward: 0,
        done: true,
        truncated: false,
        info: this.info(false, false),
      };
    }
    scaleAction(action, this.scaled);
    // The latency line keeps references, so it needs its own copy; without latency it returns the input.
    const delayed = this.noise.latency(this.latencyTicks > 0 ? new Float32Array(this.scaled) : this.scaled);
    this.held[0] = delayed[0] ?? 0;
    this.held[1] = delayed[1] ?? 0;
    this.held[2] = delayed[2] ?? 0;
    this.held[3] = delayed[3] ?? 0;

    const wind = this.cfg.wind;
    let hit = false;
    const substeps = this.cfg.dynamics === 'quad' ? PHYSICS_SUBSTEPS : 1;
    const dt = this.cfg.dynamics === 'quad' ? PHYSICS_DT : POLICY_DT;
    for (let i = 0; i < substeps; i++) {
      const before = this.before;
      before[0] = this.state.pos[0];
      before[1] = this.state.pos[1];
      before[2] = this.state.pos[2];
      this.dyn.step(this.held, dt);
      if (wind[0] !== 0 || wind[1] !== 0) {
        this.state.vel[0] += wind[0] * dt * 0.35;
        this.state.vel[1] += wind[1] * dt * 0.35;
      }
      const delta = this.delta;
      delta[0] = this.state.pos[0] - before[0];
      delta[1] = this.state.pos[1] - before[1];
      delta[2] = this.state.pos[2] - before[2];
      const resolved = this.physics.resolve(before, delta);
      this.state.pos[0] = resolved.pos[0];
      this.state.pos[1] = resolved.pos[1];
      this.state.pos[2] = resolved.pos[2];
      if (resolved.hit) {
        hit = true;
        if (resolved.blocked[0]) this.state.vel[0] = 0;
        if (resolved.blocked[1]) this.state.vel[1] = 0;
        if (resolved.blocked[2]) this.state.vel[2] = 0;
      }
    }

    // Wrap so a yaw crossing ±π reads as a small turn, not a 2π spike.
    let dyaw = this.state.yaw - this.prevYaw;
    dyaw -= Math.round(dyaw / (Math.PI * 2)) * Math.PI * 2;
    this.yawRate = dyaw / POLICY_DT;
    this.prevYaw = this.state.yaw;

    const spec = this.spec;
    let newVoxels = 0;
    if (markVisited(spec, this.visited, this.state.pos[0], this.state.pos[1], this.state.pos[2])) {
      this.visitedCount += 1;
      newVoxels += 1;
    }
    const room = this.roomAt(this.state.pos[0], this.state.pos[1], this.state.pos[2]);
    let newRoom = false;
    if (room >= 0 && !this.roomsSeen.has(room)) {
      this.roomsSeen.add(room);
      newRoom = true;
    }
    const floor = this.floorAt(this.state.pos[2]);
    let newFloor = false;
    if (!this.floorsSeen.has(floor)) {
      this.floorsSeen.add(floor);
      newFloor = true;
    }
    const impact = hit && !this.inContact;
    if (hit) {
      if (impact) {
        this.hard += 1;
        this.collisions += 1;
      }
      this.inContact = true;
    } else {
      this.inContact = false;
    }
    if (newVoxels > 0) this.stall = 0;
    else this.stall += 1;

    const coverage = spec.reachableVoxels > 0 ? this.visitedCount / spec.reachableVoxels : 0;
    const nextFrontier = frontierDistance(spec, this.visited, this.state.pos);
    const shaping = 1.5 * (this.frontierDist - nextFrontier);
    this.frontierDist = nextFrontier;
    const base = rawReward({
      newVoxels,
      newRoom,
      newFloor,
      impact,
      inContact: hit && !impact,
      action,
      prevAction: this.prevAction,
      coverage,
    }) + shaping;
    this.penalty = this.progress.update(this.state.pos[0], this.state.pos[1], newVoxels, action);
    const reward = base - this.penalty.total;
    this.prevAction.set(action.length >= 4 ? action.subarray(0, 4) : action);
    this.prevVel[0] = this.state.vel[0];
    this.prevVel[1] = this.state.vel[1];
    this.prevVel[2] = this.state.vel[2];
    this.steps += 1;

    const success = coverage >= COVERAGE_SUCCESS;
    const truncated = this.stall >= this.stallLimit;
    const crashed = this.hard > (this.cfg.maxCollisions ?? MAX_HARD_COLLISIONS);
    const oob = !this.inside(this.state.pos);
    this.done = success || truncated || crashed || oob || this.steps >= this.maxSteps;
    const obs = this.observe();
    return {
      obs,
      reward,
      done: this.done,
      truncated: truncated && !success,
      info: this.info(newRoom, newFloor, success),
    };
  }

  /** Free the Rapier world if this env owns it (a pooled world is left alone). The env can be reset again afterwards. */
  dispose(): void {
    if (this.ownsPhysics) this.physics?.free();
    this.physics = null;
    this.ownsPhysics = true;
    this.spec = null;
  }

  snapshot(): DroneState {
    return cloneDroneState(this.state);
  }

  /** Observation at a pose, without touching the episode RNG or drone state. */
  observeAt(x: number, y: number, z: number, yaw = 0): Float32Array {
    if (!this.spec || !this.physics) throw new Error('observeAt before reset');
    const ghost = makeDroneState([x, y, z], yaw);
    const out = new Float32Array(this.obs.length);
    const quiet = new SensorNoise(zeroNoise(), new RNG(1, 'probe'));
    sense(
      {
        world: this.physics,
        state: ghost,
        spec: this.spec,
        visited: this.visited,
        yawRate: 0,
        floor: this.floorAt(z),
        stepsLeft: Math.max(0, this.maxSteps - this.steps),
        maxSteps: this.maxSteps,
        lighting: this.cfg.lighting,
        noise: quiet,
        prevVel: [0, 0, 0],
        seen: this.seen.data,
        markSeen: null,
        nav: this.nav,
        maps: this.maps,
        stall: this.stall,
      },
      out,
    );
    return out;
  }

  get visitedGrid(): Uint8Array {
    return this.visited;
  }

  /** Sensor-only seen-free voxel grid (same layout as visitedGrid). */
  get seenGrid(): Uint8Array {
    return this.seen.data;
  }

  get episodeLighting() {
    return this.cfg.lighting;
  }

  /** Live lighting edits. The policy observation stays lidar/IMU based. */
  setLighting(partial: { timeOfDay?: number; flashlightOn?: boolean; sensorMode?: EpisodeConfig['lighting']['sensorMode']; lightsOn?: boolean }): void {
    if (partial.timeOfDay != null) this.cfg.lighting.timeOfDay = partial.timeOfDay;
    if (partial.flashlightOn != null) this.cfg.lighting.flashlightOn = partial.flashlightOn;
    if (partial.sensorMode != null) this.cfg.lighting.sensorMode = partial.sensorMode;
    if (partial.lightsOn != null) this.cfg.lighting.interiorLightsOn.fill(partial.lightsOn ? 1 : 0);
  }

  /** The physics world in use (owned or borrowed). */
  get physicsWorld(): PhysicsWorld | null {
    return this.physics;
  }

  get building(): BuildingSpec | null {
    return this.spec;
  }

  get coverage(): number {
    if (!this.spec || this.spec.reachableVoxels <= 0) return 0;
    return this.visitedCount / this.spec.reachableVoxels;
  }

  get stepCount(): number {
    return this.steps;
  }

  private observe(): Float32Array {
    const spec = this.spec!;
    const floor = this.floorAt(this.state.pos[2]);
    sense(
      {
        world: this.physics!,
        state: this.state,
        spec,
        visited: this.visited,
        yawRate: this.yawRate,
        floor,
        stepsLeft: Math.max(0, this.maxSteps - this.steps),
        maxSteps: this.maxSteps,
        lighting: this.cfg.lighting,
        noise: this.noise,
        prevVel: this.prevVel,
        seen: this.seen.data,
        markSeen: this.seen,
        nav: this.nav,
        maps: this.maps,
        stall: this.stall,
      },
      this.obs,
    );
    applyObsAblation(this.obs);
    return this.obs.slice();
  }

  private floorAt(z: number): number {
    const spec = this.spec!;
    return Math.max(0, Math.min(spec.floors.length - 1, Math.floor(z / spec.floorHeight)));
  }

  private roomAt(x: number, y: number, z: number): number {
    const spec = this.spec!;
    const f = this.floorAt(z);
    const floor = spec.floors[f]!;
    const gx = Math.floor(x / spec.cellSize);
    const gz = Math.floor(y / spec.cellSize);
    if (!isWalkable(floor.cells[gz * floor.w + gx] ?? 1)) return -1;
    for (const room of floor.rooms) {
      if (gx >= room.x && gx < room.x + room.w && gz >= room.z && gz < room.z + room.d) {
        return f * 1000 + room.id;
      }
    }
    return -1;
  }

  private inside(pos: [number, number, number]): boolean {
    const spec = this.spec!;
    const w = spec.floors[0]!.w * spec.cellSize;
    const d = spec.floors[0]!.d * spec.cellSize;
    return pos[0] > 0 && pos[1] > 0 && pos[0] < w && pos[1] < d && pos[2] > -1 && pos[2] < spec.floors.length * spec.floorHeight + 1;
  }

  private info(newRoom: boolean, newFloor: boolean, success = false) {
    return {
      coverage: this.coverage,
      collisions: this.collisions,
      floor: this.floorAt(this.state.pos[2]),
      newRoom,
      newFloor,
      success,
      penalty: this.penalty,
    };
  }
}

export async function createEnv(): Promise<ExplorerEnv> {
  await initPhysics();
  return new ExplorerEnv();
}

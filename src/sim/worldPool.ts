import { STAGE0_SEEDS, STAGE3_SEEDS } from '../eval/coverage';
import { generateBuilding } from '../gen/building';
import { Curriculum } from '../rl/curriculum';
import { RNG } from '../shared/rng';
import type { BuildingSpec, GenParams } from '../shared/types';
import { initPhysics, PhysicsWorld } from './world';

/**
 * A building and the Rapier world built for it. PhysicsWorld holds no
 * per-episode state (static colliders queried by raycasts, plus analytic grid
 * contact), so an episode on a reused world is bit-identical to one on a fresh
 * world. Pass `physics` as the 4th argument of ExplorerEnv.reset; the env
 * borrows it and never frees it.
 */
export interface PooledWorld {
  readonly spec: BuildingSpec;
  readonly physics: PhysicsWorld;
  readonly stage: number;
  readonly params: GenParams;
}

export interface WorldPoolOptions {
  /** Stages to pre-generate. Default 0..4. */
  stages?: number[];
  /** Worlds per stage, a number or a per-stage map. Default 24. */
  perStage?: number | Record<number, number>;
  /** Base seed. Different seeds give disjoint building sets (barring 1e-9 collisions). Default 1. */
  seed?: number;
  /**
   * Probability, per acquire(), of replacing the oldest world of that stage with
   * a freshly generated one. Keeps the building distribution open-ended while
   * paying the rebuild cost on only that fraction of resets. Default 0.
   */
  churn?: number;
  /** Cap on worlds kept by bySeed() (LRU). Default 16. */
  maxCached?: number;
}

interface Slot {
  world: PooledWorld;
  refs: number;
  retired: boolean;
}

function key(p: GenParams): string {
  return `${p.stage}:${p.seed}:${p.floors}x${p.w}x${p.d}`;
}

/** Identity of a generated building: generateBuilding is a pure function of these. */
function specKey(stage: number, spec: BuildingSpec): string {
  const floor0 = spec.floors[0]!;
  return `${stage}:${spec.seed}:${spec.floors.length}x${floor0.w}x${floor0.d}`;
}

let evalKeys: Set<string> | null = null;

/**
 * Keys of the fixed coverage-eval buildings (src/eval/coverage.ts: stage 0
 * seeds 101/202/303/404, stage 3 seeds 4242/5150/6160). Keyed on the generated
 * spec, so a pool draw that lands on an eval building through
 * generateBuilding's seed+1 retry is caught too.
 */
export function evalBuildingKeys(): Set<string> {
  if (evalKeys) return evalKeys;
  const curriculum = new Curriculum();
  const keys = new Set<string>();
  const add = (stage: number, seed: number) => {
    const params = curriculum.genParams(stage, new RNG(seed, `eval-${stage}`));
    keys.add(specKey(stage, generateBuilding(params)));
  };
  for (const seed of STAGE0_SEEDS) add(0, seed);
  for (const seed of STAGE3_SEEDS) add(3, seed);
  evalKeys = keys;
  return keys;
}

/** True when `spec` (generated for `stage`) is one of the eval buildings. */
export function isEvalBuilding(stage: number, spec: BuildingSpec): boolean {
  return evalBuildingKeys().has(specKey(stage, spec));
}

/**
 * Pre-generated training buildings with their physics worlds, so episode resets
 * skip generateBuilding + the Rapier rebuild. Memory is bounded: a fixed number
 * of worlds per stage (plus at most `maxCached` from bySeed). Single-threaded:
 * each worker thread owns its own pool.
 */
export class WorldPool {
  private readonly curriculum = new Curriculum();
  private readonly byStage = new Map<number, Slot[]>();
  private readonly cursor = new Map<number, number>();
  private readonly cache = new Map<string, Slot>();
  private readonly retired = new Set<Slot>();
  private readonly slotOf = new WeakMap<PooledWorld, Slot>();
  private readonly seed: number;
  private readonly churn: number;
  private readonly maxCached: number;
  private generated = 0;
  private serial = 0;
  private freed = false;

  private constructor(opts: WorldPoolOptions) {
    this.seed = (opts.seed ?? 1) >>> 0;
    this.churn = Math.max(0, Math.min(1, opts.churn ?? 0));
    this.maxCached = Math.max(0, opts.maxCached ?? 16);
  }

  static async create(opts: WorldPoolOptions = {}): Promise<WorldPool> {
    await initPhysics();
    const pool = new WorldPool(opts);
    const stages = opts.stages ?? [0, 1, 2, 3, 4];
    for (const stage of stages) {
      const count = typeof opts.perStage === 'number' ? opts.perStage : (opts.perStage?.[stage] ?? 24);
      const slots: Slot[] = [];
      for (let i = 0; i < count; i++) slots.push(pool.fresh(stage));
      pool.byStage.set(stage, slots);
      pool.cursor.set(stage, 0);
    }
    return pool;
  }

  /** Worlds currently held for training (excludes the bySeed cache). */
  get size(): number {
    let n = 0;
    for (const slots of this.byStage.values()) n += slots.length;
    return n;
  }

  get stages(): number[] {
    return [...this.byStage.keys()].sort((a, b) => a - b);
  }

  /** Buildings generated so far (pre-generation + churn + lazily added stages). */
  get builds(): number {
    return this.generated;
  }

  /**
   * A random world of `stage`. Stages not pre-generated get a slot list lazily
   * (size 8). Call release() when the env moves on, so churned-out worlds can
   * be freed once nothing uses them.
   */
  acquire(stage: number, rng: RNG): PooledWorld {
    if (this.freed) throw new Error('WorldPool used after free()');
    let slots = this.byStage.get(stage);
    if (!slots || slots.length === 0) {
      slots = [];
      for (let i = 0; i < 8; i++) slots.push(this.fresh(stage));
      this.byStage.set(stage, slots);
      this.cursor.set(stage, 0);
    }
    if (this.churn > 0 && rng.next() < this.churn) {
      const at = this.cursor.get(stage) ?? 0;
      const old = slots[at]!;
      slots[at] = this.fresh(stage);
      this.cursor.set(stage, (at + 1) % slots.length);
      this.retire(old);
    }
    const slot = slots[rng.int(slots.length)]!;
    slot.refs += 1;
    return slot.world;
  }

  /** Marks a world from acquire()/bySeed() as no longer used by the caller. */
  release(world: PooledWorld | null | undefined): void {
    if (!world) return;
    const slot = this.slotOf.get(world);
    if (!slot) return;
    slot.refs = Math.max(0, slot.refs - 1);
    if (slot.retired && slot.refs === 0) {
      slot.world.physics.free();
      this.retired.delete(slot);
    }
  }

  /**
   * The world for explicit GenParams (e.g. an eval building), generated on a
   * miss and kept in a small LRU cache. Not excluded from eval seeds.
   */
  bySeed(params: GenParams): PooledWorld {
    if (this.freed) throw new Error('WorldPool used after free()');
    const k = key(params);
    const hit = this.cache.get(k);
    if (hit) {
      this.cache.delete(k);
      this.cache.set(k, hit);
      hit.refs += 1;
      return hit.world;
    }
    const spec = generateBuilding(params);
    this.generated += 1;
    const slot = this.slot({ spec, physics: new PhysicsWorld(spec), stage: params.stage, params });
    slot.refs = 1;
    this.cache.set(k, slot);
    while (this.cache.size > this.maxCached) {
      const [oldKey, old] = this.cache.entries().next().value as [string, Slot];
      this.cache.delete(oldKey);
      this.retire(old);
    }
    return slot.world;
  }

  /** Frees every Rapier world. Envs still holding one must be reset or disposed first. */
  free(): void {
    if (this.freed) return;
    this.freed = true;
    for (const slots of this.byStage.values()) for (const slot of slots) slot.world.physics.free();
    for (const slot of this.cache.values()) slot.world.physics.free();
    for (const slot of this.retired) slot.world.physics.free();
    this.byStage.clear();
    this.cache.clear();
    this.retired.clear();
  }

  private retire(slot: Slot): void {
    slot.retired = true;
    if (slot.refs === 0) slot.world.physics.free();
    else this.retired.add(slot);
  }

  private slot(world: PooledWorld): Slot {
    const slot: Slot = { world, refs: 0, retired: false };
    this.slotOf.set(world, slot);
    return slot;
  }

  private fresh(stage: number): Slot {
    for (;;) {
      const n = this.serial;
      this.serial += 1;
      this.generated += 1;
      const params = this.curriculum.genParams(stage, new RNG(this.seed, `pool-${stage}-${n}`));
      const spec = generateBuilding(params);
      if (isEvalBuilding(stage, spec)) continue;
      return this.slot({ spec, physics: new PhysicsWorld(spec), stage, params });
    }
  }
}

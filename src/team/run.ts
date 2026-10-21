import { RNG } from '../shared/rng';
import type { BuildingSpec, EpisodeConfig } from '../shared/types';
import { sensorFrame } from '../eval/pilots';
import { createEnv, type ExplorerEnv } from '../sim/env';
import {
  commandToAction,
  descentClear,
  filterVelocity,
  OccupancyMap,
  QUAD_BRAKING,
  SensorPlanner,
  type SensorCommand,
} from '../sim/sensorPlanner';
import { initPhysics, PhysicsWorld } from '../sim/world';
import { ClaimPenalty, DEFAULT_CLAIMS, type ClaimOptions } from './allocate';
import { exchange, MeshNode, type MeshStats } from './mesh';
import { basePosition, RadioModel, type RadioParams } from './radio';

export type Role = 'explore' | 'relay';
/** Coverage levels the time-to-clear metric is reported at, percent. */
export const TIME_KEYS = ['30', '50', '60', '80', '90'] as const;
export type TimeKey = (typeof TIME_KEYS)[number];
/** radio: the lossy wall-attenuated model. perfect: every packet arrives, no size cap. none: no radio at all. */
export type Comms = 'radio' | 'perfect' | 'none';

export interface TeamOptions {
  drones: number;
  /** Per drone; missing entries explore. */
  roles?: Role[];
  comms?: Comms;
  radio?: Partial<RadioParams>;
  /** Team steps between exchange rounds (10 steps = 0.5 s = 2 Hz). */
  exchangeEvery?: number;
  /** Map entries per packet. */
  maxEntries?: number;
  /** Team steps between launches through the entry (one drone at a time). */
  launchGap?: number;
  /** Frontier claims; null shares maps but does not split frontiers. */
  claims?: ClaimOptions | null;
  /** Mean link margin to the base, dB, below which a relay stops and backs up. */
  relayMargin?: number;
  /** Planner cruise speed, m/s. */
  speed?: number;
  /** Team steps between samples of the coverage curves. */
  sampleEvery?: number;
  /** Record every drone's track (for the page), one point per `traceEvery` steps. */
  traceEvery?: number;
}

export interface TeamResult {
  drones: number;
  steps: number;
  /** Union of flown voxels over reachable voxels at the end. */
  coverage: number;
  /** Team step at which coverage first reached each percentage (null if never). */
  timeTo: Record<TimeKey, number | null>;
  /** Flown voxels the base had heard about at the end, over reachable voxels: what the operator can see. */
  baseKnown: number;
  /** Coverage and base-known coverage, one sample per `sampleEvery` steps. */
  curve: number[];
  baseCurve: number[];
  /** Coverage minus base-known coverage, mean and worst over the samples: how far behind the operator's picture runs. */
  awarenessGap: number;
  awarenessGapMax: number;
  collisions: number;
  /** Steps on which two drones were closer than 0.35 m (centers): they would have touched. */
  droneContacts: number;
  /** Each drone's own flown voxels over reachable. */
  perDrone: number[];
  /** Sum of each drone's flown voxels over the union: 1 means no voxel was flown twice. */
  overlap: number;
  packets: MeshStats;
  /** Per drone: [x, y, z] samples, when traceEvery is set. */
  tracks?: number[][];
  /** Step each relay parked at (null if it never needed to). */
  relayParkedAt: (number | null)[];
}

/** Separation the team keeps between drone centers, meters (two 0.15 m airframes plus air). */
const SEPARATION = 0.45;

/**
 * Cut the part of a world-frame velocity that closes on a nearby teammate, the
 * way the lidar filter treats a wall. Drones are not physics bodies in the
 * sim, so this stands in for a drone seeing another one on its lidar.
 */
export function separate(vx: number, vy: number, me: readonly number[], others: readonly (readonly number[])[]): [number, number] {
  for (const o of others) {
    if (Math.abs(o[2]! - me[2]!) > 1) continue;
    const dx = o[0]! - me[0]!;
    const dy = o[1]! - me[1]!;
    const d = Math.hypot(dx, dy);
    if (d > 1.5 || d < 1e-6) continue;
    const ux = dx / d;
    const uy = dy / d;
    const allowed = (d - SEPARATION) / 0.3;
    const along = vx * ux + vy * uy;
    if (along > allowed) {
      vx -= (along - allowed) * ux;
      vy -= (along - allowed) * uy;
    }
  }
  return [vx, vy];
}

interface Drone {
  env: ExplorerEnv;
  planner: SensorPlanner;
  occ: OccupancyMap | null;
  node: MeshNode;
  claim: ClaimPenalty;
  claimSource: { heard: MeshNode['heard']; selfId: number; pos: [number, number, number]; step: number; floorHeight: number; drones: number };
  obs: Float32Array;
  role: Role;
  launched: boolean;
  done: boolean;
  collisions: number;
  /** Relay state: breadcrumbs with their mean base margin. */
  crumbs: { x: number; y: number; z: number; margin: number }[];
  relay: 'explore' | 'return' | 'hold';
  relayTarget: number;
  relayParkedAt: number | null;
}

/** What the page draws each frame. */
export interface TeamView {
  step: number;
  pos: [number, number, number][];
  yaw: number[];
  goal: ([number, number] | null)[];
  role: Role[];
  relay: Drone['relay'][];
  launched: boolean[];
  done: boolean[];
  base: [number, number, number];
  coverage: number;
  baseKnown: number;
}

/**
 * A team of sensor-only planners in one building. Every drone runs its own
 * SensorPlanner on the team map as far as it knows it (its own lidar plus
 * whatever reached it over the radio), and avoids frontiers its teammates have
 * claimed. Drones launch one at a time from the same entry point.
 *
 * Coverage is measured on the truth (the union of voxels flown); base-known
 * coverage is what reached the base radio outside, which is what an operator
 * could act on.
 */
export class TeamSim {
  readonly spec: BuildingSpec;
  readonly radio: RadioModel;
  readonly base: [number, number, number];
  readonly maxSteps: number;
  readonly size: number;
  /** Which drone first flew each voxel (index + 1; 0 = nobody), for the page. */
  readonly firstBy: Uint8Array;
  readonly baseNode: MeshNode;
  private readonly drones: Drone[] = [];
  private readonly nodes: MeshNode[];
  private readonly physics: PhysicsWorld;
  private readonly rng: RNG;
  private readonly comms: Comms;
  private readonly exchangeEvery: number;
  private readonly maxEntries: number;
  private readonly launchGap: number;
  private readonly relayMargin: number;
  private readonly sampleEvery: number;
  private readonly traceEvery: number;
  private readonly reach: number;
  private unionCount = 0;
  private readonly curve: number[] = [];
  private readonly baseCurve: number[] = [];
  private readonly timeTo = Object.fromEntries(TIME_KEYS.map((k) => [k, null])) as TeamResult['timeTo'];
  readonly packets: MeshStats = { sent: 0, delivered: 0, bytes: 0 };
  private readonly tracks: number[][] | undefined;
  private droneContacts = 0;
  private readonly out = new Float32Array(4);
  private t = 0;
  private finished = false;

  private constructor(spec: BuildingSpec, cfg: EpisodeConfig & { maxSteps: number }, opts: TeamOptions, seed: number) {
    this.spec = spec;
    this.comms = opts.comms ?? 'radio';
    this.exchangeEvery = opts.exchangeEvery ?? 10;
    this.maxEntries = this.comms === 'perfect' ? Infinity : (opts.maxEntries ?? 1500);
    this.launchGap = opts.launchGap ?? 20;
    this.relayMargin = opts.relayMargin ?? 6;
    this.sampleEvery = opts.sampleEvery ?? 20;
    this.traceEvery = opts.traceEvery ?? 0;
    this.maxSteps = cfg.maxSteps;
    this.rng = new RNG(seed, 'team');
    this.radio = new RadioModel(spec, opts.radio);
    this.base = basePosition(spec);
    this.physics = new PhysicsWorld(spec);
    const floor0 = spec.floors[0]!;
    this.size = spec.floors.length * floor0.w * 2 * floor0.d * 2;
    this.firstBy = new Uint8Array(this.size);
    this.reach = Math.max(1, spec.reachableVoxels);
    this.baseNode = new MeshNode(opts.drones, this.size, opts.drones + 1);
    this.baseNode.pos = [...this.base];
    this.tracks = this.traceEvery ? Array.from({ length: opts.drones }, () => []) : undefined;
    this.nodes = [this.baseNode];
  }

  /** Build the team, reset and ready to step. */
  static async create(spec: BuildingSpec, envSeed: number, cfg: EpisodeConfig & { maxSteps: number }, opts: TeamOptions, seed: number): Promise<TeamSim> {
    await initPhysics();
    const sim = new TeamSim(spec, cfg, opts, seed);
    const n = opts.drones;
    const claims = opts.claims === undefined ? DEFAULT_CLAIMS : opts.claims;
    const quad = cfg.dynamics === 'quad';
    for (let i = 0; i < n; i++) {
      const env = await createEnv();
      // Idle hovering is part of the job here (a parked relay, a drone with nothing left): no stall cut-off.
      const obs = env.reset(spec, envSeed + i * 7919, { ...cfg, stallSteps: cfg.maxSteps + 1 }, sim.physics);
      const node = new MeshNode(i, sim.size, n + 1);
      const claimSource = { heard: node.heard, selfId: i, pos: [0, 0, 0] as [number, number, number], step: 0, floorHeight: spec.floorHeight, drones: n };
      const claim = new ClaimPenalty(claims ?? DEFAULT_CLAIMS, claimSource);
      const planner = new SensorPlanner({ speed: opts.speed, braking: quad ? QUAD_BRAKING : undefined, penalty: claims && sim.comms !== 'none' ? claim : undefined });
      sim.drones.push({
        env,
        planner,
        occ: null,
        node,
        claim,
        claimSource,
        obs,
        role: opts.roles?.[i] ?? 'explore',
        launched: false,
        done: false,
        collisions: 0,
        crumbs: [],
        relay: 'explore',
        relayTarget: -1,
        relayParkedAt: null,
      });
    }
    sim.nodes.unshift(...sim.drones.map((d) => d.node));
    return sim;
  }

  get done(): boolean {
    return this.finished;
  }

  get stepCount(): number {
    return this.t;
  }

  get coverage(): number {
    return this.unionCount / this.reach;
  }

  get baseKnown(): number {
    return countOnes(this.baseNode.visited) / this.reach;
  }

  /** Advance the whole team one policy step (50 ms). Returns false once the run is over. */
  step(): boolean {
    if (this.finished) return false;
    const t = this.t;
    const drones = this.drones;
    const n = drones.length;
    const comms = this.comms;
    const poses = drones.map((d) => d.env.snapshot().pos);
    for (let i = 0; i < n; i++) {
      const d = drones[i]!;
      if (!d.launched && t >= i * this.launchGap) d.launched = true;
      d.node.pos = [poses[i]![0], poses[i]![1], poses[i]![2]];
      d.node.absorbOwn(d.env.seenGrid, d.env.visitedGrid);
      if (!d.launched || d.done) continue;

      const f = sensorFrame(d.env, d.obs);
      if (!d.occ) d.occ = new OccupancyMap(f.vw, f.vd, f.floors);
      const floor = Math.max(0, Math.min(f.floors - 1, Math.floor(f.pos[2] / f.floorHeight)));
      d.occ.mark(floor, f.pos[0], f.pos[1], f.yaw, f.lidar);
      f.seen = d.node.seen;
      f.visited = d.node.visited;
      f.occupied = d.occ.occupied;
      d.claimSource.pos = d.node.pos;
      d.claimSource.step = t;

      let cmd: SensorCommand;
      if (d.role === 'relay' && comms !== 'none') cmd = relayAct(d, f, this.radio, this.base, this.relayMargin, t);
      else cmd = d.planner.act(f);
      d.node.goal = d.relay === 'explore' ? d.planner.goalPoint : null;
      const others = poses.filter((_, j) => j !== i && drones[j]!.launched && !drones[j]!.done);
      [cmd.vx, cmd.vy] = separate(cmd.vx, cmd.vy, poses[i]!, others);
      const step = d.env.step(commandToAction(cmd, f.yaw, this.out));
      d.obs = step.obs;
      d.collisions = step.info.collisions;
      if (step.done) d.done = true;
    }

    // Truth: the union of flown voxels.
    const union = this.firstBy;
    for (let i = 0; i < n; i++) {
      const v = drones[i]!.env.visitedGrid;
      for (let k = 0; k < this.size; k++) {
        if (v[k] === 1 && union[k] === 0) {
          union[k] = i + 1;
          this.unionCount += 1;
        }
      }
    }
    const cov = this.coverage;
    for (const key of TIME_KEYS) if (this.timeTo[key] === null && cov >= Number(key) / 100) this.timeTo[key] = t + 1;

    for (let i = 0; i < n; i++) {
      for (let j = i + 1; j < n; j++) {
        if (!drones[i]!.launched || !drones[j]!.launched) continue;
        const a = drones[i]!.env.snapshot().pos;
        const b = drones[j]!.env.snapshot().pos;
        if (Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 0.35) this.droneContacts += 1;
      }
    }

    if (comms !== 'none' && (t + 1) % this.exchangeEvery === 0) exchange(this.nodes, this.radio, this.rng, t + 1, this.maxEntries, this.packets, comms === 'perfect');
    if ((t + 1) % this.sampleEvery === 0) {
      this.curve.push(cov);
      this.baseCurve.push(this.baseKnown);
    }
    if (this.tracks && t % this.traceEvery === 0) {
      for (let i = 0; i < n; i++) {
        const p = drones[i]!.env.snapshot().pos;
        this.tracks[i]!.push(round2(p[0]), round2(p[1]), round2(p[2]));
      }
    }
    this.t = t + 1;
    if (cov >= 0.95 || drones.every((d) => d.done) || this.t >= this.maxSteps) this.finished = true;
    return !this.finished;
  }

  view(): TeamView {
    const ds = this.drones;
    return {
      step: this.t,
      pos: ds.map((d) => {
        const p = d.env.snapshot().pos;
        return [p[0], p[1], p[2]];
      }),
      yaw: ds.map((d) => d.env.snapshot().yaw),
      goal: ds.map((d) => d.node.goal),
      role: ds.map((d) => d.role),
      relay: ds.map((d) => d.relay),
      launched: ds.map((d) => d.launched),
      done: ds.map((d) => d.done),
      base: this.base,
      coverage: this.coverage,
      baseKnown: this.baseKnown,
    };
  }

  result(): TeamResult {
    const drones = this.drones;
    const reach = this.reach;
    let own = 0;
    const perDrone = drones.map((d) => {
      const c = countOnes(d.env.visitedGrid);
      own += c;
      return c / reach;
    });
    const curve = this.curve;
    const baseCurve = this.baseCurve;
    const result: TeamResult = {
      drones: drones.length,
      steps: this.t,
      coverage: this.coverage,
      timeTo: { ...this.timeTo },
      baseKnown: this.baseKnown,
      curve: [...curve],
      baseCurve: [...baseCurve],
      awarenessGap: curve.length ? curve.reduce((acc, c, i) => acc + c - baseCurve[i]!, 0) / curve.length : 0,
      awarenessGapMax: curve.reduce((acc, c, i) => Math.max(acc, c - baseCurve[i]!), 0),
      collisions: drones.reduce((s, d) => s + d.collisions, 0),
      droneContacts: this.droneContacts,
      perDrone,
      overlap: this.unionCount > 0 ? own / this.unionCount : 1,
      packets: { ...this.packets },
      relayParkedAt: drones.map((d) => d.relayParkedAt),
    };
    if (this.tracks) result.tracks = this.tracks.map((tr) => [...tr]);
    return result;
  }

  dispose(): void {
    for (const d of this.drones) d.env.dispose();
    this.physics.free();
  }
}

/** Fly a team to the end of its budget (see TeamSim). */
export async function runTeam(spec: BuildingSpec, envSeed: number, cfg: EpisodeConfig & { maxSteps: number }, opts: TeamOptions, seed: number): Promise<TeamResult> {
  const sim = await TeamSim.create(spec, envSeed, cfg, opts, seed);
  while (sim.step());
  const r = sim.result();
  sim.dispose();
  return r;
}

function countOnes(a: Uint8Array): number {
  let c = 0;
  for (let k = 0; k < a.length; k++) c += a[k]!;
  return c;
}

function round2(v: number): number {
  return Math.round(v * 100) / 100;
}

/**
 * A relay explores like the others until its link to the base gets weak
 * (mean margin below `relayMargin`). Then it flies back along its own track to
 * the deepest point where the link was still good, and hovers there so the
 * explorers' maps keep reaching the base through it.
 */
function relayAct(d: Drone, f: ReturnType<typeof sensorFrame>, radio: RadioModel, base: readonly [number, number, number], relayMargin: number, t: number): SensorCommand {
  const pos = f.pos;
  const margin = radio.margin([pos[0], pos[1], pos[2]], base);
  if (d.relay === 'explore') {
    const last = d.crumbs[d.crumbs.length - 1];
    if (!last || Math.hypot(last.x - pos[0], last.y - pos[1]) > 0.5) d.crumbs.push({ x: pos[0], y: pos[1], z: pos[2], margin });
    if (margin >= relayMargin) return d.planner.act(f);
    // Deepest breadcrumb with a few dB of headroom.
    let target = 0;
    for (let i = d.crumbs.length - 1; i >= 0; i--) {
      if (d.crumbs[i]!.margin >= relayMargin + 3) {
        target = i;
        break;
      }
    }
    d.relay = 'return';
    d.relayTarget = target;
  }
  const floorZ = Math.floor(pos[2] / f.floorHeight) * f.floorHeight;
  let vz = Math.max(-0.6, Math.min(0.6, (floorZ + 1.2 - pos[2]) * 2));
  if (vz < 0 && !descentClear(f.lidar)) vz = 0;
  if (d.relay === 'hold') return { vx: 0, vy: 0, vz, yawRate: 0, exploring: false };
  // Back along the breadcrumbs: aim at the newest crumb not yet reached, down to the target.
  while (d.crumbs.length - 1 > d.relayTarget) {
    const c = d.crumbs[d.crumbs.length - 1]!;
    if (Math.hypot(c.x - pos[0], c.y - pos[1]) > 0.4) break;
    d.crumbs.pop();
  }
  const c = d.crumbs[d.crumbs.length - 1]!;
  const ex = c.x - pos[0];
  const ey = c.y - pos[1];
  const el = Math.hypot(ex, ey);
  if (d.crumbs.length - 1 <= d.relayTarget && el < 0.3) {
    d.relay = 'hold';
    d.relayParkedAt = t;
    return { vx: 0, vy: 0, vz, yawRate: 0, exploring: false };
  }
  const sp = Math.min(1.5, el * 2);
  let vx = el > 1e-6 ? (ex / el) * sp : 0;
  let vy = el > 1e-6 ? (ey / el) * sp : 0;
  [vx, vy] = filterVelocity(vx, vy, f.yaw, f.lidar);
  return { vx, vy, vz, yawRate: 0, exploring: true };
}

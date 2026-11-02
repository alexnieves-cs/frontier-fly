// The Node half of the Python package (python/frontierfly). One process holds one
// ExplorerEnv and speaks a framed protocol on stdin/stdout:
//
//   frame   = u32 total length (LE) | u32 JSON length (LE) | JSON header | binary payload
//
// Python sends {cmd, ...} headers; the server answers {ok: true, ...} or {ok: false, error}.
// Float arrays travel as the binary payload (little-endian float32), never as JSON.
// `npm run build:py` bundles this file with esbuild into python/frontierfly/_server/server.mjs,
// so a Python user needs Node 22+ on PATH and nothing from npm.
//
// Commands
//   hello                                      versions, sizes, the FrontierBench tasks and seeds
//   reset  {seed, task? | stage, lighting, dynamics, speedH, dr, maxSteps?}   training episode
//   bench_reset {task, split, index}           flight `index` of a FrontierBench split, scored
//   step   {pilot?} + 4 float32 action         one policy step (20 Hz); `pilot` lets a baseline act
//   act    {pilot}                             the baseline's action for the current observation
//   frame                                      the bench sensor frame (pose, lidar, seen/visited grids)
//   summarize {episodes}                       the leaderboard summary of scored episodes
//   close
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { BASELINES, policyPilot } from '../../src/bench/baselines';
import type { BenchContext, BenchPilot } from '../../src/bench/pilot';
import { BenchEpisodeRecorder, benchEpisodeSetup, type BenchEpisode } from '../../src/bench/run';
import { BENCH_SEEDS, BENCH_TASKS, BENCH_VERSION, benchRuns, type BenchSplit, type BenchTask } from '../../src/bench/tasks';
import { parsePlanSet, type PlanSet } from '../../src/plans';
import { parseCheckpoint, SHIPPED_POLICY_URL } from '../../src/rl/checkpoint';
import { actingWeights, InferPolicy } from '../../src/rl/infer';
import { RNG } from '../../src/shared/rng';
import { ACT_DIM, FLOOR_HEIGHT, OBS, OBS_DIM, OBS_VERSION } from '../../src/shared/constants';
import type { StepResult } from '../../src/shared/types';
import { createEnv, type ExplorerEnv } from '../../src/sim/env';
import { LIGHTING_PRESETS, type LightingPreset } from '../../src/sim/lighting';
import { sensorFrame } from '../../src/sim/sensorPlanner';
import { summarize } from '../bench/lib';

/** Bumped when the wire protocol changes; the Python side refuses a mismatch. */
const PROTOCOL = 1;

const assetsArg = process.argv.indexOf('--assets');
const ASSETS = assetsArg >= 0 ? process.argv[assetsArg + 1]! : 'public';

type Header = Record<string, unknown> & { cmd: string };

let planSet: PlanSet | null = null;
function plans(): PlanSet {
  planSet ??= parsePlanSet(JSON.parse(readFileSync(join(ASSETS, 'plans/swiss-dwellings.json'), 'utf8')));
  return planSet;
}

/**
 * The shipped network, deterministic. Same as bench:score's loadPolicy, but from the
 * actor-only policy.bin (half the size; its observation normalizer is the identity).
 */
function loadShippedPolicy(path: string): (obs: Float32Array) => Float32Array {
  const ck = parseCheckpoint(readFileSync(path));
  const policy = new InferPolicy(ck.obsDim, ck.actDim, ck.hidden, 1);
  policy.setWeights(actingWeights(ck.weights, ck.obsDim, ck.actDim, ck.hidden));
  const rng = new RNG(1);
  return (obs) => policy.actWith(obs, 1, rng, true).action.slice();
}

let shippedPolicy: ((obs: Float32Array) => Float32Array) | null = null;
function makePilot(id: string): BenchPilot {
  if (id === 'policy' || id === 'guarded') {
    shippedPolicy ??= loadShippedPolicy(join(ASSETS, SHIPPED_POLICY_URL));
    return policyPilot(id === 'policy' ? 'Shipped policy (DAgger + PPO)' : 'Shipped policy under the lidar guard', shippedPolicy, id === 'guarded');
  }
  const make = BASELINES[id];
  if (!make) throw new Error(`unknown pilot "${id}"; known: ${[...Object.keys(BASELINES), 'policy', 'guarded'].join(', ')}`);
  return make();
}

function findTask(id: unknown): { task: BenchTask; index: number } {
  const index = BENCH_TASKS.findIndex((t) => t.id === id);
  if (index < 0) throw new Error(`unknown task "${String(id)}"; known: ${BENCH_TASKS.map((t) => t.id).join(', ')}`);
  return { task: BENCH_TASKS[index]!, index };
}

function num(h: Header, key: string, fallback: number): number {
  const v = h[key];
  if (v === undefined || v === null) return fallback;
  if (typeof v !== 'number' || !Number.isFinite(v)) throw new Error(`${key} must be a finite number`);
  return v;
}

/** A custom task from reset() options, or a named FrontierBench task. */
function taskFrom(h: Header): BenchTask {
  if (h.task !== undefined && h.task !== null) return findTask(h.task).task;
  const lighting = (h.lighting ?? 'day') as LightingPreset;
  if (!LIGHTING_PRESETS.includes(lighting)) throw new Error(`lighting must be one of ${LIGHTING_PRESETS.join(', ')}`);
  const dynamics = h.dynamics ?? 'kinematic';
  if (dynamics !== 'kinematic' && dynamics !== 'quad') throw new Error('dynamics must be "kinematic" or "quad"');
  const stage = num(h, 'stage', 0);
  if (!Number.isInteger(stage) || stage < 0 || stage > 5) throw new Error('stage must be an integer 0..5');
  return { id: 'custom', label: 'Custom', stage, dynamics, lighting, speedH: num(h, 'speedH', 2), dr: num(h, 'dr', 0) };
}

class Session {
  private env!: ExplorerEnv;
  private obs: Float32Array | null = null;
  private ctx: BenchContext | null = null;
  private maxSteps = 0;
  private steps = 0;
  private bench: BenchEpisodeRecorder | null = null;
  private pilots = new Map<string, BenchPilot>();

  async init(): Promise<void> {
    this.env = await createEnv();
  }

  private startEpisode(obs: Float32Array, ctx: BenchContext): void {
    this.obs = obs;
    this.ctx = ctx;
    this.maxSteps = ctx.maxSteps;
    this.steps = 0;
    this.pilots.clear();
  }

  private pilot(id: string): BenchPilot {
    let p = this.pilots.get(id);
    if (!p) {
      p = makePilot(id);
      p.reset(this.ctx!);
      this.pilots.set(id, p);
    }
    return p;
  }

  private requireEpisode(): Float32Array {
    if (!this.obs) throw new Error('call reset() first');
    return this.obs;
  }

  private pilotAction(id: string): Float32Array {
    const obs = this.requireEpisode();
    return this.pilot(id).act(obs, sensorFrame(this.env, obs));
  }

  private episodeInfo() {
    const spec = this.env.building!;
    return {
      maxSteps: this.maxSteps,
      speedH: this.ctx!.speedH,
      dynamics: this.ctx!.dynamics,
      floors: spec.floors.length,
      rooms: spec.floors.reduce((n, f) => n + f.rooms.length, 0),
      reachableVoxels: spec.reachableVoxels,
    };
  }

  handle(h: Header, payload: Buffer): [Record<string, unknown>, Uint8Array?] {
    switch (h.cmd) {
      case 'hello':
        return [
          {
            protocol: PROTOCOL,
            obsVersion: OBS_VERSION,
            obsDim: OBS_DIM,
            obs: OBS,
            floorHeight: FLOOR_HEIGHT,
            actDim: ACT_DIM,
            benchVersion: BENCH_VERSION,
            tasks: BENCH_TASKS,
            seeds: BENCH_SEEDS,
            lighting: LIGHTING_PRESETS,
            pilots: [...Object.keys(BASELINES), 'policy', 'guarded'],
          },
        ];
      case 'reset': {
        const seed = num(h, 'seed', 0);
        if (!Number.isInteger(seed) || seed < 0 || seed > 0xffffffff) throw new Error('seed must be an integer in [0, 2^32)');
        const task = taskFrom(h);
        if (task.plans) throw new Error('the apartments task flies fixed plans: use bench_reset');
        const { spec, envSeed, cfg } = benchEpisodeSetup(task, { seed });
        if (h.maxSteps !== undefined && h.maxSteps !== null) cfg.maxSteps = num(h, 'maxSteps', cfg.maxSteps);
        this.bench = null;
        const obs = this.env.reset(spec, envSeed, cfg);
        this.startEpisode(obs, { speedH: task.speedH, maxSteps: cfg.maxSteps, dynamics: task.dynamics });
        return [{ info: { task: task.id, seed, ...this.episodeInfo() } }, f32(obs)];
      }
      case 'bench_reset': {
        const { task, index: ti } = findTask(h.task);
        const split = h.split as BenchSplit;
        if (split !== 'dev' && split !== 'test') throw new Error('split must be "dev" or "test"');
        const runs = benchRuns(split, ti, task.plans ? plans() : undefined);
        const i = num(h, 'index', 0);
        if (!Number.isInteger(i) || i < 0 || i >= runs.length) throw new Error(`index must be in [0, ${runs.length})`);
        const run = runs[i]!;
        this.bench = new BenchEpisodeRecorder(this.env, task, run);
        this.startEpisode(this.bench.obs, this.bench.ctx);
        const info = { task: task.id, split, index: i, flights: runs.length, seed: run.seed, ...(run.plan ? { plan: run.plan.record.id } : {}), ...this.episodeInfo() };
        return [{ info }, f32(this.bench.obs)];
      }
      case 'step': {
        this.requireEpisode();
        let action: Float32Array;
        if (typeof h.pilot === 'string') action = this.pilotAction(h.pilot);
        else {
          if (payload.byteLength !== ACT_DIM * 4) throw new Error(`step needs ${ACT_DIM} float32 actions`);
          action = new Float32Array(ACT_DIM);
          for (let k = 0; k < ACT_DIM; k++) {
            const v = payload.readFloatLE(k * 4);
            // NaN would poison the dynamics; reject it like any bad input.
            if (!Number.isFinite(v)) throw new Error('action must be finite');
            action[k] = Math.max(-1, Math.min(1, v));
          }
        }
        const s: StepResult = this.env.step(action);
        this.steps += 1;
        this.obs = s.obs;
        // Gymnasium: truncated = cut short (stall limit or out of time), terminated = an end state (covered, crashed, left the building).
        const truncated = !s.info.success && (s.truncated || this.steps >= this.maxSteps);
        const terminated = s.done && !truncated;
        const info: Record<string, unknown> = {
          coverage: s.info.coverage,
          collisions: s.info.collisions,
          floor: s.info.floor,
          newRoom: s.info.newRoom,
          newFloor: s.info.newFloor,
          success: s.info.success,
          energyWh: s.info.energyWh,
        };
        if (h.pilot !== undefined) info.action = Array.from(action);
        let over = terminated || truncated;
        if (this.bench) {
          over = this.bench.record(s) || over;
          if (over) {
            info.episode = this.bench.finish();
            this.bench = null;
          }
        }
        if (over) this.obs = null;
        return [{ reward: s.reward, terminated, truncated: truncated || (over && !terminated), info }, f32(s.obs)];
      }
      case 'act':
        if (typeof h.pilot !== 'string') throw new Error('act needs a pilot id');
        return [{}, f32(this.pilotAction(h.pilot))];
      case 'frame': {
        const obs = this.requireEpisode();
        const f = sensorFrame(this.env, obs);
        const lidar = Float32Array.from(f.lidar);
        const head = { vw: f.vw, vd: f.vd, floors: f.floors, floorHeight: f.floorHeight, pos: Array.from(f.pos), yaw: f.yaw, alt: f.alt, vel: f.vel, lidarRays: lidar.length };
        const out = new Uint8Array(lidar.byteLength + f.seen.byteLength + f.visited.byteLength);
        out.set(new Uint8Array(lidar.buffer), 0);
        out.set(f.seen, lidar.byteLength);
        out.set(f.visited, lidar.byteLength + f.seen.byteLength);
        return [head, out];
      }
      case 'summarize':
        return [{ summary: summarize(h.episodes as BenchEpisode[]) }];
      default:
        throw new Error(`unknown command "${h.cmd}"`);
    }
  }
}

function f32(a: Float32Array): Uint8Array {
  return new Uint8Array(a.buffer, a.byteOffset, a.byteLength);
}

function send(head: Record<string, unknown>, payload?: Uint8Array): void {
  const json = Buffer.from(JSON.stringify(head));
  const body = payload ?? new Uint8Array(0);
  const frame = Buffer.allocUnsafe(8 + json.length + body.byteLength);
  frame.writeUInt32LE(4 + json.length + body.byteLength, 0);
  frame.writeUInt32LE(json.length, 4);
  json.copy(frame, 8);
  frame.set(body, 8 + json.length);
  process.stdout.write(frame);
}

async function main(): Promise<void> {
  // Anything the sim logs must not corrupt the protocol stream.
  console.log = console.info = console.warn = console.debug = (...args: unknown[]) => process.stderr.write(`${args.join(' ')}\n`);
  const session = new Session();
  await session.init();
  let buf: Buffer = Buffer.alloc(0);
  process.stdin.on('data', (chunk: Buffer) => {
    buf = buf.length ? Buffer.concat([buf, chunk]) : chunk;
    while (buf.length >= 4) {
      const total = buf.readUInt32LE(0);
      if (buf.length < 4 + total) break;
      const jsonLen = buf.readUInt32LE(4);
      const header = JSON.parse(buf.subarray(8, 8 + jsonLen).toString('utf8')) as Header;
      const payload = buf.subarray(8 + jsonLen, 4 + total);
      buf = buf.subarray(4 + total);
      if (header.cmd === 'close') process.exit(0);
      try {
        const [head, out] = session.handle(header, payload);
        send({ ok: true, ...head }, out);
      } catch (err) {
        send({ ok: false, error: err instanceof Error ? err.message : String(err) });
      }
    }
  });
  process.stdin.on('end', () => process.exit(0));
}

void main();

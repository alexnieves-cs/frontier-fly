// A sandbox for submitted FrontierBench pilots. The pilot's module is loaded and run in a worker
// thread (pilot-worker.mjs) with an empty environment; the scorer and the env stay in this thread.
//
// What it stops: a pilot reading the building it flies, the env's state or the scorer's objects,
// or changing the scorer's code, because none of those live in the worker's heap. The pilot gets a
// copy of the observation and the sensor frame each step and returns four numbers.
//
// What it does not stop: the test seeds are public, so a pilot can still be written around answers
// worked out offline, and a worker can still read files. A maintainer reads every entry's code.
//
// `act` is synchronous in the bench, so the two threads talk over shared memory: the scorer copies
// the step in, wakes the worker and waits (Atomics) for the action.
import { fileURLToPath } from 'node:url';
import { MessageChannel, receiveMessageOnPort, Worker } from 'node:worker_threads';
import { resolve } from 'node:path';
import type { BenchContext, BenchPilot } from '../../src/bench/pilot';
import type { SensorFrame } from '../../src/sim/sensorPlanner';

const META = { speedH: 0, maxSteps: 1, dynamics: 2, cells: 3, hasOccupied: 4, vw: 5, vd: 6, floors: 7, floorHeight: 8, pos: 9, yaw: 12, lidarLen: 13, alt: 14, hasVel: 15, vel: 16, obsLen: 18 } as const;
const CMD = { reset: 1, act: 2, quit: 3 } as const;
const OBS_CAP = 1 << 14;
const LIDAR_CAP = 1 << 12;
const GRID_CAP = 1 << 21;
/** A pilot gets this long to load, and this long for any one step, before the run fails. */
const LOAD_MS = 120_000;
const STEP_MS = 30_000;

export interface SandboxedPilot extends BenchPilot {
  /** Stop the worker. The pilot cannot be used afterwards. */
  close(): Promise<void>;
}

export async function sandboxedPilot(file: string, opts: { stepMs?: number } = {}): Promise<SandboxedPilot> {
  const stepMs = opts.stepMs ?? STEP_MS;
  const ctrlBuf = new SharedArrayBuffer(4 * Int32Array.BYTES_PER_ELEMENT);
  const metaBuf = new SharedArrayBuffer(32 * Float64Array.BYTES_PER_ELEMENT);
  const obsBuf = new SharedArrayBuffer(OBS_CAP * Float32Array.BYTES_PER_ELEMENT);
  const lidarBuf = new SharedArrayBuffer(LIDAR_CAP * Float64Array.BYTES_PER_ELEMENT);
  const gridBuf = new SharedArrayBuffer(3 * GRID_CAP);
  const actionBuf = new SharedArrayBuffer(4 * Float32Array.BYTES_PER_ELEMENT);
  const ctrl = new Int32Array(ctrlBuf);
  const meta = new Float64Array(metaBuf);
  const obsShared = new Float32Array(obsBuf);
  const lidarShared = new Float64Array(lidarBuf);
  const grids = new Uint8Array(gridBuf);
  const action = new Float32Array(actionBuf);
  const { port1, port2 } = new MessageChannel();
  const worker = new Worker(fileURLToPath(new URL('./pilot-worker.mjs', import.meta.url)), {
    workerData: { port: port2, file: resolve(file), ctrl: ctrlBuf, meta: metaBuf, obs: obsBuf, lidar: lidarBuf, grids: gridBuf, action: actionBuf, layout: { meta: META, cmd: CMD, gridCap: GRID_CAP } },
    transferList: [port2],
    // No secrets, tokens or CI variables reach the pilot.
    env: { PATH: process.env.PATH ?? '', NODE_ENV: process.env.NODE_ENV ?? 'production' },
    resourceLimits: { maxOldGenerationSizeMb: 4096 },
  });
  let dead: Error | null = null;
  worker.on('error', (e) => (dead = e instanceof Error ? e : new Error(String(e))));

  const first = await new Promise<{ ready?: boolean; name?: string; error?: string }>((done) => {
    const timer = setTimeout(() => done({ error: `the pilot did not load within ${LOAD_MS / 1000} s` }), LOAD_MS);
    const settle = (m: { ready?: boolean; name?: string; error?: string }) => {
      clearTimeout(timer);
      done(m);
    };
    port1.once('message', settle);
    worker.once('error', (e) => settle({ error: String(e) }));
    worker.once('exit', (code) => settle({ error: `the pilot's worker exited with code ${code} while loading` }));
  });
  if (!first.ready) {
    await worker.terminate();
    throw new Error(`${file}: ${first.error ?? 'the pilot failed to load'}`);
  }
  // From here the port is read synchronously (receiveMessageOnPort), so it must not keep the loop alive.
  port1.unref();
  worker.unref();

  const call = (cmd: number): void => {
    if (dead) throw dead;
    const before = Atomics.load(ctrl, 1);
    Atomics.store(ctrl, 3, cmd);
    Atomics.add(ctrl, 0, 1);
    Atomics.notify(ctrl, 0);
    if (Atomics.wait(ctrl, 1, before, stepMs) === 'timed-out') {
      void worker.terminate();
      dead = new Error(`${file}: the pilot took longer than ${stepMs / 1000} s on one step`);
      throw dead;
    }
    if (Atomics.load(ctrl, 2) !== 0) {
      const m = receiveMessageOnPort(port1)?.message as { error?: string } | undefined;
      void worker.terminate();
      dead = new Error(`${file}: the pilot threw: ${m?.error ?? 'unknown error'}`);
      throw dead;
    }
  };

  const grid = (src: Uint8Array, slot: number): void => {
    if (src.length > GRID_CAP) throw new Error(`building too large for the pilot sandbox (${src.length} voxels)`);
    grids.set(src, slot * GRID_CAP);
  };

  return {
    name: first.name ?? 'unnamed',
    reset(ctx: BenchContext): void {
      meta[META.speedH] = ctx.speedH;
      meta[META.maxSteps] = ctx.maxSteps;
      meta[META.dynamics] = ctx.dynamics === 'quad' ? 1 : 0;
      call(CMD.reset);
    },
    act(obs: Float32Array, frame: SensorFrame): Float32Array {
      if (obs.length > OBS_CAP || frame.lidar.length > LIDAR_CAP) throw new Error('observation too large for the pilot sandbox');
      obsShared.set(obs);
      meta[META.obsLen] = obs.length;
      grid(frame.seen, 0);
      grid(frame.visited, 1);
      meta[META.hasOccupied] = frame.occupied ? 1 : 0;
      if (frame.occupied) grid(frame.occupied, 2);
      meta[META.cells] = frame.seen.length;
      meta[META.vw] = frame.vw;
      meta[META.vd] = frame.vd;
      meta[META.floors] = frame.floors;
      meta[META.floorHeight] = frame.floorHeight;
      meta[META.pos] = frame.pos[0];
      meta[META.pos + 1] = frame.pos[1];
      meta[META.pos + 2] = frame.pos[2];
      meta[META.yaw] = frame.yaw;
      for (let i = 0; i < frame.lidar.length; i++) lidarShared[i] = frame.lidar[i]!;
      meta[META.lidarLen] = frame.lidar.length;
      meta[META.alt] = frame.alt;
      meta[META.hasVel] = frame.vel ? 1 : 0;
      if (frame.vel) {
        meta[META.vel] = frame.vel[0];
        meta[META.vel + 1] = frame.vel[1];
      }
      call(CMD.act);
      return Float32Array.from(action);
    },
    async close(): Promise<void> {
      if (!dead) {
        Atomics.store(ctrl, 3, CMD.quit);
        Atomics.add(ctrl, 0, 1);
        Atomics.notify(ctrl, 0);
      }
      await worker.terminate();
    },
  };
}

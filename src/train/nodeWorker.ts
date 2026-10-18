/**
 * worker_threads side of NodeRolloutPool. This file is not loaded directly:
 * nodePool.ts bundles it (with any job modules) into one .mjs with esbuild and
 * starts that, so it runs as plain JS without a TS loader in the worker.
 */
import { parentPort } from 'node:worker_threads';
import type { RunningNormState } from '../rl/normalize';
import type { WorldPoolOptions } from '../sim/worldPool';
import { RolloutCore, type CollectPpoOptions, type EvalJob } from './rolloutCore';

export { registerJobs } from './rolloutCore';

export interface InitMsg {
  type: 'init';
  workerIndex: number;
  nEnvs: number;
  seed: number;
  obsDim?: number;
  hidden?: number[];
  worldPool?: WorldPoolOptions;
}

export type WorkerRequest =
  | InitMsg
  | { type: 'weights'; weights: ArrayBuffer[]; norm: RunningNormState | null }
  | { type: 'collectPpo'; opts: CollectPpoOptions }
  | { type: 'evaluate'; jobs: EvalJob[] }
  | { type: 'job'; name: string; payload: unknown }
  | { type: 'close' };

export interface WorkerReply {
  id: number;
  ok: boolean;
  result?: unknown;
  error?: string;
}

/** Every ArrayBuffer reachable from `value` (typed-array views included), for zero-copy transfer. */
export function collectTransfers(value: unknown, out: Set<ArrayBuffer> = new Set(), depth = 0): ArrayBuffer[] {
  if (value == null || depth > 6) return [...out];
  if (ArrayBuffer.isView(value)) {
    if (value.buffer instanceof ArrayBuffer) out.add(value.buffer);
  } else if (value instanceof ArrayBuffer) {
    out.add(value);
  } else if (Array.isArray(value)) {
    for (const item of value) collectTransfers(item, out, depth + 1);
  } else if (typeof value === 'object') {
    for (const item of Object.values(value as Record<string, unknown>)) collectTransfers(item, out, depth + 1);
  }
  return [...out];
}

export function startWorker(): void {
  const port = parentPort;
  if (!port) throw new Error('nodeWorker must run inside worker_threads');
  let core: RolloutCore | null = null;
  // Requests run strictly in order; the main thread never overlaps two on one worker anyway.
  let chain: Promise<void> = Promise.resolve();

  const handle = async (msg: WorkerRequest): Promise<unknown> => {
    if (msg.type === 'init') {
      core = await RolloutCore.create({
        workerIndex: msg.workerIndex,
        nEnvs: msg.nEnvs,
        seed: msg.seed,
        obsDim: msg.obsDim,
        hidden: msg.hidden,
        worldPool: msg.worldPool,
      });
      return { obsDim: core.obsDim, worlds: core.worlds.size };
    }
    if (!core) throw new Error(`worker got "${msg.type}" before init`);
    switch (msg.type) {
      case 'weights':
        core.setWeights(msg.weights, msg.norm);
        return null;
      case 'collectPpo':
        return core.collectPpo(msg.opts);
      case 'evaluate':
        return core.evaluate(msg.jobs);
      case 'job':
        return await core.runJob(msg.name, msg.payload);
      case 'close':
        core.dispose();
        core = null;
        return null;
    }
  };

  port.on('message', (raw: { id: number; msg: WorkerRequest }) => {
    chain = chain.then(async () => {
      try {
        const result = await handle(raw.msg);
        const reply: WorkerReply = { id: raw.id, ok: true, result };
        try {
          // Results are moved, not copied: a job must not return views of state it keeps using.
          port.postMessage(reply, collectTransfers(result));
        } catch {
          // e.g. a view on WebAssembly memory, which cannot be transferred: fall back to a copy.
          port.postMessage(reply);
        }
        if (raw.msg.type === 'close') port.close();
      } catch (error) {
        const reply: WorkerReply = { id: raw.id, ok: false, error: error instanceof Error ? (error.stack ?? error.message) : String(error) };
        port.postMessage(reply);
      }
    });
  });
}

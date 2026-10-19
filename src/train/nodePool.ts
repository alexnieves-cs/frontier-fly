/**
 * Parallel rollouts for headless Node training on worker_threads.
 *
 * Each worker owns `envsPerWorker` ExplorerEnvs, an InferPolicy (batched
 * inference across its envs, SIMD gemm when available) and a WorldPool of
 * pre-built buildings, so resets skip generateBuilding + the Rapier rebuild.
 *
 * Loading TS in workers: worker_threads cannot run .ts and vite-node/vitest
 * transforms do not reach into worker threads, so init() bundles
 * src/train/nodeWorker.ts (plus job modules) into one ESM file with esbuild
 * (already installed as a vite dependency) and starts the workers from it.
 * The bundle is content-hashed under node_modules/.cache/frontier-fly-workers/
 * so npm packages (Rapier, TF.js if a job module pulls it in) stay external and
 * resolve from the repo's node_modules. Works the same from `vite-node
 * scripts/x.ts`, vitest, or a script bundled with esbuild (pass `root` if the
 * bundle lives outside the repo). Bundling takes ~0.2 s.
 */
import { createHash } from 'node:crypto';
import { existsSync, mkdirSync, readFileSync, renameSync, writeFileSync } from 'node:fs';
import { availableParallelism } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { Worker } from 'node:worker_threads';
import type { RunningNormState } from '../rl/normalize';
import type { WorldPoolOptions } from '../sim/worldPool';
import type { CollectPpoOptions, EpisodeStat, EvalJob, EvalResult, ObsMoments, PpoChunk } from './rolloutCore';
import type { WorkerReply, WorkerRequest } from './nodeWorker';

export interface NodeRolloutPoolOptions {
  /** Worker threads. Default os.availableParallelism(). */
  workers?: number;
  /** Envs per worker, stepped in lockstep with one batched forward pass. Default 8. */
  envsPerWorker?: number;
  /** Base seed; worker i derives its policy, action and world-pool streams from it. Default 1. */
  seed?: number;
  /** Must equal the env OBS_DIM (checked in the worker). Default OBS_DIM. */
  obsDim?: number;
  hidden?: number[];
  /** Per-worker WorldPool options (seed is offset per worker so pools are disjoint). */
  worldPool?: WorldPoolOptions;
  /** Extra modules (paths, absolute or relative to `root`) that export `jobs: Record<string, JobFn>`. */
  jobModules?: string[];
  /** Register src/rl/dagger.ts `collectDagger` as job "dagger" when that file exists. Default true. */
  autoDagger?: boolean;
  /** Repo root used to find src/ and node_modules. Default: found from this file, else process.cwd(). */
  root?: string;
}

/** Concatenation of every worker's PpoChunk: env-major, workers in order. */
export interface PpoBatchResult extends Omit<PpoChunk, 'ms' | 'stepsPerSec' | 'episodes' | 'obsMoments'> {
  /** Every finished episode, `env` is the global env index (worker * envsPerWorker + local). */
  episodes: EpisodeStat[];
  /** Merged raw-observation moments (when a norm was set); feed to RunningNorm.merge. */
  obsMoments: ObsMoments | null;
  perWorker: { stepsPerSec: number; ms: number; steps: number }[];
  /** Total env steps / wall-clock time of the call. */
  stepsPerSec: number;
  wallMs: number;
}

function findRoot(start: string): string | null {
  let dir = start;
  for (let i = 0; i < 12; i++) {
    const pkg = join(dir, 'package.json');
    if (existsSync(pkg) && existsSync(join(dir, 'src', 'train', 'nodeWorker.ts'))) return dir;
    const up = dirname(dir);
    if (up === dir) break;
    dir = up;
  }
  return null;
}

function repoRoot(explicit?: string): string {
  if (explicit) return resolve(explicit);
  let here: string | null = null;
  try {
    here = dirname(fileURLToPath(import.meta.url));
  } catch {
    here = null;
  }
  return (here && findRoot(here)) ?? findRoot(process.cwd()) ?? process.cwd();
}

const bundles = new Map<string, Promise<string>>();

/**
 * Bundles the worker entry (plus job modules) into an ESM file and returns
 * its path. Content-addressed, so concurrent pools and processes share files.
 */
export async function bundleWorker(opts: { root?: string; jobModules?: string[]; autoDagger?: boolean } = {}): Promise<string> {
  const root = repoRoot(opts.root);
  const toImport = (p: string) => JSON.stringify(resolve(root, p).replaceAll('\\', '/'));
  const lines = [`import { startWorker, registerJobs } from ${toImport('src/train/nodeWorker.ts')};`];
  const daggerPath = join(root, 'src', 'rl', 'dagger.ts');
  if ((opts.autoDagger ?? true) && existsSync(daggerPath)) {
    lines.push(`import * as daggerModule from ${toImport('src/rl/dagger.ts')};`);
    lines.push(`import { makeDaggerJob } from ${toImport('src/train/daggerJob.ts')};`);
    lines.push(`const collectDagger = daggerModule['collect' + 'Dagger'];`);
    lines.push(`if (typeof collectDagger === 'function') registerJobs({ dagger: makeDaggerJob(collectDagger) });`);
  }
  (opts.jobModules ?? []).forEach((mod, i) => {
    lines.push(`import * as jobs${i} from ${toImport(mod)};`);
    lines.push(`registerJobs(jobs${i}.jobs);`);
  });
  lines.push('startWorker();');
  const entry = lines.join('\n');
  const cacheKey = `${root}\n${entry}`;
  const cached = bundles.get(cacheKey);
  if (cached) return cached;
  const job = (async () => {
    const esbuild = await import('esbuild');
    const built = await esbuild.build({
      stdin: { contents: entry, resolveDir: root, sourcefile: 'frontier-node-worker.js', loader: 'js' },
      bundle: true,
      platform: 'node',
      format: 'esm',
      target: 'node20',
      packages: 'external',
      write: false,
      logLevel: 'silent',
    });
    const text = built.outputFiles[0]!.text;
    const hash = createHash('sha256').update(text).digest('hex').slice(0, 16);
    const dir = join(root, 'node_modules', '.cache', 'frontier-fly-workers');
    mkdirSync(dir, { recursive: true });
    const file = join(dir, `worker-${hash}.mjs`);
    if (!existsSync(file) || readFileSync(file, 'utf8') !== text) {
      const tmp = `${file}.${process.pid}.${Math.random().toString(36).slice(2)}.tmp`;
      writeFileSync(tmp, text);
      renameSync(tmp, file);
    }
    return file;
  })();
  bundles.set(cacheKey, job);
  job.catch(() => bundles.delete(cacheKey));
  return job;
}

interface Pending {
  resolve: (value: unknown) => void;
  reject: (error: Error) => void;
}

class WorkerHandle {
  readonly worker: Worker;
  private nextId = 1;
  private readonly pending = new Map<number, Pending>();
  private failure: Error | null = null;

  constructor(file: string, readonly index: number) {
    this.worker = new Worker(file, { name: `frontier-rollout-${index}` });
    this.worker.on('message', (reply: WorkerReply) => {
      const p = this.pending.get(reply.id);
      if (!p) return;
      this.pending.delete(reply.id);
      if (reply.ok) p.resolve(reply.result);
      else p.reject(new Error(`rollout worker ${index}: ${reply.error}`));
    });
    const fail = (error: Error) => {
      this.failure = error;
      for (const p of this.pending.values()) p.reject(error);
      this.pending.clear();
    };
    this.worker.on('error', (error) => fail(error instanceof Error ? error : new Error(String(error))));
    this.worker.on('exit', (code) => {
      if (this.pending.size > 0) fail(new Error(`rollout worker ${index} exited with code ${code}`));
    });
  }

  request<T>(msg: WorkerRequest, transfer: ArrayBuffer[] = []): Promise<T> {
    if (this.failure) return Promise.reject(this.failure);
    const id = this.nextId++;
    return new Promise<T>((resolvePromise, reject) => {
      this.pending.set(id, { resolve: resolvePromise as (v: unknown) => void, reject });
      this.worker.postMessage({ id, msg }, transfer);
    });
  }
}

function concatF32(parts: Float32Array[]): Float32Array {
  const out = new Float32Array(parts.reduce((n, p) => n + p.length, 0));
  let at = 0;
  for (const p of parts) {
    out.set(p, at);
    at += p.length;
  }
  return out;
}

function concatU8(parts: Uint8Array[]): Uint8Array {
  const out = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let at = 0;
  for (const p of parts) {
    out.set(p, at);
    at += p.length;
  }
  return out;
}

function mergeMoments(parts: (ObsMoments | null)[]): ObsMoments | null {
  let acc: ObsMoments | null = null;
  for (const m of parts) {
    if (!m || m.count <= 0) continue;
    if (!acc) {
      acc = { count: m.count, mean: Float64Array.from(m.mean), m2: Float64Array.from(m.m2) };
      continue;
    }
    const total = acc.count + m.count;
    for (let d = 0; d < acc.mean.length; d++) {
      const delta = m.mean[d]! - acc.mean[d]!;
      acc.mean[d] = acc.mean[d]! + (delta * m.count) / total;
      acc.m2[d] = acc.m2[d]! + m.m2[d]! + (delta * delta * acc.count * m.count) / total;
    }
    acc.count = total;
  }
  return acc;
}

export class NodeRolloutPool {
  readonly workers: number;
  readonly envsPerWorker: number;
  private readonly opts: NodeRolloutPoolOptions;
  private handles: WorkerHandle[] = [];
  private ready: Promise<void> | null = null;
  private obsDimValue = 0;

  constructor(opts: NodeRolloutPoolOptions = {}) {
    this.opts = opts;
    this.workers = Math.max(1, opts.workers ?? availableParallelism());
    this.envsPerWorker = Math.max(1, opts.envsPerWorker ?? 8);
  }

  get envsTotal(): number {
    return this.workers * this.envsPerWorker;
  }

  get obsDim(): number {
    return this.obsDimValue;
  }

  /** Bundles the worker, starts the threads and pre-builds every worker's WorldPool. Idempotent. */
  init(): Promise<void> {
    this.ready ??= (async () => {
      const file = await bundleWorker({ root: this.opts.root, jobModules: this.opts.jobModules, autoDagger: this.opts.autoDagger });
      this.handles = Array.from({ length: this.workers }, (_, i) => new WorkerHandle(file, i));
      const replies = await Promise.all(
        this.handles.map((h) =>
          h.request<{ obsDim: number }>({
            type: 'init',
            workerIndex: h.index,
            nEnvs: this.envsPerWorker,
            seed: this.opts.seed ?? 1,
            obsDim: this.opts.obsDim,
            hidden: this.opts.hidden,
            worldPool: this.opts.worldPool,
          }),
        ),
      );
      this.obsDimValue = replies[0]!.obsDim;
    })();
    return this.ready;
  }

  private async live(): Promise<WorkerHandle[]> {
    if (!this.ready) throw new Error('NodeRolloutPool.init() was not called');
    await this.ready;
    return this.handles;
  }

  /** Same weights (InferPolicy buffer order) on every worker; `norm` is applied to observations before acting. */
  async setWeights(buffers: ArrayBuffer[], norm: RunningNormState | null = null): Promise<void> {
    const handles = await this.live();
    await Promise.all(handles.map((h) => h.request({ type: 'weights', weights: buffers, norm })));
  }

  /** stepsPerEnv steps on every env of every worker, in parallel; results concatenated worker-major. */
  async collectPpo(opts: CollectPpoOptions): Promise<PpoBatchResult> {
    const handles = await this.live();
    const t0 = performance.now();
    const chunks = await Promise.all(handles.map((h) => h.request<PpoChunk>({ type: 'collectPpo', opts })));
    const wallMs = performance.now() - t0;
    const n = chunks.reduce((s, c) => s + c.n, 0);
    let envBase = 0;
    const episodes: EpisodeStat[] = [];
    for (const c of chunks) {
      for (const e of c.episodes) episodes.push({ ...e, env: e.env + envBase });
      envBase += c.nEnvs;
    }
    return {
      obs: concatF32(chunks.map((c) => c.obs)),
      actions: concatF32(chunks.map((c) => c.actions)),
      logp: concatF32(chunks.map((c) => c.logp)),
      values: concatF32(chunks.map((c) => c.values)),
      rewards: concatF32(chunks.map((c) => c.rewards)),
      dones: concatU8(chunks.map((c) => c.dones)),
      truncated: concatU8(chunks.map((c) => c.truncated)),
      bootstrap: concatF32(chunks.map((c) => c.bootstrap)),
      n,
      nEnvs: envBase,
      stepsPerEnv: chunks[0]?.stepsPerEnv ?? 0,
      obsDim: chunks[0]?.obsDim ?? this.obsDimValue,
      episodes,
      obsMoments: mergeMoments(chunks.map((c) => c.obsMoments)),
      perWorker: chunks.map((c) => ({ stepsPerSec: c.stepsPerSec, ms: c.ms, steps: c.n })),
      stepsPerSec: (n * 1000) / Math.max(1e-3, wallMs),
      wallMs,
    };
  }

  /**
   * Coverage episodes (src/eval/coverage.ts protocol), spread over workers by
   * estimated cost and run in lockstep per worker. Results are in job order
   * and identical to running coverEpisode sequentially.
   */
  async evaluate(jobs: EvalJob[]): Promise<EvalResult[]> {
    const handles = await this.live();
    const cost = (job: EvalJob) => (job.stage <= 1 ? 1000 : 2400) * (job.stage <= 1 ? 64 : 400);
    const order = jobs.map((job, i) => ({ job, i })).sort((a, b) => cost(b.job) - cost(a.job) || a.i - b.i);
    const load = handles.map(() => 0);
    const assigned: { job: EvalJob; i: number }[][] = handles.map(() => []);
    for (const item of order) {
      let w = 0;
      for (let k = 1; k < load.length; k++) if (load[k]! < load[w]!) w = k;
      load[w]! += cost(item.job);
      assigned[w]!.push(item);
    }
    const results: EvalResult[] = new Array(jobs.length);
    await Promise.all(
      handles.map(async (h, w) => {
        const mine = assigned[w]!;
        if (mine.length === 0) return;
        const out = await h.request<EvalResult[]>({ type: 'evaluate', jobs: mine.map((m) => m.job) });
        mine.forEach((m, k) => (results[m.i] = out[k]!));
      }),
    );
    return results;
  }

  /** Run a registered job on one worker. Typed arrays in the result are transferred. */
  async runJob<R = unknown>(name: string, payload: unknown, worker = 0, transfer: ArrayBuffer[] = []): Promise<R> {
    const handles = await this.live();
    const h = handles[worker];
    if (!h) throw new Error(`no worker ${worker} (pool has ${handles.length})`);
    return h.request<R>({ type: 'job', name, payload }, transfer);
  }

  /** Run a registered job on every worker in parallel; `payload` may be a function of the worker index. */
  async runJobAll<R = unknown>(name: string, payload: unknown | ((worker: number) => unknown)): Promise<R[]> {
    const handles = await this.live();
    return Promise.all(
      handles.map((h) =>
        h.request<R>({ type: 'job', name, payload: typeof payload === 'function' ? (payload as (w: number) => unknown)(h.index) : payload }),
      ),
    );
  }

  /** Frees worker state and terminates the threads. */
  async close(): Promise<void> {
    if (!this.ready) return;
    const handles = this.handles;
    this.handles = [];
    this.ready = null;
    await Promise.all(
      handles.map(async (h) => {
        try {
          await Promise.race([h.request({ type: 'close' }), new Promise((r) => setTimeout(r, 2000))]);
        } catch {
          // already dead
        }
        await h.worker.terminate();
      }),
    );
  }
}

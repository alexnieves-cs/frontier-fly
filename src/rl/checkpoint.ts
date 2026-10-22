/**
 * Checkpoints as JSON or packed binary, plus an IndexedDB helper. IndexedDB is
 * touched only through globalThis.
 *
 * The binary form is what ships: a magic tag, a JSON header holding everything
 * but the weights, then the weights as little-endian float32. The weights are
 * float32 in memory, so the two forms decode to identical numbers; the binary
 * one is about a sixth of the size.
 */
import { OBS_DIM, OBS_VERSION } from '../shared/constants';

export interface Checkpoint {
  version: number;
  /** Observation layout version (src/sim/OBS_LAYOUT.md). Files without it are v1. */
  obsVersion: number;
  obsDim: number;
  actDim: number;
  hidden: number[];
  weights: number[][];
  obsMean: number[];
  obsVar: number[];
  obsCount: number;
  rewVar: number;
  stage: number;
  /** GRU memory size (src/rl/memPolicy.ts); absent or 0 for the feed-forward policy. Adds 4 weight arrays. */
  memDim?: number;
  /** Only the policy's 7 arrays, without the value head: enough to fly, not to train. */
  actorOnly?: boolean;
}

const WEIGHT_BUFFERS = 13;
/** The policy's arrays (w1 to logStd); the value head follows them. */
const ACTOR_BUFFERS = 7;
/** Extra arrays of a checkpoint with a GRU memory. */
const MEM_WEIGHT_BUFFERS = 4;
/** "FFW1": frontier-fly weights, format 1. */
const MAGIC = [0x46, 0x46, 0x57, 0x31];

/** The shipped checkpoint, relative to the site root (browser) or under public/ (Node). */
export const SHIPPED_CHECKPOINT_URL = 'checkpoints/pretrained.bin';
export const SHIPPED_CHECKPOINT_PATH = `public/${SHIPPED_CHECKPOINT_URL}`;
/**
 * The shipped checkpoint without its value head, which only training reads: the
 * pages that fly the policy load this, half the size. `npm run pack:policy` writes it.
 */
export const SHIPPED_POLICY_URL = 'checkpoints/policy.bin';
export const SHIPPED_POLICY_PATH = `public/${SHIPPED_POLICY_URL}`;
const DB_NAME = 'frontier-fly';
const STORE = 'checkpoints';
const DB_VERSION = 1;

/** Shortest decimal that reads back as the same float32 (at most 9 significant digits). */
function shortestFloat32(v: number): number {
  const target = Math.fround(v);
  for (let p = 1; p < 9; p++) {
    const s = Number(v.toPrecision(p));
    if (Math.fround(s) === target) return s;
  }
  return Number(v.toPrecision(9));
}

/** Weights are float32, so printing them with float64 precision only makes the file bigger. Lossless. */
export function serializeCheckpoint(c: Checkpoint): string {
  return JSON.stringify({ ...c, weights: c.weights.map((row) => row.map(shortestFloat32)) });
}

/**
 * Packed binary form: MAGIC, uint32 header length, the UTF-8 JSON header (the
 * checkpoint with `weightShapes` in place of `weights`), zero padding to a
 * 4-byte boundary, then every weight array as float32 in order.
 */
export function serializeCheckpointBinary(c: Checkpoint): Uint8Array {
  const { weights, ...rest } = c;
  const header = new TextEncoder().encode(JSON.stringify({ ...rest, weightShapes: weights.map((w) => w.length) }));
  const start = Math.ceil((8 + header.length) / 4) * 4;
  const total = weights.reduce((a, w) => a + w.length, 0);
  const out = new Uint8Array(start + total * 4);
  out.set(MAGIC, 0);
  new DataView(out.buffer).setUint32(4, header.length, true);
  out.set(header, 8);
  const view = new DataView(out.buffer, start);
  let k = 0;
  for (const w of weights) for (const v of w) view.setFloat32(4 * k++, v, true);
  return out;
}

/** The checkpoint without its value head (see `actorOnly`). */
export function actorOnly(c: Checkpoint): Checkpoint {
  if (c.memDim) throw new Error('actorOnly: checkpoints with a GRU memory are not supported');
  return { ...c, weights: c.weights.slice(0, ACTOR_BUFFERS), actorOnly: true };
}

/** The file contents to write for a path: packed binary for `.bin`, JSON otherwise. */
export function serializeCheckpointFor(path: string, c: Checkpoint): string | Uint8Array {
  return path.endsWith('.bin') ? serializeCheckpointBinary(c) : serializeCheckpoint(c);
}

/** True when the bytes start with the binary checkpoint tag. */
function isBinary(bytes: Uint8Array): boolean {
  return bytes.length >= 8 && MAGIC.every((b, i) => bytes[i] === b);
}

/**
 * Reads either form: JSON text, or the bytes of a JSON or binary file (a
 * fetch's arrayBuffer(), or readFileSync without an encoding).
 */
export function parseCheckpoint(data: string | ArrayBuffer | Uint8Array): Checkpoint {
  if (typeof data !== 'string') {
    const bytes = data instanceof Uint8Array ? data : new Uint8Array(data);
    if (!isBinary(bytes)) return parseCheckpoint(new TextDecoder().decode(bytes));
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    const headerLen = view.getUint32(4, true);
    if (8 + headerLen > bytes.length) throw new Error('invalid checkpoint: truncated header');
    let header: Record<string, unknown>;
    try {
      header = JSON.parse(new TextDecoder().decode(bytes.subarray(8, 8 + headerLen))) as Record<string, unknown>;
    } catch {
      throw new Error('invalid checkpoint: bad header');
    }
    const shapes = numList(header.weightShapes, 'weightShapes');
    let offset = Math.ceil((8 + headerLen) / 4) * 4;
    const weights = shapes.map((n) => {
      if (offset + 4 * n > bytes.length) throw new Error('invalid checkpoint: truncated weights');
      const row = new Array<number>(n);
      for (let i = 0; i < n; i++) row[i] = view.getFloat32(offset + 4 * i, true);
      offset += 4 * n;
      return row;
    });
    delete header.weightShapes;
    return fromRaw({ ...header, weights });
  }
  let raw: unknown;
  try {
    raw = JSON.parse(data);
  } catch {
    throw new Error('invalid checkpoint: not JSON');
  }
  return fromRaw(raw);
}

function fromRaw(raw: unknown): Checkpoint {
  if (raw == null || typeof raw !== 'object') throw new Error('invalid checkpoint: expected object');
  const o = raw as Record<string, unknown>;
  const hidden = numList(o.hidden, 'hidden');
  const weights = numMatrix(o.weights, 'weights');
  const memDim = o.memDim === undefined ? 0 : num(o.memDim, 'memDim');
  const actor = o.actorOnly === true;
  if (actor && memDim > 0) throw new Error('invalid checkpoint: actorOnly with a GRU memory');
  const expected = actor ? ACTOR_BUFFERS : WEIGHT_BUFFERS + (memDim > 0 ? MEM_WEIGHT_BUFFERS : 0);
  if (weights.length !== expected) {
    throw new Error(`invalid checkpoint: expected ${expected} weight arrays`);
  }
  return {
    version: num(o.version, 'version'),
    obsVersion: o.obsVersion === undefined ? 1 : num(o.obsVersion, 'obsVersion'),
    obsDim: num(o.obsDim, 'obsDim'),
    actDim: num(o.actDim, 'actDim'),
    hidden,
    weights,
    obsMean: numList(o.obsMean, 'obsMean'),
    obsVar: numList(o.obsVar, 'obsVar'),
    obsCount: num(o.obsCount, 'obsCount'),
    rewVar: num(o.rewVar, 'rewVar'),
    stage: num(o.stage, 'stage'),
    ...(memDim > 0 ? { memDim } : {}),
    ...(actor ? { actorOnly: true } : {}),
  };
}

/**
 * Null when the checkpoint can drive the current observation, otherwise a
 * human-readable reason (an old v1 file against a v2 build, say).
 */
export function checkpointMismatch(c: Pick<Checkpoint, 'obsDim' | 'obsVersion'>, obsDim = OBS_DIM, obsVersion = OBS_VERSION): string | null {
  if (c.obsVersion === obsVersion && c.obsDim === obsDim) return null;
  return `checkpoint is for observation v${c.obsVersion} (${c.obsDim} inputs) but this build uses v${obsVersion} (${obsDim} inputs)`;
}

function idbFactory(): IDBFactory | null {
  const idb = globalThis.indexedDB;
  if (idb == null) return null;
  return idb;
}

function openDb(): Promise<IDBDatabase | null> {
  const idb = idbFactory();
  if (!idb) return Promise.resolve(null);
  return new Promise((resolve, reject) => {
    const req = idb.open(DB_NAME, DB_VERSION);
    req.onupgradeneeded = () => {
      const db = req.result;
      if (!db.objectStoreNames.contains(STORE)) db.createObjectStore(STORE);
    };
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error ?? new Error('indexedDB open failed'));
  });
}

export async function saveCheckpointIdb(c: Checkpoint, key = 'frontier-fly'): Promise<void> {
  const db = await openDb();
  if (!db) return;
  try {
    await new Promise<void>((resolve, reject) => {
      const tx = db.transaction(STORE, 'readwrite');
      tx.objectStore(STORE).put(serializeCheckpoint(c), key);
      tx.oncomplete = () => resolve();
      tx.onerror = () => reject(tx.error ?? new Error('indexedDB save failed'));
      tx.onabort = () => reject(tx.error ?? new Error('indexedDB save aborted'));
    });
  } finally {
    db.close();
  }
}

export async function loadCheckpointIdb(key = 'frontier-fly'): Promise<Checkpoint | null> {
  const db = await openDb();
  if (!db) return null;
  try {
    const text = await new Promise<string | undefined>((resolve, reject) => {
      const tx = db.transaction(STORE, 'readonly');
      const req = tx.objectStore(STORE).get(key);
      req.onsuccess = () => {
        const value: unknown = req.result;
        resolve(typeof value === 'string' ? value : undefined);
      };
      req.onerror = () => reject(req.error ?? new Error('indexedDB load failed'));
    });
    if (text == null) return null;
    return parseCheckpoint(text);
  } finally {
    db.close();
  }
}

function num(value: unknown, label: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    throw new Error(`invalid checkpoint: ${label}`);
  }
  return value;
}

function numList(value: unknown, label: string): number[] {
  if (!Array.isArray(value) || value.some((v) => typeof v !== 'number' || !Number.isFinite(v))) {
    throw new Error(`invalid checkpoint: ${label}`);
  }
  return value as number[];
}

function numMatrix(value: unknown, label: string): number[][] {
  if (!Array.isArray(value)) throw new Error(`invalid checkpoint: ${label}`);
  return value.map((row, i) => numList(row, `${label}[${i}]`));
}

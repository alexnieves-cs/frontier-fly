/** JSON checkpoint plus an IndexedDB helper. IndexedDB is touched only through globalThis. */
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
}

const WEIGHT_BUFFERS = 13;
const DB_NAME = 'frontier-fly';
const STORE = 'checkpoints';
const DB_VERSION = 1;

export function serializeCheckpoint(c: Checkpoint): string {
  return JSON.stringify(c);
}

export function parseCheckpoint(text: string): Checkpoint {
  let raw: unknown;
  try {
    raw = JSON.parse(text);
  } catch {
    throw new Error('invalid checkpoint: not JSON');
  }
  if (raw == null || typeof raw !== 'object') throw new Error('invalid checkpoint: expected object');
  const o = raw as Record<string, unknown>;
  const hidden = numList(o.hidden, 'hidden');
  const weights = numMatrix(o.weights, 'weights');
  if (weights.length !== WEIGHT_BUFFERS) {
    throw new Error(`invalid checkpoint: expected ${WEIGHT_BUFFERS} weight arrays`);
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

/// <reference lib="webworker" />
import { checkpointMismatch, parseCheckpoint } from '../../rl/checkpoint';
import type { InferPolicy } from '../../rl/infer';
import type { RunningNorm } from '../../rl/normalize';
import { ACT_DIM, OBS_DIM, POLICY_DT } from '../../shared/constants';
import { RNG } from '../../shared/rng';
import { policyFromCheckpoint } from '../checkpointIo';
import { influenceFromActions, INPUT_BLOCKS, occlusionBatch } from '../inspector/influence';
import { RaceEngine } from './engine';
import type { PolicyFn } from './pilots';
import type { FromWorker, InspectFrame, LaneFrame, ToWorker } from './protocol';

/**
 * Flies the race off the main thread. The page sends a start and a pace; the
 * worker steps every lane in lockstep and posts a frame about 30 times a second.
 */

const scope = self as unknown as DedicatedWorkerGlobalScope;
const FRAME_MS = 33;
/** Wall-clock budget per tick when running flat out, so frames keep flowing. */
const MAX_TICK_MS = 28;
/** The inspector's attribution runs this often, in policy steps, to keep the sim cost flat. */
const INSPECT_EVERY = 2;

let net: { policy: InferPolicy; norm: RunningNorm; rawObs: boolean } | null = null;
let policyFn: PolicyFn | null = null;
let engine: RaceEngine | null = null;
let generation = 0;
let rate = 200;
let paused = false;
let inspectLane = 0;
let timer = 0;
let lastTick = 0;
let carry = 0;
let lastInspect: InspectFrame | null = null;
let throughput = 0;
const rng = new RNG(1);
const batch = new Float32Array((INPUT_BLOCKS.length + 1) * OBS_DIM);

function post(msg: FromWorker, transfer: Transferable[] = []): void {
  scope.postMessage(msg, transfer);
}

function normed(obs: Float32Array, n: number): Float32Array {
  return net!.rawObs ? obs : net!.norm.apply(obs, n);
}

async function init(url: string): Promise<void> {
  try {
    const response = await fetch(url);
    if (!response.ok) throw new Error(`checkpoint HTTP ${response.status}`);
    const checkpoint = parseCheckpoint(await response.arrayBuffer());
    const mismatch = checkpointMismatch(checkpoint);
    if (mismatch) throw new Error(mismatch);
    net = policyFromCheckpoint(checkpoint);
    // Deterministic: the mean action, as in the coverage study and the eval.
    policyFn = (obs) => net!.policy.actWith(normed(obs, 1), 1, rng, true).action.slice();
    post({ type: 'ready', policy: true, note: '' });
  } catch (error) {
    post({ type: 'ready', policy: false, note: error instanceof Error ? error.message : String(error) });
  }
}

async function start(msg: Extract<ToWorker, { type: 'start' }>): Promise<void> {
  const mine = ++generation;
  clearTimeout(timer);
  engine?.dispose();
  engine = null;
  lastInspect = null;
  rate = msg.rate;
  paused = false;
  carry = 0;
  try {
    const next = await new RaceEngine(msg.setup, policyFn).init();
    if (mine !== generation) {
      next.dispose();
      return;
    }
    engine = next;
    inspectLane = Math.min(inspectLane, engine.lanes.length - 1);
    post({ type: 'started', setup: msg.setup, spec: engine.spec, maxSteps: engine.maxSteps, maxCollisions: engine.maxCollisions, stepsPerSecond: 1 / POLICY_DT });
    lastTick = performance.now();
    sendFrame();
    timer = scope.setTimeout(tick, FRAME_MS);
  } catch (error) {
    post({ type: 'error', message: error instanceof Error ? error.message : String(error) });
  }
}

function tick(): void {
  if (!engine) return;
  const now = performance.now();
  const dt = Math.min(0.25, (now - lastTick) / 1000);
  lastTick = now;
  if (!paused && !engine.finished) {
    const t0 = performance.now();
    let n = 0;
    if (rate <= 0) {
      while (!engine.finished && performance.now() - t0 < MAX_TICK_MS) {
        engine.advance();
        n += 1;
      }
    } else {
      carry += dt * rate;
      const due = Math.floor(carry);
      carry -= due;
      for (; n < due && !engine.finished; n++) {
        engine.advance();
        if (performance.now() - t0 > MAX_TICK_MS) {
          carry = 0;
          break;
        }
      }
    }
    const spent = (performance.now() - t0) / 1000;
    if (n > 0 && spent > 0) throughput = throughput ? throughput * 0.8 + (n / spent) * 0.2 : n / spent;
    if (n > 0 || engine.finished) sendFrame();
  }
  if (!engine.finished) timer = scope.setTimeout(tick, FRAME_MS);
}

function inspect(): InspectFrame | null {
  if (!engine) return null;
  const lane = engine.lanes[inspectLane];
  if (!lane) return null;
  const refresh = !lastInspect || lastInspect.lane !== inspectLane || engine.step % INSPECT_EVERY === 0 || engine.finished || paused;
  if (!refresh) return lastInspect;
  let influence: Float32Array | null = null;
  let policyAction: Float32Array | null = null;
  if (net) {
    // Row 0 is the real observation: its action is what the policy commands here.
    occlusionBatch(lane.obs, batch);
    const out = net.policy.actWith(normed(batch, INPUT_BLOCKS.length + 1), INPUT_BLOCKS.length + 1, rng, true).action;
    if (lane.pilot === 'policy' || lane.pilot === 'guarded') influence = influenceFromActions(out, ACT_DIM);
    else policyAction = out.slice(0, ACT_DIM);
  }
  lastInspect = {
    lane: inspectLane,
    pilot: lane.pilot,
    obs: lane.obs.slice(),
    action: lane.action.slice(),
    influence,
    policyAction,
  };
  return lastInspect;
}

function sendFrame(): void {
  if (!engine) return;
  const lanes: LaneFrame[] = engine.lanes.map((l) => {
    const s = l.env.snapshot();
    return {
      pos: [s.pos[0], s.pos[1], s.pos[2]],
      yaw: s.yaw,
      coverage: l.coverage,
      collisions: l.collisions,
      steps: l.steps,
      floor: l.floor,
      reason: l.reason,
      action: [l.action[0]!, l.action[1]!, l.action[2]!, l.action[3]!],
    };
  });
  const visited = engine.lanes.map((l) => l.env.visitedGrid.slice());
  const seen = engine.lanes.map((l) => l.env.seenGrid.slice());
  const info = inspect();
  const frame: FromWorker = {
    type: 'frame',
    step: engine.step,
    lanes,
    visited,
    seen,
    inspect: info ? { ...info, obs: info.obs.slice() } : null,
    finished: engine.finished,
    throughput,
  };
  post(frame, [...visited.map((v) => v.buffer), ...seen.map((v) => v.buffer)]);
}

scope.onmessage = (event: MessageEvent<ToWorker>) => {
  const msg = event.data;
  if (msg.type === 'init') void init(msg.checkpointUrl);
  else if (msg.type === 'start') void start(msg);
  else if (msg.type === 'rate') {
    rate = msg.rate;
    carry = 0;
  } else if (msg.type === 'pause') {
    paused = msg.paused;
    lastTick = performance.now();
  } else if (msg.type === 'inspect') {
    inspectLane = msg.lane;
    lastInspect = null;
    if (engine) sendFrame();
  }
};

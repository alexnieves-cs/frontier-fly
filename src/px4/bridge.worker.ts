/// <reference lib="webworker" />
import type { GeoPoint } from '../mission/export/geo';
import { sampleHeight } from '../world/outdoor/terrain';
import type { Terrain } from '../world/outdoor/types';
import { eulerOf } from './quad';
import { Px4SimSession } from './session';

/**
 * Runs the PX4 lockstep loop off the main thread, so rendering never holds
 * PX4's clock back. Talks to PX4 through scripts/px4/bridge.mjs over a
 * WebSocket and posts the vehicle state to the page about 30 times a second.
 */

export interface StartMessage {
  type: 'start';
  url: string;
  home: GeoPoint;
  /** Sim-frame pad: the NED origin. */
  pad: { x: number; y: number; groundZ: number };
  terrain: Terrain | null;
  /** Wall-clock pacing: 1 is real time, 0 runs as fast as PX4 answers. */
  speed: number;
}

export type WorkerOut =
  | { type: 'status'; link: 'connecting' | 'bridge' | 'px4' | 'closed' | 'error'; text: string }
  | {
      type: 'state';
      /** Sim frame: x east, y north, z up, meters. */
      pos: [number, number, number];
      /** Sim-frame yaw (counter-clockwise from east), radians. */
      yaw: number;
      roll: number;
      pitch: number;
      vel: [number, number, number];
      motors: number[];
      armed: boolean;
      onGround: boolean;
      simTimeSec: number;
      /** Simulated seconds per wall second over the last report. */
      rtf: number;
      stepsPerSec: number;
      timeouts: number;
    };

const post = (m: WorkerOut): void => (self as DedicatedWorkerGlobalScope).postMessage(m);

let socket: WebSocket | null = null;
let running = false;

function start(msg: StartMessage): void {
  socket?.close();
  post({ type: 'status', link: 'connecting', text: `Connecting to ${msg.url}` });
  const ws = new WebSocket(msg.url);
  ws.binaryType = 'arraybuffer';
  socket = ws;
  const session = new Px4SimSession((bytes) => ws.readyState === WebSocket.OPEN && ws.send(bytes), {
    home: msg.home,
    lockstepTimeoutMs: 100,
  });
  const { terrain, pad } = msg;
  if (terrain) {
    session.quad.groundDown = (north, east) => -(sampleHeight(terrain, pad.x + east, pad.y + north) - pad.groundZ);
  }
  ws.onopen = () => post({ type: 'status', link: 'bridge', text: 'Bridge connected. Waiting for PX4.' });
  ws.onerror = () => post({ type: 'status', link: 'error', text: `No bridge at ${msg.url}` });
  ws.onclose = () => {
    running = false;
    session.abort();
    post({ type: 'status', link: 'closed', text: 'Bridge closed. Restart PX4 and the bridge, then reload.' });
  };
  ws.onmessage = (ev) => {
    if (typeof ev.data === 'string') {
      if (ev.data === 'px4-connected' && !running) {
        post({ type: 'status', link: 'px4', text: 'PX4 connected: flying in lockstep.' });
        void loop(session, msg);
      } else if (ev.data === 'px4-closed') {
        running = false;
        session.abort();
        post({ type: 'status', link: 'closed', text: 'PX4 disconnected.' });
      } else if (ev.data === 'px4-waiting') {
        post({ type: 'status', link: 'bridge', text: 'Bridge connected. Waiting for PX4.' });
      }
      return;
    }
    session.receive(new Uint8Array(ev.data as ArrayBuffer));
  };
}

async function loop(session: Px4SimSession, msg: StartMessage): Promise<void> {
  running = true;
  const wall0 = performance.now();
  const sim0 = session.simTimeSec;
  let lastPost = 0;
  let lastWall = wall0;
  let lastSim = sim0;
  let lastSteps = session.stats.steps;
  while (running) {
    await session.step();
    const now = performance.now();
    if (msg.speed > 0) {
      const ahead = (session.simTimeSec - sim0) * 1000 - (now - wall0) * msg.speed;
      if (ahead > 4) await new Promise((r) => setTimeout(r, ahead / msg.speed));
    }
    if (now - lastPost >= 33) {
      const s = session.quad.s;
      const e = eulerOf(s.q);
      const dt = Math.max(1e-3, (now - lastWall) / 1000);
      post({
        type: 'state',
        pos: [msg.pad.x + s.pos[1], msg.pad.y + s.pos[0], msg.pad.groundZ - s.pos[2]],
        yaw: Math.PI / 2 - e.yaw,
        roll: e.roll,
        pitch: e.pitch,
        vel: [s.vel[1], s.vel[0], -s.vel[2]],
        motors: Array.from(session.motors),
        armed: session.armed,
        onGround: s.onGround,
        simTimeSec: session.simTimeSec,
        rtf: (session.simTimeSec - lastSim) / dt,
        stepsPerSec: (session.stats.steps - lastSteps) / dt,
        timeouts: session.stats.timeouts,
      });
      lastPost = now;
      lastWall = now;
      lastSim = session.simTimeSec;
      lastSteps = session.stats.steps;
    }
  }
}

self.onmessage = (ev: MessageEvent<StartMessage>) => {
  if (ev.data.type === 'start') start(ev.data);
};

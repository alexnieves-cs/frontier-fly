/**
 * PX4 side of the mission link (missionLink.ts): a worker thread that is PX4 SITL's simulator over
 * TCP (lockstep HIL, as scripts/px4/sim-node.ts) and its offboard companion over UDP (setpoints,
 * parameters, mode and arming, external vision), driven one runner tick at a time through shared
 * memory.
 */
import { createSocket } from 'node:dgram';
import { createServer, type Socket } from 'node:net';
import { parentPort, workerData, type MessagePort } from 'node:worker_threads';
import type { DynParams } from '../shared/types';
import { FrontierPlant } from './frontierPlant';
import {
  decodeCommandAck,
  decodeExtendedSysState,
  decodeHeartbeat,
  decodeLocalPositionNed,
  decodeParamValue,
  decodeStatusText,
  MavlinkParser,
  MavlinkWriter,
  MSG,
} from './mavlink';
import { CMD, CTL, OP, PHASE, ST, px4Mode, type LinkEvent, type MissionLinkInit } from './missionLink';
import { Px4SimSession } from './session';

// workerData is { init, port }: events go out on `port` so the main thread, blocked in Atomics.wait
// between ticks, can drain them synchronously with receiveMessageOnPort.
const wd = workerData as { init: MissionLinkInit; port?: MessagePort };
const init = wd.init;
const ctl = new Int32Array(init.ctl);
const data = new Float64Array(init.data);
const port = wd.port ?? parentPort!;
// The main thread sits in Atomics.wait between ticks and never sees a worker 'error' event, so a
// crash here is reported through the shared phase and the event port.
for (const ev of ['uncaughtException', 'unhandledRejection'] as const) {
  process.on(ev, (err: unknown) => {
    port.postMessage({ type: 'event', event: { simSec: 0, kind: 'link', text: `worker crashed: ${err instanceof Error ? err.stack : String(err)}`, severity: 2 } });
    Atomics.store(ctl, CTL.PHASE, PHASE.FAILED);
    Atomics.notify(ctl, CTL.PHASE);
  });
}
/** PX4's MAVLink system id: instance N boots as N+1. */
const SYS = init.sysId ?? 1;

const events: LinkEvent[] = [];
function event(kind: LinkEvent['kind'], text: string, severity?: number): void {
  const e: LinkEvent = { simSec: session ? +session.simTimeSec.toFixed(2) : 0, kind, text, ...(severity === undefined ? {} : { severity }) };
  events.push(e);
  port.postMessage({ type: 'event', event: e });
}

const plant = new FrontierPlant(init.dyn as DynParams);
const g = init.ground;
if (g) {
  plant.groundDown = (north, east) => {
    const fx = (east - g.originEast) / g.step;
    const fy = (north - g.originNorth) / g.step;
    const ix = Math.max(0, Math.min(g.nx - 2, Math.floor(fx)));
    const iy = Math.max(0, Math.min(g.ny - 2, Math.floor(fy)));
    const tx = Math.max(0, Math.min(1, fx - ix));
    const ty = Math.max(0, Math.min(1, fy - iy));
    const at = (x: number, y: number) => g.down[y * g.nx + x]!;
    return (at(ix, iy) * (1 - tx) + at(ix + 1, iy) * tx) * (1 - ty) + (at(ix, iy + 1) * (1 - tx) + at(ix + 1, iy + 1) * tx) * ty;
  };
}

let session: Px4SimSession | null = null;
let hil: Socket | null = null;

// ------------------------------------------------------------------ offboard link (UDP)
const udp = createSocket('udp4');
const gcs = new MavlinkWriter(255, 190);
const offParser = new MavlinkParser();
let px4Seen = false;
let customMode = 0;
let baseMode = 0;
let landedState = 0;
const ekfPos = [0, 0, 0];
const ekfVel = [0, 0, 0];
const paramOk = new Set<string>();
let setpoints = 0;
let evSent = 0;

function sendOff(bytes: Uint8Array): void {
  udp.send(bytes, init.offboardRemotePort, '127.0.0.1');
}

udp.on('message', (msg) => {
  for (const f of offParser.push(new Uint8Array(msg.buffer, msg.byteOffset, msg.byteLength))) {
    if (f.sysid !== SYS) continue;
    switch (f.msgid) {
      case MSG.HEARTBEAT: {
        if (f.compid !== 1) break;
        const h = decodeHeartbeat(f);
        if (!px4Seen) event('link', 'PX4 heartbeat on the offboard link');
        px4Seen = true;
        const armed = (h.baseMode & 128) !== 0;
        if (armed !== ((baseMode & 128) !== 0)) event('arm', armed ? 'armed' : 'disarmed');
        if (h.customMode !== customMode) event('mode', px4Mode(h.customMode));
        customMode = h.customMode;
        baseMode = h.baseMode;
        break;
      }
      case MSG.LOCAL_POSITION_NED: {
        const p = decodeLocalPositionNed(f);
        for (let i = 0; i < 3; i++) {
          ekfPos[i] = p.pos[i]!;
          ekfVel[i] = p.vel[i]!;
        }
        break;
      }
      case MSG.STATUSTEXT: {
        const s = decodeStatusText(f);
        event('statustext', s.text, s.severity);
        break;
      }
      case MSG.EXTENDED_SYS_STATE:
        landedState = decodeExtendedSysState(f).landedState;
        break;
      case MSG.PARAM_VALUE: {
        const p = decodeParamValue(f);
        const want = init.params.find((q) => q.id === p.id);
        if (want) {
          const got = want.type === 'int32' ? p.raw.getInt32(0, true) : p.raw.getFloat32(0, true);
          if (Math.abs(got - want.value) <= 1e-4 * Math.max(1, Math.abs(want.value))) paramOk.add(p.id);
        }
        break;
      }
      case MSG.COMMAND_ACK: {
        const a = decodeCommandAck(f);
        if (a.result !== 0 && a.result !== 5) event('link', `command ${a.command} answered ${a.result}`);
        break;
      }
      default:
        break;
    }
  }
});

// ------------------------------------------------------------------ the sim loop
const STEP_US = 4000;
const SETPOINT_EVERY = 5; // 50 Hz at 250 Hz physics
const EV_EVERY = 8; // 31 Hz

/** One lockstep step with the current command. */
async function stepOnce(): Promise<void> {
  const s = session!;
  const k = s.stats.steps;
  if (k % SETPOINT_EVERY === 0) {
    sendOff(
      gcs.velocityTarget({ targetSystem: SYS,
        timeBootMs: s.simTimeSec * 1000,
        vn: data[CMD.VEL]!,
        ve: data[CMD.VEL + 1]!,
        vd: data[CMD.VEL + 2]!,
        yawRate: data[CMD.YAW_RATE]!,
      }),
    );
    setpoints++;
  }
  if (data[CMD.EV_ON] === 1 && k % EV_EVERY === 0) {
    sendOff(
      gcs.odometry({
        timeUsec: 1_000_000 + s.simTimeSec * 1e6,
        pos: [data[CMD.EV_POS]!, data[CMD.EV_POS + 1]!, data[CMD.EV_POS + 2]!],
        vel: [data[CMD.EV_VEL]!, data[CMD.EV_VEL + 1]!, data[CMD.EV_VEL + 2]!],
        posVar: data[CMD.EV_POS_VAR]!,
        velVar: data[CMD.EV_VEL_VAR]!,
        resetCounter: data[CMD.EV_RESET]! & 0xff,
      }),
    );
    evSent++;
  }
  if (k % 250 === 0) sendOff(gcs.heartbeat({ type: 18, autopilot: 8, baseMode: 0, customMode: 0, systemStatus: 4 }));
  await s.step();
}

function writeState(): void {
  const s = session!;
  const q = plant.s;
  data[ST.SIM_TIME] = s.simTimeSec;
  for (let i = 0; i < 3; i++) {
    data[ST.POS + i] = q.pos[i]!;
    data[ST.VEL + i] = q.vel[i]!;
    data[ST.OMEGA + i] = q.omega[i]!;
    data[ST.SPECIFIC_FORCE + i] = plant.specificForce[i]!;
    data[ST.EKF_POS + i] = ekfPos[i]!;
    data[ST.EKF_VEL + i] = ekfVel[i]!;
  }
  for (let i = 0; i < 4; i++) {
    data[ST.Q + i] = q.q[i]!;
    data[ST.MOTORS + i] = s.motors[i]!;
  }
  data[ST.ARMED] = (baseMode & 128) !== 0 ? 1 : 0;
  data[ST.CUSTOM_MODE] = customMode;
  data[ST.ON_GROUND] = q.onGround ? 1 : 0;
  data[ST.LANDED_STATE] = landedState;
  data[ST.TIMEOUTS] = s.stats.timeouts;
  data[ST.SETPOINTS] = setpoints;
  data[ST.EV_SENT] = evSent;
}

function setPhase(p: number): void {
  Atomics.store(ctl, CTL.PHASE, p);
  Atomics.notify(ctl, CTL.PHASE);
}

/** Boot: free-run the lockstep until PX4 takes the parameters, then hold offboard and arm. */
async function boot(): Promise<void> {
  const s = session!;
  const deadline = 240; // sim seconds
  let nextParams = 0;
  let nextArm = 0;
  let paramsDone = false;
  let streamSince = -1;
  data[CMD.VEL] = 0;
  data[CMD.VEL + 1] = 0;
  data[CMD.VEL + 2] = 0;
  data[CMD.YAW_RATE] = 0;
  data[CMD.GPS_FIX] = 1;
  for (let i = 0; i < 4; i++) data[CMD.THRUST + i] = 1;
  while (s.simTimeSec < deadline) {
    await stepOnce();
    const t = s.simTimeSec;
    if (!px4Seen) continue;
    if (!paramsDone && t >= nextParams) {
      nextParams = t + 1;
      const missing = init.params.filter((p) => !paramOk.has(p.id));
      if (missing.length === 0) {
        paramsDone = true;
        event('link', `${init.params.length} parameters set`);
        streamSince = t;
      } else for (const p of missing) sendOff(gcs.paramSet({ ...p, targetSystem: SYS }));
      continue;
    }
    if (!paramsDone) continue;
    // Setpoints stream before the switch: PX4 refuses offboard without them.
    if (t - streamSince < 2 || t < nextArm) continue;
    nextArm = t + 1;
    const offboard = ((customMode >>> 16) & 0xff) === 6;
    const armed = (baseMode & 128) !== 0;
    if (offboard && armed) {
      writeState();
      event('link', `offboard and armed at ${t.toFixed(1)} s`);
      return;
    }
    if (!offboard) sendOff(gcs.commandLong({ targetSystem: SYS, command: 176, params: [1, 6, 0, 0, 0, 0, 0] }));
    else sendOff(gcs.commandLong({ targetSystem: SYS, command: 400, params: [1, 0, 0, 0, 0, 0, 0] }));
  }
  throw new Error(`PX4 did not reach offboard and armed within ${deadline} s (params ${paramOk.size}/${init.params.length}, mode ${px4Mode(customMode)})`);
}

/**
 * The runner owns the flight from takeoff to touchdown, so PX4 must stay in offboard and armed. If it
 * drops out (a failsafe, a mode change), ask again once a second and log each drop as an event, so a
 * run that needed it says so.
 */
let dropSince: number | null = null;
let nextRetry = 0;
function keepOffboard(): void {
  const t = session!.simTimeSec;
  const offboard = ((customMode >>> 16) & 0xff) === 6;
  const armed = (baseMode & 128) !== 0;
  if (offboard && armed) {
    if (dropSince !== null) event('link', `back in offboard and armed after ${(t - dropSince).toFixed(1)} s`);
    dropSince = null;
    return;
  }
  if (dropSince === null) {
    dropSince = t;
    event('link', `PX4 left offboard or disarmed (mode ${px4Mode(customMode)}, ${armed ? 'armed' : 'disarmed'}); asking again`, 4);
  }
  if (t < nextRetry) return;
  nextRetry = t + 1;
  if (!offboard) sendOff(gcs.commandLong({ targetSystem: SYS, command: 176, params: [1, 6, 0, 0, 0, 0, 0] }));
  else sendOff(gcs.commandLong({ targetSystem: SYS, command: 400, params: [1, 0, 0, 0, 0, 0, 0] }));
}

/** Atomics.waitAsync (Node 16+; this TypeScript lib predates it). */
const waitAsync = (Atomics as unknown as { waitAsync: (a: Int32Array, i: number, v: number) => { async: boolean; value: Promise<string> | string } }).waitAsync;

/** Serve runner requests until told to stop. */
async function serve(): Promise<void> {
  let seen = Atomics.load(ctl, CTL.REQ);
  for (;;) {
    const w = waitAsync(ctl, CTL.REQ, seen);
    if (w.async) await w.value;
    const req = Atomics.load(ctl, CTL.REQ);
    if (req === seen) continue;
    seen = req;
    if (Atomics.load(ctl, CTL.OP) === OP.STOP) break;
    const s = session!;
    if (data[CMD.TELEPORT] === 1) {
      plant.teleport([data[CMD.TP_POS]!, data[CMD.TP_POS + 1]!, data[CMD.TP_POS + 2]!], [data[CMD.TP_VEL]!, data[CMD.TP_VEL + 1]!, data[CMD.TP_VEL + 2]!]);
      data[CMD.TELEPORT] = 0;
    }
    plant.wind = [data[CMD.WIND]!, data[CMD.WIND + 1]!, data[CMD.WIND + 2]!];
    for (let i = 0; i < 4; i++) plant.dyn.thrustScale[i] = data[CMD.THRUST + i]!;
    s.gpsFix = data[CMD.GPS_FIX] === 1;
    const n = Math.max(1, Math.round((data[CMD.DT]! * 1e6) / STEP_US));
    for (let i = 0; i < n; i++) await stepOnce();
    keepOffboard();
    writeState();
    Atomics.store(ctl, CTL.ACK, seen);
    Atomics.notify(ctl, CTL.ACK);
  }
}

const server = createServer((socket) => {
  if (hil) {
    socket.destroy();
    return;
  }
  hil = socket;
  socket.setNoDelay(true);
  session = new Px4SimSession((bytes) => socket.write(bytes), { home: init.home, quad: plant, lockstepTimeoutMs: 200, seed: init.seed, stepUs: STEP_US });
  socket.on('data', (chunk: Buffer) => session!.receive(chunk));
  socket.on('close', () => {
    session?.abort();
    setPhase(PHASE.CLOSED);
  });
  event('link', 'PX4 connected to the simulator port');
  void (async () => {
    try {
      await boot();
      setPhase(PHASE.READY);
      await serve();
      // Disarm on the pad (forced: the land detector may not have caught up) and step a little so PX4 logs it.
      sendOff(gcs.commandLong({ targetSystem: SYS, command: 400, params: [0, 21196, 0, 0, 0, 0, 0] }));
      for (let i = 0; i < 250; i++) await stepOnce();
      writeState();
      port.postMessage({ type: 'done', stats: session!.stats, events });
      Atomics.store(ctl, CTL.ACK, Atomics.load(ctl, CTL.REQ));
      Atomics.notify(ctl, CTL.ACK);
    } catch (err) {
      event('link', String(err instanceof Error ? err.message : err));
      setPhase(PHASE.FAILED);
    }
  })();
});

udp.bind(init.offboardLocalPort, '127.0.0.1', () => {
  server.listen(init.hilPort, '127.0.0.1', () => setPhase(PHASE.LISTENING));
});

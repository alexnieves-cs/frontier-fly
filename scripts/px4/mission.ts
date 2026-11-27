/**
 * Flies a whole autonomous mission (preflight, search, building entry, indoor search, confirm, exit,
 * return, landing) with PX4 SITL as the flight controller.
 *
 *   PX4_BUILD=/path/to/PX4-Autopilot/build/px4_sitl_default \
 *     npx vite-node scripts/px4/mission.ts -- --dir DIR [--case atlas.json --index N | --preset NAME]
 *       [--mode policyQuad] [--indoor-nav vision|gps] [--frames] [--film] [--instance N] [--param ID[:i]=VALUE ...]
 *
 * --film also writes DIR/<id>.film.json, every tick with PX4's state, for film.html?px4=<url>.
 *
 * The runner, planners, perception and safety ladder are the ones every other study flies; only the
 * airframe changes: Px4Vehicle (src/mission/autonomy/px4Vehicle.ts) hands each tick's velocity command
 * to PX4 as an offboard setpoint, and PX4's EKF2 and multicopter controllers fly frontier-fly's own
 * rigid body (FrontierPlant) through the lockstep HIL link in a worker thread (src/px4/missionLink*.ts).
 * Indoors, with --indoor-nav vision (the default), PX4 gets no GNSS fix and fuses external vision
 * instead (VIO-grade noise and drift on the true pose, see visionModel below).
 *
 * (--dir, not --out: scripts/physics/maneuvers.ts writes its own file whenever the args hold --out.)
 *
 * Writes DIR/<id>.json: the run summary, PX4's events and, with --frames, a 10 Hz state log
 * (true pose, PX4's estimate, attitude, motors, flight mode) for the replay page.
 */
import { spawn, type ChildProcess } from 'node:child_process';
import { mkdirSync, mkdtempSync, readFileSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { MessageChannel, receiveMessageOnPort, Worker } from 'node:worker_threads';
import { buildSync } from 'esbuild';
import { BINS, caseSetup, DIMS, type AtlasCase } from '../../src/eval/atlas/space';
import { missionModeOptions, isMissionMode, type MissionMode } from '../../src/eval/missions';
import type { Phase, V3 } from '../../src/mission/autonomy/contracts';
import { buildingAt, createPerceptionSource } from '../../src/mission/autonomy/perception';
import { policyFromFile } from '../../src/mission/autonomy/policy';
import { Px4Vehicle, type Px4Link, type Px4LinkCommand, type Px4LinkState } from '../../src/mission/autonomy/px4Vehicle';
import { missionQuadParams } from '../../src/mission/autonomy/quadVehicle';
import { buildReport } from '../../src/mission/autonomy/report';
import { FlightRecorder } from '../../src/film/flight';
import { PX4_FILM_SCHEMA, type Px4FilmFile } from '../../src/film/px4Film';
import { createMissionRunner } from '../../src/mission/autonomy/runner';
import { buildMissionWorld, missionForWorld } from '../../src/mission/autonomy/world';
import { GEO_ORIGINS } from '../../src/mission/export/geo';
import { CMD, CTL, DATA_LENGTH, OP, PHASE, px4Mode, ST, type LinkEvent, type MissionLinkInit, type Px4Param } from '../../src/px4/missionLink';
import { SHIPPED_CHECKPOINT_PATH } from '../../src/rl/checkpoint';
import { sampleHeight } from '../../src/world/outdoor/terrain';
import { px4Params } from '../physics/maneuvers';

const HERE = dirname(fileURLToPath(import.meta.url));
const ROOT = resolve(HERE, '../..');
const args = process.argv.slice(2);
const opt = (name: string): string | undefined => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const has = (name: string) => args.includes(`--${name}`);
const outDir = resolve(opt('dir') ?? 'test-results/px4-mission');
const mode = (opt('mode') ?? 'policyQuad') as MissionMode;
if (!isMissionMode(mode)) throw new Error(`unknown mode ${mode}`);
const indoorNav = (opt('indoor-nav') ?? 'vision') as 'vision' | 'gps';
const px4Build = process.env.PX4_BUILD;
if (!px4Build) throw new Error('set PX4_BUILD to the px4_sitl_default build directory');
const speed = Number(process.env.PX4_SIM_SPEED_FACTOR ?? 1);
/** PX4 instance: its simulator port is 4560+N and its offboard link 14580+N (PX4 side) to 14540+N, so runs can go side by side. */
const instance = Number(opt('instance') ?? 0);

/** The case to fly: an atlas case (file + index) or a named preset. */
function pickCase(): { id: string; c: AtlasCase } {
  const file = opt('case');
  if (file) {
    const all = JSON.parse(readFileSync(file, 'utf8')) as AtlasCase[] | { results: { case: AtlasCase }[] };
    const list = Array.isArray(all) ? all : all.results.map((r) => r.case);
    const idx = Number(opt('index') ?? 0);
    const c = list[idx]!;
    return { id: opt('id') ?? `case-${c.i}-${c.layoutSeed}`, c };
  }
  // Presets, built from named values (dims in src/eval/atlas/space.ts): midday, light air, no fault.
  const preset = opt('preset') ?? 'find-person-2';
  const make = (i: number, cat: Record<string, string>, unitVals: Record<string, number>): AtlasCase => {
    const k = DIMS.map((d) => (d.continuous ? Math.min(BINS - 1, Math.floor((unitVals[d.id] ?? 0.5) * BINS)) : Math.max(0, d.values.indexOf(cat[d.id] ?? d.values[0]!))));
    const u = DIMS.map((d) => (d.continuous ? (unitVals[d.id] ?? 0.5) : 0));
    return { i, layoutSeed: 270_000 + i, placementSeed: 9_100_000 + i, k, u };
  };
  const calm = { timeOfDay: 0.5, windMean: 0.2, windDir: 0.3, windGust: 0.3, windOnset: 0.1, targetBuilding: 0.75 };
  const presets: Record<string, AtlasCase> = {
    'find-person-2': make(11, { mission: 'find-person', buildings: '2', personState: 'standing', decoys: '1', lights: 'on', fault: 'none' }, calm),
    'sweep-occupied-hiding': make(21, { mission: 'sweep-occupied', buildings: '2', personState: 'hiding', decoys: '2', lights: 'off', fault: 'none' }, { ...calm, timeOfDay: 0.9 }),
    'sweep-empty-1': make(31, { mission: 'sweep-empty', buildings: '1', personState: 'standing', decoys: '1', lights: 'on', fault: 'none' }, calm),
  };
  const c = presets[preset];
  if (!c) throw new Error(`unknown preset ${preset}`);
  return { id: preset, c };
}

/** VIO-grade external vision: the true pose plus a slow random-walk drift and white noise. */
function visionModel(seed: number) {
  let s = seed >>> 0 || 1;
  const uni = () => {
    s = (s + 0x6d2b79f5) >>> 0;
    let t = s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
  const gauss = () => Math.sqrt(-2 * Math.log(Math.max(1e-12, uni()))) * Math.cos(2 * Math.PI * uni());
  const drift: V3 = [0, 0, 0];
  let lastT = 0;
  const DRIFT = 0.02; // m per sqrt(s)
  const POS_SIGMA = 0.03;
  const VEL_SIGMA = 0.05;
  return (truth: { pos: V3; vel: V3; tSec: number }) => {
    const dt = Math.max(0, truth.tSec - lastT);
    lastT = truth.tSec;
    for (let i = 0; i < 3; i++) drift[i] = drift[i]! + gauss() * DRIFT * Math.sqrt(dt);
    return {
      pos: [0, 1, 2].map((i) => truth.pos[i]! + drift[i]! + gauss() * POS_SIGMA) as V3,
      vel: [0, 1, 2].map((i) => truth.vel[i]! + gauss() * VEL_SIGMA) as V3,
      posVar: POS_SIGMA ** 2 + 0.05 ** 2,
      velVar: VEL_SIGMA ** 2,
      reset: 0,
    };
  };
}

class LinkClient implements Px4Link {
  readonly ctl = new Int32Array(new SharedArrayBuffer(CTL.LENGTH * 4));
  readonly data = new Float64Array(new SharedArrayBuffer(DATA_LENGTH * 8));
  advances = 0;

  state(): Px4LinkState {
    const d = this.data;
    const v3 = (i: number): V3 => [d[i]!, d[i + 1]!, d[i + 2]!];
    return {
      simSec: d[ST.SIM_TIME]!,
      posNed: v3(ST.POS),
      velNed: v3(ST.VEL),
      q: [d[ST.Q]!, d[ST.Q + 1]!, d[ST.Q + 2]!, d[ST.Q + 3]!],
      omegaFrd: v3(ST.OMEGA),
      specificForceFrd: v3(ST.SPECIFIC_FORCE),
      armed: d[ST.ARMED] === 1,
      customMode: d[ST.CUSTOM_MODE]!,
      onGround: d[ST.ON_GROUND] === 1,
      motors: [d[ST.MOTORS]!, d[ST.MOTORS + 1]!, d[ST.MOTORS + 2]!, d[ST.MOTORS + 3]!],
      ekfPosNed: v3(ST.EKF_POS),
      ekfVelNed: v3(ST.EKF_VEL),
      landedState: d[ST.LANDED_STATE]!,
      timeouts: d[ST.TIMEOUTS]!,
    };
  }

  private request(op: number): void {
    const seq = Atomics.load(this.ctl, CTL.REQ) + 1;
    Atomics.store(this.ctl, CTL.OP, op);
    Atomics.store(this.ctl, CTL.REQ, seq);
    Atomics.notify(this.ctl, CTL.REQ);
    const t0 = Date.now();
    while (Atomics.load(this.ctl, CTL.ACK) !== seq) {
      const phase = Atomics.load(this.ctl, CTL.PHASE);
      if (phase === PHASE.FAILED || phase === PHASE.CLOSED) throw new Error(`PX4 link ${phase === PHASE.FAILED ? 'failed' : 'closed'}`);
      Atomics.wait(this.ctl, CTL.ACK, Atomics.load(this.ctl, CTL.ACK), 1000);
      if (Date.now() - t0 > 60_000) throw new Error('PX4 link: no answer in 60 s');
    }
  }

  advance(cmd: Px4LinkCommand): Px4LinkState {
    const d = this.data;
    d[CMD.DT] = cmd.dt;
    for (let i = 0; i < 3; i++) {
      d[CMD.VEL + i] = cmd.velNed[i]!;
      d[CMD.WIND + i] = cmd.windNed[i]!;
    }
    d[CMD.YAW_RATE] = cmd.yawRateDown;
    if (cmd.teleport) {
      d[CMD.TELEPORT] = 1;
      for (let i = 0; i < 3; i++) {
        d[CMD.TP_POS + i] = cmd.teleport.posNed[i]!;
        d[CMD.TP_VEL + i] = cmd.teleport.velNed[i]!;
      }
    }
    for (let i = 0; i < 4; i++) d[CMD.THRUST + i] = cmd.thrust[i] ?? 1;
    d[CMD.GPS_FIX] = cmd.gpsFix ? 1 : 0;
    d[CMD.EV_ON] = cmd.ev ? 1 : 0;
    if (cmd.ev) {
      for (let i = 0; i < 3; i++) {
        d[CMD.EV_POS + i] = cmd.ev.posNed[i]!;
        d[CMD.EV_VEL + i] = cmd.ev.velNed[i]!;
      }
      d[CMD.EV_POS_VAR] = cmd.ev.posVar;
      d[CMD.EV_VEL_VAR] = cmd.ev.velVar;
      d[CMD.EV_RESET] = cmd.ev.reset;
    }
    this.request(OP.ADVANCE);
    this.advances++;
    return this.state();
  }

  stop(): void {
    try {
      this.request(OP.STOP);
    } catch {
      // The link may already be down; the report says so.
    }
  }

  /**
   * Waits for the worker to reach a phase without blocking this thread: the worker loads its code
   * and boots PX4 while the main event loop stays free (a loader hook on this thread can deadlock a
   * worker that starts while the main thread sits in Atomics.wait). Only the per-tick advance blocks.
   */
  async waitPhase(want: number, timeoutSec: number, drain: () => void): Promise<void> {
    const t0 = Date.now();
    for (;;) {
      drain();
      const p = Atomics.load(this.ctl, CTL.PHASE);
      if (p === want) return;
      if (p === PHASE.FAILED || p === PHASE.CLOSED) throw new Error(`PX4 link ${p === PHASE.FAILED ? 'failed' : 'closed'} during boot`);
      if (Date.now() - t0 > timeoutSec * 1000) throw new Error(`PX4 link: phase ${want} not reached in ${timeoutSec} s`);
      await new Promise((r) => setTimeout(r, 50));
    }
  }
}

async function main(): Promise<void> {
  const { id, c } = pickCase();
  mkdirSync(outDir, { recursive: true });
  const setup = caseSetup(c);
  const world = buildMissionWorld(setup.world);
  const mission = missionForWorld(world, setup.mission);
  const raw = createPerceptionSource(world);
  const shipped = mode.startsWith('policy') ? policyFromFile(readFileSync(SHIPPED_CHECKPOINT_PATH)) : undefined;

  // Ground under the plant: the terrain on a 1 m grid over the geofence, down from the pad.
  const home = world.home;
  const terrain = world.outdoor.terrain;
  const fence = world.outdoor.geofence;
  const xs = fence.polygon.map((p) => p[0]);
  const ys = fence.polygon.map((p) => p[1]);
  const step = 1;
  const x0 = Math.floor(Math.min(...xs)) - 10;
  const y0 = Math.floor(Math.min(...ys)) - 10;
  const nx = Math.ceil(Math.max(...xs) - x0) + 20;
  const ny = Math.ceil(Math.max(...ys) - y0) + 20;
  const homeGround = sampleHeight(terrain, home[0], home[1]);
  const down = new Float32Array(nx * ny);
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) down[j * nx + i] = -(sampleHeight(terrain, x0 + i * step, y0 + j * step) - homeGround);

  const params: Px4Param[] = Object.entries(px4Params('bridge')).map(([k, v]) => ({
    id: k,
    value: v,
    type: /^(COM_RCL_EXCEPT|NAV_RCL_ACT)$/u.test(k) ? 'int32' : 'real32',
  }));
  params.push(
    { id: 'NAV_DLL_ACT', value: 0, type: 'int32' },
    { id: 'COM_RC_IN_MODE', value: 4, type: 'int32' },
    // The runner decides when the drone is done (it lands to swap packs and takes off again), so PX4
    // keeps the motors armed on the ground; the link disarms at the end.
    { id: 'COM_DISARM_PRFLT', value: -1, type: 'real32' },
    { id: 'COM_DISARM_LAND', value: -1, type: 'real32' },
    // External vision: horizontal position and velocity; height stays on the barometer.
    ...(indoorNav === 'vision' ? [{ id: 'EKF2_EV_CTRL', value: 5, type: 'int32' as const }, { id: 'EKF2_EV_DELAY', value: 0, type: 'real32' as const }] : []),
    // A stiffer horizontal velocity loop. On the stock gains (P 1.8, I 0.4, D 0.2) this 0.45 kg,
    // high-drag airframe lags its setpoint by about a quarter at entry speed, drifts wide on the
    // roof-hatch descent and clips the eave (find-person-2: 3 contacts). These gains cut the
    // fly-through velocity error from 0.224 to 0.123 m/s RMS and flew find-person-2,
    // sweep-occupied-hiding and sweep-empty-1 with no contact (PX4 v1.15.4 SITL).
    { id: 'MPC_XY_VEL_P_ACC', value: 4, type: 'real32' },
    { id: 'MPC_XY_VEL_I_ACC', value: 3, type: 'real32' },
    { id: 'MPC_XY_VEL_D_ACC', value: 0.4, type: 'real32' },
  );

  // --param ID=VALUE (repeatable) overrides or adds a PX4 parameter; integer-looking ids need ':i'.
  for (let i = 0; i < args.length; i++) {
    if (args[i] !== '--param') continue;
    const [lhs, val] = args[i + 1]!.split('=') as [string, string];
    const [pid, kind] = lhs.split(':');
    const at = params.findIndex((p) => p.id === pid);
    const entry: Px4Param = { id: pid!, value: Number(val), type: kind === 'i' ? 'int32' : 'real32' };
    if (at >= 0) params[at] = entry;
    else params.push(entry);
  }

  const link = new LinkClient();
  const channel = new MessageChannel();
  const events: LinkEvent[] = [];
  let workerDone: { stats: unknown } | null = null;
  const drain = () => {
    for (;;) {
      const m = receiveMessageOnPort(channel.port1);
      if (!m) break;
      const msg = m.message as { type: 'event'; event: LinkEvent } | { type: 'done'; stats: unknown };
      if (msg.type === 'event') {
        events.push(msg.event);
        if (msg.event.kind !== 'statustext' || (msg.event.severity ?? 6) <= 4) console.log(`  px4 ${msg.event.simSec.toFixed(1)}s ${msg.event.kind}: ${msg.event.text}`);
      } else workerDone = msg;
    }
  };

  const bundle = join(mkdtempSync(join(tmpdir(), 'ff-px4-')), 'missionLink.worker.mjs');
  buildSync({ entryPoints: [join(ROOT, 'src/px4/missionLink.worker.ts')], bundle: true, platform: 'node', format: 'esm', outfile: bundle, logLevel: 'error' });
  const init: MissionLinkInit = {
    ctl: link.ctl.buffer as SharedArrayBuffer,
    data: link.data.buffer as SharedArrayBuffer,
    hilPort: 4560 + instance,
    offboardLocalPort: 14540 + instance,
    offboardRemotePort: 14580 + instance,
    home: GEO_ORIGINS['px4-sitl'],
    params,
    dyn: missionQuadParams(),
    ground: { originEast: x0 - home[0], originNorth: y0 - home[1], step, nx, ny, down },
    seed: world.seed,
    sysId: instance + 1,
  };
  // The worker posts through `port`; it replaces parentPort's messages so they can be read synchronously.
  // execArgv []: the bundle is plain JS and needs none of the parent's loader flags.
  const worker = new Worker(bundle, { workerData: { init, port: channel.port2 }, transferList: [channel.port2], execArgv: [] });
  worker.on('error', (e) => console.error('PX4 link worker error:', e));
  console.log(`${id}: ${JSON.stringify(setup.decoded)}`);
  await link.waitPhase(PHASE.LISTENING, 60, drain);

  const work = mkdtempSync(join(tmpdir(), 'px4-'));
  const px4: ChildProcess = spawn(join(px4Build!, 'bin/px4'), ['-i', String(instance), '-d', join(px4Build!, 'etc')], {
    cwd: work,
    env: { ...process.env, PX4_SYS_AUTOSTART: '10016', PX4_SIM_SPEED_FACTOR: String(speed), PX4_HOME_LAT: '473977420', PX4_HOME_LON: '85455940' },
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  // PX4 must not outlive this run (a stale instance holds its ports and the next run hangs).
  const killPx4 = () => {
    if (px4.exitCode === null) px4.kill('SIGKILL');
  };
  process.on('exit', killPx4);
  for (const sig of ['SIGINT', 'SIGTERM'] as const) process.on(sig, () => process.exit(130));
  process.on('uncaughtException', (e) => {
    console.error(e);
    process.exit(1);
  });
  const px4Log: string[] = [];
  px4.stdout!.on('data', (b: Buffer) => px4Log.push(b.toString()));
  px4.stderr!.on('data', (b: Buffer) => px4Log.push(b.toString()));
  const wall0 = Date.now();
  try {
    await link.waitPhase(PHASE.READY, 300, drain);
  } catch (e) {
    writeFileSync(join(outDir, `${id}.px4.log`), px4Log.join(''));
    throw e;
  }
  drain();
  const bootSec = link.state().simSec;
  console.log(`  PX4 ready (offboard, armed) after ${bootSec.toFixed(1)} s sim, ${((Date.now() - wall0) / 1000).toFixed(1)} s wall`);

  const vision = visionModel(world.seed ^ 0x5eed);
  let px4Vehicle: Px4Vehicle | null = null;
  const runner = await createMissionRunner(mission, world, raw, {
    dt: 0.05,
    operator: () => 'approve',
    ...missionModeOptions(mode, shipped),
    ...(setup.faults.length ? { faults: { plan: setup.faults, seed: world.seed } } : {}),
    measureClearance: true,
    vehicleFactory: ({ physics, battery, start, yaw, outdoor }) => {
      px4Vehicle = new Px4Vehicle(physics, battery, start, yaw, outdoor, link, link.state(), {
        gnssDenied: indoorNav === 'vision' ? (p) => buildingAt(world, p) !== null : undefined,
        vision: indoorNav === 'vision' ? vision : undefined,
      });
      return px4Vehicle;
    },
  });
  const v = px4Vehicle as unknown as Px4Vehicle;
  const frames: number[][] = [];
  // --film: every tick as the film page replays it (src/film/px4Film.ts).
  const recorder = has('film') ? new FlightRecorder(0.05) : null;
  recorder?.capture(runner.snapshot());
  const modes: { t: number; mode: string }[] = [];
  let lastMode = '';
  let nextFrame = 0;
  let ekfErrMax = 0;
  let ekfErrSum = 0;
  let ekfErrN = 0;
  const limit = Math.ceil((mission.budgets.timeSec + 5) / 0.05);
  const t0 = Date.now();
  let failed: string | null = null;
  for (let i = 0; i < limit && !runner.done; i++) {
    try {
      runner.step();
    } catch (e) {
      failed = e instanceof Error ? e.message : String(e);
      break;
    }
    drain();
    const s = v.last;
    const m = px4Mode(s.customMode);
    const t = i * 0.05;
    if (m !== lastMode) {
      modes.push({ t: +t.toFixed(2), mode: m });
      lastMode = m;
    }
    // PX4's estimate against the truth, horizontal.
    const err = Math.hypot(s.ekfPosNed[0] - s.posNed[0], s.ekfPosNed[1] - s.posNed[1]);
    if (s.armed && !s.onGround) {
      ekfErrMax = Math.max(ekfErrMax, err);
      ekfErrSum += err * err;
      ekfErrN++;
    }
    if (recorder) {
      const frame = recorder.capture(runner.snapshot());
      const ekfNow: V3 = [v.origin[0] + s.ekfPosNed[1], v.origin[1] + s.ekfPosNed[0], v.origin[2] - s.ekfPosNed[2]];
      const r3 = (x: number) => Math.round(x * 1000) / 1000;
      frame.px4 = {
        ekf: [r3(ekfNow[0]), r3(ekfNow[1]), r3(ekfNow[2])],
        roll: r3(v.attitude.roll),
        pitch: r3(v.attitude.pitch),
        motors: [r3(s.motors[0] ?? 0), r3(s.motors[1] ?? 0), r3(s.motors[2] ?? 0), r3(s.motors[3] ?? 0)],
        mode: m,
        armed: s.armed,
        denied: v.lastDenied,
        vision: v.lastVision,
        sp: [r3(v.lastSetpoint[0]), r3(v.lastSetpoint[1]), r3(v.lastSetpoint[2])],
      };
      frame.pos = [r3(frame.pos[0]), r3(frame.pos[1]), r3(frame.pos[2])];
      frame.vel = [r3(frame.vel[0]), r3(frame.vel[1]), r3(frame.vel[2])];
      frame.yaw = r3(frame.yaw);
      frame.battery = Math.round(frame.battery * 10) / 10;
    }
    if (has('frames') && t >= nextFrame) {
      nextFrame += 0.1;
      const p = v.pos;
      const snap = runner.snapshot();
      const ekf: V3 = [v.origin[0] + s.ekfPosNed[1], v.origin[1] + s.ekfPosNed[0], v.origin[2] - s.ekfPosNed[2]];
      frames.push([
        +t.toFixed(2),
        +p[0].toFixed(3), +p[1].toFixed(3), +p[2].toFixed(3),
        +v.yaw.toFixed(3), +v.attitude.roll.toFixed(3), +v.attitude.pitch.toFixed(3),
        +ekf[0].toFixed(3), +ekf[1].toFixed(3), +ekf[2].toFixed(3),
        ...v.vel.map((x) => +x.toFixed(3)),
        ...v.lastSetpoint.map((x) => +x.toFixed(3)),
        ...s.motors.map((x) => +x.toFixed(3)),
        s.customMode, s.armed ? 1 : 0, snap.inside ? 1 : 0,
        +snap.batteryPct.toFixed(1),
        PHASES.indexOf(snap.phase),
      ]);
    }
    if (i % 2000 === 0) console.log(`  t=${t.toFixed(0)}s phase=${runner.snapshot().phase} px4=${m} pos=(${v.pos.map((x) => x.toFixed(1)).join(', ')}) wall=${((Date.now() - t0) / 1000).toFixed(0)}s`);
  }
  link.stop();
  drain();
  const run = runner.record();
  const report = buildReport(run, raw, world);
  px4.kill('SIGTERM');
  await new Promise((r) => setTimeout(r, 300));
  drain();
  void workerDone;
  await worker.terminate();
  const wallSec = (Date.now() - t0) / 1000;
  const summary = {
    schema: 1,
    id,
    case: c,
    decoded: setup.decoded,
    mode,
    flightController: 'PX4 v1.15.4 SITL (EKF2 + multicopter position/attitude/rate control), offboard velocity setpoints at 50 Hz',
    plant: "frontier-fly's own rigid body (FrontierPlant), lockstep HIL at 250 Hz",
    indoorNav,
    failed,
    outcome: run.outcome,
    landedHome: run.landedHome,
    durationSec: Math.round(run.tSec),
    collisions: run.collisions.length,
    falseClears: report.falseClears,
    confirmed: run.confirmed.map((x) => ({ classId: x.classId, conf: +x.confidence.toFixed(3), tp: x.truth.kind === 'tp' })),
    declaredClear: run.declaredClear.map((d) => d.scope),
    entries: run.entries.length,
    gnssDeniedSec: +(v.deniedTicks * 0.05).toFixed(1),
    visionSec: +(v.visionTicks * 0.05).toFixed(1),
    px4: {
      bootSimSec: +bootSec.toFixed(1),
      lockstepTimeouts: v.last.timeouts,
      ekfHorizErrRmsM: ekfErrN ? +Math.sqrt(ekfErrSum / ekfErrN).toFixed(3) : null,
      ekfHorizErrMaxM: +ekfErrMax.toFixed(3),
      modes,
      warnings: events.filter((e) => e.kind === 'statustext' && (e.severity ?? 6) <= 4).map((e) => `${e.simSec}s ${e.text}`),
    },
    wallSec: +wallSec.toFixed(1),
    log: run.log.map((l) => ({ t: +l.tSec.toFixed(1), tone: l.tone, msg: l.message })),
    phases: run.phases,
    ...(has('frames')
      ? { frameColumns: ['t', 'x', 'y', 'z', 'yaw', 'roll', 'pitch', 'ekfX', 'ekfY', 'ekfZ', 'vx', 'vy', 'vz', 'spX', 'spY', 'spZ', 'm0', 'm1', 'm2', 'm3', 'px4Mode', 'armed', 'inside', 'battery', 'phase'], phaseNames: PHASES, frames }
      : {}),
  };
  writeFileSync(join(outDir, `${id}.json`), `${JSON.stringify(summary)}\n`);
  writeFileSync(join(outDir, `${id}.px4.log`), px4Log.join(''));
  if (recorder) {
    const film: Px4FilmFile = {
      schema: PX4_FILM_SCHEMA,
      id,
      ...filmTitle(setup.decoded),
      case: c,
      mode,
      dt: 0.05,
      frames: recorder.frames,
      plans: recorder.plans,
      confirmed: run.confirmed,
      log: run.log,
      // The track and phase list are in the frames already; the film reads entries and confirmations.
      record: { ...run, track: [] },
      report,
      explorer: run.indoorExplorer ?? 'frontier',
      px4: {
        autopilot: 'PX4 v1.15.4 SITL',
        lockstepHz: 250,
        offboardHz: 50,
        indoorNav,
        gnssDeniedSec: summary.gnssDeniedSec,
        visionSec: summary.visionSec,
        ekfHorizErrRmsM: summary.px4.ekfHorizErrRmsM,
        ekfHorizErrMaxM: summary.px4.ekfHorizErrMaxM,
        lockstepTimeouts: summary.px4.lockstepTimeouts,
        params: Object.fromEntries(params.map((p) => [p.id, p.value])),
        wallSec: summary.wallSec,
      },
    };
    writeFileSync(join(outDir, `${id}.film.json`), JSON.stringify(film));
  }
  console.log(
    `${id}: ${run.outcome}, landed home ${run.landedHome}, ${summary.durationSec} s, ${summary.collisions} collision(s), ${summary.falseClears} false clear(s), ` +
      `GNSS denied ${summary.gnssDeniedSec} s, EKF horizontal error RMS ${summary.px4.ekfHorizErrRmsM} m (max ${summary.px4.ekfHorizErrMaxM}), ${summary.px4.lockstepTimeouts} lockstep timeouts, wall ${summary.wallSec} s` +
      (failed ? `, FAILED: ${failed}` : ''),
  );
  process.exit(failed ? 1 : 0);
}

/** Title and one-line blurb for the film's intro card. */
function filmTitle(d: ReturnType<typeof caseSetup>['decoded']): { title: string; blurb: string } {
  const where = `${d.buildings} building${d.buildings === 1 ? '' : 's'}`;
  const light = d.lights === 'off' ? 'lights off' : 'lights on';
  const how =
    'PX4 v1.15.4 flies it in software-in-the-loop: its EKF2 estimator and position, attitude and rate controllers fly every second of the mission, ' +
    'and indoors, with GNSS denied, EKF2 navigates on simulated visual odometry.';
  if (d.mission === 'find-person')
    return { title: 'PX4 in the loop: find the missing person', blurb: `Search ${where} (${light}) for a person who is ${d.personState}. ${how}` };
  if (d.mission === 'find-object') return { title: 'PX4 in the loop: find the dropped object', blurb: `Search ${where} (${light}) for a ${d.object}. ${how}` };
  if (d.mission === 'sweep-occupied')
    return { title: 'PX4 in the loop: sweep an occupied site', blurb: `Prove ${where} clear or find who is inside: a person is ${d.personState}, with ${d.decoys} lookalike decoy(s), ${light}. ${how}` };
  return { title: 'PX4 in the loop: confirm the site is clear', blurb: `Prove ${where} empty with ${d.decoys} lookalike decoy(s), ${light}. ${how}` };
}

const PHASES: readonly Phase[] = ['preflight', 'coverage', 'approach', 'inspect', 'entry_select', 'await_operator', 'fly_through', 'indoor_explore', 'confirm', 'exit', 'return_home', 'land', 'recover', 'done'];

void main();

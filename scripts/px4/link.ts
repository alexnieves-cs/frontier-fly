/**
 * One PX4 SITL instance behind the mission link (src/px4/missionLink*.ts), for scripts that fly
 * several at once (scripts/px4/team.ts). scripts/px4/mission.ts has its own copy of the client.
 *
 * Instance N: PX4's simulator port 4560+N, its offboard link 14580+N (PX4 side) to 14540+N, MAVLink
 * system id N+1, so up to a handful run side by side on one machine.
 */
import { spawn, type ChildProcess } from 'node:child_process';
import { mkdtempSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import { MessageChannel, receiveMessageOnPort, Worker } from 'node:worker_threads';
import { buildSync } from 'esbuild';
import type { V3 } from '../../src/mission/autonomy/contracts';
import type { Px4Link, Px4LinkCommand, Px4LinkState } from '../../src/mission/autonomy/px4Vehicle';
import { missionQuadParams } from '../../src/mission/autonomy/quadVehicle';
import { GEO_ORIGINS } from '../../src/mission/export/geo';
import { CMD, CTL, DATA_LENGTH, OP, PHASE, ST, type LinkEvent, type MissionLinkInit, type Px4Param } from '../../src/px4/missionLink';
import { px4Params } from '../physics/maneuvers';

const HERE = dirname(fileURLToPath(import.meta.url));
const ROOT = resolve(HERE, '../..');

export class LinkClient implements Px4Link {
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
      // The link may already be down.
    }
  }

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

/** The parameters every frontier-fly PX4 flight uses (scripts/px4/mission.ts), with external vision on. */
export function teamPx4Params(): Px4Param[] {
  const params: Px4Param[] = Object.entries(px4Params('bridge')).map(([k, v]) => ({
    id: k,
    value: v,
    type: /^(COM_RCL_EXCEPT|NAV_RCL_ACT)$/u.test(k) ? 'int32' : 'real32',
  }));
  params.push(
    { id: 'NAV_DLL_ACT', value: 0, type: 'int32' },
    { id: 'COM_RC_IN_MODE', value: 4, type: 'int32' },
    { id: 'COM_DISARM_PRFLT', value: -1, type: 'real32' },
    { id: 'COM_DISARM_LAND', value: -1, type: 'real32' },
    { id: 'EKF2_EV_CTRL', value: 5, type: 'int32' },
    { id: 'EKF2_EV_DELAY', value: 0, type: 'real32' },
    { id: 'MPC_XY_VEL_P_ACC', value: 4, type: 'real32' },
    { id: 'MPC_XY_VEL_I_ACC', value: 3, type: 'real32' },
    { id: 'MPC_XY_VEL_D_ACC', value: 0.4, type: 'real32' },
  );
  return params;
}

let bundled: string | null = null;
function workerBundle(): string {
  if (bundled) return bundled;
  bundled = join(mkdtempSync(join(tmpdir(), 'ff-px4team-')), 'missionLink.worker.mjs');
  buildSync({ entryPoints: [join(ROOT, 'src/px4/missionLink.worker.ts')], bundle: true, platform: 'node', format: 'esm', outfile: bundled, logLevel: 'error' });
  return bundled;
}

export interface Px4Instance {
  instance: number;
  link: LinkClient;
  events: LinkEvent[];
  /** Read the worker's queued events (call between ticks). */
  drain(): void;
  bootSimSec: number;
  bootWallSec: number;
  log: string[];
  close(): Promise<void>;
}

/**
 * Boot PX4 instance N on flat ground spanning [x0, x0 + w] x [y0, y0 + d] around the plant's origin
 * (east/north offsets from it, metres), and wait until it holds offboard and is armed.
 */
export async function bootPx4(opts: { instance: number; px4Build: string; speed?: number; seed: number; area: { x0: number; y0: number; w: number; d: number }; params?: Px4Param[]; verbose?: boolean }): Promise<Px4Instance> {
  const { instance } = opts;
  const link = new LinkClient();
  const channel = new MessageChannel();
  const events: LinkEvent[] = [];
  const drain = () => {
    for (;;) {
      const m = receiveMessageOnPort(channel.port1);
      if (!m) break;
      const msg = m.message as { type: 'event'; event: LinkEvent } | { type: 'done' };
      if (msg.type === 'event') {
        events.push(msg.event);
        if (opts.verbose && (msg.event.kind !== 'statustext' || (msg.event.severity ?? 6) <= 4)) console.log(`  px4[${instance}] ${msg.event.simSec.toFixed(1)}s ${msg.event.kind}: ${msg.event.text}`);
      }
    }
  };
  const step = 1;
  const nx = Math.ceil(opts.area.w) + 1;
  const ny = Math.ceil(opts.area.d) + 1;
  const init: MissionLinkInit = {
    ctl: link.ctl.buffer as SharedArrayBuffer,
    data: link.data.buffer as SharedArrayBuffer,
    hilPort: 4560 + instance,
    offboardLocalPort: 14540 + instance,
    offboardRemotePort: 14580 + instance,
    home: GEO_ORIGINS['px4-sitl'],
    params: opts.params ?? teamPx4Params(),
    dyn: missionQuadParams(),
    ground: { originEast: opts.area.x0, originNorth: opts.area.y0, step, nx, ny, down: new Float32Array(nx * ny) },
    seed: opts.seed,
    sysId: instance + 1,
  };
  const worker = new Worker(workerBundle(), { workerData: { init, port: channel.port2 }, transferList: [channel.port2], execArgv: [] });
  worker.on('error', (e) => console.error(`PX4 link worker ${instance} error:`, e));
  await link.waitPhase(PHASE.LISTENING, 60, drain);
  const work = mkdtempSync(join(tmpdir(), `px4-${instance}-`));
  const px4: ChildProcess = spawn(join(opts.px4Build, 'bin/px4'), ['-i', String(instance), '-d', join(opts.px4Build, 'etc')], {
    cwd: work,
    env: { ...process.env, PX4_SYS_AUTOSTART: '10016', PX4_SIM_SPEED_FACTOR: String(opts.speed ?? 1), PX4_HOME_LAT: '473977420', PX4_HOME_LON: '85455940' },
    stdio: ['ignore', 'pipe', 'pipe'],
  });
  const kill = () => {
    if (px4.exitCode === null) px4.kill('SIGKILL');
  };
  process.on('exit', kill);
  const log: string[] = [];
  px4.stdout!.on('data', (b: Buffer) => log.push(b.toString()));
  px4.stderr!.on('data', (b: Buffer) => log.push(b.toString()));
  const wall0 = Date.now();
  await link.waitPhase(PHASE.READY, 300, drain);
  drain();
  return {
    instance,
    link,
    events,
    drain,
    bootSimSec: link.state().simSec,
    bootWallSec: (Date.now() - wall0) / 1000,
    log,
    async close() {
      link.stop();
      drain();
      px4.kill('SIGTERM');
      await new Promise((r) => setTimeout(r, 300));
      kill();
      drain();
      await worker.terminate();
      process.off('exit', kill);
      rmSync(work, { recursive: true, force: true });
    },
  };
}

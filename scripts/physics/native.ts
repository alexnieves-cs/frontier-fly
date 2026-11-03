/**
 * Fly one physics-validation maneuver with frontier-fly's own flight controller.
 *
 *   npx vite-node scripts/physics/native.ts -- --id step-forward --out log.json
 *
 * The vehicle is the mission quad as QuadVehicle flies it outdoors: the same
 * QuadrotorDynamics plant PX4 flies through the bridge, the angle-mode
 * controller with PX4's mixer priority, the velocity integral and the 55°
 * tilt limit. The outer loop is PX4's position controller law with PX4's
 * default gains (MPC_XY_P 0.95, MPC_Z_P 1.0), so the two flights differ only
 * in the velocity and attitude loops below it. Physics at 400 Hz, the outer
 * loop at 50 Hz, the log at 50 Hz.
 */
import { mkdirSync, writeFileSync } from 'node:fs';
import { dirname } from 'node:path';
import { OUTDOOR_LIMITS } from '../../src/mission/autonomy/vehicle';
import { RIG_COLUMNS, RigEnvironment } from '../../src/px4/rig';
import { defaultDynParams } from '../../src/sim/dynamics/dr';
import { QuadrotorDynamics } from '../../src/sim/dynamics/quad';
import { makeDroneState } from '../../src/sim/drone';
import { MANEUVERS, type Maneuver } from './maneuvers';

const PHYSICS_HZ = 400;
const OUTER_HZ = 50;
const LOG_HZ = 50;
const SETTLE_SEC = 10;
const XY_P = 0.95;
const Z_P = 1.0;
/** QuadVehicle's outdoor settings (quadVehicle.ts, phases/core.ts). */
const VEL_KI = 1;
const TILT_RAD = 0.96;
const MOTOR_MAX = 1100;

export interface NativeLog {
  id: string;
  sim: 'native';
  columns: string[];
  rows: number[][];
  segmentStartsUs: number[];
}

export function flyNative(m: Maneuver): NativeLog {
  const env = new RigEnvironment({ wind: m.wind, soc: m.soc, aero: m.aero });
  const dyn = new QuadrotorDynamics();
  const [n0, e0, d0] = m.start;
  const s = makeDroneState([e0, n0, -d0], Math.PI / 2);
  dyn.reset(s, env.params);
  dyn.aero = env.aero;
  const ctrl = dyn.controller;
  const lim = OUTDOOR_LIMITS;
  ctrl.setLimits(lim.speedH, lim.speedV, lim.yawRate, env.params.inertia[0] / defaultDynParams().inertia[0], TILT_RAD);
  ctrl.setAttitudeFirst(true);
  ctrl.setVelocityIntegral(VEL_KI);

  const cmd = new Float32Array(4);
  const dt = 1 / PHYSICS_HZ;
  const outerEvery = PHYSICS_HZ / OUTER_HZ;
  const logEvery = PHYSICS_HZ / LOG_HZ;
  const rows: number[][] = [];
  const starts: number[] = [];
  const total = m.segments.reduce((a, g) => a + g.dur, 0);
  const steps = Math.round((SETTLE_SEC + total) * PHYSICS_HZ);
  let velNed: [number, number, number] = [0, 0, 0];
  let segIdx = -1;
  let segEnd = 0;
  const hold = { pos: m.start as (number | null)[], vel: [null, null, null] as (number | null)[] };
  let sp = hold;
  const r3 = (v: number): number => Math.round(v * 1000) / 1000;
  const deg = 180 / Math.PI;

  for (let k = 0; k < steps; k++) {
    const t = k * dt - SETTLE_SEC;
    if (t >= 0 && (segIdx < 0 || t >= segEnd - 1e-9) && segIdx < m.segments.length - 1) {
      segIdx++;
      const g = m.segments[segIdx]!;
      sp = { pos: g.pos ?? [null, null, null], vel: g.vel ?? [null, null, null] };
      segEnd += g.dur;
      starts.push(Math.round(t * 1e6));
    }
    const pos: [number, number, number] = [s.pos[1], s.pos[0], -s.pos[2]];
    if (k % outerEvery === 0) {
      const v: [number, number, number] = [0, 0, 0];
      for (let i = 0; i < 3; i++) {
        const p = sp.pos[i];
        const ff = sp.vel[i];
        v[i] = (ff ?? 0) + (p === null || p === undefined ? 0 : (i < 2 ? XY_P : Z_P) * (p - pos[i]!));
      }
      const h = Math.hypot(v[0], v[1]);
      if (h > lim.speedH) {
        v[0] *= lim.speedH / h;
        v[1] *= lim.speedH / h;
      }
      v[2] = Math.max(-lim.speedV, Math.min(lim.speedV, v[2]));
      velNed = v;
    }
    const w = env.windEnu(t >= 0 ? t : null);
    dyn.wind[0] = w[0];
    dyn.wind[1] = w[1];
    dyn.wind[2] = w[2];
    env.updatePack(s.motors);
    // World ENU velocity command to the body frame, heading held north.
    const ve = velNed[1];
    const vn = velNed[0];
    const c = Math.cos(s.yaw);
    const sn = Math.sin(s.yaw);
    cmd[0] = c * ve + sn * vn;
    cmd[1] = -sn * ve + c * vn;
    cmd[2] = -velNed[2];
    let yawErr = Math.PI / 2 - s.yaw;
    yawErr -= Math.round(yawErr / (Math.PI * 2)) * Math.PI * 2;
    cmd[3] = 2 * yawErr;
    dyn.step(cmd, dt);
    if (s.pos[2] < 0) {
      // The ground: a level surface the low hover must not reach.
      s.pos[2] = 0;
      if (s.vel[2] < 0) s.vel[2] = 0;
    }
    if (t >= -5 && k % logEvery === 0) {
      rows.push([
        Math.round(t * 1e6),
        r3(s.pos[1]),
        r3(s.pos[0]),
        r3(-s.pos[2]),
        r3(s.vel[1]),
        r3(s.vel[0]),
        r3(-s.vel[2]),
        // FLU roll and pitch about east/north at a north heading: report as PX4's FRD roll and pitch.
        r3(s.roll * deg),
        r3(-s.pitch * deg),
        r3((Math.PI / 2 - s.yaw) * deg),
        ...Array.from(s.motors, (x) => r3((x ?? 0) / MOTOR_MAX)),
        r3(w[1]),
        r3(w[0]),
        r3(env.packV),
        r3(env.aero?.motorMaxScale ?? 1),
      ]);
    }
  }
  return { id: m.id, sim: 'native', columns: RIG_COLUMNS, rows, segmentStartsUs: starts };
}

if (process.env.VITEST === undefined && process.argv.includes('--id')) {
  const id = process.argv[process.argv.indexOf('--id') + 1]!;
  const out = process.argv.includes('--out') ? process.argv[process.argv.indexOf('--out') + 1]! : null;
  const m = MANEUVERS.find((x) => x.id === id);
  if (!m) throw new Error(`no maneuver ${id}`);
  const log = flyNative(m);
  if (out) {
    mkdirSync(dirname(out), { recursive: true });
    writeFileSync(out, `${JSON.stringify(log)}\n`);
  }
  const last = log.rows[log.rows.length - 1]!;
  console.log(`${id} native: ${log.rows.length} rows, end at n ${last[1]} e ${last[2]} d ${last[3]}`);
}

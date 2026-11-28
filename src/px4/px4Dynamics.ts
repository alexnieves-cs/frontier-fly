import { MAX_SPEED_H, MAX_SPEED_V, MAX_YAW_RATE } from '../shared/constants';
import type { DroneState, DynParams, Dynamics } from '../shared/types';
import type { Px4Link, Px4LinkState, VisionSource } from '../mission/autonomy/px4Vehicle';
import { LINK_STEP_US } from './missionLink';

type V3 = [number, number, number];

/**
 * An explorer drone flown by PX4 SITL.
 *
 * ExplorerEnv's planners command body-frame velocity and a yaw rate every 50 ms. Here that command
 * goes to one PX4 SITL instance as an offboard velocity setpoint, and PX4's EKF2 and multicopter
 * controllers fly frontier-fly's own rigid body (FrontierPlant, in the link's worker) for those
 * 50 ms in lockstep. What comes back is where the airframe really went, its attitude and motors.
 *
 * The plant knows the ground but not the walls. The env resolves each tick's motion against the
 * building after this step (ExplorerEnv.step), so when it finds the drone moved since this wrote
 * the state, a wall stopped it: the plant is put back where the wall left it before PX4's next tick,
 * as Px4Vehicle does for the mission airframe.
 *
 * Indoors there is no GNSS. With `vision`, PX4 fuses external vision (VIO-grade position and
 * velocity from the caller) instead, so its controllers fly on an estimate that drifts; the explorer
 * and its maps still use the true pose (the same split the mission runner makes).
 */
export interface Px4DynamicsOptions {
  /** World point (env frame, z up) of the plant's NED origin. */
  origin: V3;
  /** External vision for PX4. Without it PX4 keeps its GNSS fix. */
  vision?: VisionSource;
}

const enu = (n: readonly number[]): V3 => [n[1]!, n[0]!, -n[2]!];
const ned = (e: readonly number[]): V3 => [e[1]!, e[0]!, -e[2]!];
const clamp = (v: number, lo: number, hi: number) => (v < lo ? lo : v > hi ? hi : v);

export class Px4Dynamics implements Dynamics {
  /** Last state PX4 and the plant reported. */
  last: Px4LinkState;
  /** World-frame velocity setpoint sent on the last tick. */
  setpoint: V3 = [0, 0, 0];
  /** Ticks flown, walls that put the plant back, and PX4's horizontal estimate error against the truth. */
  ticks = 0;
  teleports = 0;
  ekfErrMax = 0;
  private ekfErrSq = 0;
  private ekfErrN = 0;
  private trackErrSq = 0;
  private s: DroneState | null = null;
  private speedH = MAX_SPEED_H;
  private readonly written: V3 = [Number.NaN, Number.NaN, Number.NaN];
  private readonly thrust = [1, 1, 1, 1];
  private readonly zero: V3 = [0, 0, 0];
  /** Sim time asked for but not yet flown: a link advance is a whole number of 4 ms lockstep steps. */
  private owed = 0;

  constructor(
    private readonly link: Px4Link,
    first: Px4LinkState,
    private readonly opts: Px4DynamicsOptions,
  ) {
    this.last = first;
  }

  setSpeedLimit(speedH: number): void {
    this.speedH = speedH;
  }

  reset(s: DroneState, p: DynParams): void {
    void p;
    this.s = s;
    if (s.motors.length < 4) s.motors = new Float32Array(4);
    this.absorb(this.last);
  }

  /** The drone state this flies (the env's own object; null before reset). */
  get state(): DroneState | null {
    return this.s;
  }

  /** RMS of PX4's horizontal position error, m. */
  get ekfErrRms(): number {
    return this.ekfErrN ? Math.sqrt(this.ekfErrSq / this.ekfErrN) : 0;
  }

  /** RMS of the horizontal velocity error against the setpoint, m/s. */
  get trackErrRms(): number {
    return this.ticks ? Math.sqrt(this.trackErrSq / this.ticks) : 0;
  }

  step(cmd: Float32Array, dt: number): DroneState {
    const s = this.s;
    if (!s) throw new Error('Px4Dynamics.step before reset');
    const o = this.opts.origin;
    let teleport: { posNed: V3; velNed: V3 } | null = null;
    const moved = Math.hypot(s.pos[0] - this.written[0], s.pos[1] - this.written[1], s.pos[2] - this.written[2]);
    if (moved > 1e-6) {
      teleport = { posNed: ned([s.pos[0] - o[0], s.pos[1] - o[1], s.pos[2] - o[2]]), velNed: ned(s.vel) };
      this.teleports++;
    }
    const vxB = clamp(cmd[0] ?? 0, -this.speedH, this.speedH);
    const vyB = clamp(cmd[1] ?? 0, -this.speedH, this.speedH);
    const vz = clamp(cmd[2] ?? 0, -MAX_SPEED_V, MAX_SPEED_V);
    const rate = clamp(cmd[3] ?? 0, -MAX_YAW_RATE, MAX_YAW_RATE);
    const cy = Math.cos(s.yaw);
    const sy = Math.sin(s.yaw);
    const sp: V3 = [cy * vxB - sy * vyB, sy * vxB + cy * vyB, vz];
    this.setpoint = sp;

    let ev: Parameters<Px4Link['advance']>[0]['ev'] = null;
    const v = this.opts.vision?.({ pos: s.pos, vel: s.vel, tSec: s.t });
    if (v) ev = { posNed: ned([v.pos[0] - o[0], v.pos[1] - o[1], v.pos[2] - o[2]]), velNed: ned(v.vel), posVar: v.posVar, velVar: v.velVar, reset: v.reset };

    // 50 ms is 12.5 lockstep steps: alternate 12 and 13 so PX4's clock keeps pace with the env's.
    const stepS = LINK_STEP_US / 1e6;
    this.owed += dt;
    const n = Math.max(1, Math.round(this.owed / stepS));
    this.owed -= n * stepS;
    const st = this.link.advance({ dt: n * stepS, velNed: ned(sp), yawRateDown: -rate, windNed: this.zero, teleport, thrust: this.thrust, gpsFix: !this.opts.vision, ev });
    this.last = st;
    this.ticks++;
    this.absorb(st);
    s.t += dt;
    const e = Math.hypot(st.ekfPosNed[0] - st.posNed[0], st.ekfPosNed[1] - st.posNed[1]);
    this.ekfErrMax = Math.max(this.ekfErrMax, e);
    this.ekfErrSq += e * e;
    this.ekfErrN++;
    this.trackErrSq += (s.vel[0] - sp[0]) ** 2 + (s.vel[1] - sp[1]) ** 2;
    return s;
  }

  private absorb(st: Px4LinkState): void {
    const s = this.s!;
    const o = this.opts.origin;
    const p = enu(st.posNed);
    s.pos[0] = o[0] + p[0];
    s.pos[1] = o[1] + p[1];
    s.pos[2] = o[2] + p[2];
    const v = enu(st.velNed);
    s.vel[0] = v[0];
    s.vel[1] = v[1];
    s.vel[2] = v[2];
    this.written[0] = s.pos[0];
    this.written[1] = s.pos[1];
    this.written[2] = s.pos[2];
    const [w, x, y, z] = st.q;
    // NED/FRD Euler angles to the env's z-up frame: heading from north clockwise becomes yaw from
    // +x counter-clockwise; FLU pitch is positive nose down.
    const psi = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    const pitch = -Math.asin(clamp(2 * (w * y - z * x), -1, 1));
    const tau = Math.PI * 2;
    const yaw = (((Math.PI / 2 - psi) % tau) + tau) % tau;
    s.yaw = yaw;
    s.roll = roll;
    s.pitch = pitch;
    // Body to world, z-y-x: yaw, then pitch, then roll (xyzw).
    const [cr, sr] = [Math.cos(roll / 2), Math.sin(roll / 2)];
    const [cp, spi] = [Math.cos(pitch / 2), Math.sin(pitch / 2)];
    const [cyw, syw] = [Math.cos(yaw / 2), Math.sin(yaw / 2)];
    s.quat[0] = sr * cp * cyw - cr * spi * syw;
    s.quat[1] = cr * spi * cyw + sr * cp * syw;
    s.quat[2] = cr * cp * syw - sr * spi * cyw;
    s.quat[3] = cr * cp * cyw + sr * spi * syw;
    for (let i = 0; i < 4; i++) s.motors[i] = st.motors[i] ?? 0;
  }
}

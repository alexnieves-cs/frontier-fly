import type { DroneState, DynParams, Dynamics } from '../../shared/types';
import { GRAVITY, MAX_SPEED_V, MAX_YAW_RATE } from '../../shared/constants';
import { integrateQuat, quatFromEulerZXY, quatNormalize, quatToEulerInto, rotateBodyToWorld, rotateWorldToBody } from './spatial';
import { AngleController } from './controller';

const MOTOR_MAX = 1100;
const ACCEL_MAX = 50;
const VEL_MAX = 20;
const OMEGA_MAX = 30;
/**
 * Induced-drag force (N per m/s) at hover rotor speed.
 * Applied as 0.02 * (Σ|ω| / Σ|ω|_hover) so the gain stays small; a raw
 * 0.02 * Σ|ω| with ω in rad/s would be tens of newtons and could not track 1 m/s.
 */
const INDUCED = 0.02;

function clamp(x: number, lo: number, hi: number): number {
  return x < lo ? lo : x > hi ? hi : x;
}

function isZeroQuat(q: ArrayLike<number>): boolean {
  return q[0] === 0 && q[1] === 0 && q[2] === 0 && q[3] === 0;
}

function isIdentityish(q: ArrayLike<number>): boolean {
  return Math.abs(q[0] ?? 0) < 1e-8 && Math.abs(q[1] ?? 0) < 1e-8 && Math.abs(q[2] ?? 0) < 1e-8;
}

/**
 * X-config quadrotor. One semi-implicit Euler step per `step` call.
 * Motors: 0 front-right, 1 rear-right, 2 rear-left, 3 front-left.
 * Spin signs for yaw reaction: +1, -1, +1, -1.
 */
export class QuadrotorDynamics implements Dynamics {
  private state: DroneState | null = null;
  private readonly ctrl = new AngleController();
  private mass = 0.45;
  private ix = 0.0022;
  private iy = 0.0024;
  private iz = 0.0036;
  private kf = 1e-5;
  private km = 1e-7;
  private motorTau = 0.03;
  private dLx = 0.12;
  private dLy = 0.12;
  private dLz = 0.18;
  private dQx = 0.05;
  private dQy = 0.05;
  private dQz = 0.06;
  private wHover = 700;
  private readonly px = new Float64Array(4);
  private readonly py = new Float64Array(4);
  private readonly spin = new Float64Array([1, -1, 1, -1]);
  private readonly omega = new Float64Array(3);
  private readonly wCmd = new Float32Array(4);
  private readonly bodyVel: [number, number, number] = [0, 0, 0];
  private readonly bodyForce: [number, number, number] = [0, 0, 0];
  private readonly worldForce: [number, number, number] = [0, 0, 0];
  private readonly euler = { roll: 0, pitch: 0, yaw: 0 };
  /** Air velocity in the world frame, m/s: drag acts on velocity relative to it. */
  readonly wind: [number, number, number] = [0, 0, 0];
  /** Per-motor thrust authority, 0..1 (a damaged prop or a weak motor). */
  readonly thrustScale = new Float64Array([1, 1, 1, 1]);
  /** Body rates of the last step, rad/s (what a gyro measures). */
  get bodyRates(): ArrayLike<number> {
    return this.omega;
  }
  /** Specific force of the last step in the body frame, m/s² (what an accelerometer measures). */
  readonly specificForce: [number, number, number] = [0, 0, 0];

  setSpeedLimit(speedH: number): void {
    this.ctrl.setLimits(speedH, MAX_SPEED_V, MAX_YAW_RATE);
  }

  /** The flight controller, so a vehicle can set its limits and gains. */
  get controller(): AngleController {
    return this.ctrl;
  }

  reset(s: DroneState, p: DynParams): void {
    this.state = s;
    this.mass = p.mass > 1e-6 ? p.mass : 1e-6;
    this.ix = p.inertia[0] > 1e-8 ? p.inertia[0] : 1e-8;
    this.iy = p.inertia[1] > 1e-8 ? p.inertia[1] : 1e-8;
    this.iz = p.inertia[2] > 1e-8 ? p.inertia[2] : 1e-8;
    const arm = p.arm > 1e-4 ? p.arm : 1e-4;
    this.kf = p.kf > 1e-12 ? p.kf : 1e-12;
    this.km = p.km;
    this.motorTau = p.motorTau > 1e-5 ? p.motorTau : 1e-5;
    this.dLx = p.dragLin[0];
    this.dLy = p.dragLin[1];
    this.dLz = p.dragLin[2];
    this.dQx = p.dragQuad[0];
    this.dQy = p.dragQuad[1];
    this.dQz = p.dragQuad[2];
    const a = arm * Math.SQRT1_2;
    this.px[0] = a;
    this.py[0] = -a;
    this.px[1] = -a;
    this.py[1] = -a;
    this.px[2] = -a;
    this.py[2] = a;
    this.px[3] = a;
    this.py[3] = a;
    const hoverF = (this.mass * GRAVITY) / 4;
    this.wHover = Math.sqrt(hoverF / this.kf);

    if (!s.quat || isZeroQuat(s.quat)) {
      s.quat = [0, 0, 0, 1];
    }
    quatNormalize(s.quat, s.quat);
    if (isIdentityish(s.quat)) {
      quatFromEulerZXY(0, 0, s.yaw, s.quat);
    }
    quatToEulerInto(s.quat, this.euler);
    s.roll = this.euler.roll;
    s.pitch = this.euler.pitch;
    s.yaw = this.euler.yaw;

    if (!s.motors || s.motors.length !== 4) {
      s.motors = new Float32Array(4);
    }
    let msum = 0;
    for (let i = 0; i < 4; i++) msum += s.motors[i] ?? 0;
    if (msum === 0) {
      const w = this.wHover;
      s.motors[0] = w;
      s.motors[1] = w;
      s.motors[2] = w;
      s.motors[3] = w;
    }

    this.omega[0] = 0;
    this.omega[1] = 0;
    this.omega[2] = 0;
    this.ctrl.setParams(p);
    this.ctrl.reset();
  }

  step(cmd: Float32Array, dt: number): DroneState {
    const s = this.state;
    if (!s) throw new Error('QuadrotorDynamics.reset must be called before step');
    const motors = s.motors;

    this.ctrl.compute(
      s.roll,
      s.pitch,
      s.yaw,
      s.vel[0],
      s.vel[1],
      s.vel[2],
      this.omega[0] ?? 0,
      this.omega[1] ?? 0,
      this.omega[2] ?? 0,
      cmd,
      dt,
      this.wCmd,
    );

    const lag = 1 - Math.exp(-dt / this.motorTau);
    for (let i = 0; i < 4; i++) {
      const cur = motors[i] ?? 0;
      motors[i] = clamp(cur + ((this.wCmd[i] ?? 0) - cur) * lag, 0, MOTOR_MAX);
    }

    const z = s.pos[2];
    let ge = 1;
    if (z < 0.4) {
      const depth = z > 0 ? 0.4 - z : 0.4;
      ge = 1 + 0.08 * depth;
    }

    let thrust = 0;
    let tx = 0;
    let ty = 0;
    let tz = 0;
    let wSum = 0;
    const kf = this.kf;
    const km = this.km;
    for (let i = 0; i < 4; i++) {
      const w = motors[i] ?? 0;
      const aw = w < 0 ? -w : w;
      wSum += aw;
      const f = kf * w * w * ge * (this.thrustScale[i] ?? 1);
      thrust += f;
      tx += (this.py[i] ?? 0) * f;
      ty += -(this.px[i] ?? 0) * f;
      tz += (this.spin[i] ?? 0) * km * w * w;
    }

    const q = s.quat;
    rotateWorldToBody(q, s.vel, this.bodyVel);
    const wRef = 4 * this.wHover;
    const ind = wRef > 0 ? INDUCED * (wSum / wRef) : 0;
    this.bodyForce[0] = -ind * this.bodyVel[0];
    this.bodyForce[1] = -ind * this.bodyVel[1];
    this.bodyForce[2] = thrust;
    rotateBodyToWorld(q, this.bodyForce, this.worldForce);

    const vx = s.vel[0];
    const vy = s.vel[1];
    const vz = s.vel[2];
    const rx = vx - this.wind[0];
    const ry = vy - this.wind[1];
    const rz = vz - this.wind[2];
    this.worldForce[0] += -this.dLx * rx - this.dQx * Math.abs(rx) * rx;
    this.worldForce[1] += -this.dLy * ry - this.dQy * Math.abs(ry) * ry;
    this.worldForce[2] += -this.mass * GRAVITY - this.dLz * rz - this.dQz * Math.abs(rz) * rz;
    // An accelerometer reads everything but gravity.
    const invMass = 1 / this.mass;
    this.specificForce[0] = this.worldForce[0];
    this.specificForce[1] = this.worldForce[1];
    this.specificForce[2] = this.worldForce[2] + this.mass * GRAVITY;
    rotateWorldToBody(q, this.specificForce, this.specificForce);
    this.specificForce[0] *= invMass;
    this.specificForce[1] *= invMass;
    this.specificForce[2] *= invMass;

    const invM = 1 / this.mass;
    const ax = clamp(this.worldForce[0] * invM, -ACCEL_MAX, ACCEL_MAX);
    const ay = clamp(this.worldForce[1] * invM, -ACCEL_MAX, ACCEL_MAX);
    const az = clamp(this.worldForce[2] * invM, -ACCEL_MAX, ACCEL_MAX);

    s.vel[0] = clamp(vx + ax * dt, -VEL_MAX, VEL_MAX);
    s.vel[1] = clamp(vy + ay * dt, -VEL_MAX, VEL_MAX);
    s.vel[2] = clamp(vz + az * dt, -VEL_MAX, VEL_MAX);
    s.pos[0] += s.vel[0] * dt;
    s.pos[1] += s.vel[1] * dt;
    s.pos[2] += s.vel[2] * dt;

    const wx = this.omega[0] ?? 0;
    const wy = this.omega[1] ?? 0;
    const wz = this.omega[2] ?? 0;
    const ix = this.ix;
    const iy = this.iy;
    const iz = this.iz;
    const cx = wy * wz * (iz - iy);
    const cy = wz * wx * (ix - iz);
    const cz = wx * wy * (iy - ix);
    const alphax = (tx - cx) / ix;
    const alphay = (ty - cy) / iy;
    const alphaz = (tz - cz) / iz;
    this.omega[0] = clamp(wx + alphax * dt, -OMEGA_MAX, OMEGA_MAX);
    this.omega[1] = clamp(wy + alphay * dt, -OMEGA_MAX, OMEGA_MAX);
    this.omega[2] = clamp(wz + alphaz * dt, -OMEGA_MAX, OMEGA_MAX);

    integrateQuat(q, this.omega, dt, q);
    quatToEulerInto(q, this.euler);
    s.roll = this.euler.roll;
    s.pitch = this.euler.pitch;
    s.yaw = this.euler.yaw;
    s.t += dt;
    return s;
  }
}

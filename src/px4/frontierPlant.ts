import { makeDroneState } from '../sim/drone';
import { QuadrotorDynamics } from '../sim/dynamics/quad';
import type { AeroModel } from '../sim/dynamics/aero';
import type { DroneState, DynParams } from '../shared/types';
import { GRAVITY, type Quat, type QuadState, type Vec3 } from './quad';

/**
 * Frontier-fly's own quadrotor plant (QuadrotorDynamics, the rigid body the
 * missions and the policy fly) with PX4 as its flight controller.
 *
 * Px4Quad (quad.ts) is a separate motor-level model of the 3DR Iris, made so
 * PX4's default gains fly the bridge. This adapter instead puts PX4 on
 * frontier-fly's own airframe, so the physics validation
 * (scripts/physics/) compares like with like: the same plant under PX4's
 * controllers, under PX4's own simulator (SIH) with matched constants, and
 * under frontier-fly's controller.
 *
 * Frames: frontier-fly is world ENU (x east, y north, z up) and body FLU;
 * PX4 is NED and FRD. Motor i here is PX4 rotor i; `px4Geometry` gives the
 * CA_ROTORn_* parameters that tell PX4's control allocation where each one
 * is and which way it spins. A PX4 output u (0..1) commands rotor speed
 * u · MOTOR_MAX, so thrust goes with u², which PX4 is told with
 * THR_MDL_FAC = 1.
 */

export const MOTOR_MAX = 1100;

/** Body FLU positions of QuadrotorDynamics' rotors (0 front-right, 1 rear-right, 2 rear-left, 3 front-left), as multiples of arm/√2. */
const LAYOUT: readonly [number, number, number][] = [
  // [x, y, spin]: spin +1 pushes the body counter-clockwise seen from above (the rotor turns clockwise).
  [1, -1, 1],
  [-1, -1, -1],
  [-1, 1, 1],
  [1, 1, -1],
];

export interface RotorParam {
  /** Forward, m (CA_ROTORn_PX). */
  px: number;
  /** Right, m (CA_ROTORn_PY). */
  py: number;
  /** Drag torque over thrust, m, positive for a counter-clockwise rotor (CA_ROTORn_KM). */
  km: number;
}

/** The CA_ROTORn_* parameters for an airframe. */
export function px4Geometry(p: DynParams): RotorParam[] {
  const a = p.arm * Math.SQRT1_2;
  const ratio = p.km / p.kf;
  // QuadrotorDynamics: yaw torque about FLU +z is spin · km · ω². PX4: torque about FRD +z is KM · thrust.
  return LAYOUT.map(([x, y, spin]) => ({ px: x * a, py: -y * a, km: -spin * ratio }));
}

/** World ENU (x east, y north, z up) to NED. */
function enuToNed(v: ArrayLike<number>): Vec3 {
  return [v[1] ?? 0, v[0] ?? 0, -(v[2] ?? 0)];
}

/** Body FLU to world ENU (xyzw) as body FRD to world NED (wxyz). */
export function nedQuat(q: ArrayLike<number>): Quat {
  const x = q[0] ?? 0;
  const y = q[1] ?? 0;
  const z = q[2] ?? 0;
  const w = q[3] ?? 1;
  // R (FLU -> ENU).
  const r00 = 1 - 2 * (y * y + z * z);
  const r01 = 2 * (x * y - z * w);
  const r02 = 2 * (x * z + y * w);
  const r10 = 2 * (x * y + z * w);
  const r11 = 1 - 2 * (x * x + z * z);
  const r12 = 2 * (y * z - x * w);
  const r20 = 2 * (x * z - y * w);
  const r21 = 2 * (y * z + x * w);
  const r22 = 1 - 2 * (x * x + y * y);
  // M = P R F with P: ENU -> NED (swap the first two rows, negate the third) and F = diag(1, -1, -1) (FRD -> FLU).
  const m00 = r10;
  const m01 = -r11;
  const m02 = -r12;
  const m10 = r00;
  const m11 = -r01;
  const m12 = -r02;
  const m20 = -r20;
  const m21 = r21;
  const m22 = r22;
  // Rotation matrix to quaternion (Shepperd).
  const tr = m00 + m11 + m22;
  let qw: number;
  let qx: number;
  let qy: number;
  let qz: number;
  if (tr > 0) {
    const s = Math.sqrt(tr + 1) * 2;
    qw = 0.25 * s;
    qx = (m21 - m12) / s;
    qy = (m02 - m20) / s;
    qz = (m10 - m01) / s;
  } else if (m00 > m11 && m00 > m22) {
    const s = Math.sqrt(1 + m00 - m11 - m22) * 2;
    qw = (m21 - m12) / s;
    qx = 0.25 * s;
    qy = (m01 + m10) / s;
    qz = (m02 + m20) / s;
  } else if (m11 > m22) {
    const s = Math.sqrt(1 + m11 - m00 - m22) * 2;
    qw = (m02 - m20) / s;
    qx = (m01 + m10) / s;
    qy = 0.25 * s;
    qz = (m12 + m21) / s;
  } else {
    const s = Math.sqrt(1 + m22 - m00 - m11) * 2;
    qw = (m10 - m01) / s;
    qx = (m02 + m20) / s;
    qy = (m12 + m21) / s;
    qz = 0.25 * s;
  }
  if (qw < 0) return [-qw, -qx, -qy, -qz];
  return [qw, qx, qy, qz];
}

/**
 * Same surface as Px4Quad (what Px4SimSession reads and drives), backed by
 * QuadrotorDynamics. State is NED and FRD, refreshed after every step.
 */
export class FrontierPlant {
  readonly dyn = new QuadrotorDynamics();
  readonly state: DroneState;
  readonly params: DynParams;
  readonly s: QuadState;
  specificForce: Vec3 = [0, 0, -GRAVITY];
  /** World-frame wind, NED m/s. */
  wind: Vec3 = [0, 0, 0];
  /** Ground height under a NED position, as a down coordinate (0 at the pad). */
  groundDown: (north: number, east: number) => number = () => 0;
  private readonly wCmd = new Float32Array(4);

  constructor(params: DynParams, aero: AeroModel | null = null) {
    this.params = params;
    // Heading north: PX4's yaw 0.
    this.state = makeDroneState([0, 0, 0], Math.PI / 2);
    this.dyn.reset(this.state, params);
    this.state.motors.fill(0);
    this.dyn.aero = aero;
    this.s = { pos: [0, 0, 0], vel: [0, 0, 0], q: [1, 0, 0, 0], omega: [0, 0, 0], rotor: [0, 0, 0, 0], onGround: true };
    this.dyn.holdOnGround();
    this.sync(true);
  }

  get aero(): AeroModel | null {
    return this.dyn.aero;
  }

  /** Advance by dt under PX4's motor outputs (0..1). All zero is disarmed: the rotors spin down. */
  step(u: ArrayLike<number>, dt: number): void {
    const st = this.state;
    for (let i = 0; i < 4; i++) this.wCmd[i] = Math.max(0, Math.min(1, u[i] ?? 0)) * MOTOR_MAX;
    // NED wind to ENU.
    this.dyn.wind[0] = this.wind[1];
    this.dyn.wind[1] = this.wind[0];
    this.dyn.wind[2] = -this.wind[2];
    this.dyn.stepPlant(this.wCmd, dt);
    // Ground contact, as Px4Quad: the ground holds the vehicle up, and with thrust below weight it rests.
    const groundUp = -this.groundDown(st.pos[1], st.pos[0]);
    let resting = false;
    if (st.pos[2] <= groundUp) {
      st.pos[2] = groundUp;
      if (st.vel[2] < 0) st.vel[2] = 0;
      let thrust = 0;
      for (let i = 0; i < 4; i++) thrust += this.params.kf * (st.motors[i] ?? 0) ** 2;
      if (thrust < this.params.mass * GRAVITY) {
        this.dyn.holdOnGround();
        resting = true;
      }
    }
    this.sync(resting);
  }

  private sync(onGround: boolean): void {
    const st = this.state;
    this.s.pos = enuToNed(st.pos);
    this.s.vel = enuToNed(st.vel);
    this.s.q = nedQuat(st.quat);
    const w = this.dyn.bodyRates;
    this.s.omega = [w[0] ?? 0, -(w[1] ?? 0), -(w[2] ?? 0)];
    this.s.rotor = Array.from(st.motors);
    this.s.onGround = onGround;
    const f = this.dyn.specificForce;
    this.specificForce = [f[0], -f[1], -f[2]];
  }
}

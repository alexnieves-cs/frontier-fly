/**
 * A motor-level quadrotor for PX4 software-in-the-loop.
 *
 * Frontier-fly's own vehicles take velocity or attitude commands and close
 * their loops internally. Under PX4 the autopilot is the controller, so this
 * model takes PX4's four motor outputs (0..1) and nothing else: rotor speed
 * with a first-order lag, thrust and drag torque per rotor, rigid-body
 * rotation, linear air drag and a ground contact.
 *
 * Frames follow PX4: world NED (north, east, down) with the origin at the
 * takeoff pad, body FRD (forward, right, down). Quaternions are [w, x, y, z],
 * body to world.
 *
 * The airframe matches PX4's generic quadrotor SITL airframe 10016_none_iris
 * (rotor positions and spin directions from its CA_ROTOR* parameters); mass,
 * inertia and rotor constants are those of the 3DR Iris model PX4's own
 * simulators use, so PX4's default gains fly it as they fly theirs.
 */

export const GRAVITY = 9.80665;

export interface Rotor {
  /** Position in the body frame, meters (forward, right). */
  x: number;
  y: number;
  /** Drag-torque to thrust ratio, meters. Positive spins counter-clockwise seen from above (PX4 CA_ROTORn_KM sign). */
  km: number;
}

export interface QuadParams {
  massKg: number;
  inertia: [number, number, number];
  rotors: Rotor[];
  /** Thrust per rotor speed squared, N/(rad/s)². */
  kf: number;
  /** Rotor speed for an output of 0 (armed idle) and the gain per unit output, rad/s. */
  omegaIdle: number;
  omegaScale: number;
  /** Rotor speed time constant, s. */
  motorTau: number;
  /** Linear air drag, N per m/s, body axes (forward, right, down). */
  drag: [number, number, number];
  /** Rotational damping, N·m per rad/s. */
  angDamping: number;
}

export const IRIS: QuadParams = {
  massKg: 1.5,
  inertia: [0.029125, 0.029125, 0.055225],
  rotors: [
    { x: 0.1515, y: 0.245, km: 0.06 },
    { x: -0.1515, y: -0.1875, km: 0.06 },
    { x: 0.1515, y: -0.245, km: -0.06 },
    { x: -0.1515, y: 0.1875, km: -0.06 },
  ],
  kf: 5.84e-6,
  omegaIdle: 100,
  omegaScale: 1000,
  motorTau: 0.02,
  drag: [0.25, 0.25, 0.4],
  angDamping: 0.004,
};

export type Vec3 = [number, number, number];
export type Quat = [number, number, number, number];

export function rotate(q: Quat, v: Vec3): Vec3 {
  const [w, x, y, z] = q;
  // v' = q v q*, expanded.
  const tx = 2 * (y * v[2] - z * v[1]);
  const ty = 2 * (z * v[0] - x * v[2]);
  const tz = 2 * (x * v[1] - y * v[0]);
  return [v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)];
}

export function rotateInv(q: Quat, v: Vec3): Vec3 {
  return rotate([q[0], -q[1], -q[2], -q[3]], v);
}

export function quatFromYaw(yaw: number): Quat {
  return [Math.cos(yaw / 2), 0, 0, Math.sin(yaw / 2)];
}

/** Roll, pitch, yaw (radians) of a body-to-NED quaternion, ZYX order. */
export function eulerOf(q: Quat): { roll: number; pitch: number; yaw: number } {
  const [w, x, y, z] = q;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return { roll, pitch, yaw };
}

export interface QuadState {
  /** NED meters from the pad. */
  pos: Vec3;
  vel: Vec3;
  /** Body to NED. */
  q: Quat;
  /** Body rates, FRD, rad/s. */
  omega: Vec3;
  /** Rotor speeds, rad/s. */
  rotor: number[];
  onGround: boolean;
}

export class Px4Quad {
  readonly p: QuadParams;
  readonly s: QuadState;
  /** Specific force of the last step, body FRD, m/s²: what an accelerometer reads. */
  specificForce: Vec3 = [0, 0, -GRAVITY];
  /** World-frame wind, NED m/s. */
  wind: Vec3 = [0, 0, 0];
  /** Ground height under a NED position, as a down coordinate (0 at the pad, negative on higher ground). */
  groundDown: (north: number, east: number) => number = () => 0;

  constructor(params: QuadParams = IRIS, yaw = 0) {
    this.p = params;
    this.s = {
      pos: [0, 0, 0],
      vel: [0, 0, 0],
      q: quatFromYaw(yaw),
      omega: [0, 0, 0],
      rotor: params.rotors.map(() => 0),
      onGround: true,
    };
  }

  /**
   * Advance by dt seconds under motor outputs u (0..1 per rotor). Outputs of
   * exactly zero on every rotor mean disarmed: the rotors spin down.
   */
  step(u: ArrayLike<number>, dt: number): void {
    const { p, s } = this;
    const armed = p.rotors.some((_, i) => (u[i] ?? 0) > 0);
    const lag = 1 - Math.exp(-dt / p.motorTau);
    let thrust = 0;
    const torque: Vec3 = [0, 0, 0];
    for (let i = 0; i < p.rotors.length; i++) {
      const r = p.rotors[i]!;
      const ui = Math.max(0, Math.min(1, u[i] ?? 0));
      const target = armed ? p.omegaIdle + p.omegaScale * ui : 0;
      const w = s.rotor[i]! + (target - s.rotor[i]!) * lag;
      s.rotor[i] = w;
      const f = p.kf * w * w;
      thrust += f;
      // Thrust acts along -z (up) at (x, y): moment r × F = (-y f, x f, 0).
      torque[0] += -r.y * f;
      torque[1] += r.x * f;
      // Drag torque: a counter-clockwise rotor (km > 0) pushes the body clockwise, +z in FRD.
      torque[2] += r.km * f;
    }

    // Forces in the body frame: thrust and linear drag on the air-relative velocity.
    const air: Vec3 = [s.vel[0] - this.wind[0], s.vel[1] - this.wind[1], s.vel[2] - this.wind[2]];
    const airBody = rotateInv(s.q, air);
    const fBody: Vec3 = [-p.drag[0] * airBody[0], -p.drag[1] * airBody[1], -thrust - p.drag[2] * airBody[2]];
    const fWorld = rotate(s.q, fBody);
    const acc: Vec3 = [fWorld[0] / p.massKg, fWorld[1] / p.massKg, fWorld[2] / p.massKg + GRAVITY];

    // Rigid-body rotation: I ω̇ = τ - ω × Iω - c ω.
    const [ix, iy, iz] = p.inertia;
    const [wx, wy, wz] = s.omega;
    const ax = (torque[0] - (wy * iz * wz - wz * iy * wy) - p.angDamping * wx) / ix;
    const ay = (torque[1] - (wz * ix * wx - wx * iz * wz) - p.angDamping * wy) / iy;
    const az = (torque[2] - (wx * iy * wy - wy * ix * wx) - p.angDamping * wz) / iz;

    // Semi-implicit Euler.
    for (let k = 0; k < 3; k++) s.vel[k]! += acc[k]! * dt;
    s.omega = [wx + ax * dt, wy + ay * dt, wz + az * dt];
    for (let k = 0; k < 3; k++) s.pos[k]! += s.vel[k]! * dt;
    this.integrateAttitude(dt);

    // Ground contact: the pad and the terrain hold the vehicle up.
    const ground = this.groundDown(s.pos[0], s.pos[1]);
    s.onGround = false;
    if (s.pos[2] >= ground) {
      s.pos[2] = ground;
      if (s.vel[2] > 0) s.vel[2] = 0;
      if (acc[2] >= 0) {
        // Resting: thrust below weight. Friction stops sliding and the legs keep it level.
        s.vel = [0, 0, 0];
        s.q = quatFromYaw(eulerOf(s.q).yaw);
        s.omega = [0, 0, 0];
        s.onGround = true;
        acc[0] = 0;
        acc[1] = 0;
        acc[2] = 0;
      }
    }

    // Accelerometer: specific force = acceleration minus gravity, in the body frame.
    this.specificForce = rotateInv(s.q, [acc[0], acc[1], acc[2] - GRAVITY]);
  }

  private integrateAttitude(dt: number): void {
    const s = this.s;
    const [w, x, y, z] = s.q;
    const [p, q, r] = s.omega;
    const nw = w + 0.5 * dt * (-x * p - y * q - z * r);
    const nx = x + 0.5 * dt * (w * p + y * r - z * q);
    const ny = y + 0.5 * dt * (w * q + z * p - x * r);
    const nz = z + 0.5 * dt * (w * r + x * q - y * p);
    const n = Math.hypot(nw, nx, ny, nz) || 1;
    s.q = [nw / n, nx / n, ny / n, nz / n];
  }
}

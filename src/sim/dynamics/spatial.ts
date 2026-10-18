/** Body-to-world quaternion helpers. xyzw, right-handed, Z-up. No allocations on the hot path. */

export interface Mut3 {
  0: number;
  1: number;
  2: number;
}

export interface Mut4 {
  0: number;
  1: number;
  2: number;
  3: number;
}

export interface Euler {
  roll: number;
  pitch: number;
  yaw: number;
}

export function quatNormalize(q: ArrayLike<number>, out: Mut4): void {
  const x = q[0] ?? 0;
  const y = q[1] ?? 0;
  const z = q[2] ?? 0;
  const w = q[3] ?? 0;
  let n = x * x + y * y + z * z + w * w;
  if (n < 1e-20) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 1;
    return;
  }
  n = 1 / Math.sqrt(n);
  out[0] = x * n;
  out[1] = y * n;
  out[2] = z * n;
  out[3] = w * n;
}

/** Hamilton product, xyzw. `out` may alias `a` or `b`. */
export function quatMultiply(a: ArrayLike<number>, b: ArrayLike<number>, out: Mut4): void {
  const ax = a[0] ?? 0;
  const ay = a[1] ?? 0;
  const az = a[2] ?? 0;
  const aw = a[3] ?? 0;
  const bx = b[0] ?? 0;
  const by = b[1] ?? 0;
  const bz = b[2] ?? 0;
  const bw = b[3] ?? 0;
  out[0] = aw * bx + ax * bw + ay * bz - az * by;
  out[1] = aw * by - ax * bz + ay * bw + az * bx;
  out[2] = aw * bz + ax * by - ay * bx + az * bw;
  out[3] = aw * bw - ax * bx - ay * by - az * bz;
}

/**
 * dq/dt = 0.5 * q ⊗ [wx, wy, wz, 0], then renormalize.
 * `omegaBody` is rad/s in the body frame. `out` may alias `q`.
 */
export function integrateQuat(
  q: ArrayLike<number>,
  omegaBody: ArrayLike<number>,
  dt: number,
  out: Mut4,
): void {
  const qx = q[0] ?? 0;
  const qy = q[1] ?? 0;
  const qz = q[2] ?? 0;
  const qw = q[3] ?? 0;
  const ox = omegaBody[0] ?? 0;
  const oy = omegaBody[1] ?? 0;
  const oz = omegaBody[2] ?? 0;
  const hx = 0.5 * (qw * ox + qy * oz - qz * oy);
  const hy = 0.5 * (qw * oy - qx * oz + qz * ox);
  const hz = 0.5 * (qw * oz + qx * oy - qy * ox);
  const hw = 0.5 * (-qx * ox - qy * oy - qz * oz);
  const x = qx + hx * dt;
  const y = qy + hy * dt;
  const z = qz + hz * dt;
  const w = qw + hw * dt;
  let n = x * x + y * y + z * z + w * w;
  if (n < 1e-20) {
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
    out[3] = 1;
    return;
  }
  n = 1 / Math.sqrt(n);
  out[0] = x * n;
  out[1] = y * n;
  out[2] = z * n;
  out[3] = w * n;
}

function rotateBy(x: number, y: number, z: number, w: number, v: ArrayLike<number>, out: Mut3): void {
  const vx = v[0] ?? 0;
  const vy = v[1] ?? 0;
  const vz = v[2] ?? 0;
  const tx = 2 * (y * vz - z * vy);
  const ty = 2 * (z * vx - x * vz);
  const tz = 2 * (x * vy - y * vx);
  out[0] = vx + w * tx + (y * tz - z * ty);
  out[1] = vy + w * ty + (z * tx - x * tz);
  out[2] = vz + w * tz + (x * ty - y * tx);
}

/** Rotate a body-frame vector into world. `out` may alias `v`. */
export function rotateBodyToWorld(q: ArrayLike<number>, v: ArrayLike<number>, out: Mut3): void {
  rotateBy(q[0] ?? 0, q[1] ?? 0, q[2] ?? 0, q[3] ?? 1, v, out);
}

/** Rotate a world-frame vector into body. `out` may alias `v`. */
export function rotateWorldToBody(q: ArrayLike<number>, v: ArrayLike<number>, out: Mut3): void {
  rotateBy(-(q[0] ?? 0), -(q[1] ?? 0), -(q[2] ?? 0), q[3] ?? 1, v, out);
}

/**
 * Intrinsic ZXY Euler (Three.js order): yaw about Z, then roll about X, then pitch about Y.
 * Positive pitch is nose-down (right-handed about +Y): body +Z tilts toward world +X.
 */
export function quatFromEulerZXY(roll: number, pitch: number, yaw: number, out: Mut4): void {
  const c1 = Math.cos(roll * 0.5);
  const c2 = Math.cos(pitch * 0.5);
  const c3 = Math.cos(yaw * 0.5);
  const s1 = Math.sin(roll * 0.5);
  const s2 = Math.sin(pitch * 0.5);
  const s3 = Math.sin(yaw * 0.5);
  out[0] = s1 * c2 * c3 - c1 * s2 * s3;
  out[1] = c1 * s2 * c3 + s1 * c2 * s3;
  out[2] = c1 * c2 * s3 + s1 * s2 * c3;
  out[3] = c1 * c2 * c3 - s1 * s2 * s3;
}

/** Write ZXY Euler angles (radians) matching `quatFromEulerZXY`. */
export function quatToEulerInto(q: ArrayLike<number>, out: Euler): void {
  const x = q[0] ?? 0;
  const y = q[1] ?? 0;
  const z = q[2] ?? 0;
  const w = q[3] ?? 0;
  const x2 = x + x;
  const y2 = y + y;
  const z2 = z + z;
  const xx = x * x2;
  const xy = x * y2;
  const xz = x * z2;
  const yy = y * y2;
  const yz = y * z2;
  const zz = z * z2;
  const wx = w * x2;
  const wy = w * y2;
  const wz = w * z2;
  const m12 = xy - wz;
  const m22 = 1 - (xx + zz);
  const m31 = xz - wy;
  const m32 = yz + wx;
  const m33 = 1 - (xx + yy);
  const m32c = m32 > 1 ? 1 : m32 < -1 ? -1 : m32;
  out.roll = nzero(Math.asin(m32c));
  if (Math.abs(m32c) < 0.9999999) {
    out.pitch = nzero(Math.atan2(-m31, m33));
    out.yaw = nzero(Math.atan2(-m12, m22));
  } else {
    out.pitch = 0;
    const m11 = 1 - (yy + zz);
    const m21 = xy + wz;
    out.yaw = nzero(Math.atan2(m21, m11));
  }
}

/** Map -0 to +0 so an identity quaternion yields exact zeros. */
function nzero(x: number): number {
  return x === 0 ? 0 : x;
}

export function quatToEuler(q: ArrayLike<number>): Euler {
  const out = { roll: 0, pitch: 0, yaw: 0 };
  quatToEulerInto(q, out);
  return out;
}

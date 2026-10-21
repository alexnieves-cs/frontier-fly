import type { CameraView } from '../perception/camera';

/** Surface look for one triangle. Colors are linear 0..1. */
export interface Paint {
  r: number;
  g: number;
  b: number;
  /** Procedural grain amplitude, 0 for smooth plastic and skin. */
  grain: number;
  /** Thermal emission for the IR sensor, 0 (ambient) to 1 (body heat). */
  heat: number;
  /** Specular gloss, 0..1. Plastic mannequins shine; fabric does not. */
  gloss: number;
}

/**
 * A z-buffered triangle rasterizer for the drone's low-resolution camera.
 * Geometry goes in world coordinates (sim frame: x, y horizontal, z up) and
 * is projected with the same pinhole model as src/perception/camera.ts. The
 * first pass keeps only the nearest triangle per pixel; shading is deferred
 * to render.ts, which reads `tri`, `invZ` and the per-triangle tables.
 */
export class Raster {
  readonly invZ: Float32Array;
  /** Triangle index per pixel, -1 for sky. */
  readonly tri: Int32Array;
  triCount = 0;
  /** Per triangle: paint (r, g, b, grain, heat, gloss), world normal, instance. */
  paint = new Float32Array(6 * 4096);
  normal = new Float32Array(3 * 4096);
  inst = new Int32Array(4096);

  private px = 0;
  private py = 0;
  private pz = 0;
  private rx = 0;
  private ry = 0;
  private rz = 0;
  private ux = 0;
  private uy = 0;
  private uz = 0;
  private fx = 0;
  private fy = 0;
  private fz = 0;
  private sx = 1;
  private sy = 1;
  private near = 0.05;
  private far = 80;
  private readonly cam = new Float64Array(12 * 3);

  constructor(
    readonly width: number,
    readonly height: number,
  ) {
    this.invZ = new Float32Array(width * height);
    this.tri = new Int32Array(width * height);
  }

  begin(view: CameraView): void {
    [this.px, this.py, this.pz] = view.position;
    [this.rx, this.ry, this.rz] = view.right;
    [this.ux, this.uy, this.uz] = view.up;
    [this.fx, this.fy, this.fz] = view.forward;
    const tanY = Math.tan(view.fovY / 2);
    const tanX = tanY * Math.max(0.05, view.aspect);
    this.sx = this.width / (2 * tanX);
    this.sy = this.height / (2 * tanY);
    this.near = view.near;
    this.far = view.far;
    this.invZ.fill(0);
    this.tri.fill(-1);
    this.triCount = 0;
  }

  /** Cheap sphere test against the view frustum, world frame. */
  visible(cx: number, cy: number, cz: number, radius: number): boolean {
    const dx = cx - this.px;
    const dy = cy - this.py;
    const dz = cz - this.pz;
    const z = dx * this.fx + dy * this.fy + dz * this.fz;
    if (z < -radius || z - radius > this.far) return false;
    const x = dx * this.rx + dy * this.ry + dz * this.rz;
    const y = dx * this.ux + dy * this.uy + dz * this.uz;
    const zz = Math.max(z, 0) + radius * 2;
    const halfW = (this.width / 2 / this.sx) * zz + radius * 1.5;
    const halfH = (this.height / 2 / this.sy) * zz + radius * 1.5;
    return Math.abs(x) <= halfW && Math.abs(y) <= halfH;
  }

  private record(p: Paint, nx: number, ny: number, nz: number, inst: number): number {
    const id = this.triCount++;
    if (id >= this.inst.length) {
      const paint = new Float32Array(this.paint.length * 2);
      paint.set(this.paint);
      this.paint = paint;
      const normal = new Float32Array(this.normal.length * 2);
      normal.set(this.normal);
      this.normal = normal;
      const inst = new Int32Array(this.inst.length * 2);
      inst.set(this.inst);
      this.inst = inst;
    }
    const o = id * 6;
    this.paint[o] = p.r;
    this.paint[o + 1] = p.g;
    this.paint[o + 2] = p.b;
    this.paint[o + 3] = p.grain;
    this.paint[o + 4] = p.heat;
    this.paint[o + 5] = p.gloss;
    this.normal[id * 3] = nx;
    this.normal[id * 3 + 1] = ny;
    this.normal[id * 3 + 2] = nz;
    this.inst[id] = inst;
    return id;
  }

  /** One world-frame triangle. */
  triangle(
    ax: number, ay: number, az: number,
    bx: number, by: number, bz: number,
    cx: number, cy: number, cz: number,
    paint: Paint,
    inst: number,
  ): void {
    const c = this.cam;
    this.toCam(ax, ay, az, 0);
    this.toCam(bx, by, bz, 3);
    this.toCam(cx, cy, cz, 6);
    const near = this.near;
    const za = c[2]!;
    const zb = c[5]!;
    const zc = c[8]!;
    if (za < near && zb < near && zc < near) return;
    if (za > this.far && zb > this.far && zc > this.far) return;
    // Trivial reject against the side planes.
    const kx = this.width / 2 / this.sx;
    const ky = this.height / 2 / this.sy;
    if (c[0]! > za * kx && c[3]! > zb * kx && c[6]! > zc * kx) return;
    if (-c[0]! > za * kx && -c[3]! > zb * kx && -c[6]! > zc * kx) return;
    if (c[1]! > za * ky && c[4]! > zb * ky && c[7]! > zc * ky) return;
    if (-c[1]! > za * ky && -c[4]! > zb * ky && -c[7]! > zc * ky) return;
    // World normal for shading.
    const e1x = bx - ax;
    const e1y = by - ay;
    const e1z = bz - az;
    const e2x = cx - ax;
    const e2y = cy - ay;
    const e2z = cz - az;
    let nx = e1y * e2z - e1z * e2y;
    let ny = e1z * e2x - e1x * e2z;
    let nz = e1x * e2y - e1y * e2x;
    const nl = Math.sqrt(nx * nx + ny * ny + nz * nz);
    if (nl < 1e-12) return;
    nx /= nl;
    ny /= nl;
    nz /= nl;
    let id = -1;
    const getId = (): number => (id >= 0 ? id : (id = this.record(paint, nx, ny, nz, inst)));
    if (za >= near && zb >= near && zc >= near) {
      this.fill(0, 3, 6, getId);
      return;
    }
    // Clip against the near plane: up to four vertices, written after the first three.
    let n = 0;
    const order = [0, 3, 6];
    for (let i = 0; i < 3; i++) {
      const p = order[i]!;
      const q = order[(i + 1) % 3]!;
      const zp = c[p + 2]!;
      const zq = c[q + 2]!;
      if (zp >= near) {
        const o = 9 + n * 3;
        c[o] = c[p]!;
        c[o + 1] = c[p + 1]!;
        c[o + 2] = zp;
        n++;
      }
      if ((zp >= near) !== (zq >= near)) {
        const t = (near - zp) / (zq - zp);
        const o = 9 + n * 3;
        c[o] = c[p]! + (c[q]! - c[p]!) * t;
        c[o + 1] = c[p + 1]! + (c[q + 1]! - c[p + 1]!) * t;
        c[o + 2] = near;
        n++;
      }
    }
    for (let i = 1; i + 1 < n; i++) this.fill(9, 9 + i * 3, 9 + (i + 1) * 3, getId);
  }

  private toCam(x: number, y: number, z: number, o: number): void {
    const dx = x - this.px;
    const dy = y - this.py;
    const dz = z - this.pz;
    this.cam[o] = dx * this.rx + dy * this.ry + dz * this.rz;
    this.cam[o + 1] = dx * this.ux + dy * this.uy + dz * this.uz;
    this.cam[o + 2] = dx * this.fx + dy * this.fy + dz * this.fz;
  }

  private fill(a: number, b: number, c: number, getId: () => number): void {
    const cam = this.cam;
    const W = this.width;
    const H = this.height;
    const wa = 1 / cam[a + 2]!;
    const wb = 1 / cam[b + 2]!;
    const wc = 1 / cam[c + 2]!;
    const x0 = W / 2 + cam[a]! * wa * this.sx;
    const y0 = H / 2 - cam[a + 1]! * wa * this.sy;
    const x1 = W / 2 + cam[b]! * wb * this.sx;
    const y1 = H / 2 - cam[b + 1]! * wb * this.sy;
    const x2 = W / 2 + cam[c]! * wc * this.sx;
    const y2 = H / 2 - cam[c + 1]! * wc * this.sy;
    const area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
    if (Math.abs(area) < 1e-9) return;
    const minX = Math.max(0, Math.floor(Math.min(x0, x1, x2)));
    const maxX = Math.min(W - 1, Math.ceil(Math.max(x0, x1, x2)));
    const minY = Math.max(0, Math.floor(Math.min(y0, y1, y2)));
    const maxY = Math.min(H - 1, Math.ceil(Math.max(y0, y1, y2)));
    if (minX > maxX || minY > maxY) return;
    const inv = 1 / area;
    const invZ = this.invZ;
    const triBuf = this.tri;
    // Edge functions, stepped per pixel center.
    const A0 = (y1 - y2) * inv;
    const A1 = (y2 - y0) * inv;
    const A2 = -A0 - A1;
    for (let py = minY; py <= maxY; py++) {
      const cy = py + 0.5;
      // Barycentrics are linear along the row: l_k(px) = c_k + A_k * px at pixel centers.
      const c0 = (x1 * (y2 - cy) - x2 * (y1 - cy)) * inv + A0 * 0.5;
      const c1 = (x2 * (y0 - cy) - x0 * (y2 - cy)) * inv + A1 * 0.5;
      const c2 = 1 - c0 - c1;
      let lo = minX;
      let hi = maxX;
      let empty = false;
      for (let k = 0; k < 3; k++) {
        const c = k === 0 ? c0 : k === 1 ? c1 : c2;
        const A = k === 0 ? A0 : k === 1 ? A1 : A2;
        if (A > 1e-12) lo = Math.max(lo, Math.ceil(-c / A - 1e-7));
        else if (A < -1e-12) hi = Math.min(hi, Math.floor(-c / A + 1e-7));
        else if (c < 0) empty = true;
      }
      if (empty || lo > hi) continue;
      const row = py * W;
      let l0 = c0 + A0 * lo;
      let l1 = c1 + A1 * lo;
      for (let px = lo; px <= hi; px++, l0 += A0, l1 += A1) {
        const l2 = 1 - l0 - l1;
        if (l0 < -1e-6 || l1 < -1e-6 || l2 < -1e-6) continue;
        const w = l0 * wa + l1 * wb + l2 * wc;
        const i = row + px;
        // First drawn wins ties, so coplanar faces do not stripe.
        if (w <= invZ[i]! * (1 + 2e-5)) continue;
        invZ[i] = w;
        triBuf[i] = getId();
      }
    }
  }

  /** Axis-aligned box. */
  box(x0: number, y0: number, z0: number, x1: number, y1: number, z1: number, paint: Paint, inst: number): void {
    if (!this.visible((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2, Math.hypot(x1 - x0, y1 - y0, z1 - z0) / 2)) return;
    this.quad(x0, y0, z0, x1, y0, z0, x1, y0, z1, x0, y0, z1, paint, inst);
    this.quad(x0, y1, z0, x1, y1, z0, x1, y1, z1, x0, y1, z1, paint, inst);
    this.quad(x0, y0, z0, x0, y1, z0, x0, y1, z1, x0, y0, z1, paint, inst);
    this.quad(x1, y0, z0, x1, y1, z0, x1, y1, z1, x1, y0, z1, paint, inst);
    this.quad(x0, y0, z0, x1, y0, z0, x1, y1, z0, x0, y1, z0, paint, inst);
    this.quad(x0, y0, z1, x1, y0, z1, x1, y1, z1, x0, y1, z1, paint, inst);
  }

  /** Box turned by `yaw` about +z around its center. */
  orientedBox(cx: number, cy: number, cz: number, hx: number, hy: number, hz: number, yaw: number, paint: Paint, inst: number): void {
    if (!this.visible(cx, cy, cz, Math.hypot(hx, hy, hz))) return;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    const corner = (sx: number, sy: number, sz: number): [number, number, number] => [
      cx + c * sx * hx - s * sy * hy,
      cy + s * sx * hx + c * sy * hy,
      cz + sz * hz,
    ];
    const p = [
      corner(-1, -1, -1), corner(1, -1, -1), corner(1, 1, -1), corner(-1, 1, -1),
      corner(-1, -1, 1), corner(1, -1, 1), corner(1, 1, 1), corner(-1, 1, 1),
    ];
    const face = (a: number, b: number, d: number, e: number): void => {
      const A = p[a]!;
      const B = p[b]!;
      const D = p[d]!;
      const E = p[e]!;
      this.quad(A[0], A[1], A[2], B[0], B[1], B[2], D[0], D[1], D[2], E[0], E[1], E[2], paint, inst);
    };
    face(0, 1, 2, 3);
    face(4, 5, 6, 7);
    face(0, 1, 5, 4);
    face(1, 2, 6, 5);
    face(2, 3, 7, 6);
    face(3, 0, 4, 7);
  }

  quad(
    ax: number, ay: number, az: number,
    bx: number, by: number, bz: number,
    cx: number, cy: number, cz: number,
    dx: number, dy: number, dz: number,
    paint: Paint,
    inst: number,
  ): void {
    this.triangle(ax, ay, az, bx, by, bz, cx, cy, cz, paint, inst);
    this.triangle(ax, ay, az, cx, cy, cz, dx, dy, dz, paint, inst);
  }
}

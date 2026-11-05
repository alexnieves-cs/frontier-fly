import * as THREE from 'three';
import { mergeGeometries } from 'three/addons/utils/BufferGeometryUtils.js';
import { CELL, SURFACE } from '../../shared/constants';
import { RNG } from '../../shared/rng';
import { jambSolids, roofSolids } from '../../world/outdoor/colliders';
import type { BuildingShell, Opening, Solid } from '../../world/outdoor/types';

/**
 * Exterior and interior trim for one building shell, all visual: none of it is a collider,
 * and none of it changes what the sim's lidar or camera sees.
 *
 * - Every wall opening keeps its sill, lintel and jamb blocks (the colliders), drawn as siding
 *   outside and plaster inside, with casing, sills and the leaf, sash or garage door its kind has.
 * - Exterior glass cells get framed windows. The mission camera treats exterior glass as opaque
 *   (perception viewSpecOf), so the panes are dark, reflective and draw nothing behind them.
 * - A concrete foundation band, corner boards and, on flat roofs, coping and the hatch curb.
 * - Inside: skirting boards, casings and a lintel on every interior doorway.
 *
 * Coordinates are shell-local Three axes: x = sim x, y = up, z = sim y, origin at the shell's
 * min corner on the pad.
 */
export interface FacadeMaterials {
  siding: THREE.Material;
  wall: THREE.Material;
  cap: THREE.Material;
  trim: THREE.MeshStandardMaterial;
  door: THREE.Material;
  night: boolean;
  seed: number;
  /** Software GL has no antialiasing: keep bands nearly flush so thin ledges don't alias into dashes. */
  flush: boolean;
}

export interface Facade {
  group: THREE.Group;
  /** Flat roof pieces (hatch shells), hidden with the roof in the cutaway. */
  roof: THREE.Object3D[];
  /** Glass cells that got a window, as `${floor}:${x}:${z}`. */
  windows: Set<string>;
}

type V3 = [number, number, number];

/** Opening frame: origin on the outer wall plane at the hole centre; u along the wall, w outward. */
interface Frame {
  o: V3;
  u: V3;
  w: V3;
}

const SHUTTERS = ['#2f4a3a', '#3a3f4a', '#5a2f2a', '#2e3d52', '#4a4234'];

/** Collects boxes and shapes per material and merges them into one mesh each. */
class Batch {
  private readonly parts = new Map<THREE.Material, THREE.BufferGeometry[]>();

  /** Axis-aligned box from its min and max corner (Three axes). */
  box(mat: THREE.Material, min: V3, max: V3): void {
    const sx = max[0] - min[0];
    const sy = max[1] - min[1];
    const sz = max[2] - min[2];
    if (sx < 1e-3 || sy < 1e-3 || sz < 1e-3) return;
    const g = new THREE.BoxGeometry(sx, sy, sz);
    g.translate((min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2);
    this.add(mat, g);
  }

  add(mat: THREE.Material, geo: THREE.BufferGeometry): void {
    const g = geo.index ? geo.toNonIndexed() : geo;
    if (!g.getAttribute('uv')) g.setAttribute('uv', new THREE.Float32BufferAttribute(new Float32Array(g.getAttribute('position').count * 2), 2));
    let list = this.parts.get(mat);
    if (!list) this.parts.set(mat, (list = []));
    list.push(g);
  }

  flush(into: THREE.Group, cast = true): THREE.Mesh[] {
    const out: THREE.Mesh[] = [];
    for (const [mat, list] of this.parts) {
      const merged = mergeGeometries(list);
      if (!merged) continue;
      const mesh = new THREE.Mesh(merged, mat);
      mesh.castShadow = cast;
      mesh.receiveShadow = true;
      into.add(mesh);
      out.push(mesh);
    }
    this.parts.clear();
    return out;
  }
}

export function buildFacade(shell: BuildingShell, m: FacadeMaterials): Facade {
  const group = new THREE.Group();
  const spec = shell.spec;
  const cs = spec.cellSize;
  const H = spec.floorHeight;
  const floor0 = spec.floors[0]!;
  const W = floor0.w * cs;
  const D = floor0.d * cs;
  const top = spec.floors.length * H;
  const origin = shell.origin;
  const rng = new RNG(m.seed + shell.id * 31, 'facade');
  const trim = m.trim;
  const shutter = new THREE.MeshStandardMaterial({ color: SHUTTERS[rng.int(SHUTTERS.length)], roughness: 0.7 });
  const concrete = new THREE.MeshStandardMaterial({ color: m.night ? '#4d4f52' : '#8d8b86', roughness: 0.95 });
  const metal = new THREE.MeshStandardMaterial({ color: '#7c838b', roughness: 0.45, metalness: 0.7 });
  const garage = new THREE.MeshStandardMaterial({ color: '#d8d4cb', roughness: 0.5, metalness: 0.25 });
  const dark = new THREE.MeshStandardMaterial({ color: '#141a20', roughness: 0.9 });
  // Exterior panes: dark and mirror-like, so they read as glass in daylight without showing a room.
  const pane = new THREE.MeshStandardMaterial({ color: m.night ? '#0b1118' : '#1b252d', roughness: 0.12, metalness: 0.45, envMapIntensity: 0.9 });
  // The inner face of the same window: bright with daylight, dim blue at night.
  const daylight = new THREE.MeshStandardMaterial({
    color: m.night ? '#1a2433' : '#dce9f2',
    emissive: m.night ? '#0d1626' : '#b9d3e6',
    emissiveIntensity: m.night ? 0.4 : 0.75,
    roughness: 0.3,
  });
  const shard = new THREE.MeshStandardMaterial({ color: '#9fb6c4', roughness: 0.05, metalness: 0.6, transparent: true, opacity: 0.55, side: THREE.DoubleSide });
  const b = new Batch();
  // Sim (x, y, z) relative to the shell origin -> Three local.
  const T = (p: V3): V3 => [p[0] - origin[0], p[2] - origin[2], p[1] - origin[1]];
  const boxIn = (mat: THREE.Material, f: Frame, u0: number, u1: number, v0: number, v1: number, w0: number, w1: number) => {
    // Corners in sim axes (frames are axis aligned, so the min/max corners are a box).
    const pts: V3[] = [];
    for (const u of [u0, u1]) for (const v of [v0, v1]) for (const w of [w0, w1]) pts.push([f.o[0] + f.u[0] * u + f.w[0] * w, f.o[1] + f.u[1] * u + f.w[1] * w, f.o[2] + v]);
    const t = pts.map(T);
    const min: V3 = [Math.min(...t.map((p) => p[0])), Math.min(...t.map((p) => p[1])), Math.min(...t.map((p) => p[2]))];
    const max: V3 = [Math.max(...t.map((p) => p[0])), Math.max(...t.map((p) => p[1])), Math.max(...t.map((p) => p[2]))];
    b.box(mat, min, max);
  };
  const shapeIn = (mat: THREE.Material, f: Frame, shape: THREE.Shape, w: number) => {
    const g = new THREE.ShapeGeometry(shape);
    const o = T([f.o[0] + f.w[0] * w, f.o[1] + f.w[1] * w, f.o[2]]);
    const basis = new THREE.Matrix4().makeBasis(new THREE.Vector3(f.u[0], 0, f.u[1]), new THREE.Vector3(0, 1, 0), new THREE.Vector3(f.w[0], 0, f.w[1]));
    basis.setPosition(o[0], o[1], o[2]);
    g.applyMatrix4(basis);
    b.add(mat, g);
  };

  /** A framed window on a wall plane: casing, two-light sash, sill and optional shutters. */
  const drawWindow = (f: Frame, width: number, sill: number, height: number, glass: THREE.Material, out: number, shutters: boolean) => {
    const c = 0.06;
    const s = out;
    // Glass just proud of the wall, frame and casing proud of the glass.
    boxIn(glass, f, -width / 2, width / 2, sill, sill + height, 0, 0.012 * s);
    for (const [u0, u1, v0, v1] of [
      [-width / 2 - c, -width / 2, sill, sill + height],
      [width / 2, width / 2 + c, sill, sill + height],
      [-width / 2 - c, width / 2 + c, sill + height, sill + height + c * 1.4],
    ] as const) {
      boxIn(trim, f, u0, u1, v0, v1, 0, 0.05 * s);
    }
    // Sill: deeper and wider than the casing.
    boxIn(trim, f, -width / 2 - c - 0.03, width / 2 + c + 0.03, sill - 0.05, sill, 0, 0.09 * s);
    // Muntins: one vertical, one horizontal.
    boxIn(trim, f, -0.016, 0.016, sill, sill + height, 0, 0.03 * s);
    boxIn(trim, f, -width / 2, width / 2, sill + height * 0.52 - 0.016, sill + height * 0.52 + 0.016, 0, 0.03 * s);
    if (shutters) {
      for (const side of [-1, 1]) {
        const u0 = side < 0 ? -width / 2 - c - 0.34 : width / 2 + c + 0.02;
        boxIn(shutter, f, u0, u0 + 0.32, sill - 0.02, sill + height + 0.04, 0, 0.035 * s);
        // Louvre shadow lines.
        for (let k = 1; k < 9; k++) {
          const v = sill + ((height + 0.04) * k) / 9;
          boxIn(dark, f, u0 + 0.04, u0 + 0.28, v - 0.006, v, 0.035 * s, 0.04 * s);
        }
      }
    }
  };

  // ------------------------------------------------------------ windows on exterior glass
  const windows = new Set<string>();
  const nearOpening = (f: number, x: number, z: number) =>
    shell.openings.some((o) => o.floor === f && o.apertureCells.some((c) => Math.abs(c.x - x) + Math.abs(c.z - z) <= 1));
  spec.floors.forEach((floor, fi) => {
    for (let z = 0; z < floor.d; z++) {
      for (let x = 0; x < floor.w; x++) {
        const i = z * floor.w + x;
        if (floor.cells[i] !== CELL.WALL || floor.surfaces?.[i] !== SURFACE.GLASS) continue;
        const edgeX = x === 0 || x === floor.w - 1;
        const edgeZ = z === 0 || z === floor.d - 1;
        if (edgeX === edgeZ) continue; // interior glass or a corner
        // Every other exterior glass cell (they alternate with siding), and none beside an opening.
        if ((x + z) % 4 !== 0 || nearOpening(fi, x, z)) continue;
        windows.add(`${fi}:${x}:${z}`);
        const n: V3 = edgeX ? [x === 0 ? -1 : 1, 0, 0] : [0, z === 0 ? -1 : 1, 0];
        const u: V3 = [Math.abs(n[1]), Math.abs(n[0]), 0];
        const cx = origin[0] + (x + 0.5) * cs;
        const cy = origin[1] + (z + 0.5) * cs;
        const base = origin[2] + fi * H;
        const outer: Frame = { o: [cx + n[0] * cs * 0.5, cy + n[1] * cs * 0.5, base], u, w: n };
        drawWindow(outer, 0.72, 0.95, 1.2, pane, 1, true);
        const inner: Frame = { o: [cx - n[0] * cs * 0.5, cy - n[1] * cs * 0.5, base], u, w: [-n[0], -n[1], 0] };
        drawWindow(inner, 0.72, 0.95, 1.2, daylight, 1, false);
      }
    }
  });

  // ------------------------------------------------------------ openings
  const jambs: THREE.Mesh[] = [];
  for (const opening of shell.openings) {
    if (opening.kind === 'roof_hatch') continue;
    for (const solid of jambSolids(shell, opening)) jambs.push(jambMesh(solid, opening, origin, m, top, H));
    dressOpening(opening);
  }

  function dressOpening(o: Opening): void {
    const n: V3 = [Math.round(o.normal[0]), Math.round(o.normal[1]), 0];
    const u: V3 = [Math.abs(n[1]), Math.abs(n[0]), 0];
    // Outer wall plane: the aperture cells' face on the normal's side.
    const cells = o.apertureCells;
    const along = (c: { x: number; z: number }) => (n[0] !== 0 ? c.x : c.z);
    const k = n[0] + n[1] > 0 ? Math.max(...cells.map(along)) + 1 : Math.min(...cells.map(along));
    const plane = k * cs + (n[0] !== 0 ? origin[0] : origin[1]);
    const base = origin[2] + o.floor * H;
    const oc: V3 = n[0] !== 0 ? [plane, o.center[1], base] : [o.center[0], plane, base];
    const out: Frame = { o: oc, u, w: n };
    const thick = cs;
    const inn: Frame = { o: [oc[0] - n[0] * thick, oc[1] - n[1] * thick, base], u, w: [-n[0], -n[1], 0] };
    const w = o.width;
    const h = o.height;
    const v0 = o.center[2] - base - h / 2;
    const v1 = v0 + h;
    const casing = (fr: Frame, bottom: boolean, depth = 0.04, c = 0.08) => {
      boxIn(trim, fr, -w / 2 - c, -w / 2, bottom ? v0 - c : v0, v1, 0, depth);
      boxIn(trim, fr, w / 2, w / 2 + c, bottom ? v0 - c : v0, v1, 0, depth);
      boxIn(trim, fr, -w / 2 - c, w / 2 + c, v1, v1 + c, 0, depth);
      if (bottom) boxIn(trim, fr, -w / 2 - c - 0.03, w / 2 + c + 0.03, v0 - 0.05, v0, 0, 0.09);
    };
    switch (o.kind) {
      case 'open_window': {
        casing(out, true);
        casing(inn, true, 0.02, 0.06);
        if (o.decoy) {
          // A small sash window propped open a crack: glass fills the casing above the gap.
          boxIn(pane, out, -w / 2, w / 2, v1 - 0.02, v1 + 0.0, 0, 0.012);
        } else {
          // Double-hung window with the lower sash pushed all the way up behind the upper one.
          const sash = Math.min(h, top - (base - origin[2]) - v1 - 0.12);
          if (sash > 0.2) {
            boxIn(pane, out, -w / 2, w / 2, v1, v1 + sash, 0, 0.012);
            boxIn(trim, out, -w / 2, w / 2, v1 - 0.04, v1 + 0.02, 0, 0.05);
            boxIn(trim, out, -w / 2, w / 2, v1 + sash * 0.5 - 0.02, v1 + sash * 0.5 + 0.02, 0, 0.035);
            boxIn(trim, out, -w / 2 - 0.08, -w / 2, v1, v1 + sash, 0, 0.04);
            boxIn(trim, out, w / 2, w / 2 + 0.08, v1, v1 + sash, 0, 0.04);
            boxIn(trim, out, -w / 2 - 0.08, w / 2 + 0.08, v1 + sash, v1 + sash + 0.1, 0, 0.04);
          }
        }
        break;
      }
      case 'broken_window': {
        // A pane in a frame a little larger than the hole; the hole is where the glass broke out.
        const mw = w / 2 + 0.12;
        const top2 = v1 + 0.14;
        const bot = v0 - 0.08;
        const s = new THREE.Shape();
        s.moveTo(-mw, bot);
        s.lineTo(mw, bot);
        s.lineTo(mw, top2);
        s.lineTo(-mw, top2);
        s.lineTo(-mw, bot);
        const hole = new THREE.Path();
        const jag = (a: number, bnd: number) => a + (rng.next() - 0.5) * bnd;
        const pts: [number, number][] = [];
        const steps = 5;
        for (let i = 0; i < steps; i++) pts.push([jag(-w / 2 + (w * i) / steps, 0.08), jag(v0, 0.06)]);
        for (let i = 0; i < steps; i++) pts.push([jag(w / 2, 0.06), jag(v0 + (h * i) / steps, 0.08)]);
        for (let i = 0; i < steps; i++) pts.push([jag(w / 2 - (w * i) / steps, 0.08), jag(v1, 0.06)]);
        for (let i = 0; i < steps; i++) pts.push([jag(-w / 2, 0.06), jag(v1 - (h * i) / steps, 0.08)]);
        // Keep the jagged edge outside the clear hole, so no shard sits in the drone's path.
        for (const p of pts) {
          p[0] = Math.sign(p[0]) * Math.min(mw - 0.02, Math.max(Math.abs(p[0]), w / 2 + 0.005));
          if (p[1] < v0 + h / 2) p[1] = Math.max(bot + 0.02, Math.min(p[1], v0 - 0.005));
          else p[1] = Math.min(top2 - 0.02, Math.max(p[1], v1 + 0.005));
        }
        hole.moveTo(pts[0]![0], pts[0]![1]);
        for (const p of pts.slice(1)) hole.lineTo(p[0], p[1]);
        hole.lineTo(pts[0]![0], pts[0]![1]);
        s.holes.push(hole);
        shapeIn(shard, out, s, 0.012);
        const c = 0.07;
        boxIn(trim, out, -mw - c, -mw, bot, top2, 0, 0.05);
        boxIn(trim, out, mw, mw + c, bot, top2, 0, 0.05);
        boxIn(trim, out, -mw - c, mw + c, top2, top2 + c, 0, 0.05);
        boxIn(trim, out, -mw - c - 0.03, mw + c + 0.03, bot - 0.05, bot, 0, 0.09);
        casing(inn, true, 0.02, 0.06);
        // Glass on the ground below the window.
        for (let i = 0; i < 7; i++) {
          const g = new THREE.Shape();
          const r = 0.03 + rng.next() * 0.05;
          g.moveTo(0, 0);
          g.lineTo(r, rng.next() * r);
          g.lineTo(rng.next() * r, r);
          const f2: Frame = { o: [oc[0] + n[0] * (0.25 + rng.next() * 0.5) + u[0] * (rng.next() - 0.5) * w, oc[1] + n[1] * (0.25 + rng.next() * 0.5) + u[1] * (rng.next() - 0.5) * w, base], u, w: n };
          const geo = new THREE.ShapeGeometry(g);
          geo.rotateX(-Math.PI / 2);
          const p = T([f2.o[0], f2.o[1], base + 0.015]);
          geo.translate(p[0], p[1], p[2]);
          b.add(shard, geo);
        }
        break;
      }
      case 'garage_gap': {
        // A sectional garage door, stuck partway open: the gap under it is the way in.
        const doorTop = Math.min(2.3, top - (base - origin[2]) - 0.25);
        const c = 0.1;
        boxIn(trim, out, -w / 2 - c, -w / 2, 0, doorTop, 0, 0.05);
        boxIn(trim, out, w / 2, w / 2 + c, 0, doorTop, 0, 0.05);
        boxIn(trim, out, -w / 2 - c, w / 2 + c, doorTop, doorTop + c, 0, 0.05);
        const panels = Math.max(1, Math.round((doorTop - v1) / 0.5));
        const ph = (doorTop - v1) / panels;
        for (let i = 0; i < panels; i++) {
          const a = v1 + i * ph;
          boxIn(garage, out, -w / 2, w / 2, a + 0.01, a + ph - 0.01, 0, 0.03);
          boxIn(dark, out, -w / 2, w / 2, a, a + 0.012, 0, 0.032);
          // Two raised fields per panel.
          for (const side of [-1, 1]) boxIn(garage, out, side < 0 ? -w / 2 + 0.06 : 0.04, side < 0 ? -0.04 : w / 2 - 0.06, a + 0.08, a + ph - 0.08, 0, 0.045);
        }
        // Bottom rubber seal, a handle, and a concrete apron in front.
        boxIn(dark, out, -w / 2, w / 2, v1, v1 + 0.025, 0, 0.04);
        boxIn(metal, out, -0.08, 0.08, v1 + 0.1, v1 + 0.13, 0.03, 0.07);
        boxIn(concrete, out, -w / 2 - 0.3, w / 2 + 0.3, -0.12, Math.max(0.02, v0 - 0.01), -0.02, 1.4);
        break;
      }
      case 'ajar_door':
      case 'closed_door': {
        casing(out, false, 0.05, 0.1);
        casing(inn, false, 0.02, 0.07);
        // Stoop: a concrete step under the door.
        boxIn(concrete, out, -w / 2 - 0.35, w / 2 + 0.35, -0.3, Math.max(0.02, v0 - 0.01), -0.02, 0.9);
        // A porch light beside the frame.
        boxIn(metal, out, w / 2 + 0.2, w / 2 + 0.32, 1.85, 2.05, 0, 0.1);
        boxIn(daylight, out, w / 2 + 0.215, w / 2 + 0.305, 1.88, 2.0, 0.1, 0.11);
        break;
      }
      case 'vent': {
        // A louvred vent cover; the slats sit outside the clear hole's edges only.
        const c = 0.04;
        boxIn(metal, out, -w / 2 - c, -w / 2, v0 - c, v1 + c, 0, 0.03);
        boxIn(metal, out, w / 2, w / 2 + c, v0 - c, v1 + c, 0, 0.03);
        boxIn(metal, out, -w / 2, w / 2, v1, v1 + c, 0, 0.03);
        boxIn(metal, out, -w / 2, w / 2, v0 - c, v0, 0, 0.03);
        boxIn(dark, out, -w / 2, w / 2, v0, v1, -0.05, -0.04);
        break;
      }
    }
  }

  // ------------------------------------------------------------ foundation and corners
  const ox = m.flush ? 0.004 : 0.04;
  for (const [x0, x1, z0, z1] of [
    [-ox, W + ox, -ox, 0],
    [-ox, W + ox, D, D + ox],
    [-ox, 0, 0, D],
    [W, W + ox, 0, D],
  ] as const) {
    b.box(concrete, [x0, -0.5, z0], [x1, 0.32, z1]);
  }
  for (const [x, z] of [
    [0, 0],
    [W, 0],
    [0, D],
    [W, D],
  ] as const) {
    const sx = x === 0 ? -1 : 1;
    const sz = z === 0 ? -1 : 1;
    b.box(trim, [Math.min(x, x + sx * 0.07), 0.32, Math.min(z, z + sz * 0.13)], [Math.max(x, x + sx * 0.07), top, Math.max(z, z + sz * 0.13)]);
    b.box(trim, [Math.min(x, x + sx * 0.13), 0.32, Math.min(z, z + sz * 0.07)], [Math.max(x, x + sx * 0.13), top, Math.max(z, z + sz * 0.07)]);
  }
  // A frieze board under the eaves, and a band between floors.
  for (let f = 1; f <= spec.floors.length; f++) {
    const y = f * H;
    const t = f === spec.floors.length ? 0.22 : 0.12;
    const p = m.flush ? 0.004 : 0.03;
    for (const [x0, x1, z0, z1] of [
      [-p, W + p, -p, 0],
      [-p, W + p, D, D + p],
      [-p, 0, 0, D],
      [W, W + p, 0, D],
    ] as const) {
      b.box(trim, [x0, y - t, z0], [x1, y, z1]);
    }
  }

  // ------------------------------------------------------------ interior trim
  // Baked contact shadow where floor meets wall: a dark strip fading out over 0.45 m.
  const ao = new THREE.MeshBasicMaterial({ color: '#000000', alphaMap: aoTexture(), transparent: true, opacity: m.night ? 0.4 : 0.6, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -2 });
  function aoStrip(ax: number, az: number, bx: number, bz: number, nx: number, nz: number, y: number): void {
    const r = 0.45;
    const pos = [ax, y, az, bx, y, bz, bx + nx * r, y, bz + nz * r, ax, y, az, bx + nx * r, y, bz + nz * r, ax + nx * r, y, az + nz * r];
    const uv = [0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1];
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('normal', new THREE.Float32BufferAttribute([0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0], 3));
    g.setAttribute('uv', new THREE.Float32BufferAttribute(uv, 2));
    b.add(ao, g);
  }
  spec.floors.forEach((floor, fi) => {
    const base = fi * H + (fi === 0 ? 0.1 : 0.05);
    const at = (x: number, z: number) => (x < 0 || z < 0 || x >= floor.w || z >= floor.d ? CELL.WALL : floor.cells[z * floor.w + x]);
    for (let z = 0; z < floor.d; z++) {
      for (let x = 0; x < floor.w; x++) {
        const cell = floor.cells[z * floor.w + x];
        if (cell === CELL.WALL || cell === CELL.SHAFT) continue;
        if (cell === CELL.DOOR) {
          // Casing on both faces and a lintel over the door leaf (visual; the drone flies far below it).
          const vertical = at(x - 1, z) === CELL.WALL;
          const leafTop = fi * H + H * 0.8;
          b.box(m.wall, [x * cs, leafTop, z * cs], [(x + 1) * cs, (fi + 1) * H - 0.02, (z + 1) * cs]);
          for (const side of [0, 1]) {
            const p = (vertical ? z : x) + side;
            const d0 = side ? p * cs : p * cs - 0.025;
            const d1 = side ? p * cs + 0.025 : p * cs;
            for (const e of [0, 1]) {
              const q = (vertical ? x : z) + e;
              const a0 = e ? q * cs - 0.07 : q * cs;
              const a1 = e ? q * cs : q * cs + 0.07;
              if (vertical) b.box(trim, [a0, fi * H, d0], [a1, leafTop + 0.07, d1]);
              else b.box(trim, [d0, fi * H, a0], [d1, leafTop + 0.07, a1]);
            }
            if (vertical) b.box(trim, [x * cs, leafTop, d0], [(x + 1) * cs, leafTop + 0.07, d1]);
            else b.box(trim, [d0, leafTop, z * cs], [d1, leafTop + 0.07, (z + 1) * cs]);
          }
          continue;
        }
        // Skirting along every wall this cell touches, and a soft contact shadow on the floor.
        const h = 0.1;
        const t = 0.018;
        const y = base + 0.004;
        if (at(x - 1, z) === CELL.WALL) {
          b.box(trim, [x * cs, base, z * cs], [x * cs + t, base + h, (z + 1) * cs]);
          aoStrip(x * cs, z * cs, x * cs, (z + 1) * cs, 1, 0, y);
        }
        if (at(x + 1, z) === CELL.WALL) {
          b.box(trim, [(x + 1) * cs - t, base, z * cs], [(x + 1) * cs, base + h, (z + 1) * cs]);
          aoStrip((x + 1) * cs, z * cs, (x + 1) * cs, (z + 1) * cs, -1, 0, y);
        }
        if (at(x, z - 1) === CELL.WALL) {
          b.box(trim, [x * cs, base, z * cs], [(x + 1) * cs, base + h, z * cs + t]);
          aoStrip(x * cs, z * cs, (x + 1) * cs, z * cs, 0, 1, y);
        }
        if (at(x, z + 1) === CELL.WALL) {
          b.box(trim, [x * cs, base, (z + 1) * cs - t], [(x + 1) * cs, base + h, (z + 1) * cs]);
          aoStrip(x * cs, (z + 1) * cs, (x + 1) * cs, (z + 1) * cs, 0, -1, y);
        }
      }
    }
  });

  for (const mesh of b.flush(group)) {
    if (mesh.material === ao) {
      mesh.castShadow = false;
      mesh.renderOrder = 1;
    }
  }
  for (const j of jambs) group.add(j);

  // ------------------------------------------------------------ flat roof (hatch shells)
  const roof: THREE.Object3D[] = [];
  const hatch = shell.openings.find((o) => o.kind === 'roof_hatch');
  if (hatch) {
    const membrane = new THREE.MeshStandardMaterial({ color: m.night ? '#2c3036' : '#5c6168', roughness: 0.92 });
    const rb = new Batch();
    for (const s of roofSolids(shell)) {
      if (s.shape !== 'cuboid') continue;
      const c = T([s.cx, s.cy, s.cz]);
      rb.box(membrane, [c[0] - s.hx, c[1] - s.hz, c[2] - s.hy], [c[0] + s.hx, c[1] + s.hz, c[2] + s.hy]);
    }
    // Parapet coping round the edge.
    const y = top + 0.08;
    for (const [x0, x1, z0, z1] of [
      [-0.06, W + 0.06, -0.06, 0.12],
      [-0.06, W + 0.06, D - 0.12, D + 0.06],
      [-0.06, 0.12, 0, D],
      [W - 0.12, W + 0.06, 0, D],
    ] as const) {
      rb.box(trim, [x0, y, z0], [x1, y + 0.16, z1]);
    }
    // Hatch curb and its lid, swung open.
    const hc = T(hatch.center);
    const hw = hatch.width / 2;
    const hd = hatch.height / 2;
    const curb = 0.06;
    for (const [x0, x1, z0, z1] of [
      [-hw - curb, hw + curb, -hd - curb, -hd],
      [-hw - curb, hw + curb, hd, hd + curb],
      [-hw - curb, -hw, -hd, hd],
      [hw, hw + curb, -hd, hd],
    ] as const) {
      rb.box(metal, [hc[0] + x0, y, hc[2] + z0], [hc[0] + x1, y + 0.14, hc[2] + z1]);
    }
    const meshes = rb.flush(group);
    if (hatch.width > 0.4) {
      const lid = new THREE.Mesh(new THREE.BoxGeometry(hatch.width + 2 * curb, 0.04, hatch.height + 2 * curb), metal);
      lid.geometry.translate(0, 0, (hatch.height + 2 * curb) / 2);
      lid.position.set(hc[0], y + 0.16, hc[2] + hd + curb);
      lid.rotation.x = -1.15;
      lid.castShadow = true;
      group.add(lid);
      meshes.push(lid);
    }
    roof.push(...meshes);
  }
  return { group, roof, windows };
}

/** One jamb block: siding outside, plaster inside and on the reveals, dark section cut on top. */
function jambMesh(solid: Solid, opening: Opening, origin: readonly number[], m: FacadeMaterials, top: number, H: number): THREE.Mesh {
  if (solid.shape !== 'cuboid') throw new Error('jambs are cuboids');
  const n = opening.normal;
  // Three box face order: +x, -x, +y, -y, +z, -z (Three z is sim y).
  const faceNormals: V3[] = [
    [1, 0, 0],
    [-1, 0, 0],
    [0, 0, 1],
    [0, 0, -1],
    [0, 1, 0],
    [0, -1, 0],
  ];
  const zTop = solid.cz + solid.hz - origin[2];
  const atFloorTop = Math.abs(((zTop % H) + H) % H) < 0.02 || zTop >= top - 0.02;
  const faces = faceNormals.map(([x, y, z], i) => {
    const dot = x * n[0] + y * n[1] + z * n[2];
    if (dot > 0.5) return m.siding;
    if (dot < -0.5) return m.wall;
    if (i === 2 && atFloorTop) return m.cap;
    // Reveals (the hole's sides, sill and head) are plastered like the room inside.
    return m.wall;
  });
  const geo = new THREE.BoxGeometry(solid.hx * 2, solid.hz * 2, solid.hy * 2);
  const mesh = new THREE.Mesh(geo, faces);
  mesh.position.set(solid.cx - origin[0], solid.cz - origin[2], solid.cy - origin[1]);
  planarUv(geo, mesh.position, 1);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  return mesh;
}

/**
 * UVs from shell-local position, in metres / scale: walls map (along-wall, height), floors and
 * caps map (x, z). Neighbouring boxes then line up, so siding courses run unbroken past openings.
 */
export function planarUv(geo: THREE.BufferGeometry, offset: THREE.Vector3, scale: number): void {
  const pos = geo.getAttribute('position') as THREE.BufferAttribute;
  const nor = geo.getAttribute('normal') as THREE.BufferAttribute;
  const uv = geo.getAttribute('uv') as THREE.BufferAttribute;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i) + offset.x;
    const y = pos.getY(i) + offset.y;
    const z = pos.getZ(i) + offset.z;
    const nx = Math.abs(nor.getX(i));
    const ny = Math.abs(nor.getY(i));
    if (ny > 0.5) uv.setXY(i, x / scale, z / scale);
    else if (nx > 0.5) uv.setXY(i, z / scale, y / scale);
    else uv.setXY(i, x / scale, y / scale);
  }
  uv.needsUpdate = true;
}

/** Alpha ramp across a strip (v = 0 at the wall, 1 away from it), eased so it reads as soft. */
function aoTexture(): THREE.Texture {
  const canvas = document.createElement('canvas');
  canvas.width = 4;
  canvas.height = 64;
  const g = canvas.getContext('2d')!;
  const grad = g.createLinearGradient(0, 0, 0, 64);
  // Canvas rows run top-down and the texture is flipped, so the bottom row is v = 0 (the wall).
  grad.addColorStop(1, 'rgb(255,255,255)');
  grad.addColorStop(0.75, 'rgb(130,130,130)');
  grad.addColorStop(0.4, 'rgb(35,35,35)');
  grad.addColorStop(0, 'rgb(0,0,0)');
  g.fillStyle = grad;
  g.fillRect(0, 0, 4, 64);
  return new THREE.CanvasTexture(canvas);
}

import type { MissionWorld, V3 } from '../mission/autonomy/contracts';
import { apertureBoxes, slabBoxes, viewSpecOf } from '../mission/autonomy/perception';
import type { CameraView } from '../perception/camera';
import { FLASH_HALF } from '../perception/model';
import { CELL } from '../shared/constants';
import type { SensorMode } from '../shared/types';
import { skyFromTime } from '../sim/lighting';
import { sizeOf } from '../targets/catalog';
import { resolveScene } from '../targets/motion';
import type { ResolvedTarget, TargetCategory, TargetClass } from '../targets/types';
import { sampleHeight } from '../world/outdoor/terrain';
import type { TreeInstance } from '../world/outdoor/types';
import { PART, poseIndex, type HumanoidPoses } from './humanoid';
import { defaultPalette, type Palette } from './palette';
import { Raster, type Paint } from './raster';

/** Camera image size. 4:3 like the FPV and gimbal lenses. */
export const VISION_W = 160;
export const VISION_H = 120;
/** Height a door cell's lintel starts at, meters above its floor. */
const LINTEL_Z = 2.1;
const TREE_BIN_M = 8;
const TERRAIN_STEP_M = 2;

/** What a drawn instance is, for labels and scoring. Instance k is truth[k - 1]; 0 is background. */
export interface FrameTruth {
  id: string;
  classId: TargetClass;
  category: TargetCategory;
  buildingId: string | null;
  /** World-frame center. */
  center: V3;
}

export interface Frame {
  width: number;
  height: number;
  /** 8-bit RGB, row-major, origin top-left. */
  rgb: Uint8Array;
  /** Range along each pixel's ray, meters. Infinity where the ray sees sky. */
  depth: Float32Array;
  /** Instance per pixel; see FrameTruth. */
  inst: Int32Array;
  truth: FrameTruth[];
  /** Auto-exposure gain the sensor applied. High means a dark, noisy frame. */
  gain: number;
  view: CameraView;
}

export interface FrameLighting {
  timeOfDay: number;
  flashlightOn: boolean;
  sensorMode: SensorMode;
}

type Item =
  | { t: 0; x0: number; y0: number; z0: number; x1: number; y1: number; z1: number; p: Paint }
  | { t: 1; cx: number; cy: number; cz: number; hx: number; hy: number; hz: number; yaw: number; p: Paint };

interface BuildingGeom {
  id: string;
  origin: V3;
  box: { minX: number; minY: number; maxX: number; maxY: number; minZ: number; maxZ: number };
  items: Item[];
  cs: number;
  floorH: number;
  rooms: Array<{ w: number; d: number; roomOf: Int16Array }>;
  lights: Uint8Array;
  /** 1 every room lit, 0 none, -1 mixed (per-pixel lookup). */
  lightsUniform: 1 | 0 | -1;
  targets: () => ReturnType<typeof resolveScene>;
  setTime: (t: number) => void;
}

const GAMMA_STEPS = 4096;
const GAMMA_LUT = (() => {
  const lut = new Uint8Array(GAMMA_STEPS);
  for (let i = 0; i < GAMMA_STEPS; i++) lut[i] = Math.round(Math.pow(i / (GAMMA_STEPS - 1), 1 / 2.2) * 255);
  return lut;
})();
const NOISE_MASK = 65535;
/** Unit-variance Gaussian samples, drawn once with a fixed seed. */
const NOISE_TABLE = (() => {
  const t = new Float32Array(NOISE_MASK + 1);
  let s = 0x2545f491;
  const u = (): number => {
    s ^= s << 13;
    s ^= s >>> 17;
    s ^= s << 5;
    return ((s >>> 0) + 0.5) / 4294967296;
  };
  for (let i = 0; i < t.length; i++) t[i] = Math.sqrt(-2 * Math.log(u())) * Math.cos(2 * Math.PI * u());
  return t;
})();

/** Approximate luminance percentile of a linear color buffer, from a 256-bin histogram on 0..2. */
function percentileLum(color: Float32Array, n: number, q: number): number {
  const bins = new Uint32Array(256);
  for (let i = 0; i < n; i++) {
    const l = (color[i * 3]! + color[i * 3 + 1]! + color[i * 3 + 2]!) / 3;
    bins[Math.min(255, (l * 128) | 0)]!++;
  }
  let acc = 0;
  const want = q * n;
  for (let k = 0; k < 256; k++) {
    acc += bins[k]!;
    if (acc >= want) return (k + 1) / 128;
  }
  return 2;
}

function hash3(x: number, y: number, z: number): number {
  let h = Math.imul(x | 0, 0x27d4eb2d) ^ Math.imul(y | 0, 0x165667b1) ^ Math.imul(z | 0, 0x9e3779b1);
  h = Math.imul(h ^ (h >>> 15), 0x85ebca6b);
  h ^= h >>> 13;
  return (h >>> 0) / 4294967296;
}

/** Two-octave cell noise in 0..1, for surface grain. */
function grainAt(x: number, y: number, z: number): number {
  return 0.6 * hash3(Math.floor(x * 4), Math.floor(y * 4), Math.floor(z * 4)) + 0.4 * hash3(Math.floor(x * 13), Math.floor(y * 13), Math.floor(z * 13));
}

function buildingGeometry(world: MissionWorld, index: number, palette: Palette): BuildingGeom {
  const b = world.buildings[index]!;
  const spec = viewSpecOf(b);
  const cs = spec.cellSize;
  const H = spec.floorHeight;
  const [ox, oy, oz] = b.shell.origin;
  const items: Item[] = [];
  const box = (x0: number, y0: number, z0: number, x1: number, y1: number, z1: number, p: Paint): void => {
    items.push({ t: 0, x0: x0 + ox, y0: y0 + oy, z0: z0 + oz, x1: x1 + ox, y1: y1 + oy, z1: z1 + oz, p });
  };
  const rooms: BuildingGeom['rooms'] = [];
  spec.floors.forEach((floor, k) => {
    const z0 = k * H;
    // Wall cells in row runs of one surface.
    for (let iy = 0; iy < floor.d; iy++) {
      let start = -1;
      let surf = -1;
      for (let ix = 0; ix <= floor.w; ix++) {
        const i = iy * floor.w + ix;
        const wall = ix < floor.w && floor.cells[i] === CELL.WALL;
        const s = wall ? (floor.surfaces?.[i] ?? 0) : -1;
        if (start >= 0 && (!wall || s !== surf)) {
          box(start * cs, iy * cs, z0, ix * cs, (iy + 1) * cs, z0 + H, palette.wall[surf] ?? palette.wall[0]!);
          start = -1;
        }
        if (wall && start < 0) {
          start = ix;
          surf = s;
        }
        if (ix < floor.w && floor.cells[i] === CELL.DOOR) box(ix * cs, iy * cs, z0 + LINTEL_Z, (ix + 1) * cs, (iy + 1) * cs, z0 + H, palette.frame);
      }
    }
    const roomOf = new Int16Array(floor.w * floor.d).fill(-1);
    rooms.push({ w: floor.w, d: floor.d, roomOf });
  });
  let roomIndex = 0;
  spec.floors.forEach((floor, k) => {
    const roomOf = rooms[k]!.roomOf;
    for (const room of floor.rooms) {
      for (let iy = room.z; iy < room.z + room.d; iy++) for (let ix = room.x; ix < room.x + room.w; ix++) roomOf[iy * floor.w + ix] = roomIndex;
      roomIndex++;
    }
  });
  const w0 = spec.floors[0]!;
  box(0, 0, -0.02, w0.w * cs, w0.d * cs, 0.005, palette.floor);
  for (const slab of slabBoxes(b)) box(slab.min[0], slab.min[1], slab.min[2], slab.max[0], slab.max[1], slab.max[2], palette.ceiling);
  for (const wall of apertureBoxes(b)) box(wall.min[0], wall.min[1], wall.min[2], wall.max[0], wall.max[1], wall.max[2], palette.wall[0]!);
  const furniture = spec.furniture;
  if (furniture) {
    for (const part of furniture.boxes) {
      const kind = furniture.props[part.prop]?.kind ?? 'table';
      items.push({ t: 1, cx: part.cx + ox, cy: part.cy + oy, cz: part.cz + oz, hx: part.hx, hy: part.hy, hz: part.hz, yaw: part.yaw, p: palette.furniture[kind] });
    }
  }
  const xs = b.footprint.map((p) => p.x);
  const ys = b.footprint.map((p) => p.y);
  let time = 0;
  return {
    id: b.id,
    origin: [ox, oy, oz],
    box: { minX: Math.min(...xs), minY: Math.min(...ys), maxX: Math.max(...xs), maxY: Math.max(...ys), minZ: oz, maxZ: oz + b.heightM },
    items,
    cs,
    floorH: H,
    rooms,
    lights: b.interiorLightsOn,
    lightsUniform: b.interiorLightsOn.every((v) => v > 0) ? 1 : b.interiorLightsOn.every((v) => v === 0) ? 0 : -1,
    targets: () => resolveScene(b.scene.targets, time),
    setTime: (t) => (time = t),
  };
}

function boxDistance(p: V3, box: BuildingGeom['box']): number {
  const dx = Math.max(box.minX - p[0], 0, p[0] - box.maxX);
  const dy = Math.max(box.minY - p[1], 0, p[1] - box.maxY);
  const dz = Math.max(box.minZ - p[2], 0, p[2] - box.maxZ);
  return Math.hypot(dx, dy, dz);
}

export interface VisionSceneOptions {
  width?: number;
  height?: number;
  palette?: Palette;
  /** Buildings and trees farther than this are not drawn, meters. */
  drawDistanceM?: number;
}

/**
 * The drone camera's view of a MissionWorld: building shells, furniture,
 * terrain, trees, and the planted people, objects and decoys, rasterized at
 * 160 x 120 and shaded with the world's sun, room lights and flashlight, then
 * passed through a simple sensor (auto exposure, read noise, 8-bit output).
 */
export class VisionScene {
  readonly width: number;
  readonly height: number;
  readonly palette: Palette;
  private readonly raster: Raster;
  private readonly buildings: BuildingGeom[];
  private readonly trees = new Map<string, TreeInstance[]>();
  private readonly drawDistance: number;
  private readonly color: Float32Array;
  private triSide = new Int16Array(4096);

  constructor(
    readonly world: MissionWorld,
    readonly poses: HumanoidPoses,
    opts: VisionSceneOptions = {},
  ) {
    this.width = opts.width ?? VISION_W;
    this.height = opts.height ?? VISION_H;
    this.palette = opts.palette ?? defaultPalette();
    this.drawDistance = opts.drawDistanceM ?? 60;
    this.raster = new Raster(this.width, this.height);
    this.color = new Float32Array(this.width * this.height * 3);
    this.buildings = world.buildings.map((_, i) => buildingGeometry(world, i, this.palette));
    for (const tree of world.outdoor.trees) {
      const key = `${Math.floor(tree.x / TREE_BIN_M)}:${Math.floor(tree.y / TREE_BIN_M)}`;
      const list = this.trees.get(key) ?? [];
      list.push(tree);
      this.trees.set(key, list);
    }
  }

  /** Building containing a world point, or null outdoors. */
  private buildingAt(x: number, y: number, z: number): BuildingGeom | null {
    for (const b of this.buildings) {
      const bx = b.box;
      if (x > bx.minX && x < bx.maxX && y > bx.minY && y < bx.maxY && z > bx.minZ - 0.05 && z < bx.maxZ) return b;
    }
    return null;
  }

  private roomLit(b: BuildingGeom, x: number, y: number, z: number): boolean {
    const k = Math.floor((z - b.origin[2]) / b.floorH);
    const floor = b.rooms[k];
    if (!floor) return false;
    const ix = Math.floor((x - b.origin[0]) / b.cs);
    const iy = Math.floor((y - b.origin[1]) / b.cs);
    if (ix < 0 || iy < 0 || ix >= floor.w || iy >= floor.d) return false;
    const room = floor.roomOf[iy * floor.w + ix]!;
    return room >= 0 && (b.lights[room] ?? 0) > 0;
  }

  render(view: CameraView, tSec: number, lighting: FrameLighting, noiseSeed: number): Frame {
    const r = this.raster;
    r.begin(view);
    const truth: FrameTruth[] = [];
    const eye = view.position;
    const draw = (item: Item): void => {
      if (item.t === 0) r.box(item.x0, item.y0, item.z0, item.x1, item.y1, item.z1, item.p, 0);
      else r.orientedBox(item.cx, item.cy, item.cz, item.hx, item.hy, item.hz, item.yaw, item.p, 0);
    };
    for (const b of this.buildings) {
      if (boxDistance(eye, b.box) > this.drawDistance) continue;
      const bx = b.box;
      const cx = (bx.minX + bx.maxX) / 2;
      const cy = (bx.minY + bx.maxY) / 2;
      const cz = (bx.minZ + bx.maxZ) / 2;
      if (!r.visible(cx, cy, cz, Math.hypot(bx.maxX - bx.minX, bx.maxY - bx.minY, bx.maxZ - bx.minZ) / 2)) continue;
      for (const item of b.items) draw(item);
      b.setTime(tSec);
      for (const target of b.targets()) {
        truth.push({ id: target.id, classId: target.classId, category: target.category, buildingId: b.id, center: [target.center[0] + b.origin[0], target.center[1] + b.origin[1], target.center[2] + b.origin[2]] });
        this.drawTarget(target, b.origin, truth.length, tSec);
      }
    }
    if (!this.buildingAt(eye[0], eye[1], eye[2])) {
      this.drawOutdoor(view);
      for (const target of resolveScene(this.world.outdoorScene.targets, tSec)) {
        if (Math.hypot(target.center[0] - eye[0], target.center[1] - eye[1]) > this.drawDistance) continue;
        truth.push({ id: target.id, classId: target.classId, category: target.category, buildingId: null, center: [...target.center] });
        this.drawTarget(target, [0, 0, 0], truth.length, tSec);
      }
    }
    return this.shade(view, lighting, truth, noiseSeed);
  }

  private drawOutdoor(view: CameraView): void {
    const r = this.raster;
    const pal = this.palette;
    const [ex, ey] = view.position;
    const terrain = this.world.outdoor.terrain;
    const size = this.world.outdoor.size;
    const reach = Math.min(this.drawDistance, view.far);
    const step = TERRAIN_STEP_M;
    const x0 = Math.max(0, Math.floor((ex - reach) / step) * step);
    const y0 = Math.max(0, Math.floor((ey - reach) / step) * step);
    const x1 = Math.min(size, ex + reach);
    const y1 = Math.min(size, ey + reach);
    const nx = Math.ceil((x1 - x0) / step) + 1;
    const ny = Math.ceil((y1 - y0) / step) + 1;
    const h = new Float32Array(nx * ny);
    for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) h[j * nx + i] = sampleHeight(terrain, x0 + i * step, y0 + j * step);
    for (let j = 0; j + 1 < ny; j++) {
      const ya = y0 + j * step;
      for (let i = 0; i + 1 < nx; i++) {
        const xa = x0 + i * step;
        const h00 = h[j * nx + i]!;
        if (!r.visible(xa + step / 2, ya + step / 2, h00, step)) continue;
        r.quad(xa, ya, h00, xa + step, ya, h[j * nx + i + 1]!, xa + step, ya + step, h[(j + 1) * nx + i + 1]!, xa, ya + step, h[(j + 1) * nx + i]!, pal.ground, 0);
      }
    }
    const bx0 = Math.floor((ex - reach) / TREE_BIN_M);
    const bx1 = Math.floor((ex + reach) / TREE_BIN_M);
    const by0 = Math.floor((ey - reach) / TREE_BIN_M);
    const by1 = Math.floor((ey + reach) / TREE_BIN_M);
    for (let by = by0; by <= by1; by++) {
      for (let bx = bx0; bx <= bx1; bx++) {
        for (const tree of this.trees.get(`${bx}:${by}`) ?? []) this.drawTree(tree);
      }
    }
  }

  private drawTree(tree: TreeInstance): void {
    const r = this.raster;
    const pal = this.palette;
    const top = tree.z + tree.canopyZ + tree.canopyRadius;
    if (!r.visible(tree.x, tree.y, (tree.z + top) / 2, Math.max(tree.canopyRadius, (top - tree.z) / 2))) return;
    const tint = (p: Paint): Paint => ({ ...p, r: p.r * (0.8 + 0.4 * tree.tint), g: p.g * (0.85 + 0.3 * tree.tint) });
    if (tree.kind === 'undergrowth') {
      const rad = tree.canopyRadius;
      this.octahedron(tree.x, tree.y, tree.z + rad * 0.6, rad, rad * 0.7, tree.yaw, tint(pal.leaf));
      return;
    }
    const trunkTop = tree.z + Math.min(tree.trunkHeight, tree.canopyZ);
    r.orientedBox(tree.x, tree.y, (tree.z + trunkTop) / 2, tree.trunkRadius, tree.trunkRadius, (trunkTop - tree.z) / 2, tree.yaw, pal.trunk, 0);
    const cz = tree.z + tree.canopyZ;
    if (tree.kind === 'pine') {
      // Cone as an eight-sided pyramid.
      const rad = tree.canopyRadius;
      const base = cz - rad;
      const apex = cz + rad * 1.4;
      const p = tint(pal.pine);
      for (let k = 0; k < 8; k++) {
        const a0 = tree.yaw + (k * Math.PI) / 4;
        const a1 = a0 + Math.PI / 4;
        r.triangle(tree.x + Math.cos(a0) * rad, tree.y + Math.sin(a0) * rad, base, tree.x + Math.cos(a1) * rad, tree.y + Math.sin(a1) * rad, base, tree.x, tree.y, apex, p, 0);
      }
      return;
    }
    this.octahedron(tree.x, tree.y, cz, tree.canopyRadius, tree.canopyRadius * 0.85, tree.yaw, tint(pal.leaf));
  }

  /** A faceted blob: an octahedron with its equator split into eight. */
  private octahedron(x: number, y: number, z: number, rad: number, radZ: number, yaw: number, p: Paint): void {
    const r = this.raster;
    for (let k = 0; k < 8; k++) {
      const a0 = yaw + (k * Math.PI) / 4;
      const a1 = a0 + Math.PI / 4;
      const ax = x + Math.cos(a0) * rad;
      const ay = y + Math.sin(a0) * rad;
      const bx = x + Math.cos(a1) * rad;
      const by = y + Math.sin(a1) * rad;
      r.triangle(ax, ay, z, bx, by, z, x, y, z + radZ, p, 0);
      r.triangle(ax, ay, z, bx, by, z, x, y, z - radZ * 0.7, p, 0);
    }
  }

  private drawTarget(target: ResolvedTarget, origin: V3, inst: number, tSec: number): void {
    const r = this.raster;
    const pal = this.palette;
    const fx = target.feet[0] + origin[0];
    const fy = target.feet[1] + origin[1];
    const fz = target.feet[2] + origin[2];
    for (const occ of target.occluders) {
      r.box(occ.min[0] + origin[0], occ.min[1] + origin[1], occ.min[2] + origin[2], occ.max[0] + origin[0], occ.max[1] + origin[1], occ.max[2] + origin[2], pal.crate, 0);
    }
    const humanoid = target.classId === 'person' || target.classId === 'mannequin';
    if (humanoid) {
      if (!r.visible(fx, fy, fz + 0.9, 1.4)) return;
      const state = target.classId === 'mannequin' ? 'standing' : target.state;
      if (state === 'sitting') r.orientedBox(fx, fy, fz + 0.21, 0.24, 0.23, 0.21, target.yaw - Math.PI / 2, pal.chair, 0);
      const poses = this.poses;
      const k = poseIndex(poses, state, target.id, tSec);
      const base = k * poses.verts * 3;
      const c = Math.cos(target.yaw);
      const s = Math.sin(target.yaw);
      const pos = poses.positions;
      const idx = poses.indices;
      const parts = poses.parts;
      const shirt = pal.shirt(state, target.id);
      const paints: Paint[] = target.classId === 'mannequin'
        ? [pal.mannequin, pal.mannequin, pal.mannequin, pal.mannequin]
        : [pal.skin, shirt, shirt, pal.pants];
      for (let t = 0; t < idx.length; t += 3) {
        const a = idx[t]!;
        const b = idx[t + 1]!;
        const d = idx[t + 2]!;
        const ia = base + a * 3;
        const ib = base + b * 3;
        const id = base + d * 3;
        r.triangle(
          fx + c * pos[ia]! - s * pos[ia + 1]!, fy + s * pos[ia]! + c * pos[ia + 1]!, fz + pos[ia + 2]!,
          fx + c * pos[ib]! - s * pos[ib + 1]!, fy + s * pos[ib]! + c * pos[ib + 1]!, fz + pos[ib + 2]!,
          fx + c * pos[id]! - s * pos[id + 1]!, fy + s * pos[id]! + c * pos[id + 1]!, fz + pos[id + 2]!,
          paints[parts[a] ?? PART.body]!,
          inst,
        );
      }
      return;
    }
    const size = sizeOf(target.classId, target.state);
    const hz = size.height / 2;
    const p = pal.prop(target.classId);
    r.orientedBox(fx, fy, fz + hz, size.hx, size.hy, hz, target.yaw, p, inst);
    if (target.classId === 'backpack') {
      // Dark top flap and straps: what a real pack has and the look-alike bag lacks.
      r.orientedBox(fx, fy, fz + size.height * 0.86, size.hx * 1.06, size.hy * 1.06, size.height * 0.14, target.yaw, pal.strap, inst);
      const c = Math.cos(target.yaw);
      const s = Math.sin(target.yaw);
      for (const side of [-0.5, 0.5]) {
        const ly = side * size.hy * 1.2;
        r.orientedBox(fx - s * ly, fy + c * ly, fz + hz, size.hx * 1.08, 0.025, hz * 0.95, target.yaw, pal.strap, inst);
      }
    } else if (target.classId === 'toolbox') {
      r.orientedBox(fx, fy, fz + size.height + 0.04, size.hx * 0.6, 0.02, 0.04, target.yaw, pal.strap, inst);
    } else if (target.classId === 'medkit') {
      r.orientedBox(fx, fy, fz + size.height + 0.01, Math.min(0.045, size.hx), Math.min(0.08, size.hy), 0.012, target.yaw, pal.cross, inst);
      r.orientedBox(fx, fy, fz + size.height + 0.01, Math.min(0.08, size.hx), Math.min(0.045, size.hy), 0.012, target.yaw, pal.cross, inst);
    }
  }

  private shade(view: CameraView, lighting: FrameLighting, truth: FrameTruth[], noiseSeed: number): Frame {
    const r = this.raster;
    const W = this.width;
    const H = this.height;
    const n = W * H;
    const color = this.color;
    const depth = new Float32Array(n);
    const inst = new Int32Array(n);
    const sky = skyFromTime(lighting.timeOfDay);
    const day = 1 - sky.night;
    const [sx, sy, sz] = sky.sunDir;
    const sunUp = Math.max(0, Math.min(1, sky.sunIntensity / 2.5));
    const gainL = this.palette.lightGain;
    const [ex, ey, ez] = view.position;
    const [fx, fy, fz] = view.forward;
    const [rx, ry, rz] = view.right;
    const [ux, uy, uz] = view.up;
    const tanY = Math.tan(view.fovY / 2);
    const tanX = tanY * Math.max(0.05, view.aspect);
    const flashCos0 = Math.cos(FLASH_HALF + 0.07);
    const flashCos1 = Math.cos(FLASH_HALF - 0.07);
    const ir = lighting.sensorMode === 'ir';
    const depthMode = lighting.sensorMode === 'depth';
    let lumSum = 0;
    if (this.triSide.length < r.triCount) this.triSide = new Int16Array(r.triCount * 2);
    const triSide = this.triSide;
    triSide.fill(-2, 0, r.triCount);
    const skyK = ir ? 0.05 : depthMode ? 0 : 0.06 + 0.94 * day;
    for (let py = 0; py < H; py++) {
      const ndcY = (0.5 - (py + 0.5) / H) * 2 * tanY;
      for (let px = 0; px < W; px++) {
        const i = py * W + px;
        const ndcX = ((px + 0.5) / W - 0.5) * 2 * tanX;
        const dx = fx + rx * ndcX + ux * ndcY;
        const dy = fy + ry * ndcX + uy * ndcY;
        const dz = fz + rz * ndcX + uz * ndcY;
        const dl = Math.sqrt(dx * dx + dy * dy + dz * dz);
        const t = r.tri[i]!;
        const o = i * 3;
        if (t < 0) {
          depth[i] = Infinity;
          inst[i] = 0;
          const v = skyK;
          color[o] = sky.skyColor[0] * v * gainL;
          color[o + 1] = sky.skyColor[1] * v * gainL;
          color[o + 2] = sky.skyColor[2] * v * gainL;
          lumSum += (color[o]! + color[o + 1]! + color[o + 2]!) / 3;
          continue;
        }
        const z = 1 / r.invZ[i]!;
        const range = z * dl;
        depth[i] = range;
        inst[i] = r.inst[t]!;
        const wx = ex + dx * z;
        const wy = ey + dy * z;
        const wz = ez + dz * z;
        let nx = r.normal[t * 3]!;
        let ny = r.normal[t * 3 + 1]!;
        let nz = r.normal[t * 3 + 2]!;
        if (nx * dx + ny * dy + nz * dz > 0) {
          nx = -nx;
          ny = -ny;
          nz = -nz;
        }
        const vx = -dx / dl;
        const vy = -dy / dl;
        const vz = -dz / dl;
        const nv = Math.max(0, nx * vx + ny * vy + nz * vz);
        const po = t * 6;
        const grain = r.paint[po + 3]!;
        const tex = grain > 0 ? 1 + grain * (grainAt(wx, wy, wz) - 0.5) : 1;
        if (depthMode) {
          const v = Math.max(0, 1 - range / 12);
          color[o] = color[o + 1] = color[o + 2] = v;
          lumSum += v;
          continue;
        }
        // Which side of a shell a triangle faces is the same for all its pixels.
        let bi = triSide[t]!;
        if (bi === -2) {
          const hit = this.buildingAt(wx + nx * 0.15, wy + ny * 0.15, wz + nz * 0.15);
          bi = hit ? this.buildings.indexOf(hit) : -1;
          if (t < triSide.length) triSide[t] = bi;
        }
        const b = bi >= 0 ? this.buildings[bi]! : null;
        if (ir) {
          const heat = r.paint[po + 4]!;
          const sunWarm = b ? 0 : 0.12 * day * Math.max(0, nx * sx + ny * sy + nz * sz);
          const v = heat * 0.85 + (1 - heat) * (0.16 + 0.1 * (tex - 1) * 4 + sunWarm);
          color[o] = color[o + 1] = color[o + 2] = v;
          lumSum += v;
          continue;
        }
        let light: number;
        let spot = 0;
        if (b) {
          light = 0.02 + day * 0.3 * (0.7 + 0.3 * Math.abs(nz));
          const lit = b.lightsUniform === 1 || (b.lightsUniform === -1 && this.roomLit(b, wx + nx * 0.15, wy + ny * 0.15, wz + nz * 0.15));
          if (lit) {
            const up = Math.max(0, nz);
            light += 0.7 * (0.65 + 0.35 * up);
            spot += 0.3;
          }
        } else {
          light = 0.04 + day * (0.32 + 0.68 * sunUp * Math.max(0, nx * sx + ny * sy + nz * sz));
          spot += day * 0.3;
        }
        if (lighting.flashlightOn) {
          const cosA = 1 / dl;
          if (cosA > flashCos0) {
            const edge = cosA >= flashCos1 ? 1 : (cosA - flashCos0) / (flashCos1 - flashCos0);
            const f = (1.3 * edge * nv) / (1 + (range / 6) * (range / 6));
            light += f;
            spot += f;
          }
        }
        light *= gainL;
        const gloss = r.paint[po + 5]!;
        const spec = gloss > 0 ? gloss * 0.6 * Math.pow(nv, 10) * spot * gainL : 0;
        color[o] = r.paint[po]! * tex * light + spec;
        color[o + 1] = r.paint[po + 1]! * tex * light + spec;
        color[o + 2] = r.paint[po + 2]! * tex * light + spec;
        lumSum += (color[o]! + color[o + 1]! + color[o + 2]!) / 3;
      }
    }
    // Sensor: auto exposure toward a mid-grey mean, held back so the brightest 1% does not clip
    // (a flashlight spot stays readable), read noise that grows with gain, gamma, 8 bits.
    const mean = lumSum / n;
    let gain = 1;
    if (!ir && !depthMode) {
      gain = Math.max(0.5, Math.min(10, 0.18 / Math.max(mean, 1e-4)));
      const p99 = percentileLum(color, n, 0.99);
      if (p99 > 0) gain = Math.max(0.5, Math.min(gain, 0.9 / p99));
    }
    const sigma = ir ? 0.02 : 0.004 + 0.006 * gain;
    const rgb = new Uint8Array(n * 3);
    let s = (noiseSeed >>> 0) || 1;
    const noise = NOISE_TABLE;
    const lut = GAMMA_LUT;
    const scale = GAMMA_STEPS - 1;
    for (let i = 0; i < n * 3; i++) {
      s ^= s << 13;
      s ^= s >>> 17;
      s ^= s << 5;
      const lin = color[i]! * gain + noise[(s >>> 0) & NOISE_MASK]! * sigma;
      rgb[i] = lin <= 0 ? 0 : lin >= 1 ? 255 : lut[(lin * scale) | 0]!;
    }
    return { width: W, height: H, rgb, depth, inst, truth, gain, view };
  }
}

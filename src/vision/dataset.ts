import type { MissionWorld, MissionWorldOptions, TargetWant, V3 } from '../mission/autonomy/contracts';
import { viewSpecOf } from '../mission/autonomy/perception';
import { buildMissionWorld } from '../mission/autonomy/world';
import { cameraLookingAt, fpvCameraFromDrone, type CameraView } from '../perception/camera';
import { CELL } from '../shared/constants';
import { RNG } from '../shared/rng';
import { resolveScene } from '../targets/motion';
import { OBJECT_IDS, PERSON_STATES, type TargetClass } from '../targets/types';
import { sampleHeight } from '../world/outdoor/terrain';
import type { HumanoidPoses } from './humanoid';
import { defaultPalette, randomPalette } from './palette';
import { VisionScene, type Frame } from './scene';

/** Classes the detector reports. Decoys are not a class: they are what it must learn to ignore. */
export const DETECT_CLASSES = ['person', ...OBJECT_IDS] as const satisfies readonly TargetClass[];
export type DetectClass = (typeof DETECT_CLASSES)[number];

export function classIndex(classId: TargetClass): number {
  return (DETECT_CLASSES as readonly TargetClass[]).indexOf(classId);
}

/** FPV and gimbal lenses, as in src/mission/autonomy/perception.ts. */
const FPV_ASPECT = 4 / 3;
const GIMBAL_FOV_Y = (70 * Math.PI) / 180;

/** A visible instance in a frame: its box from the pixels it owns. */
export interface FrameLabel {
  classId: TargetClass;
  /** Index into DETECT_CLASSES, or -1 for a decoy. */
  cls: number;
  /** Normalized, origin top-left. */
  x: number;
  y: number;
  w: number;
  h: number;
  pixels: number;
  distance: number;
  targetId: string;
}

/** Labels from the instance buffer: one box per instance that owns at least `minPixels`. */
export function frameLabels(frame: Frame, minPixels = 4): FrameLabel[] {
  const W = frame.width;
  const H = frame.height;
  const n = frame.truth.length;
  const minX = new Int32Array(n + 1).fill(W);
  const minY = new Int32Array(n + 1).fill(H);
  const maxX = new Int32Array(n + 1).fill(-1);
  const maxY = new Int32Array(n + 1).fill(-1);
  const count = new Int32Array(n + 1);
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const k = frame.inst[y * W + x]!;
      if (k <= 0) continue;
      count[k]!++;
      if (x < minX[k]!) minX[k] = x;
      if (x > maxX[k]!) maxX[k] = x;
      if (y < minY[k]!) minY[k] = y;
      if (y > maxY[k]!) maxY[k] = y;
    }
  }
  const out: FrameLabel[] = [];
  const eye = frame.view.position;
  for (let k = 1; k <= n; k++) {
    if (count[k]! < minPixels) continue;
    const t = frame.truth[k - 1]!;
    out.push({
      classId: t.classId,
      cls: classIndex(t.classId),
      x: minX[k]! / W,
      y: minY[k]! / H,
      w: (maxX[k]! - minX[k]! + 1) / W,
      h: (maxY[k]! - minY[k]! + 1) / H,
      pixels: count[k]!,
      distance: Math.hypot(t.center[0] - eye[0], t.center[1] - eye[1], t.center[2] - eye[2]),
      targetId: t.id,
    });
  }
  return out;
}

/** A seeded world with a mix of people, objects and decoys, lighting and lights. */
export function trainingWorldOptions(seed: number): MissionWorldOptions {
  const rng = new RNG(seed >>> 0, 'vision-world');
  const buildings = 1 + Math.floor(rng.next() * 3);
  const u = rng.next();
  const timeOfDay = u < 0.55 ? 8 + rng.next() * 9 : u < 0.7 ? (rng.next() < 0.5 ? 6 + rng.next() * 1.5 : 18 + rng.next() * 2) : 21 + rng.next() * 6;
  const night = timeOfDay >= 20.5 || timeOfDay < 6.5;
  const targets: TargetWant[] = [];
  const people = Math.floor(rng.next() * 4);
  for (let i = 0; i < people; i++) {
    const state = PERSON_STATES[Math.floor(rng.next() * PERSON_STATES.length)]!;
    const outdoor = rng.next() < 0.25;
    targets.push({ classId: 'person', state, where: outdoor ? 'outdoor' : 'indoor', building: Math.floor(rng.next() * buildings) });
  }
  const objects = Math.floor(rng.next() * 4);
  for (let i = 0; i < objects; i++) {
    const classId = OBJECT_IDS[Math.floor(rng.next() * OBJECT_IDS.length)]!;
    targets.push({ classId, where: rng.next() < 0.15 ? 'outdoor' : 'indoor', building: Math.floor(rng.next() * buildings) });
  }
  const lights = rng.next();
  return {
    seed,
    buildings,
    timeOfDay: timeOfDay % 24,
    flashlightOn: night ? rng.next() < 0.85 : rng.next() < 0.1,
    interiorLights: lights < 0.45 ? 'on' : lights < 0.7 ? 'off' : 'seeded',
    targets,
    decoys: Math.floor(rng.next() * 4),
  };
}

/** The training world for a seed. Outdoor targets the planter cannot fit move indoors. */
export function buildTrainingWorld(seed: number): MissionWorld {
  const options = trainingWorldOptions(seed);
  try {
    return buildMissionWorld(options);
  } catch {
    return buildMissionWorld({ ...options, targets: options.targets?.map((t) => ({ ...t, where: 'indoor' as const })) });
  }
}

/** Empty cells a camera can sit in, per building, with their floor. */
function freeCells(world: MissionWorld, index: number): Array<{ x: number; y: number; floor: number }> {
  const b = world.buildings[index]!;
  const spec = viewSpecOf(b);
  const out: Array<{ x: number; y: number; floor: number }> = [];
  spec.floors.forEach((floor, k) => {
    for (let iy = 1; iy < floor.d - 1; iy++) {
      for (let ix = 1; ix < floor.w - 1; ix++) {
        if (floor.cells[iy * floor.w + ix] !== CELL.EMPTY) continue;
        out.push({ x: b.shell.origin[0] + (ix + 0.5) * spec.cellSize, y: b.shell.origin[1] + (iy + 0.5) * spec.cellSize, floor: k });
      }
    }
  });
  return out;
}

function cellFree(world: MissionWorld, index: number, x: number, y: number, z: number): boolean {
  const b = world.buildings[index]!;
  const spec = b.shell.spec;
  const lx = x - b.shell.origin[0];
  const ly = y - b.shell.origin[1];
  const k = Math.floor((z - b.shell.origin[2]) / spec.floorHeight);
  const floor = spec.floors[k];
  if (!floor) return false;
  // Keep 0.25 m from walls: sample the cell under each corner of a small square.
  for (const [ox, oy] of [[-0.25, -0.25], [0.25, -0.25], [-0.25, 0.25], [0.25, 0.25]] as const) {
    const ix = Math.floor((lx + ox) / spec.cellSize);
    const iy = Math.floor((ly + oy) / spec.cellSize);
    if (ix < 0 || iy < 0 || ix >= floor.w || iy >= floor.d) return false;
    const c = floor.cells[iy * floor.w + ix];
    if (c !== CELL.EMPTY && c !== CELL.DOOR) return false;
  }
  for (const part of spec.furniture?.boxes ?? []) {
    if (part.floor !== k) continue;
    const dx = lx - part.cx;
    const dy = ly - part.cy;
    const c = Math.cos(-part.yaw);
    const s = Math.sin(-part.yaw);
    const ux = c * dx - s * dy;
    const uy = s * dx + c * dy;
    const zLocal = z - b.shell.origin[2];
    if (Math.abs(ux) < part.hx + 0.2 && Math.abs(uy) < part.hy + 0.2 && Math.abs(zLocal - part.cz) < part.hz + 0.2) return false;
  }
  return true;
}

export interface SampledView {
  view: CameraView;
  /** What the pose was aimed at, for the dataset's bookkeeping. */
  kind: 'target' | 'room' | 'outdoor' | 'gimbal';
}

/**
 * Camera poses like the ones a mission flies: FPV inside rooms at flight
 * height, FPV outdoors near the buildings, and the nadir gimbal at altitude.
 * Most poses are aimed near a target or decoy, at 1.2 to 14 m, with the
 * yaw jittered so the target lands anywhere in the frame.
 */
export function sampleViews(world: MissionWorld, count: number, seed: number): SampledView[] {
  const rng = new RNG(seed >>> 0, 'vision-views');
  const out: SampledView[] = [];
  const free = world.buildings.map((_, i) => freeCells(world, i));
  const anchors: Array<{ c: V3; building: number; floorZ: number }> = [];
  world.buildings.forEach((b, i) => {
    for (const t of resolveScene(b.scene.targets, 0)) {
      anchors.push({ c: [t.center[0] + b.shell.origin[0], t.center[1] + b.shell.origin[1], t.center[2] + b.shell.origin[2]], building: i, floorZ: b.shell.origin[2] + t.floor * b.shell.spec.floorHeight });
    }
  });
  const outdoorAnchors = resolveScene(world.outdoorScene.targets, 0).map((t) => t.center);
  const terrain = world.outdoor.terrain;
  let guard = 0;
  while (out.length < count && guard++ < count * 40) {
    const u = rng.next();
    if (u < 0.55 && anchors.length > 0) {
      const a = anchors[Math.floor(rng.next() * anchors.length)]!;
      const d = Math.exp(Math.log(1.2) + rng.next() * (Math.log(14) - Math.log(1.2)));
      const th = rng.next() * Math.PI * 2;
      const z = a.floorZ + 0.7 + rng.next() * 1.2;
      const x = a.c[0] - Math.cos(th) * d;
      const y = a.c[1] - Math.sin(th) * d;
      if (!cellFree(world, a.building, x, y, z)) continue;
      const yaw = th + rng.gaussian() * 0.45;
      out.push({ view: fpvCameraFromDrone([x, y, z], yaw, FPV_ASPECT), kind: 'target' });
    } else if (u < 0.78) {
      const bi = Math.floor(rng.next() * world.buildings.length);
      const cells = free[bi]!;
      if (!cells.length) continue;
      const cell = cells[Math.floor(rng.next() * cells.length)]!;
      const b = world.buildings[bi]!;
      const z = b.shell.origin[2] + cell.floor * b.shell.spec.floorHeight + 0.7 + rng.next() * 1.2;
      const x = cell.x + (rng.next() - 0.5) * 0.4;
      const y = cell.y + (rng.next() - 0.5) * 0.4;
      if (!cellFree(world, bi, x, y, z)) continue;
      out.push({ view: fpvCameraFromDrone([x, y, z], rng.next() * Math.PI * 2, FPV_ASPECT), kind: 'room' });
    } else if (u < 0.9) {
      // Outdoors near a building or an outdoor target, FPV.
      const near = outdoorAnchors.length > 0 && rng.next() < 0.5
        ? outdoorAnchors[Math.floor(rng.next() * outdoorAnchors.length)]!
        : (() => {
            const b = world.buildings[Math.floor(rng.next() * world.buildings.length)]!;
            const xs = b.footprint.map((p) => p.x);
            const ys = b.footprint.map((p) => p.y);
            return [(Math.min(...xs) + Math.max(...xs)) / 2, (Math.min(...ys) + Math.max(...ys)) / 2, b.shell.origin[2]] as V3;
          })();
      const d = 3 + rng.next() * 16;
      const th = rng.next() * Math.PI * 2;
      const x = near[0] - Math.cos(th) * d;
      const y = near[1] - Math.sin(th) * d;
      if (world.buildings.some((b) => {
        const xs = b.footprint.map((p) => p.x);
        const ys = b.footprint.map((p) => p.y);
        return x > Math.min(...xs) - 0.5 && x < Math.max(...xs) + 0.5 && y > Math.min(...ys) - 0.5 && y < Math.max(...ys) + 0.5;
      })) continue;
      const z = sampleHeight(terrain, x, y) + 1 + rng.next() * 2.5;
      out.push({ view: fpvCameraFromDrone([x, y, z], th + rng.gaussian() * 0.4, FPV_ASPECT), kind: 'outdoor' });
    } else {
      // Nadir gimbal over an outdoor target or open ground.
      const c = outdoorAnchors.length > 0 && rng.next() < 0.7
        ? outdoorAnchors[Math.floor(rng.next() * outdoorAnchors.length)]!
        : ([world.home[0] + (rng.next() - 0.5) * 60, world.home[1] + (rng.next() - 0.5) * 60, 0] as V3);
      const agl = 4 + rng.next() * 18;
      const x = c[0] + (rng.next() - 0.5) * agl;
      const y = c[1] + (rng.next() - 0.5) * agl;
      const z = sampleHeight(terrain, x, y) + agl;
      const yaw = rng.next() * Math.PI * 2;
      const eye: V3 = [x, y, z - 0.08];
      out.push({ view: cameraLookingAt(eye, [x, y, z - 1.08], [Math.cos(yaw), Math.sin(yaw), 0], GIMBAL_FOV_Y, 4 / 3, 0.05, 120), kind: 'gimbal' });
    }
  }
  return out;
}

export interface DatasetFrame {
  seed: number;
  index: number;
  kind: SampledView['kind'];
  frame: Frame;
  labels: FrameLabel[];
  night: boolean;
}

/** Render `perWorld` frames from one training world. Palettes are randomized on most worlds. */
export function* worldFrames(seed: number, perWorld: number, poses: HumanoidPoses, randomize = true): Generator<DatasetFrame> {
  const world = buildTrainingWorld(seed);
  const rng = new RNG(seed >>> 0, 'vision-palette');
  const palette = randomize && rng.next() < 0.75 ? randomPalette(seed) : defaultPalette();
  const scene = new VisionScene(world, poses, { palette });
  const lighting = { timeOfDay: world.timeOfDay, flashlightOn: world.flashlightOn, sensorMode: world.sensorMode };
  const night = world.timeOfDay >= 20.5 || world.timeOfDay < 6.5;
  let index = 0;
  for (const sampled of sampleViews(world, perWorld, seed)) {
    const tSec = rng.next() * 30;
    const frame = scene.render(sampled.view, tSec, lighting, (seed * 7919 + index * 104729) >>> 0);
    yield { seed, index, kind: sampled.kind, frame, labels: frameLabels(frame), night };
    index++;
  }
}

import { cameraLookingAt, fpvCameraFromDrone, type CameraView } from '../../perception/camera';
import { detect, type DetectRequest, type Detection } from '../../perception/detector';
import { RESOLVE_DISTANCE } from '../../perception/model';
import { indoorCells, SeenEmptyTracker, type ClearReport, type GridCell } from '../../perception/seenEmpty';
import { CELL, SURFACE } from '../../shared/constants';
import { RNG } from '../../shared/rng';
import type { BuildingSpec } from '../../shared/types';
import { resolveScene } from '../../targets/motion';
import type { Aabb, ResolvedTarget } from '../../targets/types';
import { containsPoint } from '../../world/outdoor/geofence';
import { sampleHeight } from '../../world/outdoor/terrain';
import type { Opening, TreeInstance } from '../../world/outdoor/types';
import type {
  ClearScope,
  Contact,
  DronePose,
  LiveDetectionSource,
  MissionBuilding,
  MissionWorld,
  Observation,
  TruthTarget,
  V3,
} from './contracts';
import { CANOPY_CORE, plantingObstacles, type GroundBox } from './world';

/*
 * Camera model.
 * - Inside a shell, or below GIMBAL_MIN_AGL_M outdoors, the forward FPV camera
 *   (fpvCameraFromDrone: 110° vertical FOV, nose slightly down) is used.
 * - Outdoors at altitude a gimbal camera points straight down (nadir), image
 *   up along the nose. Its narrower lens on the same sensor resolves targets
 *   GIMBAL_RANGE_SCALE times farther than the FPV camera, so detection and
 *   seen-empty reach scale by it (DetectRequest.rangeScale).
 * The ground footprint of the gimbal is gimbalFootprint(agl); the report uses
 * the same function so coverage and perception agree.
 */

/** FPV vertical field of view, matching fpvCameraFromDrone. */
export const FPV_FOV_Y = (110 * Math.PI) / 180;
export const FPV_ASPECT = 4 / 3;
/** Gimbal vertical field of view (along the nose). */
export const GIMBAL_FOV_Y = (70 * Math.PI) / 180;
export const GIMBAL_ASPECT = 4 / 3;
/** Radians below the horizon. Nadir. */
export const GIMBAL_PITCH = Math.PI / 2;
/** Below this height above ground, outdoors, the FPV camera is used. */
export const GIMBAL_MIN_AGL_M = 4;
/** Pixel-density gain of the gimbal lens over the FPV lens. */
export const GIMBAL_RANGE_SCALE = Math.tan(FPV_FOV_Y / 2) / Math.tan(GIMBAL_FOV_Y / 2);
/** Seen-empty cells are sampled at chest height. */
export const CELL_SAMPLE_HEIGHT_M = 1.15;
/** Outdoor seen-empty cell edge. */
export const OUTDOOR_CELL_M = 1;

const TILE_M = 4;
const TREE_BIN_M = 8;

/**
 * Seen-empty maps refresh only after the camera moved or turned this much
 * since their last refresh. Frames in between overlap the last one and add
 * nothing, and skipping them keeps observe() cheap.
 */
export const SEEN_REFRESH = { indoorM: 0.4, outdoorM: 2.5, turnRad: 0.35 } as const;

interface Gate {
  pos: V3 | null;
  yaw: number;
  gimbal: boolean;
}

function passGate(gate: Gate, pos: V3, yaw: number, gimbal: boolean, meters: number): boolean {
  if (gate.pos) {
    const moved = Math.hypot(pos[0] - gate.pos[0], pos[1] - gate.pos[1], pos[2] - gate.pos[2]);
    const turned = Math.abs(Math.atan2(Math.sin(yaw - gate.yaw), Math.cos(yaw - gate.yaw)));
    if (moved < meters && turned < SEEN_REFRESH.turnRad && gimbal === gate.gimbal) return false;
  }
  gate.pos = [pos[0], pos[1], pos[2]];
  gate.yaw = yaw;
  gate.gimbal = gimbal;
  return true;
}

/**
 * Half extents of the gimbal's resolved ground rectangle, centered under the
 * drone: along the nose and across it. Clipped to the rectangle that stays
 * inside the resolve distance for the category. Zero when too high to resolve.
 */
export function gimbalFootprint(aglM: number, category: 'person' | 'object' = 'person'): { halfAlongM: number; halfAcrossM: number } {
  const h = aglM - CELL_SAMPLE_HEIGHT_M;
  const reach = RESOLVE_DISTANCE[category] * GIMBAL_RANGE_SCALE;
  if (h <= 0 || h >= reach) return { halfAlongM: 0, halfAcrossM: 0 };
  let along = h * Math.tan(GIMBAL_FOV_Y / 2);
  let across = along * GIMBAL_ASPECT;
  const ground = Math.sqrt(reach * reach - h * h);
  const corner = Math.hypot(along, across);
  if (corner > ground) {
    along *= ground / corner;
    across *= ground / corner;
  }
  return { halfAlongM: along, halfAcrossM: across };
}

/**
 * A detector that looks at pixels (src/vision). When set, it replaces the
 * analytic detector for contacts; seen-empty maps and truth stay as they are.
 */
export interface CameraContactSource {
  readonly id: string;
  /** Contacts from one frame of the world-frame camera `view`. `frame` counts observe() calls. */
  contacts(view: CameraView, pose: DronePose, tSec: number, frame: number): Contact[];
}

export interface PerceptionOptions {
  /** Relative 1-sigma error of the range estimate. Default 0.05. */
  rangeNoise?: number;
  /** False forces the FPV camera everywhere. Default true. */
  gimbal?: boolean;
  /** Pixel detector for contacts. Default: the analytic detector. */
  camera?: CameraContactSource;
}

interface BuildingView {
  index: number;
  building: MissionBuilding;
  origin: V3;
  viewSpec: BuildingSpec;
  /** Slabs, roof, and wall around openings, local frame. */
  occluders: Aabb[];
  box: { minX: number; minY: number; maxX: number; maxY: number; minZ: number; maxZ: number };
  roofOpen: boolean;
  keys: string[];
  person: SeenEmptyTracker;
  object: SeenEmptyTracker;
  gate: Gate;
}

interface Tile {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
  keys: string[];
  person: SeenEmptyTracker;
  object: SeenEmptyTracker;
}

interface TreeBoxes {
  tree: TreeInstance;
  canopy: Aabb | null;
  trunk: Aabb | null;
}

function shifted(view: CameraView, origin: V3): CameraView {
  return { ...view, position: [view.position[0] - origin[0], view.position[1] - origin[1], view.position[2] - origin[2]] };
}

function seeThrough(opening: Opening): boolean {
  if (opening.kind === 'closed_door' || opening.kind === 'vent') return false;
  if (opening.kind === 'ajar_door') return opening.angle >= 0.3;
  return true;
}

/** The shell as the camera sees it: exterior walls opaque (no glass), shut openings walled back up. */
export function viewSpecOf(building: MissionBuilding): BuildingSpec {
  const spec = building.shell.spec;
  const floors = spec.floors.map((floor) => {
    const cells = floor.cells.slice();
    const surfaces = floor.surfaces ? floor.surfaces.slice() : undefined;
    if (surfaces) {
      for (let iy = 0; iy < floor.d; iy++) {
        for (let ix = 0; ix < floor.w; ix++) {
          const border = ix === 0 || iy === 0 || ix === floor.w - 1 || iy === floor.d - 1;
          const i = iy * floor.w + ix;
          if (border && cells[i] === CELL.WALL && surfaces[i] === SURFACE.GLASS) surfaces[i] = SURFACE.WOOD;
        }
      }
    }
    return { ...floor, cells, surfaces };
  });
  for (const opening of building.shell.openings) {
    if (opening.kind === 'roof_hatch' || seeThrough(opening)) continue;
    const floor = floors[opening.floor];
    if (!floor) continue;
    for (const cell of opening.apertureCells) floor.cells[cell.z * floor.w + cell.x] = CELL.WALL;
  }
  return { ...spec, floors };
}

/** Floor slabs and the roof as boxes, with holes at shafts and at a passable roof hatch. */
export function slabBoxes(building: MissionBuilding): Aabb[] {
  const spec = building.shell.spec;
  const cs = spec.cellSize;
  const top = spec.floors.length;
  const boxes: Aabb[] = [];
  const hatch = building.shell.openings.find((opening) => opening.kind === 'roof_hatch' && opening.passable);
  for (let k = 1; k <= top; k++) {
    const below = spec.floors[k - 1]!;
    const above = spec.floors[k];
    const hole = (ix: number, iy: number): boolean => {
      if (k < top) return below.cells[iy * below.w + ix] === CELL.SHAFT && above?.cells[iy * above.w + ix] === CELL.SHAFT;
      return !!hatch && hatch.apertureCells.some((cell) => cell.x === ix && cell.z === iy);
    };
    const z = k * spec.floorHeight;
    let runStart = 0;
    let runSig = '';
    const flush = (sig: string, from: number, to: number): void => {
      if (!sig) return;
      for (const part of sig.split(';')) {
        const [x0, x1] = part.split('-').map(Number) as [number, number];
        boxes.push({ min: [x0 * cs, from * cs, z - 0.12], max: [(x1 + 1) * cs, to * cs, z + 0.04] });
      }
    };
    for (let iy = 0; iy < below.d; iy++) {
      const runs: string[] = [];
      let start = -1;
      for (let ix = 0; ix <= below.w; ix++) {
        const solid = ix < below.w && !hole(ix, iy);
        if (solid && start < 0) start = ix;
        if (!solid && start >= 0) {
          runs.push(`${start}-${ix - 1}`);
          start = -1;
        }
      }
      const sig = runs.join(';');
      if (iy === 0) {
        runSig = sig;
        runStart = 0;
      } else if (sig !== runSig) {
        flush(runSig, runStart, iy);
        runSig = sig;
        runStart = iy;
      }
    }
    flush(runSig, runStart, below.d);
  }
  return boxes;
}

/** Wall left around a see-through opening: beside, below the sill, and above the head. */
export function apertureBoxes(building: MissionBuilding): Aabb[] {
  const shell = building.shell;
  const spec = shell.spec;
  const cs = spec.cellSize;
  const boxes: Aabb[] = [];
  for (const opening of shell.openings) {
    if (opening.kind === 'roof_hatch' || !seeThrough(opening) || opening.apertureCells.length === 0) continue;
    const xs = opening.apertureCells.map((cell) => cell.x);
    const ys = opening.apertureCells.map((cell) => cell.z);
    const x0 = Math.min(...xs) * cs;
    const x1 = (Math.max(...xs) + 1) * cs;
    const y0 = Math.min(...ys) * cs;
    const y1 = (Math.max(...ys) + 1) * cs;
    const alongX = Math.abs(opening.tangent[0]) > 0.5;
    const c = alongX ? opening.center[0] - shell.origin[0] : opening.center[1] - shell.origin[1];
    const width = opening.kind === 'ajar_door' && opening.clearWidth > 0 ? opening.clearWidth : opening.width;
    const zFloor = opening.floor * spec.floorHeight;
    const zLo = zFloor + opening.sill;
    const zHi = Math.min(zFloor + spec.floorHeight, zLo + opening.height);
    const t0 = alongX ? x0 : y0;
    const t1 = alongX ? x1 : y1;
    const a = Math.max(t0, c - width / 2);
    const b = Math.min(t1, c + width / 2);
    const span = (lo: number, hi: number, zl: number, zh: number): void => {
      if (hi - lo < 0.01 || zh - zl < 0.01) return;
      boxes.push(alongX ? { min: [lo, y0, zl], max: [hi, y1, zh] } : { min: [x0, lo, zl], max: [x1, hi, zh] });
    };
    span(t0, a, zFloor, zFloor + spec.floorHeight);
    span(b, t1, zFloor, zFloor + spec.floorHeight);
    span(a, b, zFloor, zLo);
    span(a, b, zHi, zFloor + spec.floorHeight);
  }
  return boxes;
}

function buildingView(building: MissionBuilding, index: number): BuildingView {
  const shell = building.shell;
  const viewSpec = viewSpecOf(building);
  const cells = indoorCells(shell.spec);
  const xs = building.footprint.map((p) => p.x);
  const ys = building.footprint.map((p) => p.y);
  return {
    index,
    building,
    origin: [shell.origin[0], shell.origin[1], shell.origin[2]],
    viewSpec,
    occluders: [...slabBoxes(building), ...apertureBoxes(building)],
    box: {
      minX: Math.min(...xs),
      minY: Math.min(...ys),
      maxX: Math.max(...xs),
      maxY: Math.max(...ys),
      minZ: shell.origin[2],
      maxZ: shell.origin[2] + building.heightM,
    },
    roofOpen: shell.openings.some((opening) => opening.kind === 'roof_hatch' && opening.passable),
    keys: cells.map((cell) => cell.key),
    person: new SeenEmptyTracker(cells, 'person', viewSpec),
    object: new SeenEmptyTracker(cells, 'object', viewSpec),
    gate: { pos: null, yaw: 0, gimbal: false },
  };
}

function inBox(x: number, y: number, box: GroundBox): boolean {
  return x >= box.minX && x < box.maxX && y >= box.minY && y < box.maxY;
}

/** Geofence ground outside buildings, vegetation cores, and no-fly zones, cut into tiles. */
function outdoorTiles(world: MissionWorld): Tile[] {
  const outdoor = world.outdoor;
  const blocked = plantingObstacles(outdoor);
  const xs = outdoor.geofence.polygon.map(([x]) => x);
  const ys = outdoor.geofence.polygon.map(([, y]) => y);
  const minX = Math.floor(Math.min(...xs) / TILE_M) * TILE_M;
  const minY = Math.floor(Math.min(...ys) / TILE_M) * TILE_M;
  const maxX = Math.max(...xs);
  const maxY = Math.max(...ys);
  const tiles: Tile[] = [];
  const step = OUTDOOR_CELL_M;
  for (let ty = minY; ty < maxY; ty += TILE_M) {
    for (let tx = minX; tx < maxX; tx += TILE_M) {
      const cells: GridCell[] = [];
      for (let y = ty; y < ty + TILE_M; y += step) {
        for (let x = tx; x < tx + TILE_M; x += step) {
          const cx = x + step / 2;
          const cy = y + step / 2;
          if (!containsPoint(outdoor.geofence, cx, cy)) continue;
          if (blocked.some((box) => inBox(cx, cy, box))) continue;
          cells.push({
            key: `o:${Math.floor(x / step)}:${Math.floor(y / step)}`,
            center: [cx, cy, sampleHeight(outdoor.terrain, cx, cy) + CELL_SAMPLE_HEIGHT_M],
            floor: 0,
            roomIndex: -1,
          });
        }
      }
      if (cells.length === 0) continue;
      tiles.push({
        minX: tx,
        minY: ty,
        maxX: tx + TILE_M,
        maxY: ty + TILE_M,
        keys: cells.map((cell) => cell.key),
        person: new SeenEmptyTracker(cells, 'person'),
        object: new SeenEmptyTracker(cells, 'object'),
      });
    }
  }
  return tiles;
}

function treeBoxes(tree: TreeInstance): TreeBoxes {
  if (tree.kind === 'undergrowth') {
    const r = tree.canopyRadius * 0.8;
    return { tree, canopy: { min: [tree.x - r, tree.y - r, tree.z], max: [tree.x + r, tree.y + r, tree.z + tree.canopyRadius * 1.5] }, trunk: null };
  }
  const r = tree.canopyRadius * CANOPY_CORE;
  const cz = tree.z + tree.canopyZ;
  return {
    tree,
    canopy: { min: [tree.x - r, tree.y - r, cz - r], max: [tree.x + r, tree.y + r, cz + r] },
    trunk: {
      min: [tree.x - tree.trunkRadius, tree.y - tree.trunkRadius, tree.z],
      max: [tree.x + tree.trunkRadius, tree.y + tree.trunkRadius, tree.z + Math.min(tree.trunkHeight, tree.canopyZ)],
    },
  };
}

function mixSeed(seed: number, frame: number, scope: number): number {
  let h = Math.imul(seed ^ 0x9e3779b9, 0x85ebca6b) ^ Math.imul(frame + 1, 0xc2b2ae35) ^ Math.imul(scope + 7, 0x27d4eb2f);
  h ^= h >>> 15;
  h = Math.imul(h, 0x2c1b3c6d);
  h ^= h >>> 12;
  return h >>> 0;
}

function add3(a: V3, b: V3): V3 {
  return [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
}

export function bboxRay(view: CameraView, bbox: Detection['bbox']): V3 {
  const tanY = Math.tan(view.fovY / 2);
  const tanX = tanY * Math.max(0.05, view.aspect);
  const ndcX = (bbox.x + bbox.w / 2 - 0.5) * 2;
  const ndcY = (0.5 - (bbox.y + bbox.h / 2)) * 2;
  const d: V3 = [
    view.forward[0] + view.right[0] * ndcX * tanX + view.up[0] * ndcY * tanY,
    view.forward[1] + view.right[1] * ndcX * tanX + view.up[1] * ndcY * tanY,
    view.forward[2] + view.right[2] * ndcX * tanX + view.up[2] * ndcY * tanY,
  ];
  const len = Math.hypot(d[0], d[1], d[2]) || 1;
  return [d[0] / len, d[1] / len, d[2] / len];
}

function boxDistance(p: V3, box: BuildingView['box']): number {
  const dx = Math.max(box.minX - p[0], 0, p[0] - box.maxX);
  const dy = Math.max(box.minY - p[1], 0, p[1] - box.maxY);
  const dz = Math.max(box.minZ - p[2], 0, p[2] - box.maxZ);
  return Math.hypot(dx, dy, dz);
}

function sumReports(reports: ClearReport[]): ClearReport {
  let seenEmpty = 0;
  let occupied = 0;
  let unseen = 0;
  let total = 0;
  for (const report of reports) {
    seenEmpty += report.seenEmpty;
    occupied += report.occupied;
    unseen += report.unseen;
    total += report.total;
  }
  const status = occupied > 0 ? 'occupied' : unseen > 0 || total === 0 ? 'incomplete' : 'clear';
  return { status, seenEmpty, occupied, unseen, total };
}

/**
 * Real camera perception over a MissionWorld. Each observe() is one frame of
 * the simulated detector (src/perception), occluded by shell walls, slabs,
 * the roof, trees, and buildings, and lit by the world's lighting. The same
 * world seed and frame sequence always return the same contacts.
 */
export function createPerceptionSource(world: MissionWorld, opts: PerceptionOptions = {}): LiveDetectionSource {
  const rangeNoise = opts.rangeNoise ?? 0.05;
  const useGimbal = opts.gimbal ?? true;
  const views = world.buildings.map((building, index) => buildingView(building, index));
  const tiles = outdoorTiles(world);
  const tileIndex = new Map<string, Tile>();
  for (const tile of tiles) tileIndex.set(`${tile.minX / TILE_M}:${tile.minY / TILE_M}`, tile);
  const trees = new Map<string, TreeBoxes[]>();
  for (const tree of world.outdoor.trees) {
    const key = `${Math.floor(tree.x / TREE_BIN_M)}:${Math.floor(tree.y / TREE_BIN_M)}`;
    const list = trees.get(key) ?? [];
    list.push(treeBoxes(tree));
    trees.set(key, list);
  }
  const buildingBoxes: Aabb[] = views.map((view) => ({
    min: [view.box.minX, view.box.minY, view.box.minZ],
    max: [view.box.maxX, view.box.maxY, view.box.maxZ],
  }));
  let frame = 0;
  const outdoorGate: Gate = { pos: null, yaw: 0, gimbal: false };
  let lastT = 0;

  const lightingFor = (lights: ArrayLike<number>): DetectRequest['lighting'] => ({
    timeOfDay: world.timeOfDay,
    interiorLightsOn: lights,
    flashlightOn: world.flashlightOn,
    sensorMode: world.sensorMode,
  });
  const noLights = new Uint8Array(0);

  const insideOf = (p: V3): BuildingView | null =>
    views.find((view) => p[0] >= view.box.minX && p[0] <= view.box.maxX && p[1] >= view.box.minY && p[1] <= view.box.maxY && p[2] >= view.box.minZ - 0.2 && p[2] <= view.box.maxZ) ?? null;

  const cameraFor = (pose: DronePose): { view: CameraView; gimbal: boolean; agl: number } => {
    const pos = pose.pos;
    const agl = pos[2] - sampleHeight(world.outdoor.terrain, pos[0], pos[1]);
    if (useGimbal && agl >= GIMBAL_MIN_AGL_M && !insideOf(pos)) {
      const eye: V3 = [pos[0], pos[1], pos[2] - 0.08];
      const look: V3 = [eye[0], eye[1], eye[2] - 1];
      const nose: V3 = [Math.cos(pose.yaw), Math.sin(pose.yaw), 0];
      return { view: cameraLookingAt(eye, look, nose, GIMBAL_FOV_Y, GIMBAL_ASPECT, 0.05, 120), gimbal: true, agl };
    }
    return { view: fpvCameraFromDrone([pos[0], pos[1], pos[2]], pose.yaw, FPV_ASPECT), gimbal: false, agl };
  };

  /** Ground rectangle the camera can resolve, as an AABB. */
  const groundReach = (view: CameraView, gimbal: boolean, agl: number, yaw: number, reach: number): GroundBox => {
    const [px, py] = view.position;
    if (gimbal) {
      const fp = gimbalFootprint(agl, 'person');
      const c = Math.abs(Math.cos(yaw));
      const s = Math.abs(Math.sin(yaw));
      const hx = fp.halfAlongM * c + fp.halfAcrossM * s + 1;
      const hy = fp.halfAlongM * s + fp.halfAcrossM * c + 1;
      return { minX: px - hx, minY: py - hy, maxX: px + hx, maxY: py + hy };
    }
    return { minX: px - reach, minY: py - reach, maxX: px + reach, maxY: py + reach };
  };

  const treesIn = (box: GroundBox, trunks: boolean): Aabb[] => {
    const out: Aabb[] = [];
    for (let by = Math.floor((box.minY - 3) / TREE_BIN_M); by <= Math.floor((box.maxY + 3) / TREE_BIN_M); by++) {
      for (let bx = Math.floor((box.minX - 3) / TREE_BIN_M); bx <= Math.floor((box.maxX + 3) / TREE_BIN_M); bx++) {
        for (const item of trees.get(`${bx}:${by}`) ?? []) {
          const t = item.tree;
          if (t.x < box.minX - 3 || t.x > box.maxX + 3 || t.y < box.minY - 3 || t.y > box.maxY + 3) continue;
          if (item.canopy && (trunks || item.tree.kind !== 'undergrowth')) out.push(item.canopy);
          if (trunks && item.trunk) out.push(item.trunk);
        }
      }
    }
    return out;
  };

  const toContacts = (
    detections: Detection[],
    view: CameraView,
    origin: V3,
    buildingId: string | null,
    seed: number,
    tSec: number,
  ): Contact[] => {
    const eye = add3(view.position, origin);
    return detections.map((det) => {
      const ray = bboxRay(view, det.bbox);
      const noise = new RNG(seed, `range:${det.id}`).gaussian();
      const range = Math.max(0.3, det.distance * (1 + rangeNoise * noise));
      return {
        id: `f${frame}:${det.id}`,
        tSec,
        classId: det.classId,
        confidence: det.confidence,
        estimate: [eye[0] + ray[0] * range, eye[1] + ray[1] * range, eye[2] + ray[2] * range],
        range,
        bearing: ray,
        buildingId,
        truth: { targetId: det.targetId, trueClass: det.trueClass, kind: det.kind },
      };
    });
  };

  const resolvedIndoor = (view: BuildingView, tSec: number): ResolvedTarget[] => resolveScene(view.building.scene.targets, tSec);

  const camera = opts.camera ?? null;

  const observeAnalytic = (pose: DronePose, tSec: number, view: CameraView, gimbal: boolean, agl: number): Observation => {
    const rangeScale = gimbal ? GIMBAL_RANGE_SCALE : 1;
    const reach = RESOLVE_DISTANCE.person * rangeScale;
    const cameraSpeed = Math.hypot(pose.vel[0], pose.vel[1], pose.vel[2]);
    const contacts: Contact[] = [];
    const camPos = view.position;

    for (const b of views) {
      if (boxDistance(camPos, b.box) > reach) continue;
      if (camPos[2] > b.box.maxZ + 0.5 && !b.roofOpen) continue;
      const local = shifted(view, b.origin);
      const lighting = lightingFor(b.building.interiorLightsOn);
      const resolved = resolvedIndoor(b, tSec);
      const seed = mixSeed(world.seed, frame, b.index + 1);
      if (resolved.length > 0 && !camera) {
        const targets = resolved.map((target) => ({ ...target, occluders: [...target.occluders, ...b.occluders] }));
        const found = detect({ seed, camera: local, targets, spec: b.viewSpec, lighting, cameraSpeed, rangeScale });
        contacts.push(...toContacts(found, local, b.origin, b.building.id, seed, tSec));
      }
      if (passGate(b.gate, camPos, pose.yaw, gimbal, SEEN_REFRESH.indoorM)) {
        const cellOccluders = [...b.occluders, ...resolved.flatMap((target) => target.occluders)];
        b.person.observe(local, resolved, lighting, cellOccluders, rangeScale);
        b.object.observe(local, resolved, lighting, cellOccluders, rangeScale);
      }
    }

    // Inside a shell the walls hide the woods; the outdoor pass is skipped.
    if (insideOf(camPos)) {
      contacts.sort((a, b) => b.confidence - a.confidence || (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
      return { tSec, contacts };
    }
    const ground = groundReach(view, gimbal, agl, pose.yaw, reach);
    const blockers = [...treesIn(ground, !gimbal), ...buildingBoxes.filter((box) =>
      box.max[0] > ground.minX - 3 && box.min[0] < ground.maxX + 3 && box.max[1] > ground.minY - 3 && box.min[1] < ground.maxY + 3)];
    const outdoorTargets = resolveScene(world.outdoorScene.targets, tSec);
    const near = outdoorTargets.filter((target) => target.center[0] >= ground.minX - 2 && target.center[0] <= ground.maxX + 2 && target.center[1] >= ground.minY - 2 && target.center[1] <= ground.maxY + 2);
    const lighting = lightingFor(noLights);
    if (near.length > 0 && !camera) {
      const seed = mixSeed(world.seed, frame, 0);
      const targets = near.map((target) => ({ ...target, occluders: [...target.occluders, ...blockers] }));
      const found = detect({ seed, camera: view, targets, spec: null, lighting, cameraSpeed, rangeScale });
      contacts.push(...toContacts(found, view, [0, 0, 0], null, seed, tSec));
    }
    if (!passGate(outdoorGate, camPos, pose.yaw, gimbal, SEEN_REFRESH.outdoorM)) {
      contacts.sort((a, b) => b.confidence - a.confidence || (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
      return { tSec, contacts };
    }
    const fwdX = view.forward[0];
    const fwdY = view.forward[1];
    for (let ty = Math.floor(ground.minY / TILE_M); ty <= Math.floor(ground.maxY / TILE_M); ty++) {
      for (let tx = Math.floor(ground.minX / TILE_M); tx <= Math.floor(ground.maxX / TILE_M); tx++) {
        const tile = tileIndex.get(`${tx}:${ty}`);
        if (!tile) continue;
        if (!gimbal) {
          // Skip tiles wholly behind the FPV camera.
          const cx = (tile.minX + tile.maxX) / 2 - camPos[0];
          const cy = (tile.minY + tile.maxY) / 2 - camPos[1];
          if (cx * fwdX + cy * fwdY < -TILE_M) continue;
        }
        tile.person.observe(view, outdoorTargets, lighting, blockers, rangeScale);
        tile.object.observe(view, outdoorTargets, lighting, blockers, rangeScale);
      }
    }
    contacts.sort((a, b) => b.confidence - a.confidence || (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
    return { tSec, contacts };
  };

  return {
    id: camera ? `${camera.id}-${world.seed}` : `perception-${world.seed}`,
    observe(pose: DronePose, tSec: number): Observation {
      frame += 1;
      lastT = tSec;
      const { view, gimbal, agl } = cameraFor(pose);
      const observation = observeAnalytic(pose, tSec, view, gimbal, agl);
      if (!camera) return observation;
      const contacts = camera.contacts(view, pose, tSec, frame);
      contacts.sort((a, b) => b.confidence - a.confidence || (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
      return { tSec, contacts };
    },
    clearStatus(scope: ClearScope, category: 'person' | 'object'): ClearReport {
      if (scope === 'outdoor') {
        const targets = resolveScene(world.outdoorScene.targets, lastT);
        return sumReports(tiles.map((tile) => (category === 'person' ? tile.person : tile.object).report(tile.keys, targets)));
      }
      const view = views.find((item) => item.building.id === scope);
      if (!view) return { status: 'incomplete', seenEmpty: 0, occupied: 0, unseen: 0, total: 0 };
      const tracker = category === 'person' ? view.person : view.object;
      return tracker.report(view.keys, resolvedIndoor(view, lastT));
    },
    truthTargets(): TruthTarget[] {
      const out: TruthTarget[] = [];
      for (const view of views) {
        for (const target of resolvedIndoor(view, lastT)) {
          out.push({ id: target.id, classId: target.classId, category: target.category, buildingId: view.building.id, world: add3(target.center, view.origin) });
        }
      }
      for (const target of resolveScene(world.outdoorScene.targets, lastT)) {
        out.push({ id: target.id, classId: target.classId, category: target.category, buildingId: null, world: [...target.center] });
      }
      return out;
    },
  };
}

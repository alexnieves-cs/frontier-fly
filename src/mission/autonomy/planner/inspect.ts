import { fpvCameraFromDrone } from '../../../perception/camera';
import type { OutdoorPhysics } from '../../../world/outdoor/physics';
import type { Opening } from '../../../world/outdoor/types';
import type { MissionBuilding, V3 } from '../contracts';
import { VIEW_HEIGHT } from './approach';
import { type Airspace, buildingBox, dot, legBetween, operable, pointFree, sub } from './geometry';

/**
 * Envelope inspection. An orbit of viewpoints at standoff around the shell,
 * at opening height, each looking square at the face and then toward its
 * middle, plus two looks across the roof for hatches. Openings
 * count as observed only when the camera actually saw them: inside the FOV,
 * within range, facing the camera, and with a clear ray.
 */

export const CAMERA_ASPECT = 4 / 3;
/** Openings further than this are not resolved well enough to measure. */
export const INSPECT_RANGE = 9;
/** Extra clearance, m, a straight leg between inspection viewpoints keeps from walls and corners. */
export const INSPECT_LEG_MARGIN = 0.25;

export interface InspectStep {
  pos: V3;
  /** Heading to hold at this step. Null faces the direction of travel. */
  yaw: number | null;
  /** Hover time at the step, seconds. */
  dwell: number;
}

interface Viewpoint {
  pos: V3;
  yaw: number;
  /** Second heading toward the middle of the face, so openings near it fall inside the FOV. */
  midYaw: number | null;
  angle: number;
}

function faceViewpoints(building: MissionBuilding, physics: OutdoorPhysics, air: Airspace): Viewpoint[] {
  const box = buildingBox(building);
  const z = box.baseZ + VIEW_HEIGHT;
  const out: Viewpoint[] = [];
  const faces: { at: (f: number, s: number) => [number, number]; yaw: number }[] = [
    { at: (f, s) => [box.minX + (box.maxX - box.minX) * f, box.minY - s], yaw: Math.PI / 2 },
    { at: (f, s) => [box.maxX + s, box.minY + (box.maxY - box.minY) * f], yaw: Math.PI },
    { at: (f, s) => [box.maxX - (box.maxX - box.minX) * f, box.maxY + s], yaw: -Math.PI / 2 },
    { at: (f, s) => [box.minX - s, box.maxY - (box.maxY - box.minY) * f], yaw: 0 },
  ];
  for (const face of faces) {
    for (const f of [0.25, 0.75]) {
      for (const s of [3, 2.2, 4.5]) {
        const [x, y] = face.at(f, s);
        const pos: V3 = [x, y, z];
        if (!operable(air, x, y, 1) || !pointFree(physics, pos, 0.5)) continue;
        const [mx, my] = face.at(0.5, 0);
        out.push({ pos, yaw: face.yaw, midYaw: Math.atan2(my - y, mx - x), angle: Math.atan2(y - box.cy, x - box.cx) });
        break;
      }
    }
  }
  return out;
}

function roofViewpoints(building: MissionBuilding, physics: OutdoorPhysics): Viewpoint[] {
  const box = buildingBox(building);
  const z = box.roofZ + 3;
  const out: Viewpoint[] = [];
  for (const [x, y] of [
    [box.cx, box.minY - 0.8],
    [box.cx, box.maxY + 0.8],
  ] as const) {
    const pos: V3 = [x, y, z];
    if (!pointFree(physics, pos, 0.5)) continue;
    out.push({ pos, yaw: Math.atan2(box.cy - y, box.cx - x), midYaw: null, angle: 0 });
  }
  return out;
}

/**
 * Ordered inspection steps from `start`, with hops over the roof line where trees block.
 *
 * With `backtrack`, a viewpoint with no clear leg from the current one is reached by
 * retracing the route flown so far to an earlier viewpoint that has one. Without it such a
 * viewpoint is skipped, so the number of faces seen depends on where the orbit started.
 */
export function planInspection(
  building: MissionBuilding,
  start: V3,
  physics: OutdoorPhysics,
  air: Airspace,
  backtrack = false,
): InspectStep[] {
  const box = buildingBox(building);
  const startAngle = Math.atan2(start[1] - box.cy, start[0] - box.cx);
  const tau = Math.PI * 2;
  const faces = faceViewpoints(building, physics, air).sort((a, b) => {
    const da = (((a.angle - startAngle) % tau) + tau) % tau;
    const db = (((b.angle - startAngle) % tau) + tau) % tau;
    return da - db;
  });
  const views = [...faces, ...roofViewpoints(building, physics)];
  const steps: InspectStep[] = [];
  let cur = start;
  const hopZ = box.roofZ + 2.5;
  // Positions flown so far, and the index in it of each viewpoint reached.
  const route: V3[] = [start];
  const anchors: number[] = [0];
  for (const view of views) {
    // Viewpoint to viewpoint legs pass building corners; keep a gust's worth of air off them.
    let leg = legBetween(physics, cur, view.pos, hopZ, INSPECT_LEG_MARGIN);
    if (!leg && backtrack) {
      for (let a = anchors.length - 2; a >= 0 && !leg; a--) {
        const from = route[anchors[a]!]!;
        const onward = legBetween(physics, from, view.pos, hopZ, INSPECT_LEG_MARGIN);
        if (!onward) continue;
        const back = route.slice(anchors[a]!, route.length - 1).reverse();
        for (const p of back) steps.push({ pos: p, yaw: null, dwell: 0 });
        route.push(...back);
        leg = onward;
      }
    }
    if (!leg) continue;
    route.push(...leg);
    anchors.push(route.length - 1);
    for (let i = 0; i < leg.length - 1; i++) steps.push({ pos: leg[i]!, yaw: null, dwell: 0 });
    steps.push({ pos: view.pos, yaw: view.yaw, dwell: 0.8 });
    if (view.midYaw !== null) steps.push({ pos: view.pos, yaw: view.midYaw, dwell: 0.8 });
    cur = view.pos;
  }
  return steps;
}

/** Openings of `building` the camera sees from this pose. */
export function observeOpenings(pos: V3, yaw: number, building: MissionBuilding, physics: OutdoorPhysics): Opening[] {
  const cam = fpvCameraFromDrone(pos, yaw, CAMERA_ASPECT);
  const tanY = Math.tan(cam.fovY / 2);
  const tanX = tanY * cam.aspect;
  const seen: Opening[] = [];
  for (const opening of building.shell.openings) {
    const c = opening.center as V3;
    const d = sub(c, cam.position);
    const range = Math.hypot(d[0], d[1], d[2]);
    if (range > INSPECT_RANGE || range < 0.3) continue;
    const z = dot(d, cam.forward);
    if (z <= cam.near) continue;
    if (Math.abs(dot(d, cam.right) / z) > tanX * 0.95 || Math.abs(dot(d, cam.up) / z) > tanY * 0.95) continue;
    // The face must point at the camera.
    if (-dot(d, opening.normal as V3) / range < 0.25) continue;
    const hit = physics.cast(cam.position[0], cam.position[1], cam.position[2], d[0], d[1], d[2], range);
    if (hit && hit.dist < range - 0.6) continue;
    seen.push(opening);
  }
  return seen;
}

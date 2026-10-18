import type { Aabb, Vec3 } from '../targets/types';
import { clamp, cross, dot, len, norm, sub } from './vec';

export interface CameraView {
  position: Vec3;
  forward: Vec3;
  up: Vec3;
  right: Vec3;
  /** Vertical field of view, radians. */
  fovY: number;
  aspect: number;
  near: number;
  far: number;
}

export interface ProjectedBox {
  bbox: { x: number; y: number; w: number; h: number };
  area: number;
}

export function cameraLookingAt(eye: Vec3, look: Vec3, upHint: Vec3, fovY: number, aspect: number, near = 0.05, far = 80): CameraView {
  const forward = norm(sub(look, eye));
  let right = cross(forward, upHint);
  if (len(right) < 1e-6) right = cross(forward, [0, 1, 0]);
  right = norm(right);
  const up = norm(cross(right, forward));
  return { position: eye, forward, up, right, fovY, aspect, near, far };
}

/** Drone-mounted camera. Matches the FPV pose: 110° and a slight downward tilt. */
export function fpvCameraFromDrone(pos: Vec3, yaw: number, aspect: number): CameraView {
  const noseX = Math.cos(yaw);
  const noseY = Math.sin(yaw);
  const eye: Vec3 = [pos[0] + noseX * 0.1, pos[1] + noseY * 0.1, pos[2] + 0.02];
  const look: Vec3 = [pos[0] + noseX * 3, pos[1] + noseY * 3, pos[2] - 0.55];
  return cameraLookingAt(eye, look, [0, 0, 1], (110 * Math.PI) / 180, aspect);
}

interface CamPoint {
  x: number;
  y: number;
  z: number;
}

function toCamera(cam: CameraView, p: Vec3): CamPoint {
  const d = sub(p, cam.position);
  return { x: dot(d, cam.right), y: dot(d, cam.up), z: dot(d, cam.forward) };
}

function projectCam(cam: CameraView, p: CamPoint): { x: number; y: number } | null {
  if (p.z <= cam.near) return null;
  const tanY = Math.tan(cam.fovY / 2);
  const tanX = tanY * Math.max(0.05, cam.aspect);
  const ndcX = p.x / p.z / tanX;
  const ndcY = p.y / p.z / tanY;
  return { x: ndcX * 0.5 + 0.5, y: 0.5 - ndcY * 0.5 };
}

function clipEdge(cam: CameraView, a: CamPoint, b: CamPoint): CamPoint | null {
  if ((a.z <= cam.near && b.z <= cam.near) || (a.z > cam.near && b.z > cam.near)) return null;
  const t = (cam.near - a.z) / (b.z - a.z);
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, z: cam.near + 1e-4 };
}

/** Project an axis-aligned box into normalized image coordinates, origin top-left. */
export function projectAabb(cam: CameraView, aabb: Aabb): ProjectedBox | null {
  const corners: Vec3[] = [];
  for (const x of [aabb.min[0], aabb.max[0]]) {
    for (const y of [aabb.min[1], aabb.max[1]]) {
      for (const z of [aabb.min[2], aabb.max[2]]) corners.push([x, y, z]);
    }
  }
  const camPts = corners.map((corner) => toCamera(cam, corner));
  const projected: Array<{ x: number; y: number }> = [];
  for (const point of camPts) {
    const p = projectCam(cam, point);
    if (p) projected.push(p);
  }
  for (let i = 0; i < camPts.length; i++) {
    for (let j = i + 1; j < camPts.length; j++) {
      const mid = clipEdge(cam, camPts[i]!, camPts[j]!);
      if (!mid) continue;
      const p = projectCam(cam, mid);
      if (p) projected.push(p);
    }
  }
  if (!projected.length) return null;
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const p of projected) {
    if (p.x < minX) minX = p.x;
    if (p.y < minY) minY = p.y;
    if (p.x > maxX) maxX = p.x;
    if (p.y > maxY) maxY = p.y;
  }
  const x0 = clamp(minX, 0, 1);
  const y0 = clamp(minY, 0, 1);
  const x1 = clamp(maxX, 0, 1);
  const y1 = clamp(maxY, 0, 1);
  const w = x1 - x0;
  const h = y1 - y0;
  if (w < 1e-4 || h < 1e-4) return null;
  return { bbox: { x: x0, y: y0, w, h }, area: w * h };
}

export function inFront(cam: CameraView, point: Vec3): boolean {
  return dot(sub(point, cam.position), cam.forward) > cam.near;
}

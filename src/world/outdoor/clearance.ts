import { DOOR_THICKNESS, PASS_RADIUS, REQUIRED_CLEARANCE } from './constants';
import { add, cross, norm, quatFromBasis, scale, type Vec3 } from './math';
import type { Opening, OpeningKind } from './types';

export interface Clearance {
  clearWidth: number;
  clearHeight: number;
  passable: boolean;
}

export interface DoorPose {
  center: Vec3;
  axisX: Vec3;
  axisY: Vec3;
  axisZ: Vec3;
  hx: number;
  hy: number;
  hz: number;
  quat: { x: number; y: number; z: number; w: number };
}

const UP: Vec3 = [0, 0, 1];

export function isDoor(kind: OpeningKind): boolean {
  return kind === 'ajar_door' || kind === 'closed_door';
}

/**
 * Largest sphere that can travel from outside to inside through the aperture.
 * Doors use the swung leaf, not the nominal frame width.
 */
export function measureClearance(opening: Pick<Opening, 'kind' | 'width' | 'height' | 'angle'>): Clearance {
  const { kind, width, height } = opening;
  const angle = isDoor(kind) ? Math.max(0, Math.min(Math.PI / 2, opening.angle)) : 0;
  if (!isDoor(kind)) {
    const passable = width >= REQUIRED_CLEARANCE - 1e-6 && height >= REQUIRED_CLEARANCE - 1e-6;
    return { clearWidth: width, clearHeight: height, passable };
  }
  if (angle < 1e-3) return { clearWidth: 0, clearHeight: 0, passable: false };
  let lo = 0;
  let hi = Math.min(width, height) / 2;
  for (let i = 0; i < 16; i++) {
    const mid = (lo + hi) / 2;
    if (sphereClearsDoor(width, height, angle, mid)) lo = mid;
    else hi = mid;
  }
  const clearWidth = lo * 2;
  const passable = lo >= PASS_RADIUS - 1e-4 && height >= REQUIRED_CLEARANCE - 1e-6;
  return { clearWidth, clearHeight: passable || lo > 0.02 ? height : 0, passable };
}

export function doorPose(opening: Pick<Opening, 'kind' | 'center' | 'normal' | 'tangent' | 'width' | 'height' | 'angle'>): DoorPose | null {
  if (!isDoor(opening.kind)) return null;
  const angle = Math.max(0, Math.min(Math.PI / 2, opening.angle));
  const tangent = norm(opening.tangent);
  const normal = norm(opening.normal);
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  const axisX = norm([
    cos * tangent[0] - sin * normal[0],
    cos * tangent[1] - sin * normal[1],
    cos * tangent[2] - sin * normal[2],
  ]);
  const hinge = add(opening.center, scale(tangent, -opening.width * 0.5));
  const center = add(hinge, scale(axisX, opening.width * 0.5));
  const axisZ = UP;
  const axisY = norm(cross(axisZ, axisX));
  const quat = quatFromBasis(axisX, axisY, axisZ);
  return {
    center,
    axisX,
    axisY,
    axisZ,
    hx: opening.width * 0.5,
    hy: DOOR_THICKNESS * 0.5,
    hz: opening.height * 0.5,
    quat,
  };
}

function sphereClearsDoor(width: number, height: number, angle: number, radius: number): boolean {
  if (height < radius * 2 - 1e-6 || width < radius * 2 - 1e-6) return false;
  const u0 = -width / 2 + radius;
  const u1 = width / 2 - radius;
  if (u1 < u0 - 1e-8) return false;
  const samples = 7;
  for (let i = 0; i < samples; i++) {
    const u = u0 + ((u1 - u0) * i) / (samples - 1);
    if (pathClear(width, height, angle, u, radius)) return true;
  }
  return false;
}

function pathClear(width: number, height: number, angle: number, u: number, radius: number): boolean {
  const pose = doorPose({
    kind: 'ajar_door',
    center: [0, 0, 0],
    normal: [0, 1, 0],
    tangent: [1, 0, 0],
    width,
    height,
    angle,
  });
  if (!pose) return false;
  for (let v = 0.85; v >= -0.85; v -= 0.05) {
    if (hitsDoor(pose, u, v, radius)) return false;
  }
  return true;
}

function hitsDoor(pose: DoorPose, u: number, v: number, radius: number): boolean {
  const px = u - pose.center[0];
  const py = v - pose.center[1];
  const pz = -pose.center[2];
  const lx = px * pose.axisX[0] + py * pose.axisX[1] + pz * pose.axisX[2];
  const ly = px * pose.axisY[0] + py * pose.axisY[1] + pz * pose.axisY[2];
  const lz = px * pose.axisZ[0] + py * pose.axisZ[1] + pz * pose.axisZ[2];
  const dx = Math.max(Math.abs(lx) - pose.hx, 0);
  const dy = Math.max(Math.abs(ly) - pose.hy, 0);
  const dz = Math.max(Math.abs(lz) - pose.hz, 0);
  const limit = Math.max(0, radius - 1e-4);
  return dx * dx + dy * dy + dz * dz < limit * limit;
}

export function sphereHitsObb(px: number, py: number, pz: number, radius: number, pose: DoorPose): boolean {
  const dx0 = px - pose.center[0];
  const dy0 = py - pose.center[1];
  const dz0 = pz - pose.center[2];
  const lx = dx0 * pose.axisX[0] + dy0 * pose.axisX[1] + dz0 * pose.axisX[2];
  const ly = dx0 * pose.axisY[0] + dy0 * pose.axisY[1] + dz0 * pose.axisY[2];
  const lz = dx0 * pose.axisZ[0] + dy0 * pose.axisZ[1] + dz0 * pose.axisZ[2];
  const dx = Math.max(Math.abs(lx) - pose.hx, 0);
  const dy = Math.max(Math.abs(ly) - pose.hy, 0);
  const dz = Math.max(Math.abs(lz) - pose.hz, 0);
  return dx * dx + dy * dy + dz * dz < radius * radius;
}

import type { PersonState } from '../targets/types';

/** Body part per vertex, as baked: head, body (torso), arm, leg. */
export const PART = { head: 0, body: 1, arm: 2, leg: 3 } as const;

/**
 * The CC0 KayKit humanoid as posed triangle soups (scripts/vision/bake-humanoid.mjs).
 * Positions are sim frame, meters, feet at the origin, facing +x at yaw 0.
 */
export interface HumanoidPoses {
  names: string[];
  verts: number;
  indices: Uint16Array;
  parts: Uint8Array;
  /** poses x verts x 3, meters. */
  positions: Float32Array;
  walkFrames: number;
}

/** Walk-cycle length of the Walking_A clip, seconds. */
export const WALK_CYCLE_SEC = 1.04;

export function parseHumanoidPoses(buffer: ArrayBuffer): HumanoidPoses {
  const view = new DataView(buffer);
  const headLen = view.getUint32(0, true);
  const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 4, headLen)).trim()) as {
    poses: string[];
    verts: number;
    tris: number;
  };
  const pad = (n: number): number => (4 - (n % 4)) % 4;
  let off = 4 + headLen;
  const indices = new Uint16Array(buffer.slice(off, off + header.tris * 6));
  off += header.tris * 6;
  off += pad(header.tris * 6);
  const parts = new Uint8Array(buffer.slice(off, off + header.verts));
  off += header.verts;
  off += pad(header.verts);
  const raw = new Int16Array(buffer.slice(off, off + header.poses.length * header.verts * 6));
  const positions = new Float32Array(raw.length);
  for (let i = 0; i < raw.length; i++) positions[i] = raw[i]! / 1000;
  return {
    names: header.poses,
    verts: header.verts,
    indices,
    parts,
    positions,
    walkFrames: header.poses.filter((name) => name.startsWith('walking')).length,
  };
}

function phaseOf(id: string): number {
  // Same per-target phase as TargetView, so the camera and the 3D view walk in step.
  let h = 0;
  for (let i = 0; i < id.length; i++) h = (h * 33 + id.charCodeAt(i)) >>> 0;
  return (h % 1000) / 700;
}

/** Pose index for a person state at a time. */
export function poseIndex(poses: HumanoidPoses, state: PersonState | 'static', id: string, tSec: number): number {
  const name = state === 'static' ? 'standing' : state;
  if (name !== 'walking') return Math.max(0, poses.names.indexOf(name));
  const cycle = (((tSec + phaseOf(id)) % WALK_CYCLE_SEC) + WALK_CYCLE_SEC) % WALK_CYCLE_SEC;
  const k = Math.floor((cycle / WALK_CYCLE_SEC) * poses.walkFrames) % poses.walkFrames;
  return Math.max(0, poses.names.indexOf(`walking${k}`));
}

import type { PersonState, PlacedTarget, Vec3 } from './types';
import { followPath } from './motion';

/**
 * Joint names follow the KayKit Rig_Medium skeleton so a later animation
 * binding can share them. Velocities are finite differences of this pose.
 *
 * Future safety hook: a later feature can read these joint velocities so the
 * drone dodges a person who steps into its path. This module does not steer
 * the drone and does not plan contact. The only intended consumer is a
 * drone-only avoidance response.
 */
export const JOINT_NAMES = [
  'hips',
  'spine',
  'chest',
  'neck',
  'head',
  'upperarm.l',
  'lowerarm.l',
  'hand.l',
  'upperarm.r',
  'lowerarm.r',
  'hand.r',
  'upperleg.l',
  'lowerleg.l',
  'foot.l',
  'upperleg.r',
  'lowerleg.r',
  'foot.r',
] as const;

export type JointName = (typeof JOINT_NAMES)[number];

export interface JointSample {
  name: JointName;
  position: Vec3;
  velocity: Vec3;
}

export interface PersonMotionSample {
  personId: string;
  time: number;
  joints: JointSample[];
}

function pose(state: PersonState, time: number): Record<JointName, Vec3> {
  const sway = Math.sin(time * 2) * 0.008;
  const p: Record<JointName, Vec3> = {
    hips: [0, 0, 0.95 + sway],
    spine: [0, 0, 1.12 + sway],
    chest: [0, 0, 1.32 + sway],
    neck: [0, 0, 1.48 + sway],
    head: [0, 0, 1.64 + sway],
    'upperarm.l': [0, 0.2, 1.38],
    'lowerarm.l': [0, 0.22, 1.08],
    'hand.l': [0, 0.22, 0.82],
    'upperarm.r': [0, -0.2, 1.38],
    'lowerarm.r': [0, -0.22, 1.08],
    'hand.r': [0, -0.22, 0.82],
    'upperleg.l': [0, 0.1, 0.9],
    'lowerleg.l': [0, 0.1, 0.48],
    'foot.l': [0.06, 0.1, 0.04],
    'upperleg.r': [0, -0.1, 0.9],
    'lowerleg.r': [0, -0.1, 0.48],
    'foot.r': [0.06, -0.1, 0.04],
  };
  if (state === 'walking') {
    const phase = time * Math.PI * 2 * 0.9;
    const swing = Math.sin(phase) * 0.28;
    p['foot.l'][0] += swing;
    p['lowerleg.l'][0] += swing * 0.55;
    p['foot.r'][0] -= swing;
    p['lowerleg.r'][0] -= swing * 0.55;
    p['hand.l'][0] -= swing;
    p['hand.r'][0] += swing;
  } else if (state === 'sitting') {
    p.hips[2] = 0.5;
    p.spine[2] = 0.7;
    p.chest[2] = 0.9;
    p.neck[2] = 1.05;
    p.head[2] = 1.18;
    p['upperleg.l'] = [0.16, 0.1, 0.48];
    p['upperleg.r'] = [0.16, -0.1, 0.48];
    p['lowerleg.l'] = [0.32, 0.1, 0.22];
    p['lowerleg.r'] = [0.32, -0.1, 0.22];
    p['foot.l'] = [0.34, 0.1, 0.04];
    p['foot.r'] = [0.34, -0.1, 0.04];
  } else if (state === 'hiding') {
    p.hips[2] = 0.55;
    p.spine[2] = 0.7;
    p.chest[2] = 0.84;
    p.neck[2] = 0.96;
    p.head[2] = 1.05;
    p['upperleg.l'] = [0.18, 0.1, 0.42];
    p['upperleg.r'] = [0.18, -0.1, 0.42];
    p['lowerleg.l'] = [0.22, 0.1, 0.16];
    p['lowerleg.r'] = [0.22, -0.1, 0.16];
    p['foot.l'] = [0.16, 0.1, 0.04];
    p['foot.r'] = [0.16, -0.1, 0.04];
  } else if (state === 'lying') {
    for (const name of JOINT_NAMES) {
      const q = p[name];
      p[name] = [q[2] - 0.85, q[1], 0.1 + q[0] * 0.12];
    }
  }
  return p;
}

function toWorld(local: Vec3, feet: Vec3, yaw: number): Vec3 {
  const c = Math.cos(yaw);
  const s = Math.sin(yaw);
  return [feet[0] + c * local[0] - s * local[1], feet[1] + s * local[0] + c * local[1], feet[2] + local[2]];
}

function feetAt(person: PlacedTarget, time: number): { feet: Vec3; yaw: number } {
  if (person.path && person.path.length >= 2 && person.state === 'walking') {
    return followPath(person.path, person.speed, time);
  }
  return { feet: person.feet, yaw: person.yaw };
}

function worldPose(person: PlacedTarget, time: number): Record<JointName, Vec3> {
  const state: PersonState = person.state === 'static' ? 'standing' : person.state;
  const local = pose(state, time);
  const { feet, yaw } = feetAt(person, time);
  const out = {} as Record<JointName, Vec3>;
  for (const name of JOINT_NAMES) out[name] = toWorld(local[name], feet, yaw);
  return out;
}

/** Sample skeleton joint positions and velocities. See the note on JOINT_NAMES. */
export function samplePersonMotion(person: PlacedTarget, time: number, dt = 1 / 20): PersonMotionSample | null {
  if (person.classId !== 'person') return null;
  const step = Math.max(1e-3, dt);
  const now = worldPose(person, time);
  const next = worldPose(person, time + step);
  const joints: JointSample[] = JOINT_NAMES.map((name) => {
    const a = now[name];
    const b = next[name];
    return {
      name,
      position: a,
      velocity: [(b[0] - a[0]) / step, (b[1] - a[1]) / step, (b[2] - a[2]) / step],
    };
  });
  return { personId: person.id, time, joints };
}

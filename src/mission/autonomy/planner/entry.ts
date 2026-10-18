import { DRONE_RADIUS } from '../../../shared/constants';
import { doorPose, isDoor, measureClearance } from '../../../world/outdoor/clearance';
import { PASS_RADIUS, REQUIRED_CLEARANCE } from '../../../world/outdoor/constants';
import type { OutdoorPhysics } from '../../../world/outdoor/physics';
import type { Opening, OpeningKind } from '../../../world/outdoor/types';
import type { EntryPolicy } from '../../types';
import type { EntryCandidate, MissionBuilding, V3 } from '../contracts';
import { add, clamp, cross, dot, scale, sub, unit } from './geometry';

/**
 * Entry-point selection. Each observed opening gets a clearance margin over
 * the airframe and a 0..1 safety score from clearance, opening kind, and
 * approach obstruction (casts along the opening normal). Below 0.5 the
 * drone never enters, 0.5 to 0.75 asks the operator, above 0.75 is auto.
 * The mission entry policy caps the decision.
 */

export const SAFETY_ASK = 0.5;
export const SAFETY_AUTO = 0.75;
/** Clearance margin that earns a full clearance score, meters. */
export const FULL_MARGIN = 0.5;
/** Distance outside the opening where the crossing starts. */
export const ALIGN_OUT = 2;
/** Distance inside the opening where the crossing ends. */
export const PASS_IN = 1.2;

/** Kind factor: leaf and shard hazards, vertical entries, and openings that never pass. */
export const KIND_FACTOR: Record<OpeningKind, number> = {
  open_window: 1,
  garage_gap: 1,
  ajar_door: 0.85,
  broken_window: 0.8,
  roof_hatch: 0.9,
  vent: 0.7,
  closed_door: 0,
};

/** The line a crossing follows, with the error budget on each axis. */
export interface PassageAxis {
  /** Point on the aperture plane the centerline passes through, world frame. */
  center: V3;
  /** Outward normal. */
  normal: V3;
  /** Width axis. */
  tangent: V3;
  /** Height axis (up for walls). */
  bitangent: V3;
  /** Allowed error along the tangent before the pass sphere touches. */
  marginU: number;
  /** Allowed error along the bitangent. */
  marginV: number;
  vertical: boolean;
}

/** Point-to-oriented-box distance for a door leaf. */
function leafDistance(p: V3, pose: NonNullable<ReturnType<typeof doorPose>>): number {
  const d = sub(p, pose.center as V3);
  const lx = Math.max(Math.abs(dot(d, pose.axisX as V3)) - pose.hx, 0);
  const ly = Math.max(Math.abs(dot(d, pose.axisY as V3)) - pose.hy, 0);
  const lz = Math.max(Math.abs(dot(d, pose.axisZ as V3)) - pose.hz, 0);
  return Math.hypot(lx, ly, lz);
}

export function passageAxis(opening: Opening): PassageAxis {
  const normal = unit(opening.normal as V3);
  const tangent = unit(opening.tangent as V3);
  const bitangent = unit(cross(normal, tangent));
  const vertical = Math.abs(normal[2]) > 0.7;
  const up = vertical ? bitangent : ([0, 0, 1] as V3);
  const halfH = opening.height / 2;
  let offset = 0;
  let halfU = opening.width / 2;
  if (isDoor(opening.kind)) {
    const pose = doorPose(opening);
    halfU = 0;
    if (pose) {
      // Slide the centerline away from the swung leaf to the widest gap.
      for (let u = -opening.width / 2; u <= opening.width / 2 + 1e-9; u += 0.01) {
        let free = opening.width / 2 - Math.abs(u);
        for (let s = -PASS_IN - 0.2; s <= 0.5; s += 0.05) {
          const p = add(add(opening.center as V3, scale(tangent, u)), scale(normal, s));
          free = Math.min(free, leafDistance(p, pose));
        }
        if (free > halfU) {
          halfU = free;
          offset = u;
        }
      }
    }
  }
  return {
    center: add(opening.center as V3, scale(tangent, offset)),
    normal,
    tangent,
    bitangent: up,
    marginU: halfU - PASS_RADIUS,
    marginV: halfH - PASS_RADIUS,
    vertical,
  };
}

/** Free distance along the outward normal (approach lane) and inward (landing room). */
export function approachObstruction(opening: Opening, axis: PassageAxis, physics: OutdoorPhysics): { outward: number; inward: number } {
  const n = axis.normal;
  const du = clamp(axis.marginU + DRONE_RADIUS, 0, 0.4) * 0.8;
  const dv = clamp(axis.marginV + DRONE_RADIUS, 0, 0.4) * 0.8;
  const offsets: [number, number][] = [
    [0, 0],
    [du, 0],
    [-du, 0],
    [0, dv],
    [0, -dv],
  ];
  const outReach = ALIGN_OUT + 0.5;
  const inReach = PASS_IN + 0.5;
  const wallOut = opening.kind === 'roof_hatch' ? 0.15 : 0.6;
  let outward = outReach;
  let inward = inReach;
  for (const [u, v] of offsets) {
    const base = add(add(axis.center, scale(axis.tangent, u)), scale(axis.bitangent, v));
    const o = add(base, scale(n, wallOut));
    const hitOut = physics.cast(o[0], o[1], o[2], n[0], n[1], n[2], outReach);
    if (hitOut) outward = Math.min(outward, hitOut.dist);
    const i0 = add(base, scale(n, -0.05));
    const hitIn = physics.cast(i0[0], i0[1], i0[2], -n[0], -n[1], -n[2], inReach);
    if (hitIn) inward = Math.min(inward, hitIn.dist);
  }
  return { outward, inward };
}

export function entryDecision(safety: number, policy: EntryPolicy): EntryCandidate['decision'] {
  if (policy === 'never' || safety < SAFETY_ASK) return 'never';
  const raw = safety > SAFETY_AUTO ? 'auto' : 'ask';
  return policy === 'ask' ? 'ask' : raw;
}

/** Safety from the parts. Exposed so tests can check the thresholds. */
export function safetyScore(kind: OpeningKind, clearanceM: number, obstruction: number): number {
  if (clearanceM <= 0) return 0;
  const k = KIND_FACTOR[kind] ?? 0;
  const c = clamp(clearanceM / FULL_MARGIN, 0, 1);
  return clamp(k * (0.35 + 0.65 * c) * (0.4 + 0.6 * clamp(obstruction, 0, 1)), 0, 1);
}

export function scoreOpening(
  building: MissionBuilding,
  opening: Opening,
  physics: OutdoorPhysics | null,
  policy: EntryPolicy,
): EntryCandidate {
  const clear = measureClearance(opening);
  const clearanceM = Math.min(clear.clearWidth - REQUIRED_CLEARANCE, clear.clearHeight - REQUIRED_CLEARANCE);
  let obstruction = 1;
  if (physics && clearanceM > 0 && clear.passable) {
    const axis = passageAxis(opening);
    const lane = approachObstruction(opening, axis, physics);
    const fo = clamp((lane.outward - 0.3) / (ALIGN_OUT - 0.3), 0, 1);
    const fi = clamp((lane.inward - 0.4) / (PASS_IN - 0.2), 0, 1);
    obstruction = Math.min(fo, fi);
  }
  const safety = clear.passable && opening.kind !== 'closed_door' ? safetyScore(opening.kind, clearanceM, obstruction) : 0;
  return {
    buildingId: building.id,
    openingId: opening.id,
    kind: opening.kind,
    clearWidth: clear.clearWidth,
    clearHeight: clear.clearHeight,
    clearanceM,
    safety,
    decision: entryDecision(safety, policy),
  };
}

/** Viable candidates, best first: auto before ask, then by safety, then by id. */
export function rankCandidates(candidates: readonly EntryCandidate[]): EntryCandidate[] {
  const order = { auto: 0, ask: 1, never: 2 } as const;
  return candidates
    .filter((c) => c.decision !== 'never')
    .sort((a, b) => order[a.decision] - order[b.decision] || b.safety - a.safety || a.openingId - b.openingId);
}

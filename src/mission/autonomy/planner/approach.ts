import type { OutdoorPhysics } from '../../../world/outdoor/physics';
import type { MissionBuilding, V3 } from '../contracts';
import {
  type Airspace,
  buildingBox,
  type BuildingBox,
  columnClear,
  CRUISE_AGL,
  cruisePath,
  cruiseZ,
  operable,
  pointFree,
} from './geometry';

/**
 * Building approach: cruise to a point above the building's clearing,
 * then descend straight down through open air to a standoff beside the
 * shell. The descent column is checked with casts so it never cuts canopy.
 */

/** Height above the pad that inspection and entry fly at. */
export const VIEW_HEIGHT = 1.4;
/** Standoff distances from the wall that are tried, meters. */
export const STANDOFFS = [3, 4.5, 6, 7.5];

export interface Standoff {
  pos: V3;
  /** Yaw that faces the shell. */
  yaw: number;
  /** Outward wall normal of the face it looks at. */
  face: 'n' | 's' | 'e' | 'w' | 'corner';
}

export interface ApproachPlan {
  ok: boolean;
  reason: string;
  standoff: Standoff | null;
  /** Cruise-height route ending above the standoff. */
  cruise: V3[];
  /** Straight-down descent from cruise to the standoff. */
  descent: V3[];
}

/** Ring of candidate standoff points around the footprint. */
export function standoffCandidates(box: BuildingBox, z: number, offsets = STANDOFFS): Standoff[] {
  const out: Standoff[] = [];
  for (const s of offsets) {
    for (const f of [0.5, 0.3, 0.7, 0.15, 0.85]) {
      const x = box.minX + (box.maxX - box.minX) * f;
      const y = box.minY + (box.maxY - box.minY) * f;
      out.push({ pos: [x, box.minY - s, z], yaw: Math.PI / 2, face: 's' });
      out.push({ pos: [x, box.maxY + s, z], yaw: -Math.PI / 2, face: 'n' });
      out.push({ pos: [box.minX - s, y, z], yaw: 0, face: 'w' });
      out.push({ pos: [box.maxX + s, y, z], yaw: Math.PI, face: 'e' });
    }
    const d = s * Math.SQRT1_2;
    for (const [cx, cy] of [
      [box.minX - d, box.minY - d],
      [box.maxX + d, box.minY - d],
      [box.maxX + d, box.maxY + d],
      [box.minX - d, box.maxY + d],
    ] as const) {
      out.push({ pos: [cx, cy, z], yaw: Math.atan2(box.cy - cy, box.cx - cx), face: 'corner' });
    }
  }
  return out;
}

/** A standoff is usable when it is operable, free, and has an open column up to cruise. */
export function standoffOpen(physics: OutdoorPhysics, air: Airspace, s: Standoff, agl = CRUISE_AGL): boolean {
  const [x, y, z] = s.pos;
  if (!operable(air, x, y, 1)) return false;
  if (!pointFree(physics, s.pos, 0.6)) return false;
  return columnClear(physics, x, y, z, cruiseZ(air, x, y, agl) + 0.5, 0.6);
}

export function planApproach(
  building: MissionBuilding,
  from: V3,
  physics: OutdoorPhysics,
  air: Airspace,
  agl = CRUISE_AGL,
): ApproachPlan {
  const box = buildingBox(building);
  const z = box.baseZ + VIEW_HEIGHT;
  const cands = standoffCandidates(box, z);
  // Nearer offsets first, then the side facing the arrival.
  const scored = cands.map((s, i) => ({ s, i, cost: Math.hypot(s.pos[0] - from[0], s.pos[1] - from[1]) + i * 0.02 }));
  scored.sort((a, b) => a.cost - b.cost || a.i - b.i);
  for (const { s } of scored) {
    if (!standoffOpen(physics, air, s, agl)) continue;
    const top: V3 = [s.pos[0], s.pos[1], cruiseZ(air, s.pos[0], s.pos[1], agl)];
    const cruise = cruisePath(air, from, { x: top[0], y: top[1] }, agl);
    if (cruise.length) cruise[cruise.length - 1] = top;
    else cruise.push(top);
    const descent: V3[] = [];
    const steps = Math.max(1, Math.ceil((top[2] - s.pos[2]) / 3));
    for (let k = 1; k <= steps; k++) descent.push([s.pos[0], s.pos[1], top[2] + ((s.pos[2] - top[2]) * k) / steps]);
    return { ok: true, reason: '', standoff: s, cruise, descent };
  }
  return { ok: false, reason: 'no open descent column near the building', standoff: null, cruise: [], descent: [] };
}

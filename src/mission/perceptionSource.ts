import { OBJECT_IDS } from '../targets/catalog';
import type { TargetClass } from '../targets/types';
import { sampleHeight } from '../world/outdoor/terrain';
import type { Contact, MissionWorld, MissionWorldOptions, V3 } from './autonomy/contracts';
import { createPerceptionSource, gimbalFootprint, GIMBAL_MIN_AGL_M } from './autonomy/perception';
import { buildMissionWorld } from './autonomy/world';
import type { Detection, DetectionSource } from './detections';
import { centroid, pointInRing, roundTo } from './polygon';
import { closestAlong, isOperable, pathLength, planPath, pointAtDistance } from './path';
import type { Mission, TargetKind, TargetSpec } from './types';
import type { WorldSnapshot } from './world';

/** Sweep height above ground when the mission band allows it. */
export const SWEEP_AGL_M = 12;
/** Preview cruise speed, matching the session. */
const CRUISE_M_PER_S = 3;
/** Contacts closer than this, of the same reported class, are one object. */
const CLUSTER_M = 2.5;
/** A cluster needs this many frames before it becomes a detection. */
const MIN_VIEWS = 2;

export interface PerceptionCatalogOptions {
  /** World options besides the seed. Must match what built the mission. */
  world?: Omit<MissionWorldOptions, 'seed'>;
  sweepAglM?: number;
  /** Meters between camera frames along the sweep. Default 1.5. */
  sampleM?: number;
}

interface Cluster {
  classId: TargetClass;
  sum: V3;
  views: number;
  confidence: number;
  tSec: number;
  buildingId: string | null;
}

const OBJECT_SET = new Set<string>(OBJECT_IDS);

function kindOf(classId: TargetClass): TargetKind | null {
  if (classId === 'person') return 'person';
  if (OBJECT_SET.has(classId)) return 'object';
  // A correctly named decoy (mannequin, coat, ...) is not a search target.
  return null;
}

function cluster(contacts: Contact[]): Cluster[] {
  const out: Cluster[] = [];
  for (const contact of contacts) {
    const hit = out.find((item) => {
      if (item.classId !== contact.classId) return false;
      const n = item.views;
      return Math.hypot(item.sum[0] / n - contact.estimate[0], item.sum[1] / n - contact.estimate[1], item.sum[2] / n - contact.estimate[2]) <= CLUSTER_M;
    });
    if (hit) {
      hit.sum = [hit.sum[0] + contact.estimate[0], hit.sum[1] + contact.estimate[1], hit.sum[2] + contact.estimate[2]];
      hit.views += 1;
      hit.confidence = Math.max(hit.confidence, contact.confidence);
      hit.buildingId = hit.buildingId ?? contact.buildingId;
    } else {
      out.push({ classId: contact.classId, sum: [...contact.estimate], views: 1, confidence: contact.confidence, tSec: contact.tSec, buildingId: contact.buildingId });
    }
  }
  return out;
}

/** Sweep frames along the plan at `agl`, then a low look-in loop around each nearby building. */
function flyMission(mission: Mission, world: MissionWorld, agl: number, sampleM: number): { contacts: Contact[]; surveyed: Map<string, number>; clear: ReturnType<ReturnType<typeof createPerceptionSource>['truthClearStatus']> } {
  const source = createPerceptionSource(world);
  const terrain = world.outdoor.terrain;
  const plan = planPath(mission);
  const total = pathLength(plan);
  const contacts: Contact[] = [];
  const poses: { x: number; y: number; yaw: number; z: number }[] = [];
  let tSec = 0;
  for (let d = 0; d <= total + 1e-6; d += sampleM) {
    const at = pointAtDistance(plan, d);
    const z = sampleHeight(terrain, at.x, at.y) + agl;
    const vel: V3 = [Math.cos(at.yawRad) * CRUISE_M_PER_S, Math.sin(at.yawRad) * CRUISE_M_PER_S, 0];
    tSec = d / CRUISE_M_PER_S;
    contacts.push(...source.observe({ pos: [at.x, at.y, z], yaw: at.yawRad, vel }, tSec).contacts);
    poses.push({ x: at.x, y: at.y, yaw: at.yawRad, z });
  }
  for (const building of world.buildings) {
    const center = centroid(building.footprint);
    if (closestAlong(plan, center).gap > 20) continue;
    if (mission.noFlyZones.some((zone) => pointInRing(center, zone.polygon.points))) continue;
    const xs = building.footprint.map((p) => p.x);
    const ys = building.footprint.map((p) => p.y);
    const pad = 2.5;
    const loop = [
      { x: Math.min(...xs) - pad, y: Math.min(...ys) - pad },
      { x: Math.max(...xs) + pad, y: Math.min(...ys) - pad },
      { x: Math.max(...xs) + pad, y: Math.max(...ys) + pad },
      { x: Math.min(...xs) - pad, y: Math.max(...ys) + pad },
    ];
    loop.push(loop[0]!);
    const z = building.shell.origin[2] + 1.5;
    for (let leg = 0; leg < 4; leg++) {
      const a = loop[leg]!;
      const b = loop[leg + 1]!;
      const len = Math.hypot(b.x - a.x, b.y - a.y);
      for (let s = 0; s < len; s += 1) {
        const p = { x: a.x + ((b.x - a.x) * s) / len, y: a.y + ((b.y - a.y) * s) / len };
        tSec += 1 / CRUISE_M_PER_S;
        if (!isOperable(p, mission)) continue;
        const yaw = Math.atan2(center.y - p.y, center.x - p.x);
        contacts.push(...source.observe({ pos: [p.x, p.y, z], yaw, vel: [0, 0, 0] }, tSec).contacts);
      }
    }
    // Hover just outside each wall opening and pan across the room behind it.
    for (const opening of building.shell.openings) {
      if (opening.kind === 'roof_hatch' || opening.floor !== 0) continue;
      const p = { x: opening.center[0] + opening.normal[0] * 0.6, y: opening.center[1] + opening.normal[1] * 0.6 };
      if (!isOperable(p, mission)) continue;
      const inward = Math.atan2(-opening.normal[1], -opening.normal[0]);
      for (const pan of [-1, -0.5, 0, 0.5, 1]) {
        tSec += 0.5;
        contacts.push(...source.observe({ pos: [p.x, p.y, opening.center[2]], yaw: inward + pan, vel: [0, 0, 0] }, tSec).contacts);
      }
    }
  }
  // Roof survey: the fraction of each roof inside a gimbal footprint.
  const surveyed = new Map<string, number>();
  for (const building of world.buildings) {
    const xs = building.footprint.map((p) => p.x);
    const ys = building.footprint.map((p) => p.y);
    const roofZ = building.shell.origin[2] + building.heightM;
    let seen = 0;
    let count = 0;
    for (let x = Math.min(...xs) + 0.5; x < Math.max(...xs); x += 1) {
      for (let y = Math.min(...ys) + 0.5; y < Math.max(...ys); y += 1) {
        count += 1;
        const hit = agl >= GIMBAL_MIN_AGL_M && poses.some((pose) => {
          // Footprint on the roof plane: the camera's height over the roof stands in for AGL.
          const fp = gimbalFootprint(pose.z - roofZ + 1.15, 'object');
          const dx = x - pose.x;
          const dy = y - pose.y;
          const along = dx * Math.cos(pose.yaw) + dy * Math.sin(pose.yaw);
          const across = -dx * Math.sin(pose.yaw) + dy * Math.cos(pose.yaw);
          return Math.abs(along) <= fp.halfAlongM && Math.abs(across) <= fp.halfAcrossM;
        });
        if (hit) seen += 1;
      }
    }
    surveyed.set(building.id, count === 0 ? 0 : seen / count);
  }
  return { contacts, surveyed, clear: source.truthClearStatus('outdoor', 'person') };
}

function detectionFrom(item: Cluster, id: string, kind: TargetKind, label: string): Detection {
  const n = item.views;
  return {
    id,
    tSec: 0,
    kind,
    label,
    confidence: roundTo(item.confidence, 2),
    x: roundTo(item.sum[0] / n, 2),
    y: roundTo(item.sum[1] / n, 2),
    z: roundTo(item.sum[2] / n, 2),
    structureId: item.buildingId,
    entryRequested: item.buildingId !== null,
  };
}

/**
 * Legacy DetectionSource backed by real perception. It rebuilds the
 * MissionWorld from the snapshot seed, flies the mission's lawnmower plan at
 * sweep height with the gimbal camera, loops low around nearby buildings to
 * look in through their openings, and turns multi-view contact clusters into
 * detections. Contacts seen inside a building request entry. Hazards are not
 * perceived, so hazard targets get no detection.
 */
export function createPerceptionDetectionSource(options: PerceptionCatalogOptions = {}): DetectionSource {
  let cached: MissionWorld | null = null;
  const worldFor = (seed: number): MissionWorld => {
    if (cached?.seed !== seed >>> 0) cached = buildMissionWorld({ ...options.world, seed: seed >>> 0 });
    return cached;
  };
  return {
    id: 'perception-detections',
    catalog(mission: Mission, snapshot: WorldSnapshot): Detection[] {
      const world = worldFor(snapshot.seed);
      const agl = Math.min(mission.altitude.maxM, Math.max(mission.altitude.minM, options.sweepAglM ?? SWEEP_AGL_M));
      const flight = flyMission(mission, world, agl, options.sampleM ?? 1.5);
      const clusters = cluster(flight.contacts)
        .filter((item) => item.views >= MIN_VIEWS && kindOf(item.classId) !== null)
        .sort((a, b) => b.confidence - a.confidence || a.tSec - b.tSec);
      const used = new Set<Cluster>();
      const out: Detection[] = [];
      const take = (target: TargetSpec, kind: TargetKind): void => {
        const matches = clusters.filter((item) => !used.has(item) && kindOf(item.classId) === kind);
        matches.slice(0, Math.max(1, target.minCount)).forEach((item, index) => {
          used.add(item);
          out.push(detectionFrom(item, index === 0 ? `det-${target.id}` : `det-${target.id}-${index + 1}`, kind, target.label));
        });
      };
      for (const target of mission.targets) {
        if (target.kind === 'person' || target.kind === 'object') take(target, target.kind);
        if (target.kind === 'structure') {
          const best = [...flight.surveyed.entries()].sort((a, b) => b[1] - a[1])[0];
          const building = best ? world.buildings.find((item) => item.id === best[0]) : undefined;
          if (building && best && best[1] > 0) {
            const center = centroid(building.footprint);
            out.push({
              id: `det-${target.id}`,
              tSec: 0,
              kind: 'structure',
              label: `${target.label}, ${building.label}`,
              confidence: roundTo(Math.min(0.99, 0.55 + 0.44 * best[1]), 2),
              x: roundTo(center.x, 2),
              y: roundTo(center.y, 2),
              z: roundTo(building.heightM, 2),
              structureId: building.id,
              entryRequested: false,
            });
          }
        }
        if (target.kind === 'clearance' && flight.clear.status !== 'occupied' && flight.clear.total > 0) {
          const plan = planPath(mission);
          const last = plan[plan.length - 1] ?? { x: mission.startPose.x, y: mission.startPose.y };
          const seen = flight.clear.seenEmpty / flight.clear.total;
          out.push({
            id: `det-${target.id}`,
            tSec: 0,
            kind: 'clearance',
            label: `${target.label}, ${Math.round(seen * 100)}% seen empty`,
            confidence: roundTo(seen, 2),
            x: roundTo(last.x, 2),
            y: roundTo(last.y, 2),
            z: agl,
            structureId: null,
            entryRequested: false,
          });
        }
      }
      let extra = 0;
      for (const item of clusters) {
        if (used.has(item)) continue;
        const kind = kindOf(item.classId)!;
        extra += 1;
        out.push(detectionFrom(item, `det-contact-${extra}`, kind, `Possible ${item.classId}`));
      }
      return out;
    },
  };
}

/** Default perception-backed source for the mission page. */
export const perceptionDetectionSource: DetectionSource = createPerceptionDetectionSource();

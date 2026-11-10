import { RNG } from '../shared/rng';
import type { BuildingSpec, SensorMode } from '../shared/types';
import type { Aabb, ResolvedTarget, TargetClass, Vec3 } from '../targets/types';
import { projectAabb, type CameraView } from './camera';
import { occlusionFraction } from './occlusion';
import { smokePath, smokeTransmittance } from '../gen/hazards';
import { CLASS_PROFILE, CONFUSION, DECOY_FP_BASE, FLASH_HALF, illuminationOf, recallProbability, THERMAL } from './model';
import { clamp, dist, dot, norm, sub } from './vec';

export interface Detection {
  id: string;
  targetId: string;
  classId: TargetClass;
  trueClass: TargetClass;
  confidence: number;
  bbox: { x: number; y: number; w: number; h: number };
  distance: number;
  occlusion: number;
  kind: 'tp' | 'fp' | 'confused';
}

export interface DetectRequest {
  seed: number;
  camera: CameraView;
  targets: ResolvedTarget[];
  spec?: BuildingSpec | null;
  lighting: {
    timeOfDay: number;
    interiorLightsOn: ArrayLike<number>;
    flashlightOn: boolean;
    sensorMode: SensorMode;
  };
  cameraSpeed?: number;
  /**
   * Optics gain over the FPV reference camera. Recall and blur use
   * distance / rangeScale, so a narrower lens on the same sensor resolves
   * targets farther away. Default 1.
   */
  rangeScale?: number;
  /**
   * The drone also carries a thermal camera (model.ts THERMAL): a person is seen by whichever of
   * the two cameras sees them better. Default off.
   */
  thermal?: boolean;
}

function angleTo(cam: CameraView, point: Vec3): number {
  return Math.acos(clamp(dot(cam.forward, norm(sub(point, cam.position))), -1, 1));
}

function sampleLabel(classId: TargetClass, rng: RNG): TargetClass {
  const row = CONFUSION[classId];
  let u = rng.next();
  const keys = Object.keys(row) as TargetClass[];
  for (const key of keys) {
    u -= row[key] ?? 0;
    if (u <= 0) return key;
  }
  return keys[keys.length - 1] ?? classId;
}

function jitter(bbox: Detection['bbox'], rng: RNG): Detection['bbox'] {
  const jx = (rng.next() - 0.5) * 0.012;
  const jy = (rng.next() - 0.5) * 0.012;
  const x = clamp(bbox.x + jx, 0, 0.98);
  const y = clamp(bbox.y + jy, 0, 0.98);
  return { x, y, w: Math.min(bbox.w, 1 - x), h: Math.min(bbox.h, 1 - y) };
}

/**
 * Contrast the smoke along the camera-to-target line leaves, 1 without smoke. Building-local frame.
 * `gain` stretches the visibility for a band smoke scatters less (thermal).
 */
function smokeFactor(req: DetectRequest, center: readonly number[], gain = 1): number {
  const smoke = req.spec?.hazards?.smoke;
  if (!req.spec || !smoke) return 1;
  const cam = req.camera.position;
  const f = Math.max(0, Math.min(req.spec.floors.length - 1, Math.floor(cam[2] / req.spec.floorHeight)));
  return smokeTransmittance(smokePath(req.spec, f, cam[0], cam[1], center[0]!, center[1]!), smoke.visibility * gain);
}

/**
 * Simulated camera detector. The same seed, pose, and targets always
 * return the same detections. False positives are draws on decoys, so
 * they cluster on the decoy's projected box.
 */
export function detect(req: DetectRequest): Detection[] {
  const rng = new RNG(req.seed >>> 0, 'detect');
  const cameraSpeed = req.cameraSpeed ?? 0;
  const rangeScale = req.rangeScale && req.rangeScale > 0 ? req.rangeScale : 1;
  const out: Detection[] = [];
  const ordered = [...req.targets].sort((a, b) => (a.id < b.id ? -1 : a.id > b.id ? 1 : 0));
  for (const target of ordered) {
    const blockers: Aabb[] = [...target.occluders];
    for (const other of ordered) {
      if (other.id === target.id) continue;
      blockers.push(other.aabb, ...other.occluders);
    }
    const projected = projectAabb(req.camera, target.aabb);
    const distance = dist(req.camera.position, target.center);
    const seenDistance = distance / rangeScale;
    const occlusion = projected ? occlusionFraction(req.camera, target.aabb, req.spec, blockers) : 0;
    const interiorOn = (req.lighting.interiorLightsOn[target.roomIndex] ?? 0) > 0;
    const inFlashlight = req.lighting.flashlightOn && angleTo(req.camera, target.center) <= FLASH_HALF;
    const illumination = illuminationOf({
      timeOfDay: req.lighting.timeOfDay,
      interiorOn,
      flashlightOn: req.lighting.flashlightOn,
      inFlashlight,
      sensorMode: req.lighting.sensorMode,
    });
    const area = projected?.area ?? 0;
    // Smoke (gen/hazards.ts) takes contrast off the line of sight: recall scales by what is left.
    const smoke = smokeFactor(req, target.center);
    const visible = smoke * (projected
      ? recallProbability({
          classId: target.classId,
          distance: seenDistance,
          occlusion,
          illumination,
          speed: target.speed,
          cameraSpeed,
          area,
        })
      : 0);
    // The thermal camera: its own range and smoke loss, no need for light, and glass is a wall to it.
    const warm = req.thermal === true && projected !== null && THERMAL.classes.includes(target.classId);
    const thermal = warm
      ? smokeFactor(req, target.center, THERMAL.smokeGain) *
        recallProbability({
          classId: target.classId,
          distance: seenDistance / THERMAL.rangeScale,
          occlusion: occlusionFraction(req.camera, target.aabb, req.spec, blockers, 1),
          illumination: 1,
          speed: target.speed,
          cameraSpeed,
          area,
        })
      : 0;
    const recall = Math.max(visible, thermal);
    const lookalike = target.looksLike;
    const lookRecall = smoke * (projected && lookalike
      ? recallProbability({
          classId: lookalike,
          distance: seenDistance,
          occlusion,
          illumination,
          speed: target.speed,
          cameraSpeed,
          area,
        })
      : 0);
    const uSee = rng.next();
    const uFp = rng.next();
    const label = sampleLabel(target.classId, rng);
    const noise = rng.gaussian();
    const box = projected ? jitter(projected.bbox, rng) : null;
    if (projected && box) {
      if (target.category === 'decoy' && lookalike) {
        const pFp = clamp(DECOY_FP_BASE[target.classId as keyof typeof DECOY_FP_BASE] * lookRecall, 0, 0.95);
        if (uFp < pFp) {
          const profile = CLASS_PROFILE[lookalike];
          out.push({
            id: `${target.id}:fp`,
            targetId: target.id,
            classId: lookalike,
            trueClass: target.classId,
            confidence: clamp(profile.confidence * (0.45 + 0.4 * pFp) + noise * 0.02, 0.05, 0.99),
            bbox: box,
            distance,
            occlusion,
            kind: 'fp',
          });
        }
        if (uSee < recall) {
          out.push({
            id: `${target.id}:id`,
            targetId: target.id,
            classId: target.classId,
            trueClass: target.classId,
            confidence: clamp(CLASS_PROFILE[target.classId].confidence * (0.5 + 0.4 * recall) + noise * 0.015, 0.05, 0.99),
            bbox: box,
            distance,
            occlusion,
            kind: 'tp',
          });
        }
      } else if (uSee < recall) {
        const profile = CLASS_PROFILE[label];
        out.push({
          id: `${target.id}:${label}`,
          targetId: target.id,
          classId: label,
          trueClass: target.classId,
          confidence: clamp(profile.confidence * (0.55 + 0.45 * recall) + noise * 0.02, 0.05, 0.99),
          bbox: box,
          distance,
          occlusion,
          kind: label === target.classId ? 'tp' : 'confused',
        });
      }
    }
  }
  out.sort((a, b) => b.confidence - a.confidence || (a.id < b.id ? -1 : 1));
  return out;
}

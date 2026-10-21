import type { Contact, DronePose, MissionWorld, V3 } from '../mission/autonomy/contracts';
import { bboxRay, createPerceptionSource, type CameraContactSource, type PerceptionOptions } from '../mission/autonomy/perception';
import type { CameraView } from '../perception/camera';
import { RNG } from '../shared/rng';
import type { FrameDetector, VisionDetection } from './detector';
import type { HumanoidPoses } from './humanoid';
import { VisionScene, type Frame } from './scene';

export interface VisionOptions {
  /** Relative 1-sigma error of the depth camera's range. Default 0.03. */
  depthNoise?: number;
  /** Called with every frame and its detections (the live camera inset, the bench's frame log). */
  onFrame?: (frame: Frame, detections: VisionDetection[]) => void;
}

/**
 * Range to a detection from the depth image, over the central part of the
 * box: depths are grouped into surfaces (gaps over 0.3 m split them) and the
 * nearest surface holding at least 15% of the core is taken, at its median.
 * The floor showing past a lying person, or a wall behind someone, is
 * farther and loses; clutter at the box edge mostly falls outside the core.
 * Null when the core sees only sky.
 */
export function boxRange(frame: Frame, bbox: VisionDetection['bbox']): number | null {
  const W = frame.width;
  const H = frame.height;
  const x0 = Math.max(0, Math.floor((bbox.x + bbox.w * 0.25) * W));
  const x1 = Math.min(W - 1, Math.ceil((bbox.x + bbox.w * 0.75) * W));
  const y0 = Math.max(0, Math.floor((bbox.y + bbox.h * 0.2) * H));
  const y1 = Math.min(H - 1, Math.ceil((bbox.y + bbox.h * 0.8) * H));
  const values: number[] = [];
  for (let y = y0; y <= y1; y++) {
    for (let x = x0; x <= x1; x++) {
      const d = frame.depth[y * W + x]!;
      if (Number.isFinite(d)) values.push(d);
    }
  }
  if (!values.length) return null;
  values.sort((a, b) => a - b);
  const need = Math.max(1, Math.ceil(values.length * 0.15));
  let start = 0;
  for (let i = 1; i <= values.length; i++) {
    if (i < values.length && values[i]! - values[i - 1]! <= 0.3) continue;
    if (i - start >= need) return values[(start + i - 1) >> 1]!;
    start = i;
  }
  return values[values.length >> 1]!;
}

const SURFACE_TO_CENTER_M = 0.15;

/** Ground truth behind a box, for scoring only: the instance owning most of its pixels. */
function truthOf(frame: Frame, det: VisionDetection): Contact['truth'] {
  const W = frame.width;
  const H = frame.height;
  const x0 = Math.max(0, Math.floor(det.bbox.x * W));
  const x1 = Math.min(W - 1, Math.ceil((det.bbox.x + det.bbox.w) * W) - 1);
  const y0 = Math.max(0, Math.floor(det.bbox.y * H));
  const y1 = Math.min(H - 1, Math.ceil((det.bbox.y + det.bbox.h) * H) - 1);
  const counts = new Map<number, number>();
  let area = 0;
  for (let y = y0; y <= y1; y++) {
    for (let x = x0; x <= x1; x++) {
      area++;
      const k = frame.inst[y * W + x]!;
      if (k > 0) counts.set(k, (counts.get(k) ?? 0) + 1);
    }
  }
  let best = 0;
  let bestN = 0;
  for (const [k, n] of counts) {
    if (n > bestN) {
      best = k;
      bestN = n;
    }
  }
  if (!best || bestN < Math.max(3, area * 0.08)) return { targetId: 'background', trueClass: det.classId, kind: 'fp' };
  const t = frame.truth[best - 1]!;
  if (t.category === 'decoy') return { targetId: t.id, trueClass: t.classId, kind: 'fp' };
  return { targetId: t.id, trueClass: t.classId, kind: t.classId === det.classId ? 'tp' : 'confused' };
}

/** Which building a world point is in, by footprint box and height. */
function buildingOf(world: MissionWorld, p: V3): string | null {
  for (const b of world.buildings) {
    const xs = b.footprint.map((q) => q.x);
    const ys = b.footprint.map((q) => q.y);
    if (p[0] >= Math.min(...xs) && p[0] <= Math.max(...xs) && p[1] >= Math.min(...ys) && p[1] <= Math.max(...ys) && p[2] >= b.shell.origin[2] - 0.3 && p[2] <= b.shell.origin[2] + b.heightM + 0.3) return b.id;
  }
  return null;
}

/**
 * Contacts from pixels: each frame is rendered (src/vision/scene.ts), the
 * learned detector reads it, and every detection becomes a contact whose
 * position is the box-center ray at the depth image's range.
 */
export function visionContacts(world: MissionWorld, poses: HumanoidPoses, detector: FrameDetector, opts: VisionOptions = {}): CameraContactSource {
  const scene = new VisionScene(world, poses);
  const lighting = { timeOfDay: world.timeOfDay, flashlightOn: world.flashlightOn, sensorMode: world.sensorMode };
  const depthNoise = opts.depthNoise ?? 0.03;
  return {
    id: 'vision',
    contacts(view: CameraView, _pose: DronePose, tSec: number, frameNo: number): Contact[] {
      const noiseSeed = (Math.imul(world.seed ^ 0x51ed27, 2654435761) ^ Math.imul(frameNo + 1, 40503)) >>> 0;
      const frame = scene.render(view, tSec, lighting, noiseSeed);
      const detections = detector.detect(frame);
      opts.onFrame?.(frame, detections);
      const eye = view.position;
      const out: Contact[] = [];
      detections.forEach((det, i) => {
        const measured = boxRange(frame, det.bbox);
        if (measured == null) return;
        const noise = new RNG(noiseSeed, `depth:${i}`).gaussian();
        // The depth camera sees the near surface; the body center is a little past it.
        const range = Math.max(0.3, measured * (1 + depthNoise * noise) + SURFACE_TO_CENTER_M);
        const ray = bboxRay(view, det.bbox);
        const estimate: V3 = [eye[0] + ray[0] * range, eye[1] + ray[1] * range, eye[2] + ray[2] * range];
        out.push({
          id: `f${frameNo}:v${i}:${det.classId}`,
          tSec,
          classId: det.classId,
          confidence: det.confidence,
          estimate,
          range,
          bearing: ray,
          buildingId: buildingOf(world, estimate),
          truth: truthOf(frame, det),
        });
      });
      return out;
    },
  };
}

/** A LiveDetectionSource whose contacts come from the camera and the learned detector. */
export function createVisionSource(world: MissionWorld, poses: HumanoidPoses, detector: FrameDetector, opts: VisionOptions & PerceptionOptions = {}) {
  return createPerceptionSource(world, { ...opts, camera: visionContacts(world, poses, detector, opts) });
}

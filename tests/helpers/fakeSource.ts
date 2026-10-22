import type { ClearReport } from '../../src/perception/seenEmpty';
import { indoorCells } from '../../src/perception/seenEmpty';
import { fpvCameraFromDrone } from '../../src/perception/camera';
import type { DronePose, LiveDetectionSource, MissionWorld, Observation, TruthTarget, V3 } from '../../src/mission/autonomy/contracts';
import { hashSeed } from '../../src/shared/rng';
import { resolveTarget } from '../../src/targets/motion';
import type { TargetClass } from '../../src/targets/types';
import { withSensorFrame, type RawContact } from '../../src/mission/autonomy/perception';
import { initOutdoorPhysics, OutdoorPhysics } from '../../src/world/outdoor/physics';

/**
 * Test-only detection source. Reveals truth targets that are in the camera
 * FOV, in range, and in line of sight (outdoor physics ray). Decoys report
 * their look-alike class at low confidence on some frames. Seen-empty cells
 * are tracked with the same FOV and ray test, so truthClearStatus is honest.
 */

export interface FakeSourceOptions {
  range?: number;
  personConfidence?: number;
  objectConfidence?: number;
  decoyConfidence?: number;
  /** Extra class/confidence overrides by target id, e.g. a single high-confidence ghost. */
  ghosts?: { at: V3; classId: TargetClass; confidence: number; frames: number }[];
}

interface Item {
  truth: TruthTarget;
  reportAs: TargetClass;
  confidence: number;
}

function unit(v: V3): V3 {
  const l = Math.hypot(v[0], v[1], v[2]) || 1;
  return [v[0] / l, v[1] / l, v[2] / l];
}

export async function createFakeSource(world: MissionWorld, options: FakeSourceOptions = {}): Promise<LiveDetectionSource & { physics: OutdoorPhysics }> {
  await initOutdoorPhysics();
  const physics = new OutdoorPhysics(world.outdoor);
  const range = options.range ?? 8;
  const items: Item[] = [];
  for (const b of world.buildings) {
    for (const t of b.scene.targets) {
      const r = resolveTarget(t, 0);
      const c = r.center;
      const pos: V3 = [c[0] + b.shell.origin[0], c[1] + b.shell.origin[1], c[2] + b.shell.origin[2]];
      const truth: TruthTarget = { id: t.id, classId: t.classId, category: t.category, buildingId: b.id, world: pos };
      const decoy = t.category === 'decoy';
      items.push({
        truth,
        reportAs: decoy ? (t.looksLike ?? 'person') : t.classId,
        confidence: decoy ? (options.decoyConfidence ?? 0.62) : t.classId === 'person' ? (options.personConfidence ?? 0.9) : (options.objectConfidence ?? 0.86),
      });
    }
  }
  const cells = new Map<string, { center: V3; seen: boolean }[]>();
  for (const b of world.buildings) {
    cells.set(
      b.id,
      indoorCells(b.shell.spec).map((cell) => ({
        center: [cell.center[0] + b.shell.origin[0], cell.center[1] + b.shell.origin[1], cell.center[2] + b.shell.origin[2]] as V3,
        seen: false,
      })),
    );
  }
  const ghosts = (options.ghosts ?? []).map((g) => ({ ...g, left: g.frames }));
  let frame = 0;

  const visible = (eye: V3, cam: ReturnType<typeof fpvCameraFromDrone>, p: V3, reach: number): number | null => {
    const d: V3 = [p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]];
    const r = Math.hypot(d[0], d[1], d[2]);
    if (r > reach || r < 0.2) return null;
    const z = d[0] * cam.forward[0] + d[1] * cam.forward[1] + d[2] * cam.forward[2];
    if (z <= 0.05) return null;
    const tanY = Math.tan(cam.fovY / 2);
    const x = (d[0] * cam.right[0] + d[1] * cam.right[1] + d[2] * cam.right[2]) / z;
    const y = (d[0] * cam.up[0] + d[1] * cam.up[1] + d[2] * cam.up[2]) / z;
    if (Math.abs(x) > tanY * cam.aspect || Math.abs(y) > tanY) return null;
    const hit = physics.cast(eye[0], eye[1], eye[2], d[0], d[1], d[2], r);
    if (hit && hit.dist < r - 0.35) return null;
    return r;
  };

  return {
    id: `fake-${world.seed}`,
    physics,
    observe(pose: DronePose, tSec: number): Observation {
      frame += 1;
      const cam = fpvCameraFromDrone(pose.pos, pose.yaw, 16 / 9);
      const eye = cam.position as V3;
      const contacts: RawContact[] = [];
      for (const it of items) {
        const r = visible(eye, cam, it.truth.world, range);
        if (r === null) continue;
        const decoy = it.truth.category === 'decoy';
        if (decoy && hashSeed(`${it.truth.id}:${frame}`) % 3 !== 0) continue;
        const jitter = ((hashSeed(`${it.truth.id}:e:${frame}`) % 1000) / 1000 - 0.5) * 0.2;
        const est: V3 = [it.truth.world[0] + jitter, it.truth.world[1] - jitter, it.truth.world[2]];
        contacts.push({
          id: `c${frame}-${it.truth.id}`,
          tSec,
          classId: it.reportAs,
          confidence: it.confidence,
          estimate: est,
          range: r,
          bearing: unit([it.truth.world[0] - eye[0], it.truth.world[1] - eye[1], it.truth.world[2] - eye[2]]),
          buildingId: it.truth.buildingId,
          truth: { targetId: it.truth.id, trueClass: it.truth.classId, kind: decoy ? 'fp' : 'tp' },
        });
      }
      for (const g of ghosts) {
        if (g.left <= 0) continue;
        const r = visible(eye, cam, g.at, 30);
        if (r === null) continue;
        g.left -= 1;
        contacts.push({
          id: `g${frame}`,
          tSec,
          classId: g.classId,
          confidence: g.confidence,
          estimate: g.at,
          range: r,
          bearing: unit([g.at[0] - eye[0], g.at[1] - eye[1], g.at[2] - eye[2]]),
          buildingId: null,
          truth: { targetId: 'ghost', trueClass: g.classId, kind: 'fp' },
        });
      }
      for (const [, list] of cells) {
        for (const c of list) {
          if (c.seen) continue;
          if (Math.hypot(c.center[0] - eye[0], c.center[1] - eye[1]) > range) continue;
          if (visible(eye, cam, c.center, range) !== null) c.seen = true;
        }
      }
      return { tSec, contacts: contacts.map((c) => withSensorFrame(c, pose)) };
    },
    truthClearStatus(scope, category): ClearReport {
      const occupied = items.filter((it) => it.truth.buildingId === (scope === 'outdoor' ? null : scope) && it.truth.category === category).length;
      if (scope === 'outdoor') {
        return { status: occupied ? 'occupied' : 'clear', seenEmpty: 1, occupied, unseen: 0, total: 1 };
      }
      const list = cells.get(scope) ?? [];
      const seen = list.filter((c) => c.seen).length;
      const unseen = list.length - seen;
      const status = occupied ? 'occupied' : unseen > 0 ? 'incomplete' : 'clear';
      return { status, seenEmpty: seen, occupied, unseen, total: list.length };
    },
    truthTargets(): TruthTarget[] {
      return items.map((it) => it.truth);
    },
  };
}

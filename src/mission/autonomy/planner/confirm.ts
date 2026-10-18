import { OBJECT_IDS, type TargetClass } from '../../../targets/types';
import type { Mission } from '../../types';
import type { ConfirmedContact, Contact, V3 } from '../contracts';

/**
 * Target confirmation. Contacts of a mission target class are associated
 * into tracks by estimate distance. A track gains a view for each frame
 * seen from a bearing at least `minSeparationDeg` from its other views and
 * within range; each view keeps its best confidence. A track confirms with
 * `minViews` views whose mean confidence meets the threshold, so one
 * high-confidence frame never confirms. A track that runs out of
 * re-view poses without confirming is rejected.
 */

export interface ConfirmOptions {
  minViews?: number;
  minSeparationDeg?: number;
  /** Views farther than this do not count, meters. */
  maxRange?: number;
  /** Contacts within this distance of a track estimate join it, meters. */
  associateM?: number;
}

export interface ConfirmView {
  /** Horizontal bearing angle from the camera to the target, radians. */
  angle: number;
  confidence: number;
  tSec: number;
  range: number;
}

export type TrackStatus = 'open' | 'confirmed' | 'rejected';

export interface ConfirmTrack {
  id: string;
  classId: TargetClass;
  estimate: V3;
  buildingId: string | null;
  views: ConfirmView[];
  frames: number;
  status: TrackStatus;
  last: Contact;
  best: Contact;
}

/** Classes the mission is looking for. Person targets want 'person'; object targets want any object class. */
export function targetClasses(mission: Mission): Set<TargetClass> {
  const out = new Set<TargetClass>();
  for (const t of mission.targets) {
    if (t.kind === 'person' || t.kind === 'clearance') out.add('person');
    if (t.kind === 'object') for (const id of OBJECT_IDS) out.add(id);
  }
  if (mission.missionType === 'safety_sweep') out.add('person');
  return out;
}

/** Confidence a class must reach: the target's own minimum, else the mission's. */
export function thresholdFor(mission: Mission, classId: TargetClass): number {
  const kind = classId === 'person' ? 'person' : 'object';
  const spec = mission.targets.find((t) => t.kind === kind) ?? mission.targets.find((t) => t.kind === 'clearance');
  return spec?.minConfidence ?? mission.requiredConfidence;
}

/** True when every person/object target has at least `minCount` confirmations. */
export function requirementsMet(mission: Mission, confirmed: readonly ConfirmedContact[]): boolean {
  const wanted = mission.targets.filter((t) => t.kind === 'person' || t.kind === 'object');
  if (!wanted.length) return false;
  return wanted.every((t) => {
    const n = confirmed.filter((c) => (t.kind === 'person' ? c.classId === 'person' : c.classId !== 'person')).length;
    return n >= Math.max(1, t.minCount);
  });
}

function angleDiff(a: number, b: number): number {
  const d = Math.abs(a - b) % (Math.PI * 2);
  return d > Math.PI ? Math.PI * 2 - d : d;
}

export class ConfirmTracker {
  readonly tracks: ConfirmTrack[] = [];
  readonly minViews: number;
  readonly minSeparation: number;
  readonly maxRange: number;
  readonly associateM: number;
  private readonly classes: Set<TargetClass>;
  private nextId = 0;

  constructor(
    readonly mission: Mission,
    options: ConfirmOptions = {},
  ) {
    this.minViews = options.minViews ?? 3;
    this.minSeparation = ((options.minSeparationDeg ?? 25) * Math.PI) / 180;
    this.maxRange = options.maxRange ?? 9;
    this.associateM = options.associateM ?? 1.5;
    this.classes = targetClasses(mission);
  }

  wants(classId: TargetClass): boolean {
    return this.classes.has(classId);
  }

  /** Track a contact belongs to, by class and estimate distance. */
  find(contact: Pick<Contact, 'classId' | 'estimate'>, radius = this.associateM): ConfirmTrack | null {
    let best: ConfirmTrack | null = null;
    let bestD = radius;
    for (const track of this.tracks) {
      if (track.classId !== contact.classId) continue;
      const d = Math.hypot(
        track.estimate[0] - contact.estimate[0],
        track.estimate[1] - contact.estimate[1],
        track.estimate[2] - contact.estimate[2],
      );
      if (d <= bestD) {
        bestD = d;
        best = track;
      }
    }
    return best;
  }

  /** Add a contact. Returns its track, or null when the class is not a mission target. */
  ingest(contact: Contact): ConfirmTrack | null {
    if (!this.wants(contact.classId)) return null;
    let track = this.find(contact);
    if (!track) {
      track = {
        id: `trk-${this.nextId++}`,
        classId: contact.classId,
        estimate: [contact.estimate[0], contact.estimate[1], contact.estimate[2]],
        buildingId: contact.buildingId,
        views: [],
        frames: 0,
        status: 'open',
        last: contact,
        best: contact,
      };
      this.tracks.push(track);
    }
    track.frames += 1;
    track.last = contact;
    if (contact.confidence > track.best.confidence) track.best = contact;
    if (track.status !== 'open') return track;
    // Running mean of the estimate.
    const w = 1 / track.frames;
    for (let i = 0; i < 3; i++) track.estimate[i] = track.estimate[i]! * (1 - w) + contact.estimate[i]! * w;
    if (contact.range <= this.maxRange) {
      const angle = Math.atan2(contact.bearing[1], contact.bearing[0]);
      const near = track.views.find((v) => angleDiff(v.angle, angle) < this.minSeparation);
      if (near) {
        if (contact.confidence > near.confidence) near.confidence = contact.confidence;
      } else {
        track.views.push({ angle, confidence: contact.confidence, tSec: contact.tSec, range: contact.range });
      }
    }
    return track;
  }

  meanConfidence(track: ConfirmTrack): number {
    if (!track.views.length) return 0;
    return track.views.reduce((s, v) => s + v.confidence, 0) / track.views.length;
  }

  /** Confirm when enough distinct views agree. Never rejects on its own. */
  evaluate(track: ConfirmTrack): TrackStatus {
    if (track.status !== 'open') return track.status;
    if (track.views.length >= this.minViews && this.meanConfidence(track) >= thresholdFor(this.mission, track.classId)) {
      track.status = 'confirmed';
    }
    return track.status;
  }

  reject(track: ConfirmTrack): void {
    if (track.status === 'open') track.status = 'rejected';
  }

  toConfirmed(track: ConfirmTrack, tSec: number): ConfirmedContact {
    return {
      contactId: track.best.id,
      classId: track.classId,
      confidence: this.meanConfidence(track),
      views: track.views.length,
      estimate: [track.estimate[0], track.estimate[1], track.estimate[2]],
      tSec,
      buildingId: track.buildingId,
      truth: track.best.truth,
    };
  }
}

/**
 * Re-view poses around an estimate (`center`, in the caller's frame): bearings at least `minSepDeg` from
 * every existing view and from each other. `accept` filters candidates
 * (reachable, line of sight). Returns up to `count` poses, nearest first.
 */
export function confirmViewpoints(
  track: ConfirmTrack,
  center: V3,
  from: V3,
  radius: number,
  z: number,
  count: number,
  accept: (p: V3) => boolean,
  minSepDeg = 40,
): V3[] {
  const sep = (minSepDeg * Math.PI) / 180;
  const used = track.views.map((v) => v.angle);
  const out: V3[] = [];
  const cands: { p: V3; angle: number; d: number }[] = [];
  for (let k = 0; k < 24; k++) {
    const a = (k / 24) * Math.PI * 2;
    // A camera at angle a around the target looks along bearing a + pi.
    const p: V3 = [center[0] + Math.cos(a) * radius, center[1] + Math.sin(a) * radius, z];
    const bearing = Math.atan2(-Math.sin(a), -Math.cos(a));
    cands.push({ p, angle: bearing, d: Math.hypot(p[0] - from[0], p[1] - from[1]) });
  }
  cands.sort((a, b) => a.d - b.d);
  for (const c of cands) {
    if (out.length >= count) break;
    if (used.some((u) => angleDiff(u, c.angle) < sep)) continue;
    if (!accept(c.p)) continue;
    used.push(c.angle);
    out.push(c.p);
  }
  return out;
}

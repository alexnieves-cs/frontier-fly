import { OBJECT_IDS, type TargetClass } from '../../../targets/types';
import type { Mission } from '../../types';
import type { ConfirmedContact, Contact, V3 } from '../contracts';

/**
 * Target confirmation. Contacts of a mission target class are associated
 * into tracks by estimate distance. A track gains a view for each frame
 * seen from a bearing at least `minSeparationDeg` from its other views and
 * within range; each view keeps its best confidence. A track confirms with
 * `minViews` distinct views whose best confidences average to the threshold, so one
 * high-confidence frame never confirms. A track that runs out of
 * re-view poses without confirming is closed: rejected when its views score
 * no better than a lookalike does, else left unresolved. An unresolved track
 * is a probable target the drone could not confirm, and it still blocks a
 * clear.
 *
 * People move, so a person track's estimate follows its recent contacts and
 * its association gate grows with the time since it was last seen, at walking
 * speed: a walker seen again across the room joins its own track instead of
 * starting a fresh one with no views.
 */

export interface ConfirmOptions {
  minViews?: number;
  minSeparationDeg?: number;
  /** Views farther than this do not count, meters. */
  maxRange?: number;
  /** Contacts within this distance of a track estimate join it, meters. */
  associateM?: number;
  /**
   * Range gain for outdoor tracks (no building), whose views come from the
   * gimbal lens. Their views count out to maxRange times this. Default 1.
   */
  outdoorRangeScale?: number;
  /**
   * A track closed below the threshold is rejected only when its mean confidence is under this;
   * otherwise it is unresolved. Default UNRESOLVED_FLOOR.
   */
  unresolvedFloor?: number;
}

/**
 * Mean confidence above which an unconfirmed track is not explained away as a lookalike. Set from
 * the dev block 240001-240240 (docs/REPORT.md): decoy tracks closed at 0.48 to 0.68 there, the real
 * people the drone failed to confirm at 0.62 to 0.80. A different detector needs its own value, as
 * it needs its own confirm threshold.
 */
export const UNRESOLVED_FLOOR = 0.7;

/** Walking speed bound for growing a person track's association gate, m/s. */
export const PERSON_WALK_MPS = 1.2;
/** Largest gate a person track grows to, meters. */
export const PERSON_GATE_MAX_M = 4;
/** A live person track's association gate after `sinceSec` unseen, meters. */
export function personGate(radius: number, sinceSec: number): number {
  return Math.min(Math.max(radius, PERSON_GATE_MAX_M), radius + PERSON_WALK_MPS * sinceSec);
}

/** Weight of the newest contact in a person track's estimate, at least. */
export const PERSON_ESTIMATE_GAIN = 0.3;

export interface ConfirmView {
  /** Horizontal bearing angle from the camera to the target, radians. */
  angle: number;
  confidence: number;
  tSec: number;
  range: number;
}

/** 'merged': a person track folded into another live track of the same walker; it holds no evidence of its own. */
export type TrackStatus = 'open' | 'confirmed' | 'rejected' | 'unresolved' | 'merged';

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
  /** When the track last gained a contact, seconds. */
  lastSec: number;
  /** Frame times of its contacts (person tracks): two tracks seen in one frame are two people. */
  seenAt: number[];
}

/** A track that can still gain evidence. */
export const isLive = (t: ConfirmTrack): boolean => t.status === 'open' || t.status === 'unresolved';

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

export function angleDiff(a: number, b: number): number {
  const d = Math.abs(a - b) % (Math.PI * 2);
  return d > Math.PI ? Math.PI * 2 - d : d;
}

export class ConfirmTracker {
  readonly tracks: ConfirmTrack[] = [];
  readonly minViews: number;
  readonly minSeparation: number;
  readonly maxRange: number;
  readonly associateM: number;
  readonly outdoorRangeScale: number;
  readonly unresolvedFloor: number;
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
    this.outdoorRangeScale = options.outdoorRangeScale ?? 1;
    this.unresolvedFloor = options.unresolvedFloor ?? UNRESOLVED_FLOOR;
    this.classes = targetClasses(mission);
  }

  wants(classId: TargetClass): boolean {
    return this.classes.has(classId);
  }

  /**
   * Track a contact belongs to, by class and estimate distance. A person track's gate grows by
   * PERSON_WALK_MPS for every second since it was last seen (up to PERSON_GATE_MAX_M) when the
   * contact carries a time, while the track is live (open or unresolved); the nearest track by
   * distance over its gate wins. A rejected track keeps the plain radius, so a person walking past
   * a rejected lookalike is not swallowed by it.
   */
  find(contact: Pick<Contact, 'classId' | 'estimate'> & { tSec?: number }, radius = this.associateM): ConfirmTrack | null {
    let best: ConfirmTrack | null = null;
    let bestScore = 1;
    for (const track of this.tracks) {
      if (track.classId !== contact.classId || track.status === 'merged') continue;
      const d = Math.hypot(
        track.estimate[0] - contact.estimate[0],
        track.estimate[1] - contact.estimate[1],
        track.estimate[2] - contact.estimate[2],
      );
      const since = contact.tSec === undefined ? 0 : Math.max(0, contact.tSec - track.lastSec);
      const gate = track.classId === 'person' && isLive(track) ? personGate(radius, since) : radius;
      if (d / gate <= bestScore) {
        bestScore = d / gate;
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
        lastSec: contact.tSec,
        seenAt: [],
      };
      this.tracks.push(track);
    }
    track.frames += 1;
    track.last = contact;
    track.lastSec = contact.tSec;
    if (track.classId === 'person' && track.seenAt[track.seenAt.length - 1] !== contact.tSec) track.seenAt.push(contact.tSec);
    if (contact.confidence > track.best.confidence) track.best = contact;
    if (!isLive(track)) return track;
    // Running mean of the estimate; a person's leans on the newest contacts, so it follows a walker.
    const w = track.classId === 'person' ? Math.max(1 / track.frames, PERSON_ESTIMATE_GAIN) : 1 / track.frames;
    for (let i = 0; i < 3; i++) track.estimate[i] = track.estimate[i]! * (1 - w) + contact.estimate[i]! * w;
    const maxRange = track.buildingId === null ? this.maxRange * this.outdoorRangeScale : this.maxRange;
    if (contact.range <= maxRange) {
      const angle = Math.atan2(contact.bearing[1], contact.bearing[0]);
      const near = track.views.find((v) => angleDiff(v.angle, angle) < this.minSeparation);
      if (near) {
        if (contact.confidence > near.confidence) near.confidence = contact.confidence;
      } else {
        track.views.push({ angle, confidence: contact.confidence, tSec: contact.tSec, range: contact.range });
      }
    }
    if (track.classId === 'person') this.absorbWalker(track);
    return track;
  }

  /**
   * A walker that crossed out of its track's gate while unseen starts a second track. Once the
   * newest track comes back within reach of an older live one, and the two were never seen in the
   * same frame, they are one person: the older track's views and frames fold into this one.
   */
  private absorbWalker(track: ConfirmTrack): void {
    const t = track.lastSec;
    for (const other of this.tracks) {
      if (other === track || other.classId !== 'person' || !isLive(other)) continue;
      const d = Math.hypot(track.estimate[0] - other.estimate[0], track.estimate[1] - other.estimate[1], track.estimate[2] - other.estimate[2]);
      if (d > personGate(this.associateM, Math.max(0, t - other.lastSec))) continue;
      const seen = new Set(other.seenAt);
      if (track.seenAt.some((s) => seen.has(s))) continue;
      for (const v of other.views) {
        const near = track.views.find((w) => angleDiff(w.angle, v.angle) < this.minSeparation);
        if (!near) track.views.push({ ...v });
        else if (v.confidence > near.confidence) near.confidence = v.confidence;
      }
      track.frames += other.frames;
      track.seenAt = [...other.seenAt, ...track.seenAt].sort((a, b) => a - b);
      if (other.best.confidence > track.best.confidence) track.best = other.best;
      track.buildingId ??= other.buildingId;
      other.status = 'merged';
    }
  }

  /** Mean of the best `minViews` view confidences (all views when fewer). */
  meanConfidence(track: ConfirmTrack): number {
    if (!track.views.length) return 0;
    const best = track.views
      .map((v) => v.confidence)
      .sort((a, b) => b - a)
      .slice(0, this.minViews);
    return best.reduce((s, c) => s + c, 0) / best.length;
  }

  /** Confirm when enough distinct views agree; an unresolved track can still confirm. Never rejects on its own. */
  evaluate(track: ConfirmTrack): TrackStatus {
    if (!isLive(track)) return track.status;
    if (track.views.length >= this.minViews && this.meanConfidence(track) >= thresholdFor(this.mission, track.classId)) {
      track.status = 'confirmed';
    }
    return track.status;
  }

  reject(track: ConfirmTrack): void {
    if (track.status === 'open') track.status = 'rejected';
  }

  /**
   * Closes a track that ran out of re-views without confirming: rejected when its views score no
   * better than a lookalike's (under unresolvedFloor), else unresolved. `neverReject` keeps it
   * unresolved whatever it scored: its confidence says nothing (seen through smoke).
   */
  close(track: ConfirmTrack, neverReject = false): TrackStatus {
    if (track.status === 'open') track.status = neverReject || this.meanConfidence(track) >= this.unresolvedFloor ? 'unresolved' : 'rejected';
    return track.status;
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
  const rejected = new Set<number>();
  for (let i = 0; i < cands.length && out.length < count; i++) {
    const c = cands[i]!;
    if (used.some((u) => angleDiff(u, c.angle) < sep)) continue;
    if (!accept(c.p)) {
      rejected.add(i);
      continue;
    }
    used.push(c.angle);
    out.push(c.p);
  }
  // Too few fresh bearings: re-view old ones closer, which can raise their best confidence.
  // The weakest views first: they hold the mean down.
  const mine = out.map((p) => Math.atan2(center[1] - p[1], center[0] - p[0]));
  const weakness = (angle: number): number => {
    let best = Infinity;
    for (const v of track.views) if (angleDiff(v.angle, angle) < sep / 2 && v.confidence < best) best = v.confidence;
    return best;
  };
  const order = cands.map((_, i) => i).sort((a, b) => weakness(cands[a]!.angle) - weakness(cands[b]!.angle) || a - b);
  for (const i of order) {
    if (out.length >= count) break;
    const c = cands[i]!;
    if (rejected.has(i) || out.includes(c.p) || mine.some((u) => angleDiff(u, c.angle) < sep)) continue;
    if (!accept(c.p)) continue;
    mine.push(c.angle);
    out.push(c.p);
  }
  return out;
}

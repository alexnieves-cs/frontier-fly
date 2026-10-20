import type { ClearScope, V3 } from '../contracts';
import { angleDiff, confirmViewpoints, requirementsMet, type ConfirmTrack } from '../planner/confirm';
import { senseBuildings } from '../planner/coverage';
import {
  cruisePath,
  cruiseZ,
  headingTo,
  len,
  operable,
  sub,
} from '../planner/geometry';
import { toLocal, toWorld, floorOf, INDOOR_FLY_Z } from '../planner/indoor';
import { observeOpenings } from '../planner/inspect';
import { type VehicleCommand } from '../vehicle';
import {
  CONFIRM_DWELL,
  CONFIRM_DWELL_DARK,
  CONFIRM_ROUNDS,
  type Job,
  type Waypoint,
  centerDist,
  fmt,
} from './shared';
import { RunnerBase } from './base';

/** Outdoor search: the coverage sweep, periodic detection, and multi-bearing confirmation of contacts. */
export abstract class SearchPhases extends RunnerBase {
  // ----------------------------------------------------------- coverage

  protected startCoverage(): void {
    const wps = this.coverage.waypoints;
    if (!wps.length) {
      this.coverageDone = true;
      this.log('warn', 'No operable coverage lanes inside the geofence.');
      this.continueMission();
      return;
    }
    const pos = this.vehicle.pos;
    const climb: Waypoint[] = [];
    if (this.covIndex === 0 && pos[2] < cruiseZ(this.air, pos[0], pos[1], this.agl) - 1) {
      climb.push({
        pos: [pos[0], pos[1], cruiseZ(this.air, pos[0], pos[1], this.agl)],
        tol: 0.6,
      });
      this.log('action', `Takeoff. Climbing to ${fmt(this.agl, 0)} m AGL.`);
    }
    const from = climb.length ? climb[0]!.pos : pos;
    const target = wps[this.covIndex]!;
    const link = cruisePath(this.air, from, { x: target[0], y: target[1] }, this.agl).slice(0, -1);
    const lead: Waypoint[] = [...climb, ...link.map((p) => ({ pos: p }))];
    const lane: Waypoint[] = wps.slice(this.covIndex).map((p) => ({ pos: p }));
    const base = this.covIndex;
    const inner = this.followJob(
      'coverage',
      [...lead, ...lane],
      () => {
        this.covIndex = wps.length;
        this.coverageDone = true;
        this.log('info', `Coverage complete: ${fmt(this.coverage.lengthM, 0)} m of lanes, ${this.discovered.size} building(s) found.`);
        this.continueMission();
      },
      1.5,
    );
    const job: Job = {
      phase: 'coverage',
      plan: () => inner.plan(),
      tick: (): VehicleCommand => {
        const cmd = inner.tick();
        // Keep the resume index in step with the lane waypoints already passed.
        const left = inner.plan().length;
        const passed = lead.length + lane.length - left - lead.length;
        if (passed > 0) this.covIndex = Math.max(this.covIndex, Math.min(wps.length, base + passed));
        return cmd;
      },
    };
    if (this.covIndex > 0) this.log('action', `Resuming coverage at waypoint ${this.covIndex + 1}/${wps.length}.`);
    else this.log('action', 'Starting area coverage.');
    this.setJob(job);
  }

  /** Decide what to do next once a building, confirmation, or lane set is finished. */
  protected continueMission(): void {
    if (this.finished || this.returning) return;
    if (this.findMission() && requirementsMet(this.mission, this.confirmed)) {
      this.startReturn('all required targets confirmed');
      return;
    }
    if (this.pending.length) {
      const pos = this.vehicle.pos;
      this.pending.sort((a, b) => centerDist(a, pos) - centerDist(b, pos) || a.id.localeCompare(b.id));
      const next = this.pending.shift()!;
      this.startBuilding(next);
      return;
    }
    if (!this.coverageDone) {
      this.startCoverage();
      return;
    }
    if (this.mission.missionType === 'safety_sweep' && !this.outdoorClearChecked) {
      this.outdoorClearChecked = true;
      this.declareIfClear('outdoor');
    }
    this.startReturn(this.findMission() ? 'search complete, targets not all found' : 'sweep complete');
  }

  protected findMission(): boolean {
    return this.mission.missionType === 'find_person' || this.mission.missionType === 'find_objects';
  }

  protected sweptCategories(): ('person' | 'object')[] {
    const out: ('person' | 'object')[] = ['person'];
    if (this.mission.targets.some((t) => t.kind === 'object')) out.push('object');
    return out;
  }

  protected declareIfClear(scope: ClearScope): boolean {
    const reports = this.sweptCategories().map((c) => ({
      c,
      r: this.source.clearStatus(scope, c),
    }));
    const label = scope === 'outdoor' ? 'Outdoor area' : (this.world.buildings.find((b) => b.id === scope)?.label ?? scope);
    if (reports.every(({ r }) => r.status === 'clear')) {
      this.declaredClear.push({ scope, tSec: this.t });
      const r = reports[0]!.r;
      this.log('action', `${label} confirmed clear (${r.seenEmpty}/${r.total} cells seen empty).`);
      return true;
    }
    const bad = reports.find(({ r }) => r.status !== 'clear')!;
    if (bad.r.status === 'occupied') {
      this.log('warn', `${label} not clear: ${bad.r.occupied} cell(s) occupied for ${bad.c}.`);
    } else {
      this.log('warn', `${label} incomplete: ${bad.r.unseen} of ${bad.r.total} cells unseen for ${bad.c}. Not declaring clear.`);
    }
    return false;
  }

  // ----------------------------------------------------------- detection

  protected detect(): void {
    const pose = this.vehicle.pose();
    // Building discovery by camera footprint and line of sight.
    if (!this.inside) {
      for (const b of senseBuildings(pose.pos, this.world, this.physics, this.discovered)) {
        this.discovered.add(b.id);
        this.pending.push(b);
        this.log('info', `${b.label} discovered ${fmt(centerDist(b, pose.pos), 0)} m away.`);
      }
      if (this.phase === 'coverage' && this.pending.length && !this.returning) {
        this.continueMission();
        return;
      }
    }
    // Openings seen while near the building.
    const v = this.visit;
    if (v && (this.phase === 'approach' || this.phase === 'inspect') && !this.inside) {
      for (const o of observeOpenings(pose.pos, pose.yaw, v.b, this.physics)) {
        if (!v.observed.has(o.id)) {
          v.observed.set(o.id, o);
          this.log(
            'info',
            `Observed opening ${o.id} (${o.kind.replace('_', ' ')}, ${fmt(o.width, 2)} x ${fmt(o.height, 2)} m) on ${v.b.label}.`,
          );
        }
      }
    }
    if (v && this.inside && v.vis) v.vis.observe(toLocal(v.grid, pose.pos), pose.yaw);

    const obs = this.source.observe(pose, this.t);
    for (const c of obs.contacts) {
      this.recent.push(c);
      if (this.recent.length > 60) this.recent.shift();
      this.tracker.ingest(c);
    }
    for (const track of this.tracker.tracks) {
      if (track.status !== 'open') continue;
      if (this.tracker.evaluate(track) === 'confirmed') {
        const cc = this.tracker.toConfirmed(track, this.t);
        this.confirmed.push(cc);
        this.log(
          'action',
          `Confirmed ${track.classId} at ${this.where(cc.estimate)} from ${cc.views} views, mean confidence ${fmt(cc.confidence, 2)}.`,
        );
        if (this.confirmTrack === track) this.endConfirm();
      }
    }
    if (this.finished || this.returning) return;
    // Start a confirmation when a mission-class contact appears.
    if (this.phase === 'coverage' || this.phase === 'indoor_explore') {
      for (const track of this.tracker.tracks) {
        if (track.status !== 'open' || (this.attempted.get(track.id) ?? 0) > this.t) continue;
        const indoor = this.phase === 'indoor_explore';
        if (indoor && v && track.buildingId !== v.b.id) continue;
        if (!indoor && track.buildingId !== null) continue;
        this.startConfirm(track);
        break;
      }
    }
  }

  // ----------------------------------------------------------- confirm

  protected startConfirm(track: ConfirmTrack): void {
    this.attempted.set(track.id, Infinity);
    const need = Math.max(2, this.tracker.minViews - track.views.length + 1);
    const pos = this.vehicle.pos;
    const v = this.visit;
    const wps: Waypoint[] = [];
    const flown: number[] = [];
    if (this.inside && v) {
      const grid = v.grid;
      const est = toLocal(grid, track.estimate);
      const f = floorOf(grid, est[2]);
      const z = f * grid.H + INDOOR_FLY_Z;
      let cur = toLocal(grid, pos);
      let views: V3[] = [];
      // One spare bearing indoors: furniture or a nearby object often hides the target from one of
      // them. A target in a corner leaves a narrow arc, so the spacing narrows toward the tracker's own.
      const accept = (p: V3) => v.nav.canView(p, est) && v.nav.path(cur, p) !== null;
      // Bearings flown before that added no view (the target hidden or mislabeled there) are not retried.
      const tried = this.triedBearings.get(track.id) ?? [];
      const seenFrom = { ...track, views: [...track.views, ...tried.map((angle) => ({ angle, confidence: 1, tSec: 0, range: 0 }))] };
      search: for (const sep of [40, 27]) {
        const radii = track.classId === 'person' ? [2.2, 2.8, 1.6] : v.sensing?.lookCone != null ? [1.8, 2.2, 2.6] : [2.4, 2.0, 2.8, 1.8];
        for (const r of radii) {
          views = confirmViewpoints(seenFrom, est, cur, r, z, need + 1, accept, sep);
          if (views.length >= need) break search;
        }
      }
      // On a map still being built, too few bearings now usually means more later: wait for the
      // map to grow rather than spend a round. The last look before leaving takes what there is.
      if (v.sensing && views.length < need && !this.finalLook.has(track.id)) views = [];
      for (const vp of views) {
        const path = v.nav.path(cur, vp);
        if (!path) continue;
        for (const p of path.slice(0, -1)) wps.push({ pos: toWorld(grid, p) });
        const at = toWorld(grid, vp);
        flown.push(Math.atan2(est[1] - vp[1], est[0] - vp[0]));
        wps.push({
          pos: at,
          yaw: headingTo(at, track.estimate),
          // Lit by the flashlight alone, each frame is a weaker look: hold longer for the same certainty.
          dwell: v.sensing?.lookCone != null ? CONFIRM_DWELL_DARK : CONFIRM_DWELL,
        });
        cur = vp;
      }
      const policy = this.policyRun;
      this.resumeAfterConfirm = () => (policy && this.policyRun === policy ? policy.resume() : this.exploreNext());
    } else {
      const z = cruiseZ(this.air, track.estimate[0], track.estimate[1], this.agl);
      const views = confirmViewpoints(track, track.estimate, pos, 8, z, need, (p) => {
        if (!operable(this.air, p[0], p[1])) return false;
        const d = sub(track.estimate, p);
        const hit = this.physics.cast(p[0], p[1], p[2], d[0], d[1], d[2], len(d));
        return !hit || hit.dist > len(d) - 1;
      });
      let cur: V3 = pos;
      for (const vp of views) {
        for (const p of cruisePath(this.air, cur, { x: vp[0], y: vp[1] }, this.agl).slice(0, -1)) wps.push({ pos: p });
        wps.push({
          pos: vp,
          yaw: headingTo(vp, track.estimate),
          dwell: CONFIRM_DWELL,
        });
        cur = vp;
      }
      this.resumeAfterConfirm = () => this.startCoverage();
    }
    if (!wps.some((w) => w.dwell)) {
      // Nowhere to re-view from yet. Keep the track open and try again later.
      this.attempted.set(track.id, this.t + 6);
      const resume = this.resumeAfterConfirm;
      this.resumeAfterConfirm = null;
      resume?.();
      return;
    }
    this.confirmTrack = track;
    this.log(
      'action',
      `Contact ${track.classId} (${fmt(track.best.confidence, 2)}) at ${this.where(track.estimate)}. Re-viewing from ${wps.filter((w) => w.dwell).length} more bearing(s).`,
    );
    this.setJob(
      this.followJob(
        'confirm',
        wps,
        () => {
          const rounds = (this.confirmRounds.get(track.id) ?? 0) + 1;
          const empty = flown.filter((a) => !track.views.some((w) => angleDiff(w.angle, a) < this.tracker.minSeparation));
          if (empty.length) this.triedBearings.set(track.id, [...(this.triedBearings.get(track.id) ?? []), ...empty]);
          this.confirmRounds.set(track.id, rounds);
          if (track.status === 'open' && rounds < CONFIRM_ROUNDS) {
            this.attempted.set(track.id, this.t + 4);
            this.log(
              'info',
              `${track.classId} contact still unconfirmed (${track.views.length} view(s), ${fmt(this.tracker.meanConfidence(track), 2)}). Will look again.`,
            );
          } else if (track.status === 'open') {
            this.tracker.reject(track);
            this.log(
              'warn',
              `Rejected ${track.classId} contact: ${track.views.length} view(s), mean confidence ${fmt(this.tracker.meanConfidence(track), 2)} after re-views.`,
            );
          }
          this.endConfirm();
        },
        this.inside ? 0.2 : 1.5,
      ),
    );
  }

  protected endConfirm(): void {
    this.confirmTrack = null;
    const resume = this.resumeAfterConfirm;
    this.resumeAfterConfirm = null;
    if (this.findMission() && requirementsMet(this.mission, this.confirmed)) {
      this.startReturn('all required targets confirmed');
      return;
    }
    if (resume) resume();
  }
}

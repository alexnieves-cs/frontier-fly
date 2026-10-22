import type { ClearScope, DronePose, V3 } from '../contracts';
import { buildingAt, placeContact } from '../perception';
import type { ClearReport } from '../../../perception/seenEmpty';
import { angleDiff, confirmViewpoints, requirementsMet, type ConfirmTrack } from '../planner/confirm';
import { senseBuildings } from '../planner/coverage';
import {
  columnClear,
  cruisePath,
  cruiseZ,
  headingTo,
  len,
  operable,
  pointFree,
  segmentClear,
  sub,
} from '../planner/geometry';
import { toLocal, toWorld, floorOf, INDOOR_FLY_Z } from '../planner/indoor';
import { observeOpenings } from '../planner/inspect';
import { sampleHeight } from '../../../world/outdoor/terrain';
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

/**
 * Outdoor re-view poses, tried in order: height above the contact's ground and horizontal radius,
 * meters. The radius keeps the person inside the gimbal image with the nose pointed at them.
 * The last entry is cruise height, used when the low poses are blocked by trees.
 */
/** Indoor confirm-clear: share of the known-free voxels on each floor the look estimate must cover. */
export const CLEAR_MIN_LOOKED = 0.95;
/** ...and a floor with fewer known-free voxels than this (a quarter of a small room) was never really searched. */
export const CLEAR_MIN_FLOOR_VOXELS = 16;

/** A contact placed this close outside a footprint still counts as in the building: range error at a window. */
const CONTACT_FOOTPRINT_MARGIN_M = 0.5;

export const OUTDOOR_CONFIRM_LOOKS: readonly { agl: number; radius: number }[] = [
  { agl: 7, radius: 3.5 },
  { agl: 9.5, radius: 4.5 },
  { agl: Infinity, radius: 4.5 },
];

/** Outdoor search: the coverage sweep, periodic detection, and multi-bearing confirmation of contacts. */
export abstract class SearchPhases extends RunnerBase {
  // ----------------------------------------------------------- coverage

  protected startCoverage(): void {
    if (this.sar) {
      this.startProbabilitySearch();
      return;
    }
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

  /** Climb to cruise from wherever the drone is, when it is below it. */
  private climbToCruise(): Waypoint[] {
    const pos = this.vehicle.pos;
    const z = cruiseZ(this.air, pos[0], pos[1], this.agl);
    if (pos[2] >= z - 1) return [];
    return [{ pos: [pos[0], pos[1], z], tol: 0.6 }];
  }

  /**
   * One step of the probability search: ask the planner, then fly its first
   * legs, visit the building it picked, or finish. The job replans a few
   * meters before the end of its legs, so the drone never brakes between them.
   */
  protected startProbabilitySearch(): void {
    const sar = this.sar!;
    const pos = this.vehicle.pos;
    const climb = this.climbToCruise();
    if (climb.length && sar.replans === 0) this.log('action', `Takeoff. Climbing to ${fmt(this.agl, 0)} m AGL.`);
    const vel = this.vehicle.vel;
    const heading = Math.hypot(vel[0], vel[1]) > 0.5 ? Math.atan2(vel[1], vel[0]) : this.vehicle.yaw;
    const decision = sar.plan([pos[0], pos[1]], heading);
    if (sar.replans === 1) this.log('action', 'Starting the probability search.');
    if (decision.kind === 'done') {
      this.coverageDone = true;
      this.log('info', `Probability search complete: ${decision.reason}. Modelled probability of success ${fmt(sar.belief.pos() * 100, 0)}%.`);
      this.continueMission();
      return;
    }
    if (decision.kind === 'building') {
      const b = this.world.buildings.find((item) => item.id === decision.id)!;
      sar.visit(b.id);
      this.discovered.add(b.id);
      this.log('action', `${b.label} is now the most likely place to look: searching it (probability ${fmt((sar.prior.structure.get(b.id) ?? 0) * 100, 0)}% at the start).`);
      this.startBuilding(b);
      return;
    }
    const from: V3 = climb.length ? climb[0]!.pos : pos;
    const wps: Waypoint[] = [...climb];
    let cur = from;
    for (const [x, y] of decision.path) {
      for (const p of cruisePath(this.air, cur, { x, y }, this.agl, 6)) wps.push({ pos: p });
      cur = wps[wps.length - 1]!.pos;
    }
    if (!wps.length) {
      this.coverageDone = true;
      this.continueMission();
      return;
    }
    const end = wps[wps.length - 1]!.pos;
    const inner = this.followJob('coverage', wps, () => this.startProbabilitySearch(), 1.5);
    const climbing = climb.length > 0;
    let ticks = 0;
    this.setJob({
      phase: 'coverage',
      plan: () => inner.plan(),
      tick: (): VehicleCommand => {
        const p = this.vehicle.pos;
        ticks += 1;
        // Replan before the last leg ends rather than braking to a stop on it.
        if (!climbing && ticks > 2 && Math.hypot(end[0] - p[0], end[1] - p[1]) < 4 && inner.plan().length <= 1) {
          this.startProbabilitySearch();
          return this.job.tick();
        }
        return inner.tick();
      },
    });
  }

  /** Decide what to do next once a building, confirmation, or lane set is finished. */
  protected continueMission(): void {
    if (this.finished || this.returning) return;
    if (this.findMission() && requirementsMet(this.mission, this.confirmed)) {
      this.startReturn('all required targets confirmed');
      return;
    }
    if (this.pending.length && !(this.options.search?.buildingsLast && !this.coverageDone)) {
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

  /**
   * Confirm-clear from the drone's own evidence. A scope is clear when no contact of the category is
   * still open or confirmed there, and the camera has looked at enough of it by the drone's own
   * estimate: indoors, CLEAR_MIN_LOOKED of the known-free voxels on every floor of a map the
   * explorer finished (not cut short by its time slot); outdoors, every ground cell of the site map.
   * Nothing here knows where the targets are, so a person the detector missed is a false clear.
   * `occupied` counts the contacts that block the scope; `seenEmpty` the looked-at cells or voxels.
   */
  protected ownClear(scope: ClearScope, category: 'person' | 'object'): ClearReport {
    const where = scope === 'outdoor' ? null : scope;
    // A track keeps the building of its first contact, and one first seen from outside has none: place
    // those by where the track's estimate now is, so a person confirmed through a window still blocks.
    const buildingOf = (t: ConfirmTrack) => t.buildingId ?? buildingAt(this.world, t.estimate, CONTACT_FOOTPRINT_MARGIN_M);
    const blocking = this.tracker.tracks.filter(
      (t) => buildingOf(t) === where && (t.status === 'open' || t.status === 'confirmed') && (category === 'person') === (t.classId === 'person'),
    ).length;
    if (scope === 'outdoor') {
      const r = this.outdoorLook.report(category);
      return { ...r, occupied: blocking, status: blocking > 0 ? 'occupied' : r.status };
    }
    const v = this.visit?.b.id === scope ? this.visit : null;
    if (!v) return { status: 'incomplete', seenEmpty: 0, occupied: blocking, unseen: 0, total: 0 };
    let looked = 0;
    let free = 0;
    let enough = v.searchComplete;
    if (v.sensing) {
      for (const f of v.sensing.lookStats()) {
        looked += f.looked;
        free += f.free;
        if (f.free < CLEAR_MIN_FLOOR_VOXELS || f.looked < CLEAR_MIN_LOOKED * f.free) enough = false;
      }
    } else if (v.vis) {
      // Oracle reference: its own look estimate on the true floor plan.
      free = v.vis.cells.length;
      looked = Math.round(v.vis.coverage() * free);
      if (looked < CLEAR_MIN_LOOKED * free) enough = false;
    } else enough = false;
    const status = blocking > 0 ? 'occupied' : enough ? 'clear' : 'incomplete';
    return { status, seenEmpty: looked, occupied: blocking, unseen: free - looked, total: free };
  }

  protected declareIfClear(scope: ClearScope): boolean {
    const reports = this.sweptCategories().map((c) => ({
      c,
      r: this.ownClear(scope, c),
    }));
    const label = scope === 'outdoor' ? 'Outdoor area' : (this.world.buildings.find((b) => b.id === scope)?.label ?? scope);
    if (reports.every(({ r }) => r.status === 'clear')) {
      this.declaredClear.push({ scope, tSec: this.t });
      const r = reports[0]!.r;
      this.log('action', `${label} confirmed clear (${r.seenEmpty}/${r.total} cells looked at, no open contacts).`);
      return true;
    }
    const bad = reports.find(({ r }) => r.status !== 'clear')!;
    if (bad.r.status === 'occupied') {
      this.log('warn', `${label} not clear: ${bad.r.occupied} ${bad.c} contact(s) still open or confirmed there.`);
    } else {
      const floors = scope === 'outdoor' ? '' : (this.visit?.sensing?.lookStats().map((f) => `${f.looked}/${f.free}`).join(', ') ?? '');
      const why = scope === 'outdoor' ? '' : this.visit?.searchComplete ? ` (per floor ${floors})` : ` (search cut short; per floor ${floors})`;
      this.log('warn', `${label} incomplete: ${bad.r.seenEmpty} of ${bad.r.total} cells looked at for ${bad.c}${why}. Not declaring clear.`);
    }
    return false;
  }

  // ----------------------------------------------------------- detection

  /**
   * One camera frame. `sensed` is the true pose, and only the sensor models see it: the detector,
   * building discovery and opening measurement render or ray-cast from it. Everything they report
   * is placed with the drone's own pose (ownPose), as a real drone places what its camera sees.
   */
  protected detect(sensed: DronePose): void {
    const pose = this.ownPose();
    // Building discovery by camera footprint and line of sight.
    // The probability search knows the buildings from the map and schedules them itself.
    if (!this.inside && !this.sar) {
      for (const b of senseBuildings(sensed.pos, this.world, this.physics, this.discovered)) {
        this.discovered.add(b.id);
        this.pending.push(b);
        this.log('info', `${b.label} discovered ${fmt(centerDist(b, pose.pos), 0)} m away.`);
      }
      if (this.phase === 'coverage' && this.pending.length && !this.returning && !this.options.search?.buildingsLast) {
        this.continueMission();
        return;
      }
    }
    // Openings seen while near the building.
    const v = this.visit;
    if (v && (this.phase === 'approach' || this.phase === 'inspect') && !this.inside) {
      for (const seen of observeOpenings(sensed.pos, sensed.yaw, v.b, this.physics)) {
        const had = v.observed.has(seen.id);
        const o = this.openingSensor.measure(seen, sensed, pose);
        v.observed.set(o.id, o);
        if (!had) {
          this.log(
            'info',
            `Observed opening ${o.id} (${o.kind.replace('_', ' ')}, ${fmt(o.width, 2)} x ${fmt(o.height, 2)} m) on ${v.b.label}.`,
          );
        }
      }
    }
    // Oracle reference only: its visibility estimate runs on the true floor plan from the true pose.
    if (v && this.inside && v.vis) v.vis.observe(toLocal(v.grid, sensed.pos), sensed.yaw);
    // The drone's own record of where the camera has looked outdoors, for confirm-clear.
    if (!this.inside) this.outdoorLook.observe(pose);

    const obs = this.source.observe(sensed, this.t);
    // Inside a building everything the camera sees is in it; outside, a contact belongs to the footprint it was placed in.
    const placed = obs.contacts.map((raw) => {
      const c = placeContact(raw, pose);
      return { ...c, buildingId: this.inside ?? buildingAt(this.world, c.estimate, CONTACT_FOOTPRINT_MARGIN_M) };
    });
    if (this.sar && !this.inside) this.sar.observe(pose, placed, this.t);
    for (const c of placed) {
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
      const est = track.estimate;
      const ground = (x: number, y: number) => sampleHeight(this.world.outdoor.terrain, x, y);
      const sees = (p: V3) => {
        const d = sub(est, p);
        const hit = this.physics.cast(p[0], p[1], p[2], d[0], d[1], d[2], len(d));
        return !hit || hit.dist > len(d) - 1;
      };
      // Lowest re-view height with enough clear bearings: closer looks give the detector more pixels,
      // which matters most at night. A low pose needs free air around it and a clear column up to cruise.
      let views: V3[] = [];
      for (const look of OUTDOOR_CONFIRM_LOOKS) {
        const low = look.agl < this.agl;
        const z = low ? ground(est[0], est[1]) + look.agl : cruiseZ(this.air, est[0], est[1], this.agl);
        views = confirmViewpoints(track, est, pos, look.radius, z, need, (p) => {
          if (!operable(this.air, p[0], p[1])) return false;
          if (low && (p[2] - ground(p[0], p[1]) < this.air.minAgl + 0.5 || !pointFree(this.physics, p, 0.9))) return false;
          if (low && !columnClear(this.physics, p[0], p[1], p[2], cruiseZ(this.air, p[0], p[1], this.agl), 0.8)) return false;
          return sees(p);
        });
        if (views.length >= need) break;
      }
      let cur: V3 = pos;
      const cruiseAt = (p: V3): V3 => [p[0], p[1], cruiseZ(this.air, p[0], p[1], this.agl)];
      for (const vp of views) {
        const lowHop = cur[2] < cruiseAt(cur)[2] - 1 && segmentClear(this.physics, cur, vp, 0.6);
        if (!lowHop) {
          if (cur[2] < cruiseAt(cur)[2] - 1) wps.push({ pos: cruiseAt(cur), tol: 0.6 });
          const from = cur[2] < cruiseAt(cur)[2] - 1 ? cruiseAt(cur) : cur;
          for (const p of cruisePath(this.air, from, { x: vp[0], y: vp[1] }, this.agl).slice(0, -1)) wps.push({ pos: p });
          if (vp[2] < cruiseAt(vp)[2] - 1) wps.push({ pos: cruiseAt(vp), tol: 0.6 });
        }
        wps.push({
          pos: vp,
          yaw: headingTo(vp, est),
          dwell: CONFIRM_DWELL,
        });
        cur = vp;
      }
      // Back up to cruise before the search resumes.
      if (views.length && cur[2] < cruiseAt(cur)[2] - 1) wps.push({ pos: cruiseAt(cur), tol: 0.6 });
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

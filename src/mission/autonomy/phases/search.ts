import { inSmoke } from '../../../gen/hazards';
import type { ClearScope, DronePose, MissionBuilding, V3 } from '../contracts';
import { buildingAt, placeContact } from '../perception';
import type { ClearReport } from '../../../perception/seenEmpty';
import { angleDiff, confirmViewpoints, isLive, requirementsMet, thresholdFor, type ConfirmTrack } from '../planner/confirm';
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
import { sensorClearance, type SensorGrid } from '../../../sim/sensorMap';
import { VOXEL } from '../../../shared/constants';
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
import type { MissionCore } from './core';
import type { Pilot } from './pilot';

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

/** How far the last indoor look moves a contact placed in a wall to the nearest free point: about the range error at a wall. */
const FINAL_LOOK_SNAP_M = 1;

/** The point itself when the drone's own map has it free, else the nearest free voxel centre within maxM, or null. */
function nearestOpen(g: SensorGrid, floor: number, p: V3, maxM: number): V3 | null {
  if (sensorClearance(g, floor, p[0], p[1]) > 0) return p;
  const r = Math.ceil(maxM / VOXEL);
  const cx = Math.floor(p[0] / VOXEL);
  const cy = Math.floor(p[1] / VOXEL);
  let best: V3 | null = null;
  let bestD = maxM;
  for (let dy = -r; dy <= r; dy++) {
    for (let dx = -r; dx <= r; dx++) {
      const x = (cx + dx + 0.5) * VOXEL;
      const y = (cy + dy + 0.5) * VOXEL;
      const d = Math.hypot(x - p[0], y - p[1]);
      if (d <= bestD && sensorClearance(g, floor, x, y) > 0) {
        bestD = d;
        best = [x, y, p[2]];
      }
    }
  }
  return best;
}

export const OUTDOOR_CONFIRM_LOOKS: readonly { agl: number; radius: number }[] = [
  { agl: 7, radius: 3.5 },
  { agl: 9.5, radius: 4.5 },
  { agl: Infinity, radius: 4.5 },
];

/** Where the search phase hands over: to a building, home, or back to indoor exploration after a re-view. */
export interface SearchExits {
  startBuilding(b: MissionBuilding): void;
  startReturn(reason: string): void;
  /** The way back to whatever indoor exploration a confirmation interrupts. */
  resumeIndoor(): () => void;
}

/**
 * Outdoor search: the coverage sweep (lanes or probability search), periodic detection, multi-bearing
 * confirmation of contacts indoors and out, and confirm-clear from the drone's own evidence. Owns the
 * sweep's progress, the buildings found and waiting, and each track's confirmation history.
 */
export class SearchPhase {
  covIndex = 0;
  coverageDone = false;
  readonly discovered = new Set<string>();
  readonly pending: MissionBuilding[] = [];
  /** Earliest time each track may get another confirmation attempt. */
  private readonly attempted = new Map<string, number>();
  /** Per track: indoor re-view bearings (camera heading, rad) that added no view. */
  private readonly triedBearings = new Map<string, number[]>();
  private readonly confirmRounds = new Map<string, number>();
  /** Tracks already given a last look when exploration of their building ended. */
  private readonly finalLook = new Set<string>();
  private outdoorClearChecked = false;
  private confirmTrack: ConfirmTrack | null = null;
  /** What to run once a confirmation ends: back to the sweep, or back to indoor exploration. */
  private resumeAfterConfirm: (() => void) | null = null;

  constructor(
    private readonly c: MissionCore,
    private readonly pilot: Pilot,
    private readonly go: SearchExits,
  ) {}

  /**
   * An open contact in this building that has not had its last look gets one before the drone leaves:
   * true when that re-view started.
   */
  lastLook(buildingId: string): boolean {
    const open = this.c.tracker.tracks.find(
      (tr) => tr.status === 'open' && tr.buildingId === buildingId && this.attempted.get(tr.id) !== Infinity && !this.finalLook.has(tr.id),
    );
    if (!open) return false;
    this.finalLook.add(open.id);
    this.startConfirm(open);
    return true;
  }

  // ----------------------------------------------------------- coverage

  startCoverage(): void {
    if (this.c.sar) {
      this.startProbabilitySearch();
      return;
    }
    const wps = this.c.coverage.waypoints;
    if (!wps.length) {
      this.coverageDone = true;
      this.c.log('warn', 'No operable coverage lanes inside the geofence.');
      this.continueMission();
      return;
    }
    if (this.covIndex >= wps.length) {
      // Every lane is already flown (a re-view that started on the last waypoint resumes here): nothing
      // is left to cover. Reading the next waypoint used to throw and end the whole run.
      this.coverageDone = true;
      this.continueMission();
      return;
    }
    const pos = this.c.vehicle.pos;
    const climb: Waypoint[] = [];
    if (this.covIndex === 0 && pos[2] < cruiseZ(this.c.air, pos[0], pos[1], this.c.agl) - 1) {
      climb.push({
        pos: [pos[0], pos[1], cruiseZ(this.c.air, pos[0], pos[1], this.c.agl)],
        tol: 0.6,
      });
      this.c.log('action', `Takeoff. Climbing to ${fmt(this.c.agl, 0)} m AGL.`);
    }
    const from = climb.length ? climb[0]!.pos : pos;
    const target = wps[this.covIndex]!;
    const link = cruisePath(this.c.air, from, { x: target[0], y: target[1] }, this.c.agl).slice(0, -1);
    const lead: Waypoint[] = [...climb, ...link.map((p) => ({ pos: p }))];
    const lane: Waypoint[] = wps.slice(this.covIndex).map((p) => ({ pos: p }));
    const base = this.covIndex;
    const inner = this.pilot.followJob(
      'coverage',
      [...lead, ...lane],
      () => {
        this.covIndex = wps.length;
        this.coverageDone = true;
        this.c.log('info', `Coverage complete: ${fmt(this.c.coverage.lengthM, 0)} m of lanes, ${this.discovered.size} building(s) found.`);
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
    if (this.covIndex > 0) this.c.log('action', `Resuming coverage at waypoint ${this.covIndex + 1}/${wps.length}.`);
    else this.c.log('action', 'Starting area coverage.');
    this.c.setJob(job);
  }

  /**
   * One step of the probability search: ask the planner, then fly its first
   * legs, visit the building it picked, or finish. The job replans a few
   * meters before the end of its legs, so the drone never brakes between them.
   */
  startProbabilitySearch(): void {
    const sar = this.c.sar!;
    const pos = this.c.vehicle.pos;
    const climb = this.pilot.climbToCruise();
    if (climb.length && sar.replans === 0) this.c.log('action', `Takeoff. Climbing to ${fmt(this.c.agl, 0)} m AGL.`);
    const vel = this.c.vehicle.vel;
    const heading = Math.hypot(vel[0], vel[1]) > 0.5 ? Math.atan2(vel[1], vel[0]) : this.c.vehicle.yaw;
    const decision = sar.plan([pos[0], pos[1]], heading);
    if (sar.replans === 1) this.c.log('action', 'Starting the probability search.');
    if (decision.kind === 'done') {
      this.coverageDone = true;
      this.c.log('info', `Probability search complete: ${decision.reason}. Modelled probability of success ${fmt(sar.belief.pos() * 100, 0)}%.`);
      this.continueMission();
      return;
    }
    if (decision.kind === 'building') {
      const b = this.c.world.buildings.find((item) => item.id === decision.id)!;
      sar.visit(b.id);
      this.discovered.add(b.id);
      this.c.log('action', `${b.label} is now the most likely place to look: searching it (probability ${fmt((sar.prior.structure.get(b.id) ?? 0) * 100, 0)}% at the start).`);
      this.go.startBuilding(b);
      return;
    }
    const from: V3 = climb.length ? climb[0]!.pos : pos;
    const wps: Waypoint[] = [...climb];
    let cur = from;
    for (const [x, y] of decision.path) {
      for (const p of cruisePath(this.c.air, cur, { x, y }, this.c.agl, 6)) wps.push({ pos: p });
      cur = wps[wps.length - 1]!.pos;
    }
    if (!wps.length) {
      this.coverageDone = true;
      this.continueMission();
      return;
    }
    const end = wps[wps.length - 1]!.pos;
    const inner = this.pilot.followJob('coverage', wps, () => this.startProbabilitySearch(), 1.5);
    const climbing = climb.length > 0;
    let ticks = 0;
    this.c.setJob({
      phase: 'coverage',
      plan: () => inner.plan(),
      tick: (): VehicleCommand => {
        const p = this.c.vehicle.pos;
        ticks += 1;
        // Replan before the last leg ends rather than braking to a stop on it.
        if (!climbing && ticks > 2 && Math.hypot(end[0] - p[0], end[1] - p[1]) < 4 && inner.plan().length <= 1) {
          this.startProbabilitySearch();
          return this.c.job.tick();
        }
        return inner.tick();
      },
    });
  }

  /** Decide what to do next once a building, confirmation, or lane set is finished. */
  continueMission(): void {
    if (this.c.finished || this.c.returning) return;
    if (this.findMission() && requirementsMet(this.c.mission, this.c.confirmed)) {
      this.go.startReturn('all required targets confirmed');
      return;
    }
    if (this.pending.length && !(this.c.options.search?.buildingsLast && !this.coverageDone)) {
      const pos = this.c.vehicle.pos;
      this.pending.sort((a, b) => centerDist(a, pos) - centerDist(b, pos) || a.id.localeCompare(b.id));
      const next = this.pending.shift()!;
      this.go.startBuilding(next);
      return;
    }
    if (!this.coverageDone) {
      this.startCoverage();
      return;
    }
    if (this.c.mission.missionType === 'safety_sweep' && !this.outdoorClearChecked) {
      this.outdoorClearChecked = true;
      this.declareIfClear('outdoor');
    }
    this.go.startReturn(this.findMission() ? 'search complete, targets not all found' : 'sweep complete');
  }

  findMission(): boolean {
    return this.c.mission.missionType === 'find_person' || this.c.mission.missionType === 'find_objects';
  }

  sweptCategories(): ('person' | 'object')[] {
    const out: ('person' | 'object')[] = ['person'];
    if (this.c.mission.targets.some((t) => t.kind === 'object')) out.push('object');
    return out;
  }

  /**
   * Confirm-clear from the drone's own evidence. A scope is clear when no contact of the category is
   * still open, unresolved or confirmed there, and the camera has looked at enough of it by the drone's own
   * estimate: indoors, CLEAR_MIN_LOOKED of the known-free voxels on every floor of a map the
   * explorer finished (not cut short by its time slot); outdoors, every ground cell of the site map.
   * Nothing here knows where the targets are, so a person the detector missed is a false clear.
   * `occupied` counts the contacts that block the scope; `seenEmpty` the looked-at cells or voxels.
   */
  ownClear(scope: ClearScope, category: 'person' | 'object'): ClearReport {
    const where = scope === 'outdoor' ? null : scope;
    // A track keeps the building of its first contact, and one first seen from outside has none: place
    // those by where the track's estimate now is, so a person confirmed through a window still blocks.
    const buildingOf = (t: ConfirmTrack) => t.buildingId ?? buildingAt(this.c.world, t.estimate, CONTACT_FOOTPRINT_MARGIN_M);
    const blocking = this.c.tracker.tracks.filter(
      (t) => buildingOf(t) === where && (isLive(t) || t.status === 'confirmed') && (category === 'person') === (t.classId === 'person'),
    ).length;
    if (scope === 'outdoor') {
      const r = this.c.outdoorLook.report(category);
      return { ...r, occupied: blocking, status: blocking > 0 ? 'occupied' : r.status };
    }
    const v = this.c.visit?.b.id === scope ? this.c.visit : null;
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
      // A shut door with unsearched space behind it: nobody can say what is there.
      if (v.sensing.hazardAware) {
        const shut = v.sensing.unsearchedDoors();
        this.c.shutDoors.set(scope, shut);
        if (shut > 0) enough = false;
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

  declareIfClear(scope: ClearScope): boolean {
    const reports = this.sweptCategories().map((c) => ({
      c,
      r: this.ownClear(scope, c),
    }));
    const label = scope === 'outdoor' ? 'Outdoor area' : (this.c.world.buildings.find((b) => b.id === scope)?.label ?? scope);
    if (reports.every(({ r }) => r.status === 'clear')) {
      this.c.declaredClear.push({ scope, tSec: this.c.t });
      const r = reports[0]!.r;
      this.c.log('action', `${label} confirmed clear (${r.seenEmpty}/${r.total} cells looked at, no open contacts).`);
      return true;
    }
    const bad = reports.find(({ r }) => r.status !== 'clear')!;
    if (bad.r.status === 'occupied') {
      this.c.log('warn', `${label} not clear: ${bad.r.occupied} ${bad.c} contact(s) still open or confirmed there.`);
    } else if (scope !== 'outdoor' && (this.c.shutDoors.get(scope) ?? 0) > 0) {
      this.c.log('warn', `${label} not cleared: ${this.c.shutDoors.get(scope)} shut door(s) with unsearched space behind them. Not declaring clear.`);
    } else {
      const floors = scope === 'outdoor' ? '' : (this.c.visit?.sensing?.lookStats().map((f) => `${f.looked}/${f.free}`).join(', ') ?? '');
      const why = scope === 'outdoor' ? '' : this.c.visit?.searchComplete ? ` (per floor ${floors})` : ` (search cut short; per floor ${floors})`;
      this.c.log('warn', `${label} incomplete: ${bad.r.seenEmpty} of ${bad.r.total} cells looked at for ${bad.c}${why}. Not declaring clear.`);
    }
    return false;
  }

  // ----------------------------------------------------------- detection

  /**
   * One camera frame. `sensed` is the true pose, and only the sensor models see it: the detector,
   * building discovery and opening measurement render or ray-cast from it. Everything they report
   * is placed with the drone's own pose (ownPose), as a real drone places what its camera sees.
   */
  detect(sensed: DronePose): void {
    const pose = this.c.ownPose();
    // Building discovery by camera footprint and line of sight.
    // The probability search knows the buildings from the map and schedules them itself.
    if (!this.c.inside && !this.c.sar) {
      for (const b of senseBuildings(sensed.pos, this.c.world, this.c.physics, this.discovered)) {
        this.discovered.add(b.id);
        this.pending.push(b);
        this.c.log('info', `${b.label} discovered ${fmt(centerDist(b, pose.pos), 0)} m away.`);
      }
      if (this.c.phase === 'coverage' && this.pending.length && !this.c.returning && !this.c.options.search?.buildingsLast) {
        this.continueMission();
        return;
      }
    }
    // Openings seen while near the building.
    const v = this.c.visit;
    if (v && (this.c.phase === 'approach' || this.c.phase === 'inspect') && !this.c.inside) {
      for (const seen of observeOpenings(sensed.pos, sensed.yaw, v.b, this.c.physics)) {
        const had = v.observed.has(seen.id);
        const o = this.c.openingSensor.measure(seen, sensed, pose);
        v.observed.set(o.id, o);
        if (!had) {
          this.c.log(
            'info',
            `Observed opening ${o.id} (${o.kind.replace('_', ' ')}, ${fmt(o.width, 2)} x ${fmt(o.height, 2)} m) on ${v.b.label}.`,
          );
        }
      }
    }
    // Oracle reference only: its visibility estimate runs on the true floor plan from the true pose.
    if (v && this.c.inside && v.vis) v.vis.observe(toLocal(v.grid, sensed.pos), sensed.yaw);
    // The drone's own record of where the camera has looked outdoors, for confirm-clear.
    if (!this.c.inside) this.c.outdoorLook.observe(pose);

    const obs = this.c.source.observe(sensed, this.c.t);
    // Inside a building everything the camera sees is in it; outside, a contact belongs to the footprint it was placed in.
    const placed = obs.contacts.map((raw) => {
      const c = placeContact(raw, pose);
      return { ...c, buildingId: this.c.inside ?? buildingAt(this.c.world, c.estimate, CONTACT_FOOTPRINT_MARGIN_M) };
    });
    if (this.c.sar && !this.c.inside) this.c.sar.observe(pose, placed, this.c.t);
    for (const c of placed) {
      this.c.recent.push(c);
      if (this.c.recent.length > 60) this.c.recent.shift();
      this.c.tracker.ingest(c);
    }
    for (const track of this.c.tracker.tracks) {
      if (!isLive(track)) continue;
      if (this.c.tracker.evaluate(track) === 'confirmed') {
        const cc = this.c.tracker.toConfirmed(track, this.c.t);
        this.c.confirmed.push(cc);
        this.c.log(
          'action',
          `Confirmed ${track.classId} at ${this.c.where(cc.estimate)} from ${cc.views} views, mean confidence ${fmt(cc.confidence, 2)}.`,
        );
        if (this.confirmTrack === track) this.endConfirm();
      }
    }
    if (this.c.finished || this.c.returning) return;
    // Start a confirmation when a mission-class contact appears.
    if (this.c.phase === 'coverage' || this.c.phase === 'indoor_explore') {
      for (const track of this.c.tracker.tracks) {
        if (track.status !== 'open' || (this.attempted.get(track.id) ?? 0) > this.c.t) continue;
        const indoor = this.c.phase === 'indoor_explore';
        if (indoor && v && track.buildingId !== v.b.id) continue;
        if (!indoor && track.buildingId !== null) continue;
        this.startConfirm(track);
        break;
      }
    }
  }

  // ----------------------------------------------------------- confirm

  startConfirm(track: ConfirmTrack): void {
    this.attempted.set(track.id, Infinity);
    const need = Math.max(2, this.c.tracker.minViews - track.views.length + 1);
    const pos = this.c.vehicle.pos;
    const v = this.c.visit;
    const wps: Waypoint[] = [];
    const flown: number[] = [];
    if (this.c.inside && v) {
      const grid = v.grid;
      const placed = toLocal(grid, track.estimate);
      const f = floorOf(grid, placed[2]);
      // The last look before leaving re-views a contact placed in a wall or outside the mapped interior
      // (range error at a wall, a detection through a window) around the nearest free point instead:
      // no free, visible hover point surrounds a point in a wall, and a contact nobody can look at again
      // stays open and blocks the building's clear for good.
      const est = this.finalLook.has(track.id) && v.sensing ? nearestOpen(v.sensing, f, placed, FINAL_LOOK_SNAP_M) ?? placed : placed;
      const z = f * grid.H + INDOOR_FLY_Z;
      let cur = toLocal(grid, pos);
      let views: V3[] = [];
      // One spare bearing indoors: furniture or a nearby object often hides the target from one of
      // them. A target in a corner leaves a narrow arc, so the spacing narrows toward the tracker's own.
      const accept = (p: V3) => v.nav.canView(p, est) && v.nav.path(cur, p) !== null;
      // Bearings flown before that added no view (the target hidden or mislabeled there) are not retried.
      const tried = this.triedBearings.get(track.id) ?? [];
      const seenFrom = { ...track, views: [...track.views, ...tried.map((angle) => ({ angle, confidence: 1, tSec: 0, range: 0 }))] };
      const smoky = this.inSmoke(track);
      search: for (const sep of [40, 27]) {
        // A contact in smoke is re-viewed from closer first: every meter of smoke on the line of sight
        // costs contrast, whichever camera looks.
        const person = smoky ? [2.0, 1.6, 2.6, 3.2] : [3.2, 2.6, 2.0];
        const radii = track.classId === 'person' ? person : v.sensing?.lookCone != null ? [1.8, 2.2, 2.6] : [2.4, 2.0, 2.8, 1.8];
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
          yaw: headingTo(at, toWorld(grid, est)),
          // Lit by the flashlight alone, each frame is a weaker look: hold longer for the same certainty.
          dwell: v.sensing?.lookCone != null ? CONFIRM_DWELL_DARK : CONFIRM_DWELL,
        });
        cur = vp;
      }
      this.resumeAfterConfirm = this.go.resumeIndoor();
    } else {
      const est = track.estimate;
      const ground = (x: number, y: number) => sampleHeight(this.c.world.outdoor.terrain, x, y);
      const sees = (p: V3) => {
        const d = sub(est, p);
        const hit = this.c.physics.cast(p[0], p[1], p[2], d[0], d[1], d[2], len(d));
        return !hit || hit.dist > len(d) - 1;
      };
      // Lowest re-view height with enough clear bearings: closer looks give the detector more pixels,
      // which matters most at night. A low pose needs free air around it and a clear column up to cruise.
      let views: V3[] = [];
      for (const look of OUTDOOR_CONFIRM_LOOKS) {
        const low = look.agl < this.c.agl;
        const z = low ? ground(est[0], est[1]) + look.agl : cruiseZ(this.c.air, est[0], est[1], this.c.agl);
        views = confirmViewpoints(track, est, pos, look.radius, z, need, (p) => {
          if (!operable(this.c.air, p[0], p[1])) return false;
          if (low && (p[2] - ground(p[0], p[1]) < this.c.air.minAgl + 0.5 || !pointFree(this.c.physics, p, 0.9))) return false;
          if (low && !columnClear(this.c.physics, p[0], p[1], p[2], cruiseZ(this.c.air, p[0], p[1], this.c.agl), 0.8)) return false;
          return sees(p);
        });
        if (views.length >= need) break;
      }
      let cur: V3 = pos;
      const cruiseAt = (p: V3): V3 => [p[0], p[1], cruiseZ(this.c.air, p[0], p[1], this.c.agl)];
      for (const vp of views) {
        const lowHop = cur[2] < cruiseAt(cur)[2] - 1 && segmentClear(this.c.physics, cur, vp, 0.6);
        if (!lowHop) {
          if (cur[2] < cruiseAt(cur)[2] - 1) wps.push({ pos: cruiseAt(cur), tol: 0.6 });
          const from = cur[2] < cruiseAt(cur)[2] - 1 ? cruiseAt(cur) : cur;
          for (const p of cruisePath(this.c.air, from, { x: vp[0], y: vp[1] }, this.c.agl).slice(0, -1)) wps.push({ pos: p });
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
    if (!wps.some((w) => w.dwell) && this.c.inside && this.finalLook.has(track.id)) {
      // The last look found no free point in the drone's own map that sees the contact: it is not in
      // the space the drone searched. Rejecting it rather than leaving it open lets the clear rest on
      // the look estimate, which already requires the camera to have covered that space.
      this.c.tracker.reject(track);
      this.c.log('warn', `Rejected ${track.classId} contact: nowhere in the mapped interior sees it again (${track.views.length} view(s), best ${fmt(track.best.confidence, 2)}).`);
      const resume = this.resumeAfterConfirm;
      this.resumeAfterConfirm = null;
      resume?.();
      return;
    }
    if (!wps.some((w) => w.dwell)) {
      // Nowhere to re-view from yet. Keep the track open and try again later.
      this.attempted.set(track.id, this.c.t + 6);
      const resume = this.resumeAfterConfirm;
      this.resumeAfterConfirm = null;
      resume?.();
      return;
    }
    this.confirmTrack = track;
    this.c.log(
      'action',
      `Contact ${track.classId} (${fmt(track.best.confidence, 2)}) at ${this.c.where(track.estimate)}. Re-viewing from ${wps.filter((w) => w.dwell).length} more bearing(s).`,
    );
    this.c.setJob(
      this.pilot.followJob(
        'confirm',
        wps,
        () => {
          const rounds = (this.confirmRounds.get(track.id) ?? 0) + 1;
          const empty = flown.filter((a) => !track.views.some((w) => angleDiff(w.angle, a) < this.c.tracker.minSeparation));
          if (empty.length) this.triedBearings.set(track.id, [...(this.triedBearings.get(track.id) ?? []), ...empty]);
          this.confirmRounds.set(track.id, rounds);
          if (track.status === 'open' && rounds < CONFIRM_ROUNDS) {
            this.attempted.set(track.id, this.c.t + 4);
            this.c.log(
              'info',
              `${track.classId} contact still unconfirmed (${track.views.length} view(s), ${fmt(this.c.tracker.meanConfidence(track), 2)}). Will look again.`,
            );
          } else if (track.status === 'open') {
            const mean = fmt(this.c.tracker.meanConfidence(track), 2);
            const smoky = this.inSmoke(track);
            if (smoky) this.c.log('warn', `${track.classId} contact at ${this.c.where(track.estimate)} is in smoke: its low confidence can be the smoke, so it is not rejected.`);
            if (this.c.tracker.close(track, smoky) === 'unresolved') {
              this.c.log(
                'warn',
                `Unresolved ${track.classId} contact at ${this.c.where(track.estimate)}: ${track.views.length} view(s), mean confidence ${mean}, under the ${fmt(thresholdFor(this.c.mission, track.classId), 2)} to confirm but above what a lookalike scores. It blocks a clear there; flag it for a person to check.`,
              );
            } else {
              this.c.log('warn', `Rejected ${track.classId} contact: ${track.views.length} view(s), mean confidence ${mean} after re-views.`);
            }
          }
          this.endConfirm();
        },
        this.c.inside ? 0.2 : 1.5,
      ),
    );
  }

  /** The building being searched while one of its contacts is having a re-view, else null. */
  confirmingBuilding(): MissionBuilding | null {
    const v = this.c.visit;
    return this.confirmTrack && v && this.confirmTrack.buildingId === v.b.id && this.c.phase !== 'exit' ? v.b : null;
  }

  /**
   * A battery-swap return cuts a re-view short: forget it, so the contact stays open and the next
   * flight into that building re-views it again rather than waiting on a confirmation that never ends.
   */
  dropConfirm(): void {
    if (this.confirmTrack) this.attempted.delete(this.confirmTrack.id);
    this.confirmTrack = null;
    this.resumeAfterConfirm = null;
  }

  /**
   * With hazard awareness: the contact sits in smoke the drone can see (gen/hazards.ts smoke cells),
   * so smoke takes contrast off every view of it and a lookalike-level score proves nothing.
   */
  inSmoke(track: ConfirmTrack): boolean {
    if (this.c.options.hazardAware === false) return false;
    const id = track.buildingId ?? buildingAt(this.c.world, track.estimate, CONTACT_FOOTPRINT_MARGIN_M);
    const b = this.c.world.buildings.find((x) => x.id === id);
    const spec = b?.shell.spec;
    if (!b || !spec?.hazards?.smoke) return false;
    const [ox, oy, oz] = b.shell.origin;
    const f = Math.max(0, Math.min(spec.floors.length - 1, Math.floor((track.estimate[2] - oz) / spec.floorHeight)));
    // The estimate carries range error: any smoke cell within a meter counts.
    for (const [dx, dy] of [[0, 0], [1, 0], [-1, 0], [0, 1], [0, -1]] as const) {
      if (inSmoke(spec, f, track.estimate[0] - ox + dx, track.estimate[1] - oy + dy)) return true;
    }
    return false;
  }

  endConfirm(): void {
    this.confirmTrack = null;
    const resume = this.resumeAfterConfirm;
    this.resumeAfterConfirm = null;
    if (this.findMission() && requirementsMet(this.c.mission, this.c.confirmed)) {
      this.go.startReturn('all required targets confirmed');
      return;
    }
    if (!resume) return;
    // A confirmation can end mid-dwell, at a low re-view pose: back up to cruise before the search resumes.
    const climb = this.c.inside ? [] : this.pilot.climbToCruise();
    if (climb.length) this.c.setJob(this.pilot.followJob('confirm', climb, resume, 0.6));
    else resume();
  }
}

import type { ClearCategory } from '../../perception/seenEmpty';
import type { TargetClass } from '../../targets/types';
import { sampleHeight } from '../../world/outdoor/terrain';
import { bounds, pointInRing } from '../polygon';
import { MISSION_TYPE_LABELS, type Mission, type Point } from '../types';
import type { AutonomyReport, LiveDetectionSource, MissionWorld, RunRecord, TruthTarget, V3 } from './contracts';

/**
 * Report metrics for one autonomous run.
 *
 * Coverage uses a self-contained downward camera footprint model: a cone of
 * half-angle FOOTPRINT_HALF_ANGLE_RAD under the drone, capped at
 * FOOTPRINT_MAX_RADIUS_M because a detector cannot resolve a person past
 * about that ground range. perception.ts carries its own camera constants;
 * the two are reconciled at integration.
 */

/** Half-angle of the ground footprint cone. A 90 degree downward FOV. */
export const FOOTPRINT_HALF_ANGLE_RAD = Math.PI / 4;
/** Largest footprint radius that still resolves a person, meters. */
export const FOOTPRINT_MAX_RADIUS_M = 15;
/** Outdoor coverage grid cell, meters. */
export const COVERAGE_CELL_M = 2;
/** Track samples closer than this to the last used one are skipped, meters. */
const TRACK_STEP_M = 1;

/** Ground footprint radius for a downward camera at `aglM` meters above ground. */
export function cameraFootprintRadius(aglM: number): number {
  if (!(aglM > 0)) return 0;
  return Math.min(FOOTPRINT_MAX_RADIUS_M, aglM * Math.tan(FOOTPRINT_HALF_ANGLE_RAD));
}

/** Mission target classes: person kind -> 'person', object kind -> any object class. */
function wantsPerson(mission: Mission): boolean {
  return mission.targets.some((t) => t.kind === 'person');
}

function wantsObject(mission: Mission): boolean {
  return mission.targets.some((t) => t.kind === 'object');
}

function isMissionClass(mission: Mission, cls: TargetClass, category?: TruthTarget['category']): boolean {
  if (cls === 'person') return wantsPerson(mission);
  if (category === 'decoy') return false;
  return wantsObject(mission) && !isDecoyClass(cls);
}

function isDecoyClass(cls: TargetClass): boolean {
  return cls === 'mannequin' || cls === 'coat' || cls === 'decoy_pack' || cls === 'decoy_case';
}

/** Categories a declared clear vouches for. Clearance and person sweeps clear people. */
export function sweptCategories(mission: Mission): ClearCategory[] {
  const out: ClearCategory[] = [];
  if (mission.targets.some((t) => t.kind === 'person' || t.kind === 'clearance') || mission.missionType === 'safety_sweep') {
    out.push('person');
  }
  if (wantsObject(mission)) out.push('object');
  return out;
}

interface CoverageGrid {
  cells: Point[];
}

function inBuilding(world: MissionWorld | undefined, pos: V3): boolean {
  if (!world) return false;
  const p = { x: pos[0], y: pos[1] };
  return world.buildings.some((b) => pos[2] < b.shell.origin[2] + b.heightM && pointInRing(p, b.footprint));
}

/** Operable cells: inside the fence, outside no-fly zones and building footprints. */
export function operableCells(mission: Mission, world?: MissionWorld, cellM = COVERAGE_CELL_M): CoverageGrid {
  const fence = mission.geofence.points;
  const cells: Point[] = [];
  if (fence.length < 3) return { cells };
  const b = bounds(fence);
  for (let y = b.minY + cellM / 2; y < b.maxY; y += cellM) {
    for (let x = b.minX + cellM / 2; x < b.maxX; x += cellM) {
      const p = { x, y };
      if (!pointInRing(p, fence)) continue;
      if (mission.noFlyZones.some((z) => pointInRing(p, z.polygon.points))) continue;
      if (world) {
        if (world.buildings.some((bl) => pointInRing(p, bl.footprint))) continue;
        // A site-plan zone blocks the cell when its band covers the lowest legal altitude there.
        const z = sampleHeight(world.outdoor.terrain, x, y) + mission.altitude.minM;
        const blocked = world.outdoor.noFly.some(
          (zone) => z >= zone.minZ && z <= zone.maxZ && pointInRing(p, zone.polygon.map(([px, py]) => ({ x: px, y: py }))),
        );
        if (blocked) continue;
      }
      cells.push(p);
    }
  }
  return { cells };
}

/**
 * Fraction of operable cells inside the camera footprint at some track point.
 * Without a world, AGL is measured from the first track point (the pad).
 */
export function outdoorCoverage(run: RunRecord, world?: MissionWorld): number {
  const { cells } = operableCells(run.mission, world);
  if (cells.length === 0 || run.track.length === 0) return 0;
  const seen = new Uint8Array(cells.length);
  const padZ = run.track[0]!.pos[2];
  let last: V3 | null = null;
  for (const sample of run.track) {
    const pos = sample.pos;
    if (last && Math.hypot(pos[0] - last[0], pos[1] - last[1], pos[2] - last[2]) < TRACK_STEP_M) continue;
    last = pos;
    if (inBuilding(world, pos)) continue;
    const ground = world ? sampleHeight(world.outdoor.terrain, pos[0], pos[1]) : padZ;
    const r = cameraFootprintRadius(pos[2] - ground);
    if (r <= 0) continue;
    const r2 = r * r;
    for (let i = 0; i < cells.length; i++) {
      if (seen[i]) continue;
      const c = cells[i]!;
      const dx = c.x - pos[0];
      const dy = c.y - pos[1];
      if (dx * dx + dy * dy <= r2) seen[i] = 1;
    }
  }
  let count = 0;
  for (const s of seen) count += s;
  return count / cells.length;
}

function enteredBuildings(run: RunRecord): Set<string> {
  const ids = new Set<string>();
  for (const e of run.entries) if (e.decision === 'auto' || e.decision === 'approved') ids.add(e.buildingId);
  return ids;
}

/**
 * Report from a run record and its detection source. Pass the world for
 * terrain-relative coverage, building footprints, and the safety-sweep
 * building list; without it the report falls back to what the run recorded.
 */
export function buildReport(run: RunRecord, source: LiveDetectionSource, world?: MissionWorld): AutonomyReport {
  const mission = run.mission;
  const truth = source.truthTargets();
  const notes: string[] = [];

  // Time to find.
  const confirmed = [...run.confirmed].sort((a, b) => a.tSec - b.tSec);
  const firstFind = confirmed.find((c) => c.truth.kind === 'tp' && isMissionClass(mission, c.truth.trueClass));
  const timeToFindSec = firstFind ? firstFind.tSec : null;

  // False alarms and misses.
  const falseAlarms = confirmed.filter((c) => c.truth.kind !== 'tp' || c.truth.trueClass !== c.classId).length;
  const found = new Set(confirmed.filter((c) => c.truth.kind === 'tp').map((c) => c.truth.targetId));
  const findMission = wantsPerson(mission) || wantsObject(mission);
  const missed = findMission
    ? truth.filter((t) => isMissionClass(mission, t.classId, t.category) && !found.has(t.id)).length
    : 0;

  // False clears.
  const swept = sweptCategories(mission);
  const scopes = new Set(run.declaredClear.map((d) => d.scope));
  let falseClears = 0;
  for (const scope of scopes) {
    const where = scope === 'outdoor' ? null : scope;
    if (truth.some((t) => t.buildingId === where && swept.includes(t.category as ClearCategory))) falseClears++;
  }

  // Indoor coverage.
  const buildingIds = enteredBuildings(run);
  if (mission.missionType === 'safety_sweep') {
    if (world) for (const b of world.buildings) buildingIds.add(b.id);
    else {
      for (const t of truth) if (t.buildingId) buildingIds.add(t.buildingId);
      for (const s of scopes) if (s !== 'outdoor') buildingIds.add(s);
    }
  }
  const perBuildingCoverage: Record<string, number> = {};
  let resolved = 0;
  let total = 0;
  const incomplete: string[] = [];
  for (const id of [...buildingIds].sort()) {
    const r = source.truthClearStatus(id, 'person');
    perBuildingCoverage[id] = r.total > 0 ? (r.seenEmpty + r.occupied) / r.total : 0;
    resolved += r.seenEmpty + r.occupied;
    total += r.total;
    if (r.status === 'incomplete') incomplete.push(id);
  }
  const indoorCoverage = total > 0 ? resolved / total : 0;

  const cover = outdoorCoverage(run, world);

  notes.push('Battery uses the momentum-theory model (hover, forward flight, climb).');
  notes.push(`Detector is simulated (${source.id}); people and objects are synthetic.`);
  notes.push(
    `Outdoor coverage counts ${COVERAGE_CELL_M} m cells under a ${Math.round((FOOTPRINT_HALF_ANGLE_RAD * 360) / Math.PI)} degree downward footprint, capped at ${FOOTPRINT_MAX_RADIUS_M} m radius.`,
  );
  if (!world) notes.push('No world given: AGL is relative to the takeoff point and building footprints are not excluded.');
  if (incomplete.length > 0) notes.push(`Incomplete indoor scopes: ${incomplete.join(', ')}.`);
  if (run.declaredClear.length === 0 && mission.missionType === 'safety_sweep') notes.push('No scope was declared clear.');
  if (falseClears > 0) notes.push(`${falseClears} scope(s) were declared clear while a target was present.`);
  if (!run.landedHome) notes.push('The drone did not land within 1.5 m of home.');
  if (run.indoorExplorer) notes.push(`Indoor exploration used the ${run.indoorExplorer} explorer.`);

  return {
    missionName: mission.name,
    missionType: mission.missionType,
    outcome: run.outcome,
    durationSec: run.tSec,
    timeToFindSec,
    outdoorCoverage: cover,
    indoorCoverage,
    perBuildingCoverage,
    falseClears,
    falseAlarms,
    missed,
    collisions: run.collisions.length,
    batteryUsedPct: Math.max(0, run.batteryStartPct - run.batteryEndPct),
    landedHome: run.landedHome,
    entries: run.entries,
    safetyEvents: run.safetyEvents.length,
    notes,
  };
}

function pct(x: number): string {
  return `${Math.round(x * 100)}%`;
}

function secs(x: number): string {
  if (x < 60) return `${x.toFixed(1)} s`;
  return `${Math.floor(x / 60)} min ${Math.round(x % 60)} s`;
}

/** Label and value rows for UI tables. */
export function reportRows(report: AutonomyReport): { label: string; value: string }[] {
  const rows = [
    { label: 'Mission', value: report.missionName },
    { label: 'Type', value: MISSION_TYPE_LABELS[report.missionType] },
    { label: 'Outcome', value: report.outcome.replace('_', ' ') },
    { label: 'Duration', value: secs(report.durationSec) },
    { label: 'Time to find', value: report.timeToFindSec === null ? 'not found' : secs(report.timeToFindSec) },
    { label: 'Outdoor coverage', value: pct(report.outdoorCoverage) },
    { label: 'Indoor coverage', value: pct(report.indoorCoverage) },
  ];
  for (const [id, value] of Object.entries(report.perBuildingCoverage)) rows.push({ label: `Coverage ${id}`, value: pct(value) });
  rows.push(
    { label: 'False clears', value: String(report.falseClears) },
    { label: 'False alarms', value: String(report.falseAlarms) },
    { label: 'Missed targets', value: String(report.missed) },
    { label: 'Collisions', value: String(report.collisions) },
    { label: 'Battery used', value: `${report.batteryUsedPct.toFixed(1)}%` },
    { label: 'Landed home', value: report.landedHome ? 'yes' : 'no' },
    { label: 'Entries', value: String(report.entries.length) },
    { label: 'Safety events', value: String(report.safetyEvents) },
  );
  return rows;
}

/** Markdown summary: a metrics table, entries, and notes. */
export function reportMarkdown(report: AutonomyReport): string {
  const esc = (s: string): string => s.replace(/\|/g, '\\|');
  const lines = [`# ${esc(report.missionName)}`, '', '| Metric | Value |', '| --- | --- |'];
  for (const row of reportRows(report)) lines.push(`| ${esc(row.label)} | ${esc(row.value)} |`);
  if (report.entries.length > 0) {
    lines.push('', '## Entries', '');
    for (const e of report.entries) {
      lines.push(`- ${e.tSec.toFixed(1)} s: ${e.buildingId} opening ${e.openingId}, safety ${e.safety.toFixed(2)}, ${e.decision}`);
    }
  }
  if (report.notes.length > 0) {
    lines.push('', '## Notes', '');
    for (const n of report.notes) lines.push(`- ${n}`);
  }
  return `${lines.join('\n')}\n`;
}

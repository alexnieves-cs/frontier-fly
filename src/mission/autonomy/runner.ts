// STARTER
// Minimal straight-line runner so the demo page and scenario tests have
// something to drive: home -> each building -> home, no collision physics,
// no coverage planner, no safety monitor. The planner workstream's runner
// replaces this file at integration and keeps these two exports.
import { sampleHeight } from '../../world/outdoor/terrain';
import type { Opening } from '../../world/outdoor/types';
import type { Mission } from '../types';
import { createBattery } from './battery';
import type {
  ConfirmedContact,
  Contact,
  DronePose,
  EntryCandidate,
  EntryRecord,
  LiveDetectionSource,
  LogEntry,
  MissionBuilding,
  MissionRunner,
  MissionWorld,
  Phase,
  RunnerOptions,
  RunnerSnapshot,
  RunOutcome,
  RunRecord,
  V3,
} from './contracts';

interface Leg {
  phase: Phase;
  target: V3;
  speed: number;
  /** Hover at the target this long before the next leg. */
  holdSec?: number;
  inside?: string | null;
  /** Runs once when the leg finishes. */
  onArrive?: () => void;
}

const CRUISE_AGL = 6;
const OUTDOOR_SPEED = 5;
const INDOOR_SPEED = 1.2;

export async function createMissionRunner(
  mission: Mission,
  world: MissionWorld,
  source: LiveDetectionSource,
  options: RunnerOptions = {},
): Promise<MissionRunner> {
  return new StarterRunner(mission, world, source, options);
}

export async function runMission(
  mission: Mission,
  world: MissionWorld,
  source: LiveDetectionSource,
  options: RunnerOptions = {},
): Promise<RunRecord> {
  const runner = await createMissionRunner(mission, world, source, options);
  while (!runner.done) runner.step();
  return runner.record();
}

class StarterRunner implements MissionRunner {
  private readonly dt: number;
  private readonly maxSimSec: number;
  private readonly battery = createBattery();
  private readonly batteryStartPct: number;
  private t = 0;
  private pose: DronePose;
  private phase: Phase = 'preflight';
  private phaseStart = 0;
  private inside: string | null = null;
  private legs: Leg[] = [];
  private hold = 0;
  private finished = false;
  private outcome: RunOutcome = 'incomplete';
  private pending: EntryCandidate | null = null;
  private pendingBuilding: MissionBuilding | null = null;
  private lastObserve = -1;
  private lastTrack = -1;
  private readonly track: RunRecord['track'] = [];
  private readonly phases: RunRecord['phases'] = [];
  private readonly contacts: Contact[] = [];
  private readonly confirmed: ConfirmedContact[] = [];
  private readonly declaredClear: RunRecord['declaredClear'] = [];
  private readonly entries: EntryRecord[] = [];
  private readonly log: LogEntry[] = [];
  private readonly wanted: Set<string>;

  constructor(
    private readonly mission: Mission,
    private readonly world: MissionWorld,
    private readonly source: LiveDetectionSource,
    private readonly options: RunnerOptions,
  ) {
    this.dt = options.dt ?? 0.05;
    this.maxSimSec = options.maxSimSec ?? mission.budgets.timeSec;
    this.batteryStartPct = this.battery.remainingPct();
    this.pose = { pos: [...world.home], yaw: world.homeYaw, vel: [0, 0, 0] };
    this.wanted = wantedClasses(mission);
    this.note('info', `Preflight on the pad. ${world.buildings.length} building(s) in the site.`);
    this.planOutbound();
  }

  get done(): boolean {
    return this.finished;
  }

  step(dtSec?: number): void {
    if (this.finished) return;
    const dt = dtSec ?? this.dt;
    this.t += dt;
    if (this.t >= this.maxSimSec) {
      this.note('warn', 'Time budget reached. Stopping the run.');
      this.finish('incomplete');
      return;
    }
    if (this.phase === 'await_operator') {
      this.hover(dt);
      return;
    }
    const leg = this.legs[0];
    if (!leg) {
      this.finish(this.outcome);
      return;
    }
    if (leg.phase !== this.phase) this.enter(leg.phase);
    this.inside = leg.inside ?? null;
    const dx = leg.target[0] - this.pose.pos[0];
    const dy = leg.target[1] - this.pose.pos[1];
    const dz = leg.target[2] - this.pose.pos[2];
    const dist = Math.hypot(dx, dy, dz);
    const stepLen = leg.speed * dt;
    const prev: V3 = [...this.pose.vel];
    if (dist <= stepLen) {
      this.pose.pos = [...leg.target];
      this.pose.vel = [0, 0, 0];
      this.hold += dt;
      if (this.hold >= (leg.holdSec ?? 0)) {
        this.hold = 0;
        this.legs.shift();
        leg.onArrive?.();
      }
    } else {
      const k = stepLen / dist;
      this.pose.pos = [this.pose.pos[0] + dx * k, this.pose.pos[1] + dy * k, this.pose.pos[2] + dz * k];
      this.pose.vel = [(dx * k) / dt, (dy * k) / dt, (dz * k) / dt];
      if (Math.hypot(dx, dy) > 0.05) this.pose.yaw = Math.atan2(dy, dx);
    }
    const accel: V3 = [(this.pose.vel[0] - prev[0]) / dt, (this.pose.vel[1] - prev[1]) / dt, (this.pose.vel[2] - prev[2]) / dt];
    this.battery.draw(this.battery.powerFor(this.pose.vel, accel), dt);
    this.sense();
  }

  approve(): void {
    this.answer('approved');
  }

  deny(): void {
    this.answer('denied');
  }

  abort(): void {
    if (this.finished) return;
    this.note('warn', 'Operator aborted the run.');
    this.finish('aborted');
  }

  snapshot(): RunnerSnapshot {
    return {
      tSec: this.t,
      phase: this.phase,
      pose: { pos: [...this.pose.pos], yaw: this.pose.yaw, vel: [...this.pose.vel] },
      batteryPct: this.battery.remainingPct(),
      inside: this.inside,
      plan: this.legs.map((leg) => leg.target),
      track: this.track.map((sample) => sample.pos),
      contacts: this.contacts.slice(-40),
      confirmed: [...this.confirmed],
      pendingEntry: this.pending,
      log: [...this.log],
      done: this.finished,
    };
  }

  record(): RunRecord {
    return {
      mission: this.mission,
      seed: this.world.seed,
      outcome: this.outcome,
      tSec: this.t,
      track: [...this.track],
      phases: [...this.phases],
      confirmed: [...this.confirmed],
      declaredClear: [...this.declaredClear],
      collisions: [],
      entries: [...this.entries],
      safetyEvents: [],
      batteryStartPct: this.batteryStartPct,
      batteryEndPct: this.battery.remainingPct(),
      landedHome: this.finished && Math.hypot(this.pose.pos[0] - this.world.home[0], this.pose.pos[1] - this.world.home[1]) < 1.5 && this.outcome !== 'aborted',
      indoorExplorer: this.entries.some((entry) => entry.decision === 'auto' || entry.decision === 'approved') ? 'frontier' : null,
      log: [...this.log],
    };
  }

  // ------------------------------------------------------------ planning

  private cruise(x: number, y: number): V3 {
    return [x, y, sampleHeight(this.world.outdoor.terrain, x, y) + CRUISE_AGL];
  }

  private planOutbound(): void {
    const [hx, hy] = this.world.home;
    this.legs.push({ phase: 'preflight', target: [...this.world.home], speed: 1, holdSec: 0.5 });
    this.legs.push({ phase: 'coverage', target: this.cruise(hx, hy), speed: 1.5 });
    const first = this.world.buildings[0];
    if (first) this.planBuilding(0);
    else this.planHome();
  }

  private planBuilding(index: number): void {
    const building = this.world.buildings[index]!;
    const opening = bestOpening(building);
    if (!opening) {
      this.entries.push({ tSec: this.t, buildingId: building.id, openingId: -1, safety: 0, decision: 'none_viable' });
      this.next(index);
      return;
    }
    const [nx, ny] = opening.normal;
    const [cx, cy, cz] = opening.center;
    const mouth: V3 = [cx + nx * 3, cy + ny * 3, cz];
    this.legs.push({ phase: 'coverage', target: this.cruise(mouth[0], mouth[1]), speed: OUTDOOR_SPEED });
    this.legs.push({ phase: 'approach', target: mouth, speed: 2 });
    this.legs.push({
      phase: 'inspect',
      target: mouth,
      speed: 1,
      holdSec: 2,
      onArrive: () => this.selectEntry(index, building, opening),
    });
  }

  private selectEntry(index: number, building: MissionBuilding, opening: Opening): void {
    this.enter('entry_select');
    const candidate = toCandidate(building, opening, this.mission.entryPolicy);
    this.note('info', `${building.label}: ${opening.kind.replaceAll('_', ' ')} clears by ${candidate.clearanceM.toFixed(2)} m.`);
    if (candidate.decision === 'never') {
      this.entries.push({ tSec: this.t, buildingId: building.id, openingId: opening.id, safety: candidate.safety, decision: 'never' });
      this.note('warn', `Entry policy forbids entering ${building.label}.`);
      this.next(index);
      return;
    }
    if (candidate.decision === 'ask') {
      const answer = this.options.operator?.(candidate) ?? 'pending';
      if (answer === 'pending') {
        this.pending = candidate;
        this.pendingBuilding = building;
        this.enter('await_operator');
        this.note('action', `Asking the operator to enter ${building.label}.`);
        return;
      }
      this.decide(answer === 'approve' ? 'approved' : 'denied', building, opening, candidate);
      return;
    }
    this.decide('auto', building, opening, candidate);
  }

  private answer(decision: 'approved' | 'denied'): void {
    if (!this.pending || !this.pendingBuilding) return;
    const building = this.pendingBuilding;
    const candidate = this.pending;
    const opening = building.shell.openings.find((item) => item.id === candidate.openingId)!;
    this.pending = null;
    this.pendingBuilding = null;
    this.enter('entry_select');
    this.decide(decision, building, opening, candidate);
  }

  private decide(decision: EntryRecord['decision'], building: MissionBuilding, opening: Opening, candidate: EntryCandidate): void {
    const index = this.world.buildings.indexOf(building);
    this.entries.push({ tSec: this.t, buildingId: building.id, openingId: opening.id, safety: candidate.safety, decision });
    if (decision === 'denied') {
      this.note('warn', `Operator denied entry to ${building.label}.`);
      this.enter('entry_select');
      this.next(index);
      return;
    }
    this.note('action', `Entering ${building.label} (${decision}).`);
    const [nx, ny] = opening.normal;
    const [cx, cy, cz] = opening.center;
    const mouth: V3 = [cx + nx * 1.2, cy + ny * 1.2, cz];
    const inner: V3 = [cx - nx * 1.4, cy - ny * 1.4, cz];
    const floorZ = building.shell.origin[2] + 1.2;
    this.legs.push(
      { phase: 'fly_through', target: mouth, speed: 1 },
      { phase: 'fly_through', target: inner, speed: 0.8, inside: building.id },
      ...roomCenters(building).map((point): Leg => ({ phase: 'indoor_explore', target: [point[0], point[1], floorZ], speed: INDOOR_SPEED, holdSec: 0.6, inside: building.id })),
      { phase: 'confirm', target: inner, speed: INDOOR_SPEED, inside: building.id, onArrive: () => this.afterIndoor(index, building) },
      { phase: 'exit', target: mouth, speed: 0.8, inside: building.id },
    );
  }

  private afterIndoor(index: number, building: MissionBuilding): void {
    if (this.mission.missionType === 'safety_sweep') {
      const report = this.source.clearStatus(building.id, 'person');
      if (report.status === 'clear' && !this.confirmed.some((c) => c.buildingId === building.id && c.classId === 'person')) {
        this.declaredClear.push({ scope: building.id, tSec: this.t });
        this.note('action', `${building.label} confirmed clear.`);
      } else {
        this.note('warn', `${building.label} not confirmed clear (${report.status}).`);
      }
    }
    // Remove the legs left for this building except the exit, then continue.
    const exit = this.legs.find((leg) => leg.phase === 'exit');
    this.legs = exit ? [exit] : [];
    if (this.isFound()) this.planHome();
    else this.next(index);
  }

  private next(index: number): void {
    if (index + 1 < this.world.buildings.length && !this.isFound()) this.planBuilding(index + 1);
    else this.planHome();
  }

  private planHome(): void {
    const [hx, hy] = this.world.home;
    const here = this.legs.at(-1)?.target ?? this.pose.pos;
    this.legs.push({ phase: 'return_home', target: this.cruise(here[0], here[1]), speed: 1.5 });
    this.legs.push({ phase: 'return_home', target: this.cruise(hx, hy), speed: OUTDOOR_SPEED });
    this.legs.push({
      phase: 'land',
      target: [...this.world.home],
      speed: 1,
      onArrive: () => this.finish(this.finalOutcome()),
    });
  }

  private finalOutcome(): RunOutcome {
    if (this.mission.missionType === 'safety_sweep') {
      return this.world.buildings.every((b) => this.declaredClear.some((d) => d.scope === b.id)) ? 'clear' : 'incomplete';
    }
    return this.isFound() ? 'found' : 'not_found';
  }

  // ------------------------------------------------------------ sensing

  private sense(): void {
    if (this.t - this.lastTrack >= 0.5) {
      this.lastTrack = this.t;
      this.track.push({ t: this.t, pos: [...this.pose.pos], phase: this.phase });
    }
    if (this.t - this.lastObserve < 0.2) return;
    this.lastObserve = this.t;
    const observation = this.source.observe(this.pose, this.t);
    for (const contact of observation.contacts) {
      this.contacts.push(contact);
      if (this.contacts.length > 400) this.contacts.shift();
      this.tryConfirm(contact);
    }
  }

  private tryConfirm(contact: Contact): void {
    if (!this.wanted.has(contact.classId) || contact.confidence < this.mission.requiredConfidence) return;
    if (this.confirmed.some((c) => c.classId === contact.classId && dist3(c.estimate, contact.estimate) < 1.5)) return;
    const views = this.contacts.filter(
      (other) => other.classId === contact.classId && other.confidence >= this.mission.requiredConfidence && dist3(other.estimate, contact.estimate) < 1.5,
    );
    if (views.length < 2) return;
    this.enter('confirm');
    this.confirmed.push({
      contactId: contact.id,
      classId: contact.classId,
      confidence: views.reduce((sum, view) => sum + view.confidence, 0) / views.length,
      views: views.length,
      estimate: [...contact.estimate],
      tSec: this.t,
      buildingId: contact.buildingId,
      truth: contact.truth,
    });
    this.note('action', `Confirmed ${contact.classId.replaceAll('_', ' ')} from ${views.length} views.`);
  }

  private isFound(): boolean {
    return this.mission.missionType !== 'safety_sweep' && this.confirmed.length > 0;
  }

  // ------------------------------------------------------------ bookkeeping

  private hover(dt: number): void {
    this.pose.vel = [0, 0, 0];
    this.battery.draw(this.battery.powerFor(this.pose.vel, [0, 0, 0]), dt);
    this.sense();
  }

  private enter(phase: Phase): void {
    if (phase === this.phase) return;
    this.phases.push({ phase: this.phase, tStart: this.phaseStart, tEnd: this.t });
    this.phase = phase;
    this.phaseStart = this.t;
  }

  private finish(outcome: RunOutcome): void {
    this.outcome = outcome;
    this.enter('done');
    this.phases.push({ phase: 'done', tStart: this.t, tEnd: this.t });
    this.finished = true;
    this.pending = null;
    this.note(outcome === 'aborted' || outcome === 'incomplete' ? 'warn' : 'info', `Run finished: ${outcome.replaceAll('_', ' ')}.`);
  }

  private note(tone: LogEntry['tone'], message: string): void {
    this.log.push({ tSec: this.t, tone, message });
  }
}

function wantedClasses(mission: Mission): Set<string> {
  if (mission.missionType === 'find_person' || mission.missionType === 'safety_sweep') return new Set(['person']);
  const out = new Set<string>();
  for (const target of mission.targets) {
    if (target.kind === 'person') out.add('person');
    const label = target.label.toLowerCase();
    for (const id of ['backpack', 'phone', 'toolbox', 'medkit', 'bottle', 'radio', 'helmet', 'blanket']) if (label.includes(id)) out.add(id);
  }
  return out;
}

function bestOpening(building: MissionBuilding): Opening | null {
  let best: Opening | null = null;
  for (const opening of building.shell.openings) {
    if (!opening.passable || opening.kind === 'roof_hatch') continue;
    if (!best || Math.min(opening.clearWidth, opening.clearHeight) > Math.min(best.clearWidth, best.clearHeight)) best = opening;
  }
  return best;
}

function toCandidate(building: MissionBuilding, opening: Opening, policy: Mission['entryPolicy']): EntryCandidate {
  const clearanceM = Math.min(opening.clearWidth, opening.clearHeight) - 0.38;
  const safety = Math.max(0, Math.min(1, clearanceM / 1.2));
  return {
    buildingId: building.id,
    openingId: opening.id,
    kind: opening.kind,
    clearWidth: opening.clearWidth,
    clearHeight: opening.clearHeight,
    clearanceM,
    safety,
    decision: policy === 'never' ? 'never' : policy === 'ask' ? 'ask' : 'auto',
  };
}

function roomCenters(building: MissionBuilding): [number, number][] {
  const { spec, origin } = building.shell;
  const floor = spec.floors[0]!;
  return floor.rooms.map((room) => [origin[0] + (room.x + room.w / 2) * spec.cellSize, origin[1] + (room.z + room.d / 2) * spec.cellSize]);
}

function dist3(a: V3, b: V3): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

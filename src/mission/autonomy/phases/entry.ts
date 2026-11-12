import type { EntryCandidate, MissionBuilding, V3 } from '../contracts';
import type { MeasuredOpening } from '../openingSensor';
import { planApproach } from '../planner/approach';
import { passageAxis, rankCandidates, scoreOpening } from '../planner/entry';
import { FlyThrough } from '../planner/flythrough';
import { buildingBox, legBetween } from '../planner/geometry';
import { indoorGrid, oracleNav } from '../planner/indoor';
import { planInspection } from '../planner/inspect';
import { type VehicleCommand } from '../vehicle';
import {
  type Visit,
  type Waypoint,
  fmt,
  hold,
} from './shared';
import type { MissionCore } from './core';
import type { Pilot } from './pilot';

/** Where the entry phase hands over: back to the search, out and away, or into indoor exploration. */
export interface EntryExits {
  continueMission(): void;
  leaveBuilding(): void;
  startExplore(): void;
}

/**
 * Building approach: inspect the shell, rank the observed openings, ask the operator, and fly through.
 * Owns the operator's answer to the pending request; the visit it builds is shared (MissionCore.visit).
 */
export class EntryPhase {
  private operatorAnswer: 'approve' | 'deny' | null = null;
  /** Openings inspected and ranked per building, for a return after a battery swap. */
  private readonly known = new Map<string, { observed: Map<number, MeasuredOpening>; ranked: EntryCandidate[]; visit: Visit }>();

  constructor(
    private readonly c: MissionCore,
    private readonly pilot: Pilot,
    private readonly go: EntryExits,
  ) {}

  /** The operator's answer to the pending entry request (MissionRunner.approve / deny). */
  answer(a: 'approve' | 'deny'): void {
    this.operatorAnswer = a;
  }

  // ----------------------------------------------------------- building

  startBuilding(b: MissionBuilding): void {
    const box = buildingBox(b);
    const grid = indoorGrid(b);
    const v: Visit = {
      b,
      box,
      grid,
      standoff: null,
      observed: new Map(),
      ranked: [],
      candIdx: 0,
      denied: 0,
      opening: null,
      axis: null,
      aborts: 0,
      exitTries: 0,
      vis: null,
      explorer: null,
      sensing: null,
      nav: oracleNav(grid),
      searchComplete: false,
      sensorJob: null,
      exploreStart: 0,
      pass: 0,
      policyCollisions: 0,
      entered: false,
      trail: [],
    };
    this.c.visit = v;
    const plan = planApproach(b, this.c.vehicle.pos, this.c.physics, this.c.air, this.c.agl);
    if (!plan.ok || !plan.standoff) {
      this.c.log('warn', `${b.label}: ${plan.reason}. Skipping.`);
      this.c.visit = null;
      this.go.continueMission();
      return;
    }
    v.standoff = plan.standoff;
    this.c.log('action', `Approaching ${b.label}. Standoff ${this.c.where(plan.standoff.pos)}, open descent column checked.`);
    const wps: Waypoint[] = [
      ...plan.cruise.map((p) => ({ pos: p, tol: 1.5 })),
      ...plan.descent.map((p, i) => ({
        pos: p,
        tol: 0.3,
        yaw: i === plan.descent.length - 1 ? plan.standoff!.yaw : null,
      })),
    ];
    // Back after a battery swap: the openings were inspected and ranked on the first visit.
    const known = this.known.get(b.id);
    if (known) {
      for (const [k, o] of known.observed) v.observed.set(k, o);
      v.ranked = known.ranked;
      // And the maps it built inside, on the true pose: the search picks up where it left off. On the
      // pose estimate the new entry starts a new estimate, which the old map is not aligned to.
      if (this.c.options.localization !== 'estimated') v.sensing = known.visit.sensing;
      this.c.setJob(
        this.pilot.followJob(
          'approach',
          wps,
          () => {
            this.c.setPhase('entry_select');
            this.c.log('action', `Back at ${b.label}: entering through the openings ranked on the first visit.`);
            this.tryCandidate();
          },
          0.3,
        ),
      );
      return;
    }
    this.c.setJob(this.pilot.followJob('approach', wps, () => this.startInspect(), 0.3));
  }

  startInspect(): void {
    const v = this.c.visit!;
    const steps = planInspection(v.b, this.c.vehicle.pos, this.c.physics, this.c.air, this.c.options.search !== undefined);
    this.c.log('action', `Inspecting the ${v.b.label} envelope from ${steps.filter((s) => s.dwell > 0).length} viewpoints.`);
    const wps: Waypoint[] = steps.map((s) => ({
      pos: s.pos,
      yaw: s.yaw,
      dwell: s.dwell || undefined,
    }));
    this.c.setJob(this.pilot.followJob('inspect', wps, () => this.selectEntry(), 0.25));
  }

  selectEntry(): void {
    const v = this.c.visit!;
    this.c.setPhase('entry_select');
    const cands = [...v.observed.values()]
      .sort((a, b) => a.id - b.id)
      .map((o) => scoreOpening(v.b, o, this.c.physics, this.c.mission.entryPolicy));
    for (const c of cands) {
      this.c.log(
        'info',
        `Opening ${c.openingId} ${c.kind.replace('_', ' ')}: clearance ${fmt(c.clearanceM, 2)} m, safety ${fmt(c.safety, 2)}, ${c.decision}.`,
      );
    }
    v.ranked = rankCandidates(cands);
    v.candIdx = 0;
    this.known.set(v.b.id, { observed: new Map(v.observed), ranked: v.ranked, visit: v });
    if (!cands.length) {
      this.c.log('warn', `No openings observed on ${v.b.label}.`);
    }
    if (this.c.mission.entryPolicy === 'never') {
      const best = [...cands].sort((a, b) => b.safety - a.safety)[0];
      this.c.entries.push({
        tSec: this.c.t,
        buildingId: v.b.id,
        openingId: best?.openingId ?? -1,
        safety: best?.safety ?? 0,
        decision: 'never',
      });
      this.c.log('info', `Entry policy is never. Not entering ${v.b.label}.`);
      this.go.leaveBuilding();
      return;
    }
    this.tryCandidate();
  }

  tryCandidate(): void {
    const v = this.c.visit!;
    const cand = v.ranked[v.candIdx];
    if (!cand) {
      if (v.denied === 0) {
        const best = [...v.observed.values()][0];
        this.c.entries.push({
          tSec: this.c.t,
          buildingId: v.b.id,
          openingId: best?.id ?? -1,
          safety: 0,
          decision: 'none_viable',
        });
      }
      this.c.log('warn', `No viable entry into ${v.b.label}. Skipping the interior.`);
      this.go.leaveBuilding();
      return;
    }
    if (cand.decision === 'auto') {
      this.c.entries.push({
        tSec: this.c.t,
        buildingId: v.b.id,
        openingId: cand.openingId,
        safety: cand.safety,
        decision: 'auto',
      });
      this.c.log(
        'action',
        `Entering ${v.b.label} through opening ${cand.openingId} (${cand.kind.replace('_', ' ')}), safety ${fmt(cand.safety, 2)}.`,
      );
      this.goEnter(cand);
      return;
    }
    this.c.pendingEntry = cand;
    this.operatorAnswer = null;
    this.c.log(
      'action',
      `Operator approval needed: opening ${cand.openingId} (${cand.kind.replace('_', ' ')}) on ${v.b.label}, safety ${fmt(cand.safety, 2)}.`,
    );
    const at: V3 = [this.c.vehicle.pos[0], this.c.vehicle.pos[1], this.c.vehicle.pos[2]];
    this.c.setJob({
      phase: 'await_operator',
      plan: () => [],
      tick: (): VehicleCommand => {
        let answer = this.operatorAnswer;
        if (this.c.linkDown) {
          // No link, no consent: an unanswerable request is a denial (SAFETY_CASE.md H2).
          answer = 'deny';
          this.c.log('warn', `Operator link down. Treating the request for opening ${cand.openingId} as denied.`);
        } else if (!answer && this.c.options.operator) {
          const r = this.c.options.operator(cand);
          if (r !== 'pending') answer = r;
        }
        if (answer) {
          this.operatorAnswer = null;
          this.c.pendingEntry = null;
          if (answer === 'approve') {
            this.c.entries.push({
              tSec: this.c.t,
              buildingId: v.b.id,
              openingId: cand.openingId,
              safety: cand.safety,
              decision: 'approved',
            });
            this.c.log('action', `Operator approved opening ${cand.openingId}.`);
            this.goEnter(cand);
          } else {
            this.c.entries.push({
              tSec: this.c.t,
              buildingId: v.b.id,
              openingId: cand.openingId,
              safety: cand.safety,
              decision: 'denied',
            });
            this.c.log('info', `Operator denied opening ${cand.openingId}.`);
            v.denied += 1;
            v.candIdx += 1;
            this.tryCandidate();
          }
        }
        return { vel: hold(this.c.vehicle.pos, at) };
      },
    });
  }

  goEnter(cand: EntryCandidate): void {
    const v = this.c.visit!;
    // The opening as measured from outside: the plan, its margins and the crossing line all come from it.
    const opening = v.observed.get(cand.openingId)!;
    v.opening = opening;
    v.axis = passageAxis(opening);
    v.aborts = 0;
    const ft = new FlyThrough(v.axis, 'in');
    const hop = v.box.roofZ + 2.5;
    let leg = legBetween(this.c.physics, this.c.vehicle.pos, ft.start, hop);
    const refuge = v.standoff?.pos;
    if (!leg && refuge) {
      // Retrace the inspection path until the alignment point is reachable.
      const back = this.pilot.retrace(v.trail, (p) => legBetween(this.c.physics, p, ft.start, hop) !== null);
      const end = back.length ? back[back.length - 1]!.pos : this.c.vehicle.pos;
      const tail = legBetween(this.c.physics, end, ft.start, hop);
      if (tail) leg = [...back.map((w) => w.pos), ...tail];
    }
    if (!leg) {
      this.c.log('warn', `No clear path to the alignment point of opening ${opening.id}.`);
      v.candIdx += 1;
      this.tryCandidate();
      return;
    }
    this.c.setJob(
      this.pilot.followJob(
        'fly_through',
        leg.map((p) => ({ pos: p })),
        () => this.crossIn(),
        0.25,
      ),
    );
  }

  crossIn(): void {
    const v = this.c.visit!;
    this.pilot.trackAperture();
    const ft = new FlyThrough(v.axis!, 'in');
    this.c.setPhase('fly_through');
    this.c.setJob(
      this.pilot.flyThroughJob(ft, (ok) => {
        if (ok) {
          this.c.inside = v.b.id;
          v.entered = true;
          this.c.log(
            'action',
            `Inside ${v.b.label}. Centerline error peaked at ${fmt(ft.maxErrU, 3)} / ${fmt(ft.maxErrV, 3)} m (margin ${fmt(v.axis!.marginU, 2)} / ${fmt(v.axis!.marginV, 2)} m).`,
          );
          this.go.startExplore();
          return;
        }
        v.aborts += 1;
        this.c.log('warn', `Fly-through aborted: ${ft.abortReason}. Backed out.`);
        if (v.aborts < 3) {
          this.crossIn();
        } else {
          v.candIdx += 1;
          this.tryCandidate();
        }
      }),
    );
  }
}

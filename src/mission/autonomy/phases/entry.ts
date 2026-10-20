import type { EntryCandidate, MissionBuilding, V3 } from '../contracts';
import { planApproach } from '../planner/approach';
import { passageAxis, rankCandidates, scoreOpening } from '../planner/entry';
import { FlyThrough } from '../planner/flythrough';
import { buildingBox, legBetween } from '../planner/geometry';
import { indoorGrid, oracleNav } from '../planner/indoor';
import { planInspection } from '../planner/inspect';
import { type VehicleCommand } from '../vehicle';
import {
  type Job,
  type Visit,
  type Waypoint,
  fmt,
  hold,
} from './shared';
import { SearchPhases } from './search';

/** Building approach: inspect the shell, rank the observed openings, ask the operator, and fly through. */
export abstract class EntryPhases extends SearchPhases {
  // ----------------------------------------------------------- building

  protected startBuilding(b: MissionBuilding): void {
    this.visitedBuildings.add(b.id);
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
      vis: null,
      explorer: null,
      sensing: null,
      nav: oracleNav(grid),
      sensorJob: null,
      exploreStart: 0,
      pass: 0,
      policyCollisions: 0,
      entered: false,
      trail: [],
    };
    this.visit = v;
    const plan = planApproach(b, this.vehicle.pos, this.physics, this.air, this.agl);
    if (!plan.ok || !plan.standoff) {
      this.log('warn', `${b.label}: ${plan.reason}. Skipping.`);
      this.visit = null;
      this.continueMission();
      return;
    }
    v.standoff = plan.standoff;
    this.log('action', `Approaching ${b.label}. Standoff ${this.where(plan.standoff.pos)}, open descent column checked.`);
    const wps: Waypoint[] = [
      ...plan.cruise.map((p) => ({ pos: p, tol: 1.5 })),
      ...plan.descent.map((p, i) => ({
        pos: p,
        tol: 0.3,
        yaw: i === plan.descent.length - 1 ? plan.standoff!.yaw : null,
      })),
    ];
    this.setJob(this.followJob('approach', wps, () => this.startInspect(), 0.3));
  }

  protected startInspect(): void {
    const v = this.visit!;
    const steps = planInspection(v.b, this.vehicle.pos, this.physics, this.air);
    this.log('action', `Inspecting the ${v.b.label} envelope from ${steps.filter((s) => s.dwell > 0).length} viewpoints.`);
    const wps: Waypoint[] = steps.map((s) => ({
      pos: s.pos,
      yaw: s.yaw,
      dwell: s.dwell || undefined,
    }));
    this.setJob(this.followJob('inspect', wps, () => this.selectEntry(), 0.25));
  }

  protected selectEntry(): void {
    const v = this.visit!;
    this.setPhase('entry_select');
    const cands = [...v.observed.values()]
      .sort((a, b) => a.id - b.id)
      .map((o) => scoreOpening(v.b, o, this.physics, this.mission.entryPolicy));
    for (const c of cands) {
      this.log(
        'info',
        `Opening ${c.openingId} ${c.kind.replace('_', ' ')}: clearance ${fmt(c.clearanceM, 2)} m, safety ${fmt(c.safety, 2)}, ${c.decision}.`,
      );
    }
    v.ranked = rankCandidates(cands);
    v.candIdx = 0;
    if (!cands.length) {
      this.log('warn', `No openings observed on ${v.b.label}.`);
    }
    if (this.mission.entryPolicy === 'never') {
      const best = [...cands].sort((a, b) => b.safety - a.safety)[0];
      this.entries.push({
        tSec: this.t,
        buildingId: v.b.id,
        openingId: best?.openingId ?? -1,
        safety: best?.safety ?? 0,
        decision: 'never',
      });
      this.log('info', `Entry policy is never. Not entering ${v.b.label}.`);
      this.leaveBuilding();
      return;
    }
    this.tryCandidate();
  }

  protected tryCandidate(): void {
    const v = this.visit!;
    const cand = v.ranked[v.candIdx];
    if (!cand) {
      if (v.denied === 0) {
        const best = [...v.observed.values()][0];
        this.entries.push({
          tSec: this.t,
          buildingId: v.b.id,
          openingId: best?.id ?? -1,
          safety: 0,
          decision: 'none_viable',
        });
      }
      this.log('warn', `No viable entry into ${v.b.label}. Skipping the interior.`);
      this.leaveBuilding();
      return;
    }
    if (cand.decision === 'auto') {
      this.entries.push({
        tSec: this.t,
        buildingId: v.b.id,
        openingId: cand.openingId,
        safety: cand.safety,
        decision: 'auto',
      });
      this.log(
        'action',
        `Entering ${v.b.label} through opening ${cand.openingId} (${cand.kind.replace('_', ' ')}), safety ${fmt(cand.safety, 2)}.`,
      );
      this.goEnter(cand);
      return;
    }
    this.pendingEntry = cand;
    this.operatorAnswer = null;
    this.log(
      'action',
      `Operator approval needed: opening ${cand.openingId} (${cand.kind.replace('_', ' ')}) on ${v.b.label}, safety ${fmt(cand.safety, 2)}.`,
    );
    const at: V3 = [this.vehicle.pos[0], this.vehicle.pos[1], this.vehicle.pos[2]];
    this.setJob({
      phase: 'await_operator',
      plan: () => [],
      tick: (): VehicleCommand => {
        let answer = this.operatorAnswer;
        if (!answer && this.options.operator) {
          const r = this.options.operator(cand);
          if (r !== 'pending') answer = r;
        }
        if (answer) {
          this.operatorAnswer = null;
          this.pendingEntry = null;
          if (answer === 'approve') {
            this.entries.push({
              tSec: this.t,
              buildingId: v.b.id,
              openingId: cand.openingId,
              safety: cand.safety,
              decision: 'approved',
            });
            this.log('action', `Operator approved opening ${cand.openingId}.`);
            this.goEnter(cand);
          } else {
            this.entries.push({
              tSec: this.t,
              buildingId: v.b.id,
              openingId: cand.openingId,
              safety: cand.safety,
              decision: 'denied',
            });
            this.log('info', `Operator denied opening ${cand.openingId}.`);
            v.denied += 1;
            v.candIdx += 1;
            this.tryCandidate();
          }
        }
        return { vel: hold(this.vehicle.pos, at) };
      },
    });
  }

  protected goEnter(cand: EntryCandidate): void {
    const v = this.visit!;
    const opening = v.b.shell.openings.find((o) => o.id === cand.openingId)!;
    v.opening = opening;
    v.axis = passageAxis(opening);
    v.aborts = 0;
    const ft = new FlyThrough(v.axis, 'in');
    const hop = v.box.roofZ + 2.5;
    let leg = legBetween(this.physics, this.vehicle.pos, ft.start, hop);
    const refuge = v.standoff?.pos;
    if (!leg && refuge) {
      // Retrace the inspection path until the alignment point is reachable.
      const back = this.retrace(v.trail, (p) => legBetween(this.physics, p, ft.start, hop) !== null);
      const end = back.length ? back[back.length - 1]!.pos : this.vehicle.pos;
      const tail = legBetween(this.physics, end, ft.start, hop);
      if (tail) leg = [...back.map((w) => w.pos), ...tail];
    }
    if (!leg) {
      this.log('warn', `No clear path to the alignment point of opening ${opening.id}.`);
      v.candIdx += 1;
      this.tryCandidate();
      return;
    }
    this.setJob(
      this.followJob(
        'fly_through',
        leg.map((p) => ({ pos: p })),
        () => this.crossIn(),
        0.25,
      ),
    );
  }

  protected crossIn(): void {
    const v = this.visit!;
    const ft = new FlyThrough(v.axis!, 'in');
    this.setPhase('fly_through');
    this.setJob(
      this.flyThroughJob(ft, (ok) => {
        if (ok) {
          this.inside = v.b.id;
          v.entered = true;
          this.log(
            'action',
            `Inside ${v.b.label}. Centerline error peaked at ${fmt(ft.maxErrU, 3)} / ${fmt(ft.maxErrV, 3)} m (margin ${fmt(v.axis!.marginU, 2)} / ${fmt(v.axis!.marginV, 2)} m).`,
          );
          this.startExplore();
          return;
        }
        v.aborts += 1;
        this.log('warn', `Fly-through aborted: ${ft.abortReason}. Backed out.`);
        if (v.aborts < 3) {
          this.crossIn();
        } else {
          v.candIdx += 1;
          this.tryCandidate();
        }
      }),
    );
  }


  protected flyThroughJob(ft: FlyThrough, onDone: (ok: boolean) => void): Job {
    let called = false;
    return {
      phase: this.phase === 'exit' ? 'exit' : 'fly_through',
      plan: () => [ft.start, ft.end],
      tick: (): VehicleCommand => {
        const s = ft.step(this.vehicle.pos, this.vehicle.vel, this.lastCollided);
        if ((s.state === 'done' || s.state === 'aborted') && !called) {
          called = true;
          onDone(s.state === 'done');
        }
        return s.cmd;
      },
    };
  }
}

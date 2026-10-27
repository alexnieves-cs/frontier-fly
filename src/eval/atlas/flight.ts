// Flies one atlas case and scores it for the two events the atlas estimates: a false clear (a scope
// declared clear that held the target) and a collision. Each event also gets a continuous score in
// [0, 1], 1 exactly when it happened, that the cross-entropy search climbs on the way to the event.
import { readFileSync } from 'node:fs';
import type { LiveDetectionSource, Phase } from '../../mission/autonomy/contracts';
import { createPerceptionSource } from '../../mission/autonomy/perception';
import { policyFromFile } from '../../mission/autonomy/policy';
import { buildReport } from '../../mission/autonomy/report';
import { runMission } from '../../mission/autonomy/runner';
import { buildMissionWorld, missionForWorld } from '../../mission/autonomy/world';
import { SHIPPED_CHECKPOINT_PATH } from '../../rl/checkpoint';
import { missionModeOptions, type MissionMode } from '../missions';
import { caseSetup, type AtlasCase } from './space';

export const ATLAS_EVENTS = ['falseClear', 'collision'] as const;
export type AtlasEvent = (typeof ATLAS_EVENTS)[number];

/** The autopilot the atlas measures: the shipped policy indoors with the planner as backup, on the quadrotor. */
export const ATLAS_MODE: MissionMode = 'policyQuad';

/** A clearance this large or more scores zero on the collision scale, m. */
const CLEARANCE_SCALE_M = 1;

export interface AtlasFlight {
  case: AtlasCase;
  /** The world seed actually flown (the generator moves on when a seed fails to build). */
  worldSeed: number;
  mode: MissionMode;
  outcome: string;
  expected: boolean;
  landedHome: boolean;
  airborne: boolean;
  durationSec: number;
  falseClear: boolean;
  collision: boolean;
  collisions: number;
  /** First contact: when, in which phase, inside which building. */
  firstCollision: { tSec: number; phase: Phase; inside: string | null } | null;
  /** Closest approach to a solid other than the ground. */
  minClearanceM: number;
  clearance: { tSec: number; phase: Phase; inside: string | null; tag: string } | null;
  /** Highest detector confidence on the planted target, 0 when it was never seen (null without a target). */
  targetConf: number | null;
  /** The planted target was confirmed (null without a target). */
  targetConfirmed: boolean | null;
  declaredClear: string[];
  score: Record<AtlasEvent, number>;
}

/**
 * Continuous score toward a false clear: 1 for one. A false clear needs two things, a target the
 * drone never confirmed and a clear declared anyway, so the score climbs with both: an unconfirmed
 * target scores 0.5, plus up to 0.2 for the share of buildings declared clear and up to 0.2 for how
 * close the detector came to believing in it (seen but not confirmed is the precursor). A confirmed
 * target scores under 0.3. The first search scored a target never seen at all highest, and climbed
 * toward flights that saw nothing (camera dropouts, strong wind) instead of toward false clears.
 */
export function falseClearScore(falseClear: boolean, target: { conf: number; confirmed: boolean; clearedShare: number } | null): number {
  if (falseClear) return 1;
  if (!target) return 0;
  if (target.confirmed) return 0.3 * (1 - Math.min(1, target.conf));
  return 0.5 + 0.2 * Math.min(1, target.clearedShare) + 0.2 * Math.min(1, target.conf / 0.8);
}

/** Continuous score toward a collision: 1 for contact, else how close the drone came to a solid. */
export function collisionScore(collision: boolean, minClearanceM: number): number {
  if (collision) return 1;
  return 0.9 * Math.max(0, 1 - Math.max(0, minClearanceM) / CLEARANCE_SCALE_M);
}

/** Wraps a detection source to record the best confidence it ever reported on the planted targets. */
function watchTargets(source: LiveDetectionSource, targetIds: ReadonlySet<string>): { source: LiveDetectionSource; best: () => number } {
  let best = 0;
  return {
    source: {
      id: source.id,
      observe(pose, tSec) {
        const obs = source.observe(pose, tSec);
        for (const c of obs.contacts) if (c.truth.kind === 'tp' && targetIds.has(c.truth.targetId)) best = Math.max(best, c.confidence);
        return obs;
      },
      truthClearStatus: (scope, category) => source.truthClearStatus(scope, category),
      truthTargets: () => source.truthTargets(),
    },
    best: () => best,
  };
}

let shipped: ReturnType<typeof policyFromFile> | null = null;

export async function flyCase(c: AtlasCase, mode: MissionMode = ATLAS_MODE): Promise<AtlasFlight> {
  const setup = caseSetup(c);
  const world = buildMissionWorld(setup.world);
  const mission = missionForWorld(world, setup.mission);
  const raw = createPerceptionSource(world);
  const planted = new Set(raw.truthTargets().filter((t) => t.category !== 'decoy').map((t) => t.id));
  const watched = watchTargets(raw, planted);
  if (mode.startsWith('policy') && !shipped) shipped = policyFromFile(readFileSync(SHIPPED_CHECKPOINT_PATH));
  const run = await runMission(mission, world, watched.source, {
    dt: 0.05,
    operator: () => 'approve',
    ...missionModeOptions(mode, shipped ?? undefined),
    ...(setup.faults.length ? { faults: { plan: setup.faults, seed: world.seed } } : {}),
    measureClearance: true,
  });
  const report = buildReport(run, raw, world);
  const phaseAt = (t: number): Phase => run.phases.find((p) => t >= p.tStart - 1e-9 && t <= p.tEnd + 1e-9)?.phase ?? run.phases[run.phases.length - 1]!.phase;
  const first = run.collisions[0];
  const falseClear = report.falseClears > 0;
  const collision = run.collisions.length > 0;
  const minClearanceM = Math.min(run.clearance?.minM ?? Infinity, collision ? 0 : Infinity);
  const targetConf = planted.size ? watched.best() : null;
  const targetConfirmed = planted.size ? run.confirmed.some((c) => c.truth.kind === 'tp' && planted.has(c.truth.targetId)) : null;
  const clearedShare = new Set(run.declaredClear.map((d) => d.scope).filter((s) => s !== 'outdoor')).size / world.buildings.length;
  return {
    case: c,
    worldSeed: world.seed,
    mode,
    outcome: run.outcome,
    expected: run.outcome === (setup.decoded.mission === 'sweep-empty' ? 'clear' : 'found') && run.landedHome,
    landedHome: run.landedHome,
    airborne: run.log.some((l) => /Time limit .* reached before landing/.test(l.message)),
    durationSec: Math.round(run.tSec),
    falseClear,
    collision,
    collisions: run.collisions.length,
    firstCollision: first ? { tSec: Math.round(first.tSec * 10) / 10, phase: phaseAt(first.tSec), inside: first.inside } : null,
    minClearanceM: Number.isFinite(minClearanceM) ? Math.round(minClearanceM * 1000) / 1000 : 99,
    clearance: run.clearance
      ? { tSec: Math.round(run.clearance.tSec * 10) / 10, phase: run.clearance.phase, inside: run.clearance.inside, tag: run.clearance.tag }
      : null,
    targetConf: targetConf === null ? null : Math.round(targetConf * 1000) / 1000,
    targetConfirmed,
    declaredClear: run.declaredClear.map((d) => d.scope),
    score: {
      falseClear: falseClearScore(falseClear, targetConf === null ? null : { conf: targetConf, confirmed: !!targetConfirmed, clearedShare }),
      collision: collisionScore(collision, minClearanceM),
    },
  };
}

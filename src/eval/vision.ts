// Search by sight: the mission bench flown twice per world, once with the analytic detector and once
// with the rendered camera and the learned detector, on the same fresh seeds.
// scripts/study/vision-study.ts runs it in shards.
import { createPerceptionSource } from '../mission/autonomy/perception';
import { SCENARIOS } from '../mission/autonomy/scenarios';
import type { FrameDetector } from '../vision/detector';
import type { HumanoidPoses } from '../vision/humanoid';
import { createVisionSource } from '../vision/source';
import { flyMission, summarizeMissions, wilson, type MissionFlight, type MissionMode } from './missions';

/**
 * Fresh block for this track; 66601-66616 were the only mission seeds looked at while building it.
 * 67101-67130 was the first reported block. 67201-67230 is reported since the wall-placed contact fix
 * (phases/search.ts nearestOpen), flown once on the fix and once on the code before it.
 */
export const VISION_SEED_BASE = 67201;
export const VISION_SEEDS_PER_SCENARIO = 30;
export const VISION_MODES = ['policy', 'sensor'] as const satisfies readonly MissionMode[];
export const PERCEPTIONS = ['analytic', 'vision'] as const;
export type Perception = (typeof PERCEPTIONS)[number];

export interface VisionJob {
  scenario: string;
  seed: number;
  indoor: (typeof VISION_MODES)[number];
  perception: Perception;
}

export interface VisionFlight extends MissionFlight {
  perception: Perception;
  /** Camera frames rendered and read by the detector. Zero for the analytic arm. */
  frames: number;
  /** Seconds from launch to the first confirmed true find, or null. */
  firstFindSec: number | null;
  wallSec: number;
}

export function visionJobs(seeds = VISION_SEEDS_PER_SCENARIO, base = VISION_SEED_BASE): VisionJob[] {
  const out: VisionJob[] = [];
  for (const scenario of SCENARIOS) {
    for (let i = 0; i < seeds; i++) {
      for (const indoor of VISION_MODES) for (const perception of PERCEPTIONS) out.push({ scenario: scenario.id, seed: base + i, indoor, perception });
    }
  }
  return out;
}

export async function flyVisionMission(job: VisionJob, poses: HumanoidPoses, detector: FrameDetector): Promise<VisionFlight> {
  let frames = 0;
  let firstFind: number | null = null;
  const t0 = Date.now();
  const flight = await flyMission({ scenario: job.scenario, seed: job.seed, indoor: job.indoor }, (world) => {
    const source = job.perception === 'vision'
      ? createVisionSource(world, poses, detector, { onFrame: () => frames++ })
      : createPerceptionSource(world);
    return source;
  }, (run) => {
    const hit = run.confirmed.find((c) => c.truth.kind === 'tp');
    firstFind = hit ? Math.round(hit.tSec) : null;
  });
  return { ...flight, perception: job.perception, frames, firstFindSec: firstFind, wallSec: Math.round((Date.now() - t0) / 1000) };
}

/** Paired difference in expected-outcome rate, vision minus analytic, over the same (scenario, seed, mode). */
function paired(flights: VisionFlight[]) {
  const key = (f: VisionFlight) => `${f.scenario}:${f.seed}:${f.indoor}`;
  const analytic = new Map(flights.filter((f) => f.perception === 'analytic').map((f) => [key(f), f]));
  let both = 0;
  let onlyAnalytic = 0;
  let onlyVision = 0;
  let neither = 0;
  for (const v of flights.filter((f) => f.perception === 'vision')) {
    const a = analytic.get(key(v));
    if (!a) continue;
    if (a.expected && v.expected) both++;
    else if (a.expected) onlyAnalytic++;
    else if (v.expected) onlyVision++;
    else neither++;
  }
  return { pairs: both + onlyAnalytic + onlyVision + neither, both, onlyAnalytic, onlyVision, neither };
}

export function visionBenchReport(flights: VisionFlight[], commit: string, seeds: number, base: number) {
  const group = (perception: Perception, indoor?: string, scenario?: string) => {
    const sel = flights.filter((f) => f.perception === perception && (!indoor || f.indoor === indoor) && (!scenario || f.scenario === scenario));
    const finds = sel.map((f) => f.firstFindSec).filter((t): t is number => t != null);
    const ok = sel.filter((f) => f.expected).length;
    return {
      ...summarizeMissions(sel),
      expectedOutcomeCi: wilson(ok, sel.length),
      meanFirstFindSec: finds.length ? Math.round(finds.reduce((a, b) => a + b, 0) / finds.length) : null,
      framesPerFlight: sel.length ? Math.round(sel.reduce((a, f) => a + f.frames, 0) / sel.length) : 0,
      wallSecPerFlight: sel.length ? Math.round(sel.reduce((a, f) => a + f.wallSec, 0) / sel.length) : 0,
    };
  };
  const arms = Object.fromEntries(
    PERCEPTIONS.map((p) => [
      p,
      {
        all: group(p),
        ...Object.fromEntries(VISION_MODES.map((m) => [m, group(p, m)])),
        ...Object.fromEntries(SCENARIOS.map((s) => [s.id, group(p, undefined, s.id)])),
      },
    ]),
  );
  return { schema: 1, commit, seedsPerScenario: seeds, seedBase: base, modes: VISION_MODES, arms, paired: paired(flights), flights };
}

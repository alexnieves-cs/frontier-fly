// Headline benchmark numbers shared by the homepage and the guided tour's closing card.
// They come from the coverage study (docs/study.json: every pilot on the same 30 fresh buildings per
// condition, bootstrap 95% intervals), the same cells the README, GUIDE and BENCHMARKS target tables quote.
// The small fixed protocol in docs/benchmarks.json is a smoke check and is not shown here.
// docs/headline.json is the small extract of those cells that scripts/docs/generate.mjs writes, so the page
// does not ship the study's per-episode rows.
import { cells as studyCells, commit, conditions, seedsPerCondition } from '../../docs/headline.json';

interface StudyCell {
  condition: string;
  lighting: string;
  pilot: string;
  episodes: number;
  coverage: number;
  coverageCi: number[];
  collisionsPerEpisode: number;
  episodesWithCollision: number;
}

export const pct = (x: number): string => `${(x * 100).toFixed(1)}%`;

const cells = studyCells as StudyCell[];

function cell(condition: string, lighting: string, pilot: string): StudyCell {
  const c = cells.find((x) => x.condition === condition && x.lighting === lighting && x.pilot === pilot);
  if (!c) throw new Error(`docs/study.json has no ${condition}/${lighting}/${pilot} cell`);
  return c;
}

export interface PilotResult {
  name: string;
  value: number;
  ci: [number, number];
  /** Wall contacts per flight on the same buildings. */
  contacts: number;
  tone: 'policy' | 'teacher' | 'base';
}

function pilot(id: string, name: string, tone: PilotResult['tone']): PilotResult {
  const c = cell('hard', 'day', id);
  return { name, value: c.coverage, ci: [c.coverageCi[0] ?? NaN, c.coverageCi[1] ?? NaN], contacts: c.collisionsPerEpisode, tone };
}

/** Headline numbers from docs/study.json, computed the way scripts/docs/generate.mjs computes its target table. */
export function results() {
  const hard = conditions.find((c) => c.id === 'hard')!;
  const policy = pilot('policy', 'Learned policy (ships in the page)', 'policy');
  const teacher = pilot('teacher', 'Oracle (the privileged teacher)', 'teacher');
  return {
    hardStage: hard.stage,
    seeds: seedsPerCondition,
    pilots: [
      policy,
      teacher,
      pilot('classical', 'Sensor-only planner (no learning)', 'base'),
      pilot('waypoint', 'Frontier follower (no learning)', 'base'),
      pilot('heuristic', 'Lidar heuristic', 'base'),
    ],
    policyLeads: policy.value > teacher.value,
    easy: cell('easy', 'day', 'policy').coverage,
    nightOverDay: cell('hard', 'night-dark', 'policy').coverage / policy.value,
    commit: commit.slice(0, 7),
  };
}

export type Results = ReturnType<typeof results>;

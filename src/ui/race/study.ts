import study from '../../../docs/study.json';
import type { RacePilot } from './pilots';

/** The published coverage study (docs/study.json), for the race's "same as the study" check and the averages. */

export interface StudyEpisode {
  coverage: number;
  collisions: number;
  steps: number;
}

export interface StudyCell {
  coverage: number;
  coverageCi: [number, number];
  episodes: number;
  collisionsPerEpisode: number;
}

export function studyEpisode(condition: string, seed: number, lighting: string, pilot: RacePilot): StudyEpisode | null {
  const e = study.episodes.find((r) => r.condition === condition && r.seed === seed && r.lighting === lighting && r.pilot === pilot);
  return e ? { coverage: e.coverage, collisions: e.collisions, steps: e.steps } : null;
}

export function studyCell(condition: string, lighting: string, pilot: RacePilot): StudyCell | null {
  const c = study.cells.find((r) => r.condition === condition && r.lighting === lighting && r.pilot === pilot);
  return c ? { coverage: c.coverage, coverageCi: c.coverageCi as [number, number], episodes: c.episodes, collisionsPerEpisode: c.collisionsPerEpisode } : null;
}

export const STUDY_COMMIT: string = study.commit;

/** True when a live lane ended exactly where the study's episode did. */
export function matchesStudy(live: StudyEpisode, ref: StudyEpisode): boolean {
  return live.steps === ref.steps && live.collisions === ref.collisions && Math.abs(live.coverage - ref.coverage) < 1e-9;
}

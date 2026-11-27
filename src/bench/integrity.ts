/**
 * The integrity track (FrontierBench Hard, round 6): the drift track's flights, where the pilot also
 * says how far off its own pose might be. After every action the pilot reports a protection level
 * (BenchPilot.protectionLevel): a horizontal radius, metres, that it claims holds its true position.
 * A level at or under the alert limit says "my pose is good enough to fly on"; above it, or Infinity,
 * says "I am not sure".
 *
 * Borrowed from satellite navigation's integrity terms (alert limit, time to alert, hazardously
 * misleading information), scored per step against the truth:
 *
 * - available: the level is within INTEGRITY.alertLimit.
 * - hazardous: available, but the true error is over the alert limit (false confidence).
 * - HMI: a hazardous step after the first INTEGRITY.ttaS seconds of a hazardous run, the time a
 *   monitor gets to notice before the confidence counts as misleading.
 *
 * Score = availability - INTEGRITY.hmiWeight × HMI rate. Honest uncertainty costs the step's
 * availability; false confidence costs ten times as much, so the score goes below zero when a pilot
 * is confidently wrong for more than a tenth of the flight (the floor is -9). There is no floor at 0
 * per flight: with one, a pilot that is never unsure loses nothing more on a flight where it is lost
 * for minutes than on one where it is lost for seconds, and "always confident" beat every honest
 * monitor on dev (docs/INTEGRITY.md). A pilot that reports nothing claims a level of 0 on every
 * step: it flies as if its pose were exact.
 */
import type { PoseHealth } from '../loc/types';

export const INTEGRITY = {
  /** Alert limit, horizontal metres: the return tasks' HOME_RADIUS. */
  alertLimit: 1.5,
  /** Time to alert, seconds. */
  ttaS: 2,
  /** What one second of hazardously misleading confidence costs, in seconds of honest "not sure". */
  hmiWeight: 10,
} as const;

/** One flight's integrity columns. */
export interface IntegrityStats {
  /** Availability - hmiWeight × HMI rate (may be negative): the track's score. */
  score: number;
  /** Share of the step budget the pilot said its pose was good (level within the alert limit). */
  availability: number;
  /** Share of the step budget that was hazardously misleading (after the time to alert). */
  hmiRate: number;
  /** Share of steps the true error was over the alert limit, whatever the pilot said. */
  badShare: number;
  /** Share of steps the true error was over the level the pilot claimed (misleading, hazardous or not). */
  misleadShare: number;
  /** Share of the bad-pose steps the pilot flagged (level over the alert limit), or null with none. */
  caught: number | null;
}

/** Steps the integrity scorer through a flight. */
export class IntegrityMeter {
  private n = 0;
  private available = 0;
  private hmi = 0;
  private bad = 0;
  private mislead = 0;
  private flagged = 0;
  private run = 0;

  /** One step: the true horizontal error of the pose the pilot flew on, and the level it claimed. */
  step(error: number, level: number): void {
    const pl = Number.isNaN(level) ? 0 : level;
    const ok = pl <= INTEGRITY.alertLimit;
    const bad = error > INTEGRITY.alertLimit;
    this.n += 1;
    if (ok) this.available += 1;
    if (error > pl) this.mislead += 1;
    if (bad) {
      this.bad += 1;
      if (!ok) this.flagged += 1;
    }
    if (ok && bad) {
      this.run += 1;
      if (this.run > INTEGRITY.ttaS * 20) this.hmi += 1;
    } else this.run = 0;
  }

  /**
   * The flight's columns. `budget` is the task's step budget: a flight a crash ended early is
   * unavailable for the steps it did not fly, so crashing out of a bad pose is no way to score.
   * Bad-pose and misleading shares stay over the steps flown.
   */
  stats(budget = 0): IntegrityStats {
    const n = Math.max(1, this.n, budget);
    const flown = Math.max(1, this.n);
    const availability = this.available / n;
    const hmiRate = this.hmi / n;
    return {
      score: availability - INTEGRITY.hmiWeight * hmiRate,
      availability,
      hmiRate,
      badShare: this.bad / flown,
      misleadShare: this.mislead / flown,
      caught: this.bad ? this.flagged / this.bad : null,
    };
  }
}

/** How a pilot turns the estimator's own health (SensorFrame.health) into a protection level. */
export type IntegrityMonitor = (health: PoseHealth | undefined) => number;

/** The house pilot's monitor (planner-view): the estimator's own 1σ, chosen on dev 160001+ (docs/INTEGRITY.md). */
export const HOUSE_MONITOR = { id: 'sigma', k: 1 } as const;

/** The monitors the integrity study compares (docs/INTEGRITY.md), all on the estimator's own health. */
export const MONITORS: Record<string, { label: string; make: (k: number) => IntegrityMonitor }> = {
  always: { label: 'Always confident (level 0)', make: () => () => 0 },
  never: { label: 'Never confident (level ∞)', make: () => () => Infinity },
  sigma: { label: 'k × the estimator’s 1σ', make: (k) => (h) => (h ? k * h.sigmaM : 0) },
  'sigma-health': { label: 'k × 1σ while tracking, ∞ once degraded or lost', make: (k) => (h) => (!h ? 0 : h.state === 'tracking' && h.ok ? k * h.sigmaM : Infinity) },
};

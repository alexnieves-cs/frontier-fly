// Cross-entropy search and importance-sampled rates for the failure atlas.
//
// The search (Rubinstein's cross-entropy method for rare events) draws a batch from the current
// proposal, keeps the top RHO of it by the event's score (or every flight where the event happened,
// once that is more than RHO), and moves the proposal toward the kept flights, weighting each by
// its likelihood ratio to the nominal so the update targets the nominal's own failure region. The
// final estimate draws fresh cases from a defensive mixture of the nominal and the learned proposal,
// which bounds every weight by 1/alpha, and reports the weighted failure rate with a CLT interval.
import { wilson } from '../missions';
import type { AtlasEvent, AtlasFlight } from './flight';
import { DIMS, mixture, nominalProposal, weight, type Proposal } from './space';

/** Share of a batch kept as the elite. */
export const RHO = 0.1;
/** Weight of the new estimate in each update; the rest stays on the previous proposal. */
export const SMOOTHING = 0.7;
/** No category falls below this probability, so every case stays reachable and weights stay bounded. */
export const FLOOR = 0.01;
/** Share of the nominal in the estimation mixture. Weights are at most 1/ALPHA. */
export const ALPHA = 0.3;

export interface CeStep {
  /** Score level that defined the elite. 1 means only flights where the event happened. */
  level: number;
  elite: number;
  failures: number;
  next: Proposal;
}

/** One cross-entropy update from a batch drawn from `current`. */
export function ceUpdate(current: Proposal, flights: readonly AtlasFlight[], event: AtlasEvent): CeStep {
  const scores = flights.map((f) => f.score[event]).sort((a, b) => b - a);
  const failures = flights.filter((f) => f.score[event] >= 1).length;
  const quantile = scores[Math.max(0, Math.ceil(RHO * scores.length) - 1)] ?? 1;
  const level = Math.min(1, quantile);
  const elite = flights.filter((f) => f.score[event] >= level);
  const next = current.map((p, d) => {
    const counts = p.map(() => 0);
    let total = 0;
    for (const f of elite) {
      const w = weight(current, f.case);
      counts[f.case.k[d]!]! += w;
      total += w;
    }
    const fit = total > 0 ? counts.map((x) => x / total) : p;
    const smoothed = fit.map((x, j) => SMOOTHING * x + (1 - SMOOTHING) * p[j]!);
    const floored = smoothed.map((x) => Math.max(FLOOR, x));
    const sum = floored.reduce((a, b) => a + b, 0);
    return floored.map((x) => x / sum);
  });
  return { level, elite: elite.length, failures, next };
}

export interface RateEstimate {
  flights: number;
  failures: number;
  /** Estimated probability per flight under the nominal distribution. */
  p: number;
  ci: [number, number];
  /** Effective sample size of the weights (n for plain Monte Carlo). */
  ess: number;
  method: 'importance' | 'monte-carlo';
}

/** Plain Monte Carlo rate, flights drawn from the nominal, with a Wilson interval. */
export function monteCarloRate(flights: readonly AtlasFlight[], event: AtlasEvent): RateEstimate {
  const k = flights.filter((f) => f[event]).length;
  return { flights: flights.length, failures: k, p: flights.length ? k / flights.length : 0, ci: wilson(k, flights.length), ess: flights.length, method: 'monte-carlo' };
}

/**
 * Importance-sampled rate, flights drawn from `sampling`. The interval is the normal one on the
 * weighted mean. With no failure at all it falls back to the bound the mixture guarantees: every
 * failing case is at least ALPHA times as likely under the mixture, so 3 / (ALPHA n) bounds the
 * rate at 95%.
 */
export function importanceRate(flights: readonly AtlasFlight[], event: AtlasEvent, sampling: Proposal, alpha = ALPHA): RateEstimate {
  const n = flights.length;
  const w = flights.map((f) => weight(sampling, f.case));
  const y = flights.map((f, i) => (f[event] ? w[i]! : 0));
  const mean = y.reduce((a, b) => a + b, 0) / Math.max(1, n);
  const varY = y.reduce((a, b) => a + (b - mean) ** 2, 0) / Math.max(1, n - 1);
  const se = Math.sqrt(varY / Math.max(1, n));
  const sw = w.reduce((a, b) => a + b, 0);
  const sw2 = w.reduce((a, b) => a + b * b, 0);
  const failures = flights.filter((f) => f[event]).length;
  const ci: [number, number] = failures ? [Math.max(0, mean - 1.96 * se), Math.min(1, mean + 1.96 * se)] : [0, Math.min(1, 3 / (alpha * Math.max(1, n)))];
  return { flights: n, failures, p: mean, ci, ess: sw2 > 0 ? (sw * sw) / sw2 : 0, method: 'importance' };
}

/** The proposal the estimate draws from: the learned one mixed with the nominal. */
export function estimationProposal(learned: Proposal, alpha = ALPHA): Proposal {
  return mixture(learned, alpha);
}

export interface Slice {
  dim: string;
  value: string;
  flights: number;
  failures: number;
  rate: number;
  ci: [number, number];
}

/** Conditional failure rates per value of every dimension, from nominal draws. Values with no flights are left out. */
export function slices(flights: readonly AtlasFlight[], event: AtlasEvent): Slice[] {
  const out: Slice[] = [];
  DIMS.forEach((dim, d) => {
    dim.values.forEach((value, j) => {
      const mine = flights.filter((f) => f.case.k[d] === j);
      if (!mine.length) return;
      const k = mine.filter((f) => f[event]).length;
      out.push({ dim: dim.id, value, flights: mine.length, failures: k, rate: k / mine.length, ci: wilson(k, mine.length) });
    });
  });
  return out;
}

/** How far a learned proposal moved from the nominal, per dimension: the categories it favours most. */
export function proposalShift(learned: Proposal): { dim: string; label: string; top: { value: string; nominal: number; learned: number }[] }[] {
  const nominal = nominalProposal();
  return DIMS.map((dim, d) => ({
    dim: dim.id,
    label: dim.label,
    top: dim.values
      .map((value, j) => ({ value, nominal: nominal[d]![j]!, learned: learned[d]![j]! }))
      .sort((a, b) => b.learned / b.nominal - a.learned / a.nominal)
      .slice(0, 3),
  }));
}

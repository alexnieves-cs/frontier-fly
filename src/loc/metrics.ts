import type { PoseEstimate, TrackState, V3 } from './types';

/**
 * Accuracy of an estimated trajectory against the truth, and how honest the
 * estimator was about it. All errors are in the building frame the mission
 * plans in, so the initial offset counts: a map that starts 10 cm off is 10 cm
 * off for the whole visit.
 */
export interface LocMetrics {
  ticks: number;
  /** Metres flown (truth). */
  pathM: number;
  /** Root mean square horizontal position error, m. */
  ateRmse: number;
  ateMax: number;
  finalErr: number;
  /** Final error per metre flown, %. */
  driftPct: number;
  /** Mean over 5 m windows of the change in horizontal error across the window, m (relative drift). */
  rpe5m: number;
  yawRmseDeg: number;
  zRmse: number;
  /** RMSE after removing the entry position offset: drift against the drone's own map, m. */
  ateMapRmse: number;
  /**
   * Share of ticks with the map-relative error inside the filter's own 95%
   * ellipse (2-dof chi-square 5.99). Ideal 0.95; lower is overconfident.
   */
  within95: number;
  /** Mean normalized estimation error squared over (x, y), map-relative; 2 is ideal. */
  nees: number;
  /** Share of ticks with the building-frame error under 2.45 x the reported sigmaM (95% if isotropic). */
  within95Abs: number;
  /** Ticks with error > 0.5 m. */
  bigErrTicks: number;
  /** Episodes of error > 0.5 m (count of onsets). */
  bigErrEvents: number;
  /** Of ticks with error > 0.5 m, the share the estimator flagged as degraded or lost. */
  failDetected: number;
  /** Of ticks the estimator called lost, the share where it really was > 0.5 m off. */
  lostPrecision: number;
  lostTicks: number;
  degradedTicks: number;
}

const BIG = 0.5;

export class LocScorer {
  private n = 0;
  private path = 0;
  private sqErr = 0;
  private maxErr = 0;
  private lastErr = 0;
  private sqYaw = 0;
  private sqZ = 0;
  private inside = 0;
  private insideAbs = 0;
  private sqMap = 0;
  private e0: [number, number] | null = null;
  private neesSum = 0;
  private big = 0;
  private bigEvents = 0;
  private wasBig = false;
  private bigFlagged = 0;
  private lost = 0;
  private lostRight = 0;
  private degraded = 0;
  private prev: V3 | null = null;
  /** (path at sample, error) samples for the 5 m relative drift. */
  private readonly trace: { s: number; ex: number; ey: number }[] = [];

  add(truth: { pos: V3; yaw: number }, est: PoseEstimate): void {
    const ex = est.pos[0] - truth.pos[0];
    const ey = est.pos[1] - truth.pos[1];
    const e = Math.hypot(ex, ey);
    if (this.prev) this.path += Math.hypot(truth.pos[0] - this.prev[0], truth.pos[1] - this.prev[1]);
    this.prev = [truth.pos[0], truth.pos[1], truth.pos[2]];
    this.n += 1;
    this.sqErr += e * e;
    this.maxErr = Math.max(this.maxErr, e);
    this.lastErr = e;
    const dyaw = Math.atan2(Math.sin(est.yaw - truth.yaw), Math.cos(est.yaw - truth.yaw));
    this.sqYaw += dyaw * dyaw;
    const dz = est.pos[2] - truth.pos[2];
    this.sqZ += dz * dz;
    if (!this.e0) this.e0 = [ex, ey];
    const mx = ex - this.e0[0];
    const my = ey - this.e0[1];
    this.sqMap += mx * mx + my * my;
    // Mahalanobis over (x, y) with the reported map-relative covariance.
    const a = est.cov[0]!;
    const b = est.cov[1]!;
    const d = est.cov[4]!;
    const det = a * d - b * b;
    const m2 = det > 1e-18 ? (d * mx * mx - 2 * b * mx * my + a * my * my) / det : Infinity;
    this.neesSum += Math.min(m2, 1e4);
    if (m2 <= 5.991) this.inside += 1;
    if (e <= 2.448 * est.health.sigmaM) this.insideAbs += 1;
    const state: TrackState = est.health.state;
    if (state === 'lost') {
      this.lost += 1;
      if (e > BIG) this.lostRight += 1;
    } else if (state === 'degraded') this.degraded += 1;
    if (e > BIG) {
      this.big += 1;
      if (state !== 'tracking') this.bigFlagged += 1;
      if (!this.wasBig) this.bigEvents += 1;
      this.wasBig = true;
    } else this.wasBig = false;
    this.trace.push({ s: this.path, ex, ey });
  }

  result(): LocMetrics {
    const n = Math.max(1, this.n);
    let rpe = 0;
    let windows = 0;
    let j = 0;
    for (let i = 0; i < this.trace.length; i++) {
      const a = this.trace[i]!;
      while (j < this.trace.length && this.trace[j]!.s - a.s < 5) j++;
      if (j >= this.trace.length) break;
      const b = this.trace[j]!;
      rpe += Math.hypot(b.ex - a.ex, b.ey - a.ey);
      windows += 1;
      // Non-overlapping windows.
      i = j - 1;
    }
    return {
      ticks: this.n,
      pathM: this.path,
      ateRmse: Math.sqrt(this.sqErr / n),
      ateMax: this.maxErr,
      finalErr: this.lastErr,
      driftPct: this.path > 1 ? (this.lastErr / this.path) * 100 : 0,
      rpe5m: windows ? rpe / windows : 0,
      yawRmseDeg: (Math.sqrt(this.sqYaw / n) * 180) / Math.PI,
      zRmse: Math.sqrt(this.sqZ / n),
      ateMapRmse: Math.sqrt(this.sqMap / n),
      within95: this.inside / n,
      nees: this.neesSum / n,
      within95Abs: this.insideAbs / n,
      bigErrTicks: this.big,
      bigErrEvents: this.bigEvents,
      failDetected: this.big ? this.bigFlagged / this.big : 1,
      lostPrecision: this.lost ? this.lostRight / this.lost : 1,
      lostTicks: this.lost,
      degradedTicks: this.degraded,
    };
  }
}

import { LikelihoodGrid } from './grid';
import { DEFAULT_MATCH, eigSym3, endpoints, invert3, matchScan, searchScan, type MatchOptions } from './match';
import type { AltSample, FlowSample, ImuSample, LocFrame, PoseEstimate, PoseHealth, ScanSample, TrackState, V3 } from './types';

/**
 * Indoor pose estimator: an extended Kalman filter over (x, y, yaw, gyro
 * bias) driven by the gyro and a body-velocity filter (accelerometer
 * prediction, optical-flow correction), corrected by matching each lidar
 * sweep against the map the estimator builds itself (2D SLAM without loop
 * closure, in the spirit of Hector SLAM with an odometry prior). Height comes
 * from the barometer, refined by the down range finder.
 *
 * It reports its own health from its covariance and from how well the
 * sweeps agree with its map, and re-acquires with a correlative search when
 * the sweeps stop agreeing. Nothing here reads the floor plan or the true pose.
 */

export type EstimatorMode =
  /** Odometry, scan matching and the map, as shipped. */
  | 'fused'
  /** Gyro and flow only: what the drone would have without the lidar. */
  | 'odometry'
  /** Scan matching with a constant-position prior: no IMU or flow between sweeps. */
  | 'scan';

export interface EstimatorOptions {
  mode: EstimatorMode;
  /** Map extent in the building frame, m. */
  bounds: { x0: number; y0: number; x1: number; y1: number };
  floors: number;
  floorHeight: number;
  /** Map cell size, m. */
  res: number;
  match: MatchOptions;
  /** Gyro white noise, rad/s per sample, and bias random walk, rad/s/√s. */
  gyroSigma: number;
  gyroBiasRW: number;
  /** Initial gyro bias 1-sigma, rad/s. */
  gyroBiasSigma: number;
  /** Accelerometer noise including bias, m/s^2. */
  accelSigma: number;
  /** Flow noise: abs + rel * speed, m/s. */
  flowSigma: number;
  flowSigmaRel: number;
  /**
   * Position random walk per metre flown, m/√m. Far above the flow scale
   * error alone: it also stands for the correlation between ticks the filter
   * doesn't model. Tuned with match.sigma on the dev seeds so the reported
   * covariance holds the true error about 95% of the time.
   */
  odomRel: number;
  /** A new map keyframe after this much motion, m or rad. */
  keyframeDist: number;
  keyframeYaw: number;
  /** A match needs this many returns near the map, and this share of them close. */
  minUsed: number;
  minInlierShare: number;
  /** Health thresholds on sigmaM, m. */
  degradedSigma: number;
  lostSigma: number;
  /** Sweeps in a row that contradict the map before the estimator calls itself lost. */
  lostAfter: number;
  /**
   * Manhattan heading: walls run along the building's axes (the frame the
   * drone measured from outside), so the dominant wall direction in a sweep
   * measures yaw modulo 90°. Removes the entry heading error and gyro drift.
   */
  manhattan: boolean;
  /** 1-sigma of a Manhattan heading fix at full wall agreement, rad. */
  manhattanSigma: number;
  /**
   * Scan information below this along a direction (1/m^2, or 1/rad^2 for a
   * turn) marks the direction degenerate: the match may not move the pose
   * along it, and it adds no information there (Zhang, Kaess & Singh 2016).
   */
  degenerateInfo: number;
}

export function defaultEstimatorOptions(
  bounds: EstimatorOptions['bounds'],
  floors = 1,
  floorHeight = 3,
): EstimatorOptions {
  return {
    mode: 'fused',
    bounds,
    floors,
    floorHeight,
    res: 0.2,
    match: DEFAULT_MATCH,
    gyroSigma: 0.012,
    gyroBiasRW: 0.0006,
    gyroBiasSigma: 0.006,
    accelSigma: 0.15,
    flowSigma: 0.03,
    flowSigmaRel: 0.06,
    odomRel: 0.25,
    keyframeDist: 0.25,
    keyframeYaw: 0.12,
    minUsed: 8,
    minInlierShare: 0.55,
    degradedSigma: 0.2,
    lostSigma: 0.5,
    lostAfter: 10,
    manhattan: true,
    manhattanSigma: 0.01,
    degenerateInfo: 25,
  };
}

/** Diagnostics of the last tick, for tests and the study. */
export interface EstimatorStats {
  sweeps: number;
  matched: number;
  rejected: number;
  keyframes: number;
  recoveries: number;
  /** Sweeps whose returns landed in space the map knows is free. */
  conflicts: number;
  /** Manhattan heading fixes applied. */
  headingFixes: number;
  lastInlierShare: number;
  lastConflictShare: number;
}

const N = 4;
/** Height band around an upper slab in which a sweep is neither matched nor mapped, m. */
const SLAB_BAND_M = 0.35;

function wrap(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

export class PoseEstimator {
  readonly opts: EstimatorOptions;
  x = 0;
  y = 0;
  yaw = 0;
  z = 0;
  /** Gyro bias estimate, rad/s. */
  bias = 0;
  /** Body-frame velocity estimate and its per-axis variance. */
  private vb: [number, number] = [0, 0];
  private vVar = 0.01;
  /** Row-major 4x4 covariance over (x, y, yaw, bias). */
  readonly P = new Float64Array(N * N);
  private readonly grids = new Map<number, LikelihoodGrid>();
  private kfX = NaN;
  private kfY = NaN;
  private kfYaw = NaN;
  private kfFloor = -1;
  private inconsistent = 0;
  private lostLatched = false;
  private sinceSearch = 0;
  private searchLevel = 0;
  private baroZ = NaN;
  private baroBias = 0;
  /** Height of the surface under the drone on each floor, learned from the range finder. */
  private readonly floorRef = new Map<number, number>();
  /**
   * 1-sigma of where the map itself sits in the building frame: the entry
   * position error, which matching against the drone's own map cannot remove.
   * P is relative to the map; health adds this back.
   */
  anchorSigma = 0;
  readonly stats: EstimatorStats = {
    sweeps: 0,
    matched: 0,
    rejected: 0,
    keyframes: 0,
    recoveries: 0,
    conflicts: 0,
    headingFixes: 0,
    lastInlierShare: 0,
    lastConflictShare: 0,
  };

  constructor(opts: EstimatorOptions) {
    this.opts = opts;
  }

  /**
   * Start at a pose the drone believes, with its 1-sigma uncertainty. At a
   * window entry this comes from the outdoor phase: GNSS plus the lidar
   * envelope the drone measured from outside. The position error becomes the
   * map's anchor error; the heading error stays in the filter, where Manhattan
   * fixes can remove it.
   */
  reset(pos: V3, yaw: number, sigmaXY = 0.1, sigmaYaw = 0.03): void {
    this.x = pos[0];
    this.y = pos[1];
    this.z = pos[2];
    this.yaw = wrap(yaw);
    this.bias = 0;
    this.vb = [0, 0];
    this.vVar = 0.01;
    this.P.fill(0);
    this.anchorSigma = sigmaXY;
    this.P[0] = 1e-4;
    this.P[5] = 1e-4;
    this.P[10] = sigmaYaw * sigmaYaw;
    this.P[15] = this.opts.gyroBiasSigma ** 2;
    this.grids.clear();
    this.kfX = NaN;
    this.kfFloor = -1;
    this.inconsistent = 0;
    this.lostLatched = false;
    this.sinceSearch = 0;
    this.searchLevel = 0;
    this.baroZ = NaN;
    this.floorRef.clear();
    for (const k of Object.keys(this.stats) as (keyof EstimatorStats)[]) this.stats[k] = 0;
  }

  get floor(): number {
    return Math.max(0, Math.min(this.opts.floors - 1, Math.floor(this.z / this.opts.floorHeight)));
  }

  /** The map of a floor, built on first use. */
  grid(floor: number): LikelihoodGrid {
    let g = this.grids.get(floor);
    if (!g) {
      const b = this.opts.bounds;
      g = new LikelihoodGrid(b.x0, b.y0, b.x1 - b.x0, b.y1 - b.y0, this.opts.res, 1);
      this.grids.set(floor, g);
    }
    return g;
  }

  /** One tick: predict with the IMU and flow, then correct with the sweep and height sensors. */
  step(frame: LocFrame): PoseEstimate {
    this.predict(frame.imu, frame.flow);
    this.altitude(frame.alt);
    if (frame.scan) this.correct(frame.scan);
    return this.estimate();
  }

  predict(imu: ImuSample, flow: FlowSample | null): void {
    const o = this.opts;
    const dt = imu.dt;
    if (!(dt > 0)) return;
    if (o.mode === 'scan') {
      // No inertial or flow data: the pose only diffuses between sweeps.
      const q = 1.0 * dt;
      this.P[0] += q;
      this.P[5] += q;
      this.P[10] += 0.6 * dt;
      return;
    }
    const w = imu.gyro[2] - this.bias;
    // Body-velocity filter: accelerometer predicts (with the rotating-frame term), flow corrects.
    // The body frame turns by w dt under the velocity, then the accelerometer adds its part.
    const cr = Math.cos(w * dt);
    const sr = Math.sin(w * dt);
    let vx = cr * this.vb[0] + sr * this.vb[1] + imu.accel[0] * dt;
    let vy = -sr * this.vb[0] + cr * this.vb[1] + imu.accel[1] * dt;
    this.vVar += o.accelSigma * o.accelSigma * dt * dt + 1e-4 * dt;
    if (flow) {
      const sig = o.flowSigma + o.flowSigmaRel * Math.hypot(flow.vel[0], flow.vel[1]);
      const r = sig * sig;
      const k = this.vVar / (this.vVar + r);
      vx += k * (flow.vel[0] - vx);
      vy += k * (flow.vel[1] - vy);
      this.vVar *= 1 - k;
    }
    this.vb = [vx, vy];

    // Heading first, then the step along it: the order both simulated vehicles integrate in.
    this.yaw = wrap(this.yaw + w * dt);
    const c = Math.cos(this.yaw);
    const s = Math.sin(this.yaw);
    const dxw = (c * vx - s * vy) * dt;
    const dyw = (s * vx + c * vy) * dt;
    this.x += dxw;
    this.y += dyw;

    // P = F P F^T + Q, F = I with F[0][2] = -dyw, F[1][2] = dxw, F[2][3] = -dt.
    const F = new Float64Array(N * N);
    F[0] = 1;
    F[5] = 1;
    F[10] = 1;
    F[15] = 1;
    F[2] = -dyw;
    F[6] = dxw;
    F[11] = -dt;
    const FP = mul4(F, this.P);
    const P = mul4T(FP, F);
    const dist = Math.hypot(dxw, dyw);
    const qPos = this.vVar * dt * dt + o.odomRel * o.odomRel * dist + 1e-7;
    P[0] += qPos;
    P[5] += qPos;
    P[10] += o.gyroSigma * o.gyroSigma * dt * dt;
    P[15] += o.gyroBiasRW * o.gyroBiasRW * dt;
    this.P.set(P);
  }

  /**
   * Height: the smoothed barometer, replaced by the range finder whenever the
   * two agree (not over furniture, not over a shaft). Each floor's surface
   * height is learned the first time the range finder sees it, so the slab
   * thickness never has to be known.
   */
  altitude(alt: AltSample): void {
    if (!Number.isFinite(this.baroZ)) {
      this.baroZ = this.z;
      this.baroBias = alt.baro - this.z;
    } else this.baroZ += 0.1 * (alt.baro - this.baroBias - this.baroZ);
    let z = this.baroZ;
    const H = this.opts.floorHeight;
    const f = Math.max(0, Math.min(this.opts.floors - 1, Math.floor(z / H)));
    if (alt.down !== null) {
      let ref = this.floorRef.get(f);
      if (ref === undefined && Math.abs(z - f * H - alt.down) < 0.35) {
        ref = z - alt.down;
        this.floorRef.set(f, ref);
      }
      if (ref !== undefined) {
        const zr = ref + alt.down;
        if (Math.abs(zr - z) < 0.3) {
          z = zr;
          // The range finder pins the baro drift while it agrees.
          this.baroBias += 0.02 * (alt.baro - zr - this.baroBias);
        }
      }
    }
    this.z = z;
  }

  correct(scan: ScanSample): void {
    const o = this.opts;
    this.stats.sweeps += 1;
    if (o.mode === 'odometry') return;
    // Climbing a shaft, the ring passes through the slab: at that height it sees the slab's edge
    // around the hole, not the floor's walls, and a map founded on it matches nothing above. Neither
    // match nor map until the drone is clear of the slab.
    const slab = Math.round(this.z / o.floorHeight);
    if (slab > 0 && slab < o.floors && Math.abs(this.z - slab * o.floorHeight) < SLAB_BAND_M) return;
    const floor = this.floor;
    const grid = this.grid(floor);
    grid.refresh();
    const pts = endpoints(scan.ranges, scan.maxRange);
    if (o.manhattan) this.manhattanFix(scan);
    if (grid.occupiedCount === 0) {
      // First sweep on this floor: it founds the map.
      this.keyframe(grid, floor, scan);
      return;
    }

    // Prior information over the pose block.
    const Ppose = block3(this.P);
    const priorInfo = new Float64Array(9);
    if (!invert3(Ppose, priorInfo)) priorInfo.fill(0);
    const m = matchScan(grid, pts, this.x, this.y, this.yaw, priorInfo, o.match);

    // Returns that land in space this map has seen as free contradict it.
    let conflicts = 0;
    {
      const c = Math.cos(m.yaw);
      const s = Math.sin(m.yaw);
      for (let k = 0; k < pts.n; k++) {
        const px = m.x + c * pts.bx[k]! - s * pts.by[k]!;
        const py = m.y + s * pts.bx[k]! + c * pts.by[k]!;
        const cx = Math.floor((px - grid.ox) / grid.res);
        const cy = Math.floor((py - grid.oy) / grid.res);
        if (grid.deepFree(cx, cy) && grid.dist[cy * grid.w + cx]! > 0.3) conflicts += 1;
      }
    }
    const conflictShare = pts.n ? conflicts / pts.n : 0;
    const inlierShare = m.used ? m.inliers / m.used : 0;
    this.stats.lastInlierShare = inlierShare;
    this.stats.lastConflictShare = conflictShare;
    const good = m.used >= o.minUsed && inlierShare >= o.minInlierShare && conflictShare < 0.15;

    if (good) {
      this.stats.matched += 1;
      this.applyMatch(m.x, m.y, m.yaw, m.info);
      this.inconsistent = 0;
      this.searchLevel = 0;
      this.lostLatched = false;
    } else {
      this.stats.rejected += 1;
      if (conflictShare >= 0.15 || (m.used >= o.minUsed && inlierShare < 0.3)) {
        this.inconsistent += 1;
        this.stats.conflicts += 1;
      }
    }

    this.sinceSearch += 1;
    if (this.inconsistent >= o.lostAfter / 2 && this.sinceSearch >= 5) {
      this.sinceSearch = 0;
      this.recover(grid, pts);
    }

    // Map: from a sweep that matched (along a bare corridor the unconstrained direction can be
    // uncertain without hurting a map that looks the same along it), or from a sweep into new
    // space while odometry is still good. Never from a sweep that contradicts the map.
    if (this.inconsistent === 0 && ((good && conflictShare < 0.05) || (conflictShare < 0.05 && this.sigmaXY() < o.degradedSigma))) {
      this.keyframe(grid, floor, scan);
    }
  }

  /** Re-acquire after sweeps keep contradicting the map: search a window that grows with the uncertainty. */
  private recover(grid: LikelihoodGrid, pts: ReturnType<typeof endpoints>): void {
    const o = this.opts;
    // Each failed attempt widens the window: 0.6 m / 11°, 1.2 m / 23°, 2 m / 34°.
    const level = Math.min(2, this.searchLevel);
    this.searchLevel += 1;
    const half = Math.min(2, Math.max([0.6, 1.2, 2][level]!, 3 * this.sigmaXY()));
    const halfYaw = Math.min(0.6, Math.max([0.2, 0.4, 0.6][level]!, 3 * Math.sqrt(this.P[10]!)));
    const best = searchScan(grid, pts, this.x, this.y, this.yaw, half, halfYaw, o.match);
    if (best.score < 0.5) return;
    const m = matchScan(grid, pts, best.x, best.y, best.yaw, null, o.match);
    if (m.used < o.minUsed || m.inliers / m.used < 0.7) return;
    this.stats.recoveries += 1;
    this.x = m.x;
    this.y = m.y;
    this.yaw = wrap(m.yaw);
    // The scan alone now sets the pose; keep a floor on the covariance for directions it can't see.
    const cov = new Float64Array(9);
    const info = new Float64Array(m.info);
    info[0] += 1 / 0.25;
    info[4] += 1 / 0.25;
    info[8] += 1 / 0.04;
    invert3(info, cov);
    const bb = this.P[15]!;
    this.P.fill(0);
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) this.P[r * N + c] = cov[r * 3 + c]!;
    this.P[15] = bb;
    this.inconsistent = 0;
    this.searchLevel = 0;
    this.lostLatched = false;
  }

  /**
   * Manhattan heading fix. Neighbouring returns less than 0.4 m apart form
   * short wall segments; their directions, folded modulo 90° (angle x 4 on
   * the circle), agree on the building's axes when the sweep sees walls. The
   * spread of the fold sets the fix's weight, so a sweep of clutter or a
   * rotated shelf barely moves the heading.
   */
  private manhattanFix(scan: ScanSample): void {
    const r = scan.ranges;
    const n = r.length;
    const max = scan.maxRange - 1e-3;
    let C = 0;
    let S = 0;
    let W = 0;
    let segs = 0;
    for (let i = 0; i < n; i++) {
      const j = (i + 1) % n;
      const ri = r[i]!;
      const rj = r[j]!;
      if (!(ri < max) || !(rj < max) || ri < 0.2 || rj < 0.2) continue;
      const ai = this.yaw + (i / n) * Math.PI * 2;
      const aj = this.yaw + (j / n) * Math.PI * 2;
      const dx = Math.cos(aj) * rj - Math.cos(ai) * ri;
      const dy = Math.sin(aj) * rj - Math.sin(ai) * ri;
      const len = Math.hypot(dx, dy);
      if (len < 0.05 || len > 0.4) continue;
      // Beams at grazing incidence give noisy directions: weight by how square the beam hits.
      const phi = Math.atan2(dy, dx);
      const inc = Math.abs(Math.sin(phi - (ai + aj) / 2));
      const w = len * inc;
      C += w * Math.cos(4 * phi);
      S += w * Math.sin(4 * phi);
      W += w;
      segs += 1;
    }
    if (segs < 10 || W <= 0) return;
    const R = Math.hypot(C, S) / W;
    if (R < 0.6) return;
    const off = Math.atan2(S, C) / 4;
    if (Math.abs(off) > 0.2) return;
    // Scalar EKF update on yaw: measured yaw = yaw - off.
    const P = this.P;
    const sig = this.opts.manhattanSigma / R;
    const s = P[10]! + sig * sig;
    const innov = -off;
    if ((innov * innov) / s > 16) return;
    const K = [P[2]! / s, P[6]! / s, P[10]! / s, P[14]! / s];
    this.x += K[0]! * innov;
    this.y += K[1]! * innov;
    this.yaw = wrap(this.yaw + K[2]! * innov);
    this.bias += K[3]! * innov;
    const row = [P[8]!, P[9]!, P[10]!, P[11]!];
    for (let a = 0; a < N; a++) for (let b = 0; b < N; b++) P[a * N + b] -= K[a]! * row[b]!;
    this.stats.headingFixes += 1;
  }

  /**
   * Fold the match into the filter. The match already solved prior + scan
   * for the pose block; the bias follows through its correlation with yaw.
   * Along directions the sweep can't see (a bare corridor's axis) the
   * correction and the information are dropped: tiny tilts in the map's wall
   * normals would otherwise drag the pose along the wall a little every sweep.
   */
  private applyMatch(mx: number, my: number, myaw: number, scanInfo: Float64Array): void {
    const P = this.P;
    const vals = new Float64Array(3);
    const vecs = new Float64Array(9);
    eigSym3(scanInfo, vals, vecs);
    const info = new Float64Array(9);
    const proj = new Float64Array(9);
    for (let e = 0; e < 3; e++) {
      if (!(vals[e]! >= this.opts.degenerateInfo)) continue;
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
          const vv = vecs[r * 3 + e]! * vecs[c * 3 + e]!;
          info[r * 3 + c] += vals[e]! * vv;
          proj[r * 3 + c] += vv;
        }
      }
    }
    {
      const d0 = mx - this.x;
      const d1 = my - this.y;
      const d2 = wrap(myaw - this.yaw);
      mx = this.x + proj[0]! * d0 + proj[1]! * d1 + proj[2]! * d2;
      my = this.y + proj[3]! * d0 + proj[4]! * d1 + proj[5]! * d2;
      myaw = this.yaw + proj[6]! * d0 + proj[7]! * d1 + proj[8]! * d2;
    }
    const Pxx = block3(P);
    const Pinv = new Float64Array(9);
    if (!invert3(Pxx, Pinv)) return;
    const post = new Float64Array(9);
    for (let k = 0; k < 9; k++) post[k] = Pinv[k]! + info[k]!;
    const Pnew = new Float64Array(9);
    if (!invert3(post, Pnew)) return;
    // L = P_b,pose * Pxx^-1 (1x3).
    const pb = [P[12]!, P[13]!, P[14]!];
    const L = [0, 1, 2].map((c) => pb[0]! * Pinv[c]! + pb[1]! * Pinv[3 + c]! + pb[2]! * Pinv[6 + c]!);
    const d = [mx - this.x, my - this.y, wrap(myaw - this.yaw)];
    this.bias += L[0]! * d[0]! + L[1]! * d[1]! + L[2]! * d[2]!;
    this.x = mx;
    this.y = my;
    this.yaw = wrap(myaw);
    // Covariance: pose block from the information sum, bias rows by conditioning.
    const LPb = L[0]! * pb[0]! + L[1]! * pb[1]! + L[2]! * pb[2]!;
    const pbNew = [0, 1, 2].map((c) => L[0]! * Pnew[c]! + L[1]! * Pnew[3 + c]! + L[2]! * Pnew[6 + c]!);
    const LPL = pbNew[0]! * L[0]! + pbNew[1]! * L[1]! + pbNew[2]! * L[2]!;
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) P[r * N + c] = Pnew[r * 3 + c]!;
    for (let c = 0; c < 3; c++) {
      P[12 + c] = pbNew[c]!;
      P[c * N + 3] = pbNew[c]!;
    }
    P[15] = Math.max(1e-10, P[15]! - LPb + LPL);
  }

  private keyframe(grid: LikelihoodGrid, floor: number, scan: ScanSample): void {
    const o = this.opts;
    const moved =
      floor !== this.kfFloor ||
      !Number.isFinite(this.kfX) ||
      Math.hypot(this.x - this.kfX, this.y - this.kfY) >= o.keyframeDist ||
      Math.abs(wrap(this.yaw - this.kfYaw)) >= o.keyframeYaw;
    if (!moved) return;
    grid.integrate(this.x, this.y, this.yaw, scan.ranges, scan.maxRange);
    this.kfX = this.x;
    this.kfY = this.y;
    this.kfYaw = this.yaw;
    this.kfFloor = floor;
    this.stats.keyframes += 1;
  }

  sigmaXY(): number {
    const a = this.P[0]!;
    const b = this.P[1]!;
    const d = this.P[5]!;
    const tr = (a + d) / 2;
    const disc = Math.sqrt(Math.max(0, ((a - d) / 2) ** 2 + b * b));
    return Math.sqrt(Math.max(0, tr + disc));
  }

  /** 1-sigma error against the building frame: relative to the map, plus where the map sits. */
  sigmaAbs(): number {
    return Math.hypot(this.sigmaXY(), this.anchorSigma);
  }

  health(): PoseHealth {
    const o = this.opts;
    const sigmaM = this.sigmaAbs();
    const sigmaYawRad = Math.sqrt(Math.max(0, this.P[10]!));
    let state: TrackState = 'tracking';
    if (sigmaM > o.lostSigma || this.inconsistent >= o.lostAfter) this.lostLatched = true;
    if (this.lostLatched) state = 'lost';
    else if (sigmaM > o.degradedSigma || this.inconsistent >= 3) state = 'degraded';
    return { ok: state !== 'lost', sigmaM, sigmaMapM: this.sigmaXY(), sigmaYawRad, state };
  }

  estimate(): PoseEstimate {
    const c = Math.cos(this.yaw);
    const s = Math.sin(this.yaw);
    const [vx, vy] = this.vb;
    return {
      pos: [this.x, this.y, this.z],
      yaw: this.yaw,
      vel: [c * vx - s * vy, s * vx + c * vy, 0],
      cov: block3(this.P),
      health: this.health(),
    };
  }
}

function block3(P: Float64Array): Float64Array {
  const out = new Float64Array(9);
  for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) out[r * 3 + c] = P[r * N + c]!;
  return out;
}

function mul4(a: Float64Array, b: Float64Array): Float64Array {
  const out = new Float64Array(N * N);
  for (let r = 0; r < N; r++) {
    for (let c = 0; c < N; c++) {
      let sum = 0;
      for (let k = 0; k < N; k++) sum += a[r * N + k]! * b[k * N + c]!;
      out[r * N + c] = sum;
    }
  }
  return out;
}

/** a * b^T. */
function mul4T(a: Float64Array, b: Float64Array): Float64Array {
  const out = new Float64Array(N * N);
  for (let r = 0; r < N; r++) {
    for (let c = 0; c < N; c++) {
      let sum = 0;
      for (let k = 0; k < N; k++) sum += a[r * N + k]! * b[c * N + k]!;
      out[r * N + c] = sum;
    }
  }
  return out;
}

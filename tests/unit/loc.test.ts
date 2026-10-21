import { describe, expect, it } from 'vitest';
import { defaultEstimatorOptions, PoseEstimator } from '../../src/loc/estimator';
import { LikelihoodGrid } from '../../src/loc/grid';
import { endpoints, matchScan } from '../../src/loc/match';
import { LocScorer } from '../../src/loc/metrics';
import { replay } from '../../src/loc/replay';
import { LocSensors, locNoise, type LocNoise } from '../../src/loc/sensors';
import type { PoseEstimate } from '../../src/loc/types';
import { lap, makeLog, SegmentWorld, truthOf } from '../helpers/segmentWorld';

const ROOM = { x0: -1, y0: -1, x1: 9, y1: 7 };

/** An L-shaped room: 8 x 6 m with a 3 x 2 m notch, plus a pillar, so no direction is unconstrained. */
function room(): SegmentWorld {
  return new SegmentWorld()
    .wall(0, 0, 8, 0)
    .wall(8, 0, 8, 4)
    .wall(8, 4, 5, 4)
    .wall(5, 4, 5, 6)
    .wall(5, 6, 0, 6)
    .wall(0, 6, 0, 0)
    .box(2.5, 2.5, 3.0, 3.0);
}

function quiet(): LocNoise {
  return {
    ...locNoise(),
    gyroSigma: 0,
    gyroBias: 0,
    gyroBiasRW: 0,
    accelSigma: 0,
    accelBias: 0,
    flowSigma: 0,
    flowSigmaRel: 0,
    flowScale: 0,
    flowDropout: 0,
    lidarSigma: 0,
    lidarSigmaRel: 0,
    lidarDropout: 0,
    glassDropout: 0,
    altSigma: 0,
    baroSigma: 0,
    baroRW: 0,
  };
}

describe('localization sensors', () => {
  it('reports the truth when every sigma is zero', () => {
    const s = new LocSensors(room(), 1, quiet());
    const f = s.read({ pos: [2, 1, 1], yaw: 0, vel: [0.5, 0.25, 0], accel: [0, 0, 0], yawRate: 0.1, dt: 0.05 });
    expect(f.imu.gyro[2]).toBeCloseTo(0.1, 12);
    expect(f.flow!.vel[0]).toBeCloseTo(0.5, 12);
    expect(f.flow!.vel[1]).toBeCloseTo(0.25, 12);
    // Beam 0 looks along +x from x = 2 to the wall at x = 8.
    expect(f.scan!.ranges[0]).toBeCloseTo(6, 9);
    expect(f.alt.down).toBeCloseTo(1, 12);
  });

  it('loses most flow samples in the dark and sweeps the lidar at 10 Hz', () => {
    const s = new LocSensors(room(), 2, locNoise(true));
    let flows = 0;
    let scans = 0;
    for (let i = 0; i < 2000; i++) {
      const f = s.read({ pos: [2, 1, 1], yaw: 0, vel: [0, 0, 0], accel: [0, 0, 0], yawRate: 0, dt: 0.05 });
      if (f.flow) flows += 1;
      if (f.scan) scans += 1;
    }
    expect(flows / 2000).toBeGreaterThan(0.33);
    expect(flows / 2000).toBeLessThan(0.47);
    expect(scans).toBe(1000);
  });
});

describe('likelihood grid', () => {
  it('matches a brute-force distance transform', () => {
    const g = new LikelihoodGrid(0, 0, 3, 2, 0.1, 1);
    // Fold in a few returns from the middle.
    g.integrate(1.5, 1.0, 0, [1.2, 0.7, 0.9, 1.4, 0.8], 8);
    g.refresh();
    const occ: number[] = [];
    for (let k = 0; k < g.w * g.h; k++) if (g.isOccupied(k)) occ.push(k);
    expect(occ.length).toBeGreaterThan(0);
    for (let k = 0; k < g.w * g.h; k += 7) {
      const x = k % g.w;
      const y = Math.floor(k / g.w);
      let best = Infinity;
      for (const o of occ) best = Math.min(best, Math.hypot((o % g.w) - x, Math.floor(o / g.w) - y) * g.res);
      expect(g.dist[k]).toBeCloseTo(Math.min(best, 1), 5);
      if (best < 1) expect(occ).toContain(g.near[k]);
    }
  });

  it('measures point-to-wall distance at the returns, not at cell centres', () => {
    const g = new LikelihoodGrid(-1, -1, 10, 8, 0.2, 1);
    const s = new LocSensors(room(), 3, quiet());
    for (const [x, y] of [
      [6, 1],
      [6.5, 2],
      [7, 1.5],
    ]) {
      const f = s.read({ pos: [x!, y!, 1], yaw: 0, vel: [0, 0, 0], accel: [0, 0, 0], yawRate: 0, dt: 0.05 });
      if (f.scan) g.integrate(x!, y!, 0, f.scan.ranges, 8);
    }
    g.refresh();
    const out = new Float64Array(4);
    // 7 cm in front of the x = 8 wall, between two returns.
    g.residual(7.93, 1.37, out);
    expect(Math.abs(out[0]!)).toBeCloseTo(0.07, 2);
    expect(Math.abs(out[1]!)).toBeCloseTo(1, 3);
  });
});

describe('scan matcher', () => {
  it('recovers a 20 cm, 3° offset against a map of the same room', () => {
    const s = new LocSensors(room(), 4, quiet());
    const f = s.read({ pos: [3.7, 1.6, 1], yaw: 0.4, vel: [0, 0, 0], accel: [0, 0, 0], yawRate: 0, dt: 0.05 });
    const g = new LikelihoodGrid(ROOM.x0, ROOM.y0, 10, 8, 0.2, 1);
    g.integrate(3.7, 1.6, 0.4, f.scan!.ranges, 8);
    g.refresh();
    const m = matchScan(g, endpoints(f.scan!.ranges, 8), 3.85, 1.47, 0.45, null);
    expect(Math.hypot(m.x - 3.7, m.y - 1.6)).toBeLessThan(0.02);
    expect(Math.abs(m.yaw - 0.4)).toBeLessThan(0.005);
    expect(m.inliers / m.used).toBeGreaterThan(0.9);
  });
});

function run(world: SegmentWorld, points: [number, number, number][], noise: LocNoise, opts: Parameters<typeof replay>[6] = {}) {
  const ests: PoseEstimate[] = [];
  const r = replay(makeLog(points), world, ROOM, 1, 3, 11, { noise, ...opts, onTick: (_t, e) => ests.push(e) });
  return { ...r, ests };
}

describe('pose estimator', () => {
  const path = lap(1, 1, 7, 3.5, 1, 3);

  it('tracks a noise-free lap within 3 cm (an eighth of a map cell)', () => {
    const r = run(room(), path, quiet(), { initSigmaXY: 0, initSigmaYaw: 0 });
    expect(r.metrics.ateMax).toBeLessThan(0.03);
    expect(r.stats.matched).toBeGreaterThan(0.95 * r.stats.sweeps);
  });

  it('keeps realistic sensor noise bounded where dead reckoning drifts', () => {
    const noise = { ...locNoise(), gyroBias: 0.02 };
    const fused = run(room(), path, noise);
    const odo = run(room(), path, noise, { estimator: { mode: 'odometry' } });
    expect(fused.metrics.ateMapRmse).toBeLessThan(0.1);
    expect(fused.metrics.yawRmseDeg).toBeLessThan(1);
    expect(odo.metrics.ateMapRmse).toBeGreaterThan(3 * fused.metrics.ateMapRmse);
    expect(odo.metrics.yawRmseDeg).toBeGreaterThan(5);
    // Its own covariance is honest about the error most of the time.
    expect(fused.metrics.within95).toBeGreaterThan(0.8);
  });

  it('removes an entry heading error with Manhattan fixes, and cannot without them', () => {
    const still: [number, number, number][] = Array.from({ length: 100 }, () => [3.5, 1.5, 0.2]);
    const on = new PoseEstimator(defaultEstimatorOptions(ROOM));
    const off = new PoseEstimator({ ...defaultEstimatorOptions(ROOM), manhattan: false });
    const sOn = new LocSensors(room(), 5, quiet());
    const sOff = new LocSensors(room(), 5, quiet());
    on.reset([3.5, 1.5, 1], 0.2 + 0.05, 0.05, 0.05);
    off.reset([3.5, 1.5, 1], 0.2 + 0.05, 0.05, 0.05);
    for (let i = 0; i < still.length; i++) {
      on.step(sOn.read(truthOf(still, i)));
      off.step(sOff.read(truthOf(still, i)));
    }
    expect(Math.abs(on.yaw - 0.2)).toBeLessThan(0.005);
    expect(Math.abs(off.yaw - 0.2)).toBeGreaterThan(0.04);
  });

  it('grows its uncertainty along a featureless corridor and not across it', () => {
    // 40 m corridor, 2 m wide, walls only: the scan pins y and yaw but not x.
    const corridor = new SegmentWorld().wall(-5, 0, 45, 0).wall(-5, 2, 45, 2);
    const pts: [number, number, number][] = [];
    for (let i = 0; i <= 600; i++) pts.push([i * 0.05, 1, 0]);
    const ests: PoseEstimate[] = [];
    replay(makeLog(pts), corridor, { x0: -6, y0: -2, x1: 46, y1: 4 }, 1, 3, 12, {
      noise: locNoise(),
      onTick: (_t, e) => ests.push(e),
    });
    const last = ests[ests.length - 1]!;
    expect(Math.sqrt(last.cov[0]!)).toBeGreaterThan(5 * Math.sqrt(last.cov[4]!));
    expect(Math.abs(last.pos[1] - 1)).toBeLessThan(0.05);
  });

  it('notices a kidnap, re-acquires, and reports lost meanwhile', () => {
    const r = run(room(), path, locNoise(), { kidnap: { tick: 300, offset: [0.6, -0.5, 0], yaw: 0.2 } });
    const after = r.ests.slice(300);
    const flagged = after.slice(0, 60).some((e) => e.health.state !== 'tracking');
    expect(flagged).toBe(true);
    expect(r.stats.recoveries).toBeGreaterThan(0);
    const back = after.findIndex((e, i) => i > 0 && Math.hypot(e.pos[0] - path[300 + i]![0], e.pos[1] - path[300 + i]![1]) < 0.15);
    expect(back).toBeGreaterThan(0);
    expect(back).toBeLessThan(80);
    expect(r.ests[r.ests.length - 1]!.health.ok).toBe(true);
  });

  it('is deterministic for a seed', () => {
    const a = run(room(), path.slice(0, 400), locNoise());
    const b = run(room(), path.slice(0, 400), locNoise());
    expect(a.metrics).toEqual(b.metrics);
  });
});

describe('localization metrics', () => {
  it('scores a constant offset as absolute error but no drift', () => {
    const s = new LocScorer();
    const cov = new Float64Array([0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.001]);
    for (let i = 0; i < 100; i++) {
      s.add(
        { pos: [i * 0.1, 0, 1], yaw: 0 },
        { pos: [i * 0.1 + 0.3, 0.3, 1], yaw: 0, vel: [0, 0, 0], cov, health: { ok: true, sigmaM: 0.1, sigmaMapM: 0.1, sigmaYawRad: 0.03, state: 'tracking' } },
      );
    }
    const m = s.result();
    expect(m.ateRmse).toBeCloseTo(Math.SQRT2 * 0.3, 6);
    expect(m.ateMapRmse).toBeCloseTo(0, 6);
    expect(m.pathM).toBeCloseTo(9.9, 4);
    expect(m.bigErrTicks).toBe(0);
    expect(m.within95).toBe(1);
    expect(m.within95Abs).toBe(0);
  });
});

import { readFileSync } from 'node:fs';
import { gunzipSync } from 'node:zlib';
import { describe, expect, it } from 'vitest';
import { odomFrame, parseFlaser, parseRelations, relationError, ringScan, type Relation } from '../../src/loc/carmen';
import { defaultEstimatorOptions, PoseEstimator } from '../../src/loc/estimator';
import { boxRange, syntheticLog } from '../helpers/carmenLog';


describe('CARMEN logs and the relation-based error', () => {
  it('parses FLASER lines and maps the 180° sweep into the estimator ring', () => {
    const log = parseFlaser(syntheticLog());
    expect(log.beams).toBe(180);
    expect(log.n).toBe(180);
    expect(log.t[1]! - log.t[0]!).toBeCloseTo(0.1, 6);
    expect(log.dt[1]).toBeCloseTo(0.1, 6);
    const ring = ringScan(log, 0, 8);
    expect(ring.ranges.length).toBe(360);
    // Beam 90 looks straight ahead (+x from (2, 1.5)): 8 m to the far wall, at the max range: a miss.
    expect(ring.ranges[0]).toBe(8);
    // Beam 179 looks 89° left: ring slot 89. Beam 0 looks right: slot 270. Behind is a miss.
    expect(ring.ranges[89]).toBeCloseTo(boxRange(2, 1.5, (89 * Math.PI) / 180, 10, 6), 3);
    expect(ring.ranges[270]).toBeCloseTo(1.5, 3);
    expect(ring.ranges[180]).toBe(8);
  });

  it('turns wheel odometry into gyro, flow and accelerometer samples', () => {
    const log = parseFlaser(syntheticLog());
    const f = odomFrame(log, 5, 8);
    expect(f.flow!.vel[0]).toBeCloseTo(1, 6);
    expect(f.flow!.vel[1]).toBeCloseTo(0, 6);
    expect(f.imu.gyro[2]).toBeCloseTo(0, 6);
    expect(f.imu.accel[0]).toBeCloseTo(0, 6);
  });

  it('scores a perfect trajectory zero and a known error exactly', () => {
    const t = Float64Array.from([1, 2, 3]);
    const pose = Float64Array.from([0, 0, 0, 1, 0, Math.PI / 2, 1, 1, Math.PI / 2]);
    const rels: Relation[] = [
      { t1: 1, t2: 2, dx: 1, dy: 0, dyaw: Math.PI / 2 },
      // In the frame of pose 2 (facing +y), pose 3 is 1 m ahead.
      { t1: 2, t2: 3, dx: 1, dy: 0, dyaw: 0 },
    ];
    const ok = relationError(t, pose, rels);
    expect(ok.n).toBe(2);
    expect(ok.transAbs.mean).toBeCloseTo(0, 9);
    expect(ok.rotAbs.mean).toBeCloseTo(0, 9);
    // Shift pose 3 by 10 cm and turn it 2°: only the second relation sees it.
    const bad = Float64Array.from(pose);
    bad[6] = 1.1;
    bad[8] = Math.PI / 2 + (2 * Math.PI) / 180;
    const e = relationError(t, bad, rels);
    expect(e.perTrans[0]).toBeCloseTo(0, 9);
    expect(e.perTrans[1]).toBeCloseTo(0.1, 9);
    expect(e.perRot[1]).toBeCloseTo(2, 9);
    expect(e.transAbs.mean).toBeCloseTo(0.05, 9);
    expect(e.transSq.mean).toBeCloseTo(0.005, 9);
    expect(relationError(t, pose, [{ t1: 1, t2: 9, dx: 0, dy: 0, dyaw: 0 }]).missing).toBe(1);
  });

  it('tracks the synthetic loop to centimetres with the shipped estimator', () => {
    const text = syntheticLog();
    const log = parseFlaser(text);
    const est = new PoseEstimator(defaultEstimatorOptions({ x0: -10, y0: -10, x1: 20, y1: 16 }));
    est.reset([log.odom[0]!, log.odom[1]!, 0], log.odom[2]!, 0, 0.01);
    est.correct(ringScan(log, 0, 8));
    const pose = new Float64Array(log.n * 3);
    pose.set([est.x, est.y, est.yaw]);
    for (let i = 1; i < log.n; i++) {
      const e = est.step(odomFrame(log, i, 8));
      pose.set([e.pos[0], e.pos[1], e.yaw], i * 3);
    }
    // Truth relation: the start and the end of the loop are the same place.
    const rel = relationError(log.t, pose, [{ t1: log.t[0]!, t2: log.t[log.n - 1]!, dx: 0, dy: 0.1, dyaw: -Math.PI / 2 }]);
    expect(rel.n).toBe(1);
    expect(rel.transAbs.mean).toBeLessThan(0.05);
  });

  it('reads the vendored Intel Research Lab log and relations', () => {
    const log = parseFlaser(gunzipSync(readFileSync('data/intel-lab/intel.flaser.gz')).toString('utf8'));
    const rels = parseRelations(readFileSync('data/intel-lab/intel.relations', 'utf8'));
    expect(log.n).toBe(13631);
    expect(log.beams).toBe(180);
    expect(rels.length).toBe(2984);
    // Every relation names two sweeps of the log; raw odometry scores (the floor any estimator must beat on long relations).
    const e = relationError(log.t, log.odom, rels);
    expect(e.missing).toBe(0);
    expect(e.n).toBe(2984);
  });
});

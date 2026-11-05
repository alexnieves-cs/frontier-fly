import { describe, expect, it } from 'vitest';
import { parseFlaser, ringScan } from '../../src/loc/carmen';
import { between, compose, LoopCloser, PoseGraph, type Pose2 } from '../../src/loc/graph';
import { syntheticLog } from '../helpers/carmenLog';

const ID = new Float64Array([400, 0, 0, 0, 400, 0, 0, 0, 3000]);

function circle(n: number, r: number): Pose2[] {
  return Array.from({ length: n }, (_, k) => {
    const a = (2 * Math.PI * k) / n;
    return [r * Math.cos(a), r * Math.sin(a), Math.atan2(Math.cos(a), -Math.sin(a))] as Pose2;
  });
}

describe('pose graph', () => {
  it('composes and inverts SE(2) poses', () => {
    const a: Pose2 = [1, 2, 0.7];
    const b: Pose2 = [-0.4, 3, -2.1];
    const d = between(a, b);
    const back = compose(a, d);
    expect(back[0]).toBeCloseTo(b[0], 12);
    expect(back[1]).toBeCloseTo(b[1], 12);
    expect(back[2]).toBeCloseTo(b[2], 12);
  });

  it('recovers exact poses from a perturbed start when every edge is consistent', () => {
    const truth = circle(30, 5);
    const g = new PoseGraph();
    truth.forEach((p, k) => g.addNode(k === 0 ? p : [p[0] + 0.3 * Math.sin(k), p[1] - 0.2 * Math.cos(3 * k), p[2] + 0.05 * Math.sin(2 * k)]));
    for (let k = 1; k < truth.length; k++) g.addEdge(k - 1, k, between(truth[k - 1]!, truth[k]!), ID, 'seq');
    // Chords across the circle: the elimination order has real fill to handle.
    for (let k = 0; k + 11 < truth.length; k += 4) g.addEdge(k, k + 11, between(truth[k]!, truth[k + 11]!), ID, 'loop');
    g.optimize(20);
    for (let k = 0; k < truth.length; k++) {
      const p = g.pose(k);
      expect(Math.hypot(p[0] - truth[k]![0], p[1] - truth[k]![1])).toBeLessThan(1e-6);
    }
    expect(g.chi2()).toBeLessThan(1e-9);
  });

  it('spreads a loop closure over the drifted chain, and a Cauchy kernel weakens a wrong closure', () => {
    const truth = circle(60, 6);
    const g = new PoseGraph(1);
    // Odometry turns 0.1° too far at every step: the chain ends most of a metre from where it began.
    let p = truth[0]!;
    g.addNode(p);
    for (let k = 1; k < truth.length; k++) {
      const z = between(truth[k - 1]!, truth[k]!);
      z[2] += (0.1 * Math.PI) / 180;
      p = compose(p, z);
      g.addNode(p);
      g.addEdge(k - 1, k, z, ID, 'seq');
    }
    const drift = Math.hypot(g.pose(59)[0] - truth[59]![0], g.pose(59)[1] - truth[59]![1]);
    expect(drift).toBeGreaterThan(0.5);
    // The true closure, plus a wrong one 3 m off between two other nodes.
    g.addEdge(0, 59, between(truth[0]!, truth[59]!), ID, 'loop');
    const wrong = between(truth[10]!, truth[40]!);
    wrong[0] += 3;
    const bad = g.addEdge(10, 40, wrong, ID, 'loop');
    g.optimize(30);
    const end = Math.hypot(g.pose(59)[0] - truth[59]![0], g.pose(59)[1] - truth[59]![1]);
    expect(end).toBeLessThan(0.1 * drift);
    expect(bad.weight).toBeLessThan(0.1);
  });
});

describe('loop closer', () => {
  it('closes the synthetic room loop driven twice on odometry with a heading bias', () => {
    const log = parseFlaser(syntheticLog());
    const lc = new LoopCloser({ loopMinGap: 20 });
    // Drive the loop twice. Odometry turns 0.4° too far per sweep.
    let odo: Pose2 = [log.odom[0]!, log.odom[1]!, log.odom[2]!];
    const truth: Pose2[] = [];
    const odoTrack: Pose2[] = [];
    for (let lap = 0; lap < 2; lap++) {
      for (let i = 0; i < log.n; i++) {
        const t: Pose2 = [log.odom[i * 3]!, log.odom[i * 3 + 1]!, log.odom[i * 3 + 2]!];
        if (truth.length) {
          const z = between(truth[truth.length - 1]!, t);
          z[2] += (0.4 * Math.PI) / 180;
          odo = compose(odo, z);
        }
        truth.push(t);
        odoTrack.push(odo);
        lc.add(odo, ringScan(log, i, 8).ranges);
      }
    }
    const last = truth.length - 1;
    // What a vehicle flying on the graph would see at the end, before the final optimisation.
    const now = lc.current();
    expect(Math.hypot(now[0] - truth[last]![0], now[1] - truth[last]![1])).toBeLessThan(0.1);
    const pose = lc.finish();
    const odoErr = Math.hypot(odoTrack[last]![0] - truth[last]![0], odoTrack[last]![1] - truth[last]![1]);
    const err = Math.hypot(pose[last * 3]! - truth[last]![0], pose[last * 3 + 1]! - truth[last]![1]);
    expect(odoErr).toBeGreaterThan(1);
    expect(err).toBeLessThan(0.05);
    expect(lc.stats.loopAccepted).toBeGreaterThan(0);
  });
});

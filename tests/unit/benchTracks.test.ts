import { beforeAll, describe, expect, it } from 'vitest';
import { BASELINES, watchLayer } from '../../src/bench/baselines';
import { HARD_TRACKS, INTEGRITY_TASKS, OVERWATCH_TASKS, trackRuns, trackTasks } from '../../src/bench/hard';
import { INTEGRITY, IntegrityMeter, MONITORS } from '../../src/bench/integrity';
import { Overwatch, viewOfPerson, walkerRoute } from '../../src/bench/overwatch';
import { benchEpisodeSetup, runBenchEpisode } from '../../src/bench/run';
import { CELL } from '../../src/shared/constants';
import type { BuildingSpec } from '../../src/shared/types';
import { createEnv } from '../../src/sim/env';
import { InformationGainExplorer } from '../../src/sim/explorers/informationGain';
import { WatchExplorer } from '../../src/sim/explorers/watch';
import { initPhysics } from '../../src/sim/world';

beforeAll(async () => {
  await initPhysics();
});

/** Two 4 x 4 m rooms side by side, a wall between them with a doorway at y = 1. */
function twoRooms(): BuildingSpec {
  const w = 10;
  const d = 6;
  const cells = new Uint8Array(w * d).fill(CELL.WALL);
  for (let y = 1; y < 5; y++) for (let x = 1; x < 9; x++) if (x !== 5) cells[y * w + x] = CELL.EMPTY;
  cells[1 * w + 5] = CELL.DOOR;
  return {
    seed: 7,
    cellSize: 1,
    floorHeight: 3,
    floors: [{ w, d, cells, rooms: [{ id: 0, x: 1, z: 1, w: 4, d: 4 }, { id: 1, x: 6, z: 1, w: 3, d: 4 }] }],
    spawn: [2.5, 2.5, 1.2],
    reachableVoxels: 0,
  };
}

describe('FrontierBench Hard tracks', () => {
  it('gives every track its own seeds', () => {
    const all = HARD_TRACKS.flatMap((track) => trackTasks(track).flatMap((_, ti) => [...trackRuns(track, 'dev', ti), ...trackRuns(track, 'test', ti)].map((r) => r.seed)));
    expect(new Set(all).size).toBe(all.length);
  });

  it('walks the person on open floor through the doorway, the same loop every time', () => {
    const spec = twoRooms();
    const a = walkerRoute(spec, 1, spec.spawn);
    expect(a).toEqual(walkerRoute(spec, 1, spec.spawn));
    expect(a.length).toBeGreaterThanOrEqual(2);
    const f = spec.floors[0]!;
    let door = false;
    for (let i = 0; i < a.length; i++) {
      const p = a[i]!;
      const q = a[(i + 1) % a.length]!;
      // Corner points of a 4-connected walk: every leg is along an axis.
      expect(p[0] === q[0] || p[1] === q[1]).toBe(true);
      for (let t = 0; t <= 20; t++) {
        const x = Math.floor(p[0] + (q[0] - p[0]) * (t / 20));
        const y = Math.floor(p[1] + (q[1] - p[1]) * (t / 20));
        expect(f.cells[y * f.w + x]).not.toBe(CELL.WALL);
        if (f.cells[y * f.w + x] === CELL.DOOR) door = true;
      }
    }
    expect(door).toBe(true);
  });

  it('sees the person in front, in range and in the open, and not through a wall', () => {
    const spec = twoRooms();
    const eye: [number, number, number] = [2, 3, 1.2];
    expect(viewOfPerson(spec, [], eye, 0, [4.5, 3, 0]).visible).toBe(true);
    // Behind the wall at x = 5.
    expect(viewOfPerson(spec, [], eye, 0, [7.5, 3.5, 0]).visible).toBe(false);
    // Out of the 90° view.
    expect(viewOfPerson(spec, [], eye, Math.PI, [4.5, 3, 0]).visible).toBe(false);
    // A shelf in between hides them.
    expect(viewOfPerson(spec, [{ min: [3, 2, 0], max: [3.4, 4, 2] }], eye, 0, [4.5, 3, 0]).visible).toBe(false);
  });

  it('scores time in view from the standoff, retention after the find and standoff breaches', () => {
    const spec = twoRooms();
    const w = new Overwatch(spec, 1, spec.spawn);
    w.observe(0, spec.spawn, 0);
    // Pin the person: look at them from fixed poses so the walk does not matter.
    const route = w.route;
    w.at = () => [4.5, 3, 0];
    void route;
    const far: [number, number, number] = [1.5, 3, 1.2];
    const near: [number, number, number] = [3.5, 3, 1.2];
    for (let s = 1; s <= 10; s++) w.observe(s, far, Math.PI); // not looking: 10 steps
    for (let s = 11; s <= 30; s++) w.observe(s, far, 0); // in view from 3 m: 20 steps
    for (let s = 31; s <= 40; s++) w.observe(s, far, Math.PI); // lost for 10 steps
    for (let s = 41; s <= 50; s++) w.observe(s, near, 0); // 1 m away: a breach, counts nothing
    for (let s = 51; s <= 60; s++) w.observe(s, far, 0); // back in view: 10 steps
    const st = w.stats();
    expect(st.score).toBeCloseTo(30 / 60, 9);
    expect(st.found).toBe(true);
    expect(st.findS).toBeCloseTo(11 / 20, 9);
    expect(st.kept).toBeCloseTo(30 / 50, 9);
    expect(st.losses).toBe(1);
    expect(st.reacquireS).toBeCloseTo(20 / 20, 9);
    expect(st.breachS).toBeCloseTo(10 / 20, 9);
    expect(w.seen()).not.toBeNull();
    // Steps a crash cut off count as out of view.
    expect(w.stats(120)).toMatchObject({ score: 30 / 120, kept: 30 / 110 });
  });

  it('gives every explorer pilot the same watch layer on overwatch tasks, and only there', () => {
    const frame = { speedH: 2, maxSteps: 2400, dynamics: 'kinematic' as const };
    const bare = new InformationGainExplorer({ speed: 2 });
    expect(watchLayer(bare, frame)).toBe(bare);
    const wrapped = watchLayer(bare, { ...frame, person: true });
    expect(wrapped).toBeInstanceOf(WatchExplorer);
    expect(watchLayer(wrapped, { ...frame, person: true })).toBe(wrapped);
  });

  it('flies an overwatch task: published explorers and the view-first planner all find and watch the person', async () => {
    const task = OVERWATCH_TASKS[0]!;
    const env = await createEnv();
    const run = { seed: 150001 };
    const house = runBenchEpisode(env, task, run, BASELINES['planner-view']!());
    const published = runBenchEpisode(env, task, run, BASELINES['info-gain']!());
    for (const ep of [house, published]) {
      expect(ep.watch!.found).toBe(true);
      // The person walks far from take-off: nothing is in view at the start.
      expect(ep.watch!.findS).toBeGreaterThan(0);
      expect(ep.watch!.score).toBeGreaterThan(0.4);
    }
    expect(house.collisions).toBe(0);
    // Deterministic.
    expect(runBenchEpisode(env, task, run, BASELINES['info-gain']!()).watch).toEqual(published.watch);
    expect(benchEpisodeSetup(task, run).cfg.maxSteps).toBe(2400);
    env.dispose();
  }, 120_000);

  it('penalises false confidence ten times harder than honest uncertainty', () => {
    const tta = INTEGRITY.ttaS * 20;
    const err = [...Array(100).fill(0.2), ...Array(100).fill(3)];
    const score = (level: (e: number, i: number) => number) => {
      const m = new IntegrityMeter();
      err.forEach((e, i) => m.step(e, level(e, i)));
      return m.stats();
    };
    const always = score(() => 0);
    expect(always.availability).toBe(1);
    expect(always.hmiRate).toBeCloseTo((100 - tta) / 200, 9);
    // No floor: confidently wrong for 30% of the flight costs three flights' worth of availability.
    expect(always.score).toBeCloseTo(1 - 10 * ((100 - tta) / 200), 9);
    const never = score(() => Infinity);
    expect(never).toMatchObject({ availability: 0, hmiRate: 0, score: 0, caught: 1 });
    expect(score(() => 0.1).caught).toBe(0);
    // A flight a crash cut short is unavailable for the rest of its budget: crashing out of a bad pose does not pay.
    const short = new IntegrityMeter();
    for (let i = 0; i < 100; i++) short.step(0.2, 0.5);
    expect(short.stats().score).toBe(1);
    expect(short.stats(400)).toMatchObject({ availability: 0.25, score: 0.25, badShare: 0 });
    // Honest: confident while the pose is good, unsure once it goes bad (a second late, inside the time to alert).
    const honest = score((_, i) => (i < 120 ? 0.5 : Infinity));
    expect(honest.hmiRate).toBe(0);
    expect(honest.score).toBeCloseTo(120 / 200, 9);
    expect(honest.caught).toBeCloseTo(80 / 100, 9);
    // The monitors read only the estimator's health.
    const h = { ok: true, sigmaM: 0.2, sigmaMapM: 0.1, sigmaYawRad: 0.01, state: 'tracking' as const };
    expect(MONITORS.sigma!.make(3)(h)).toBeCloseTo(0.6, 9);
    expect(MONITORS['sigma-health']!.make(3)({ ...h, state: 'degraded' })).toBe(Infinity);
  });

  it('flies an integrity task on the estimate and scores the protection levels against the truth', async () => {
    const task = { ...INTEGRITY_TASKS.find((t) => t.id === 'pl-tof-dark-quad')!, maxSteps: 200 };
    const env = await createEnv();
    const e = runBenchEpisode(env, task, { seed: 160101 }, BASELINES['planner-tracker']!());
    expect(e.drift).toBeDefined();
    expect(e.integrity).toBeDefined();
    // planner-tracker reports no level, so it claims its pose is exact on every step.
    expect(e.integrity!.availability).toBe(1);
    // planner-view reports the estimator's 1σ; the level never changes the flight.
    const view = BASELINES['planner-view']!();
    expect(typeof view.protectionLevel).toBe('function');
    const v = runBenchEpisode(env, task, { seed: 160101 }, view);
    expect(v.integrity!.availability).toBeLessThanOrEqual(1);
    expect(v.drift!.mean).toBeGreaterThan(0);
    env.dispose();
  }, 120_000);
});

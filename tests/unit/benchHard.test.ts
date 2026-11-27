import { beforeAll, describe, expect, it } from 'vitest';
import { BASELINES, explorerPilot, VIEW_QUAD_MARGIN } from '../../src/bench/baselines';
import { DRIFT_HARD_TASKS, DRIFT_TASKS, HARD_SEEDS, HARD_TASKS, hardRuns, LOOP_IN_FLIGHT, RETURN_TASKS, SENSOR_TASKS, TOF_RING, trackRuns } from '../../src/bench/hard';
import { benchEpisodeSetup, runBenchEpisode } from '../../src/bench/run';
import { BENCH_SEEDS } from '../../src/bench/tasks';
import { evalEpisodeSetup } from '../../src/eval/coverage';
import { FootprintCoverage } from '../../src/eval/footprint';
import { createEnv } from '../../src/sim/env';
import { initPhysics } from '../../src/sim/world';
import { QUAD_BRAKING } from '../../src/sim/lidarSafety';
import { SensorPlanner } from '../../src/sim/sensorPlanner';

beforeAll(async () => {
  await initPhysics();
});

describe('FrontierBench Hard', () => {
  it('uses its own seed blocks, apart from each other and from v3', () => {
    const all = HARD_TASKS.flatMap((_, ti) => [...hardRuns('dev', ti), ...hardRuns('test', ti)].map((r) => r.seed));
    expect(new Set(all).size).toBe(all.length);
    for (const s of all) {
      expect(s >= BENCH_SEEDS.dev.base && s < BENCH_SEEDS.test.base + 700).toBe(false);
      expect(s).toBeGreaterThanOrEqual(HARD_SEEDS.dev.base);
    }
  });

  it('flies the 40 m building with the 120 s budget and scores a camera footprint inside the ring', async () => {
    const task = HARD_TASKS.find((t) => t.id === 'large')!;
    const { spec, cfg } = benchEpisodeSetup(task, { seed: 92001 });
    expect(spec.floors[0]!.w).toBe(40);
    expect(cfg.maxSteps).toBe(2400);
    const env = await createEnv();
    const e = runBenchEpisode(env, task, { seed: 92001 }, BASELINES['planner-view']!());
    expect(e.camera).toBeDefined();
    expect(e.camera!.footprint).toBeGreaterThan(0.1);
    // The 90° camera sees a subset of what the 360° ring sees.
    expect(e.camera!.footprint).toBeLessThanOrEqual(e.footprint + 1e-9);
    env.dispose();
  });

  it('the camera footprint only counts rays near the heading', async () => {
    const env = await createEnv();
    const { spec, envSeed, cfg } = evalEpisodeSetup(3, 9101, 'day');
    env.reset(spec, envSeed, cfg);
    const p = env.position;
    const ring = new FootprintCoverage([3]);
    const cam = new FootprintCoverage([3], Math.PI / 2);
    ring.reset(spec);
    cam.reset(spec);
    ring.observe(env.world!, p[0]!, p[1]!, p[2]!);
    cam.observe(env.world!, p[0]!, p[1]!, p[2]!, 0);
    expect(cam.coverage(0)).toBeGreaterThan(0);
    expect(cam.coverage(0)).toBeLessThan(ring.coverage(0));
    env.dispose();
  });

  it('the view-first planner does not mistake mapped walls for unexplored space', async () => {
    // Dev 92003 on the 40 m building: when walls counted as unknown, every wall was an "edge of
    // unknown" goal and the drone spent the flight along the walls of one quarter (32% camera).
    const env = await createEnv();
    const ti = HARD_TASKS.findIndex((t) => t.id === 'large');
    const e = runBenchEpisode(env, HARD_TASKS[ti]!, { seed: 92003 }, BASELINES['planner-view']!());
    expect(e.camera!.footprint).toBeGreaterThan(0.6);
    env.dispose();
  });

  it('aiming the camera at what it has not seen raises the quadrotor camera footprint', async () => {
    const env = await createEnv();
    const ti = HARD_TASKS.findIndex((t) => t.id === 'quad-dark');
    const run = hardRuns('dev', ti)[0]!;
    const facing = explorerPilot('facing travel', (c) => new SensorPlanner({ speed: c.speedH, view: {}, braking: QUAD_BRAKING, tracker: {}, brakingMargin: VIEW_QUAD_MARGIN }));
    const look = runBenchEpisode(env, HARD_TASKS[ti]!, run, BASELINES['planner-view']!());
    const face = runBenchEpisode(env, HARD_TASKS[ti]!, run, facing);
    expect(look.camera!.footprint).toBeGreaterThan(face.camera!.footprint);
    expect(look.crashed).toBe(false);
    env.dispose();
  });

  it('a return task counts the camera only for a drone that came home', async () => {
    const env = await createEnv();
    const ti = RETURN_TASKS.findIndex((t) => t.id === 'return-sprint');
    const run = trackRuns('return', 'dev', ti)[0]!;
    // Every explorer pilot gets the shared return supervisor and comes back in time.
    const back = runBenchEpisode(env, RETURN_TASKS[ti]!, run, BASELINES['planner-view']!());
    expect(back.returned).toBe(true);
    expect(back.camera!.footprint).toBeGreaterThan(0.3);
    expect(back.camera!.seen).toBe(back.camera!.footprint);
    // A pilot that does not know about returning loses the flight.
    const plain = BASELINES['planner-view']!();
    const away = runBenchEpisode(env, RETURN_TASKS[ti]!, run, { name: 'no return', reset: (c) => plain.reset({ ...c, home: false }), act: (o, f) => plain.act(o, f) });
    expect(away.returned).toBe(false);
    expect(away.camera!.footprint).toBe(0);
    // It saw plenty (this 30 s building is nearly all seen either way); none of it counts.
    expect(away.camera!.seen).toBeGreaterThan(0.3);
    env.dispose();
  });

  it('a drift task flies on the pose estimate, never the true pose or the env maps', async () => {
    const env = await createEnv();
    const ti = DRIFT_TASKS.findIndex((t) => t.id === 'drift-dark-quad');
    const run = trackRuns('drift', 'dev', ti)[0]!;
    const inner = BASELINES['planner-view']!();
    let offTruth = 0;
    let envMaps = 0;
    const spy = {
      name: 'spy',
      reset: inner.reset.bind(inner),
      act: (o: Float32Array, f: Parameters<typeof inner.act>[1]) => {
        const s = env.snapshot();
        if (Math.hypot(f.pos[0] - s.pos[0], f.pos[1] - s.pos[1]) > 1e-6) offTruth += 1;
        if (f.seen === env.seenGrid || f.visited === env.visitedGrid) envMaps += 1;
        return inner.act(o, f);
      },
    };
    const ep = runBenchEpisode(env, DRIFT_TASKS[ti]!, run, spy);
    expect(envMaps).toBe(0);
    expect(offTruth).toBe(ep.steps);
    expect(ep.drift!.mean).toBeGreaterThan(0);
    expect(ep.drift!.max).toBeLessThan(1);
    expect(ep.returned).toBe(true);
    env.dispose();
  });

  it('the hard drift track flies every task home on the short-range ToF ring suite, on seeds no other track uses', () => {
    for (const t of DRIFT_HARD_TASKS) {
      expect(t.drift && t.home && t.camera).toBe(true);
      expect(t.driftProfile).toBe(TOF_RING);
    }
    const seeds = (track: Parameters<typeof trackRuns>[0], tasks: readonly unknown[]) => tasks.flatMap((_, i) => [...trackRuns(track, 'dev', i), ...trackRuns(track, 'test', i)].map((r) => r.seed));
    const mine = new Set(seeds('drift-hard', DRIFT_HARD_TASKS));
    for (const [track, tasks] of [['hard', HARD_TASKS], ['return', RETURN_TASKS], ['drift', DRIFT_TASKS]] as const) for (const s of seeds(track, tasks)) expect(mine.has(s)).toBe(false);
  });

  it('with loop closure in flight the pilot flies on the graph, and the estimator alone is reported beside it', async () => {
    const env = await createEnv();
    const ti = DRIFT_HARD_TASKS.findIndex((t) => t.id === 'tof-dark-quad');
    const task = { ...DRIFT_HARD_TASKS[ti]!, driftProfile: { ...TOF_RING, ...LOOP_IN_FLIGHT } };
    const ep = runBenchEpisode(env, task, trackRuns('drift-hard', 'dev', ti)[0]!, BASELINES['planner-view']!());
    expect(ep.drift!.loop!.keyframes).toBeGreaterThan(10);
    expect(ep.drift!.loop!.seqMatched).toBeGreaterThan(0);
    expect(ep.drift!.other!.mean).toBeGreaterThan(0);
    expect(ep.drift!.other!.mean).not.toBe(ep.drift!.mean);
    env.dispose();
  });

  it('a map-backed safety filter keeps a drone whose lidar fails mid-flight off the walls', async () => {
    const env = await createEnv();
    const ti = SENSOR_TASKS.findIndex((t) => t.id === 'failing-quad');
    const task = { ...SENSOR_TASKS[ti]!, maxSteps: 900 };
    // Dev 140314: 120° of lidar and the flow sensor die at 27 s; without the map the blind side hits a wall.
    const run = trackRuns('sensor', 'dev', ti)[13]!;
    expect(run.seed).toBe(140314);
    const make = (mapFill: boolean) => explorerPilot('view', (c) => new SensorPlanner({ speed: c.speedH, view: { look: Math.PI / 2, lookYawRate: 0.8 }, braking: QUAD_BRAKING, tracker: {}, brakingMargin: VIEW_QUAD_MARGIN, mapFill }));
    expect(runBenchEpisode(env, task, run, make(false)).crashed).toBe(true);
    expect(runBenchEpisode(env, task, run, make(true)).crashed).toBe(false);
    env.dispose();
  });
});

// The integrity study (docs/INTEGRITY.md): fly one pilot on the integrity track once, keep each step's
// true pose error and the estimator's own health, and score every monitor on the same flights. A
// protection level does not change how the drone flies, so scoring the monitors on one set of flights
// is exact: the loop below is runBenchEpisode's (bench/run.ts), with the log added.
//
//   npm run bench:integrity -- --split dev                         # pick k on dev
//   npm run bench:integrity -- --split test --out docs/integrity.json
//
import { execSync } from 'node:child_process';
import { writeFileSync } from 'node:fs';
import { BASELINES } from '../../src/bench/baselines';
import { DriftFrames } from '../../src/bench/drift';
import { INTEGRITY_SEEDS, INTEGRITY_TASKS, trackRuns } from '../../src/bench/hard';
import { INTEGRITY, IntegrityMeter, MONITORS, type IntegrityMonitor, type IntegrityStats } from '../../src/bench/integrity';
import { BenchEpisodeRecorder } from '../../src/bench/run';
import type { PoseHealth } from '../../src/loc/types';
import { createEnv } from '../../src/sim/env';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const split = arg('split', 'dev') as 'dev' | 'test';
if (split !== 'dev' && split !== 'test') throw new Error('--split dev|test');
const pilotId = arg('pilot', 'planner-view');
const out = arg('out', '');
// --dump writes every step's error and health (for designing a monitor on dev; never the test split).
const dump = arg('dump', '');
if (dump && split === 'test') throw new Error('--dump is for the dev split');
const dumped: string[] = [];
const KS = [1, 2, 3, 5, 8];
const commit = execSync('git rev-parse --short HEAD').toString().trim();

const variants: Array<{ id: string; label: string; monitor: IntegrityMonitor }> = [
  { id: 'always', label: MONITORS.always!.label, monitor: MONITORS.always!.make(0) },
  { id: 'never', label: MONITORS.never!.label, monitor: MONITORS.never!.make(0) },
  ...KS.map((k) => ({ id: `sigma-${k}`, label: `${k} × the estimator’s 1σ`, monitor: MONITORS.sigma!.make(k) })),
  ...KS.map((k) => ({ id: `sigma-health-${k}`, label: `${k} × 1σ while tracking, ∞ once degraded or lost`, monitor: MONITORS['sigma-health']!.make(k) })),
];

const env = await createEnv();
const pilot = BASELINES[pilotId]!();
const flights: Array<{ task: string; seed: number; meanErr: number; maxErr: number; monitors: Record<string, IntegrityStats> }> = [];
// How the estimator's own health reads while the pose is bad (true error over the alert limit) and while it is good.
const calib = new Map<string, { bad: number[]; good: number[]; badTracking: number }>();
const t0 = Date.now();
for (const [ti, task] of INTEGRITY_TASKS.entries()) {
  for (const run of trackRuns('integrity', split, ti)) {
    const rec = new BenchEpisodeRecorder(env, task, run);
    pilot.reset(rec.ctx);
    const drift = new DriftFrames(env, run.seed, task.lighting !== 'day', task.faults ?? null, task.driftProfile);
    const log: Array<{ err: number; health: PoseHealth }> = [];
    let obs = rec.obs;
    let first = true;
    for (;;) {
      const frame = drift.frame(obs, first);
      const action = pilot.act(obs, rec.decorate(frame));
      log.push({ err: drift.error(), health: { ...frame.health! } });
      const s = env.step(action);
      first = false;
      obs = s.obs;
      if (rec.record(s)) break;
    }
    rec.finish();
    if (!calib.has(task.id)) calib.set(task.id, { bad: [], good: [], badTracking: 0 });
    const c = calib.get(task.id)!;
    for (const st of log) {
      if (st.err > INTEGRITY.alertLimit) {
        c.bad.push(st.health.sigmaM);
        if (st.health.state === 'tracking' && st.health.ok) c.badTracking += 1;
      } else c.good.push(st.health.sigmaM);
    }
    if (dump) for (const [i, st] of log.entries()) dumped.push(`${task.id},${run.seed},${i},${st.err.toFixed(4)},${st.health.sigmaM.toFixed(4)},${st.health.sigmaMapM.toFixed(4)},${st.health.sigmaYawRad.toFixed(5)},${st.health.state},${st.health.ok ? 1 : 0}`);
    const monitors: Record<string, IntegrityStats> = {};
    for (const v of variants) {
      const m = new IntegrityMeter();
      for (const step of log) m.step(step.err, v.monitor(step.health));
      monitors[v.id] = m.stats(rec.ctx.maxSteps);
    }
    flights.push({ task: task.id, seed: run.seed, meanErr: log.reduce((a, s) => a + s.err, 0) / log.length, maxErr: Math.max(...log.map((s) => s.err)), monitors });
  }
  console.log(`${task.id}: ${flights.filter((f) => f.task === task.id).length} flights (${Math.round((Date.now() - t0) / 1000)} s)`);
}
env.dispose();

const mean = (xs: number[]) => xs.reduce((a, b) => a + b, 0) / Math.max(1, xs.length);
// Caught is null on a flight with no bad-pose step: average over the flights that had one.
const meanKnown = (xs: Array<number | null>) => {
  const k = xs.filter((x): x is number => x !== null);
  return k.length ? mean(k) : null;
};
// Track score: the mean over tasks of each task's mean, as on every FrontierBench board.
const rows = variants.map((v) => {
  const per = INTEGRITY_TASKS.map((t) => {
    const fs = flights.filter((f) => f.task === t.id).map((f) => f.monitors[v.id]!);
    return { task: t.id, score: mean(fs.map((s) => s.score)), availability: mean(fs.map((s) => s.availability)), hmiRate: mean(fs.map((s) => s.hmiRate)), caught: meanKnown(fs.map((s) => s.caught)) };
  });
  return { id: v.id, label: v.label, score: mean(per.map((p) => p.score)), availability: mean(per.map((p) => p.availability)), hmiRate: mean(per.map((p) => p.hmiRate)), caught: meanKnown(per.map((p) => p.caught)), tasks: per };
});
for (const r of rows) console.log(`${r.id.padEnd(16)} score ${(r.score * 100).toFixed(1)}%  available ${(r.availability * 100).toFixed(1)}%  HMI ${(r.hmiRate * 100).toFixed(2)}%  caught ${r.caught === null ? 'n/a' : `${(r.caught * 100).toFixed(1)}%`}`);
const bad = mean(flights.map((f) => f.monitors.always!.badShare));
console.log(`bad-pose share ${(bad * 100).toFixed(1)}% of steps; flights with an error over ${INTEGRITY.alertLimit} m: ${flights.filter((f) => f.maxErr > INTEGRITY.alertLimit).length}/${flights.length}`);
const median = (xs: number[]) => {
  if (!xs.length) return null;
  const s = [...xs].sort((a, b) => a - b);
  return s[Math.floor(s.length / 2)]!;
};
const calibration = INTEGRITY_TASKS.map((t) => {
  const c = calib.get(t.id) ?? { bad: [], good: [], badTracking: 0 };
  return { task: t.id, badSteps: c.bad.length, sigmaOnBad: median(c.bad), sigmaOnGood: median(c.good), trackingOnBad: c.bad.length ? c.badTracking / c.bad.length : null };
});
for (const c of calibration) console.log(`${c.task.padEnd(18)} bad steps ${c.badSteps}  median 1σ bad ${c.sigmaOnBad?.toFixed(2) ?? 'n/a'} m, good ${c.sigmaOnGood?.toFixed(2) ?? 'n/a'} m  tracking while bad ${c.trackingOnBad === null ? 'n/a' : `${(c.trackingOnBad * 100).toFixed(1)}%`}`);
if (dump) writeFileSync(dump, `task,seed,step,err,sigma,sigmaMap,sigmaYaw,state,ok\n${dumped.join('\n')}\n`);
if (out) {
  const { base, count } = INTEGRITY_SEEDS[split];
  writeFileSync(out, `${JSON.stringify({ study: 'integrity', pilot: pilotId, split, seeds: { base, count, perTask: 100 }, commit, rules: INTEGRITY, ks: KS, rows, badShare: bad, calibration, flights }, null, 2)}\n`);
}

// Dev sweep for the harsh drift track: fly one pilot on drift tasks under sensor profiles given as JSON.
//   vite-node scripts/study/drift-harsh-dev.ts -- --pilot planner-view --tasks drift-quad --base 98001 --count 6 --profiles '{"base":{},"r4":{"lidarRange":4}}'
import { trackRuns, trackTasks, type DriftTrack } from '../../src/bench/hard';
import { runBenchEpisode } from '../../src/bench/run';
import { createEnv } from '../../src/sim/env';
import { loadPilot } from '../bench/lib';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}
const profiles = JSON.parse(arg('profiles', '{"base":{}}')) as Record<string, object & { truth?: boolean; maxSteps?: number }>;
const only = arg('tasks', 'drift-quad').split(',');
const base = Number(arg('base', '98001'));
const count = Number(arg('count', '6'));
const track = arg('track', 'drift') as DriftTrack;
const env = await createEnv();
const { make } = await loadPilot(arg('pilot', 'planner-view'));
const pilot = await make();
for (const [name, prof] of Object.entries(profiles)) {
  const { truth, maxSteps, ...p } = prof;
  trackTasks(track).forEach((t, ti) => {
    if (!only.includes(t.id)) return;
    const task = { ...t, drift: !truth, driftProfile: { ...t.driftProfile, ...p }, ...(maxSteps ? { maxSteps } : {}) };
    const t0 = Date.now();
    const eps = trackRuns(track, { base, count }, ti).map((run) => runBenchEpisode(env, task, run, pilot));
    const mean = (f: (e: (typeof eps)[number]) => number) => eps.reduce((a, e) => a + f(e), 0) / eps.length;
    console.log(
      `${name} ${t.id}: camera ${(mean((e) => e.camera!.footprint) * 100).toFixed(1)}% seen ${(mean((e) => e.camera!.seen ?? e.camera!.footprint) * 100).toFixed(1)}% home ${eps.filter((e) => e.returned !== false).length}/${eps.length} crash ${eps.filter((e) => e.crashed).length}` +
        (truth ? '' : ` err mean ${mean((e) => e.drift!.mean).toFixed(2)} max ${Math.max(...eps.map((e) => e.drift!.max)).toFixed(2)} final ${mean((e) => e.drift!.final).toFixed(2)} lost ${mean((e) => e.drift!.lostTicks).toFixed(0)}`) +
        (eps[0]?.drift?.other ? ` | other mean ${mean((e) => e.drift!.other!.mean).toFixed(2)} max ${Math.max(...eps.map((e) => e.drift!.other!.max)).toFixed(2)} final ${mean((e) => e.drift!.other!.final).toFixed(2)}` : '') +
        ` (${Math.round((Date.now() - t0) / 1000)} s) | ${eps.map((e) => (e.camera!.footprint * 100).toFixed(0) + (e.crashed ? 'X' : '') + (e.drift ? '/' + e.drift.max.toFixed(1) + '/' + e.drift.final.toFixed(1) : '') + (e.drift?.loop ? `[${e.drift.loop.keyframes} s${e.drift.loop.seqMatched} l${e.drift.loop.loopAccepted}/${e.drift.loop.loopTried}]` : '')).join(' ')}`,
    );
  });
}
env.dispose();

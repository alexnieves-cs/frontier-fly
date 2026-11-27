// Record one FrontierBench flight per pilot for the homepage replay: docs/replay/<track>-<task>-<seed>.json.
// The flights are the bench's own (runBenchEpisode's loop, same task, seed and pilot), so the score each trace
// ends on is the score that flight has in bench/results; scripts/docs/generate.mjs checks that it is.
//
//   npm run bench:replay -- --track hard --task large-quad --pilots planner-view,info-gain,random-walk
//   (the seed defaults to the task's first test seed, so the flight is not hand-picked)
import { mkdirSync, readFileSync, existsSync, writeFileSync } from 'node:fs';
import { execSync } from 'node:child_process';
import { trackRuns, trackTasks, type HardTrack } from '../../src/bench/hard';
import { BenchEpisodeRecorder, benchEpisodeSetup, CAMERA_FOV } from '../../src/bench/run';
import { DriftFrames } from '../../src/bench/drift';
import { FOOTPRINT_RADIUS, FootprintCoverage } from '../../src/eval/footprint';
import { flightMap } from '../../src/gen/validate';
import { voxelLayout } from '../../src/shared/voxels';
import { sensorFrame } from '../../src/sim/sensorPlanner';
import { createEnv } from '../../src/sim/env';
import { loadPilot } from './lib';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] ? process.argv[i + 1]! : fallback;
}

const track = arg('track', 'hard') as HardTrack;
const taskId = arg('task', 'large-quad');
const pilots = arg('pilots', 'planner-view').split(',');
const tasks = trackTasks(track);
const ti = tasks.findIndex((t) => t.id === taskId);
if (ti < 0) throw new Error(`no task ${taskId} on track ${track}`);
const task = tasks[ti]!;
const seed = Number(arg('seed', String(trackRuns(track, 'test', ti)[0]!.seed)));
const run = { seed };
const every = 2; // a pose every 2 steps: 10 per second
const commit = execSync('git rev-parse --short HEAD').toString().trim();

const env = await createEnv();
const { spec } = benchEpisodeSetup(task, run);
const { vw, vd, floors } = voxelLayout(spec);
if (floors !== 1) throw new Error('the replay draws one floor; pick a single-floor task');
const reachable = flightMap(spec).reachable.slice(0, vw * vd);
const traces = [];
for (const id of pilots) {
  const { make, source } = await loadPilot(id);
  const pilot = await make();
  // runBenchEpisode, with a second camera footprint beside the recorder's to time when each voxel is first seen.
  const rec = new BenchEpisodeRecorder(env, task, run);
  pilot.reset(rec.ctx);
  const cam = new FootprintCoverage([FOOTPRINT_RADIUS], CAMERA_FOV);
  cam.reset(spec);
  const firstSeen = new Int32Array(vw * vd).fill(-1);
  const pose: number[] = [];
  // Overwatch tasks: where the lost person stood, and whether that step counted (in view from 2 m or more).
  const person: number[] = [];
  const watched: number[] = [];
  const look = (step: number) => {
    const s = env.snapshot();
    cam.observe(env.world!, s.pos[0], s.pos[1], s.pos[2], s.yaw);
    const m = cam.seenMask();
    for (let k = 0; k < vw * vd; k++) if (m[k] && firstSeen[k] < 0) firstSeen[k] = step;
    if (step % every !== 0) return;
    pose.push(Math.round(s.pos[0] * 100) / 100, Math.round(s.pos[1] * 100) / 100, Math.round(s.yaw * 100) / 100);
    if (rec.overwatch) {
      const p = rec.overwatch.person;
      person.push(Math.round(p[0] * 100) / 100, Math.round(p[1] * 100) / 100);
      watched.push(step > 0 && rec.overwatch.watching ? 1 : 0);
    }
  };
  look(0);
  let obs = rec.obs;
  const drift = task.drift ? new DriftFrames(env, run.seed, task.lighting !== 'day', task.faults ?? null, task.driftProfile) : null;
  let first = true;
  let step = 0;
  for (;;) {
    const s = env.step(pilot.act(obs, rec.decorate(drift ? drift.frame(obs, first) : sensorFrame(env, obs))));
    first = false;
    obs = s.obs;
    step += 1;
    const over = rec.record(s);
    look(step);
    if (over) break;
  }
  await pilot.close?.();
  const ep = rec.finish();
  if (Math.abs(cam.coverage() - (ep.camera?.seen ?? ep.camera!.footprint)) > 1e-9 && !task.home) throw new Error('replay camera disagrees with the recorder');
  // The flight's own score in the result file, when the pilot has one for this track: the replay must match it.
  const dir = track === 'hard' ? 'bench/results/hard' : `bench/results/hard-${track}`;
  const file = `${dir}/${id}.json`;
  let matches: boolean | null = null;
  if (existsSync(file)) {
    const r = JSON.parse(readFileSync(file, 'utf8')) as { episodes: { task: string; seed: number; camera: { footprint: number }; watch?: { score: number } }[] };
    const e = r.episodes.find((x) => x.task === task.id && x.seed === seed);
    matches = e ? Math.abs(e.camera.footprint - ep.camera!.footprint) < 1e-9 && (!ep.watch || Math.abs(e.watch!.score - ep.watch.score) < 1e-9) : null;
  }
  console.log(`${id}: camera ${(ep.camera!.footprint * 100).toFixed(1)}%${ep.watch ? `, person in view ${(ep.watch.score * 100).toFixed(1)}%` : ''}, ${ep.steps} steps, crashed ${ep.crashed}, matches ${file}: ${matches}`);
  traces.push({
    id,
    name: pilot.name,
    source,
    camera: ep.camera!.footprint,
    ...(ep.watch ? { watch: ep.watch.score, found: ep.watch.found, person, watched } : {}),
    crashed: ep.crashed,
    collisions: ep.collisions,
    steps: ep.steps,
    ...(ep.returned === undefined ? {} : { returned: ep.returned }),
    matchesResult: matches,
    every,
    pose,
    firstSeen: Array.from(firstSeen),
  });
}
env.dispose();
mkdirSync('docs/replay', { recursive: true });
const name = `${track}-${task.id}-${seed}`;
const out = { track, task: { id: task.id, label: task.label, dynamics: task.dynamics, lighting: task.lighting, maxSteps: task.maxSteps ?? 2400, ...(task.person ? { person: true } : {}) }, seed, split: 'test', commit, voxel: 0.5, vw, vd, reachable: Array.from(reachable), traces };
writeFileSync(`docs/replay/${name}.json`, `${JSON.stringify(out)}\n`);
const indexPath = 'docs/replay/index.json';
const index = existsSync(indexPath) ? (JSON.parse(readFileSync(indexPath, 'utf8')) as { file: string }[]) : [];
const entry = { file: `docs/replay/${name}.json`, track, task: task.id, seed, pilots: traces.map((t) => t.id), commit };
writeFileSync(indexPath, `${JSON.stringify([...index.filter((x) => x.file !== entry.file), entry], null, 2)}\n`);
console.log(`wrote docs/replay/${name}.json`);

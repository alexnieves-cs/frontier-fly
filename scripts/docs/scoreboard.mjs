// The FrontierBench scoreboard: every track down the side, every baseline across the top, every cell a
// measured score from bench/results. scripts/docs/generate.mjs writes it to docs/scoreboard.json (the homepage
// reads that file at build time) and to the README's scoreboard block. Nothing here is typed by hand: a cell
// with no result file is empty, and the seed set of each row is read from the flights themselves.
import { existsSync, readFileSync } from 'node:fs';
import { join } from 'node:path';
import { mulberry32, readResults } from './bench.mjs';

/** The pilots across the top. `cite` is the paper a baseline reimplements; `house` marks this repository's own. */
export const SCOREBOARD_PILOTS = [
  { id: 'planner-view', short: 'View-first planner', house: true },
  { id: 'planner-tracker', short: 'Sensor-only planner', house: true },
  { id: 'info-gain', short: 'Information gain', cite: 'Gonzalez-Banos & Latombe 2002' },
  { id: 'nbvp', short: 'NBVP', cite: 'Bircher et al. 2016' },
  { id: 'rapid-frontier', short: 'Rapid frontier', cite: 'Cieslewski et al. 2017' },
  { id: 'yamauchi', short: 'Nearest frontier', cite: 'Yamauchi 1997' },
  { id: 'boustrophedon', short: 'BA* coverage', cite: 'Viet et al. 2013' },
  { id: 'random-walk', short: 'Random walk', floor: true },
];

/**
 * The tracks down the side. `value` is the per-flight number the row scores (a track's score is the mean over
 * its tasks of the per-task mean), `score` the result field that holds it; `unit` says what a cell means.
 */
export const SCOREBOARD_TRACKS = [
  {
    id: 'v3',
    label: 'FrontierBench v3',
    what: 'Seven indoor tasks: one room, hard buildings, lights off, fast, multi-floor, quadrotor, apartments',
    dir: 'bench/results',
    keep: (r) => r.bench === 3,
    score: 'score',
    value: (e) => e.coverage,
    unit: 'share of the building flown through',
    note: 'v3 scores the share flown through; the frontier and next-best-view methods stop once their lidar has mapped every room, so they rank low there (judge them on the camera tracks).',
    flag: 'npm run bench:board -- --only',
  },
  {
    id: 'hard',
    label: 'Hard',
    what: 'Large buildings, a sprint, lights off, multi-floor, on the randomized quadrotor; scored by a 90° forward camera',
    dir: 'bench/results/hard',
    score: 'cameraScore',
    value: (e) => e.camera.footprint,
    unit: 'share of the building in camera view within 3 m',
    flag: 'npm run bench:hard -- --split test --only',
  },
  {
    id: 'return',
    label: 'Return',
    what: 'Hard buildings where the camera counts only if the drone lands back within 1.5 m of take-off',
    dir: 'bench/results/hard-return',
    score: 'cameraScore',
    value: (e) => e.camera.footprint,
    unit: 'camera footprint, zero unless it came home',
    flag: 'npm run bench:hard -- --track return --split test --only',
  },
  {
    id: 'drift',
    label: 'Drift',
    what: 'Every pilot flies on its own pose estimate (lidar odometry), not the true pose',
    dir: 'bench/results/hard-drift',
    control: 'bench/results/hard-drift-truth',
    score: 'cameraScore',
    value: (e) => e.camera.footprint,
    unit: 'camera footprint on the estimated pose',
    flag: 'npm run bench:hard -- --track drift --split test --only',
  },
  {
    id: 'drift-hard',
    label: 'Hard drift',
    what: 'Pose estimate from a 24-beam, 4 m time-of-flight ring: drift grows to meters',
    dir: 'bench/results/hard-drift-hard',
    control: 'bench/results/hard-drift-hard-truth',
    score: 'cameraScore',
    value: (e) => e.camera.footprint,
    unit: 'camera footprint on the estimated pose',
    flag: 'npm run bench:hard -- --track drift-hard --split test --only',
  },
  {
    id: 'sensor',
    label: 'Degraded sensors',
    what: 'A dead lidar sector, no optical flow in the dark, a sparse lidar, a sensor that fails mid-flight',
    dir: 'bench/results/hard-sensor',
    control: 'bench/results/hard-sensor-healthy',
    score: 'cameraScore',
    value: (e) => e.camera.footprint,
    unit: 'camera footprint with the faults',
    flag: 'npm run bench:hard -- --track sensor --split test --only',
  },
  {
    id: 'overwatch',
    label: 'Overwatch',
    what: 'Find a lost person walking through four rooms and keep them in camera view, never closer than 2 m',
    dir: 'bench/results/hard-overwatch',
    score: 'watchScore',
    value: (e) => e.watch.score,
    unit: 'share of the flight the person was in camera view from 2 m or more',
    flag: 'npm run bench:hard -- --track overwatch --split test --only',
  },
  {
    id: 'integrity',
    label: 'Integrity',
    what: 'Fly on the estimate and say when it has gone bad: false confidence costs more than honest doubt',
    dir: 'bench/results/hard-integrity',
    score: 'integrityScore',
    value: (e) => e.integrity.score,
    unit: 'integrity score (availability, with false confidence penalized)',
    flag: 'npm run bench:hard -- --track integrity --split test --only',
  },
];

/** Mean over tasks of the per-task mean, with a 95% bootstrap interval that resamples flights within each task. */
function trackScore(episodes, value, resamples = 2000) {
  const byTask = new Map();
  for (const e of episodes) {
    if (!byTask.has(e.task)) byTask.set(e.task, []);
    byTask.get(e.task).push(value(e));
  }
  const tasks = [...byTask.values()];
  const mean = (xs) => xs.reduce((s, x) => s + x, 0) / xs.length;
  const rand = mulberry32(11);
  const draws = [];
  for (let i = 0; i < resamples; i++) {
    let sum = 0;
    for (const d of tasks) {
      let s = 0;
      for (let j = 0; j < d.length; j++) s += d[Math.floor(rand() * d.length)];
      sum += s / d.length;
    }
    draws.push(sum / tasks.length);
  }
  draws.sort((x, y) => x - y);
  return { value: mean(tasks.map(mean)), ci: [draws[Math.floor(resamples * 0.025)], draws[Math.floor(resamples * 0.975)]] };
}

/** "seeds 93001-93030, +100 per task" from the flights a result holds. */
function seedSet(episodes, tasks) {
  const first = episodes.filter((e) => e.task === tasks[0].id).map((e) => e.seed);
  const lo = Math.min(...first);
  const hi = Math.max(...first);
  return { base: lo, count: first.length, last: hi, perTaskOffset: tasks.length > 1 ? Math.min(...episodes.filter((e) => e.task === tasks[1].id).map((e) => e.seed)) - lo : 0 };
}

/** A result that belongs to this board: test split, the current version of its track, a baseline. */
function rowsOf(root, track) {
  const all = readResults(root, track.dir).filter((r) => r.split === 'test' && r.baseline && Array.isArray(r.episodes) && r[track.score] !== undefined && (!track.keep || track.keep(r)));
  const v = Math.max(0, ...all.map((r) => r.hard ?? r.bench ?? 0));
  return all.filter((r) => (r.hard ?? r.bench ?? 0) === v && !r.loop && !r.block);
}

export function scoreboard(root) {
  const tracks = [];
  const cells = {};
  for (const track of SCOREBOARD_TRACKS) {
    const rows = rowsOf(root, track);
    if (!rows.length) continue;
    const by = Object.fromEntries(rows.map((r) => [r.source.replace('baseline:', ''), r]));
    const control = track.control ? Object.fromEntries(rowsOf(root, { ...track, dir: track.control }).map((r) => [r.source.replace('baseline:', ''), r])) : {};
    const tasks = rows[0].taskDefs;
    const seeds = seedSet(rows[0].episodes, tasks);
    const row = {};
    for (const p of SCOREBOARD_PILOTS) {
      const r = by[p.id];
      if (!r) continue;
      // Every cell in a row flew the same flights: same tasks, same seeds.
      const same = r.episodes.length === rows[0].episodes.length && r.episodes.every((e, i) => e.task === rows[0].episodes[i].task && e.seed === rows[0].episodes[i].seed);
      if (!same) throw new Error(`${r.path}: flights differ from ${rows[0].path}; a scoreboard row compares the same seeds`);
      const s = trackScore(r.episodes, track.value);
      const c = control[p.id];
      row[p.id] = {
        value: s.value,
        ci: s.ci,
        crashes: r.episodes.filter((e) => e.crashed).length,
        flights: r.episodes.length,
        ...(r.episodes.some((e) => e.returned !== undefined) ? { cameHome: r.episodes.filter((e) => e.returned).length } : {}),
        ...(c ? { control: trackScore(c.episodes, track.value).value } : {}),
        commit: r.commit,
        file: r.path,
      };
    }
    // The pilot with the best score in the row; a tie within a tenth of a point shares it.
    const best = Math.max(...Object.values(row).map((c) => c.value));
    for (const c of Object.values(row)) c.best = best - c.value < 0.0005;
    cells[track.id] = row;
    tracks.push({
      id: track.id,
      label: track.label,
      what: track.what,
      unit: track.unit,
      ...(track.note ? { note: track.note } : {}),
      tasks: tasks.map((t) => t.id),
      seeds,
      split: 'test',
      ...(track.control ? { controlLabel: track.id === 'sensor' ? 'same flights, healthy sensors' : 'same flights, true pose' } : {}),
      run: track.flag,
    });
  }
  // Columns in the order of their mean rank over the rows they appear in, so the board reads left to right.
  const rank = (id) => {
    const ranks = tracks
      .filter((t) => cells[t.id][id])
      .map((t) => {
        const vals = Object.values(cells[t.id]).map((c) => c.value);
        return vals.filter((v) => v > cells[t.id][id].value).length + 1;
      });
    return ranks.length ? ranks.reduce((a, b) => a + b, 0) / ranks.length : Infinity;
  };
  const names = Object.fromEntries(
    SCOREBOARD_TRACKS.flatMap((t) => rowsOf(root, t)).map((r) => [r.source.replace('baseline:', ''), r.name]),
  );
  const pilots = SCOREBOARD_PILOTS.filter((p) => tracks.some((t) => cells[t.id][p.id]))
    .map((p) => ({ ...p, name: names[p.id] ?? p.short, meanRank: rank(p.id) }))
    .sort((a, b) => a.meanRank - b.meanRank);
  return { source: 'bench/results (scripts/docs/scoreboard.mjs); regenerate with npm run docs', pilots, tracks, cells, replays: replays(root) };
}

/**
 * The recorded flights the homepage replays (npm run bench:replay), each checked against the flight with the
 * same task and seed in that pilot's result file: a replay that no longer matches the board fails the docs check.
 */
function replays(root) {
  const indexPath = join(root, 'docs/replay/index.json');
  if (!existsSync(indexPath)) return [];
  return JSON.parse(readFileSync(indexPath, 'utf8')).map((entry) => {
    const rec = JSON.parse(readFileSync(join(root, entry.file), 'utf8'));
    const dir = rec.track === 'hard' ? 'bench/results/hard' : `bench/results/hard-${rec.track}`;
    const flights = rec.traces.map((t) => {
      const file = join(root, dir, `${t.id}.json`);
      const r = existsSync(file) ? JSON.parse(readFileSync(file, 'utf8')) : null;
      const e = r?.episodes.find((x) => x.task === rec.task.id && x.seed === rec.seed);
      if (!e) throw new Error(`${entry.file}: ${t.id} has no ${rec.task.id} seed ${rec.seed} flight in ${dir}/${t.id}.json`);
      if (Math.abs(e.camera.footprint - t.camera) > 1e-9) throw new Error(`${entry.file}: ${t.id} replay scores ${t.camera}, the board's flight ${e.camera.footprint}; re-record it with npm run bench:replay`);
      return { id: t.id, name: t.name, camera: t.camera, crashed: t.crashed, result: `${dir}/${t.id}.json` };
    });
    return { file: entry.file, track: rec.track, task: rec.task, seed: rec.seed, commit: rec.commit, flights };
  });
}

const pct = (v) => `${(v * 100).toFixed(1)}%`;

/** The README's scoreboard block: the same cells as docs/scoreboard.json, best of each row in bold. */
export function scoreboardMarkdown(board) {
  const head = ['Track (test seeds)', ...board.pilots.map((p) => p.short)];
  const line = (cells) => `| ${cells.join(' | ')} |`;
  const rows = board.tracks.map((t) => {
    const seeds = `${t.seeds.base}+, ${t.seeds.count} per task × ${t.tasks.length}`;
    return line([
      `**${t.label}** (${seeds})`,
      ...board.pilots.map((p) => {
        const c = board.cells[t.id][p.id];
        if (!c) return '—';
        const v = pct(c.value);
        return c.best ? `**${v}**` : v;
      }),
    ]);
  });
  const notes = board.tracks.filter((t) => t.note).map((t) => ` ${t.note[0].toUpperCase()}${t.note.slice(1)}`).join('');
  const cite = board.pilots
    .filter((p) => p.cite || p.house || p.floor)
    .map((p) => `${p.short}: ${p.house ? 'this repository' : p.floor ? 'the floor, no map and no plan' : p.cite}`)
    .join('; ');
  return [
    line(head),
    line(head.map(() => '---')),
    ...rows,
    '',
    `Each cell is the mean over the track's tasks of the per-task mean on the test seeds, every pilot on the same flights; bold is the best in the row.${notes} ${cite}. Columns are ordered by mean rank. Intervals, crashes, the control runs and the commit of every cell are in [docs/scoreboard.json](docs/scoreboard.json) and [docs/LEADERBOARD.md](docs/LEADERBOARD.md).`,
  ].join('\n');
}

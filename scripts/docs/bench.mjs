// docs/LEADERBOARD.md tables, from bench/results/*.json (npm run bench:board) and
// bench/entries/*/result.json (submitted entries, re-scored in CI by npm run bench:verify).
import { existsSync, readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';

const pct = (v) => `${(v * 100).toFixed(1)}%`;
/** An entry's name is its author's text: keep it from adding links, HTML or table cells. */
const plain = (name) => String(name).replace(/[|[\]<>`\n\r]/g, ' ').replace(/\s+/g, ' ').trim().slice(0, 80);

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

function readResults(root, dir = 'bench/results') {
  const out = [];
  const results = join(root, dir);
  if (existsSync(results)) for (const f of readdirSync(results).sort()) if (f.endsWith('.json')) out.push({ ...JSON.parse(readFileSync(join(results, f), 'utf8')), path: `${dir}/${f}` });
  if (dir !== 'bench/results') return out;
  const entries = join(root, 'bench/entries');
  if (existsSync(entries))
    for (const d of readdirSync(entries).sort()) {
      const p = join(entries, d, 'result.json');
      if (existsSync(p)) out.push({ ...JSON.parse(readFileSync(p, 'utf8')), path: `bench/entries/${d}` });
    }
  return out;
}

/** Results of the current version only: a board never ranks scores flown under different rules. */
function current(rows) {
  const v = Math.max(0, ...rows.map((r) => r.bench ?? 0));
  return rows.filter((r) => (r.bench ?? 0) === v);
}

/** Flights the crash rule ended, as a share (v3 results; v2 flew without the rule). */
const crashes = (r) => {
  if (r.crashShare === undefined) return '—';
  // Counts, not a rounded share: 1 crash in 240 flights is not "0%".
  if (Array.isArray(r.episodes) && r.episodes.length) return `${r.episodes.filter((e) => e.crashed).length} of ${r.episodes.length}`;
  return `${(r.crashShare * 100).toFixed(1)}%`;
};

/** Small deterministic generator, so the board's intervals do not change between runs of the docs. */
function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

/**
 * Score of `a` minus score of `b` with a paired 95% bootstrap interval: both pilots flew the same
 * buildings, so flights are resampled as pairs, within each task (the score is a mean over tasks).
 * Null when the two results do not hold the same flights.
 */
export function pairedScoreGap(a, b, resamples = 4000, value = (e) => e.coverage) {
  if (!Array.isArray(a?.episodes) || !Array.isArray(b?.episodes)) return null;
  const key = (e) => `${e.task}:${e.seed}`;
  const other = new Map(b.episodes.map((e) => [key(e), value(e)]));
  const byTask = new Map();
  for (const e of a.episodes) {
    if (!other.has(key(e))) return null;
    if (!byTask.has(e.task)) byTask.set(e.task, []);
    byTask.get(e.task).push(value(e) - other.get(key(e)));
  }
  if (a.episodes.length !== b.episodes.length || !byTask.size) return null;
  const tasks = [...byTask.values()];
  const mean = (xs) => xs.reduce((s, x) => s + x, 0) / xs.length;
  const gap = mean(tasks.map(mean));
  const rand = mulberry32(7);
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
  return { gap, ci: [draws[Math.floor(resamples * 0.025)], draws[Math.floor(resamples * 0.975)]] };
}

const gapCell = (g) => {
  if (!g) return '—';
  const pts = (v) => `${v >= 0 ? '+' : '−'}${Math.abs(v * 100).toFixed(1)}`;
  const tied = g.ci[0] <= 0 && g.ci[1] >= 0;
  return `${pts(g.gap)} (${pts(g.ci[0])} to ${pts(g.ci[1])})${tied ? ', tied' : ''}`;
};

function readTeam(root) {
  const out = [];
  const results = join(root, 'bench/results/team');
  if (existsSync(results)) for (const f of readdirSync(results).sort()) if (f.endsWith('.json')) out.push({ ...JSON.parse(readFileSync(join(results, f), 'utf8')), path: `bench/results/team/${f}` });
  const entries = join(root, 'bench/entries');
  if (existsSync(entries))
    for (const d of readdirSync(entries).sort()) {
      const p = join(entries, d, 'team-result.json');
      if (existsSync(p)) out.push({ ...JSON.parse(readFileSync(p, 'utf8')), path: `bench/entries/${d}` });
    }
  return out.filter((r) => r.split === 'test');
}

function teamBoard(root) {
  const rows = readTeam(root);
  if (!rows.length) return '_Run `npm run bench:team -- --board`._';
  const tasks = rows.find((r) => r.taskDefs)?.taskDefs ?? [];
  rows.sort((a, b) => b.score - a.score);
  const sec = (v) => (v === null ? '—' : `${Math.round(v)} s`);
  return table(
    ['#', 'Team', 'Drones', 'Score', ...tasks.flatMap((t) => [t.id, `${t.id}: base knows`, `${t.id}: 80% in`]), 'Commit'],
    rows.map((r, i) => [
      String(i + 1),
      r.baseline ? `${r.name} (baseline)` : `[${plain(r.name)}](../${r.path})`,
      String(r.entry.drones),
      `**${pct(r.score)}**`,
      ...tasks.flatMap((t) => {
        const s = r.tasks[t.id];
        return s ? [pct(s.coverage), pct(s.baseKnown), `${sec(s.t80)} (${Math.round(s.reached80 * 100)}%)`] : ['—', '—', '—'];
      }),
      `\`${r.commit}\``,
    ]),
  );
}

/** Baselines that reimplement published exploration methods (src/sim/explorers), beside the house planner. */
const PUBLISHED = ['planner', 'planner-view', 'yamauchi', 'nbvp', 'boustrophedon'];

function publishedTable(rows) {
  rows = current(rows);
  const by = Object.fromEntries(rows.filter((r) => r.baseline).map((r) => [r.source?.replace('baseline:', ''), r]));
  const have = PUBLISHED.filter((id) => by[id]);
  if (have.length < 2) return '_Run `npm run bench:board -- --only yamauchi,nbvp,boustrophedon`._';
  const mean = (xs) => xs.reduce((a, b) => a + b, 0) / xs.length;
  const over = (r, f) => {
    const xs = Object.values(r.tasks).map(f).filter((v) => v !== null && v !== undefined);
    return xs.length ? mean(xs) : null;
  };
  return table(
    ['Pilot', 'Score (flown)', 'Footprint score', 'Footprint curve area', 'Half seen, s', 'Flights ended by a crash', 'Flights that touched nothing'],
    have.map((id) => {
      const r = by[id];
      const area = over(r, (t) => t.area?.footprint);
      const half = over(r, (t) => t.footprintHalfS);
      return [
        id === 'planner' || id === 'planner-view' ? `${r.name} (this repository)` : r.name,
        pct(r.score),
        r.footprintScore === undefined ? '—' : pct(r.footprintScore),
        area === null ? '—' : area.toFixed(2),
        half === null ? '—' : half.toFixed(1),
        crashes(r),
        `${Math.round(r.safeShare * 100)}%`,
      ];
    }),
  );
}

/** The v2 board (no crash rule, the old airframe's power model), kept so earlier citations can be checked. */
function v2Board(root) {
  const rows = readResults(root, 'bench/results/v2').filter((r) => r.split === 'test' && r.bench === 2);
  if (!rows.length) return '';
  const tasks = rows.find((r) => r.taskDefs)?.taskDefs ?? [];
  rows.sort((a, b) => b.score - a.score);
  return table(
    ['#', 'Pilot', 'Score', ...tasks.map((t) => t.id), 'Flights that touched nothing', 'Commit'],
    rows.map((r, i) => [String(i + 1), plain(r.name), pct(r.score), ...tasks.map((t) => (r.tasks[t.id] ? pct(r.tasks[t.id].coverage) : '—')), `${Math.round(r.safeShare * 100)}%`, `\`${r.commit}\``]),
  );
}

/** Two baselines' scores, for prose that compares them (null when either is missing). */
export function benchPair(root, a, b) {
  const rows = current(readResults(root).filter((r) => r.split === 'test' && r.baseline));
  const by = (id) => rows.find((r) => r.source === `baseline:${id}`);
  const ra = by(a);
  const rb = by(b);
  return ra && rb ? { a: ra, b: rb, version: ra.bench } : null;
}

/** FrontierBench Hard (src/bench/hard.ts): bench/results/hard/*.json, ranked by the camera footprint. */
function hardBoard(root) {
  return { ...trackBoard(root, 'bench/results/hard', 'BENCH_HARD', ''), ...trackBoard(root, 'bench/results/hard-return', 'BENCH_RETURN', ' --track return') };
}

/** One FrontierBench Hard track's board and task table; return tasks add the share of flights that came home. */
function trackBoard(root, dir, key, flag) {
  const rows = readResults(root, dir).filter((r) => r.split === 'test' && r.cameraScore !== undefined);
  if (!rows.length) return { [key]: `_Run \`npm run bench:hard --${flag} --only planner-view,nbvp --split test\`._`, [`${key}_TASKS`]: '' };
  const v = Math.max(...rows.map((r) => r.hard ?? 0));
  const cur = rows.filter((r) => (r.hard ?? 0) === v).sort((a, b) => b.cameraScore - a.cameraScore);
  const tasks = cur[0].taskDefs;
  const cam = (e) => e.camera.footprint;
  const home = tasks.some((t) => t.home);
  const back = (r) => {
    const eps = (r.episodes ?? []).filter((e) => e.returned !== undefined);
    return eps.length ? `${eps.filter((e) => e.returned).length} of ${eps.length}` : '—';
  };
  return {
    [key]: table(
      ['#', 'Pilot', 'Camera score', ...tasks.map((t) => t.id), ...(home ? ['Came home'] : []), '360° footprint', 'Flown', 'Flights ended by a crash', 'Camera vs the row above (95% CI)', 'Commit'],
      cur.map((r, i) => [
        String(i + 1),
        r.baseline ? `${r.name} (baseline)` : `[${plain(r.name)}](../${r.path})`,
        `**${pct(r.cameraScore)}**`,
        ...tasks.map((t) => (r.tasks[t.id]?.camera !== undefined ? pct(r.tasks[t.id].camera) : '—')),
        ...(home ? [back(r)] : []),
        r.footprintScore === undefined ? '—' : pct(r.footprintScore),
        pct(r.score),
        crashes(r),
        i === 0 ? '—' : gapCell(pairedScoreGap(r, cur[i - 1], 4000, cam)),
        `\`${r.commit}\``,
      ]),
    ),
    [`${key}_TASKS`]: table(
      ['Task', 'What', 'Airframe', 'Lighting', 'Budget'],
      tasks.map((t) => [`\`${t.id}\``, t.label, t.dynamics === 'quad' ? 'Quadrotor, randomized' : 'Kinematic', t.lighting === 'day' ? 'Day' : 'Lights off', `${(t.maxSteps ?? 2400) / 20} s`]),
    ),
  };
}

export function benchSection(root) {
  const rows = current(readResults(root).filter((r) => r.split === 'test'));
  const tasks = rows.find((r) => r.taskDefs)?.taskDefs ?? [];
  if (!rows.length) return { BENCH_BOARD: '_Run `npm run bench:board`._', BENCH_TASKS: '', BENCH_PUBLISHED: publishedTable(rows), BENCH_TEAM: teamBoard(root), BENCH_V2: v2Board(root), ...hardBoard(root) };
  rows.sort((a, b) => b.score - a.score);
  const board = rows.map((r, i) => [
    String(i + 1),
    r.baseline ? `${r.name} (baseline)` : `[${plain(r.name)}](../${r.path})`,
    `**${pct(r.score)}**`,
    r.footprintScore === undefined ? '—' : pct(r.footprintScore),
    ...tasks.map((t) => (r.tasks[t.id] ? pct(r.tasks[t.id].coverage) : '—')),
    crashes(r),
    `${Math.round(r.safeShare * 100)}%`,
    i === 0 ? '—' : gapCell(pairedScoreGap(r, rows[i - 1])),
    `\`${r.commit}\``,
  ]);
  return {
    BENCH_PUBLISHED: publishedTable(rows),
    BENCH_TEAM: teamBoard(root),
    BENCH_V2: v2Board(root),
    ...hardBoard(root),
    BENCH_BOARD: table(['#', 'Pilot', 'Score', 'Footprint score', ...tasks.map((t) => t.id), 'Flights ended by a crash', 'Flights that touched nothing', 'Score vs the row above (95% CI)', 'Commit'], board),
    BENCH_TASKS: table(
      ['Task', 'Buildings', 'Airframe', 'Lighting', 'Speed cap'],
      tasks.map((t) => [`\`${t.id}\``, t.label, t.dynamics === 'quad' ? 'Quadrotor, randomized' : 'Kinematic', t.lighting === 'day' ? 'Day' : 'Lights off', `${t.speedH} m/s`]),
    ),
  };
}

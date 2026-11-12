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
const crashes = (r) => (r.crashShare === undefined ? '—' : `${Math.round(r.crashShare * 100)}%`);

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
const PUBLISHED = ['planner', 'yamauchi', 'nbvp', 'boustrophedon'];

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
        id === 'planner' ? `${r.name} (this repository)` : r.name,
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

export function benchSection(root) {
  const rows = current(readResults(root).filter((r) => r.split === 'test'));
  const tasks = rows.find((r) => r.taskDefs)?.taskDefs ?? [];
  if (!rows.length) return { BENCH_BOARD: '_Run `npm run bench:board`._', BENCH_TASKS: '', BENCH_PUBLISHED: publishedTable(rows), BENCH_TEAM: teamBoard(root), BENCH_V2: v2Board(root) };
  rows.sort((a, b) => b.score - a.score);
  const board = rows.map((r, i) => [
    String(i + 1),
    r.baseline ? `${r.name} (baseline)` : `[${plain(r.name)}](../${r.path})`,
    `**${pct(r.score)}**`,
    r.footprintScore === undefined ? '—' : pct(r.footprintScore),
    ...tasks.map((t) => (r.tasks[t.id] ? pct(r.tasks[t.id].coverage) : '—')),
    crashes(r),
    `${Math.round(r.safeShare * 100)}%`,
    `\`${r.commit}\``,
  ]);
  return {
    BENCH_PUBLISHED: publishedTable(rows),
    BENCH_TEAM: teamBoard(root),
    BENCH_V2: v2Board(root),
    BENCH_BOARD: table(['#', 'Pilot', 'Score', 'Footprint score', ...tasks.map((t) => t.id), 'Flights ended by a crash', 'Flights that touched nothing', 'Commit'], board),
    BENCH_TASKS: table(
      ['Task', 'Buildings', 'Airframe', 'Lighting', 'Speed cap'],
      tasks.map((t) => [`\`${t.id}\``, t.label, t.dynamics === 'quad' ? 'Quadrotor, randomized' : 'Kinematic', t.lighting === 'day' ? 'Day' : 'Lights off', `${t.speedH} m/s`]),
    ),
  };
}

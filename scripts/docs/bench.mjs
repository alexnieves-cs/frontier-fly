// docs/LEADERBOARD.md tables, from bench/results/*.json (npm run bench:board) and
// bench/entries/*/result.json (submitted entries, re-scored in CI by npm run bench:verify).
import { existsSync, readdirSync, readFileSync } from 'node:fs';
import { join } from 'node:path';

const pct = (v) => `${(v * 100).toFixed(1)}%`;

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

function readResults(root) {
  const out = [];
  const results = join(root, 'bench/results');
  if (existsSync(results)) for (const f of readdirSync(results).sort()) if (f.endsWith('.json')) out.push({ ...JSON.parse(readFileSync(join(results, f), 'utf8')), path: `bench/results/${f}` });
  const entries = join(root, 'bench/entries');
  if (existsSync(entries))
    for (const d of readdirSync(entries).sort()) {
      const p = join(entries, d, 'result.json');
      if (existsSync(p)) out.push({ ...JSON.parse(readFileSync(p, 'utf8')), path: `bench/entries/${d}` });
    }
  return out;
}

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
      r.baseline ? `${r.name} (baseline)` : `[${r.name}](../${r.path})`,
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

export function benchSection(root) {
  const rows = readResults(root).filter((r) => r.split === 'test');
  const tasks = rows.find((r) => r.taskDefs)?.taskDefs ?? [];
  if (!rows.length) return { BENCH_BOARD: '_Run `npm run bench:board`._', BENCH_TASKS: '', BENCH_TEAM: teamBoard(root) };
  rows.sort((a, b) => b.score - a.score);
  const board = rows.map((r, i) => [
    String(i + 1),
    r.baseline ? `${r.name} (baseline)` : `[${r.name}](../${r.path})`,
    `**${pct(r.score)}**`,
    ...tasks.map((t) => (r.tasks[t.id] ? pct(r.tasks[t.id].coverage) : '—')),
    `${Math.round(r.safeShare * 100)}%`,
    `\`${r.commit}\``,
  ]);
  return {
    BENCH_TEAM: teamBoard(root),
    BENCH_BOARD: table(['#', 'Pilot', 'Score', ...tasks.map((t) => t.id), 'Flights that touched nothing', 'Commit'], board),
    BENCH_TASKS: table(
      ['Task', 'Buildings', 'Airframe', 'Lighting', 'Speed cap'],
      tasks.map((t) => [`\`${t.id}\``, t.label, t.dynamics === 'quad' ? 'Quadrotor, randomized' : 'Kinematic', t.lighting === 'day' ? 'Day' : 'Lights off', `${t.speedH} m/s`]),
    ),
  };
}

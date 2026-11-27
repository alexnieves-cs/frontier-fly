// The FrontierBench scoreboard at the top of the homepage. docs/scoreboard.json is generated from the bench
// result files by scripts/docs/generate.mjs (npm run docs, checked in CI), and this module only lays it out:
// no number on the board is typed by hand.
import board from '../../docs/scoreboard.json';

export interface ScoreCell {
  value: number;
  ci: number[];
  crashes: number;
  flights: number;
  cameHome?: number;
  control?: number;
  commit: string;
  file: string;
  best: boolean;
}

export interface ScoreTrack {
  id: string;
  label: string;
  what: string;
  unit: string;
  note?: string;
  tasks: string[];
  seeds: { base: number; count: number; last: number; perTaskOffset: number };
  split: string;
  controlLabel?: string;
  run: string;
}

export interface ScorePilot {
  id: string;
  short: string;
  name: string;
  cite?: string;
  house?: boolean;
  floor?: boolean;
}

export interface ReplayFlight {
  id: string;
  name: string;
  camera: number;
  crashed: boolean;
  result: string;
}

export interface ReplayEntry {
  file: string;
  track: string;
  task: { id: string; label: string; dynamics: string; lighting: string; maxSteps: number };
  seed: number;
  commit: string;
  flights: ReplayFlight[];
}

export const scoreboard = board as unknown as {
  pilots: ScorePilot[];
  tracks: ScoreTrack[];
  cells: Record<string, Record<string, ScoreCell>>;
  replays: ReplayEntry[];
};

const pct = (v: number): string => `${(v * 100).toFixed(1)}%`;

const esc = (s: string): string => s.replace(/[&<>"]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' })[c]!);

/** "test seeds 93001 to 93030, the next task +100" */
function seedText(t: ScoreTrack): string {
  const per = t.seeds.perTaskOffset ? `, each next task +${t.seeds.perTaskOffset}` : '';
  return `${t.tasks.length} tasks × ${t.seeds.count} ${t.split} seeds (${t.seeds.base} to ${t.seeds.last}${per})`;
}

function cellTitle(p: ScorePilot, t: ScoreTrack, c: ScoreCell): string {
  const parts = [
    `${p.name} on ${t.label}: ${pct(c.value)}, 95% interval ${pct(c.ci[0]!)} to ${pct(c.ci[1]!)}`,
    `${c.crashes} of ${c.flights} flights ended by a crash`,
    ...(c.cameHome !== undefined ? [`${c.cameHome} of ${c.flights} came home`] : []),
    ...(c.control !== undefined && t.controlLabel ? [`${t.controlLabel}: ${pct(c.control)}`] : []),
    `commit ${c.commit}, ${c.file}`,
  ];
  return parts.join('. ');
}

/** The board as an HTML table: tracks down the side, pilots across the top, each cell a measured score. */
export function scoreboardTable(repo: string): string {
  const { pilots, tracks, cells } = scoreboard;
  const head = pilots
    .map((p) => {
      const tag = p.house ? 'this repo' : p.floor ? 'floor' : p.cite!;
      return `<th scope="col" class="sb-pilot${p.house ? ' sb-house' : ''}${p.floor ? ' sb-floor' : ''}" title="${esc(p.name)}"><span class="sb-pilot-name">${esc(p.short)}</span><small>${esc(tag)}</small></th>`;
    })
    .join('');
  const rows = tracks
    .map((t) => {
      const tds = pilots
        .map((p) => {
          const c = cells[t.id]?.[p.id];
          if (!c) return '<td class="sb-cell sb-empty" title="not run on this track">—</td>';
          // The cell's shade follows its score, so the board reads as a heat map before the numbers do.
          return `<td class="sb-cell${c.best ? ' sb-best' : ''}" style="--v:${Math.max(0, Math.min(1, c.value)).toFixed(3)}" title="${esc(cellTitle(p, t, c))}"><span class="sb-num">${pct(c.value)}</span>${c.crashes ? `<small class="sb-crash">${c.crashes} crash${c.crashes === 1 ? '' : 'es'}</small>` : ''}</td>`;
        })
        .join('');
      return `<tr><th scope="row" class="sb-track"><a href="${repo}/blob/main/docs/LEADERBOARD.md" rel="noopener">${esc(t.label)}</a><span class="sb-what">${esc(t.what)}</span>${t.note ? `<span class="sb-what sb-note">${esc(t.note)}</span>` : ''}<span class="sb-seeds">${esc(seedText(t))}</span></th>${tds}</tr>`;
    })
    .join('');
  return `<div class="sb-scroll" role="region" aria-label="Scoreboard, scrolls sideways" tabindex="0">
      <table class="sb-table">
        <caption class="sr-only">FrontierBench scoreboard: each track's score for each baseline pilot, on the test seeds named in the row</caption>
        <thead><tr><th scope="col" class="sb-corner">Track</th>${head}</tr></thead>
        <tbody>${rows}</tbody>
      </table>
    </div>`;
}

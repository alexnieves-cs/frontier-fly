// RL fine-tuning section of docs/REPORT.md, from docs/rl-finetune.json (scripts/study/rl-finetune.ts).

const pct = (v) => `${(v * 100).toFixed(1)}%`;
const pts = (v) => `${v >= 0 ? '+' : '−'}${Math.abs(v * 100).toFixed(1)}`;
const ci = ([lo, hi]) => `${pts(lo)} to ${pts(hi)}`;

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

const SET_LABEL = {
  protocol: 'Eval protocol',
  study: 'Coverage study',
  report: 'Report block',
};

/** The learning-curve figure: tuning-block hard coverage per evaluation, one line per run. */
function curves(runs) {
  const drawn = runs.filter((r) => r.curve?.length > 1);
  if (!drawn.length) return null;
  const W = 640;
  const H = 300;
  const pad = { l: 48, r: 200, t: 16, b: 36 };
  const maxU = Math.max(...drawn.flatMap((r) => r.curve.map((p) => p.update)));
  const lo = 0.2;
  const hi = 0.8;
  const X = (u) => pad.l + (u / maxU) * (W - pad.l - pad.r);
  const Y = (v) => pad.t + (1 - (v - lo) / (hi - lo)) * (H - pad.t - pad.b);
  const colors = ['#1f6fd1', '#2a9d55', '#d93a3a', '#d08a3a', '#7a4fc9', '#555555'];
  const parts = [`<rect width="${W}" height="${H}" fill="#fff"/>`];
  for (let v = lo; v <= hi + 1e-9; v += 0.1) {
    parts.push(`<line x1="${pad.l}" x2="${W - pad.r}" y1="${Y(v)}" y2="${Y(v)}" stroke="#e3e5e8"/>`);
    parts.push(`<text x="${pad.l - 6}" y="${Y(v) + 4}" font-size="11" text-anchor="end" fill="#555" font-family="sans-serif">${Math.round(v * 100)}%</text>`);
  }
  const step = maxU > 300 ? 100 : 20;
  for (let u = 0; u <= maxU; u += step) {
    parts.push(`<text x="${X(u)}" y="${H - pad.b + 16}" font-size="11" text-anchor="middle" fill="#555" font-family="sans-serif">${u}</text>`);
  }
  parts.push(`<text x="${(pad.l + W - pad.r) / 2}" y="${H - 4}" font-size="11" text-anchor="middle" fill="#555" font-family="sans-serif">PPO update (hard coverage on the tuning block)</text>`);
  drawn.forEach((r, i) => {
    const col = colors[i % colors.length];
    const pts = r.curve.map((p) => `${X(p.update).toFixed(1)},${Y(Math.max(lo, p.large)).toFixed(1)}`).join(' ');
    parts.push(`<polyline fill="none" stroke="${col}" stroke-width="2" points="${pts}"/>`);
    parts.push(`<rect x="${W - pad.r + 12}" y="${pad.t + i * 20}" width="12" height="3" fill="${col}"/>`);
    parts.push(`<text x="${W - pad.r + 30}" y="${pad.t + i * 20 + 5}" font-size="11" fill="#333" font-family="sans-serif">${r.label ?? r.id}</text>`);
  });
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">${parts.join('')}</svg>\n`;
}

export function rlSection(doc) {
  const keys = ['RL_NOTE', 'RL_TABLE', 'RL_RUNS_TABLE', 'RL_CURVES_FIGURE', 'R_RL_BASE_HARD', 'R_RL_HARD', 'R_RL_DELTA', 'R_RL_CI', 'R_RL_N', 'R_RL_BASE_COLL', 'R_RL_COLL', 'R_RL_STUDY_DAY', 'R_RL_STUDY_NIGHT', 'R_RL_PROTO_DAY', 'R_RL_PROTO_NIGHT', 'R_RL_SHIPPED'];
  if (!doc) {
    const none = '_Not measured yet. Run scripts/study/rl-finetune.ts._';
    return { values: Object.fromEntries(keys.map((k) => [k, none])), files: {} };
  }
  const base = doc.baseline;
  const shipped = doc.shipped;
  const arms = doc.arms.map((a) => a.label);
  const cell = (set, stage, lighting) => doc.cells.find((c) => c.set === set && c.stage === stage && c.lighting === lighting);
  const rows = doc.cells.map((c) => [
    `${SET_LABEL[c.set] ?? c.set}, ${c.stage === 0 ? 'easy' : 'hard'}, ${c.lighting === 'day' ? 'day' : 'night'} (${c.episodes})`,
    ...arms.map((a) => {
      const e = c[a];
      const d = e.deltaVsBase != null && c.stage === 3 ? ` (${pts(e.deltaVsBase)})` : '';
      return `${pct(e.coverage)}${d} · ${e.collisionsPerEpisode.toFixed(1)}`;
    }),
  ]);
  const p = doc.hardPooled;
  rows.push([`**All hard flights (${p.episodes})**`, ...arms.map((a) => `**${pct(p[a].coverage)}**${p[a].deltaCi ? ` (${ci(p[a].deltaCi)})` : ''} · ${p[a].collisionsPerEpisode.toFixed(1)}`)]);
  const runs = doc.runs ?? [];
  const runRows = runs.map((r) => {
    const last = r.curve.at(-1);
    const best = r.curve.reduce((b, x) => (x.score > b.score ? x : b), r.curve[0]);
    return [r.label ?? r.id, `\`${r.flags}\``, last ? `${pct(last.large)} at ${last.update}` : '—', best ? `${pct(best.large)} at ${best.update}` : '—'];
  });
  const svg = curves(runs);
  const s = p[shipped];
  return {
    values: {
      RL_NOTE: `Every number below is on buildings that played no part in training or in picking the checkpoint (picked on the tuning block 66001–66299). Each cell is coverage, the paired change against the DAgger checkpoint in points, and collisions per episode. The last row pools every hard flight with a bootstrap 95% interval on the paired change. From \`scripts/study/rl-finetune.ts\` at \`${doc.commit}\`.`,
      RL_TABLE: table(['Set, buildings, light (flights)', ...arms], rows),
      RL_RUNS_TABLE: table(['Run', 'Recipe (on top of the defaults)', 'Hard coverage at the end', 'Best checkpoint (tuning block)'], runRows),
      RL_CURVES_FIGURE: svg ? '![Hard coverage on the tuning block per PPO update, for each run](report/rl-curves.svg)' : '',
      R_RL_BASE_HARD: pct(p[base].coverage),
      R_RL_HARD: pct(s.coverage),
      R_RL_DELTA: pts(s.deltaVsBase),
      R_RL_CI: ci(s.deltaCi),
      R_RL_N: String(p.episodes),
      R_RL_BASE_COLL: p[base].collisionsPerEpisode.toFixed(1),
      R_RL_COLL: s.collisionsPerEpisode.toFixed(1),
      R_RL_STUDY_DAY: `${pct(cell('study', 3, 'day')[shipped].coverage)} against ${pct(cell('study', 3, 'day')[base].coverage)}`,
      R_RL_STUDY_NIGHT: `${pct(cell('study', 3, 'night-dark')[shipped].coverage)} against ${pct(cell('study', 3, 'night-dark')[base].coverage)}`,
      R_RL_PROTO_DAY: pct(cell('protocol', 3, 'day')[shipped].coverage),
      R_RL_PROTO_NIGHT: pct(cell('protocol', 3, 'night-dark')[shipped].coverage),
      R_RL_SHIPPED: shipped,
    },
    files: svg ? { 'docs/report/rl-curves.svg': svg } : {},
  };
}

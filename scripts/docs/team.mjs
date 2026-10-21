// The drone-team section of docs/REPORT.md, from docs/team-bench.json (npm run study:team).
// Returns template values and the SVG figure it draws. generate.mjs merges both.

const pct = (v) => `${(v * 100).toFixed(1)}%`;
const pp = (v) => `${v >= 0 ? '+' : '−'}${Math.abs(v * 100).toFixed(1)} pts`;
const sec = (v) => (v === null || v === undefined ? 'not reached' : `${Math.round(v)} s`);

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

const MISSING = 'not measured (run `npm run study:team`)';
const COLORS = { solo: '#8a8f98', team2: '#2a9d55', team3: '#1f6fd1', team4: '#7a4fc9' };

function axisText(x, y, text, anchor = 'middle', size = 11) {
  return `<text x="${x}" y="${y}" font-size="${size}" text-anchor="${anchor}" fill="#555" font-family="sans-serif">${text}</text>`;
}

/** Mean coverage over time per team size, one panel per condition; team3's base-known curve dashed. */
function curves(bench) {
  const pw = 340;
  const ph = 230;
  const pad = { l: 44, r: 14, t: 26, b: 36 };
  const W = bench.conditions.length * pw + 10;
  const H = ph + 46;
  const parts = [`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">`, `<rect width="${W}" height="${H}" fill="#fff"/>`];
  bench.conditions.forEach((c, ci) => {
    const x0 = ci * pw + pad.l;
    const iw = pw - pad.l - pad.r;
    const ih = ph - pad.t - pad.b;
    const seconds = c.maxSteps / 20;
    // Episode curves hold one sample per 5 s; a run that stopped early (95% searched) holds its last value.
    const n = Math.floor(seconds / 5);
    const meanCurve = (arm, key) => {
      const eps = bench.episodes.filter((e) => e.condition === c.id && e.arm === arm);
      if (!eps.length) return null;
      return Array.from({ length: n }, (_, i) => eps.reduce((s, e) => s + (e[key][i] ?? e[key][e[key].length - 1] ?? 0), 0) / eps.length);
    };
    const px = (i) => x0 + ((i + 1) * 5 * iw) / seconds;
    const py = (v) => pad.t + ih - v * ih;
    parts.push(`<rect x="${x0}" y="${pad.t}" width="${iw}" height="${ih}" fill="none" stroke="#ccc"/>`);
    for (const v of [0.25, 0.5, 0.75, 1]) {
      parts.push(`<line x1="${x0}" x2="${x0 + iw}" y1="${py(v)}" y2="${py(v)}" stroke="#eee"/>`);
      parts.push(axisText(x0 - 6, py(v) + 4, `${v * 100}%`, 'end', 10));
    }
    for (let t = 0; t <= seconds; t += seconds > 150 ? 60 : 30) parts.push(axisText(x0 + (t * iw) / seconds, pad.t + ih + 14, `${t}s`, 'middle', 10));
    parts.push(axisText(x0 + iw / 2, 16, c.label, 'middle', 12));
    for (const arm of ['solo', 'team2', 'team3', 'team4']) {
      const m = meanCurve(arm, 'curve');
      if (!m) continue;
      parts.push(`<polyline fill="none" stroke="${COLORS[arm]}" stroke-width="2" points="${m.map((v, i) => `${px(i).toFixed(1)},${py(v).toFixed(1)}`).join(' ')}"/>`);
    }
    const b = meanCurve('team3', 'baseCurve');
    if (b) parts.push(`<polyline fill="none" stroke="${COLORS.team3}" stroke-width="1.5" stroke-dasharray="4 3" points="${b.map((v, i) => `${px(i).toFixed(1)},${py(v).toFixed(1)}`).join(' ')}"/>`);
  });
  const legend = [
    ['solo', '1 drone'],
    ['team2', '2 drones'],
    ['team3', '3 drones'],
    ['team4', '4 drones'],
  ];
  legend.forEach(([arm, label], i) => {
    const x = 50 + i * 105;
    parts.push(`<line x1="${x}" x2="${x + 22}" y1="${H - 12}" y2="${H - 12}" stroke="${COLORS[arm]}" stroke-width="2"/>`);
    parts.push(axisText(x + 28, H - 8, label, 'start', 11));
  });
  const x = 50 + legend.length * 105;
  parts.push(`<line x1="${x}" x2="${x + 22}" y1="${H - 12}" y2="${H - 12}" stroke="${COLORS.team3}" stroke-width="1.5" stroke-dasharray="4 3"/>`);
  parts.push(axisText(x + 28, H - 8, '3 drones, base view', 'start', 11));
  parts.push('</svg>');
  return parts.join('\n');
}

/** Paired mean difference arm - base over the episodes of one condition, with a deterministic bootstrap interval. */
function pairedFromEpisodes(bench, cond, arm, base, key) {
  const a = new Map(bench.episodes.filter((e) => e.condition === cond && e.arm === arm).map((e) => [e.seed, key(e)]));
  const diffs = bench.episodes.filter((e) => e.condition === cond && e.arm === base && a.has(e.seed)).map((e) => a.get(e.seed) - key(e));
  if (!diffs.length) return null;
  // Same xorshift for every render, so the interval never changes between runs.
  let x = 7;
  const rand = () => ((x ^= x << 13), (x ^= x >>> 17), (x ^= x << 5), (x >>> 0) / 4294967296);
  const means = [];
  for (let b = 0; b < 2000; b++) {
    let sum = 0;
    for (let i = 0; i < diffs.length; i++) sum += diffs[Math.floor(rand() * diffs.length)];
    means.push(sum / diffs.length);
  }
  means.sort((p, q) => p - q);
  return { mean: diffs.reduce((p, q) => p + q, 0) / diffs.length, ci: [means[50], means[1949]] };
}

export function teamSection(bench) {
  const keys = [
    'TEAM_NOTE',
    'TEAM_TABLE',
    'TEAM_ABLATION_TABLE',
    'TEAM_FIGURE',
    'R_TEAM_SEEDS',
    'R_TEAM_SOLO_LARGE',
    'R_TEAM_THREE_LARGE',
    'R_TEAM_FOUR_LARGE',
    'R_TEAM_THREE_LARGE_D',
    'R_TEAM_SOLO_HARD_EIGHTY',
    'R_TEAM_THREE_HARD_EIGHTY',
    'R_TEAM_NOCLAIMS_D',
    'R_TEAM_NONE_D',
    'R_TEAM_PERFECT_D',
    'R_TEAM_RELAY_D',
    'R_TEAM_GAP',
    'R_TEAM_RELAY_GAP',
    'R_TEAM_WEAK_GAP',
    'R_TEAM_WEAK_RELAY_GAP',
    'R_TEAM_WEAK_RELAY_D',
    'R_TEAM_WEAK_RELAY_GAPMAX_D',
    'R_TEAM_RELAY_GAPMAX_D',
    'R_TEAM_CONTACTS',
    'R_TEAM_PACK',
    'R_TEAM_WH_SOLO',
    'R_TEAM_WH_FOUR',
  ];
  if (!bench || !bench.cells?.length || !bench.episodes) return { values: Object.fromEntries(keys.map((k) => [k, MISSING])), files: {} };
  const cell = (cond, arm) => bench.cells.find((c) => c.condition === cond && c.arm === arm);
  const d = (x) => (x ? `${pp(x.mean)} (95% CI ${pp(x.ci[0])} to ${pp(x.ci[1])})` : '—');
  const rows = [];
  for (const c of bench.conditions) {
    for (const a of ['solo', 'team2', 'team3', 'team4']) {
      const x = cell(c.id, a);
      if (!x) continue;
      rows.push([
        c.id === 'hard' ? 'Hard 20 m' : 'Large 40 m',
        x.label,
        `${pct(x.coverage)} (${pct(x.coverageCi[0])}–${pct(x.coverageCi[1])})`,
        a === 'solo' ? '—' : pp(x.coverageVsSolo.mean),
        sec(x.timeTo['50'].medianSeconds),
        sec(x.timeTo['80'].medianSeconds),
        x.coveragePerDroneMinute.toFixed(1),
        x.overlap.toFixed(2),
        pct(x.baseKnown),
        pct(x.delivery),
        x.energyWh === undefined ? '—' : x.energyWh.toFixed(1),
        x.coveragePerWh === undefined ? '—' : x.coveragePerWh.toFixed(1),
      ]);
    }
  }
  const ab = [];
  for (const c of bench.conditions) {
    for (const a of ['team3', 'team3-none', 'team3-noclaims', 'team3-perfect', 'team3-relay', 'team3-weak', 'team3-weak-relay']) {
      const x = cell(c.id, a);
      if (!x) continue;
      ab.push([
        c.id === 'hard' ? 'Hard 20 m' : 'Large 40 m',
        x.label,
        pct(x.coverage),
        a === 'team3' ? '—' : x.coverageVsTeam3 ? d(x.coverageVsTeam3) : '—',
        sec(x.timeTo['80'].medianSeconds),
        pct(x.baseKnown),
        `${pct(x.awarenessGap)} / ${pct(x.awarenessGapMax)}`,
        x.collisionsPerEpisode.toFixed(2),
      ]);
    }
  }
  const L = (arm) => cell('large', arm);
  const H = (arm) => cell('hard', arm);
  const contacts = bench.cells.reduce((s, c) => s + c.droneContactsPerEpisode * c.episodes, 0);
  return {
    values: {
      TEAM_NOTE: `Commit \`${bench.commit}\`. ${bench.seedsPerCondition} fresh buildings per condition (hard ${bench.evalSeedBase.hard}+, large ${bench.evalSeedBase.large}+); the claim weights were tuned on ${bench.tuneSeeds[0]}–${bench.tuneSeeds[1]} only. Day light, kinematic airframe, every drone runs the sensor-only planner. Differences are paired by building.`,
      TEAM_TABLE: table(['Building', 'Team', 'Searched (95% CI)', 'vs 1 drone', 'Median time to 50%', 'Median time to 80%', 'Points per drone-minute', 'Overlap', 'Base knows at end', 'Packets through', 'Team battery, Wh', 'Points per Wh'], rows),
      TEAM_ABLATION_TABLE: table(['Building', 'Three drones', 'Searched', 'vs 3 drones, full system', 'Median time to 80%', 'Base knows at end', 'Awareness gap, mean / worst', 'Collisions per episode'], ab),
      TEAM_FIGURE: '![Mean searched floor over time for one to four drones](report/team-curves.svg)',
      R_TEAM_SEEDS: String(bench.seedsPerCondition),
      R_TEAM_SOLO_LARGE: pct(L('solo').coverage),
      R_TEAM_THREE_LARGE: pct(L('team3').coverage),
      R_TEAM_FOUR_LARGE: pct(L('team4').coverage),
      R_TEAM_THREE_LARGE_D: d(L('team3').coverageVsSolo),
      R_TEAM_SOLO_HARD_EIGHTY: sec(H('solo').timeTo['80'].medianSeconds),
      R_TEAM_THREE_HARD_EIGHTY: sec(H('team3').timeTo['80'].medianSeconds),
      R_TEAM_NOCLAIMS_D: d(L('team3-noclaims').coverageVsTeam3),
      R_TEAM_NONE_D: d(L('team3-none').coverageVsTeam3),
      R_TEAM_PERFECT_D: d(L('team3-perfect').coverageVsTeam3),
      R_TEAM_RELAY_D: d(L('team3-relay').coverageVsTeam3),
      R_TEAM_GAP: `${pct(L('team3').awarenessGap)} / ${pct(L('team3').awarenessGapMax)}`,
      R_TEAM_RELAY_GAP: `${pct(L('team3-relay').awarenessGap)} / ${pct(L('team3-relay').awarenessGapMax)}`,
      R_TEAM_WEAK_GAP: `${pct(L('team3-weak').awarenessGap)} / ${pct(L('team3-weak').awarenessGapMax)}`,
      R_TEAM_WEAK_RELAY_GAP: `${pct(L('team3-weak-relay').awarenessGap)} / ${pct(L('team3-weak-relay').awarenessGapMax)}`,
      R_TEAM_WEAK_RELAY_D: d(pairedFromEpisodes(bench, 'large', 'team3-weak-relay', 'team3-weak', (e) => e.coverage)),
      R_TEAM_WEAK_RELAY_GAPMAX_D: d(pairedFromEpisodes(bench, 'large', 'team3-weak-relay', 'team3-weak', (e) => e.awarenessGapMax)),
      R_TEAM_RELAY_GAPMAX_D: d(pairedFromEpisodes(bench, 'large', 'team3-relay', 'team3', (e) => e.awarenessGapMax)),
      R_TEAM_CONTACTS: String(Math.round(contacts)),
      R_TEAM_PACK: pct(Math.max(...bench.cells.map((c) => c.maxPackUsed ?? 0))),
      R_TEAM_WH_SOLO: L('solo').coveragePerWh?.toFixed(1) ?? '—',
      R_TEAM_WH_FOUR: L('team4').coveragePerWh?.toFixed(1) ?? '—',
    },
    files: { 'docs/report/team-curves.svg': `${curves(bench)}\n` },
  };
}

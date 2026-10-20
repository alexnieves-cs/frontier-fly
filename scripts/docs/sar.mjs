// The outdoor search section of docs/REPORT.md, from docs/sar-bench.json (npm run study:sar).
// Returns template values and the SVG figures it draws. generate.mjs merges both.

const pct = (v) => `${(v * 100).toFixed(1)}%`;
const pct0 = (v) => `${Math.round(v * 100)}%`;
const sec = (v) => (v === null || v === undefined || !Number.isFinite(v) ? '—' : `${Math.round(v)} s`);

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

const LABEL = {
  lanes: 'Lane sweep (shipped)',
  'lanes-sw': 'Lane sweep, sweep-width spacing, buildings last',
  poa: 'Probability search',
  'poa-greedy': 'Probability search, one-leg greedy',
  'poa-ring': 'Probability search, distance ring only',
  'poa-flat': 'Probability search, flat prior',
};
const CONDITION = {
  day: 'Day, behavior-model person',
  uniform: 'Day, person placed uniformly at random',
  night: 'Night (thermal), behavior-model person',
};
const COLORS = { lanes: '#8a8f98', 'lanes-sw': '#d08a3a', poa: '#1f6fd1', 'poa-greedy': '#2a9d55', 'poa-ring': '#7a4fc9', 'poa-flat': '#d93a3a' };

function axisText(x, y, text, anchor = 'middle', size = 11) {
  return `<text x="${x}" y="${y}" font-size="${size}" text-anchor="${anchor}" fill="#555" font-family="sans-serif">${text}</text>`;
}

/** P(found by t) for the main methods, one panel per condition. */
function findCurves(bench) {
  const conds = Object.keys(bench.conditions);
  const show = ['lanes', 'lanes-sw', 'poa', 'poa-flat'];
  const pw = 260;
  const ph = 220;
  const pad = { l: 44, r: 12, t: 26, b: 34 };
  const W = conds.length * pw + 10;
  const H = ph + 44;
  const grid = bench.curveGrid;
  const tMax = grid[grid.length - 1];
  const parts = [`<rect width="${W}" height="${H}" fill="#fff"/>`];
  conds.forEach((c, ci) => {
    const ox = ci * pw;
    const X = (t) => ox + pad.l + (t / tMax) * (pw - pad.l - pad.r);
    const Y = (v) => pad.t + (1 - v) * (ph - pad.t - pad.b);
    parts.push(axisText(ox + pw / 2, 14, CONDITION[c] ?? c, 'middle', 12));
    for (let v = 0; v <= 1.0001; v += 0.25) {
      parts.push(`<line x1="${X(0)}" x2="${X(tMax)}" y1="${Y(v)}" y2="${Y(v)}" stroke="#e3e5e8"/>`);
      if (ci === 0) parts.push(axisText(X(0) - 6, Y(v) + 4, `${Math.round(v * 100)}%`, 'end'));
    }
    for (let t = 0; t <= tMax; t += 300) parts.push(axisText(X(t), ph - pad.b + 16, `${t}`));
    parts.push(axisText(ox + pw / 2, ph - 4, 'seconds since takeoff'));
    for (const m of show) {
      const curve = bench.conditions[c].methods[m]?.curve;
      if (!curve) continue;
      // Step function: found fraction holds until the next grid time.
      const pts = [];
      curve.forEach((v, i) => {
        if (i > 0) pts.push(`${X(grid[i]).toFixed(1)},${Y(curve[i - 1]).toFixed(1)}`);
        pts.push(`${X(grid[i]).toFixed(1)},${Y(v).toFixed(1)}`);
      });
      const dash = m === 'poa-flat' ? ' stroke-dasharray="4 3"' : '';
      parts.push(`<polyline fill="none" stroke="${COLORS[m]}" stroke-width="2"${dash} points="${pts.join(' ')}"/>`);
    }
  });
  const short = { lanes: 'Lane sweep (shipped)', 'lanes-sw': 'Lanes at sweep width', poa: 'Probability search', 'poa-flat': 'Probability search, flat prior' };
  show.forEach((m, i) => {
    const x = 12 + i * ((W - 24) / show.length);
    parts.push(`<rect x="${x}" y="${H - 14}" width="14" height="3" fill="${COLORS[m]}"/>`);
    parts.push(`<text x="${x + 20}" y="${H - 9}" font-size="11" fill="#333" font-family="sans-serif">${short[m]}</text>`);
  });
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">${parts.join('')}</svg>\n`;
}

/** Left: per-frame POD model against the detector. Right: modelled POS against the found fraction over time. */
function calibrationFigure(bench) {
  const W = 620;
  const H = 270;
  const pw = 300;
  const pad = { l: 46, r: 14, t: 26, b: 40 };
  const parts = [`<rect width="${W}" height="${H}" fill="#fff"/>`];
  const panel = (ox, title, xLabel, yLabel, xMax) => {
    const X = (v) => ox + pad.l + (v / xMax) * (pw - pad.l - pad.r);
    const Y = (v) => pad.t + (1 - v) * (H - pad.t - pad.b);
    parts.push(axisText(ox + pw / 2, 14, title, 'middle', 12));
    for (let v = 0; v <= 1.0001; v += 0.25) {
      parts.push(`<line x1="${X(0)}" x2="${X(xMax)}" y1="${Y(v)}" y2="${Y(v)}" stroke="#e3e5e8"/>`);
      parts.push(axisText(X(0) - 6, Y(v) + 4, `${Math.round(v * 100)}%`, 'end'));
    }
    parts.push(axisText(ox + pw / 2, H - 6, xLabel));
    parts.push(`<text x="${ox + 12}" y="${(pad.t + H - pad.b) / 2}" font-size="11" fill="#555" font-family="sans-serif" transform="rotate(-90 ${ox + 12} ${(pad.t + H - pad.b) / 2})" text-anchor="middle">${yLabel}</text>`);
    return { X, Y };
  };
  const cal = bench.podCalibration;
  {
    const { X, Y } = panel(0, 'Per-frame POD: model vs detector', 'model POD (bins)', 'detector hit rate', 1);
    for (let v = 0; v <= 1.0001; v += 0.25) parts.push(axisText(X(v), H - pad.b + 14, v.toFixed(2)));
    parts.push(`<line x1="${X(0)}" y1="${Y(0)}" x2="${X(1)}" y2="${Y(1)}" stroke="#bbb" stroke-dasharray="4 3"/>`);
    for (const b of cal.bins) {
      if (!b.n) continue;
      const r = Math.max(2.5, Math.min(8, Math.sqrt(b.n) / 6));
      parts.push(`<circle cx="${X(b.predicted).toFixed(1)}" cy="${Y(b.observed).toFixed(1)}" r="${r.toFixed(1)}" fill="#1f6fd1" fill-opacity="0.8"/>`);
    }
  }
  {
    const day = bench.conditions.day?.calibration ?? [];
    const tMax = day.length ? day[day.length - 1].t : 900;
    const { X, Y } = panel(pw + 10, 'Search belief: modelled POS vs found (day)', 'seconds since takeoff', 'probability', tMax);
    for (let t = 0; t <= tMax; t += 300) parts.push(axisText(X(t), H - pad.b + 14, `${t}`));
    const line = (key, color, dash) =>
      parts.push(`<polyline fill="none" stroke="${color}" stroke-width="2"${dash ? ' stroke-dasharray="4 3"' : ''} points="${day.map((p) => `${X(p.t).toFixed(1)},${Y(p[key]).toFixed(1)}`).join(' ')}"/>`);
    line('predicted', '#1f6fd1', true);
    line('observed', '#1f6fd1', false);
    parts.push(`<rect x="${pw + 70}" y="${pad.t + 120}" width="14" height="3" fill="#1f6fd1"/>`);
    parts.push(`<text x="${pw + 90}" y="${pad.t + 125}" font-size="11" fill="#333" font-family="sans-serif">found</text>`);
    parts.push(`<line x1="${pw + 70}" x2="${pw + 84}" y1="${pad.t + 141}" y2="${pad.t + 141}" stroke="#1f6fd1" stroke-width="2" stroke-dasharray="4 3"/>`);
    parts.push(`<text x="${pw + 90}" y="${pad.t + 145}" font-size="11" fill="#333" font-family="sans-serif">model's POS, within reach</text>`);
  }
  return `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">${parts.join('')}</svg>\n`;
}

const NONE = '_Not measured yet. `npm run study:sar` writes `docs/sar-bench.json`._';
export const SAR_KEYS = [
  'SAR_NOTE',
  'SAR_TABLE',
  'SAR_PAIRED_TABLE',
  'SAR_STRATEGY_TABLE',
  'SAR_POD_NOTE',
  'SAR_CURVES_FIGURE',
  'SAR_CALIBRATION_FIGURE',
  'SAR_BELIEF_FIGURE',
  'R_SAR_SEEDS',
  'R_SAR_POA_300',
  'R_SAR_LANES_300',
  'R_SAR_SW_300',
  'R_SAR_POA_MEAN',
  'R_SAR_LANES_MEAN',
  'R_SAR_SW_MEAN',
  'R_SAR_POA_FOUND',
  'R_SAR_LANES_FOUND',
  'R_SAR_SW_FOUND',
  'R_SAR_POA_MEDIAN',
  'R_SAR_LANES_MEDIAN',
  'R_SAR_SW_MEDIAN',
  'R_SAR_D_LANES',
  'R_SAR_D_SW',
  'R_SAR_UNI_POA_MEAN',
  'R_SAR_UNI_SW_MEAN',
  'R_SAR_UNI_FLAT_MEAN',
  'R_SAR_NIGHT_POA_300',
  'R_SAR_NIGHT_LANES_300',
  'R_SAR_NIGHT_SW_300',
  'R_SAR_POA_BATTERY',
  'R_SAR_LANES_BATTERY',
  'R_SAR_SWEEP_WIDTH',
  'R_SAR_POD_PRED',
  'R_SAR_POD_OBS',
  'R_SAR_POS_90_PRED',
  'R_SAR_POS_90_OBS',
  'R_SAR_POS_300_PRED',
  'R_SAR_POS_300_OBS',
  'R_SAR_UNI_POS_900_PRED',
  'R_SAR_UNI_POS_900_OBS',
  'R_SAR_POA_BLD',
  'R_SAR_LANES_BLD',
  'R_SAR_SW_BLD',
  'R_SAR_BLD_N',
  'R_SAR_UNI_SEEDS',
  'R_SAR_NIGHT_SEEDS',
  'R_SAR_RING_MEAN',
  'R_SAR_FLAT_MEAN',
  'R_SAR_UNI_D_SW',
  'R_SAR_UNI_POA_BLD',
  'R_SAR_UNI_SW_BLD',
  'R_SAR_NIGHT_D_SW',
];

export function sarSection(bench, gallery) {
  if (!bench) {
    return { values: Object.fromEntries(SAR_KEYS.map((k) => [k, NONE])), files: {} };
  }
  const C = bench.conditions;
  const ci = ([lo, hi]) => `${Math.round(lo * 100)}–${Math.round(hi * 100)}%`;
  const rows = [];
  for (const [c, v] of Object.entries(C)) {
    for (const [m, s] of Object.entries(v.methods)) {
      rows.push([
        m === 'lanes' ? `**${CONDITION[c]}** (${v.scenes} worlds)` : '',
        LABEL[m],
        `${pct0(s.found)} (${ci(s.foundCi)})`,
        pct0(s.within300),
        sec(s.medianFindSec),
        sec(s.restrictedMeanSec),
        `${s.meanBatteryUsedPct.toFixed(1)}%`,
        String(s.falseAlarms),
      ]);
    }
  }
  const diff = (d) => {
    const [m, lo, hi] = d.dMean;
    const [f, flo, fhi] = d.dFound;
    const s = (x) => `${x > 0 ? '+' : ''}${Math.round(x)}`;
    const p = (x) => `${x > 0 ? '+' : ''}${Math.round(x * 100)}`;
    return [`${s(m)} s (${s(lo)} to ${s(hi)})`, `${p(f)} pts (${p(flo)} to ${p(fhi)})`];
  };
  const paired = [];
  for (const [c, v] of Object.entries(C)) {
    for (const [m, d] of Object.entries(v.vsLanes)) paired.push([CONDITION[c], `${LABEL[m]} vs lanes (shipped)`, ...diff(d)]);
    paired.push([CONDITION[c], `${LABEL.poa} vs ${LABEL['lanes-sw'].toLowerCase()}`, ...diff(v.poaVsSweep)]);
  }
  const strat = [];
  const day = C.day;
  for (const [s, by] of Object.entries(day?.strategies ?? {})) {
    strat.push([
      s.replace('_', ' '),
      String(by.poa.flights),
      `${pct0(by.lanes.found)} / ${sec(by.lanes.medianFindSec)}`,
      `${pct0(by['lanes-sw'].found)} / ${sec(by['lanes-sw'].medianFindSec)}`,
      `${pct0(by.poa.found)} / ${sec(by.poa.medianFindSec)}`,
      `${pct0(by['poa-flat'].found)} / ${sec(by['poa-flat'].medianFindSec)}`,
    ]);
  }
  const d = (c, m) => C[c].methods[m];
  const dl = day.vsLanes.poa.dMean;
  const ds = day.poaVsSweep.dMean;
  const cal = bench.podCalibration;
  // The calibration sample at or just before t.
  const signed = (x) => `${x > 0 ? '+' : ''}${Math.round(x)}`;
  const span = ([m, lo, hi]) => `${signed(m)} s (95% CI ${signed(lo)} to ${signed(hi)} s)`;
  const at = (cond, t) => [...cond.calibration].reverse().find((p) => p.t <= t) ?? { predicted: 0, observed: 0 };
  const values = {
    SAR_NOTE: [
      `Each condition flies the same seeded worlds (seeds ${bench.seedBase}+, ${C.day.scenes} by day, ${C.uniform.scenes} with a uniformly placed person, ${C.night.scenes} at night) with every method, from \`src/eval/sar.ts\` at \`${bench.commit}\`.`,
      `The mission is a find-person search with a ${bench.budgetSec} s and 80% battery budget, entry on auto and an operator who approves every ask. A flight counts as found when the person is confirmed from three bearings at 0.8 mean confidence.`,
      'Times to find are censored at the budget: the restricted mean counts a miss as the full budget, and the median is blank when half or more were missed. Found intervals are Wilson 95%.',
    ].join(' '),
    SAR_TABLE: table(['Condition', 'Method', 'Found (95% CI)', 'Found within 300 s', 'Median time to find', 'Restricted mean time', 'Battery used', 'False alarms'], rows),
    SAR_PAIRED_TABLE: table(['Condition', 'Comparison', 'Restricted mean time, difference (95% CI)', 'Found, difference (95% CI)'], paired),
    SAR_STRATEGY_TABLE: table(['How the person moved', 'Worlds', 'Lanes (shipped)', 'Lanes, sweep width', 'Probability search', 'Flat prior'], strat),
    SAR_POD_NOTE: `Over ${cal.frames.toLocaleString('en-US')} frames on development worlds the model predicts a mean per-frame POD of ${pct(cal.meanPredicted)} against ${pct(cal.meanObserved)} measured (Brier score ${cal.brier.toFixed(3)}).`,
    SAR_CURVES_FIGURE: '![Fraction of searches that have found the person by each time, per condition and method](report/sar-find-curves.svg)',
    SAR_CALIBRATION_FIGURE: '![Left: per-frame POD model against the detector. Right: the belief\'s probability of success against the fraction found](report/sar-calibration.svg)',
    SAR_BELIEF_FIGURE: gallery ? `![${gallery.caption}](report/sar-belief.svg)\n\n${gallery.caption}` : '_Not drawn yet. `npm run study:sar` draws it._',
    R_SAR_SEEDS: String(C.day.scenes),
    R_SAR_POA_300: pct0(d('day', 'poa').within300),
    R_SAR_LANES_300: pct0(d('day', 'lanes').within300),
    R_SAR_SW_300: pct0(d('day', 'lanes-sw').within300),
    R_SAR_POA_MEAN: sec(d('day', 'poa').restrictedMeanSec),
    R_SAR_LANES_MEAN: sec(d('day', 'lanes').restrictedMeanSec),
    R_SAR_SW_MEAN: sec(d('day', 'lanes-sw').restrictedMeanSec),
    R_SAR_POA_FOUND: pct0(d('day', 'poa').found),
    R_SAR_LANES_FOUND: pct0(d('day', 'lanes').found),
    R_SAR_SW_FOUND: pct0(d('day', 'lanes-sw').found),
    R_SAR_POA_MEDIAN: sec(d('day', 'poa').medianFindSec),
    R_SAR_LANES_MEDIAN: sec(d('day', 'lanes').medianFindSec),
    R_SAR_SW_MEDIAN: sec(d('day', 'lanes-sw').medianFindSec),
    R_SAR_D_LANES: span(dl),
    R_SAR_D_SW: span(ds),
    R_SAR_UNI_POA_MEAN: sec(d('uniform', 'poa').restrictedMeanSec),
    R_SAR_UNI_SW_MEAN: sec(d('uniform', 'lanes-sw').restrictedMeanSec),
    R_SAR_UNI_FLAT_MEAN: sec(d('uniform', 'poa-flat').restrictedMeanSec),
    R_SAR_NIGHT_POA_300: pct0(d('night', 'poa').within300),
    R_SAR_NIGHT_LANES_300: pct0(d('night', 'lanes').within300),
    R_SAR_NIGHT_SW_300: pct0(d('night', 'lanes-sw').within300),
    R_SAR_POA_BATTERY: `${d('day', 'poa').meanBatteryUsedPct.toFixed(1)}%`,
    R_SAR_LANES_BATTERY: `${d('day', 'lanes').meanBatteryUsedPct.toFixed(1)}%`,
    R_SAR_SWEEP_WIDTH: `${bench.sweepWidthM.day.toFixed(1)} m`,
    R_SAR_POD_PRED: pct(cal.meanPredicted),
    R_SAR_POD_OBS: pct(cal.meanObserved),
    R_SAR_POS_90_PRED: pct0(at(C.day, 90).predicted),
    R_SAR_POS_90_OBS: pct0(at(C.day, 90).observed),
    R_SAR_POS_300_PRED: pct0(at(C.day, 300).predicted),
    R_SAR_POS_300_OBS: pct0(at(C.day, 300).observed),
    R_SAR_UNI_POS_900_PRED: pct0(at(C.uniform, bench.budgetSec).predicted),
    R_SAR_UNI_POS_900_OBS: pct0(at(C.uniform, bench.budgetSec).observed),
    R_SAR_POA_BLD: pct0(d('day', 'poa').building.found),
    R_SAR_LANES_BLD: pct0(d('day', 'lanes').building.found),
    R_SAR_SW_BLD: pct0(d('day', 'lanes-sw').building.found),
    R_SAR_BLD_N: String(d('day', 'poa').building.flights),
    R_SAR_UNI_SEEDS: String(C.uniform.scenes),
    R_SAR_NIGHT_SEEDS: String(C.night.scenes),
    R_SAR_RING_MEAN: sec(d('day', 'poa-ring').restrictedMeanSec),
    R_SAR_FLAT_MEAN: sec(d('day', 'poa-flat').restrictedMeanSec),
    R_SAR_UNI_D_SW: span(C.uniform.poaVsSweep.dMean),
    R_SAR_UNI_POA_BLD: pct0(d('uniform', 'poa').building.found),
    R_SAR_UNI_SW_BLD: pct0(d('uniform', 'lanes-sw').building.found),
    R_SAR_NIGHT_D_SW: span(C.night.poaVsSweep.dMean),
  };
  return {
    values,
    files: {
      'docs/report/sar-find-curves.svg': findCurves(bench),
      'docs/report/sar-calibration.svg': calibrationFigure(bench),
    },
  };
}

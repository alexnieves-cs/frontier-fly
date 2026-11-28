// The "PX4 flying every drone" part of docs/REPORT.md, from docs/px4-team.json
// (scripts/px4/team.ts, then scripts/px4/team-report.ts). Returns template values only.

const pct = (v) => `${(v * 100).toFixed(1)}%`;
const pp = (v) => `${v >= 0 ? '+' : '−'}${Math.abs(v * 100).toFixed(1)} pts`;
const sec = (v) => (v === null || v === undefined ? 'not reached' : `${Math.round(v)} s`);
const m = (v) => `${v.toFixed(2)} m`;

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

const MISSING = 'not measured (run `scripts/px4/team.ts`, then `scripts/px4/team-report.ts`)';

// Categorical slots 1 to 3 of the report palette; orange and aqua sit under 3:1 on white,
// so every series also has its own marker shape, a direct label and the legend.
const ARM_STYLE = {
  px4: { color: '#257de0', shape: 'circle', label: 'PX4 in the loop' },
  quad: { color: '#f47230', shape: 'square', label: 'Quad model' },
  kinematic: { color: '#00b894', shape: 'triangle', label: 'Kinematic' },
};

function text(x, y, t, anchor = 'middle', size = 11, fill = '#555') {
  return `<text x="${x.toFixed(1)}" y="${y.toFixed(1)}" font-size="${size}" text-anchor="${anchor}" fill="${fill}" font-family="sans-serif">${t}</text>`;
}

function marker(shape, x, y, color) {
  const ring = 'stroke="#fff" stroke-width="2"';
  if (shape === 'square') return `<rect x="${(x - 4.5).toFixed(1)}" y="${(y - 4.5).toFixed(1)}" width="9" height="9" rx="1.5" fill="${color}" ${ring}/>`;
  if (shape === 'triangle') return `<path d="M${x.toFixed(1)} ${(y - 5.5).toFixed(1)} L${(x + 5.2).toFixed(1)} ${(y + 4).toFixed(1)} L${(x - 5.2).toFixed(1)} ${(y + 4).toFixed(1)} Z" fill="${color}" ${ring} stroke-linejoin="round"/>`;
  return `<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="4.5" fill="${color}" ${ring}/>`;
}

/** Searched floor against team size, one panel per building size, a line and 95% CI per airframe. */
function figure(data, conds, row, condTitle) {
  const pw = 430;
  const ph = 260;
  const pad = { l: 48, r: 160, t: 30, b: 42 };
  const W = conds.length * pw + 10;
  const H = ph + 34;
  const iw = pw - pad.l - pad.r;
  const ih = ph - pad.t - pad.b;
  const parts = [`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">`, `<rect width="${W}" height="${H}" fill="#fff"/>`];
  const offset = { px4: -7, quad: 0, kinematic: 7 };
  conds.forEach((cond, ci) => {
    const x0 = ci * pw + pad.l;
    const px = (n) => x0 + ((n - 1) * iw) / 3;
    const py = (v) => pad.t + ih - v * ih;
    for (const v of [0, 0.25, 0.5, 0.75, 1]) {
      parts.push(`<line x1="${x0 - 10}" x2="${x0 + iw + 10}" y1="${py(v)}" y2="${py(v)}" stroke="${v === 0 ? '#bbb' : '#eee'}"/>`);
      parts.push(text(x0 - 16, py(v) + 4, `${v * 100}%`, 'end', 10));
    }
    for (const n of [1, 2, 3, 4]) parts.push(text(px(n), pad.t + ih + 16, String(n), 'middle', 10));
    parts.push(text(x0 + iw / 2, pad.t + ih + 32, 'Drones in the team', 'middle', 10));
    parts.push(text(x0 + iw / 2, 18, condTitle[cond], 'middle', 12, '#222'));
    const ends = [];
    for (const arm of ['kinematic', 'quad', 'px4']) {
      const st = ARM_STYLE[arm];
      const pts = [1, 2, 3, 4].map((n) => ({ n, r: row(cond, arm, n) })).filter((p) => p.r);
      if (!pts.length) continue;
      const X = (p) => px(p.n) + offset[arm];
      parts.push(`<polyline fill="none" stroke="${st.color}" stroke-width="2" stroke-linejoin="round" points="${pts.map((p) => `${X(p).toFixed(1)},${py(p.r.coverage).toFixed(1)}`).join(' ')}"/>`);
      for (const p of pts) {
        const [lo, hi] = p.r.coverageCi;
        parts.push(`<line x1="${X(p).toFixed(1)}" x2="${X(p).toFixed(1)}" y1="${py(lo).toFixed(1)}" y2="${py(hi).toFixed(1)}" stroke="${st.color}" stroke-width="1.5"/>`);
      }
      for (const p of pts) parts.push(marker(st.shape, X(p), py(p.r.coverage), st.color));
      const last = pts[pts.length - 1];
      ends.push({ arm, y: py(last.r.coverage), x: X(last), v: last.r.coverage });
    }
    // Direct labels at the right end, pushed apart so they never overlap.
    ends.sort((a, b) => a.y - b.y);
    for (let i = 1; i < ends.length; i++) if (ends[i].y - ends[i - 1].y < 14) ends[i].y = ends[i - 1].y + 14;
    const lx = x0 + iw + 22;
    for (const e of ends) {
      parts.push(marker(ARM_STYLE[e.arm].shape, lx, e.y - 4, ARM_STYLE[e.arm].color));
      parts.push(text(lx + 9, e.y, `${ARM_STYLE[e.arm].label} ${pct(e.v)}`, 'start', 10, '#333'));
    }
  });
  ['px4', 'quad', 'kinematic'].forEach((arm, i) => {
    const x = 56 + i * 150;
    parts.push(marker(ARM_STYLE[arm].shape, x, H - 13, ARM_STYLE[arm].color));
    parts.push(text(x + 10, H - 9, ARM_STYLE[arm].label, 'start', 11, '#333'));
  });
  parts.push(text(W - 12, H - 9, 'Whiskers: 95% CI over buildings, most smaller than the markers', 'end', 10));
  parts.push('</svg>');
  return parts.join('\n');
}
const KEYS = [
  'PXT_TABLE',
  'PXT_SWAP',
  'PXT_HEALTH',
  'PXT_HARD_N',
  'PXT_LARGE_N',
  'PXT_HARD_SEEDS',
  'PXT_LARGE_SEEDS',
  'PXT_HARD_SOLO',
  'PXT_HARD_FOUR',
  'PXT_HARD_FOUR_T80',
  'PXT_HARD_QUAD_FOUR_T80',
  'PXT_LARGE_SOLO',
  'PXT_LARGE_FOUR',
  'PXT_LARGE_FOUR_D',
  'PXT_LARGE_QUAD_FOUR',
  'PXT_LARGE_SWAP_FOUR',
  'PXT_HARD_SWAP_TWO',
  'PXT_CONTACT_EPS',
  'PXT_CONTACT_FLIGHTS',
  'PXT_KIN_CONTACT_EPS',
  'PXT_KIN_FLIGHTS',
  'PXT_COLLISIONS',
  'PXT_CRASHES',
  'PXT_PX4_FLIGHTS',
  'PXT_EKF_MEAN',
  'PXT_EKF_WORST',
  'PXT_TRACK',
  'PXT_DROPS',
  'PXT_FIGURE',
  'PXT_SAFETY',
  'PXT_LOCKSTEP',
];

export function px4TeamSection(data) {
  if (!data || !data.rows?.length) return { values: Object.fromEntries(KEYS.map((k) => [k, MISSING])), files: {} };
  const row = (cond, arm, n) => data.rows.find((r) => r.cond === cond && r.arm === arm && r.drones === n);
  const swap = (cond, n) => data.comparisons.find((c) => c.cond === cond && c.drones === n);
  const label = { px4: 'PX4 in the loop', quad: 'Quad model', kinematic: 'Kinematic' };
  const condLabel = { hard: 'Hard 20 m', large: 'Large 40 m' };
  const conds = ['hard', 'large'].filter((c) => row(c, 'px4', 1));

  const rows = [];
  for (const cond of conds) {
    for (const arm of ['px4', 'quad', 'kinematic']) {
      for (const n of [1, 2, 3, 4]) {
        const r = row(cond, arm, n);
        if (!r) continue;
        rows.push([
          condLabel[cond],
          label[arm],
          `${n} drone${n > 1 ? 's' : ''}`,
          `${pct(r.coverage)} (${pct(r.coverageCi[0])}–${pct(r.coverageCi[1])})`,
          r.vsSolo === null ? '—' : pp(r.vsSolo),
          sec(r.t50),
          sec(r.t80),
          `${r.reached80} of ${r.buildings}`,
          r.contactSteps ? `${r.episodesWithContact} of ${r.buildings} (${(r.contactSteps * 0.05).toFixed(2)} s)` : '0',
          String(r.collisions),
          `${r.crashes} of ${r.flights}`,
        ]);
      }
    }
  }
  const head = ['Building', 'Airframe', 'Team', 'Searched (95% CI)', 'vs 1 drone', 'Median time to 50%', 'Median time to 80%', 'Reached 80%', 'Flights with a close pass (time under 0.35 m)', 'Wall hits', 'Flights ended by a crash'];

  const swapLines = conds
    .map((cond) => `- ${condLabel[cond]}: ${[1, 2, 3, 4].map((n) => swap(cond, n)).filter(Boolean).map((c) => `${c.drones} drone${c.drones > 1 ? 's' : ''} ${pp(c.px4MinusQuad)} (95% CI ${pp(c.ci[0])} to ${pp(c.ci[1])})`).join('; ')}.`)
    .join('\n');

  const px4Rows = data.rows.filter((r) => r.arm === 'px4' && r.px4);
  const healthRows = px4Rows.map((r) => [condLabel[r.cond], `${r.drones} drone${r.drones > 1 ? 's' : ''}`, m(r.px4.ekfRmsMean), m(r.px4.ekfRmsWorst), m(r.px4.ekfMaxWorst), `${r.px4.trackErrRmsMean.toFixed(2)} m/s`, String(r.px4.offboardDrops), String(r.px4.notOffboardAtEnd), r.px4.lockstepMissMean === undefined ? '—' : `${r.px4.lockstepMissMean.toFixed(1)} (${r.px4.lockstepMissWorst})`, String(r.failed)]);
  const healthHead = ['Building', 'Team', 'EKF2 error, RMS (mean over drones)', 'Worst drone, RMS', 'Worst instant', 'Velocity tracking error, RMS', 'Offboard drops', 'Not in offboard at the end', "Lockstep steps without PX4's answer, per flight (worst)", 'Episodes that failed to run'];

  const sum = (rs, f) => rs.reduce((s, r) => s + f(r), 0);
  const px4Team = data.rows.filter((r) => r.arm === 'px4');
  const kinTeam = data.rows.filter((r) => r.arm === 'kinematic' && r.drones > 1);
  const px4Multi = px4Team.filter((r) => r.drones > 1);
  const v = {
    PXT_TABLE: table(head, rows),
    PXT_SWAP: swapLines,
    PXT_HEALTH: table(healthHead, healthRows),
    PXT_HARD_N: String(data.buildings.hard?.length ?? 0),
    PXT_LARGE_N: String(data.buildings.large?.length ?? 0),
    PXT_HARD_SEEDS: data.buildings.hard?.length ? `${data.buildings.hard[0]}–${data.buildings.hard[data.buildings.hard.length - 1]}` : '—',
    PXT_LARGE_SEEDS: data.buildings.large?.length ? `${data.buildings.large[0]}–${data.buildings.large[data.buildings.large.length - 1]}` : '—',
    PXT_HARD_SOLO: pct(row('hard', 'px4', 1).coverage),
    PXT_HARD_FOUR: pct(row('hard', 'px4', 4).coverage),
    PXT_HARD_FOUR_T80: sec(row('hard', 'px4', 4).t80),
    PXT_HARD_QUAD_FOUR_T80: sec(row('hard', 'quad', 4).t80),
    PXT_LARGE_SOLO: row('large', 'px4', 1) ? pct(row('large', 'px4', 1).coverage) : MISSING,
    PXT_LARGE_FOUR: row('large', 'px4', 4) ? pct(row('large', 'px4', 4).coverage) : MISSING,
    PXT_LARGE_FOUR_D: row('large', 'px4', 4) ? `${pp(row('large', 'px4', 4).vsSolo)} (95% CI ${pp(row('large', 'px4', 4).vsSoloCi[0])} to ${pp(row('large', 'px4', 4).vsSoloCi[1])})` : MISSING,
    PXT_LARGE_QUAD_FOUR: row('large', 'quad', 4) ? pct(row('large', 'quad', 4).coverage) : MISSING,
    PXT_LARGE_SWAP_FOUR: swap('large', 4) ? `${pp(swap('large', 4).px4MinusQuad)} (95% CI ${pp(swap('large', 4).ci[0])} to ${pp(swap('large', 4).ci[1])})` : MISSING,
    PXT_HARD_SWAP_TWO: swap('hard', 2) ? `${pp(swap('hard', 2).px4MinusQuad)} (95% CI ${pp(swap('hard', 2).ci[0])} to ${pp(swap('hard', 2).ci[1])})` : MISSING,
    PXT_CONTACT_EPS: String(sum(px4Multi, (r) => r.episodesWithContact)),
    PXT_CONTACT_FLIGHTS: String(sum(px4Multi, (r) => r.buildings)),
    PXT_KIN_CONTACT_EPS: String(sum(kinTeam, (r) => r.episodesWithContact)),
    PXT_KIN_FLIGHTS: String(sum(kinTeam, (r) => r.buildings)),
    PXT_COLLISIONS: String(sum(px4Team, (r) => r.collisions)),
    PXT_CRASHES: String(sum(px4Team, (r) => r.crashes)),
    PXT_PX4_FLIGHTS: String(sum(px4Team, (r) => r.flights)),
    PXT_EKF_MEAN: `${m(Math.min(...px4Rows.map((r) => r.px4.ekfRmsMean)))} to ${m(Math.max(...px4Rows.map((r) => r.px4.ekfRmsMean)))}`,
    PXT_EKF_WORST: m(Math.max(...px4Rows.map((r) => r.px4.ekfMaxWorst))),
    PXT_TRACK: `${Math.min(...px4Rows.map((r) => r.px4.trackErrRmsMean)).toFixed(2)} to ${Math.max(...px4Rows.map((r) => r.px4.trackErrRmsMean)).toFixed(2)} m/s`,
    PXT_DROPS: String(sum(px4Rows, (r) => r.px4.offboardDrops)),
    PXT_FIGURE: '![Searched floor against team size for PX4, the quad model and the kinematic airframe, on hard and large buildings, with 95% intervals](report/px4-team.svg)',
  };
  const walls = sum(px4Team, (r) => r.collisions);
  const crashes = sum(px4Team, (r) => r.crashes);
  const drops = sum(px4Rows, (r) => r.px4.offboardDrops);
  const flights = sum(px4Team, (r) => r.flights);
  v.PXT_SAFETY =
    walls + crashes + drops === 0
      ? `- **Nothing else went wrong.** 0 wall hits and 0 crashes in ${flights} PX4 drone flights, and 0 drops out of offboard control.`
      : `- **Walls, crashes and offboard control.** ${walls} wall hit${walls === 1 ? '' : 's'} and ${crashes} crash${crashes === 1 ? '' : 'es'} in ${flights} PX4 drone flights, and ${drops} drop${drops === 1 ? '' : 's'} out of offboard control.`;
  const miss = px4Rows.filter((r) => r.px4.lockstepMissMean !== undefined);
  const missText = (cond) => {
    const rs = miss.filter((r) => r.cond === cond);
    if (!rs.length) return null;
    const lo = Math.min(...rs.map((r) => r.px4.lockstepMissMean)).toFixed(1);
    const hi = Math.max(...rs.map((r) => r.px4.lockstepMissMean)).toFixed(1);
    return `${lo === hi ? lo : `${lo} to ${hi}`} per flight on ${cond} buildings (worst ${Math.max(...rs.map((r) => r.px4.lockstepMissWorst))})`;
  };
  v.PXT_LOCKSTEP = miss.length
    ? `On a few lockstep steps PX4's motor outputs did not come back within the bridge's 200 ms wait, and the bridge held the last ones for that 4 ms step: ${conds.map(missText).filter(Boolean).join(' and ')}, out of 30,000 steps in a 120 s flight and 60,000 in a 240 s one. About eight came in every flight, whatever its length or team size, which puts them at start-up; the rest came while the replay films were rendering on the same four-core machine.`
    : '';
  const condTitle = { hard: 'Hard 20 m building, 120 s', large: 'Large 40 m building, 240 s' };
  return { values: v, files: { 'docs/report/px4-team.svg': `${figure(data, conds, row, condTitle)}\n` } };
}

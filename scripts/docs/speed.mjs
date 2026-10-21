// The speed and energy section of docs/REPORT.md, from docs/speed-study.json (npm run study:speed).
// Returns template values and the SVG figure it draws. generate.mjs merges both.

const pct = (v) => `${(v * 100).toFixed(1)}%`;
const ci = ([lo, hi]) => `${(lo * 100).toFixed(1)}–${(hi * 100).toFixed(1)}%`;

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

const LABEL = {
  planner: 'Planner',
  plannerMomentum: 'Planner + momentum',
  plannerBrake: 'Planner, braking filter',
  plannerBrakeMomentum: 'Planner, braking filter + momentum',
  plannerTrack: 'Planner + quad tracker',
  plannerBrakeTrack: 'Planner, braking filter + quad tracker',
  policy: 'Policy',
  guarded: 'Guarded policy',
};
const COLORS = { planner: '#8a8f98', plannerMomentum: '#1f6fd1', plannerBrake: '#d08a3a', plannerBrakeMomentum: '#2a9d55', plannerTrack: '#0f9fa8', plannerBrakeTrack: '#9a6b12', policy: '#d93a3a', guarded: '#7a4fc9' };

function text(x, y, s, anchor = 'middle', size = 11) {
  return `<text x="${x}" y="${y}" font-size="${size}" text-anchor="${anchor}" fill="#555" font-family="sans-serif">${s}</text>`;
}

/** Coverage against the speed cap, one panel per condition. */
function figure(study) {
  const conds = study.conditions;
  const pw = 330;
  const ph = 230;
  const pad = { l: 44, r: 12, t: 26, b: 36 };
  const W = conds.length * pw + 10;
  const H = ph + 70;
  const parts = [`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">`, `<rect width="${W}" height="${H}" fill="#fff"/>`];
  const speeds = study.speeds;
  conds.forEach((c, ci_) => {
    const x0 = ci_ * pw + pad.l;
    const iw = pw - pad.l - pad.r;
    const ih = ph - pad.t - pad.b;
    const X = (v) => x0 + ((v - speeds[0]) / (speeds[speeds.length - 1] - speeds[0])) * iw;
    const Y = (f) => pad.t + ih - f * ih;
    parts.push(text(x0 + iw / 2, 16, c.label, 'middle', 12));
    for (const f of [0, 0.25, 0.5, 0.75, 1]) {
      parts.push(`<line x1="${x0}" x2="${x0 + iw}" y1="${Y(f)}" y2="${Y(f)}" stroke="#eee"/>`);
      parts.push(text(x0 - 6, Y(f) + 4, `${f * 100}%`, 'end', 10));
    }
    for (const v of speeds) parts.push(text(X(v), pad.t + ih + 16, `${v}`, 'middle', 10));
    parts.push(text(x0 + iw / 2, pad.t + ih + 31, 'speed cap, m/s', 'middle', 10));
    for (const p of study.pilots) {
      const pts = speeds.map((v) => study.cells.find((x) => x.condition === c.id && x.pilot === p && x.speed === v)).filter(Boolean);
      if (!pts.length) continue;
      parts.push(`<polyline fill="none" stroke="${COLORS[p]}" stroke-width="2" points="${pts.map((x) => `${X(x.speed).toFixed(1)},${Y(x.coverage).toFixed(1)}`).join(' ')}"/>`);
      for (const x of pts) parts.push(`<circle cx="${X(x.speed).toFixed(1)}" cy="${Y(x.coverage).toFixed(1)}" r="2.5" fill="${COLORS[p]}"/>`);
    }
  });
  study.pilots.forEach((p, i) => {
    const x = 20 + (i % 3) * 220;
    const y = ph + 22 + Math.floor(i / 3) * 18;
    parts.push(`<rect x="${x}" y="${y - 9}" width="12" height="3" fill="${COLORS[p]}"/>`);
    parts.push(text(x + 18, y - 4, LABEL[p], 'start', 11));
  });
  parts.push('</svg>');
  return parts.join('\n');
}

export function speedSection(study) {
  const empty = { values: { SPEED_FINDINGS: '_Run `npm run study:speed`._', SPEED_TABLE: '', SPEED_FIGURE: '' }, files: {} };
  if (!study) return empty;
  const c = (cond, pilot, speed) => study.cells.find((x) => x.condition === cond && x.pilot === pilot && x.speed === speed);
  const lo = study.speeds[0];
  const hi = study.speeds[study.speeds.length - 1];
  const rows = [];
  for (const cond of study.conditions) {
    for (const p of study.pilots) {
      const cells = study.speeds.map((v) => c(cond.id, p, v));
      if (!cells[0]) continue;
      rows.push([
        cond.id === 'quad' ? 'Quadrotor' : 'Kinematic',
        LABEL[p],
        ...cells.map((x) => `${pct(x.coverage)} (${x.collisionsPerEpisode.toFixed(1)})`),
        `${cells[0].energyWh.toFixed(2)} → ${cells[cells.length - 1].energyWh.toFixed(2)}`,
      ]);
    }
  }
  const p2 = c('hard', 'planner', lo);
  const p5 = c('hard', 'planner', hi);
  const m2 = c('hard', 'plannerMomentum', lo);
  const m5 = c('hard', 'plannerMomentum', hi);
  const g2 = c('hard', 'guarded', lo);
  const g5 = c('hard', 'guarded', hi);
  const r2 = c('hard', 'policy', lo);
  const r5 = c('hard', 'policy', hi);
  const qb2 = c('quad', 'plannerBrake', lo);
  const qb5 = c('quad', 'plannerBrake', hi);
  const qm4 = c('quad', 'plannerBrakeMomentum', 4);
  const qb4 = c('quad', 'plannerBrake', 4);
  const qt2 = c('quad', 'plannerBrakeTrack', lo);
  const qt5 = c('quad', 'plannerBrakeTrack', hi);
  const qg2 = c('quad', 'guarded', lo);
  const qg5 = c('quad', 'guarded', hi);
  const save = (a, b) => `${Math.round((1 - b.energyWh / a.energyWh) * 100)}%`;
  const findings = [
    `- **On the kinematic drone, speed is the lever.** The planner covers ${pct(p2.coverage)} (${ci(p2.coverageCi)}) at ${lo} m/s and ${pct(p5.coverage)} (${ci(p5.coverageCi)}) at ${hi} m/s, with no collisions at any speed. Half the building is flown after ${Math.round(p2.halfS)} s at ${lo} m/s and ${Math.round(p5.halfS)} s at ${hi} m/s. This is the flight-time limit the map-prediction null result pointed to: choosing better frontiers barely matters, flying them faster does.`,
    `- **Momentum-aware goal choice saves energy, not coverage.** Scoring each frontier by how far the drone would have to swing round to reach it leaves coverage where it was (${pct(m2.coverage)} at ${lo} m/s, ${pct(m5.coverage)} at ${hi} m/s) and uses ${save(p2, m2)} less energy at ${lo} m/s and ${save(p5, m5)} less at ${hi} m/s, because the drone brakes and re-accelerates less. Energy is metered with the mission battery model for a 1.2 kg quadrotor, which charges for acceleration and gives nothing back on braking.`,
    `- **The guarded policy also gains from speed; the raw policy does not.** With the lidar guard it covers ${pct(g2.coverage)} at ${lo} m/s and ${pct(g5.coverage)} at ${hi} m/s, at ${g5.collisionsPerEpisode.toFixed(1)} collisions per episode. Without the guard it falls from ${pct(r2.coverage)} to ${pct(r5.coverage)}, and most episodes end early at the collision limit, which is why its energy falls. It was trained at ${lo} m/s and never saw these speeds.`,
    `- **On the quadrotor, a higher cap makes the planners worse.** With the braking filter the planner covers ${pct(qb2.coverage)} at ${lo} m/s and ${pct(qb5.coverage)} at ${hi} m/s, and energy barely moves: the airframe seldom reaches the cap. Indoors the quadrotor spends its time turning. It accelerates at about 3 m/s², so each change of direction costs most of a second, and a faster cruise only overshoots the next corner. On the tuning buildings, the kinematic drone with its lag slowed from 0.15 s to 0.5 s loses about as much coverage as the quadrotor does. Momentum-aware goal choice helps a little where the overshoot is worst: ${pct(qm4.coverage)} against ${pct(qb4.coverage)} at 4 m/s.`,
    ...(qt2
      ? [
          `- **A quad-aware tracker recovers part of the gap and nearly all the collisions** (\`SensorPlannerOptions.tracker\`, src/sim/tracker.ts). It ramps the command at the airframe's acceleration, slows for corners and for the U-turn at the end of each sweep lane, and caps the yaw rate while the drone faces its direction of travel. On the tuning buildings the last part mattered most: the env's command is body-frame velocity, so a drone yawing at full rate through a U-turn asks for a velocity that keeps rotating under it, and with the quadrotor's lag its sweep lanes broke up into loops. With the braking filter it covers ${pct(qt2.coverage)} (${ci(qt2.coverageCi)}) at ${lo} m/s against ${pct(qb2.coverage)} without it, at ${qt2.collisionsPerEpisode.toFixed(2)} collisions per episode against ${qb2.collisionsPerEpisode.toFixed(2)}, and ${pct(qt5.coverage)} at ${hi} m/s. A higher cap still does not pay: the U-turns between 0.5 m lanes limit the quadrotor to about 1 m/s whatever the cruise speed, and it stays well short of the kinematic drone. An earlier tracker with only the corner speeds and the ramp did no better than the planner.`,
        ]
      : []),
    `- **The guarded policy is the exception on the quadrotor.** It covers ${pct(qg2.coverage)} at ${lo} m/s and ${pct(qg5.coverage)} at ${hi} m/s, more than any planner, but at ${qg5.collisionsPerEpisode.toFixed(1)} collisions per episode. It pushes into corners the planners slow down for.`,
  ].join('\n');
  return {
    values: {
      SPEED_FINDINGS: findings,
      SPEED_TABLE: table(['Airframe', 'Pilot', ...study.speeds.map((v) => `${v} m/s`), `Energy, Wh (${lo} → ${hi} m/s)`], rows),
      SPEED_FIGURE: '![Coverage against the speed cap for each pilot, kinematic drone and quadrotor](report/speed.svg)',
    },
    files: { 'docs/report/speed.svg': figure(study) },
  };
}

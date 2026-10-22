// The sensor-footprint coverage section of docs/REPORT.md, from docs/study.json and docs/speed-study.json
// (both carry footprintStats per cell since src/eval/footprint.ts). Returns template values and the
// coverage-over-time figure. generate.mjs merges both.

const pct = (v) => `${(v * 100).toFixed(1)}%`;
const ci = ([lo, hi]) => `${(lo * 100).toFixed(1)}–${(hi * 100).toFixed(1)}`;
const sec = (h) => (h.s === null ? '—' : `${Math.round(h.s)} s`);

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

function text(x, y, s, anchor = 'middle', size = 11, fill = '#555') {
  return `<text x="${x}" y="${y}" font-size="${size}" text-anchor="${anchor}" fill="${fill}" font-family="sans-serif">${s}</text>`;
}

const PANELS = [
  { condition: 'hard', title: 'Hard buildings, kinematic drone (day)' },
  { condition: 'quad', title: 'Hard buildings, quadrotor + noise (day)' },
];
const FIG_PILOTS = [
  { id: 'teacher', label: 'Oracle teacher', color: '#8a8f98' },
  { id: 'classical', label: 'Sensor-only planner', color: '#1f6fd1' },
  { id: 'classicalPredict', label: 'Planner + learned map prediction', color: '#7a4fc9' },
  { id: 'classicalBrakeTrack', label: 'Planner, braking + quad tracker', color: '#0f9fa8' },
  { id: 'guarded', label: 'Policy + lidar guard', color: '#d93a3a' },
];

/** Mean coverage over time: footprint (solid) and flown (dashed), one panel per condition. */
function figure(study) {
  const pw = 360;
  const ph = 250;
  const pad = { l: 44, r: 14, t: 28, b: 38 };
  const W = PANELS.length * pw + 10;
  const H = ph + 74;
  const parts = [`<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">`, `<rect width="${W}" height="${H}" fill="#fff"/>`];
  PANELS.forEach((panel, pi) => {
    const x0 = pi * pw + pad.l;
    const iw = pw - pad.l - pad.r;
    const ih = ph - pad.t - pad.b;
    const cells = FIG_PILOTS.map((p) => study.cells.find((c) => c.condition === panel.condition && c.lighting === 'day' && c.pilot === p.id)).filter((c) => c?.footprintStats);
    if (!cells.length) return;
    const budget = cells[0].footprintStats.curve.budgetS;
    const n = cells[0].footprintStats.curve.footprint.length;
    const X = (k) => x0 + (k / n) * iw;
    const Y = (f) => pad.t + ih - f * ih;
    parts.push(text(x0 + iw / 2, 16, panel.title, 'middle', 12, '#222'));
    for (let v = 0; v <= 1.0001; v += 0.2) {
      parts.push(`<line x1="${x0}" x2="${x0 + iw}" y1="${Y(v).toFixed(1)}" y2="${Y(v).toFixed(1)}" stroke="#e3e5e8"/>`);
      parts.push(text(x0 - 6, Y(v) + 4, `${Math.round(v * 100)}%`, 'end', 10));
    }
    for (let s = 0; s <= budget + 1e-9; s += 30) parts.push(text(X((s / budget) * n), pad.t + ih + 16, `${s}`, 'middle', 10));
    parts.push(text(x0 + iw / 2, pad.t + ih + 31, 'seconds of flight', 'middle', 10));
    for (const p of FIG_PILOTS) {
      const c = cells.find((x) => x.pilot === p.id);
      if (!c) continue;
      const curve = c.footprintStats.curve;
      const pts = (series) => [`${X(0).toFixed(1)},${Y(0).toFixed(1)}`, ...series.map((v, k) => `${X(k + 1).toFixed(1)},${Y(v).toFixed(1)}`)].join(' ');
      parts.push(`<polyline fill="none" stroke="${p.color}" stroke-width="2" points="${pts(curve.footprint)}"/>`);
      parts.push(`<polyline fill="none" stroke="${p.color}" stroke-width="1.5" stroke-dasharray="4 3" points="${pts(curve.flown)}"/>`);
    }
  });
  FIG_PILOTS.forEach((p, i) => {
    const x = 20 + (i % 2) * 300;
    const y = ph + 22 + Math.floor(i / 2) * 18;
    parts.push(`<rect x="${x}" y="${y - 9}" width="12" height="3" fill="${p.color}"/>`);
    parts.push(text(x + 18, y - 4, p.label, 'start', 11, '#333'));
  });
  parts.push(text(W - 12, ph + 22 + 4, 'solid: inside the 3 m footprint', 'end', 11, '#333'));
  parts.push(text(W - 12, ph + 40 + 4, 'dashed: flown through', 'end', 11, '#333'));
  parts.push('</svg>');
  return `${parts.join('\n')}\n`;
}

const LABELS = {
  teacher: 'Oracle teacher (floor plan)',
  classical: 'Sensor-only planner',
  classicalBrake: 'Planner, braking filter',
  classicalTrack: 'Planner + quad tracker',
  classicalBrakeTrack: 'Planner, braking filter + quad tracker',
  classicalClimb: 'Planner, takes shafts',
  classicalPredict: 'Planner + learned map prediction',
  classicalOracleGain: 'Planner + true hidden gain (reference)',
  policy: 'Policy',
  guarded: 'Policy + lidar guard',
  guardedBrake: 'Policy + braking guard',
  waypoint: 'Waypoint follower',
  heuristic: 'Lidar heuristic',
};
const SPEED_LABELS = {
  planner: 'Planner',
  plannerMomentum: 'Planner + momentum',
  plannerBrake: 'Planner, braking filter',
  plannerBrakeMomentum: 'Planner, braking filter + momentum',
  plannerTrack: 'Planner + quad tracker',
  plannerBrakeTrack: 'Planner, braking filter + quad tracker',
  policy: 'Policy',
  guarded: 'Guarded policy',
};

export function footprintSection(study, speed) {
  const none = '_Not measured yet. Run `npm run study:coverage`._';
  const empty = { values: { FOOTPRINT_NOTE: none, FOOTPRINT_TABLE: '', FOOTPRINT_FINDINGS: '', FOOTPRINT_FIGURE: '', FOOTPRINT_SPEED_TABLE: '', FOOTPRINT_RADII: '' }, files: {} };
  if (!study?.footprint || !study.cells.some((c) => c.footprintStats)) return empty;
  const meta = study.footprint;
  const head = String(meta.headline);
  const radii = meta.radii.map(String);
  const cell = (cond, pilot, lighting = 'day') => study.cells.find((c) => c.condition === cond && c.lighting === lighting && c.pilot === pilot);
  const rows = [];
  for (const condition of study.conditions) {
    for (const lighting of condition.lighting) {
      for (const pilot of study.pilots) {
        const c = cell(condition.id, pilot, lighting);
        if (!c?.footprintStats) continue;
        const f = c.footprintStats;
        rows.push([
          condition.label,
          lighting,
          LABELS[pilot] ?? pilot,
          pct(c.coverage),
          `**${pct(f.footprint[head])}** (${ci(f.footprintCi)})`,
          ...radii.filter((r) => r !== head).map((r) => pct(f.footprint[r])),
          `${f.area.flown.toFixed(2)} / ${f.area.footprint.toFixed(2)}`,
          `${sec(f.half.flown)} / ${sec(f.half.footprint)}`,
        ]);
      }
    }
  }

  // Findings: computed sentences, so the prose cannot drift from the JSON. Wording that depends on
  // the outcome (does a gap shrink, does an interval overlap) is chosen from the numbers too.
  const fp = (c) => c.footprintStats.footprint[head];
  const fpCi = (c) => c.footprintStats.footprintCi;
  const has = (...cs) => cs.every((c) => c?.footprintStats);
  const pts = (v) => `${(v * 100).toFixed(1)} points`;
  const findings = [];
  const kin = cell('hard', 'classical');
  const quadPlan = cell('quad', 'classical');
  const quadTrack = cell('quad', 'classicalBrakeTrack');
  if (has(kin, quadPlan)) {
    const gapFlown = kin.coverage - quadPlan.coverage;
    const gapFp = fp(kin) - fp(quadPlan);
    const share = gapFlown > 0 ? 1 - gapFp / gapFlown : 0;
    const verdict =
      share >= 0.5
        ? `Most of the flown-coverage gap (${Math.round(share * 100)}%) came from the metric rewarding lanes 0.5 m apart.`
        : share >= 0.2
          ? `About ${Math.round(share * 100)}% of the flown-coverage gap came from the metric; the rest is the quadrotor seeing less of the building.`
          : 'So the gap is not an artifact of the metric: the quadrotor sees less of the building, it does not just fly fewer lanes. The lane U-turn explains why it flies fewer voxels, not why it covers less.';
    findings.push(
      `- **The quadrotor's gap is real.** On the hard buildings the sensor-only planner flies through ${pct(kin.coverage)} on the kinematic drone and ${pct(quadPlan.coverage)} on the quadrotor, a gap of ${pts(gapFlown)}. Inside the ${head} m footprint the two cover ${pct(fp(kin))} and ${pct(fp(quadPlan))}, a gap of ${pts(gapFp)}. ${verdict}` +
        (has(quadTrack) ? ` With the braking filter and quad tracker the quadrotor reaches ${pct(fp(quadTrack))} (${ci(fpCi(quadTrack))}), at ${quadTrack.collisionsPerEpisode.toFixed(2)} collisions per episode.` : ''),
    );
  }
  const pred = cell('hard', 'classicalPredict');
  const gain = cell('hard', 'classicalOracleGain');
  if (has(kin, pred, gain)) {
    const k = kin.footprintStats;
    const p = pred.footprintStats;
    const clear = fpCi(pred)[0] > fpCi(kin)[1];
    findings.push(
      `- **Map prediction is no longer a null result.** By flown coverage the learned map prediction covers ${pct(pred.coverage)} against ${pct(kin.coverage)} for the planner without it, inside the noise. By footprint it covers ${pct(fp(pred))} (${ci(fpCi(pred))}) against ${pct(fp(kin))} (${ci(fpCi(kin))})${clear ? ', with the intervals apart' : ', with the intervals overlapping'}, and it has half the building in view after ${sec(p.half.footprint)} instead of ${sec(k.half.footprint)} (area under the curve ${p.area.footprint.toFixed(2)} against ${k.area.footprint.toFixed(2)}). The true hidden gain does about as well (${pct(fp(gain))}, half after ${sec(gain.footprintStats.half.footprint)}). Preferring frontiers that open onto unseen space pays off in what the drone sees, and soon, even though it does not change which voxels it flies.`,
    );
  }
  const hardCells = study.cells.filter((c) => c.condition === 'hard' && c.lighting === 'day' && c.footprintStats);
  if (hardCells.length) {
    const byFlown = [...hardCells].sort((a, b) => b.coverage - a.coverage).map((c) => c.pilot);
    const byFp = [...hardCells].sort((a, b) => fp(b) - fp(a)).map((c) => c.pilot);
    findings.push(
      `- **Ranking on the hard buildings, by day.** By flown coverage: ${byFlown.map((p) => LABELS[p] ?? p).join(' > ')}. By ${head} m footprint: ${byFp.map((p) => LABELS[p] ?? p).join(' > ')}.` +
        (() => {
          const topFlown = cell('hard', byFlown[0]);
          const topFp = cell('hard', byFp[0]);
          if (topFlown === topFp) return '';
          return ` ${LABELS[topFlown.pilot]} flies the most (${pct(topFlown.coverage)}) but ${(LABELS[topFp.pilot] ?? topFp.pilot).toLowerCase()} sees the most: ${pct(fp(topFp))} against ${pct(fp(topFlown))}, half the building in view after ${sec(topFp.footprintStats.half.footprint)} against ${sec(topFlown.footprintStats.half.footprint)}, with ${topFp.collisionsPerEpisode.toFixed(2)} collisions per episode against ${topFlown.collisionsPerEpisode.toFixed(2)}.`;
        })(),
    );
  }
  const quadCells = study.cells.filter((c) => c.condition === 'quad' && c.lighting === 'day' && c.footprintStats);
  const qHeur = cell('quad', 'heuristic');
  if (quadCells.length && has(qHeur)) {
    const order = [...quadCells].sort((a, b) => fp(b) - fp(a));
    const learned = quadCells.filter((c) => ['policy', 'guarded', 'guardedBrake'].includes(c.pilot));
    const beats = learned.filter((c) => fp(qHeur) > fp(c));
    findings.push(
      `- **On the quadrotor, by footprint:** ${order.map((c) => `${LABELS[c.pilot] ?? c.pilot} ${pct(fp(c))}`).join(', ')}. The leader has ${order[0].collisionsPerEpisode.toFixed(2)} collisions per episode${has(quadTrack) && order[0] !== quadTrack ? `; the planner with braking and the quad tracker has ${quadTrack.collisionsPerEpisode.toFixed(2)}` : ''}.` +
        (beats.length
          ? ` The lidar heuristic, which flies the fewest voxels (${pct(qHeur.coverage)}), sees more than ${beats.map((c) => (LABELS[c.pilot] ?? c.pilot).toLowerCase()).join(' and ')}, with ${qHeur.collisionsPerEpisode.toFixed(2)} collisions per episode: it keeps moving through open space, and the ring sees the rooms it passes.`
          : ''),
    );
  }
  const fl = cell('floors', 'classical');
  const climb = cell('floors', 'classicalClimb');
  if (has(fl, climb)) {
    findings.push(
      `- **On multi-floor buildings the footprint shows what climbing is for.** A pilot that stays on the ground floor sees little more than it flies (${pct(fl.coverage)} flown, ${pct(fp(fl))} footprint for the planner): the ring looks along one floor, and the other floors count in the denominator. The planner that takes shafts flies ${pct(climb.coverage)} but sees ${pct(fp(climb))} (${ci(fpCi(climb))}).`,
    );
  }
  if (has(kin)) {
    const f = kin.footprintStats;
    const easy = cell('easy', 'classical');
    findings.push(
      `- **Footprint coverage rises much faster than flown coverage.** The kinematic planner has half the hard building in view after ${sec(f.half.footprint)} and flown after ${sec(f.half.flown)}; the area under the curve is ${f.area.footprint.toFixed(2)} against ${f.area.flown.toFixed(2)}. At the lidar's 8 m range nearly everything is seen (${pct(f.footprint['8'] ?? 0)}), so that radius no longer separates pilots; ${head} m still does.` +
        (has(easy) ? ` One-room buildings saturate under any radius (${pct(fp(easy))}), so the easy condition says nothing here.` : ''),
    );
  }

  // Speed study: the quadrotor comparison at each cap, footprint beside flown.
  const speedRows = [];
  if (speed?.cells?.some((c) => c.footprintStats)) {
    for (const cond of speed.conditions) {
      for (const p of speed.pilots) {
        const cells = speed.speeds.map((v) => speed.cells.find((x) => x.condition === cond.id && x.pilot === p && x.speed === v));
        if (!cells[0]?.footprintStats) continue;
        speedRows.push([cond.id === 'quad' ? 'Quadrotor' : 'Kinematic', SPEED_LABELS[p] ?? p, ...cells.map((x) => `${pct(x.coverage)} / **${pct(x.footprintStats.footprint[head])}**`)]);
      }
    }
  }

  return {
    values: {
      FOOTPRINT_RADII: `${meta.radii.join(', ')} m`,
      FOOTPRINT_NOTE: `Same episodes as the coverage study (\`scripts/study/coverage-study.ts\` at \`${study.commit}\`), scored both ways. The footprint is a ${meta.rays}-ray horizontal ring of ground-truth rays at the drone's altitude, blocked by walls and furniture, with no noise and no lighting effect (\`src/eval/footprint.ts\`). Both metrics divide by the same reachable voxels. Brackets are bootstrap 95% intervals. *Area* is the normalized area under the coverage-over-time curve (1 would be everything covered at the first instant); *half* is the mean time to cover half the building, over the flights that did.`,
      FOOTPRINT_TABLE: table(
        ['Condition', 'Lighting', 'Pilot', 'Flown', `Footprint ${head} m (95% CI)`, ...radii.filter((r) => r !== head).map((r) => `Footprint ${r} m`), 'Area, flown / footprint', 'Half, flown / footprint'],
        rows,
      ),
      FOOTPRINT_FINDINGS: findings.join('\n'),
      FOOTPRINT_FIGURE: '![Mean coverage over time, footprint and flown, for five pilots on the kinematic drone and the quadrotor](report/coverage-time.svg)',
      FOOTPRINT_SPEED_TABLE: speedRows.length
        ? table(['Airframe', 'Pilot', ...speed.speeds.map((v) => `${v} m/s, flown / footprint`)], speedRows)
        : '_Run `npm run study:speed`._',
    },
    files: { 'docs/report/coverage-time.svg': figure(study) },
  };
}

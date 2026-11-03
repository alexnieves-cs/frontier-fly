// The indoor hazards section of docs/REPORT.md and docs/BENCHMARKS.md, from docs/hazard-coverage.json
// (the coverage study's hazard conditions) and docs/hazard-missions.json (scripts/study/hazard-study.ts).
// Pass conditions are read from the files' own records where they carry them, else from the constants
// fixed before the eval runs (scripts/study/coverage-study.ts HAZARD_PASS, src/eval/hazardBench.ts).

const pct = (v) => `${(v * 100).toFixed(1)}%`;
const ci = ([lo, hi]) => `${(lo * 100).toFixed(1)}–${(hi * 100).toFixed(1)}`;
const mark = (ok) => (ok ? '✅ meets' : '❌ misses');

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

const NOT_MEASURED = '_Not measured yet: run `node scripts/study/parallel.mjs coverage-study docs/hazard-coverage.json --conditions hazard-control,doors,glass,smoke` and `node scripts/study/parallel.mjs hazard-study docs/hazard-missions.json`._';

/** Fixed before the first eval flight (scripts/study/coverage-study.ts). */
const COVERAGE_PASS = { pilots: ['classical', 'guarded'], maxCrashes: 2, coverageRatio: 0.9 };
const HAZARDS = ['doors', 'glass', 'smoke'];
const PILOTS = [
  ['teacher', 'Oracle teacher (reads the true map)'],
  ['classical', 'Sensor-only planner'],
  ['policy', 'Policy alone'],
  ['guarded', 'Policy + lidar guard'],
];
const MODES = { sensor: 'Sensor planner, hazard-aware', policy: 'Policy, hazard-aware', sensorNaive: 'Sensor planner, unaware (control)' };

function coverageRows(study) {
  const cell = (condition, pilot) => study.cells.find((c) => c.condition === condition && c.pilot === pilot && c.lighting === 'day');
  const rows = [];
  const checks = [];
  for (const [pilot, label] of PILOTS) {
    const control = cell('hazard-control', pilot);
    if (!control) continue;
    const row = [label, `${pct(control.coverage)} (${control.crashes ?? '—'})`];
    for (const h of HAZARDS) {
      const c = cell(h, pilot);
      if (!c) {
        row.push('—');
        continue;
      }
      const ratio = c.coverage / control.coverage;
      row.push(`${pct(c.coverage)} (${c.crashes ?? '—'}), ×${ratio.toFixed(2)}`);
      if (COVERAGE_PASS.pilots.includes(pilot)) {
        checks.push({ pilot: label, hazard: h, ratio, crashes: c.crashes, met: ratio >= COVERAGE_PASS.coverageRatio && c.crashes <= COVERAGE_PASS.maxCrashes });
      }
    }
    rows.push(row);
  }
  return { rows, checks };
}

/** Plain-language reading of both benches, every number taken from the files. */
function note(coverage, missions) {
  const out = [];
  if (missions) {
    const cells = missions.cells;
    const sum = (f) => cells.reduce((a, c) => a + f(c), 0);
    const flights = sum((c) => c.flights);
    const naiveFc = sum((c) => (c.mode === 'sensorNaive' ? c.flightsWithFalseClear : 0));
    const doorFlights = sum((c) => (c.mode !== 'sensorNaive' ? c.flightsWithShutDoors : 0));
    out.push(
      `- **No false clear in ${flights} flights, but the control had ${naiveFc === 0 ? 'none either' : naiveFc}.** ${naiveFc === 0 ? 'So this bench does not show the hazard rules preventing a false clear; it shows they cost nothing where they did not bind.' : ''} Aware flights stopped short of a clear because of a shut door they could not search behind in ${doorFlights} flights. Every room of a mission shell also has its own outside opening, so a shut interior door slows the search but cannot hide anyone from a drone that goes round; a person truly sealed behind a door needs a building the generator does not make yet.`,
    );
    const cabin = cells.filter((c) => c.scenario === 'smoke-cabin');
    const sweep = cells.filter((c) => c.scenario === 'smoke-occupied');
    if (cabin.length && sweep.length) {
      const range = (xs) => (Math.min(...xs) === Math.max(...xs) ? `${Math.min(...xs)}` : `${Math.min(...xs)}–${Math.max(...xs)}`);
      out.push(
        `- **Smoke is the gap.** The hiker in a smoky cabin is found on ${range(cabin.map((c) => c.found))} of 30 flights by every explorer, aware or not, and occupied sweeps in smoke end incomplete on ${range(sweep.map((c) => c.outcomes.incomplete ?? 0))} of 30: the camera loses the person and the lidar's early returns shrink the free space the drone will fly into. Awareness keeps those flights honest (incomplete, not clear) but does not find the person; that needs a sensor that sees through smoke, such as thermal, which the drone does not carry.`,
      );
    }
  }
  if (coverage) {
    const cell = (condition, pilot) => coverage.cells.find((c) => c.condition === condition && c.pilot === pilot && c.lighting === 'day');
    const g = cell('hazard-control', 'guarded');
    const t = cell('doors', 'teacher');
    const tc = cell('hazard-control', 'teacher');
    if (g) out.push(`- **The crash bar was set below the guarded policy's own control.** It crashes on ${g.crashes} of 30 of these buildings with no hazards at all, so its hazard misses on crashes say more about the bar than about the hazards; its coverage stays within ${pct(Math.min(...HAZARDS.map((h) => cell(h, 'guarded')?.coverage ?? g.coverage)) / g.coverage)} of the control. The bar is kept as it was set.`);
    if (t && tc) out.push(`- **Thin ajar leaves catch the oracle teacher.** It reads the true map and crashes on ${t.crashes} of 30 door buildings against ${tc.crashes} without hazards; the ajar leaves are the only new obstacle there. The sensor-only planner, which sees them with its lidar, does not crash.`);
  }
  return out.join('\n');
}

export function hazardsSection(coverage, missions) {
  const values = { HAZARD_NOTE: note(coverage, missions) };
  if (!coverage) {
    values.HAZARD_COVERAGE_TABLE = NOT_MEASURED;
    values.HAZARD_COVERAGE_PASS = '';
  } else {
    const { rows, checks } = coverageRows(coverage);
    values.HAZARD_COVERAGE_TABLE = table(['Pilot', 'No hazards', 'Doors', 'Glass and mirrors', 'Smoke'], rows);
    const passRows = checks.map((c) => [c.pilot, c.hazard, `×${c.ratio.toFixed(2)} (≥ ${COVERAGE_PASS.coverageRatio})`, `${c.crashes} (≤ ${COVERAGE_PASS.maxCrashes})`, mark(c.met)]);
    values.HAZARD_COVERAGE_PASS = table(['Pilot', 'Hazard', 'Coverage vs its own control', 'Crashes of 30', 'Pass'], passRows);
    values.R_HZ_COV_COMMIT = coverage.commit;
  }
  if (!missions) {
    values.HAZARD_MISSION_TABLE = NOT_MEASURED;
    values.HAZARD_MISSION_PASS = '';
    values.HAZARD_MISSION_SEEDS = '';
  } else {
    const title = Object.fromEntries(missions.scenarios.map((s) => [s.id, s.title]));
    const rows = missions.cells.map((c) => [
      title[c.scenario] ?? c.scenario,
      MODES[c.mode] ?? c.mode,
      `${c.found} of ${c.flights} (${ci(c.foundCi)})`,
      pct(c.expectedOutcome),
      `${c.flightsWithFalseClear} (${ci(c.falseClearCi)})`,
      `${c.flightsWithShutDoors}`,
      `${c.flightsWithCollision}`,
    ]);
    values.HAZARD_MISSION_TABLE = table(['Scenario', 'Explorer', 'Person found (95% CI)', 'Ends as expected', 'Flights with a false clear (95% CI)', 'Shut door left unsearched', 'Flights with a collision'], rows);
    const p = missions.pass;
    const passRows = [
      ['No false clear with hazard awareness on', `${p.falseClears.value}`, `${p.falseClears.target}`, mark(p.falseClears.met)],
      ...p.found.map((f) => [`Person found: ${title[f.scenario] ?? f.scenario}, ${MODES[f.mode] ?? f.mode}`, pct(f.value), `≥ ${pct(f.target)}`, mark(f.met)]),
    ];
    values.HAZARD_MISSION_PASS = table(['Condition', 'Measured', 'Target', 'Pass'], passRows);
    values.HAZARD_MISSION_SEEDS = missions.scenarios.map((s) => `\`${s.id}\`: ${s.seeds[0]}–${s.seeds[s.seeds.length - 1]} (${s.seeds.length} seeds)`).join('; ');
    values.R_HZ_MIS_COMMIT = missions.commit;
  }
  return { values };
}

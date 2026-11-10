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
const MODES = { sensor: 'Sensor planner, hazard-aware', policy: 'Policy, hazard-aware', sensorNaive: 'Sensor planner, unaware (control)', sensorThermal: 'Sensor planner, hazard-aware, thermal camera', policyThermal: 'Policy, hazard-aware, thermal camera' };

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
function note(coverage, missions, thermal) {
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
        `- **Smoke is the gap.** The hiker in a smoky cabin is found on ${range(cabin.map((c) => c.found))} of 30 flights by every explorer, aware or not, and occupied sweeps in smoke end incomplete on ${range(sweep.map((c) => c.outcomes.incomplete ?? 0))} of 30: the camera loses the person and the lidar's early returns shrink the free space the drone will fly into. Awareness keeps those flights honest (incomplete, not clear) but does not find the person; that needs a sensor that sees through smoke${thermal ? ' (the thermal follow-up below)' : ', such as thermal, which the drone does not carry'}.`,
      );
    }
  }
  if (thermal) {
    const cell = (scenario, mode) => thermal.cells.find((c) => c.scenario === scenario && c.mode === mode);
    const pair = (scenario, mode) => {
      const off = cell(scenario, mode);
      const on = cell(scenario, `${mode}Thermal`);
      return off && on ? `${off.found} of ${off.flights} without it and ${on.found} of ${on.flights} with it (${ci(on.foundCi)})` : null;
    };
    const cabin = pair('smoke-cabin', 'sensor');
    const cabinPolicy = pair('smoke-cabin', 'policy');
    const sweep = pair('smoke-occupied', 'sensor');
    const sweepPolicy = pair('smoke-occupied', 'policy');
    const on = thermal.cells.filter((c) => c.mode.endsWith('Thermal'));
    const fc = on.reduce((a, c) => a + c.flightsWithFalseClear, 0);
    const fa = on.reduce((a, c) => a + c.falseAlarms, 0);
    const worst = Math.min(...on.filter((c) => c.scenario === 'smoke-cabin').map((c) => c.foundRate));
    const first = thermal.scenarios.find((x) => x.id === 'smoke-cabin')?.seeds ?? [];
    if (cabin && sweep) {
      out.push(
        `- **Thermal follow-up: a thermal camera closes most of the smoke gap.** The camera keeps 2% of its contrast at the smoke's visibility range, so it detects a person in a smoky room but never confirms one: finds without thermal are the share of people who were not in smoke. With an opt-in thermal camera beside it (\`src/perception/model.ts\` \`THERMAL\`), on a fresh block (${first[0]}–${first[first.length - 1]}, flown once, the same seeds with and without), the sensor planner finds the hiker in the smoky cabin on ${cabin}${cabinPolicy ? `, and the policy on ${cabinPolicy}` : ''}. In the occupied sweep with smoke the person is found on ${sweep}${sweepPolicy ? ` (policy: ${sweepPolicy})` : ''}. Thermal flights had ${fc} false clear${fc === 1 ? '' : 's'} and ${fa} false alarm${fa === 1 ? '' : 's'}, and the cabin find rate ${worst >= 0.8 ? 'meets' : 'misses'} the 80% bar set for the camera-only bench (${pct(worst)} at worst). The thermal camera is a model with three stated assumptions: only people are warm enough to stand out, smoke costs it a tenth of what it costs visible light, and its small sensor resolves a person at half the visible camera's range; glass is opaque to it. Nothing warm that is not a person is simulated, so its false-alarm count is optimistic. A contact in smoke is also re-viewed from closer first (2 m before 3.2 m), with or without thermal; the main table above was re-flown with that change. On the practice seeds the misses that remained were buildings the drone could not enter or explored little of, because smoke also shortens the lidar. A clear still rests on the visible camera only.`,
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

export function hazardsSection(coverage, missions, thermal) {
  const values = { HAZARD_NOTE: note(coverage, missions, thermal) };
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

/**
 * The multi-floor budget note of docs/REPORT.md, from docs/floors-study.json (scripts/study/floors-study.ts).
 */
export function floorsBudgetNote(study) {
  if (!study) return '';
  const cell = (cells, id) => cells.find((c) => c.budget === id);
  const bench = cell(study.cells, 'bench');
  const per = cell(study.cells, 'per-floor');
  if (!bench || !per) return '';
  const before = study.control ? cell(study.control.cells, 'per-floor') : null;
  const first = study.seedBase;
  return `- **Budget, not skill, sets the multi-floor number.** Coverage counts 0.5 m voxels flown through, so at 2 m/s no pilot can enter more than four new voxels a second. On ${bench.flights} fresh two and three floor buildings (${first}–${first + study.seeds - 1}, flown once) the 120 s episode lets any pilot fly at most ${pct(bench.ceiling)} of the building on average; the planner flies ${pct(bench.coverage)} (${ci(bench.coverageCi)}), which is ${pct(bench.coverage / bench.ceiling)} of that bound, and brings ${pct(bench.footprint)} within 3 m of its sensor. Given 120 s per floor it flies ${pct(per.coverage)} (${ci(per.coverageCi)}), reaches every floor in ${per.allFloorsReached} of ${per.flights} buildings and touches nothing${before ? `; before this round it flew ${pct(before.coverage)} (${ci(before.coverageCi)}) on the same seeds` : ''}. The gain comes from the climber: it used to look only one floor up and down and to write off a floor it left on its time share, and it now goes back to finish floors it left early, through as many shafts as it takes. FrontierBench's \`floors\` task keeps its 120 s budget so submitted scores stay comparable, which means about ${pct(bench.ceiling)} is the most that cell can show.`;
}

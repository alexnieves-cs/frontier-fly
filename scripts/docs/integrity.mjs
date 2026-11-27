// docs/INTEGRITY.md: what flying on a pose estimate costs (the drift tracks, each against the same
// flights on the true pose) and which pilots know when their estimate has gone bad (the integrity
// track and the monitor study, scripts/bench/integrity-study.ts). Every number is read from
// bench/results/hard-drift*, bench/results/hard-integrity, docs/integrity-dev.json and docs/integrity.json.
import { gapPoints, pairedScoreGap, readResults } from './bench.mjs';

// A typographic minus for the integrity track, the one score that can go below zero.
const pct = (v) => `${(v * 100).toFixed(1)}%`.replace(/^-/, '−');
const pts = (v) => `${v >= 0 ? '+' : '−'}${Math.abs(v * 100).toFixed(1)}`;
const mean = (xs) => (xs.length ? xs.reduce((a, b) => a + b, 0) / xs.length : null);

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

/** A paired loss for prose: "29.0 points (95% interval 22.7 to 35.5)". */
const lossText = (g) => (g ? `${(-g.gap * 100).toFixed(1)} points (95% interval ${(-g.ci[1] * 100).toFixed(1)} to ${(-g.ci[0] * 100).toFixed(1)})` : '—');
const includesZero = (g) => Boolean(g) && g.ci[0] <= 0 && g.ci[1] >= 0;
const gapText = (g) => (g ? `${pts(g.gap)} (${pts(g.ci[0])} to ${pts(g.ci[1])})${g.ci[0] <= 0 && g.ci[1] >= 0 ? ', tied' : ''}` : '—');

/** Test rows of one Hard results folder at its newest version, by pilot name. */
function testRows(root, dir) {
  const rows = readResults(root, dir).filter((r) => r.split === 'test' && r.cameraScore !== undefined);
  const v = Math.max(0, ...rows.map((r) => r.hard ?? 0));
  return rows.filter((r) => (r.hard ?? 0) === v);
}

/** Each pilot on its own estimate against the same seeds on the true pose. */
function driftCost(root, dir, truthDir) {
  const truth = testRows(root, truthDir);
  return testRows(root, dir)
    .map((r) => {
      const t = truth.find((x) => x.name === r.name);
      if (!t) return null;
      const eps = r.episodes ?? [];
      const home = (x) => {
        const e = (x.episodes ?? []).filter((y) => y.returned !== undefined);
        return e.length ? { home: e.filter((y) => y.returned).length, of: e.length } : null;
      };
      return {
        name: r.name,
        baseline: r.baseline,
        est: r.cameraScore,
        truth: t.cameraScore,
        gap: pairedScoreGap(r, t, 4000, (e) => e.camera.footprint),
        poseErr: mean(eps.filter((e) => e.drift).map((e) => e.drift.mean)),
        worstErr: Math.max(0, ...eps.filter((e) => e.drift).map((e) => e.drift.max)),
        homeEst: home(r),
        homeTruth: home(t),
        flights: eps.length,
        commit: r.commit,
      };
    })
    .filter(Boolean)
    .sort((a, b) => b.est - a.est);
}

const ordinal = (n) => `${n}${n % 100 >= 11 && n % 100 <= 13 ? 'th' : ['th', 'st', 'nd', 'rd'][n % 10] ?? 'th'}`;
const homeCell = (h) => (h ? `${h.home} of ${h.of}` : '—');

function driftTable(rows) {
  return table(
    ['Pilot', 'On its estimate', 'Same flights, true pose', 'What the estimate costs (95% CI)', 'Home on the estimate', 'Home on the truth', 'Mean pose error', 'Worst pose error'],
    rows.map((r) => [r.name, `**${pct(r.est)}**`, pct(r.truth), gapText(r.gap), homeCell(r.homeEst), homeCell(r.homeTruth), r.poseErr === null ? '—' : `${r.poseErr.toFixed(2)} m`, `${r.worstErr.toFixed(1)} m`]),
  );
}

function pilotTable(rows) {
  return table(
    ['Pilot', 'Integrity score', 'Reports', 'Available', 'Hazardously misleading', 'Bad-pose steps', 'Bad pose flagged', 'Camera', 'Flights ended by a crash'],
    rows.map((r) => [r.name, `**${pct(r.score)}**`, r.reports, pct(r.availability), `${(r.hmiRate * 100).toFixed(2)}%`, pct(r.badShare), r.caught === null ? '—' : pct(r.caught), r.camera === undefined ? '—' : pct(r.camera), `${r.crashes} of ${r.flights}`]),
  );
}

function integrityPilots(root) {
  const rows = readResults(root, 'bench/results/hard-integrity').filter((r) => r.split === 'test' && r.integrityScore !== undefined);
  const v = Math.max(0, ...rows.map((r) => r.hard ?? 0));
  return rows
    .filter((r) => (r.hard ?? 0) === v)
    .map((r) => {
      const eps = r.episodes ?? [];
      const col = (f) => mean(eps.map((e) => e.integrity[f]));
      // Bad pose flagged: the mean over tasks of each task's share, as the score is (a task with no bad step has none).
      const caught = mean(Object.values(r.tasks).filter((t) => t.caught !== null && t.caught !== undefined).map((t) => t.caught));
      const availability = col('availability');
      const misleadShare = col('misleadShare');
      return {
        name: r.name,
        score: r.integrityScore,
        availability,
        hmiRate: col('hmiRate'),
        badShare: col('badShare'),
        caught,
        // A claimed level of 0 is under the true error on every step (a crash ends the flight, not the claim).
        reports: misleadShare > 0.99 ? 'Nothing (claims 0)' : 'A protection level',
        camera: r.cameraScore,
        failCrashes: r.tasks['pl-failing-quad']?.crashedFlights,
        crashes: eps.filter((e) => e.crashed).length,
        flights: eps.length,
        result: r,
      };
    })
    .sort((a, b) => b.score - a.score);
}

const MONITOR_ORDER = ['always', 'never', 'sigma-1', 'sigma-2', 'sigma-3', 'sigma-5', 'sigma-8', 'sigma-health-1', 'sigma-health-2', 'sigma-health-3', 'sigma-health-5', 'sigma-health-8'];

function monitorTable(dev, test, houseId) {
  const by = (s, id) => s?.rows.find((r) => r.id === id);
  const ids = MONITOR_ORDER.filter((id) => by(dev, id) || by(test, id));
  const cell = (r) => (r ? pct(r.score) : '—');
  return table(
    ['Monitor', 'Dev score', 'Test score', 'Test: available', 'Test: hazardously misleading', 'Test: bad pose flagged'],
    ids.map((id) => {
      const d = by(dev, id);
      const t = by(test, id);
      const label = (d ?? t).label;
      const name = id === houseId ? `**${label}** (house pick)` : label;
      return [name, id === houseId ? `**${cell(d)}**` : cell(d), id === houseId ? `**${cell(t)}**` : cell(t), t ? pct(t.availability) : '—', t ? `${(t.hmiRate * 100).toFixed(2)}%` : '—', t?.caught === null || !t ? '—' : pct(t.caught)];
    }),
  );
}

/** Per task on one study's flights: how often the pose went bad, how far, and which monitors flagged it. */
function taskTable(s, houseId, labels) {
  const tasks = s.rows[0].tasks.map((t) => t.task);
  const caught = (id, task) => s.rows.find((r) => r.id === id)?.tasks.find((t) => t.task === task)?.caught;
  const cell = (v) => (v === null || v === undefined ? '—' : pct(v));
  return table(
    ['Task', 'Bad-pose steps', 'Flights over 1.5 m', 'Worst pose error', 'Median 1σ, pose bad', 'Median 1σ, pose good', 'Estimator says tracking, pose bad', ...labels.map(([, l]) => `Bad pose flagged: ${l}`)],
    tasks.map((task) => {
      const fs = s.flights.filter((f) => f.task === task);
      const c = s.calibration?.find((x) => x.task === task);
      const m = (v) => (v === null || v === undefined ? '—' : `${v.toFixed(2)} m`);
      return [
        `\`${task}\``,
        pct(mean(fs.map((f) => f.monitors.always.badShare))),
        `${fs.filter((f) => f.maxErr > s.rules.alertLimit).length} of ${fs.length}`,
        `${Math.max(...fs.map((f) => f.maxErr)).toFixed(1)} m`,
        m(c?.sigmaOnBad),
        m(c?.sigmaOnGood),
        c?.trackingOnBad === null || c?.trackingOnBad === undefined ? '—' : pct(c.trackingOnBad),
        ...labels.map(([id]) => (id === houseId ? `**${cell(caught(id, task))}**` : cell(caught(id, task)))),
      ];
    }),
  );
}

/** One monitor's per-flight scores in a study, shaped like a result for pairedScoreGap. */
const asEpisodes = (s, id) => ({ episodes: s.flights.map((f) => ({ task: f.task, seed: f.seed, coverage: f.monitors[id].score })) });

/** The house monitor against claiming nothing, on the same test flights. */
function monitorGain(test, houseId) {
  if (!test) return { INT_MONITOR_GAIN: '—', INT_MONITOR_GAIN_NOTE: '' };
  const g = pairedScoreGap(asEpisodes(test, houseId), asEpisodes(test, 'always'), 4000, (e) => e.coverage);
  return { INT_MONITOR_GAIN: gapPoints(g), INT_MONITOR_GAIN_NOTE: includesZero(g) ? ', an interval that includes zero' : '' };
}

/** Inline numbers for the 1σ finding, from the test study's calibration rows. */
function calibrationValues(s) {
  const c = (task) => s?.calibration?.find((x) => x.task === task);
  const m = (v) => (v === null || v === undefined ? '—' : `${v.toFixed(2)} m`);
  const worst = (task) => (s ? `${Math.max(...s.flights.filter((f) => f.task === task).map((f) => f.maxErr)).toFixed(0)} m` : '—');
  const caught = (id, task) => s?.rows.find((r) => r.id === id)?.tasks.find((t) => t.task === task)?.caught;
  return {
    INT_LARGE_BAD: m(c('pl-tof-large-quad')?.sigmaOnBad),
    INT_LARGE_GOOD: m(c('pl-tof-large-quad')?.sigmaOnGood),
    INT_FAIL_BAD: m(c('pl-failing-quad')?.sigmaOnBad),
    INT_FAIL_GOOD: m(c('pl-failing-quad')?.sigmaOnGood),
    INT_FAIL_TRACKING: c('pl-failing-quad')?.trackingOnBad == null ? '—' : pct(c('pl-failing-quad').trackingOnBad),
    INT_FAIL_WORST: worst('pl-failing-quad'),
    INT_FAIL_CAUGHT8: caught('sigma-8', 'pl-failing-quad') == null ? '—' : pct(caught('sigma-8', 'pl-failing-quad')),
    INT_FAIL_CAUGHT1: caught('sigma-1', 'pl-failing-quad') == null ? '—' : pct(caught('sigma-1', 'pl-failing-quad')),
    INT_LARGE_AVAIL: s?.rows.find((r) => r.id === 'sigma-1')?.tasks.find((t) => t.task === 'pl-tof-large-quad')?.availability == null ? '—' : pct(s.rows.find((r) => r.id === 'sigma-1').tasks.find((t) => t.task === 'pl-tof-large-quad').availability),
  };
}

const seedsText = (s) => (s ? `${s.split} seeds ${s.seeds.base}+, ${s.flights.length / Math.max(1, s.rows[0]?.tasks.length ?? 1)} per task (${s.flights.length} flights)` : '—');

/**
 * Values for scripts/docs/INTEGRITY.template.md. `house` is HOUSE_MONITOR from src/bench/integrity.ts,
 * passed in because the docs build reads no TypeScript.
 */
export function integritySection(root, dev, test, house) {
  const drift = driftCost(root, 'bench/results/hard-drift', 'bench/results/hard-drift-truth');
  const driftHard = driftCost(root, 'bench/results/hard-drift-hard', 'bench/results/hard-drift-hard-truth');
  const pilots = integrityPilots(root);
  const houseId = `${house.id}-${house.k}`;
  const by = (s, id) => s?.rows.find((r) => r.id === id);
  const view = (rows) => rows.find((r) => r.name === 'Sensor-only planner, view first');
  const v = view(driftHard);
  const pv = pilots.find((p) => p.name === 'Sensor-only planner, view first');
  const others = pilots.filter((p) => p !== pv);
  const silent = others.filter((p) => p.reports === 'Nothing (claims 0)');
  const silentBest = silent[0];
  const silentGap = pv && silentBest ? pairedScoreGap(pv.result, silentBest.result, 4000, (e) => e.integrity.score) : null;
  const n = (x, f) => (x === null || x === undefined ? '—' : f(x));
  return {
    INT_DRIFT: drift.length ? driftTable(drift) : '_Run `npm run bench:hard -- --track drift --split test` and again with `--truth`._',
    INT_DRIFT_HARD: driftHard.length ? driftTable(driftHard) : '_Run `npm run bench:hard -- --track drift-hard --split test` and again with `--truth`._',
    INT_PILOTS: pilots.length ? pilotTable(pilots) : '_Run `npm run bench:hard -- --track integrity --split test`._',
    INT_MONITORS: dev || test ? monitorTable(dev, test, houseId) : '_Run `npm run bench:integrity -- --split dev`._',
    INT_TASKS: test ? taskTable(test, houseId, [[houseId, '1σ (house)'], ['sigma-8', '8 × 1σ'], ['sigma-health-1', '1σ, ∞ once degraded or lost']]) : '_Run `npm run bench:integrity -- --split test --out docs/integrity.json`._',
    INT_DEV_SEEDS: seedsText(dev),
    INT_TEST_SEEDS: seedsText(test),
    INT_HOUSE_LABEL: by(dev ?? test, houseId)?.label ?? houseId,
    INT_HOUSE_DEV: n(by(dev, houseId)?.score, pct),
    INT_HOUSE_TEST: n(by(test, houseId)?.score, pct),
    INT_ALWAYS_DEV: n(by(dev, 'always')?.score, pct),
    INT_ALWAYS_TEST: n(by(test, 'always')?.score, pct),
    INT_NEVER_TEST: n(by(test, 'never')?.score, pct),
    INT_HOUSE_CAUGHT_TEST: n(by(test, houseId)?.caught, pct),
    INT_HOUSE_HMI_TEST: n(by(test, houseId)?.hmiRate, (x) => `${(x * 100).toFixed(2)}%`),
    INT_BAD_TEST: n(test?.badShare, pct),
    INT_BAD_FLIGHTS_TEST: test ? `${test.flights.filter((f) => f.maxErr > test.rules.alertLimit).length} of ${test.flights.length}` : '—',
    INT_WORST_TEST: test ? `${Math.max(...test.flights.map((f) => f.maxErr)).toFixed(1)} m` : '—',
    INT_PV_SCORE: n(pv?.score, pct),
    INT_PV_CRASHES: pv ? `${pv.crashes} of ${pv.flights}` : '—',
    INT_SILENT_BEST: silentBest ? `${silentBest.name}, at ${pct(silentBest.score)}` : '—',
    INT_SILENT_GAP: gapPoints(silentGap),
    INT_HARD_COST: v ? lossText(v.gap) : '—',
    INT_HARD_EST: v ? pct(v.est) : '—',
    INT_HARD_TRUTH: v ? pct(v.truth) : '—',
    INT_HARD_HOME: v ? `${homeCell(v.homeEst)} on its estimate against ${homeCell(v.homeTruth)} on the truth` : '—',
    INT_PV_RANK: pv ? `${ordinal(pilots.indexOf(pv) + 1)} of ${pilots.length}` : '—',
    INT_PV_BAD: pv ? pct(pv.badShare) : '—',
    INT_PV_CAMERA: pv?.camera === undefined ? '—' : pct(pv.camera),
    INT_SILENT_BAD: silentBest ? pct(silentBest.badShare) : '—',
    INT_SILENT_CAMERA: silentBest?.camera === undefined ? '—' : pct(silentBest.camera),
    INT_PV_FAIL_CRASHES: pv?.failCrashes === undefined ? '—' : `${pv.failCrashes} of 30`,
    INT_OTHERS_FAIL_CRASHES: others.length ? `${Math.min(...others.map((p) => p.failCrashes ?? 0))} to ${Math.max(...others.map((p) => p.failCrashes ?? 0))} of 30` : '—',
    ...monitorGain(test, houseId),
    INT_COMMIT: test?.commit ?? dev?.commit ?? '—',
    ...calibrationValues(test),
  };
}

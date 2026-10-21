#!/usr/bin/env node
// Regenerates docs/ARCHITECTURE.md, docs/BENCHMARKS.md, and the benchmark block in README.md.
//
//   node scripts/docs/generate.mjs           write the files
//   node scripts/docs/generate.mjs --check   exit 1 if any file is out of date
//
// Inputs: the import graph under src/, docs/benchmarks.json (npm run bench),
// docs/media/media.json (npm run media), and docs/missions.json when the mission runner writes one.
// Numbers only come from those files. A missing file renders as "not measured".
import { existsSync, readFileSync, readdirSync, statSync, writeFileSync } from 'node:fs';
import { dirname, join, relative, resolve } from 'node:path';
import { sarSection } from './sar.mjs';
import { speedSection } from './speed.mjs';
import { benchSection } from './bench.mjs';

const ROOT = resolve(dirname(new URL(import.meta.url).pathname), '../..');
const CHECK = process.argv.includes('--check');
const TARGETS = { easy: 0.8, hard: 0.6, nightRatio: 0.9, envStepsPerSec: 10_000, stepMs: 0.3 };

const read = (p) => readFileSync(join(ROOT, p), 'utf8');
const readJson = (p) => (existsSync(join(ROOT, p)) ? JSON.parse(read(p)) : null);
const pct = (v) => `${(v * 100).toFixed(1)}%`;
const mean = (xs) => (xs.length ? xs.reduce((s, v) => s + v, 0) / xs.length : NaN);
const fmt = (v, digits = 0) => Number(v).toLocaleString('en-US', { maximumFractionDigits: digits, minimumFractionDigits: digits });
const mark = (ok) => (ok ? '✅ meets' : '❌ misses');

function table(head, rows) {
  const line = (cells) => `| ${cells.join(' | ')} |`;
  return [line(head), line(head.map(() => '---')), ...rows.map(line)].join('\n');
}

// ---------------------------------------------------------------------------------------------
// Architecture: module graph from relative imports.

function walk(dir) {
  return readdirSync(dir).flatMap((name) => {
    const p = join(dir, name);
    return statSync(p).isDirectory() ? walk(p) : /\.ts$/.test(name) && !name.endsWith('.d.ts') ? [p] : [];
  });
}

/** A module is the first directory under src/, except world/outdoor, ui/mission, and their viz/ split out. */
function moduleOf(file) {
  const rel = relative(join(ROOT, 'src'), file).split('/');
  if (rel.length === 1) return 'app';
  if (rel[0] === 'world' && rel[1] === 'outdoor') return rel[2] === 'viz' && rel.length > 3 ? 'world/outdoor/viz' : 'world/outdoor';
  if (rel[0] === 'ui' && rel[1] === 'mission' && rel.length > 2) return 'ui/mission';
  return rel[0];
}

const EXTERNAL = {
  three: 'Three.js',
  '@dimforge/rapier3d-compat': 'Rapier',
  '@tensorflow/tfjs': 'TF.js',
  uplot: 'uPlot',
};

function architecture() {
  const files = walk(join(ROOT, 'src')).sort();
  const modules = new Map();
  const edges = new Map();
  for (const file of files) {
    const mod = moduleOf(file);
    const info = modules.get(mod) ?? { files: 0, lines: 0, external: new Set(), workers: [] };
    const text = readFileSync(file, 'utf8');
    info.files += 1;
    info.lines += text.split('\n').length;
    if (/\.worker\.ts$/.test(file)) info.workers.push(relative(ROOT, file));
    for (const match of text.matchAll(/(?:import|export)[^'"]*?from\s+['"]([^'"]+)['"]|import\(\s*['"]([^'"]+)['"]\s*\)|new URL\(\s*['"]([^'"]+\.ts)['"]/g)) {
      const spec = match[1] ?? match[2] ?? match[3];
      if (spec.startsWith('.')) {
        const target = resolve(dirname(file), spec);
        // Data imported from outside src/ (the landing page reads docs/benchmarks.json) is not a module.
        if (relative(join(ROOT, 'src'), target).startsWith('..')) continue;
        const to = moduleOf(target.endsWith('.ts') ? target : `${target}.ts`);
        if (to !== mod) {
          const key = `${mod}→${to}`;
          edges.set(key, (edges.get(key) ?? 0) + 1);
        }
      } else {
        const pkg = Object.keys(EXTERNAL).find((name) => spec === name || spec.startsWith(`${name}/`));
        if (pkg) info.external.add(EXTERNAL[pkg]);
      }
    }
    modules.set(mod, info);
  }

  const id = (m) => m.replace(/\//g, '_');
  const names = [...modules.keys()].sort();
  const graph = [
    '```mermaid',
    'flowchart LR',
    ...names.map((m) => `  ${id(m)}["${m}"]`),
    ...[...edges.entries()].sort().map(([key, n]) => {
      const [from, to] = key.split('→');
      return `  ${id(from)} -->|${n}| ${id(to)}`;
    }),
    '```',
  ].join('\n');

  const moduleTable = table(
    ['Module', 'Files', 'Lines', 'Imports from', 'Libraries', 'Workers'],
    names.map((m) => {
      const info = modules.get(m);
      const deps = [...edges.keys()].filter((k) => k.startsWith(`${m}→`)).map((k) => `\`${k.split('→')[1]}\``);
      return [
        `\`src/${m === 'app' ? '' : m}\``,
        String(info.files),
        fmt(info.lines),
        deps.sort().join(', ') || '—',
        [...info.external].sort().join(', ') || '—',
        info.workers.map((w) => `\`${w}\``).join(', ') || '—',
      ];
    }),
  );

  const headless = ['gen', 'sim', 'rl', 'shared', 'world/outdoor', 'eval', 'mission', 'perception'].filter((m) => modules.has(m));
  const headlessRows = headless.map((m) => {
    const info = modules.get(m);
    const three = info.external.has('Three.js');
    const viaViz = [...edges.keys()].some((k) => k === `${m}→viz`);
    return [`\`src/${m}\``, three || viaViz ? '❌ imports rendering code' : '✅ no Three.js'];
  });

  const pages = readdirSync(ROOT)
    .filter((n) => n.endsWith('.html'))
    .sort()
    .map((html) => {
      const src = /<script[^>]+type="module"[^>]+src="\/?([^"]+)"/.exec(read(html))?.[1] ?? '?';
      return [`\`${html}\``, `\`${src}\``, `\`${moduleOf(join(ROOT, src))}\``];
    });

  return {
    MODULE_GRAPH: graph,
    MODULE_TABLE: moduleTable,
    HEADLESS_TABLE: table(['Module', 'Rendering imports'], headlessRows),
    ENTRY_POINTS: table(['Page', 'Entry', 'Module'], pages),
    FILE_COUNT: String(files.length),
    LINE_COUNT: fmt([...modules.values()].reduce((s, m) => s + m.lines, 0)),
  };
}

// ---------------------------------------------------------------------------------------------
// Benchmarks.

function coverageSummary(rows, stage) {
  const of = (lighting) => rows.filter((r) => r.stage === stage && r.lighting === lighting).map((r) => r.coverage);
  const day = of('day');
  const night = of('night-dark');
  return { n: day.length, day: mean(day), night: mean(night), ratio: mean(night) / mean(day), min: Math.min(...day), max: Math.max(...day) };
}

const PILOT_LABELS = {
  teacher: 'Oracle teacher (floor plan)',
  classical: 'Sensor-only planner',
  classicalBrake: 'Sensor-only planner, braking filter',
  classicalClimb: 'Sensor-only planner, takes shafts',
  classicalPredict: 'Sensor-only planner + learned map prediction',
  classicalOracleGain: 'Sensor-only planner + true hidden gain (reference)',
  policy: 'Policy',
  guarded: 'Policy + lidar guard',
  guardedBrake: 'Policy + braking guard',
  waypoint: 'Waypoint follower',
  heuristic: 'Lidar heuristic',
};

/** Coverage study (scripts/study/coverage-study.ts): every pilot on 30 fresh buildings per condition. */
function studyTables(study) {
  if (!study) {
    const none = '_Not measured yet. Run `npm run study:coverage`._';
    return { STUDY_NOTE: none, STUDY_HEADLINE: none, STUDY_TABLE: none };
  }
  const ci = ([lo, hi]) => `${(lo * 100).toFixed(1)}–${(hi * 100).toFixed(1)}`;
  const cell = (condition, lighting, pilot) => study.cells.find((c) => c.condition === condition && c.lighting === lighting && c.pilot === pilot);
  const row = (c) => [
    `${pct(c.coverage)} (${ci(c.coverageCi)})`,
    c.collisionsPerEpisode.toFixed(2),
    `${c.episodesWithCollision}/${c.episodes}`,
  ];
  const headline = [];
  for (const pilot of ['teacher', 'classical', 'policy', 'guarded']) {
    const hard = cell('hard', 'day', pilot);
    const night = cell('hard', 'night-dark', pilot);
    const floors = cell('floors', 'day', pilot);
    const quad = cell('quad', 'day', pilot);
    headline.push([
      PILOT_LABELS[pilot],
      `${pct(hard.coverage)} (${ci(hard.coverageCi)})`,
      `${pct(night.coverage)} (${ci(night.coverageCi)})`,
      hard.collisionsPerEpisode.toFixed(2),
      `${pct(floors.coverage)} (${ci(floors.coverageCi)})`,
      `${pct(quad.coverage)} (${ci(quad.coverageCi)})`,
      quad.collisionsPerEpisode.toFixed(1),
    ]);
  }
  const rows = [];
  for (const condition of study.conditions) {
    for (const lighting of condition.lighting) {
      for (const pilot of study.pilots) {
        const c = cell(condition.id, lighting, pilot);
        if (c) rows.push([condition.label, lighting, PILOT_LABELS[pilot] ?? pilot, ...row(c)]);
      }
    }
  }
  return {
    STUDY_NOTE: [
      `Every pilot flies the same ${study.seedsPerCondition} fresh buildings per condition, from \`scripts/study/coverage-study.ts\` at \`${study.commit}\` with \`${study.checkpoint}\`.`,
      'The seeds (20001+, 30001+, 40001+, 50001+) are drawn nowhere else: not by training, checkpoint selection (9001–9166), or the tuning of the sensor planner and guard (9101–9108).',
      'Brackets are bootstrap 95% intervals of the mean. Easy buildings end at 95% coverage, so that row saturates.',
    ].join(' '),
    STUDY_HEADLINE: table(
      ['Pilot', 'Hard, day', 'Hard, night (no lights)', 'Hard collisions per episode', 'Two floors', 'Quadrotor + noise', 'Quad collisions per episode'],
      headline,
    ),
    STUDY_TABLE: table(['Condition', 'Lighting', 'Pilot', 'Coverage (95% CI)', 'Collisions per episode', 'Episodes with a collision'], rows),
  };
}

/** Ablations (scripts/study/ablations.ts) and the learning-curve figure drawn from their logs. */
function ablationTables(ab) {
  if (!ab) {
    const none = '_Not measured yet. Run the ablation recipe in scripts/study/ablations.ts._';
    return { values: { ABLATION_TABLE: none, ABLATION_NOTE: none, CURVES_FIGURE: none }, files: {} };
  }
  const ci = ([lo, hi]) => `${(lo * 100).toFixed(1)}–${(hi * 100).toFixed(1)}`;
  const c = (x) => `${pct(x.coverage)} (${ci(x.coverageCi)})`;
  const rows = ab.arms.map((a) => [
    a.label,
    c(a.hard.policyDay),
    c(a.hard.policyNight),
    a.hard.policyDay.collisionsPerEpisode.toFixed(2),
    c(a.hard.guardedDay),
    a.curve.length ? pct(Math.max(...a.curve.map((r) => r.valScore))) : a.ppo ? `${pct(a.ppo.val)} (DAgger ${pct(a.ppo.dagger)})` : '—',
  ]);
  // Learning curves: validation score per round for every arm that has a curve.
  const curved = ab.arms.filter((a) => a.curve.length);
  const W = 640;
  const H = 300;
  const pad = { l: 48, r: 170, t: 16, b: 36 };
  const rounds = Math.max(1, ...curved.map((a) => a.curve.length - 1));
  const X = (r) => pad.l + (r / rounds) * (W - pad.l - pad.r);
  const Y = (v) => pad.t + (1 - v) * (H - pad.t - pad.b);
  const colors = ['#1f6fd1', '#d93a3a', '#2a9d55', '#d08a3a', '#7a4fc9'];
  const parts = [`<rect width="${W}" height="${H}" fill="#fff"/>`];
  for (let v = 0; v <= 1.0001; v += 0.2) {
    parts.push(`<line x1="${pad.l}" x2="${W - pad.r}" y1="${Y(v)}" y2="${Y(v)}" stroke="#e3e5e8"/>`);
    parts.push(`<text x="${pad.l - 6}" y="${Y(v) + 4}" font-size="11" text-anchor="end" fill="#555" font-family="sans-serif">${Math.round(v * 100)}%</text>`);
  }
  for (let r = 0; r <= rounds; r++) {
    parts.push(`<text x="${X(r)}" y="${H - pad.b + 16}" font-size="11" text-anchor="middle" fill="#555" font-family="sans-serif">${r === 0 ? 'BC' : r}</text>`);
  }
  parts.push(`<text x="${(pad.l + W - pad.r) / 2}" y="${H - 4}" font-size="11" text-anchor="middle" fill="#555" font-family="sans-serif">DAgger round (validation score, held-out seeds)</text>`);
  curved.forEach((a, i) => {
    const col = colors[i % colors.length];
    parts.push(`<polyline fill="none" stroke="${col}" stroke-width="2" points="${a.curve.map((r) => `${X(r.round).toFixed(1)},${Y(r.valScore).toFixed(1)}`).join(' ')}"/>`);
    for (const r of a.curve) parts.push(`<circle cx="${X(r.round).toFixed(1)}" cy="${Y(r.valScore).toFixed(1)}" r="3" fill="${col}"/>`);
    parts.push(`<rect x="${W - pad.r + 12}" y="${pad.t + i * 20}" width="12" height="3" fill="${col}"/>`);
    parts.push(`<text x="${W - pad.r + 30}" y="${pad.t + i * 20 + 5}" font-size="11" fill="#333" font-family="sans-serif">${a.id === 'dagger' ? 'full observation' : a.id}</text>`);
  });
  const svg = `<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 ${W} ${H}" width="${W}" height="${H}">${parts.join('')}</svg>\n`;
  return {
    values: {
      ABLATION_NOTE: `Each arm is one \`npm run train:indoor\` run with the default recipe (${ab.recipe}), the checkpoint kept by held-out validation, then flown on the coverage study's 30 hard buildings, from \`scripts/study/ablations.ts\` at \`${ab.commit}\`. Observation ablations zero the block in training and evaluation alike. Brackets are bootstrap 95% intervals.`,
      ABLATION_TABLE: table(['Arm', 'Hard, day', 'Hard, night', 'Collisions per episode', 'Under the lidar guard, day', 'Best validation'], rows),
      CURVES_FIGURE: '![Validation score per DAgger round for each arm](report/learning-curves.svg)',
    },
    files: { 'docs/report/learning-curves.svg': svg },
  };
}

/** Numbers the report's prose quotes, so the prose cannot drift from the JSON. */
function reportInline(study, bench) {
  const out = {};
  const keys = ['R_TEACHER_HARD', 'R_PLANNER_HARD', 'R_POLICY_HARD', 'R_GUARD_HARD', 'R_POLICY_COLL', 'R_GUARD_COLL', 'R_PLANNER_QUAD', 'R_POLICY_QUAD', 'R_TEACHER_QUAD'];
  const keys3 = ['R_Q_PLANNER', 'R_Q_PLANNER_COLL', 'R_Q_PLANNER_BRAKE', 'R_Q_PLANNER_BRAKE_COLL', 'R_Q_GUARD', 'R_Q_GUARD_COLL', 'R_Q_GUARD_BRAKE', 'R_Q_GUARD_BRAKE_COLL'];
  const keys4 = ['R_MQ_SENSOR', 'R_MQ_POLICY', 'R_MQ_COLL', 'R_MQ_ALL'];
  const keys5 = ['R_CLIMB_FLOORS', 'R_CLIMB_FLOORS_CI', 'R_PLANNER_FLOORS', 'R_PLANNER_FLOORS_CI', 'R_CLIMB_REACHED', 'R_CLIMB_COLL', 'R_PRED_HARD', 'R_PRED_HARD_CI', 'R_ORACLEGAIN_HARD', 'R_PRED_FLOORS', 'R_ORACLEGAIN_FLOORS', 'R_PLANNER_FLOORS_N'];
  const keys6 = ['R_MP_R2', 'R_MP_BASE_R2', 'R_MP_MAE', 'R_MP_BASE_MAE', 'R_MP_TRAIN', 'R_MP_VAL'];
  const keys2 = ['R_M_SENSOR', 'R_M_ORACLE', 'R_M_POLICY', 'R_M_FLIGHTS', 'R_M_COLL', 'R_M_HANDOFF', 'R_M_NIGHT_SENSOR', 'R_M_NIGHT_ORACLE', 'R_M_COMPOUND', 'R_M_ALL'];
  for (const k of [...keys, ...keys2, ...keys3, ...keys4, ...keys5, ...keys6]) out[k] = 'n/a';
  const ci = ([lo, hi]) => `${(lo * 100).toFixed(1)}–${(hi * 100).toFixed(1)}%`;
  const mp = readJson('public/models/mappred.json');
  if (mp?.meta) {
    out.R_MP_R2 = mp.meta.valR2.toFixed(2);
    out.R_MP_BASE_R2 = mp.meta.baselineR2.toFixed(2);
    out.R_MP_MAE = mp.meta.valMae.toFixed(1);
    out.R_MP_BASE_MAE = mp.meta.baselineMae.toFixed(1);
    out.R_MP_TRAIN = mp.meta.trainSamples.toLocaleString('en-US');
    out.R_MP_VAL = mp.meta.valSamples.toLocaleString('en-US');
  }
  if (study) {
    const c = (cond, pilot) => study.cells.find((x) => x.condition === cond && x.lighting === 'day' && x.pilot === pilot);
    out.R_TEACHER_HARD = pct(c('hard', 'teacher').coverage);
    out.R_PLANNER_HARD = pct(c('hard', 'classical').coverage);
    out.R_POLICY_HARD = pct(c('hard', 'policy').coverage);
    out.R_GUARD_HARD = pct(c('hard', 'guarded').coverage);
    out.R_POLICY_COLL = c('hard', 'policy').collisionsPerEpisode.toFixed(1);
    out.R_GUARD_COLL = c('hard', 'guarded').collisionsPerEpisode.toFixed(2);
    out.R_PLANNER_QUAD = pct(c('quad', 'classical').coverage);
    out.R_POLICY_QUAD = pct(c('quad', 'policy').coverage);
    out.R_TEACHER_QUAD = pct(c('quad', 'teacher').coverage);
    const climb = c('floors', 'classicalClimb');
    const pred = c('hard', 'classicalPredict');
    if (climb && pred) {
      const planFloors = c('floors', 'classical');
      out.R_CLIMB_FLOORS = pct(climb.coverage);
      out.R_CLIMB_FLOORS_CI = ci(climb.coverageCi);
      out.R_PLANNER_FLOORS = pct(planFloors.coverage);
      out.R_PLANNER_FLOORS_CI = ci(planFloors.coverageCi);
      out.R_PLANNER_FLOORS_N = String(climb.episodes);
      out.R_CLIMB_REACHED = `${climb.reachedAnotherFloor} of ${climb.episodes}`;
      out.R_CLIMB_COLL = climb.collisionsPerEpisode.toFixed(2);
      out.R_PRED_HARD = pct(pred.coverage);
      out.R_PRED_HARD_CI = ci(pred.coverageCi);
      out.R_ORACLEGAIN_HARD = pct(c('hard', 'classicalOracleGain').coverage);
      out.R_PRED_FLOORS = pct(c('floors', 'classicalPredict').coverage);
      out.R_ORACLEGAIN_FLOORS = pct(c('floors', 'classicalOracleGain').coverage);
    }
    const brake = c('quad', 'classicalBrake');
    const guardBrake = c('quad', 'guardedBrake');
    if (brake && guardBrake) {
      out.R_Q_PLANNER = out.R_PLANNER_QUAD;
      out.R_Q_PLANNER_COLL = c('quad', 'classical').collisionsPerEpisode.toFixed(1);
      out.R_Q_PLANNER_BRAKE = pct(brake.coverage);
      out.R_Q_PLANNER_BRAKE_COLL = brake.collisionsPerEpisode.toFixed(1);
      out.R_Q_GUARD = pct(c('quad', 'guarded').coverage);
      out.R_Q_GUARD_COLL = c('quad', 'guarded').collisionsPerEpisode.toFixed(1);
      out.R_Q_GUARD_BRAKE = pct(guardBrake.coverage);
      out.R_Q_GUARD_BRAKE_COLL = guardBrake.collisionsPerEpisode.toFixed(1);
    }
  }
  if (bench) {
    const m = bench.modes;
    out.R_M_SENSOR = pct(m.sensor.all.expectedOutcome);
    out.R_M_ORACLE = pct(m.oracle.all.expectedOutcome);
    out.R_M_POLICY = pct(m.policy.all.expectedOutcome);
    out.R_M_FLIGHTS = String(m.sensor.all.flights);
    out.R_M_ALL = String(['policy', 'sensor', 'oracle'].reduce((a, k) => a + m[k].all.flights, 0));
    out.R_M_COLL = String(['policy', 'sensor', 'oracle'].reduce((a, k) => a + m[k].all.flightsWithCollision, 0));
    out.R_M_HANDOFF = pct(m.policy.all.handoffRate ?? 0);
    out.R_M_NIGHT_SENSOR = pct(m.sensor['backpack-night'].expectedOutcome);
    out.R_M_NIGHT_ORACLE = pct(m.oracle['backpack-night'].expectedOutcome);
    out.R_M_COMPOUND = pct(m.sensor['compound-clear'].expectedOutcome);
    if (m.sensorQuad && m.policyQuad) {
      out.R_MQ_SENSOR = pct(m.sensorQuad.all.expectedOutcome);
      out.R_MQ_POLICY = pct(m.policyQuad.all.expectedOutcome);
      out.R_MQ_ALL = String(m.sensorQuad.all.flights + m.policyQuad.all.flights);
      out.R_MQ_COLL = String(m.sensorQuad.all.flightsWithCollision + m.policyQuad.all.flightsWithCollision);
    }
  }
  return out;
}

/** Failure gallery (scripts/study/gallery.ts writes the SVGs and docs/report/gallery.json). */
function galleryBlock(gallery) {
  if (!gallery) return '_Not drawn yet. Run `vite-node scripts/study/gallery.ts`._';
  return gallery
    .map(
      (g) =>
        `### ${g.title}\n\n![${g.title}](report/${g.id}.svg)\n\n${g.job.condition} building, seed ${g.job.seed}, ${g.job.lighting}: ${pct(g.coverage)} coverage, ${g.collisions} collision${g.collisions === 1 ? '' : 's'}. ${g.why}`,
    )
    .join('\n\n');
}

function benchmarks() {
  const bench = readJson('docs/benchmarks.json');
  const media = readJson('docs/media/media.json');
  const missions = readJson('docs/missions.json');
  const missionBench = readJson('docs/mission-bench.json');
  const out = {};

  if (!bench) {
    const none = '_Not measured yet. Run `npm run bench`._';
    Object.assign(out, { PROVENANCE: none, COVERAGE_TABLE: none, TARGET_TABLE: none, THROUGHPUT_TABLE: none, PER_SEED_TABLE: none, PROTOCOL: none });
  } else {
    const { protocol, coverage, throughput, machine, checkpoint } = bench;
    const short = bench.commit ? bench.commit.slice(0, 7) : 'unknown';
    out.PROVENANCE = [
      `Measured ${bench.measuredAt.slice(0, 10)} at commit \`${short}\`${bench.dirty ? ' (with uncommitted changes)' : ''}`,
      `on ${machine.cores} cores of ${machine.cpu}, Node ${machine.node}${machine.ci ? ', in CI' : ''}.`,
      `Checkpoint \`${checkpoint.path}\` (sha256 \`${checkpoint.sha256.slice(0, 12)}\`, ${checkpoint.hidden.join('→')} hidden).`,
    ].join(' ');
    out.PROTOCOL = [
      `Each episode builds a fresh building from the curriculum generator at the given stage and seed, flies ${protocol.dynamics} dynamics with the deterministic (mean) action,`,
      `and stops at ${protocol.maxSteps.easy} steps on easy buildings or ${protocol.maxSteps.hard} on hard ones, after ${protocol.stallSteps} steps without new coverage, or after ${protocol.maxCollisions} hard collisions.`,
      `Coverage is the fraction of reachable voxels the drone has seen. Easy is stage ${protocol.easy.stage} (seeds ${protocol.easy.seeds.join(', ')}).`,
      `Hard is stage ${protocol.hard.stage} (seeds ${protocol.hard.seeds.join(', ')}). The curriculum does not draw these seeds during training.`,
      'Night is the `night-dark` preset: lights off and no flashlight.',
    ].join(' ');

    const pilots = [
      ['Shipped policy, browser pilot', coverage.policy],
      ...(coverage.policyRaw ? [['Shipped policy, raw observations', coverage.policyRaw]] : []),
      ...(coverage.teacher ? [['Frontier teacher (privileged, upper reference)', coverage.teacher]] : []),
      ...(coverage.waypoint ? [['Waypoint follower (same observation, no learning)', coverage.waypoint]] : []),
      ['Lidar heuristic (baseline)', coverage.heuristic],
    ];
    out.COVERAGE_TABLE = table(
      ['Pilot', 'Easy day', 'Easy night', 'Night / day', 'Hard day', 'Hard night', 'Night / day'],
      pilots.map(([name, rows]) => {
        const e = coverageSummary(rows, protocol.easy.stage);
        const h = coverageSummary(rows, protocol.hard.stage);
        return [name, pct(e.day), pct(e.night), pct(e.ratio), pct(h.day), pct(h.night), pct(h.ratio)];
      }),
    );

    const e = coverageSummary(coverage.policy, protocol.easy.stage);
    const h = coverageSummary(coverage.policy, protocol.hard.stage);
    const allDay = mean(coverage.policy.filter((r) => r.lighting === 'day').map((r) => r.coverage));
    const allNight = mean(coverage.policy.filter((r) => r.lighting === 'night-dark').map((r) => r.coverage));
    out.TARGET_TABLE = table(
      ['Goal', 'Target', 'Measured (browser pilot)', 'Status'],
      [
        ['Coverage, easy buildings', `≥ ${pct(TARGETS.easy)}`, `${pct(e.day)} mean of ${e.n}`, mark(e.day >= TARGETS.easy)],
        ['Coverage, new hard buildings', `≥ ${pct(TARGETS.hard)}`, `${pct(h.day)} mean of ${h.n}`, mark(h.day >= TARGETS.hard)],
        ['Night coverage vs day', `≥ ${pct(TARGETS.nightRatio)}`, pct(allNight / allDay), mark(allNight / allDay >= TARGETS.nightRatio)],
        ['Kinematic env step', `≤ ${TARGETS.stepMs} ms`, `${throughput.kinematicStepMs.toFixed(3)} ms`, mark(throughput.kinematicStepMs <= TARGETS.stepMs)],
        [`Env steps/s, ${throughput.workers} workers, physics + sensors`, `≥ ${fmt(TARGETS.envStepsPerSec)}`, fmt(throughput.heuristic), mark(throughput.heuristic >= TARGETS.envStepsPerSec)],
        [`Env steps/s, ${throughput.workers} workers, with inference and resets`, `≥ ${fmt(TARGETS.envStepsPerSec)}`, fmt(throughput.policy), mark(throughput.policy >= TARGETS.envStepsPerSec)],
      ],
    );

    out.THROUGHPUT_TABLE = table(
      ['Measurement', 'Value'],
      [
        ['Kinematic env step (1 thread)', `${throughput.kinematicStepMs.toFixed(3)} ms · ${fmt(1000 / throughput.kinematicStepMs)} steps/s`],
        ['Quadrotor env step, 400 Hz physics (1 thread)', `${throughput.quadStepMs.toFixed(3)} ms · ${fmt(1000 / throughput.quadStepMs)} steps/s`],
        ['Policy inference, SIMD gemm, batch 8', `${throughput.inferMsPerEnvBatch8.toFixed(3)} ms per env`],
        [`${throughput.workers} workers, physics + sensors (lidar heuristic pilot)`, `${fmt(throughput.heuristic)} env steps/s`],
        [`${throughput.workers} workers × 8 envs, browser rollout loop: batched policy inference, resets on pooled worlds`, `${fmt(throughput.policy)} env steps/s`],
      ],
    );

    const bySeed = new Map();
    for (const [key, rows] of Object.entries(coverage)) {
      for (const r of rows) {
        const k = `${r.stage}:${r.seed}:${r.lighting}`;
        bySeed.set(k, { ...(bySeed.get(k) ?? r), [key]: r });
      }
    }
    const cols = Object.keys(coverage);
    out.PER_SEED_TABLE = table(
      ['Stage', 'Seed', 'Lighting', ...cols.map((c) => `${c} coverage (collisions)`)],
      [...bySeed.values()].map((row) => [
        String(row.stage),
        String(row.seed),
        row.lighting,
        ...cols.map((c) => (row[c] ? `${pct(row[c].coverage)} (${row[c].collisions})` : '—')),
      ]),
    );
    out.README_SUMMARY = [out.TARGET_TABLE, '', out.COVERAGE_TABLE, '', out.THROUGHPUT_TABLE].join('\n');
  }

  if (media?.clips?.length) {
    out.RENDER_TABLE = table(
      ['Clip', 'Page', 'Quality', 'Mean fps', 'Backend'],
      media.clips.filter((c) => c.fps != null).map((c) => [c.title, `\`${c.url}\``, c.quality ?? '—', fmt(c.fps, 1), c.backend ?? '—']),
    );
    out.RENDER_NOTE = `Recorded ${media.recordedAt.slice(0, 10)} in headless Chromium (${media.renderer ?? 'unknown renderer'}). Software rendering is far slower than a laptop GPU, so these are a floor, not a claim.`;
  } else {
    out.RENDER_TABLE = '_Not measured yet. Run `npm run media`._';
    out.RENDER_NOTE = '';
  }

  if (Array.isArray(missions) && missions.length) {
    const sec = (v) => (typeof v === 'number' ? `${Math.floor(v / 60)}:${String(v % 60).padStart(2, '0')}` : '—');
    out.MISSION_TABLE = table(
      ['Scenario', 'Outcome', 'Mission time', 'Time to find', 'Outdoor coverage', 'Indoor coverage', 'Entries', 'False clears', 'False alarms', 'Missed', 'Collisions', 'Battery used', 'Landed home'],
      missions.map((m) => [
        m.title ?? m.scenario,
        m.outcome,
        sec(m.durationSec),
        sec(m.timeToFindSec),
        pct(m.outdoorCoverage),
        pct(m.indoorCoverage),
        String(m.entries),
        String(m.falseClears),
        String(m.falseAlarms),
        String(m.missed),
        String(m.collisions),
        `${m.batteryUsedPct}%`,
        m.landedHome,
      ]),
    );
    const sources = [...new Set(missions.map((m) => m.source).filter(Boolean))];
    if (sources.length) out.MISSION_TABLE += `\n\nRecorded from \`${sources.join('`, `')}\` by \`npm run media\`, which reads each run's end-of-mission report from the page. Times are simulated mission time.`;
  } else {
    out.MISSION_TABLE = '_Not measured yet. The end-to-end mission runner writes `docs/missions.json`; this table fills in from it._';
  }
  if (missionBench) {
    const ci = (lo, hi) => `${(lo * 100).toFixed(0)}–${(hi * 100).toFixed(0)}%`;
    const labels = {
      policy: 'Policy, sensor planner fallback',
      sensor: 'Sensor-only planner',
      oracle: 'Oracle planner (floor plan)',
      policyQuad: 'Quadrotor: policy, planner fallback',
      sensorQuad: 'Quadrotor: sensor-only planner',
    };
    const benchModes = Object.keys(labels).filter((m) => missionBench.modes[m]);
    const scenarios = Object.keys(missionBench.modes.sensor).filter((k) => k !== 'all');
    const rows = [];
    for (const key of [...scenarios, 'all']) {
      for (const mode of benchModes) {
        const s = missionBench.modes[mode][key];
        rows.push([
          key === 'all' ? '**All**' : key,
          labels[mode],
          `${pct(s.expectedOutcome)} (${ci(...s.expectedOutcomeCi)})`,
          String(s.buildingsEntered),
          mode === 'policy' || mode === 'policyQuad' ? `${pct(s.handoffRate)} (${ci(...s.handoffRateCi)})` : '—',
          pct(s.meanIndoorCoverage),
          s.collisionsPerFlight.toFixed(2),
          `${Math.floor(s.meanDurationSec / 60)}:${String(s.meanDurationSec % 60).padStart(2, '0')}`,
          String(s.falseClears),
          String(s.falseAlarms),
        ]);
      }
    }
    out.MISSION_BENCH_TABLE = table(
      [
        'Scenario',
        'Indoors',
        'Expected outcome (95% CI)',
        'Buildings entered',
        'Handed to planner (95% CI)',
        'Indoor coverage',
        'Collisions per flight',
        'Mean mission time',
        'False clears',
        'False alarms',
      ],
      rows,
    );
    out.MISSION_BENCH_NOTE = [
      `Each scenario's settings flown on ${missionBench.seedsPerScenario} fresh world seeds (${missionBench.seedBase}–${missionBench.seedBase + missionBench.seedsPerScenario - 1}) with each indoor explorer, from \`src/eval/missions.ts\` at \`${missionBench.commit}\`.`,
      'The policy flies first under a lidar guard and hands a building to the sensor-only planner after 8 s without new coverage or on a collision; the planner always does the look-around.',
      'The sensor-only planner reads only the drone\'s own lidar, range finders and camera estimate. The oracle planner reads the true floor plan and is a reference, not a flyable mode.',
      'The quadrotor rows fly the policy and the planner on a 1.2 kg rigid-body quadrotor under an angle-mode flight controller instead of the kinematic airframe, with the braking filter as the indoor shield.',
      'Marginal openings ask an operator even under auto entry, and the bench approves every ask. Intervals are Wilson 95%.',
    ].join(' ');
  } else {
    out.MISSION_BENCH_TABLE = '';
    out.MISSION_BENCH_NOTE = '_Not measured yet. `npm run study:missions` writes `docs/mission-bench.json`._';
  }
  const study = readJson('docs/study.json');
  Object.assign(out, studyTables(study));
  out.README_MISSIONS = missionBench ? readmeMissions(missionBench) : out.MISSION_TABLE;
  // The README leads with the large study (30 fresh buildings per condition, every pilot) and keeps
  // the small fixed protocol below it, so the headline number is the one with the tighter interval.
  if (study && bench) {
    out.README_SUMMARY = [
      `**Coverage on ${study.seedsPerCondition} fresh buildings per condition**, every pilot on the same seeds, bootstrap 95% intervals ([REPORT.md](docs/REPORT.md)):`,
      '',
      out.STUDY_HEADLINE,
      '',
      `**Targets.** Fixed protocol (${bench.protocol.easy.seeds.length} easy and ${bench.protocol.hard.seeds.length} hard seeds) unless the row says otherwise:`,
      '',
      readmeTargets(out, study),
      '',
      out.THROUGHPUT_TABLE,
    ].join('\n');
  }
  out.README_SUMMARY ??= out.TARGET_TABLE;
  return out;
}


/** The fixed-protocol target table plus the same hard-building goal measured on the 30-seed study. */
function readmeTargets(out, study) {
  const lines = out.TARGET_TABLE.split('\n');
  const extra = [];
  for (const [pilot, label] of [
    ['policy', 'Coverage, new hard buildings, 30 fresh seeds (policy)'],
    ['guarded', 'Coverage, new hard buildings, 30 fresh seeds (policy + lidar guard)'],
  ]) {
    const c = study.cells.find((x) => x.condition === 'hard' && x.lighting === 'day' && x.pilot === pilot);
    if (!c) continue;
    const [lo, hi] = c.coverageCi;
    const status = lo >= TARGETS.hard ? '✅ meets' : c.coverage >= TARGETS.hard ? '⚠️ meets on the mean; interval spans the target' : '❌ misses';
    extra.push(`| ${label} | ≥ ${pct(TARGETS.hard)} | ${pct(c.coverage)} (${(lo * 100).toFixed(1)}–${(hi * 100).toFixed(1)}) | ${status} |`);
  }
  const at = lines.findIndex((l) => l.startsWith('| Coverage, new hard buildings'));
  lines.splice(at + 1, 0, ...extra);
  return lines.join('\n');
}

/** Mission outcome per scenario and indoor explorer, from the 30-world mission bench. */
function readmeMissions(mb) {
  const titles = { 'cabin-day': 'Woods to cabin: find the missing person', 'compound-clear': 'Compound search: confirm two buildings clear', 'backpack-night': 'Night search: find the lost backpack' };
  const ci = (lo, hi) => `${(lo * 100).toFixed(0)}–${(hi * 100).toFixed(0)}%`;
  const cell = (s) => `${pct(s.expectedOutcome)} (${ci(...s.expectedOutcomeCi)})`;
  const scenarios = Object.keys(mb.modes.sensor).filter((k) => k !== 'all');
  const rows = [...scenarios, 'all'].map((k) => [
    k === 'all' ? '**All**' : (titles[k] ?? k),
    cell(mb.modes.policy[k]),
    cell(mb.modes.sensor[k]),
    cell(mb.modes.oracle[k]),
  ]);
  const modes = ['policy', 'sensor', 'oracle'];
  const flights = modes.reduce((a, m) => a + mb.modes[m].all.flights, 0);
  const touched = modes.reduce((a, m) => a + mb.modes[m].all.flightsWithCollision, 0);
  const falseClears = modes.reduce((a, m) => a + mb.modes[m].all.falseClears, 0);
  const falseAlarms = modes.reduce((a, m) => a + mb.modes[m].all.falseAlarms, 0);
  return [
    table(['Scenario', 'Policy, planner fallback', 'Sensor-only planner', 'Oracle planner (floor plan, reference)'], rows),
    '',
    `Share of ${mb.seedsPerScenario} fresh mission worlds per scenario (seeds ${mb.seedBase}–${mb.seedBase + mb.seedsPerScenario - 1}) where the mission ends with the expected outcome: found, or correctly cleared. Brackets are Wilson 95% intervals.`,
    `Across all ${flights} flights: ${touched} with a collision, ${falseClears} false clears, ${falseAlarms} false alarms. The policy hands ${pct(mb.modes.policy.all.handoffRate ?? 0)} of the buildings it enters to the planner.`,
    `From \`npm run study:missions\` at \`${mb.commit}\`; per-scenario detail is in [BENCHMARKS.md](docs/BENCHMARKS.md).`,
  ].join('\n');
}

// ---------------------------------------------------------------------------------------------
// Fault bench: docs/fault-bench.json into the safety case.

function faultBench(bench, first) {
  if (!bench) {
    const none = '_Not measured yet. `npm run study:faults` writes `docs/fault-bench.json`._';
    return { FAULT_BENCH_NOTE: none, FAULT_BENCH_TABLE: '', FAULT_BENCH_SUMMARY: '', FAULT_BENCH_HISTORY: '' };
  }
  const history = (run) => {
    if (!run) return '';
    const r = run.vehicles.quad ?? Object.values(run.vehicles)[0];
    const worst = Object.entries(r.faults)
      .sort((x, y) => x[1].safeRate - y[1].safeRate)
      .slice(0, 4)
      .map(([id, s]) => `\`${id}\` ${pct(s.safeRate)}`)
      .join(', ');
    return `**An earlier evaluation on the same seeds.** These seeds were flown once before, at \`${run.commit}\` (a pre-squash commit; the flights are in \`docs/fault-bench-run1.json\`): ${pct(r.all.safeRate)} safe (${pct(r.all.safeRateCi[0])}–${pct(r.all.safeRateCi[1])}), ${r.all.collisions} collisions, weakest ${worst}. Its failures were then reproduced and fixed on the development seeds only (emergency landings pick an open column and use range-finder ground, landings cut the motors at 0.4 m, and leaving a building without a map route retraces the flown path). The table above is the second and final run on these seeds. Looking at the first run's aggregate numbers is still a form of selection, so treat the improvement as an upper bound until a third, untouched seed block is flown.`;
  };
  const resp = (r) => ({ completed: 'complete', returned_home: 'return home', landed: 'land' })[r] ?? r;
  const ci = (s) => `${pct(s.safeRate)} (${pct(s.safeRateCi[0])}–${pct(s.safeRateCi[1])})`;
  const detect = (s) => (s.medianDetectSec === null ? '–' : `${fmt(s.medianDetectSec, 1)} s (max ${fmt(s.maxDetectSec, 1)})`);
  const reasons = (s) => {
    const e = Object.entries(s.unsafeReasons);
    return e.length ? e.map(([k, v]) => `${v} ${k}`).join(', ') : '–';
  };
  const arm = bench.vehicles.quad ?? Object.values(bench.vehicles)[0];
  const rows = Object.entries(arm.faults).map(([id, s]) => [
    `\`${id}\``,
    s.hazards.length ? s.hazards.join(', ') : '–',
    `${s.flights}${s.exercised < s.flights ? ` (${s.exercised} hit)` : ''}`,
    ci(s),
    `${resp(s.intended)}: ${pct(s.intendedRate)}`,
    pct(s.expectedOutcomeRate),
    detect(s),
    reasons(s),
  ]);
  const a = arm.all;
  const nominal = arm.faults.nominal;
  return {
    FAULT_BENCH_NOTE: [
      `${bench.seedsPerScenario} fresh world seeds (${bench.seedBase}–${bench.seedBase + bench.seedsPerScenario - 1}) per mission scenario, ${arm.all.flights} flights on the quadrotor airframe (\`vehicle: 'quad'\`), frontier planner indoors, from \`src/eval/faults.ts\` at \`${bench.commit}\`. No parameter was tuned on these seeds; development used 9101–9116.`,
      'Safe: the response is one the scenario allows, with no collision, false clear, or operator-approved entry while the link was down. "Hit" counts flights whose fault trigger fired (an indoor fault cannot fire on a flight that never entered).',
      'Intended: the response the failsafe is designed for. Mission done: normal outcome and landed home. Detect: median time from the fault starting to the first failsafe event, over flights that raised one. Intervals are Wilson 95%.',
    ].join(' '),
    FAULT_BENCH_TABLE: table(['Fault scenario', 'Hazards', 'Flights', 'Safe', 'Intended response', 'Mission done', 'Detect', 'Unsafe flights'], rows),
    FAULT_BENCH_HISTORY: history(first),
    FAULT_BENCH_SUMMARY: `Across all ${a.flights} faulted flights: ${ci(a)} safe, ${a.collisions} collision${a.collisions === 1 ? '' : 's'}, ${a.falseClears} false clear${a.falseClears === 1 ? '' : 's'}, ${a.entriesWithoutLink} entr${a.entriesWithoutLink === 1 ? 'y' : 'ies'} without the link. On the ${nominal.flights} fault-free control flights the failsafe raised an event on ${nominal.flightsWithFailsafeEvent} (its own false-alarm count), and ${pct(nominal.expectedOutcomeRate)} completed the mission.`,
  };
}

// ---------------------------------------------------------------------------------------------

function fill(template, values) {
  return template.replace(/\{\{([A-Z_]+)\}\}/g, (_, key) => {
    if (!(key in values)) throw new Error(`template key ${key} has no value`);
    return values[key];
  });
}

function replaceBlock(text, name, body) {
  const start = `<!-- generated:${name}:start -->`;
  const end = `<!-- generated:${name}:end -->`;
  const i = text.indexOf(start);
  const j = text.indexOf(end);
  if (i < 0 || j < i) return text;
  return `${text.slice(0, i + start.length)}\n${body}\n${text.slice(j)}`;
}

const ablations = ablationTables(readJson('docs/ablations.json'));
const sar = sarSection(readJson('docs/sar-bench.json'), readJson('docs/report/sar-belief.json'));
const speed = speedSection(readJson('docs/speed-study.json'));
const values = {
  ...architecture(),
  ...benchmarks(),
  ...ablations.values,
  ...sar.values,
  ...speed.values,
  ...benchSection(ROOT),
  ...reportInline(readJson('docs/study.json'), readJson('docs/mission-bench.json')), GALLERY: galleryBlock(readJson('docs/report/gallery.json')),
  ...faultBench(readJson('docs/fault-bench.json'), readJson('docs/fault-bench-run1.json')),
};
const header = '<!-- Generated by scripts/docs/generate.mjs. Edit the template in scripts/docs/, not this file. -->\n\n';
const outputs = {
  'docs/ARCHITECTURE.md': header + fill(read('scripts/docs/ARCHITECTURE.template.md'), values),
  'docs/BENCHMARKS.md': header + fill(read('scripts/docs/BENCHMARKS.template.md'), values),
  'docs/REPORT.md': header + fill(read('scripts/docs/REPORT.template.md'), values),
  'docs/SAFETY_CASE.md': header + fill(read('scripts/docs/SAFETY_CASE.template.md'), values),
  'docs/LEADERBOARD.md': header + fill(read('scripts/docs/LEADERBOARD.template.md'), values),
  ...ablations.files,
  ...sar.files,
  ...speed.files,
};
let readme = read('README.md');
readme = replaceBlock(readme, 'benchmarks', values.README_SUMMARY);
readme = replaceBlock(readme, 'missions', values.README_MISSIONS);
outputs['README.md'] = readme;

let stale = false;
for (const [path, text] of Object.entries(outputs)) {
  const before = existsSync(join(ROOT, path)) ? read(path) : '';
  if (before === text) continue;
  if (CHECK) {
    console.log(`out of date: ${path}`);
    stale = true;
  } else {
    writeFileSync(join(ROOT, path), text);
    console.log(`wrote ${path}`);
  }
}
if (stale) {
  console.log('Run `npm run docs` and commit the result.');
  process.exit(1);
}

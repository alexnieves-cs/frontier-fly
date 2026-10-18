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

function benchmarks() {
  const bench = readJson('docs/benchmarks.json');
  const media = readJson('docs/media/media.json');
  const missions = readJson('docs/missions.json');
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
        [`${throughput.workers} workers, rollout loop with policy-sized network and building rebuild on reset`, `${fmt(throughput.policy)} env steps/s`],
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
    const keys = [...new Set(missions.flatMap((m) => Object.keys(m)))];
    out.MISSION_TABLE = table(
      keys,
      missions.map((m) => keys.map((k) => (typeof m[k] === 'number' && m[k] <= 1 && /coverage|rate|ratio/i.test(k) ? pct(m[k]) : String(m[k] ?? '—')))),
    );
  } else {
    out.MISSION_TABLE = '_Not measured yet. The end-to-end mission runner writes `docs/missions.json`; this table fills in from it._';
  }
  out.README_MISSIONS = out.MISSION_TABLE;
  out.README_SUMMARY ??= out.TARGET_TABLE;
  return out;
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

const values = { ...architecture(), ...benchmarks() };
const header = '<!-- Generated by scripts/docs/generate.mjs. Edit the template in scripts/docs/, not this file. -->\n\n';
const outputs = {
  'docs/ARCHITECTURE.md': header + fill(read('scripts/docs/ARCHITECTURE.template.md'), values),
  'docs/BENCHMARKS.md': header + fill(read('scripts/docs/BENCHMARKS.template.md'), values),
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

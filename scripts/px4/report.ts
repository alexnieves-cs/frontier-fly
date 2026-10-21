/**
 * Collects PX4 flight check reports into docs/px4-flight-check.json and the
 * results table in docs/PX4.md.
 *
 *   npx vite-node scripts/px4/report.ts -- --sih DIR --bridge DIR [--browser DIR] [--write]
 *   npx vite-node scripts/px4/report.ts -- --check
 *
 * Each DIR is an OUT_DIR from run-check.sh or run-browser-check.sh. For the
 * bridge and browser runs, the sim's own true track (<id>.truth.json, from
 * sim-node.ts) is compared with PX4's position estimate, which says how well
 * PX4's estimator converged on the simulated sensors. --check (CI) only
 * verifies that the table in docs/PX4.md matches the committed JSON.
 */
import { existsSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const args = process.argv.slice(2);
const opt = (name: string): string | undefined => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const JSON_PATH = 'docs/px4-flight-check.json';
const DOC_PATH = 'docs/PX4.md';
const START = '<!-- px4-results:start -->';
const END = '<!-- px4-results:end -->';

interface Check {
  id: string;
  ok: boolean;
  value: unknown;
  p95?: number;
}

interface FlightReport {
  id: string;
  label: string | null;
  ok: boolean;
  checks: Check[];
  flightSec: number;
  estimatedSec: number;
  waypoints: number;
  armedAtBootSec: number;
  track: [number, number, number, number][];
}

interface Truth {
  track: [number, number, number, number, number][];
  minTreeClearanceM: number | null;
  touchdownSpeedMps: number;
}

export interface Row {
  run: 'sih' | 'bridge' | 'browser';
  id: string;
  label: string;
  ok: boolean;
  failed: string[];
  waypoints: string;
  routeMaxM: number;
  routeP95M: number;
  altErrM: number;
  fenceMarginM: number | null;
  landM: number;
  flightSec: number;
  estimatedSec: number;
  /** PX4's estimate against the sim's true position (bridge runs only). */
  estErrP95M: number | null;
  estErrMaxM: number | null;
  minTreeClearanceM: number | null;
}

function value(r: FlightReport, id: string): Check | undefined {
  return r.checks.find((c) => c.id === id);
}

function estimateError(r: FlightReport, truth: Truth): { p95: number; max: number } | null {
  // PX4's clock is the sim's sensor clock, which starts at 1 s (src/px4/session.ts).
  const t = truth.track;
  if (t.length < 2) return null;
  const errs: number[] = [];
  for (const [rel, e, n] of r.track) {
    const simT = rel + r.armedAtBootSec - 1;
    const i = t.findIndex((row) => row[0] >= simT);
    if (i <= 0) continue;
    const a = t[i - 1]!;
    const b = t[i]!;
    const k = (simT - a[0]) / Math.max(1e-6, b[0] - a[0]);
    const te = a[1] + (b[1] - a[1]) * k;
    const tn = a[2] + (b[2] - a[2]) * k;
    errs.push(Math.hypot(e - te, n - tn));
  }
  if (!errs.length) return null;
  errs.sort((x, y) => x - y);
  return { p95: errs[Math.floor(0.95 * (errs.length - 1))]!, max: errs[errs.length - 1]! };
}

function collect(dir: string, run: Row['run']): Row[] {
  const rows: Row[] = [];
  for (const file of readdirSync(dir).filter((f) => f.endsWith('.json') && !f.endsWith('.truth.json')).sort()) {
    const r = JSON.parse(readFileSync(join(dir, file), 'utf8')) as FlightReport;
    if (!r.checks) continue;
    const truthFile = join(dir, file.replace(/\.json$/u, '.truth.json'));
    const truth = existsSync(truthFile) ? (JSON.parse(readFileSync(truthFile, 'utf8')) as Truth) : null;
    const est = truth ? estimateError(r, truth) : null;
    const route = value(r, 'route');
    rows.push({
      run,
      id: r.id,
      label: r.label ?? r.id,
      ok: r.ok,
      failed: r.checks.filter((c) => !c.ok).map((c) => c.id),
      waypoints: String(value(r, 'waypoints')?.value ?? ''),
      routeMaxM: Number(route?.value ?? NaN),
      routeP95M: Number(route?.p95 ?? NaN),
      altErrM: Number(value(r, 'altitude')?.value ?? NaN),
      fenceMarginM: (value(r, 'fence')?.value as number | null) ?? null,
      landM: Number(value(r, 'landed')?.value ?? NaN),
      flightSec: r.flightSec,
      estimatedSec: r.estimatedSec,
      estErrP95M: est ? +est.p95.toFixed(2) : null,
      estErrMaxM: est ? +est.max.toFixed(2) : null,
      minTreeClearanceM: truth?.minTreeClearanceM ?? null,
    });
  }
  return rows;
}

const RUN_NAMES: Record<Row['run'], string> = {
  sih: "PX4's own simulator (SIH)",
  bridge: 'frontier-fly physics, headless bridge',
  browser: 'frontier-fly page in Chromium',
};

function fmt(n: number | null, digits = 1): string {
  return n === null || !Number.isFinite(n) ? '–' : n.toFixed(digits);
}

export function table(rows: Row[]): string {
  const lines = [
    '| Simulator | Plan | Result | Waypoints | Off route, max / p95 (m) | Height error (m) | Fence margin (m) | Landed from target (m) | Flight (s, plan estimate) | PX4 estimate vs truth, p95 / max (m) | Closest tree (m) |',
    '| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |',
  ];
  for (const r of rows) {
    lines.push(
      `| ${RUN_NAMES[r.run]} | ${r.label} | ${r.ok ? 'pass' : `**fail** (${r.failed.join(', ')})`} | ${r.waypoints} | ` +
        `${fmt(r.routeMaxM, 2)} / ${fmt(r.routeP95M, 2)} | ${fmt(r.altErrM, 2)} | ${fmt(r.fenceMarginM, 2)} | ${fmt(r.landM, 2)} | ` +
        `${fmt(r.flightSec, 0)} (${fmt(r.estimatedSec, 0)}) | ` +
        `${r.estErrP95M === null ? '–' : `${fmt(r.estErrP95M, 2)} / ${fmt(r.estErrMaxM, 2)}`} | ${fmt(r.minTreeClearanceM, 1)} |`,
    );
  }
  return lines.join('\n');
}

function replaceBlock(doc: string, body: string): string {
  const a = doc.indexOf(START);
  const b = doc.indexOf(END);
  if (a < 0 || b < a) throw new Error(`${DOC_PATH} has no ${START} ... ${END} block`);
  return `${doc.slice(0, a + START.length)}\n${body}\n${doc.slice(b)}`;
}

if (args.includes('--check')) {
  const saved = JSON.parse(readFileSync(JSON_PATH, 'utf8')) as { rows: Row[] };
  const doc = readFileSync(DOC_PATH, 'utf8');
  if (replaceBlock(doc, table(saved.rows)) !== doc) {
    console.error(`${DOC_PATH} is out of date with ${JSON_PATH}: run scripts/px4/report.ts with --write`);
    process.exit(1);
  }
  console.log(`${DOC_PATH} matches ${JSON_PATH}`);
} else {
  const rows: Row[] = [];
  for (const run of ['sih', 'bridge', 'browser'] as const) {
    const dir = opt(run);
    if (dir && existsSync(dir)) rows.push(...collect(dir, run));
  }
  const out = { generatedBy: 'scripts/px4/report.ts', px4: 'v1.15.4', rows };
  console.log(table(rows));
  const failed = rows.filter((r) => !r.ok);
  if (args.includes('--write')) {
    writeFileSync(JSON_PATH, `${JSON.stringify(out, null, 2)}\n`);
    writeFileSync(DOC_PATH, replaceBlock(readFileSync(DOC_PATH, 'utf8'), table(rows)));
    console.log(`wrote ${JSON_PATH} and the table in ${DOC_PATH}`);
  }
  const summary = opt('summary');
  if (summary) writeFileSync(summary, `${table(rows)}\n`, { flag: 'a' });
  if (failed.length) process.exitCode = 1;
}

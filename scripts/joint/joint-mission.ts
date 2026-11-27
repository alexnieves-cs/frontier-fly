// The joint GPS-denied mission: a DSS ingress over real aerial imagery hands its position to frontier-fly at the
// treeline, and frontier-fly flies the search from there to the last room with no GNSS.
//
// The ingress is the Denied Signal Stack's footage lab: frames rendered from Spain's PNOA 2013 orthophotos, matched
// against the 2009 flight of the same tile, with GPS never fused (vision only). Its estimate at the end of the leg,
// minus the true position, is the handoff error; its protection level there says whether DSS vouches for it.
// scratch handoff.py writes those per pair into a dss-handoff@1 file.
//
// frontier-fly then flies each bench scenario with that error added to everything it believes outdoors
// (RunnerOptions.outdoorPoseError), held constant after the handoff, and indoors on its own pose estimate
// (lidar scan matching, IMU and optical flow: the sensorQuadLoc mode).
//
// Arms, paired on the same worlds:
//   control   the same flight with no outdoor error (what GNSS would give)
//   dss       the handoff error applied, whatever DSS's protection level says
// The scored "alert limit enforced" result is dss with every flight whose handoff protection level is over 10 m
// counted as held at the treeline instead of flown: DSS raised the alert, so the drone does not commit.
//
//   vite-node scripts/joint/joint-mission.ts -- --handoff handoff.json --part 0 --of 3 --out out/joint/j-0.jsonl
//   vite-node scripts/joint/joint-mission.ts -- --summary --handoff handoff.json out/joint/j-*.jsonl --out score.json
import { appendFileSync, existsSync, mkdirSync, readFileSync, writeFileSync } from 'node:fs';
import { dirname } from 'node:path';
import { BENCH_SCENARIOS, flyMission, type MissionFlight, type MissionMode, wilson } from '../../src/eval/missions';
import type { V3 } from '../../src/mission/autonomy/contracts';

function arg(name: string, fallback: string): string {
  const i = process.argv.indexOf(`--${name}`);
  return i >= 0 && process.argv[i + 1] && !process.argv[i + 1]!.startsWith('--') ? process.argv[i + 1]! : fallback;
}

/** DSS's alert limit: a handoff whose protection level is over this is not one to fly on. */
export const ALERT_LIMIT_M = 10;
export const JOINT_SEED_BASE = 47100; // a block no other study used
export const JOINT_MODE: MissionMode = 'sensorQuadLoc';

interface Handoff {
  pair: string;
  set: string;
  legSec: number;
  /** Vision-only estimate minus truth at the end of the leg, east and north, metres. */
  errEnd: [number, number];
  errEndM: number;
  hplEnd: number;
  errMax: number;
  over: number;
}

interface Row extends MissionFlight {
  arm: 'control' | 'dss';
  pair: string;
  handoffErrM: number;
  handoffHplM: number;
}

const handoffs = (): Handoff[] => (JSON.parse(readFileSync(arg('handoff', 'handoff.json'), 'utf8')) as { pairs: Handoff[] }).pairs;

async function fly(): Promise<void> {
  const H = handoffs();
  const seeds = Number(arg('seeds', '30'));
  const part = Number(arg('part', '0'));
  const of = Number(arg('of', '1'));
  const out = arg('out', `out/joint/j-${part}.jsonl`);
  mkdirSync(dirname(out) || '.', { recursive: true });
  const done = new Set(existsSync(out) ? readFileSync(out, 'utf8').split('\n').filter(Boolean).map((l) => {
    const r = JSON.parse(l) as Row;
    return `${r.arm}:${r.scenario}:${r.seed}`;
  }) : []);
  const jobs = BENCH_SCENARIOS.flatMap((s, si) => Array.from({ length: seeds }, (_, i) => ({ scenario: s.id, seed: JOINT_SEED_BASE + i, h: H[(i + si * 7) % H.length]! })))
    .flatMap((j) => (['control', 'dss'] as const).map((arm) => ({ ...j, arm })))
    .filter((_, k) => k % of === part);
  const t0 = Date.now();
  let flown = 0;
  for (const j of jobs) {
    if (done.has(`${j.arm}:${j.scenario}:${j.seed}`)) continue;
    // DSS gives east and north; frontier-fly's world is x and y across the ground, z up.
    const e: V3 = [j.h.errEnd[0], j.h.errEnd[1], 0];
    const runner = j.arm === 'dss' ? { outdoorPoseError: () => e } : {};
    const f = await flyMission({ scenario: j.scenario, seed: j.seed, indoor: JOINT_MODE }, undefined, undefined, { runner });
    const row: Row = { arm: j.arm, pair: j.h.pair, handoffErrM: j.h.errEndM, handoffHplM: j.h.hplEnd, ...f };
    appendFileSync(out, JSON.stringify(row) + '\n');
    flown++;
    if (flown % 5 === 0) console.log(`joint ${part}/${of}: ${flown}/${jobs.length - done.size} (${Math.round((Date.now() - t0) / 1000)} s)`);
  }
}

function score(rs: Row[], held = 0) {
  const n = rs.length + held;
  const ok = rs.filter((r) => r.expected).length;
  const count = (p: (r: Row) => boolean) => rs.filter(p).length;
  const entered = rs.filter((r) => r.loc);
  return {
    missions: n,
    expected: ok,
    expectedRate: +(ok / Math.max(1, n)).toFixed(3),
    expectedCI: wilson(ok, n).map((x) => +x.toFixed(3)),
    heldAtTreeline: held,
    collisions: count((r) => r.collisions > 0),
    falseClears: rs.reduce((s, r) => s + r.falseClears, 0),
    meanIndoorCoverage: +(rs.reduce((s, r) => s + r.indoorCoverage, 0) / Math.max(1, rs.length)).toFixed(3),
    indoorAteRmseM: +(entered.reduce((s, r) => s + (r.loc?.ateRmse ?? 0), 0) / Math.max(1, entered.length)).toFixed(2),
    indoorAteMapRmseM: +(entered.reduce((s, r) => s + (r.loc?.ateMapRmse ?? 0), 0) / Math.max(1, entered.length)).toFixed(2),
    meanDurationSec: Math.round(rs.reduce((s, r) => s + r.durationSec, 0) / Math.max(1, rs.length)),
  };
}

function summary(files: string[]): void {
  const rows = files.flatMap((f) => readFileSync(f, 'utf8').split('\n').filter(Boolean).map((l) => JSON.parse(l) as Row));
  const control = rows.filter((r) => r.arm === 'control');
  const dss = rows.filter((r) => r.arm === 'dss');
  const flies = (r: Row) => r.handoffHplM <= ALERT_LIMIT_M;
  const table: Record<string, ReturnType<typeof score>> = {
    'control (true outdoor pose)': score(control),
    'DSS handoff, alert ignored': score(dss),
    'DSS handoff, alert limit enforced': score(dss.filter(flies), dss.filter((r) => !flies(r)).length),
  };
  const byScenario: Record<string, unknown> = {};
  for (const s of BENCH_SCENARIOS) {
    const c = control.filter((r) => r.scenario === s.id), d = dss.filter((r) => r.scenario === s.id);
    byScenario[s.id] = {
      control: `${c.filter((r) => r.expected).length}/${c.length}`,
      dssIgnored: `${d.filter((r) => r.expected).length}/${d.length}`,
      dssEnforced: `${d.filter((r) => flies(r) && r.expected).length}/${d.length} (${d.filter((r) => !flies(r)).length} held)`,
    };
  }
  // Where the alert mattered: flights it held, and how they went when flown anyway.
  const heldRows = dss.filter((r) => !flies(r));
  const alert = {
    held: heldRows.length,
    heldWouldHaveFailed: heldRows.filter((r) => !r.expected).length,
    flownFailed: dss.filter((r) => flies(r) && !r.expected).length,
  };
  console.table(table);
  console.table(byScenario);
  console.log('alert', alert);
  const out = arg('out', '');
  if (out) {
    writeFileSync(out, JSON.stringify({
      schema: 'frontier-fly/joint-mission@1',
      mode: JOINT_MODE,
      alertLimitM: ALERT_LIMIT_M,
      seedBase: JOINT_SEED_BASE,
      handoffs: handoffs(),
      table,
      byScenario,
      alert,
    }, null, 2) + '\n');
  }
}

if (process.argv.includes('--summary')) {
  const i = process.argv.indexOf('--summary');
  summary(process.argv.slice(i + 1).filter((a) => a.endsWith('.jsonl')));
} else {
  await fly();
}

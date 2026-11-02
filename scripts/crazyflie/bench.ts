/**
 * Crazyflie arena bench: the sensor-only explorer flying generated rooms with
 * the Multi-ranger's four beams, against the same explorer with a 32-ray
 * ring, with flow-deck drift on.
 *
 *   npx vite-node scripts/crazyflie/bench.ts -- [--base 80001] [--seeds 60] [--margin 0.15] [--sweep 0.5] [--out docs/crazyflie-bench.json]
 *
 * Seeds: dev arenas 1-20 picked the margin and the yaw sweep; the published block is 80001+,
 * never tuned on.
 */
import { writeFileSync } from 'node:fs';
import { generateArena } from '../../src/crazyflie/arena';
import { CF_MARGIN, CF_SWEEP } from '../../src/crazyflie/explorer';
import { CF_BRAKING, CF_HEIGHT, CF_SPEED, FLOW_DECK } from '../../src/crazyflie/profile';
import { flyArena, type CfFlight } from '../../src/crazyflie/sim';

const args = process.argv.slice(2);
const opt = (name: string): string | undefined => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const base = Number(opt('base') ?? 80001);
const seeds = Number(opt('seeds') ?? 60);
const margin = Number(opt('margin') ?? CF_MARGIN);
const sweep = Number(opt('sweep') ?? CF_SWEEP);
const out = opt('out');

interface Arm {
  key: string;
  label: string;
  fullRing: boolean;
  drift: number;
}
const ARMS: Arm[] = [
  { key: 'multiranger', label: 'Multi-ranger (4 beams), flow drift', fullRing: false, drift: FLOW_DECK.driftFraction },
  { key: 'multirangerTruth', label: 'Multi-ranger (4 beams), perfect position', fullRing: false, drift: 0 },
  { key: 'ring32', label: '32-ray ring (the sim drone), flow drift', fullRing: true, drift: FLOW_DECK.driftFraction },
];

const mean = (xs: number[]) => xs.reduce((a, b) => a + b, 0) / Math.max(1, xs.length);
const pct = (xs: number[], p: number) => {
  const s = [...xs].sort((a, b) => a - b);
  return s[Math.min(s.length - 1, Math.floor(p * s.length))] ?? NaN;
};

const summary: Record<string, unknown> = {};
const flights: Record<string, CfFlight[]> = {};
for (const arm of ARMS) {
  const fs: CfFlight[] = [];
  for (let i = 0; i < seeds; i++) {
    const seed = base + i;
    fs.push(flyArena(generateArena(seed), { seed, fullRing: arm.fullRing, drift: arm.drift, explorer: { margin, sweep } }));
  }
  flights[arm.key] = fs;
  const cov = fs.map((f) => f.coverage);
  summary[arm.key] = {
    label: arm.label,
    flights: fs.length,
    coverageMean: round3(mean(cov)),
    coverageP10: round3(pct(cov, 0.1)),
    finished: fs.filter((f) => f.outcome === 'done').length,
    timeouts: fs.filter((f) => f.outcome === 'timeout').length,
    crashes: fs.filter((f) => f.outcome === 'crash').length,
    under5cm: fs.filter((f) => f.minClearance < 0.05).length,
    minClearanceP10: pct(
      fs.map((f) => f.minClearance),
      0.1,
    ),
    secondsMean: round3(mean(fs.map((f) => f.seconds))),
    finalEstErrMean: round3(mean(fs.map((f) => f.finalEstErr))),
  };
  console.log(arm.key, JSON.stringify(summary[arm.key]));
}

if (out) {
  writeFileSync(
    out,
    JSON.stringify(
      {
        generated: 'scripts/crazyflie/bench.ts',
        seeds: { base, count: seeds },
        config: { speed: CF_SPEED, height: CF_HEIGHT, braking: CF_BRAKING, margin, sweep, driftFraction: FLOW_DECK.driftFraction },
        summary,
        flights: Object.fromEntries(Object.entries(flights).map(([k, v]) => [k, v.map((f) => ({ ...f, track: undefined }))])),
      },
      null,
      1,
    ) + '\n',
  );
  console.log('wrote', out);
}

function round3(v: number): number {
  return Math.round(v * 1000) / 1000;
}

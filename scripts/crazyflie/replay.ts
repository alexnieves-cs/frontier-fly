/**
 * Fly one arena in the bench model (src/crazyflie/sim) for scripts/crazyflie/compare.py.
 *
 *   npx vite-node scripts/crazyflie/replay.ts -- --arena room.json [--seeds 5] [--base 1] [--drift 0.04]
 *       [--latency 0.1] [--tau 0.35] [--max-sec 180] [--steps]
 *
 * Prints one JSON object: the arena's reachable cell count and, for each seed, the flight's
 * outcome, seconds, coverage, closest pass, path, end estimate error and track. --steps adds the
 * position estimate at every control step (with --drift 0 that is the true track, which is how
 * tests/test_compare.py checks analysis.py's coverage port against this file).
 */
import { readFileSync } from 'node:fs';
import { reachableCells, validateArena, type Arena } from '../../src/crazyflie/arena';
import { flyArena } from '../../src/crazyflie/sim';

const args = process.argv.slice(2).filter((a) => a !== '--');
const opt = (name: string): string | undefined => {
  const i = args.indexOf(`--${name}`);
  return i >= 0 ? args[i + 1] : undefined;
};
const num = (name: string): number | undefined => {
  const v = opt(name);
  return v === undefined ? undefined : Number(v);
};

const path = opt('arena');
if (!path) {
  console.error('usage: replay.ts -- --arena arena.json [--seeds 5] [--base 1] [--drift 0.04] [--steps]');
  process.exit(2);
}
const arena = JSON.parse(readFileSync(path, 'utf8')) as Arena;
const errs = validateArena(arena);
if (errs.length) {
  console.error(`arena: ${errs.join('; ')}`);
  process.exit(2);
}
const seeds = num('seeds') ?? 5;
const base = num('base') ?? 1;
const withSteps = args.includes('--steps');
const round4 = (v: number) => Math.round(v * 1e4) / 1e4;

const flights: Record<string, unknown>[] = [];
for (let i = 0; i < seeds; i++) {
  const seed = base + i;
  const steps: [number, number][] = [];
  const f = flyArena(arena, {
    seed,
    drift: num('drift'),
    latency: num('latency'),
    tau: num('tau'),
    maxSec: num('max-sec'),
    onStep: withSteps ? (r) => steps.push([round4(r.x), round4(r.y)]) : undefined,
  });
  flights.push({ seed, ...f, steps: withSteps ? steps : undefined });
}
console.log(JSON.stringify({ arena: arena.name, reachable: reachableCells(arena).count, flights }));

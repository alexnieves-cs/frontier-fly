// The closing card's numbers. None is typed in: coverage comes from the same code as the homepage
// (src/landing/results.ts), failure rates from the failure atlas (docs/atlas.json), each with the conditions
// it was measured under.
import { conditional, rates } from '../../../docs/atlas.json';
import { pct, results, type Results } from '../../landing/results';

export interface Claim {
  value: string;
  label: string;
  /** What the number was measured on, for the row's accessible description. */
  condition: string;
  tone: 'result' | 'limit';
}

export interface AtlasRates {
  falseClear: { p: number; ci: number[]; flights: number };
  collision: { p: number; ci: number[]; flights: number };
  occupiedSweep: { p: number; ci: number[]; flights: number };
}

const pct2 = (v: number): string => `${(v * 100).toFixed(2)}%`;
const range = (ci: number[]): string => `${pct2(ci[0] ?? NaN)} to ${pct2(ci[1] ?? NaN)}`;

export function atlasRates(): AtlasRates {
  return {
    falseClear: rates.falseClear.importance,
    collision: rates.collision.importance,
    occupiedSweep: conditional.falseClearSweepOccupied,
  };
}

export function buildClaims(r: Results = results(), a: AtlasRates = atlasRates()): Claim[] {
  return [
    {
      value: pct(r.pilots[0]!.value),
      label: 'of unseen hard buildings searched',
      condition: `Mean of ${r.hardSeeds} held-out stage ${r.hardStage} buildings, daylight, measured ${r.measured}.`,
      tone: 'result',
    },
    {
      value: pct(Math.min(1, r.nightOverDay)),
      label: 'as good at night, lights off',
      condition: 'Night coverage relative to day on the same buildings.',
      tone: 'result',
    },
    {
      value: pct2(a.falseClear.p),
      label: 'of flights call a building clear with someone inside',
      condition: `False clear per flight, ${a.falseClear.flights.toLocaleString('en-US')} importance-sampled flights over the failure atlas mission mix, 95% interval ${range(a.falseClear.ci)}.`,
      tone: 'limit',
    },
    {
      value: pct(a.occupiedSweep.p),
      label: 'of sweeps of an occupied site end that way',
      condition: `${a.occupiedSweep.flights.toLocaleString('en-US')} "confirm the site is clear" sweeps with a person inside, 95% interval ${range(a.occupiedSweep.ci)}.`,
      tone: 'limit',
    },
    {
      value: pct2(a.collision.p),
      label: 'of flights hit something',
      condition: `Collision per flight, ${a.collision.flights.toLocaleString('en-US')} importance-sampled flights, 95% interval ${range(a.collision.ci)}.`,
      tone: 'limit',
    },
  ];
}

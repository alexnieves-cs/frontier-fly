import { RNG } from '../../shared/rng';

/**
 * A recorded wind: the same steady mean plus Gauss-Markov gusts as the fault
 * injector's wind (src/mission/autonomy/faults.ts, the low-altitude Dryden
 * model reduced to one pole per axis), sampled once on a fixed 10 Hz grid and
 * interpolated. Two simulators stepping at different rates (PX4's bridge at
 * 250 Hz, frontier-fly's controller at 400 Hz) then feel exactly the same air.
 */
export interface GustSpec {
  meanMps: number;
  /** Direction the wind blows toward, degrees counter-clockwise from east. */
  dirDeg: number;
  gustMps: number;
  gustTauSec: number;
  seed: number;
}

const GRID_HZ = 10;

export class GustTable {
  private readonly e: Float64Array;
  private readonly n: Float64Array;

  constructor(
    readonly spec: GustSpec,
    durationSec: number,
  ) {
    const count = Math.ceil(durationSec * GRID_HZ) + 2;
    this.e = new Float64Array(count);
    this.n = new Float64Array(count);
    const rng = new RNG(spec.seed, 'gust');
    const dt = 1 / GRID_HZ;
    const a = Math.exp(-dt / Math.max(1e-3, spec.gustTauSec));
    const s = spec.gustMps * Math.sqrt(1 - a * a);
    let ge = rng.gaussian() * spec.gustMps;
    let gn = rng.gaussian() * spec.gustMps;
    const dir = (spec.dirDeg * Math.PI) / 180;
    const me = spec.meanMps * Math.cos(dir);
    const mn = spec.meanMps * Math.sin(dir);
    for (let i = 0; i < count; i++) {
      this.e[i] = me + ge;
      this.n[i] = mn + gn;
      ge = ge * a + s * rng.gaussian();
      gn = gn * a + s * rng.gaussian();
    }
  }

  /** Wind at time t, [east, north] m/s. Before 0 and after the end it holds the end values. */
  at(t: number): [number, number] {
    const x = Math.max(0, t * GRID_HZ);
    const i = Math.min(Math.floor(x), this.e.length - 2);
    const f = Math.min(1, x - i);
    return [this.e[i]! + (this.e[i + 1]! - this.e[i]!) * f, this.n[i]! + (this.n[i + 1]! - this.n[i]!) * f];
  }
}

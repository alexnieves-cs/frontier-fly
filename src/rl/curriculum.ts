import { RNG } from '../shared/rng';
import type { GenParams } from '../shared/types';

const COVERAGE_GATES = [0.8, 0.75, 0.7, 0.65, 0.6, 0.55];

interface EpisodeRecord {
  coverage: number;
  reachedNewFloor: boolean;
}

/**
 * Rolling-window curriculum. A stage promotes only when the window is full
 * and only by one stage per record() call. The window clears on promotion.
 */
export class Curriculum {
  stage = 0;
  private readonly window: number;
  private history: EpisodeRecord[] = [];

  constructor(opts?: { window?: number }) {
    const window = opts?.window ?? 50;
    if (window <= 0) throw new Error('curriculum window must be positive');
    this.window = window;
  }

  record(ep: { coverage: number; reachedNewFloor: boolean }): void {
    this.history.push({ coverage: ep.coverage, reachedNewFloor: ep.reachedNewFloor });
    if (this.history.length > this.window) this.history.shift();
    if (this.history.length !== this.window) return;
    if (this.stage >= 5) return;
    let coverage = 0;
    let floors = 0;
    for (const item of this.history) {
      coverage += item.coverage;
      if (item.reachedNewFloor) floors += 1;
    }
    const mean = coverage / this.window;
    const floorRate = floors / this.window;
    const gate = COVERAGE_GATES[this.stage]!;
    if (mean <= gate) return;
    if (this.stage === 4 && floorRate <= 0.5) return;
    this.stage += 1;
    this.history = [];
  }

  /**
   * Stage 0 always returns 0. Later stages return an earlier stage with
   * probability 0.20 (uniform over 0..stage-1) and the current stage otherwise.
   */
  sampleStage(rng: RNG): number {
    if (this.stage === 0) return 0;
    if (rng.bool(0.2)) return rng.int(this.stage);
    return this.stage;
  }

  genParams(stage: number, rng: RNG): GenParams {
    let floors = 1;
    let w = 8;
    let d = 8;
    if (stage <= 0) {
      floors = 1;
      w = 8;
      d = 8;
    } else if (stage === 1) {
      floors = 1;
      w = 10;
      d = 10;
    } else if (stage === 2) {
      floors = 1;
      w = 16;
      d = 16;
    } else if (stage === 3) {
      floors = 1;
      w = 20;
      d = 20;
    } else if (stage === 4) {
      floors = 2 + rng.int(2);
      w = 16;
      d = 16;
    } else {
      floors = 2;
      w = 18;
      d = 18;
    }
    return {
      seed: rng.int(1_000_000_000),
      floors,
      w,
      d,
      stage,
    };
  }

  /**
   * Stages 0–2 are kinematic. Stage 5 and above are quadrotor.
   * Stage 3 is quadrotor with probability 0.33, stage 4 with probability 0.66.
   */
  dynamicsFor(stage: number, rng: RNG): 'kinematic' | 'quad' {
    if (stage <= 2) return 'kinematic';
    if (stage >= 5) return 'quad';
    const p = stage === 3 ? 0.33 : 0.66;
    return rng.bool(p) ? 'quad' : 'kinematic';
  }
}

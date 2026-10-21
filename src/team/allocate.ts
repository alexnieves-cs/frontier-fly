import { VOXEL } from '../shared/constants';
import type { CandidatePenalty } from '../sim/sensorPlanner';
import type { PeerState } from './mesh';

export interface ClaimOptions {
  /** Radius around a teammate's goal inside which frontiers cost more, meters. */
  goalRadius: number;
  /** Extra cost at a teammate's goal, voxel steps (falls off linearly to zero at goalRadius). */
  goalWeight: number;
  /** Extra cost of a frontier that a teammate is nearer to than this drone (straight-line), voxel steps. */
  nearerWeight: number;
  /** Teammate news older than this many team steps is ignored. */
  maxAge: number;
  /**
   * Only goals of lower-numbered teammates count. Without an order two drones
   * whose goals are close can push each other off them in turn.
   */
  priority?: boolean;
}

/**
 * Picked on the team tuning buildings (66001-66016), never on the eval blocks: the best
 * large-building coverage of 25 settings (docs/team-tuning.json). Without the goal order,
 * goal weights of 24 and up let drones push each other off their goals and lose coverage.
 */
export const DEFAULT_CLAIMS: ClaimOptions = { goalRadius: 6, goalWeight: 12, nearerWeight: 16, maxAge: 100, priority: true };

/**
 * Frontier claims for a drone team, in the spirit of coordinated multi-robot
 * exploration (Burgard et al. 2005) and RACER's pairwise split: a frontier near
 * the goal a teammate announced costs more, and so does a frontier a teammate
 * is closer to. A drone knows its teammates only from packets it received, so
 * a teammate it cannot hear claims nothing.
 */
export class ClaimPenalty implements CandidatePenalty {
  private gx: number[] = [];
  private gy: number[] = [];
  private px: number[] = [];
  private py: number[] = [];
  private vw = 1;
  private mx = 0;
  private my = 0;

  constructor(
    private readonly opts: ClaimOptions = DEFAULT_CLAIMS,
    /** Live inputs, set by the team runner before each act(). */
    private readonly source: { heard: readonly (PeerState | null)[]; selfId: number; pos: readonly [number, number, number]; step: number; floorHeight: number; drones: number },
  ) {}

  prepare(floor: number, vw: number): void {
    const s = this.source;
    this.vw = vw;
    this.mx = s.pos[0];
    this.my = s.pos[1];
    this.gx.length = this.gy.length = this.px.length = this.py.length = 0;
    for (let j = 0; j < s.drones; j++) {
      if (j === s.selfId) continue;
      const h = s.heard[j];
      if (!h || s.step - h.at > this.opts.maxAge) continue;
      if (Math.floor(h.pos[2] / s.floorHeight) !== floor) continue;
      this.px.push(h.pos[0]);
      this.py.push(h.pos[1]);
      if (h.goal && (!this.opts.priority || j < s.selfId)) {
        this.gx.push(h.goal[0]);
        this.gy.push(h.goal[1]);
      }
    }
  }

  cost(k: number): number {
    if (this.px.length === 0) return 0;
    const x = ((k % this.vw) + 0.5) * VOXEL;
    const y = (Math.floor(k / this.vw) + 0.5) * VOXEL;
    const o = this.opts;
    let c = 0;
    for (let i = 0; i < this.gx.length; i++) {
      const d = Math.hypot(x - this.gx[i]!, y - this.gy[i]!);
      if (d < o.goalRadius) c += o.goalWeight * (1 - d / o.goalRadius);
    }
    if (o.nearerWeight > 0) {
      const mine = Math.hypot(x - this.mx, y - this.my);
      for (let i = 0; i < this.px.length; i++) {
        if (Math.hypot(x - this.px[i]!, y - this.py[i]!) < mine) {
          c += o.nearerWeight;
          break;
        }
      }
    }
    return c;
  }
}

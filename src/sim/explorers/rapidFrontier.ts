/**
 * Rapid frontier selection for fast multirotor flight (T. Cieslewski,
 * E. Kaufmann, D. Scaramuzza, "Rapid exploration with multi-rotors: A
 * frontier selection method for high speed flight", IEEE/RSJ IROS 2017).
 *
 * When frontiers lie in the sensor's current field of view, the drone flies
 * reactively and straight at the one that asks for the smallest change of its
 * velocity, so it keeps its speed instead of stopping to turn. Only when no
 * frontier is in view does it fall back to classic frontier exploration: plan
 * a path to the nearest frontier (Yamauchi 1997) and fly it.
 *
 * Adaptations: the field of view is the bench's forward 90° camera out to
 * `reach` meters (the footprint radius), a frontier counts as in view when the
 * straight line to it stays on open voxels, the desired velocity is cruise
 * speed toward the frontier, and the fallback is this repository's
 * NearestFrontierExplorer on the same maps.
 */
import * as C from '../../shared/constants';
import type { SensorCommand, SensorFrame } from '../sensorPlanner';
import { DX, DY, cellX, cellY, Follower, Grid, Watchdog, type Explorer, type ExplorerOptions } from './common';
import { NearestFrontierExplorer } from './nearestFrontier';

const VOXEL = C.VOXEL;

export interface RapidFrontierOptions extends ExplorerOptions {
  /** Half-width of the field of view, radians. */
  halfFov?: number;
  /** Farthest frontier taken reactively, meters. */
  reach?: number;
  /** Nearest frontier taken reactively, meters (closer ones are already in view). */
  near?: number;
}

function wrap(a: number): number {
  return a - Math.round(a / (Math.PI * 2)) * Math.PI * 2;
}

export class RapidFrontierExplorer implements Explorer {
  readonly flyHeight: number;
  readonly halfFov: number;
  readonly reach: number;
  readonly near: number;
  private readonly fly: Follower;
  private readonly g = new Grid();
  private readonly fallback: NearestFrontierExplorer;
  private readonly watch = new Watchdog(30);
  private skip = new Uint8Array(0);
  private target = -1;
  /** The last commanded horizontal velocity, used when the frame carries no measured one. */
  private lastV: [number, number] = [0, 0];
  /** Steps flown reactively (diagnostics). */
  reactive = 0;

  constructor(opts: RapidFrontierOptions = {}) {
    this.fly = new Follower(opts);
    this.flyHeight = this.fly.flyHeight;
    this.halfFov = opts.halfFov ?? Math.PI / 4;
    this.reach = opts.reach ?? 3;
    this.near = opts.near ?? 0.75;
    this.fallback = new NearestFrontierExplorer(opts);
  }

  reset(): void {
    this.fallback.reset();
    this.watch.reset();
    this.skip = new Uint8Array(0);
    this.target = -1;
    this.lastV = [0, 0];
    this.reactive = 0;
  }

  private frontier(k: number): boolean {
    const g = this.g;
    if (!g.open(k)) return false;
    const { vw, vd } = g;
    const x = k % vw;
    const y = (k - x) / vw;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d]!;
      const ny = y + DY[d]!;
      if (nx >= 0 && ny >= 0 && nx < vw && ny < vd && g.unknown(ny * vw + nx)) return true;
    }
    return false;
  }

  /** The in-view frontier voxel needing the least change of velocity, or -1. */
  private pick(f: SensorFrame, v: readonly [number, number]): number {
    const g = this.g;
    const { vw, vd, base } = g;
    const speed = this.fly.speed;
    const r = Math.ceil(this.reach / VOXEL);
    const cx = Math.floor(f.pos[0] / VOXEL);
    const cy = Math.floor(f.pos[1] / VOXEL);
    let best = -1;
    let bestCost = Infinity;
    for (let y = Math.max(0, cy - r); y <= Math.min(vd - 1, cy + r); y++)
      for (let x = Math.max(0, cx - r); x <= Math.min(vw - 1, cx + r); x++) {
        const k = y * vw + x;
        if (this.skip[base + k] || !this.frontier(k)) continue;
        const px = cellX(k, vw);
        const py = cellY(k, vw);
        const ex = px - f.pos[0];
        const ey = py - f.pos[1];
        const d = Math.hypot(ex, ey);
        if (d < this.near || d > this.reach) continue;
        if (Math.abs(wrap(Math.atan2(ey, ex) - f.yaw)) > this.halfFov) continue;
        const dvx = (ex / d) * speed - v[0];
        const dvy = (ey / d) * speed - v[1];
        const cost = dvx * dvx + dvy * dvy;
        if (cost >= bestCost || !g.lineClear(f.pos[0], f.pos[1], px, py, 1)) continue;
        bestCost = cost;
        best = k;
      }
    return best;
  }

  private mark(k: number): void {
    const { vw, vd, base } = this.g;
    const x = k % vw;
    const y = (k - x) / vw;
    for (let dy = -1; dy <= 1; dy++)
      for (let dx = -1; dx <= 1; dx++) {
        const nx = x + dx;
        const ny = y + dy;
        if (nx >= 0 && ny >= 0 && nx < vw && ny < vd) this.skip[base + ny * vw + nx] = 1;
      }
  }

  act(f: SensorFrame): SensorCommand {
    const g = this.g;
    const floor = g.load(f);
    const n = f.floors * g.plane;
    if (this.skip.length !== n) this.skip = new Uint8Array(n);
    const v: readonly [number, number] = f.vel ?? this.lastV;
    if (this.target >= 0) {
      const d = Math.hypot(cellX(this.target, g.vw) - f.pos[0], cellY(this.target, g.vw) - f.pos[1]);
      if (this.watch.stuck(this.target, d)) this.mark(this.target);
    }
    const k = this.pick(f, v);
    let cmd: SensorCommand;
    if (k >= 0) {
      this.target = k;
      this.reactive += 1;
      cmd = this.fly.toward(f, floor, cellX(k, g.vw), cellY(k, g.vw));
    } else {
      this.target = -1;
      this.watch.reset();
      cmd = this.fallback.act(f);
    }
    this.lastV = [cmd.vx, cmd.vy];
    return cmd;
  }
}

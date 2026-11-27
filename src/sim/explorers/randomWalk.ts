/**
 * A random walk: the floor every exploration planner on the board should beat.
 *
 * No paper. The drone picks a heading uniformly at random and flies straight
 * along it, through the same lidar safety filter as every other baseline,
 * until the lidar shows a wall within `blocked` meters ahead along that
 * heading, it stops making progress, or `legSteps` steps pass. Then it draws a
 * new heading, keeping only headings the lidar shows open for at least
 * `blocked` meters when there are any. It never builds a plan and never stops
 * exploring, so on return tasks the shared return supervisor brings it home.
 *
 * The random stream restarts on every reset from a fixed seed, so a flight
 * depends only on its own task seed and the board is deterministic.
 */
import * as C from '../../shared/constants';
import { RNG } from '../../shared/rng';
import type { SensorCommand, SensorFrame } from '../sensorPlanner';
import { Follower, Grid, type Explorer, type ExplorerOptions } from './common';

const LIDAR_H = C.LIDAR_H;

export interface RandomWalkOptions extends ExplorerOptions {
  /** A heading is blocked when the lidar along it returns closer than this, meters. */
  blocked?: number;
  /** Longest leg on one heading, steps (20 steps per second). */
  legSteps?: number;
  /** RNG seed; each reset restarts the stream. */
  seed?: number;
}

function wrap(a: number): number {
  return a - Math.round(a / (Math.PI * 2)) * Math.PI * 2;
}

export class RandomWalkExplorer implements Explorer {
  readonly flyHeight: number;
  readonly blocked: number;
  readonly legSteps: number;
  readonly seed: number;
  private readonly fly: Follower;
  private readonly g = new Grid();
  private rng: RNG;
  private heading = NaN;
  private leg = 0;
  private best = Infinity;
  private still = 0;
  private start: [number, number] = [0, 0];

  constructor(opts: RandomWalkOptions = {}) {
    this.fly = new Follower(opts);
    this.flyHeight = this.fly.flyHeight;
    this.blocked = opts.blocked ?? 0.8;
    this.legSteps = opts.legSteps ?? 200;
    this.seed = opts.seed ?? 1;
    this.rng = new RNG(this.seed, 'random-walk');
  }

  reset(): void {
    this.rng = new RNG(this.seed, 'random-walk');
    this.heading = NaN;
    this.leg = 0;
  }

  /** Lidar range along a world heading. */
  private range(f: SensorFrame, h: number): number {
    const i = Math.round(wrap(h - f.yaw) / ((Math.PI * 2) / LIDAR_H));
    return f.lidar[((i % LIDAR_H) + LIDAR_H) % LIDAR_H]!;
  }

  private draw(f: SensorFrame): void {
    let h = this.rng.next() * Math.PI * 2;
    for (let t = 0; t < 16; t++) {
      if (this.range(f, h) >= this.blocked * 1.5) break;
      h = this.rng.next() * Math.PI * 2;
    }
    this.heading = h;
    this.leg = 0;
    this.best = -Infinity;
    this.still = 0;
    this.start = [f.pos[0], f.pos[1]];
  }

  act(f: SensorFrame): SensorCommand {
    const floor = this.g.load(f);
    if (Number.isNaN(this.heading)) this.draw(f);
    // Progress along the leg: distance from where it started.
    const along = Math.hypot(f.pos[0] - this.start[0], f.pos[1] - this.start[1]);
    if (along > this.best + 0.05) {
      this.best = along;
      this.still = 0;
    } else this.still += 1;
    if (++this.leg > this.legSteps || this.range(f, this.heading) < this.blocked || this.still > 30) this.draw(f);
    const tx = f.pos[0] + Math.cos(this.heading) * 2;
    const ty = f.pos[1] + Math.sin(this.heading) * 2;
    return this.fly.toward(f, floor, tx, ty);
  }
}

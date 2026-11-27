/**
 * Overwatch on top of any explorer (the overwatch track, bench/overwatch.ts): explore until the camera
 * sights the lost person, then hold a standoff from them with the camera on them; when they walk out
 * of view, fly to where they were heading and look around; after a while with no sighting, go back to
 * exploring. Reads only the sensor frame, including its camera sighting (SensorFrame.person).
 */
import * as C from '../../shared/constants';
import type { SensorCommand, SensorFrame } from '../sensorPlanner';
import { cellX, cellY, clamp, Follower, Grid, type Explorer, type ExplorerOptions } from './common';

const MAX_YAW_RATE = C.MAX_YAW_RATE;

export interface WatchOptions extends ExplorerOptions {
  /** Distance to hold from the person, metres (the track scores nothing closer than 2 m). */
  standoff?: number;
  /** Seconds to keep looking for the person after losing them before exploring again. */
  memoryS?: number;
  /** Seconds ahead to extrapolate the person's walk when they leave the view. */
  leadS?: number;
}

function wrap(a: number): number {
  return Math.atan2(Math.sin(a), Math.cos(a));
}

export class WatchExplorer implements Explorer {
  readonly flyHeight: number;
  private readonly fly: Follower;
  private readonly g = new Grid();
  private readonly standoff: number;
  private readonly memory: number;
  private readonly lead: number;
  private step = 0;
  private last: { x: number; y: number; step: number } | null = null;
  private vel: [number, number] = [0, 0];
  private anchor: { x: number; y: number; step: number } | null = null;
  private path: number[] = [];
  private pathAt = -1;

  constructor(
    private readonly inner: Explorer,
    opts: WatchOptions = {},
  ) {
    this.fly = new Follower(opts);
    this.flyHeight = inner.flyHeight;
    this.standoff = opts.standoff ?? 3.5;
    this.memory = opts.memoryS ?? 10;
    this.lead = opts.leadS ?? 3;
  }

  reset(): void {
    this.inner.reset();
    this.step = 0;
    this.last = null;
    this.vel = [0, 0];
    this.anchor = null;
    this.path = [];
    this.pathAt = -1;
  }

  act(f: SensorFrame): SensorCommand {
    this.step += 1;
    const p = f.person;
    if (p) {
      this.see(p.pos[0], p.pos[1]);
      return this.hold(f, p.pos[0], p.pos[1]);
    }
    if (this.last && this.step - this.last.step < this.memory * 20) return this.search(f);
    return this.inner.act(f);
  }

  /** Track the walk: a velocity from sightings at least half a second apart, smoothed. */
  private see(x: number, y: number): void {
    const a = this.anchor;
    if (!a || this.step - a.step > 40) this.anchor = { x, y, step: this.step };
    else if (this.step - a.step >= 10) {
      const dt = (this.step - a.step) / 20;
      const vx = (x - a.x) / dt;
      const vy = (y - a.y) / dt;
      this.vel = [0.6 * this.vel[0] + 0.4 * vx, 0.6 * this.vel[1] + 0.4 * vy];
      this.anchor = { x, y, step: this.step };
    }
    this.last = { x, y, step: this.step };
    this.path = [];
  }

  /** Hold the standoff on the drone's side of the person, camera on them. */
  private hold(f: SensorFrame, px: number, py: number): SensorCommand {
    const floor = this.g.load(f);
    let ux = f.pos[0] - px;
    let uy = f.pos[1] - py;
    const r = Math.hypot(ux, uy);
    if (r < 1e-3) {
      ux = Math.cos(f.yaw + Math.PI);
      uy = Math.sin(f.yaw + Math.PI);
    } else {
      ux /= r;
      uy /= r;
    }
    const gx = px + ux * this.standoff;
    const gy = py + uy * this.standoff;
    const gap = Math.hypot(gx - f.pos[0], gy - f.pos[1]);
    const cmd = gap > 0.25 ? this.fly.toward(f, floor, gx, gy, Math.min(this.fly.speed, 0.4 + gap)) : { ...this.fly.hover(f, floor, true), yawRate: 0 };
    return { ...cmd, yawRate: this.face(f, px, py) };
  }

  /** Fly to where the person was heading, looking ahead; once there, turn on the spot. */
  private search(f: SensorFrame): SensorCommand {
    const last = this.last!;
    const floor = this.g.load(f);
    const ahead = Math.min(this.lead, (this.step - last.step) / 20);
    const tx = last.x + this.vel[0] * ahead;
    const ty = last.y + this.vel[1] * ahead;
    const d = Math.hypot(tx - f.pos[0], ty - f.pos[1]);
    if (d < this.standoff - 0.5) return { ...this.fly.hover(f, floor, true), yawRate: clamp(1.2, -MAX_YAW_RATE, MAX_YAW_RATE) };
    const goal = this.g.cellIn(tx, ty);
    if (goal >= 0 && this.g.open(goal) && !this.g.lineClear(f.pos[0], f.pos[1], tx, ty)) {
      if (this.step - this.pathAt >= 10 || !this.path.length) {
        this.g.dijkstra(this.g.cellAt(f.pos[0], f.pos[1]));
        this.path = this.g.dist[goal]! < Infinity ? this.g.pathTo(goal) : [];
        this.pathAt = this.step;
      }
      if (this.path.length) {
        const cmd = this.fly.along(f, this.g, this.path);
        const nx = cellX(this.path[Math.min(this.path.length - 1, 2)]!, this.g.vw);
        const ny = cellY(this.path[Math.min(this.path.length - 1, 2)]!, this.g.vw);
        return { ...cmd, yawRate: this.face(f, nx, ny) };
      }
    }
    const cmd = this.fly.toward(f, floor, tx, ty);
    return { ...cmd, yawRate: this.face(f, tx, ty) };
  }

  private face(f: SensorFrame, x: number, y: number): number {
    return clamp(wrap(Math.atan2(y - f.pos[1], x - f.pos[0]) - f.yaw) * 2.5, -MAX_YAW_RATE, MAX_YAW_RATE);
  }
}

/**
 * Floor switching for the sensor-only planner in the coverage environment.
 *
 * The planner (sensorPlanner.ts) explores one floor at a time and never
 * leaves it. This wraps it with what the mission explorer already does: the
 * up and down range finders mark voxels where they see well past the ceiling
 * or the floor (a shaft), and when the floor is used up the drone flies to the
 * nearest such column, climbs or descends it, and explores the next floor.
 * It reads only the sensor frame, so it stays sensor-only.
 */
import { VOXEL } from '../shared/constants';
import { SensorPlanner, type SensorCommand, type SensorFrame } from './sensorPlanner';
import { filterVelocity, SAFETY_MARGIN } from './lidarSafety';
import { sensorPath } from './sensorMap';

/** Range past the ceiling (or below the floor) that marks a shaft, meters. */
const SHAFT_EXTRA = 0.8;

/**
 * Centre of the group of marked voxels nearest `from` on one floor plane, or
 * null when none is marked. The centre is the mean of the 8-connected group,
 * so a climb on it stays inside the column.
 */
export function shaftCentre(map: Uint8Array, base: number, vw: number, vd: number, from: readonly [number, number]): [number, number] | null {
  let best = -1;
  let bestD = Infinity;
  for (let k = 0; k < vw * vd; k++) {
    if (!map[base + k]) continue;
    const d = Math.hypot(((k % vw) + 0.5) * VOXEL - from[0], (Math.floor(k / vw) + 0.5) * VOXEL - from[1]);
    if (d < bestD) {
      bestD = d;
      best = k;
    }
  }
  if (best < 0) return null;
  const group = [best];
  const seen = new Set(group);
  let sx = 0;
  let sy = 0;
  for (let i = 0; i < group.length; i++) {
    const k = group[i]!;
    const x = k % vw;
    const y = (k - x) / vw;
    sx += (x + 0.5) * VOXEL;
    sy += (y + 0.5) * VOXEL;
    for (let dy = -1; dy <= 1; dy++) {
      for (let dx = -1; dx <= 1; dx++) {
        const nx = x + dx;
        const ny = y + dy;
        if (nx < 0 || ny < 0 || nx >= vw || ny >= vd) continue;
        const nk = ny * vw + nx;
        if (map[base + nk] && !seen.has(nk)) {
          seen.add(nk);
          group.push(nk);
        }
      }
    }
  }
  return [sx / group.length, sy / group.length];
}

export interface ClimberOptions {
  /**
   * Leave a floor once it has gone this many steps without a newly flown
   * voxel and a shaft to an unexplored floor is known, even if frontiers
   * remain (the last stragglers of a floor cost more than a fresh floor).
   * Infinity waits until the floor is used up.
   */
  stallSteps?: number;
  /** Speed on the way to the shaft, m/s. */
  speed?: number;
  /**
   * Leave a floor after this many steps on it when a shaft to an unexplored
   * floor is known: a time share of the flight budget per floor.
   */
  floorSteps?: number;
}

/** Sensor-only planner that also takes shafts between floors. */
export class Climber {
  readonly planner: SensorPlanner;
  readonly stallSteps: number;
  readonly speed: number;
  readonly floorSteps: number;
  private onFloor = 0;
  private columnZ = 0;
  private columnStuck = 0;
  private up = new Uint8Array(0);
  private down = new Uint8Array(0);
  private done = new Set<number>();
  private phase: 'sweep' | 'route' | 'column' | 'done' = 'sweep';
  private route: [number, number][] = [];
  private target: { x: number; y: number; to: number } | null = null;
  private visitedLast = 0;
  private sinceNew = 0;
  /** Floors entered by a climb or descent (diagnostics). */
  floorChanges = 0;

  constructor(planner: SensorPlanner, opts: ClimberOptions = {}) {
    this.planner = planner;
    this.stallSteps = opts.stallSteps ?? Infinity;
    this.speed = opts.speed ?? 2;
    this.floorSteps = opts.floorSteps ?? Infinity;
  }

  reset(): void {
    this.planner.reset();
    this.up = new Uint8Array(0);
    this.down = new Uint8Array(0);
    this.done.clear();
    this.phase = 'sweep';
    this.route = [];
    this.target = null;
    this.visitedLast = 0;
    this.sinceNew = 0;
    this.floorChanges = 0;
    this.onFloor = 0;
  }

  get phaseName(): string {
    return this.phase;
  }

  /** Shaft voxels marked so far, up and down (diagnostics). */
  get shaftsFound(): [number, number] {
    let u = 0;
    let d = 0;
    for (let i = 0; i < this.up.length; i++) {
      u += this.up[i]!;
      d += this.down[i]!;
    }
    return [u, d];
  }

  /** `ceil`: the up range finder's reading, meters. */
  act(f: SensorFrame, ceil: number): SensorCommand {
    const n = f.floors * f.vw * f.vd;
    if (this.up.length !== n) {
      this.up = new Uint8Array(n);
      this.down = new Uint8Array(n);
    }
    const floor = Math.max(0, Math.min(f.floors - 1, Math.floor(f.pos[2] / f.floorHeight)));
    const zf = f.pos[2] - floor * f.floorHeight;
    const vx = Math.floor(f.pos[0] / VOXEL);
    const vy = Math.floor(f.pos[1] / VOXEL);
    if (this.phase !== 'column' && vx >= 0 && vy >= 0 && vx < f.vw && vy < f.vd) {
      const k = floor * f.vw * f.vd + vy * f.vw + vx;
      if (floor < f.floors - 1 && ceil > f.floorHeight - zf + SHAFT_EXTRA) this.up[k] = 1;
      if (floor > 0 && f.alt > zf + SHAFT_EXTRA) this.down[k] = 1;
    }
    let visited = 0;
    const base = floor * f.vw * f.vd;
    for (let i = 0; i < f.vw * f.vd; i++) visited += f.visited[base + i]!;
    if (visited > this.visitedLast) this.sinceNew = 0;
    else this.sinceNew += 1;
    this.visitedLast = visited;

    if (this.phase === 'column') return this.column(f);
    if (this.phase === 'route') return this.follow(f, floor);
    if (this.phase === 'sweep') {
      const cmd = this.planner.act(f);
      this.onFloor += 1;
      const stalled = (this.sinceNew > this.stallSteps || this.onFloor > this.floorSteps) && this.next(floor, f) !== null;
      if (cmd.exploring && !stalled) return cmd;
      this.done.add(floor);
      if (this.leave(f, floor)) {
        this.columnStuck = 0;
        return this.follow(f, floor);
      }
      // Nowhere to go: if frontiers remain, keep sweeping; otherwise hover.
      if (cmd.exploring) return cmd;
      this.phase = 'done';
    }
    return { vx: 0, vy: 0, vz: 0, yawRate: 0.6, exploring: false };
  }

  /** Adjacent floor not yet explored, preferring up. */
  private next(floor: number, f: SensorFrame): number | null {
    for (const g of [floor + 1, floor - 1]) if (g >= 0 && g < f.floors && !this.done.has(g)) return g;
    return null;
  }

  private leave(f: SensorFrame, floor: number): boolean {
    for (const g of [floor + 1, floor - 1]) {
      if (g < 0 || g >= f.floors || this.done.has(g)) continue;
      const at = shaftCentre(g > floor ? this.up : this.down, floor * f.vw * f.vd, f.vw, f.vd, [f.pos[0], f.pos[1]]);
      if (!at) continue;
      const path = sensorPath(f, floor, [f.pos[0], f.pos[1]], at);
      if (!path) continue;
      this.route = path;
      this.target = { x: at[0], y: at[1], to: g };
      this.phase = 'route';
      return true;
    }
    return false;
  }

  /** Pure pursuit along the route to the shaft, through the lidar filter. */
  private follow(f: SensorFrame, floor: number): SensorCommand {
    while (this.route.length > 1 && Math.hypot(this.route[0]![0] - f.pos[0], this.route[0]![1] - f.pos[1]) < 0.6) this.route.shift();
    const [tx, ty] = this.route[0] ?? [this.target!.x, this.target!.y];
    const dx = tx - f.pos[0];
    const dy = ty - f.pos[1];
    const d = Math.hypot(dx, dy);
    if (this.route.length <= 1 && d < 0.15) {
      this.phase = 'column';
      return this.column(f);
    }
    const sp = Math.min(this.speed, d * 1.5);
    const [vx, vy] = filterVelocity((dx / d) * sp, (dy / d) * sp, f.yaw, f.lidar, SAFETY_MARGIN);
    const zTarget = floor * f.floorHeight + this.planner.flyHeight;
    return { vx, vy, vz: Math.max(-0.6, Math.min(0.6, (zTarget - f.pos[2]) * 2)), yawRate: 0, exploring: true };
  }

  /** In the column: hold xy on the shaft centre and climb or descend to the next floor's flight height. */
  private column(f: SensorFrame): SensorCommand {
    const t = this.target!;
    const ez = t.to * f.floorHeight + this.planner.flyHeight - f.pos[2];
    // Blocked in the column (the range finder saw through a gap the airframe does not fit): forget
    // this column and go back to sweeping.
    if (Math.abs(f.pos[2] - this.columnZ) < 0.01) this.columnStuck += 1;
    else this.columnStuck = 0;
    this.columnZ = f.pos[2];
    if (this.columnStuck > 10) {
      const floor = Math.max(0, Math.min(f.floors - 1, Math.floor(f.pos[2] / f.floorHeight)));
      const map = t.to > floor ? this.up : this.down;
      const base = floor * f.vw * f.vd;
      const r = 3;
      const cx = Math.floor(t.x / VOXEL);
      const cy = Math.floor(t.y / VOXEL);
      for (let y = cy - r; y <= cy + r; y++) for (let x = cx - r; x <= cx + r; x++) if (x >= 0 && y >= 0 && x < f.vw && y < f.vd) map[base + y * f.vw + x] = 0;
      this.done.delete(floor);
      this.target = null;
      this.columnStuck = 0;
      this.phase = 'sweep';
      this.sinceNew = 0;
      this.onFloor = 0;
      return { vx: 0, vy: 0, vz: 0, yawRate: 0, exploring: true };
    }
    if (Math.abs(ez) < 0.08) {
      this.target = null;
      this.planner.reset();
      this.floorChanges += 1;
      this.phase = 'sweep';
      this.sinceNew = 0;
      this.onFloor = 0;
      return { vx: 0, vy: 0, vz: 0, yawRate: 0, exploring: true };
    }
    const vx = Math.max(-0.3, Math.min(0.3, (t.x - f.pos[0]) * 2));
    const vy = Math.max(-0.3, Math.min(0.3, (t.y - f.pos[1]) * 2));
    // Climb fast through the open column, slow over the last half metre.
    return { vx, vy, vz: Math.max(-0.9, Math.min(0.9, ez * 1.5)), yawRate: 0, exploring: true };
  }
}

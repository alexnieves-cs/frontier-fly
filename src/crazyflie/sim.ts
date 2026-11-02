import { RNG } from '../shared/rng';
import { arenaSegments, castRay, clearance, reachableCells, type Arena, type Segment } from './arena';
import { CrazyflieExplorer, type CfCommand, type CfReading, type ExplorerOptions } from './explorer';
import { CF_HEIGHT, CRAZYFLIE, FLOW_DECK, MULTIRANGER } from './profile';

/**
 * A Crazyflie in an arena, at the level the explorer sees it: the stock
 * velocity loop as a lagged, acceleration-limited response, a command
 * latency, flow-deck position drift, and noisy Multi-ranger beams cast from
 * the true pose. It is a planning model, not a rigid-body one; the point is
 * to catch planner problems (sparse beams, drift, slow braking) before the
 * real drone does.
 */
export interface CfSimOptions {
  seed?: number;
  /** Fly a 32-ray ring instead of the Multi-ranger (the reference arm). */
  fullRing?: boolean;
  /** Flow-deck drift as a fraction of distance flown (0 = perfect estimate). */
  drift?: number;
  /** Seconds of flight allowed before the battery rule lands it. */
  maxSec?: number;
  /** Velocity-loop time constant, s. */
  tau?: number;
  /** Command latency, s (log + radio + planner). */
  latency?: number;
  /** Called every step with the reading and the command (for the flight-link tests). */
  onStep?: (r: CfReading, c: CfCommand) => void;
  explorer?: ExplorerOptions;
}

export interface CfFlight {
  arena: string;
  outcome: 'done' | 'timeout' | 'crash';
  seconds: number;
  /** Reachable floor within 1 m of the true track, in line of sight, fraction. */
  coverage: number;
  /** The explorer's own map: arena voxels it knows are free. */
  mapped: number;
  /** Closest the prop disc got to a wall or box, meters (negative = touched). */
  minClearance: number;
  pathM: number;
  /** Estimate minus truth at the end, meters. */
  finalEstErr: number;
  /** True track at 2 Hz, for plots. */
  track: [number, number][];
}

const DT = 0.05;
const RING = 32;

export function flyArena(arena: Arena, opts: CfSimOptions = {}): CfFlight {
  const rng = new RNG(opts.seed ?? 1, `cf-sim/${arena.name}`);
  const segs: Segment[] = arenaSegments(arena);
  const tau = opts.tau ?? 0.35;
  const delaySteps = Math.round((opts.latency ?? 0.1) / DT);
  const driftFrac = opts.drift ?? FLOW_DECK.driftFraction;
  const maxSec = opts.maxSec ?? CRAZYFLIE.flightTimeSec - 60;
  // Flow-deck error: a scale error per axis and a heading misalignment, fixed for the flight.
  const sx = rng.gaussian() * driftFrac;
  const sy = rng.gaussian() * driftFrac;
  const th = rng.gaussian() * driftFrac * 0.5;
  const explorer = new CrazyflieExplorer(arena, opts.explorer);
  const truth = { x: arena.start.x, y: arena.start.y, yaw: arena.start.yaw, vx: 0, vy: 0, wz: 0 };
  const queue: CfCommand[] = [];
  const reach = reachableCells(arena);
  const seenCells = new Uint8Array(reach.mask.length);
  const track: [number, number][] = [];
  let minClear = Infinity;
  let path = 0;
  let outcome: CfFlight['outcome'] = 'timeout';
  let t = 0;
  const estimate = () => {
    const dx = truth.x - arena.start.x;
    const dy = truth.y - arena.start.y;
    const ex = (1 + sx) * dx;
    const ey = (1 + sy) * dy;
    return {
      x: arena.start.x + Math.cos(th) * ex - Math.sin(th) * ey,
      y: arena.start.y + Math.sin(th) * ex + Math.cos(th) * ey,
      yaw: truth.yaw + th,
    };
  };
  const beam = (ang: number) => {
    if (rng.next() < 0.01) return NaN;
    const r = castRay(segs, truth.x, truth.y, ang) + rng.gaussian() * MULTIRANGER.sigma;
    return r >= MULTIRANGER.maxRange ? Infinity : Math.max(MULTIRANGER.minRange, r);
  };
  const markFootprint = () => {
    const { nx, cell } = reach;
    const i0 = Math.max(0, Math.floor((truth.x - 1) / cell));
    const i1 = Math.min(nx - 1, Math.floor((truth.x + 1) / cell));
    const j0 = Math.max(0, Math.floor((truth.y - 1) / cell));
    const j1 = Math.min(reach.ny - 1, Math.floor((truth.y + 1) / cell));
    for (let j = j0; j <= j1; j++) {
      for (let i = i0; i <= i1; i++) {
        const k = j * nx + i;
        if (!reach.mask[k] || seenCells[k]) continue;
        const cx = (i + 0.5) * cell;
        const cy = (j + 0.5) * cell;
        const d = Math.hypot(cx - truth.x, cy - truth.y);
        if (d > 1) continue;
        if (castRay(segs, truth.x, truth.y, Math.atan2(cy - truth.y, cx - truth.x)) < d) continue;
        seenCells[k] = 1;
      }
    }
  };

  for (; t < maxSec; t += DT) {
    const est = estimate();
    // Velocity estimate in the estimate's frame: the flow deck sees true velocity, rotated by the misalignment.
    const evx = Math.cos(th) * truth.vx - Math.sin(th) * truth.vy;
    const evy = Math.sin(th) * truth.vx + Math.cos(th) * truth.vy;
    const reading: CfReading = {
      t,
      x: est.x,
      y: est.y,
      z: CF_HEIGHT,
      yaw: est.yaw,
      vx: evx,
      vy: evy,
      front: beam(truth.yaw),
      left: beam(truth.yaw + Math.PI / 2),
      back: beam(truth.yaw + Math.PI),
      right: beam(truth.yaw - Math.PI / 2),
      up: Infinity,
    };
    if (opts.fullRing) {
      const ring = new Float64Array(RING);
      for (let i = 0; i < RING; i++) ring[i] = beam(truth.yaw + (i / RING) * Math.PI * 2);
      reading.ring = ring;
    }
    const cmd = explorer.step(reading);
    opts.onStep?.(reading, cmd);
    if (cmd.phase === 'done') {
      outcome = 'done';
      break;
    }
    queue.push(cmd);
    const applied = queue.length > delaySteps ? queue.shift()! : { vx: 0, vy: 0, yawRate: 0, z: CF_HEIGHT, phase: 'spin' as const };
    // The command is in the estimate's frame; the drone flies it in its body frame, so the misalignment carries over.
    const cvx = Math.cos(-th) * applied.vx - Math.sin(-th) * applied.vy;
    const cvy = Math.sin(-th) * applied.vx + Math.cos(-th) * applied.vy;
    const ax = Math.max(-2, Math.min(2, (cvx - truth.vx) / tau));
    const ay = Math.max(-2, Math.min(2, (cvy - truth.vy) / tau));
    truth.vx += ax * DT;
    truth.vy += ay * DT;
    truth.wz += ((applied.yawRate - truth.wz) / 0.15) * DT;
    const nx = truth.x + truth.vx * DT;
    const ny = truth.y + truth.vy * DT;
    path += Math.hypot(nx - truth.x, ny - truth.y);
    truth.x = nx;
    truth.y = ny;
    truth.yaw += truth.wz * DT;
    const cl = clearance(segs, truth.x, truth.y) - CRAZYFLIE.propRadius;
    minClear = Math.min(minClear, cl);
    markFootprint();
    if (Math.round(t / DT) % 10 === 0) track.push([round2(truth.x), round2(truth.y)]);
    if (cl < 0) {
      outcome = 'crash';
      break;
    }
  }
  let covered = 0;
  for (let k = 0; k < seenCells.length; k++) covered += seenCells[k]!;
  const est = estimate();
  return {
    arena: arena.name,
    outcome,
    seconds: Math.round(t * 10) / 10,
    coverage: reach.count ? covered / reach.count : 0,
    mapped: explorer.knownFree(),
    minClearance: round2(minClear),
    pathM: round2(path),
    finalEstErr: round2(Math.hypot(est.x - truth.x, est.y - truth.y)),
    track,
  };
}

function round2(v: number): number {
  return Math.round(v * 100) / 100;
}

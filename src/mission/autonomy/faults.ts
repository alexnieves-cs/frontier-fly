import { LIDAR_MAX } from '../../shared/constants';
import { RNG } from '../../shared/rng';
import { GRAVITY } from './battery';
import type { BatteryModel, Phase, V3 } from './contracts';

/**
 * Seeded fault injection for autonomous missions.
 *
 * A FaultPlan is a list of faults, each with a trigger (a mission time, or a
 * delay after the drone first enters a phase) and a duration (Infinity for
 * a permanent fault). A FaultInjector advances the plan once per tick and
 * answers two kinds of question:
 *
 * - What the hardware does: the wind the airframe flies in, how much thrust
 *   each motor still gives, how the pack drains. The vehicle applies these.
 * - What the sensors report: lidar rings with failed or noisy beams, camera
 *   frames that never arrive, a frozen or missing odometry stream, a lost
 *   GNSS fix and the dead-reckoning error that grows while it is lost, and
 *   the operator link. The autonomy only ever sees these reports.
 *
 * The injector is ground truth. The failsafe (failsafe.ts) never reads it;
 * it works from the same symptoms a flight controller would see, so a fault
 * that produces no symptom goes undetected, as it would on a real drone.
 *
 * Everything is deterministic for a given plan and seed. The wind and GNSS
 * drift processes draw from their own RNG forks, so adding a fault of one
 * kind never changes the random draws of another.
 */

export type FaultKind =
  | 'lidar_sector'
  | 'lidar_noise'
  | 'camera_dropout'
  | 'odometry_freeze'
  | 'gnss_loss'
  | 'motor'
  | 'battery_sag'
  | 'comms_loss'
  | 'wind';

/** When a fault starts: at a mission time, or a delay after the drone first enters a phase. */
export type FaultTrigger = { atSec: number } | { phase: Phase; afterSec: number };

interface FaultBase {
  /** Short id for logs and the bench, unique within a plan. */
  id: string;
  trigger: FaultTrigger;
  /** Seconds the fault lasts once started. Infinity for a permanent fault. */
  durationSec: number;
}

/**
 * A contiguous arc of lidar beams fails. `flagged`: the driver marks the
 * beams invalid (NaN), which the autonomy can see at once. `silent`: the
 * beams report no return (max range), which looks exactly like open space.
 */
export interface LidarSectorFault extends FaultBase {
  kind: 'lidar_sector';
  /** Arc centre in the body frame: 0 is straight ahead, counter-clockwise positive, degrees. */
  centerDeg: number;
  widthDeg: number;
  mode: 'flagged' | 'silent';
}

/** Extra Gaussian range noise and random missed returns on every beam. */
export interface LidarNoiseFault extends FaultBase {
  kind: 'lidar_noise';
  sigmaM: number;
  /** Chance per beam per scan of no return, 0..1. */
  missRate: number;
}

/** Detector frames stop arriving (camera, cable, or encoder failure). */
export interface CameraDropoutFault extends FaultBase {
  kind: 'camera_dropout';
}

/**
 * The odometry stream used by the indoor estimator fails. `frozen` repeats
 * the last sample; `dropped` delivers nothing.
 */
export interface OdometryFault extends FaultBase {
  kind: 'odometry_freeze';
  stream: 'flow' | 'imu' | 'both';
  mode: 'frozen' | 'dropped';
}

/**
 * The outdoor GNSS fix is lost. While it is lost the position estimate
 * dead-reckons, and its error grows at about `driftMps` in a seeded
 * direction plus a random walk.
 */
export interface GnssLossFault extends FaultBase {
  kind: 'gnss_loss';
  driftMps: number;
}

/** One motor gives only `efficiency` of its rated thrust (damaged prop, worn bearing, weak ESC). */
export interface MotorFault extends FaultBase {
  kind: 'motor';
  /** 0 front-right, 1 rear-right, 2 rear-left, 3 front-left (the quad's numbering). */
  motor: 0 | 1 | 2 | 3;
  efficiency: number;
}

/**
 * A pack fault: a weak or damaged cell. When it starts, the gauge drops by
 * `dropPct` points of usable energy, and from then on the pack drains
 * `drawScale` times faster than the energy model predicts (higher internal
 * resistance). Battery faults are permanent; `durationSec` is ignored.
 */
export interface BatterySagFault extends FaultBase {
  kind: 'battery_sag';
  dropPct: number;
  drawScale: number;
}

/** The operator link is down: no heartbeats, no answers to entry requests. */
export interface CommsLossFault extends FaultBase {
  kind: 'comms_loss';
}

/**
 * Wind: a steady mean plus horizontal gusts, each axis a first-order
 * Gauss-Markov process with standard deviation `gustMps` and correlation time
 * `gustTauSec` (the low-altitude Dryden model reduced to one pole). Wind is
 * felt outdoors only; inside a building the runner passes zero.
 */
export interface WindFault extends FaultBase {
  kind: 'wind';
  meanMps: number;
  /** Direction the wind blows toward, degrees counter-clockwise from +x (east). */
  dirDeg: number;
  gustMps: number;
  gustTauSec: number;
}

export type Fault =
  | LidarSectorFault
  | LidarNoiseFault
  | CameraDropoutFault
  | OdometryFault
  | GnssLossFault
  | MotorFault
  | BatterySagFault
  | CommsLossFault
  | WindFault;

export type FaultOf<K extends FaultKind> = Extract<Fault, { kind: K }>;

/** A fault starting or ending. Ground truth for the report and the bench, never for the failsafe. */
export interface FaultTransition {
  tSec: number;
  id: string;
  kind: FaultKind;
  edge: 'start' | 'end';
}

/** Collective thrust-to-weight ratio of the mission airframe at full throttle. */
export const THRUST_TO_WEIGHT = 2;

/** Velocity and rate caps the authority model scales. A subset of vehicle.ts VehicleLimits. */
export interface AuthorityLimits {
  speedH: number;
  speedV: number;
  accel: number;
  yawRate: number;
}

export interface AuthorityResult<L extends AuthorityLimits> {
  limits: L;
  /** Collective thrust over weight still available, at full throttle. */
  thrustToWeight: number;
  /** Sink rate forced by too little thrust to hover, m/s. 0 when the drone can hover. */
  sinkMps: number;
}

/**
 * Control authority with weak motors.
 *
 * A standard X-quad mixer keeps roll, pitch and yaw torque balanced by
 * clipping every motor to what the weakest one can match, so the usable
 * collective thrust scales with the minimum motor efficiency. (Flying on with
 * unbalanced yaw, as in Mueller and D'Andrea, ICRA 2014, recovers more, but
 * the mission airframe does not attempt it.) With collective thrust-to-weight
 * n = TW * min(e):
 *
 * - the spare vertical acceleration is g (n - 1), so accel and climb rate
 *   scale by (n - 1) / (TW - 1);
 * - the steepest level-flight tilt has tan(theta) = sqrt(n^2 - 1), so the
 *   horizontal speed cap scales by sqrt(n^2 - 1) / sqrt(TW^2 - 1);
 * - yaw comes from differential torque, so the yaw rate scales with min(e);
 * - below n = 1 the drone cannot hover and sinks. The sink rate is where
 *   rotor drag balances the deficit, approximated as linear in it.
 */
export function authorityLimits<L extends AuthorityLimits>(
  base: L,
  efficiencies: readonly number[],
  thrustToWeight = THRUST_TO_WEIGHT,
): AuthorityResult<L> {
  const e = Math.max(0, Math.min(1, ...efficiencies));
  const n = thrustToWeight * e;
  if (n <= 1) {
    const sinkMps = Math.min(6, (1 - n) * 6 + 0.3);
    return {
      limits: { ...base, speedH: 0, speedV: base.speedV, accel: base.accel * 0.1, yawRate: base.yawRate * e },
      thrustToWeight: n,
      sinkMps,
    };
  }
  const vScale = (n - 1) / (thrustToWeight - 1);
  const hScale = Math.sqrt(n * n - 1) / Math.sqrt(thrustToWeight * thrustToWeight - 1);
  return {
    limits: {
      ...base,
      speedH: base.speedH * Math.min(1, hScale),
      speedV: base.speedV * Math.min(1, vScale),
      accel: base.accel * Math.min(1, vScale),
      yawRate: base.yawRate * e,
    },
    thrustToWeight: n,
    sinkMps: 0,
  };
}

/** How much more energy a leg costs into a headwind: cruise / (cruise - headwind). Infinity when the wind wins. */
export function headwindCostScale(cruiseMps: number, headwindMps: number): number {
  const ground = cruiseMps - headwindMps;
  if (ground <= 0.25) return Infinity;
  return Math.max(1, cruiseMps / ground);
}

function triggerTime(trigger: FaultTrigger, phaseStart: ReadonlyMap<Phase, number>): number | null {
  if ('atSec' in trigger) return trigger.atSec;
  const t = phaseStart.get(trigger.phase);
  return t === undefined ? null : t + trigger.afterSec;
}

function wrapDeg(a: number): number {
  return ((((a + 180) % 360) + 360) % 360) - 180;
}

export class FaultInjector {
  /** Mission time of the last advance(). */
  tSec = 0;
  private readonly startAt = new Map<string, number>();
  private readonly phaseStart = new Map<Phase, number>();
  private readonly activeIds = new Set<string>();
  private readonly log: FaultTransition[] = [];
  private readonly gust = new Map<string, [number, number]>();
  private readonly windRng: RNG;
  private readonly gnssRng: RNG;
  private readonly lidarRng: RNG;
  private gnssError: V3 = [0, 0, 0];
  private gnssDir: [number, number] = [1, 0];
  private gnssWasLost = false;
  private lastFlow: unknown = null;
  private lastImu: unknown = null;

  constructor(
    readonly faults: readonly Fault[],
    readonly seed: number,
  ) {
    const ids = new Set<string>();
    for (const f of faults) {
      if (ids.has(f.id)) throw new Error(`Duplicate fault id ${f.id}`);
      ids.add(f.id);
    }
    const rng = new RNG(seed >>> 0, 'faults');
    this.windRng = rng.fork('wind');
    this.gnssRng = rng.fork('gnss');
    this.lidarRng = rng.fork('lidar');
  }

  /** Advance to mission time `tSec` with the drone in `phase`. Call once per tick, before reading. */
  advance(tSec: number, dt: number, phase: Phase): void {
    this.tSec = tSec;
    if (!this.phaseStart.has(phase)) this.phaseStart.set(phase, tSec);
    for (const f of this.faults) {
      let start = this.startAt.get(f.id);
      if (start === undefined) {
        const t = triggerTime(f.trigger, this.phaseStart);
        if (t === null) continue;
        start = t;
        this.startAt.set(f.id, t);
      }
      const permanent = f.kind === 'battery_sag' || !Number.isFinite(f.durationSec);
      const on = tSec + 1e-9 >= start && (permanent || tSec < start + f.durationSec - 1e-9);
      const was = this.activeIds.has(f.id);
      if (on && !was) {
        this.activeIds.add(f.id);
        this.log.push({ tSec, id: f.id, kind: f.kind, edge: 'start' });
        this.onStart(f);
      } else if (!on && was) {
        this.activeIds.delete(f.id);
        this.log.push({ tSec, id: f.id, kind: f.kind, edge: 'end' });
      }
    }
    this.stepWind(dt);
    this.stepGnss(dt);
  }

  /** Faults of a kind active now. */
  active<K extends FaultKind>(kind: K): FaultOf<K>[] {
    return this.faults.filter((f): f is FaultOf<K> => f.kind === kind && this.activeIds.has(f.id));
  }

  isActive(id: string): boolean {
    return this.activeIds.has(id);
  }

  /** Every fault start and end so far, in order. */
  transitions(): readonly FaultTransition[] {
    return this.log;
  }

  /** Resolved start time of a fault, or null while its trigger has not fired. */
  startOf(id: string): number | null {
    return this.startAt.get(id) ?? null;
  }

  // ------------------------------------------------------------ hardware

  /** World-frame wind velocity now, m/s. */
  wind(): V3 {
    let x = 0;
    let y = 0;
    for (const f of this.active('wind')) {
      const a = (f.dirDeg * Math.PI) / 180;
      const g = this.gust.get(f.id) ?? [0, 0];
      x += f.meanMps * Math.cos(a) + g[0];
      y += f.meanMps * Math.sin(a) + g[1];
    }
    return [x, y, 0];
  }

  /** Thrust efficiency of each motor now, 0..1. */
  motorEfficiency(): [number, number, number, number] {
    const out: [number, number, number, number] = [1, 1, 1, 1];
    for (const f of this.active('motor')) out[f.motor] = Math.min(out[f.motor], Math.max(0, Math.min(1, f.efficiency)));
    return out;
  }

  /**
   * Wrap a battery so it drains as the active pack faults dictate. The gauge
   * (remainingPct, usedPct) reflects the fault; powerFor and costPct keep
   * the healthy model's prediction, because that is all the autopilot knows.
   */
  wrapBattery(battery: BatteryModel): BatteryModel {
    const sags = () => this.faults.filter((f): f is BatterySagFault => f.kind === 'battery_sag' && this.activeIds.has(f.id));
    let extraUsedPct = 0;
    const applied = new Set<string>();
    const sync = (): void => {
      for (const f of sags()) {
        if (applied.has(f.id)) continue;
        applied.add(f.id);
        extraUsedPct += Math.max(0, f.dropPct);
      }
    };
    return {
      capacityWh: battery.capacityWh,
      powerFor: (vel, accel) => battery.powerFor(vel, accel),
      draw: (powerW, dtSec) => {
        sync();
        const scale = sags().reduce((s, f) => s * Math.max(1, f.drawScale), 1);
        battery.draw(powerW * scale, dtSec);
      },
      remainingPct: () => {
        sync();
        return Math.max(0, battery.remainingPct() - extraUsedPct);
      },
      usedPct: () => {
        sync();
        return Math.min(100, battery.usedPct() + extraUsedPct);
      },
      costPct: (d, climb) => battery.costPct(d, climb),
    };
  }

  // ------------------------------------------------------------- sensors

  /**
   * Apply lidar faults to a horizontal ring, in place, and return it. Beam i
   * points at yaw + i * 2 pi / ring.length (body angle i * 360 / n degrees).
   * A flagged sector reads NaN, a silent one reads `maxM` (no return), noise
   * adds Gaussian error and random misses, clipped to [0, maxM].
   */
  lidarRing<T extends Float32Array | Float64Array | number[]>(ring: T, maxM = LIDAR_MAX): T {
    const n = ring.length;
    const noise = this.active('lidar_noise');
    for (let i = 0; i < n; i++) {
      let r = ring[i]!;
      for (const f of noise) {
        if (this.lidarRng.next() < f.missRate) r = maxM;
        else if (r < maxM) r = Math.max(0, Math.min(maxM, r + this.lidarRng.gaussian() * f.sigmaM));
      }
      ring[i] = r;
    }
    for (const f of this.active('lidar_sector')) {
      for (const i of sectorBeams(n, f.centerDeg, f.widthDeg)) ring[i] = f.mode === 'flagged' ? NaN : maxM;
    }
    return ring;
  }

  /** True when a detector frame arrives this tick. */
  cameraUp(): boolean {
    return this.active('camera_dropout').length === 0;
  }

  /** True when the operator link is up. */
  linkUp(): boolean {
    return this.active('comms_loss').length === 0;
  }

  /**
   * Pass an odometry sample through the active faults. A frozen stream
   * returns the last sample delivered before the fault; a dropped one
   * returns null.
   */
  odometry<T>(stream: 'flow' | 'imu', sample: T): T | null {
    for (const f of this.active('odometry_freeze')) {
      if (f.stream !== stream && f.stream !== 'both') continue;
      if (f.mode === 'dropped') return null;
      const last = (stream === 'flow' ? this.lastFlow : this.lastImu) as T | null;
      return last;
    }
    if (stream === 'flow') this.lastFlow = sample;
    else this.lastImu = sample;
    return sample;
  }

  /** GNSS fix state and the error the reported position carries, world frame, metres. */
  gnss(): { fix: boolean; errorM: V3 } {
    const lost = this.active('gnss_loss').length > 0;
    return { fix: !lost, errorM: lost ? [this.gnssError[0], this.gnssError[1], this.gnssError[2]] : [0, 0, 0] };
  }

  // ------------------------------------------------------------ internal

  private onStart(f: Fault): void {
    if (f.kind === 'wind' && !this.gust.has(f.id)) {
      // Start the gust process from its stationary distribution, so the first gust is not always calm.
      this.gust.set(f.id, [this.windRng.gaussian() * f.gustMps, this.windRng.gaussian() * f.gustMps]);
    }
  }

  private stepWind(dt: number): void {
    if (dt <= 0) return;
    for (const f of this.active('wind')) {
      const g = this.gust.get(f.id) ?? [0, 0];
      // Exact discretisation of dx = -x / tau dt + sigma sqrt(2 / tau) dW.
      const a = Math.exp(-dt / Math.max(1e-3, f.gustTauSec));
      const s = f.gustMps * Math.sqrt(1 - a * a);
      this.gust.set(f.id, [g[0] * a + s * this.windRng.gaussian(), g[1] * a + s * this.windRng.gaussian()]);
    }
  }

  private stepGnss(dt: number): void {
    const lost = this.active('gnss_loss');
    if (lost.length === 0) {
      this.gnssWasLost = false;
      this.gnssError = [0, 0, 0];
      return;
    }
    if (!this.gnssWasLost) {
      this.gnssWasLost = true;
      const a = this.gnssRng.next() * Math.PI * 2;
      this.gnssDir = [Math.cos(a), Math.sin(a)];
      this.gnssError = [0, 0, 0];
    }
    if (dt <= 0) return;
    const drift = Math.max(...lost.map((f) => f.driftMps));
    // Bias drift along a seeded direction plus a random walk of the same order per sqrt(s).
    const walk = drift * Math.sqrt(dt);
    this.gnssError = [
      this.gnssError[0] + this.gnssDir[0] * drift * dt + walk * this.gnssRng.gaussian(),
      this.gnssError[1] + this.gnssDir[1] * drift * dt + walk * this.gnssRng.gaussian(),
      this.gnssError[2] + 0.2 * walk * this.gnssRng.gaussian(),
    ];
  }
}

/** Indices of the beams of an n-beam ring inside an arc, body frame, degrees. */
export function sectorBeams(n: number, centerDeg: number, widthDeg: number): number[] {
  const out: number[] = [];
  const half = Math.max(0, widthDeg) / 2;
  for (let i = 0; i < n; i++) {
    const a = (i * 360) / n;
    if (Math.abs(wrapDeg(a - centerDeg)) <= half + 1e-9) out.push(i);
  }
  return out;
}

/** Steady acceleration margin over hover with all motors at `efficiency`, m/s^2. */
export function spareAccel(efficiency: number, thrustToWeight = THRUST_TO_WEIGHT): number {
  return GRAVITY * (thrustToWeight * efficiency - 1);
}

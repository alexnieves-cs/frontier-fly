import { localFrame, type GeoPoint, type LocalFrame } from '../mission/export/geo';
import {
  decodeActuatorControls,
  decodeHeartbeat,
  HIL_SENSOR_FIELDS,
  MavlinkParser,
  MavlinkWriter,
  MSG,
  type ActuatorControls,
  type Heartbeat,
} from './mavlink';
import { Px4Quad, rotateInv, type Vec3 } from './quad';
import { DECLINATION, INCLINATION, STRENGTH } from './wmmTables';

/**
 * Frontier-fly as PX4's simulator.
 *
 * PX4 SITL's simulator_mavlink module connects to a simulator over TCP and
 * runs in lockstep with it: the simulator sends HIL_SENSOR stamped with its
 * own clock, PX4 advances to that time, runs its estimator and controllers,
 * and answers with HIL_ACTUATOR_CONTROLS. This class is the simulator side,
 * independent of the transport: give it bytes from PX4 with `receive`, and it
 * hands bytes for PX4 to `send`. Node pipes it straight to the TCP socket;
 * the browser reaches PX4 through a WebSocket bridge (scripts/px4/bridge.mjs).
 */

export interface SessionOptions {
  /** Where the pad is on the globe; the GNSS and magnetometer use it. */
  home: GeoPoint;
  quad?: Px4Quad;
  /** Physics and IMU step, microseconds. PX4 expects 250 Hz. */
  stepUs?: number;
  gpsRateHz?: number;
  /** Seed for sensor noise. */
  seed?: number;
  /** Wall-clock wait for PX4's answer before stepping anyway, ms. */
  lockstepTimeoutMs?: number;
}

export interface SessionStats {
  steps: number;
  simTimeSec: number;
  /** Steps that went ahead without PX4's answer (PX4 still booting, or slow). */
  timeouts: number;
  controlsReceived: number;
  badCrc: number;
}

/** Bilinear lookup in PX4's 10° WMM grid, the same as PX4's get_table_data. */
export function wmmLookup(table: readonly (readonly number[])[], latDeg: number, lonDeg: number): number {
  const res = 10;
  const lat = Math.max(-90, Math.min(90, latDeg));
  const lon = lonDeg > 180 ? lonDeg - 360 : lonDeg < -180 ? lonDeg + 360 : lonDeg;
  let minLat = Math.floor(lat / res) * res;
  let minLon = Math.floor(lon / res) * res;
  minLat = Math.max(-90, Math.min(90 - res, minLat));
  minLon = Math.max(-180, Math.min(180 - res, minLon));
  const i = Math.round((minLat + 90) / res);
  const j = Math.round((minLon + 180) / res);
  const sw = table[i]![j]!;
  const se = table[i]![j + 1]!;
  const ne = table[i + 1]![j + 1]!;
  const nw = table[i + 1]![j]!;
  const latScale = Math.max(0, Math.min(1, (lat - minLat) / res));
  const lonScale = Math.max(0, Math.min(1, (lon - minLon) / res));
  const lo = lonScale * (se - sw) + sw;
  const hi = lonScale * (ne - nw) + nw;
  return latScale * (hi - lo) + lo;
}

/** Earth's field at a place, NED gauss, from PX4's own WMM tables. */
export function earthField(latDeg: number, lonDeg: number): Vec3 {
  const dec = wmmLookup(DECLINATION, latDeg, lonDeg) * 1e-4;
  const inc = wmmLookup(INCLINATION, latDeg, lonDeg) * 1e-4;
  const strength = wmmLookup(STRENGTH, latDeg, lonDeg) * 1e-4;
  const h = strength * Math.cos(inc);
  return [h * Math.cos(dec), h * Math.sin(dec), strength * Math.sin(inc)];
}

/** International Standard Atmosphere pressure at an AMSL altitude, hPa. */
export function isaPressureHpa(altM: number): number {
  return 1013.25 * Math.pow(1 - 2.25577e-5 * altM, 5.25588);
}

/** Small deterministic Gaussian noise (mulberry32 + Box-Muller). */
class Noise {
  private s: number;
  private spare: number | null = null;
  constructor(seed: number) {
    this.s = seed >>> 0 || 1;
  }
  private uniform(): number {
    this.s = (this.s + 0x6d2b79f5) >>> 0;
    let t = this.s;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }
  gauss(sigma: number): number {
    if (this.spare !== null) {
      const v = this.spare;
      this.spare = null;
      return v * sigma;
    }
    const u = Math.max(1e-12, this.uniform());
    const v = this.uniform();
    const r = Math.sqrt(-2 * Math.log(u));
    this.spare = r * Math.sin(2 * Math.PI * v);
    return r * Math.cos(2 * Math.PI * v) * sigma;
  }
}

export class Px4SimSession {
  readonly quad: Px4Quad;
  readonly home: GeoPoint;
  readonly frame: LocalFrame;
  readonly stepUs: number;
  private readonly writer = new MavlinkWriter(1, 51);
  private readonly parser = new MavlinkParser();
  private readonly noise: Noise;
  private readonly field: Vec3;
  private readonly gpsEvery: number;
  private readonly timeoutMs: number;
  private timeUs = 0;
  private controlsSeen = 0;
  private waiter: (() => void) | null = null;
  readonly stats: SessionStats = { steps: 0, simTimeSec: 0, timeouts: 0, controlsReceived: 0, badCrc: 0 };
  /** Latest motor outputs from PX4 (0..1). */
  readonly motors = new Float32Array(4);
  lastControls: ActuatorControls | null = null;
  lastHeartbeat: Heartbeat | null = null;
  armed = false;

  constructor(
    private readonly send: (bytes: Uint8Array) => void,
    options: SessionOptions,
  ) {
    this.home = options.home;
    this.frame = localFrame(options.home);
    this.quad = options.quad ?? new Px4Quad();
    this.stepUs = options.stepUs ?? 4000;
    this.gpsEvery = Math.max(1, Math.round(1e6 / this.stepUs / (options.gpsRateHz ?? 10)));
    this.noise = new Noise(options.seed ?? 1);
    this.field = earthField(options.home.latDeg, options.home.lonDeg);
    this.timeoutMs = options.lockstepTimeoutMs ?? 50;
    // PX4's lockstep clock starts from the first sensor stamp; start at a round, nonzero time.
    this.timeUs = 1_000_000;
  }

  /** Bytes from PX4. */
  receive(chunk: Uint8Array): void {
    for (const f of this.parser.push(chunk)) {
      if (f.msgid === MSG.HIL_ACTUATOR_CONTROLS) {
        const c = decodeActuatorControls(f);
        this.lastControls = c;
        this.armed = (c.mode & 128) !== 0;
        for (let i = 0; i < 4; i++) this.motors[i] = this.armed ? (c.controls[i] ?? 0) : 0;
        this.controlsSeen++;
        this.stats.controlsReceived++;
        const w = this.waiter;
        this.waiter = null;
        w?.();
      } else if (f.msgid === MSG.HEARTBEAT) {
        this.lastHeartbeat = decodeHeartbeat(f);
      }
    }
    this.stats.badCrc = this.parser.badCrc;
  }

  /** Release a pending step, for when the transport closes. */
  abort(): void {
    const w = this.waiter;
    this.waiter = null;
    w?.();
  }

  private waitControls(after: number): Promise<boolean> {
    if (this.controlsSeen > after) return Promise.resolve(true);
    return new Promise((resolve) => {
      const timer = setTimeout(() => {
        this.waiter = null;
        resolve(false);
      }, this.timeoutMs);
      this.waiter = () => {
        clearTimeout(timer);
        resolve(true);
      };
    });
  }

  /** NED position of the vehicle as a point on the globe. */
  geodetic(): GeoPoint {
    const [n, e, d] = this.quad.s.pos;
    const g = this.frame.toGeo(e, n, 0);
    return { latDeg: g.latDeg, lonDeg: g.lonDeg, altM: this.home.altM - d };
  }

  private sendSensors(): void {
    const q = this.quad;
    const k = this.stats.steps;
    const n = this.noise;
    const sf = q.specificForce;
    const w = q.s.omega;
    let fields: number = HIL_SENSOR_FIELDS.ACCEL | HIL_SENSOR_FIELDS.GYRO;
    const slow = k % 5 === 0;
    let mag: Vec3 = [0, 0, 0];
    let pressure = 0;
    let alt = 0;
    if (slow) {
      fields |= HIL_SENSOR_FIELDS.MAG | HIL_SENSOR_FIELDS.BARO;
      mag = rotateInv(q.s.q, this.field);
      alt = this.home.altM - q.s.pos[2];
      pressure = isaPressureHpa(alt) + n.gauss(0.003);
    }
    this.send(
      this.writer.hilSensor({
        timeUsec: this.timeUs,
        xacc: sf[0] + n.gauss(0.01),
        yacc: sf[1] + n.gauss(0.01),
        zacc: sf[2] + n.gauss(0.01),
        xgyro: w[0] + n.gauss(0.0005),
        ygyro: w[1] + n.gauss(0.0005),
        zgyro: w[2] + n.gauss(0.0005),
        xmag: mag[0] + n.gauss(0.001),
        ymag: mag[1] + n.gauss(0.001),
        zmag: mag[2] + n.gauss(0.001),
        absPressure: pressure,
        diffPressure: 0,
        pressureAlt: alt,
        temperature: 15 - 0.0065 * alt,
        fieldsUpdated: fields,
        id: 0,
      }),
    );
    if (k % this.gpsEvery === 0) {
      const g = this.geodetic();
      const [vn, ve, vd] = q.s.vel;
      this.send(
        this.writer.hilGps({
          timeUsec: this.timeUs,
          latDeg: g.latDeg,
          lonDeg: g.lonDeg,
          altM: g.altM,
          ephCm: 30,
          epvCm: 40,
          vn,
          ve,
          vd,
          fixType: 3,
          satellites: 12,
        }),
      );
    }
    if (k % Math.round(1e6 / this.stepUs) === 0) {
      this.send(this.writer.heartbeat({ type: 0, autopilot: 8, baseMode: 0, customMode: 0, systemStatus: 4 }));
    }
  }

  /**
   * One lockstep cycle: sensors at the current time, wait for PX4's motor
   * outputs, then advance the physics one step.
   */
  async step(): Promise<void> {
    const seen = this.controlsSeen;
    this.sendSensors();
    const answered = await this.waitControls(seen);
    if (!answered) this.stats.timeouts++;
    this.quad.step(this.motors, this.stepUs * 1e-6);
    this.timeUs += this.stepUs;
    this.stats.steps++;
    this.stats.simTimeSec = (this.timeUs - 1_000_000) * 1e-6;
  }

  /** Simulated seconds since the session started. */
  get simTimeSec(): number {
    return this.stats.simTimeSec;
  }
}

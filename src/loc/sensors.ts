import { SURFACE } from '../shared/constants';
import { RNG } from '../shared/rng';
import type { LocFrame, V3 } from './types';

/**
 * Noise model of the localization sensors: a consumer MEMS gyro and
 * accelerometer, a downward optical-flow sensor, a 2D scanning lidar and a
 * downward range finder. Values are typical datasheet figures for parts a
 * 0.5 kg indoor drone carries (BMI088-class IMU, PMW3901 flow, RPLidar
 * A1-class lidar, VL53L1X range finder), rounded up.
 */
export interface LocNoise {
  /** White gyro noise per sample, rad/s. */
  gyroSigma: number;
  /** Turn-on gyro bias, drawn uniform in ±this, rad/s (after a still calibration). */
  gyroBias: number;
  /** Gyro bias random walk, rad/s/√s. */
  gyroBiasRW: number;
  /** White accelerometer noise per sample, m/s^2. */
  accelSigma: number;
  /** Turn-on accelerometer bias, ±this, m/s^2. */
  accelBias: number;
  /** Flow velocity noise: sigma = abs + rel * speed, m/s. */
  flowSigma: number;
  flowSigmaRel: number;
  /** Per-flight flow scale error (range-finder and lens calibration), 1-sigma fraction. */
  flowScale: number;
  /** Probability the flow sensor reports nothing this tick. */
  flowDropout: number;
  /** Lidar range noise: sigma = abs + rel * range, m. */
  lidarSigma: number;
  lidarSigmaRel: number;
  lidarDropout: number;
  /** Extra dropout on glass (matches sim/dynamics/noise.ts). */
  glassDropout: number;
  /** Range-finder noise, m. */
  altSigma: number;
  /** Barometer white noise, m, and its drift, m/√s. */
  baroSigma: number;
  baroRW: number;
  /** Beams per lidar sweep. */
  beams: number;
  /** A sweep every this many ticks (2 at 20 Hz = a 10 Hz lidar). */
  scanEvery: number;
}

/**
 * Daylight figures. In the dark the flow sensor loses texture: it drops
 * most samples and the ones left are noisier. Lidar and IMU don't care.
 */
export function locNoise(dark = false): LocNoise {
  return {
    gyroSigma: 0.01,
    gyroBias: 0.005,
    gyroBiasRW: 0.0005,
    accelSigma: 0.08,
    accelBias: 0.05,
    flowSigma: dark ? 0.08 : 0.03,
    flowSigmaRel: dark ? 0.15 : 0.06,
    flowScale: 0.03,
    flowDropout: dark ? 0.6 : 0.02,
    lidarSigma: 0.02,
    lidarSigmaRel: 0.005,
    lidarDropout: 0.01,
    glassDropout: 0.35,
    altSigma: 0.02,
    baroSigma: 0.1,
    baroRW: 0.01,
    beams: 90,
    scanEvery: 2,
  };
}

/** Any ray caster: the coverage env's PhysicsWorld, or a mission's shell lidar. */
export interface Caster {
  cast(x: number, y: number, z: number, dx: number, dy: number, dz: number, maxDist: number): { dist: number; surface: number } | null;
}

/** Ground truth for one tick: where the drone really is and how it really moves. */
export interface TruthSample {
  pos: V3;
  yaw: number;
  /** World-frame velocity, m/s. */
  vel: V3;
  /** World-frame acceleration, m/s^2. */
  accel: V3;
  yawRate: number;
  dt: number;
}

/**
 * Turns ground truth into what the sensors report. It is the only place in
 * src/loc that sees the true pose; the estimator gets only its output.
 */
export class LocSensors {
  readonly noise: LocNoise;
  private readonly rng: RNG;
  private gyroBias = 0;
  private readonly accelBias: [number, number] = [0, 0];
  private readonly flowScale: [number, number] = [1, 1];
  private ticks = 0;
  private baroDrift = 0;
  private readonly cosTable: Float64Array;
  private readonly sinTable: Float64Array;

  constructor(
    private readonly world: Caster,
    seed: number,
    noise: LocNoise = locNoise(),
    readonly maxRange = 8,
  ) {
    this.noise = noise;
    this.rng = new RNG(seed >>> 0, 'loc-sensors');
    this.gyroBias = this.rng.range(-1, 1) * noise.gyroBias;
    this.accelBias[0] = this.rng.range(-1, 1) * noise.accelBias;
    this.accelBias[1] = this.rng.range(-1, 1) * noise.accelBias;
    this.flowScale[0] = 1 + this.rng.gaussian() * noise.flowScale;
    this.flowScale[1] = 1 + this.rng.gaussian() * noise.flowScale;
    this.cosTable = new Float64Array(noise.beams);
    this.sinTable = new Float64Array(noise.beams);
    for (let i = 0; i < noise.beams; i++) {
      this.cosTable[i] = Math.cos((i / noise.beams) * Math.PI * 2);
      this.sinTable[i] = Math.sin((i / noise.beams) * Math.PI * 2);
    }
  }

  /** True gyro bias right now (for tests and reports). */
  get trueGyroBias(): number {
    return this.gyroBias;
  }

  read(t: TruthSample): LocFrame {
    const n = this.noise;
    const rng = this.rng;
    const c = Math.cos(t.yaw);
    const s = Math.sin(t.yaw);
    this.gyroBias += n.gyroBiasRW * Math.sqrt(Math.max(t.dt, 0)) * rng.gaussian();
    const gyroZ = t.yawRate + this.gyroBias + n.gyroSigma * rng.gaussian();
    const abx = c * t.accel[0] + s * t.accel[1];
    const aby = -s * t.accel[0] + c * t.accel[1];
    const accel: V3 = [
      abx + this.accelBias[0] + n.accelSigma * rng.gaussian(),
      aby + this.accelBias[1] + n.accelSigma * rng.gaussian(),
      t.accel[2] + n.accelSigma * rng.gaussian(),
    ];

    const vbx = c * t.vel[0] + s * t.vel[1];
    const vby = -s * t.vel[0] + c * t.vel[1];
    let flow: LocFrame['flow'] = null;
    if (!rng.bool(n.flowDropout)) {
      const sig = n.flowSigma + n.flowSigmaRel * Math.hypot(vbx, vby);
      flow = {
        vel: [vbx * this.flowScale[0] + sig * rng.gaussian(), vby * this.flowScale[1] + sig * rng.gaussian()],
        quality: n.flowDropout > 0.3 ? 0.4 : 1,
      };
    }

    let scan: LocFrame['scan'] = null;
    if (this.ticks % Math.max(1, n.scanEvery) === 0) {
      const ranges = new Float64Array(n.beams);
      const max = this.maxRange;
      for (let i = 0; i < n.beams; i++) {
        const dx = c * this.cosTable[i]! - s * this.sinTable[i]!;
        const dy = s * this.cosTable[i]! + c * this.sinTable[i]!;
        const hit = this.world.cast(t.pos[0], t.pos[1], t.pos[2], dx, dy, 0, max);
        let r = max;
        if (hit) {
          const p = n.lidarDropout + (hit.surface === SURFACE.GLASS ? n.glassDropout : 0);
          if (!rng.bool(p)) r = hit.dist + (n.lidarSigma + n.lidarSigmaRel * hit.dist) * rng.gaussian();
        }
        ranges[i] = r < 0 ? 0 : r > max ? max : r;
      }
      scan = { ranges, maxRange: max };
    }

    const down = this.world.cast(t.pos[0], t.pos[1], t.pos[2], 0, 0, -1, 6);
    this.baroDrift += n.baroRW * Math.sqrt(Math.max(t.dt, 0)) * rng.gaussian();
    const alt = {
      down: down ? down.dist + n.altSigma * rng.gaussian() : null,
      baro: t.pos[2] + this.baroDrift + n.baroSigma * rng.gaussian(),
    };
    this.ticks += 1;
    return { imu: { dt: t.dt, gyro: [0, 0, gyroZ], accel }, flow, scan, alt };
  }
}

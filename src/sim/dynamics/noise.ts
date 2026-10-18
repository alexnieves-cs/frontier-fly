import type { NoiseParams } from '../../shared/types';
import type { RNG } from '../../shared/rng';

/**
 * IMU / baro / lidar noise. When every sigma is 0 and latency is 0, outputs
 * match the inputs and the RNG is not consumed.
 */
export class SensorNoise {
  private params: NoiseParams;
  private rng: RNG;
  private readonly bg = [0, 0, 0];
  private readonly ba = [0, 0, 0];
  private latBuf: unknown[] | null = null;
  private latI = 0;
  private latN = 0;

  constructor(params: NoiseParams, rng: RNG) {
    this.params = params;
    this.rng = rng;
  }

  reset(params: NoiseParams, rng: RNG): void {
    this.params = params;
    this.rng = rng;
    this.bg[0] = 0;
    this.bg[1] = 0;
    this.bg[2] = 0;
    this.ba[0] = 0;
    this.ba[1] = 0;
    this.ba[2] = 0;
    this.latBuf = null;
    this.latI = 0;
    this.latN = 0;
  }

  gyro(truth: [number, number, number], dt: number): [number, number, number] {
    this.walk(this.bg, dt);
    return this.corrupt(truth, this.bg, this.params.gyroSigma);
  }

  accel(truth: [number, number, number], dt: number): [number, number, number] {
    this.walk(this.ba, dt);
    return this.corrupt(truth, this.ba, this.params.accelSigma);
  }

  baro(altitude: number): number {
    const sig = this.params.baroSigma;
    if (sig > 0) return altitude + sig * this.rng.gaussian();
    return altitude;
  }

  /**
   * Range in meters. Dropout probability is
   * lidarDropout + 0.35 on glass + 0.05 when nightFactor > 0.6.
   * A dropout returns `maxRange` (a miss). Sigma is applied in meters.
   */
  lidarMeters(meters: number, maxRange: number, glass: boolean, nightFactor: number): number {
    const p =
      this.params.lidarDropout + (glass ? 0.35 : 0) + (nightFactor > 0.6 ? 0.05 : 0);
    if (p > 0 && this.rng.bool(p)) return maxRange;
    const sig = this.params.lidarSigma;
    if (sig > 0) return meters + sig * this.rng.gaussian();
    return meters;
  }

  /**
   * Delay by `latencyTicks`, holding the first sample.
   * output[k] = input[max(0, k - latencyTicks)]. latency 0 returns `value` immediately.
   */
  latency<T>(value: T): T {
    const n = this.params.latencyTicks | 0;
    if (n <= 0) return value;
    let buf = this.latBuf;
    if (buf === null || this.latN !== n) {
      buf = new Array<unknown>(n);
      for (let i = 0; i < n; i++) buf[i] = value;
      this.latBuf = buf;
      this.latI = 0;
      this.latN = n;
      return value;
    }
    const out = buf[this.latI] as T;
    buf[this.latI] = value;
    this.latI += 1;
    if (this.latI >= n) this.latI = 0;
    return out;
  }

  private walk(bias: number[], dt: number): void {
    const rw = this.params.biasRW * this.params.lowLightImu;
    if (!(rw > 0) || !(dt > 0)) return;
    const s = Math.sqrt(dt) * rw;
    bias[0] += s * this.rng.gaussian();
    bias[1] += s * this.rng.gaussian();
    bias[2] += s * this.rng.gaussian();
  }

  private corrupt(
    truth: [number, number, number],
    bias: number[],
    sigma: number,
  ): [number, number, number] {
    const b0 = bias[0] ?? 0;
    const b1 = bias[1] ?? 0;
    const b2 = bias[2] ?? 0;
    if (sigma > 0) {
      return [
        truth[0] + b0 + sigma * this.rng.gaussian(),
        truth[1] + b1 + sigma * this.rng.gaussian(),
        truth[2] + b2 + sigma * this.rng.gaussian(),
      ];
    }
    if (b0 === 0 && b1 === 0 && b2 === 0) return [truth[0], truth[1], truth[2]];
    return [truth[0] + b0, truth[1] + b1, truth[2] + b2];
  }
}

import { describe, expect, it } from 'vitest';
import { QuadrotorDynamics } from '../../src/sim/dynamics/quad';
import { defaultDynParams, sampleDynParams, sampleNoise, zeroNoise } from '../../src/sim/dynamics/dr';
import { SensorNoise } from '../../src/sim/dynamics/noise';
import { quatFromEulerZXY, quatToEuler, rotateBodyToWorld } from '../../src/sim/dynamics/spatial';
import { RNG } from '../../src/shared/rng';
import { GRAVITY, MAX_SPEED_H, MAX_SPEED_V, MAX_YAW_RATE } from '../../src/shared/constants';
import type { DroneState } from '../../src/shared/types';

function makeState(z = 1): DroneState {
  return {
    pos: [0, 0, z],
    vel: [0, 0, 0],
    yaw: 0,
    t: 0,
    quat: [0, 0, 0, 1],
    motors: new Float32Array(4),
    roll: 0,
    pitch: 0,
  };
}

function forwardSpeed(s: DroneState): number {
  const c = Math.cos(s.yaw);
  const sy = Math.sin(s.yaw);
  return s.vel[0] * c + s.vel[1] * sy;
}

describe('quadrotor dynamics', () => {
  it('hovers within 5 cm for 10 s', () => {
    const dyn = new QuadrotorDynamics();
    const state = makeState(1);
    dyn.reset(state, defaultDynParams());
    const cmd = new Float32Array([0, 0, 0, 0]);
    const dt = 1 / 400;
    let maxAbsZ = 0;
    let maxAbsX = 0;
    let maxAbsY = 0;
    let maxAbsV = 0;
    for (let i = 0; i < 4000; i++) {
      dyn.step(cmd, dt);
      const sample = (i + 1) % 200 === 0;
      const ax = Math.abs(state.pos[0]);
      const ay = Math.abs(state.pos[1]);
      const az = Math.abs(state.pos[2] - 1);
      const av = Math.max(Math.abs(state.vel[0]), Math.abs(state.vel[1]), Math.abs(state.vel[2]));
      if (ax > maxAbsX) maxAbsX = ax;
      if (ay > maxAbsY) maxAbsY = ay;
      if (az > maxAbsZ) maxAbsZ = az;
      if (av > maxAbsV) maxAbsV = av;
      if (sample) {
        expect(Math.abs(state.pos[2] - 1)).toBeLessThan(0.05);
        expect(Math.abs(state.pos[0])).toBeLessThan(0.05);
        expect(Math.abs(state.pos[1])).toBeLessThan(0.05);
        expect(Math.abs(state.vel[0])).toBeLessThan(0.2);
        expect(Math.abs(state.vel[1])).toBeLessThan(0.2);
        expect(Math.abs(state.vel[2])).toBeLessThan(0.2);
      }
    }
    expect(Math.abs(state.pos[2] - 1)).toBeLessThan(0.05);
    console.log(
      JSON.stringify({
        hoverMaxAbsZ: maxAbsZ,
        hoverMaxAbsX: maxAbsX,
        hoverMaxAbsY: maxAbsY,
        hoverMaxAbsV: maxAbsV,
      }),
    );
  });

  it('tracks a 1 m/s forward step with rise < 0.6 s and overshoot < 20%', () => {
    const dyn = new QuadrotorDynamics();
    const state = makeState(1);
    dyn.reset(state, defaultDynParams());
    const cmd = new Float32Array([0, 0, 0, 0]);
    const dt = 1 / 400;
    for (let i = 0; i < 400; i++) dyn.step(cmd, dt);
    cmd[0] = 1;
    let rise = -1;
    let maxV = -Infinity;
    const steps = Math.round(2 / dt);
    for (let i = 0; i < steps; i++) {
      dyn.step(cmd, dt);
      const vf = forwardSpeed(state);
      if (vf > maxV) maxV = vf;
      if (rise < 0 && vf >= 0.9) rise = (i + 1) * dt;
    }
    console.log(JSON.stringify({ rise, overshoot: maxV, endVf: forwardSpeed(state), z: state.pos[2] }));
    expect(rise).toBeGreaterThanOrEqual(0);
    expect(rise).toBeLessThan(0.6);
    expect(maxV).toBeLessThan(1.2);
  });

  it('survives 1e6 random steps without NaN', () => {
    const dyn = new QuadrotorDynamics();
    const state = makeState(1);
    dyn.reset(state, defaultDynParams());
    const rng = new RNG(123);
    const cmd = new Float32Array(4);
    const dt = 1 / 400;
    const t0 = performance.now();
    for (let i = 0; i < 1_000_000; i++) {
      cmd[0] = rng.range(-1, 1) * MAX_SPEED_H;
      cmd[1] = rng.range(-1, 1) * MAX_SPEED_H;
      cmd[2] = rng.range(-1, 1) * MAX_SPEED_V;
      cmd[3] = rng.range(-1, 1) * MAX_YAW_RATE;
      dyn.step(cmd, dt);
    }
    const ms = performance.now() - t0;
    console.log(
      JSON.stringify({
        millionMs: ms,
        pos: [state.pos[0], state.pos[1], state.pos[2]],
      }),
    );
    expect(Number.isFinite(state.pos[0])).toBe(true);
    expect(Number.isFinite(state.pos[1])).toBe(true);
    expect(Number.isFinite(state.pos[2])).toBe(true);
    expect(Number.isFinite(state.vel[0])).toBe(true);
    expect(Number.isFinite(state.vel[1])).toBe(true);
    expect(Number.isFinite(state.vel[2])).toBe(true);
    expect(Number.isFinite(state.quat[0])).toBe(true);
    expect(Number.isFinite(state.quat[1])).toBe(true);
    expect(Number.isFinite(state.quat[2])).toBe(true);
    expect(Number.isFinite(state.quat[3])).toBe(true);
    const mag = Math.hypot(state.pos[0], state.pos[1], state.pos[2]);
    expect(mag).toBeLessThan(1e5);
    expect(ms).toBeLessThan(30_000);
  });

  it('is deterministic across two instances', () => {
    const run = (): [number, number, number] => {
      const rng = new RNG(99);
      const dyn = new QuadrotorDynamics();
      const state = makeState(1);
      dyn.reset(state, defaultDynParams());
      const cmd = new Float32Array(4);
      const dt = 1 / 400;
      for (let i = 0; i < 500; i++) {
        cmd[0] = rng.range(-1, 1) * MAX_SPEED_H;
        cmd[1] = rng.range(-1, 1) * MAX_SPEED_H;
        cmd[2] = rng.range(-1, 1) * MAX_SPEED_V;
        cmd[3] = rng.range(-1, 1) * MAX_YAW_RATE;
        dyn.step(cmd, dt);
      }
      return [state.pos[0], state.pos[1], state.pos[2]];
    };
    expect(run()).toEqual(run());
  });
});

describe('domain randomization', () => {
  it('copies params at scale 0 and stays within 20% at scale 1', () => {
    const base = defaultDynParams();
    const hoverW = Math.sqrt((base.mass * GRAVITY) / 4 / base.kf);
    expect(hoverW).toBeCloseTo(700, 6);
    expect(base.km / base.kf).toBeCloseTo(0.02, 8);

    const rng0 = new RNG(1);
    const before = rng0.next();
    const rng0b = new RNG(1);
    const copied = sampleDynParams(rng0b, base, 0);
    expect(copied).toEqual(base);
    expect(copied).not.toBe(base);
    expect(copied.inertia).not.toBe(base.inertia);
    expect(rng0b.next()).toBe(before);

    const s1 = sampleDynParams(new RNG(12345), base, 1);
    expect(Math.abs(s1.mass - base.mass) / base.mass).toBeLessThanOrEqual(0.2 + 1e-12);
    expect(s1.arm).toBe(base.arm);
    expect(s1.inertia[0] / s1.mass).toBeCloseTo(base.inertia[0] / base.mass, 8);

    const a = sampleDynParams(new RNG(7), base, 1);
    const b = sampleDynParams(new RNG(7), base, 1);
    expect(a).toEqual(b);

    const n0 = sampleNoise(new RNG(4), 0, false);
    expect(n0.gyroSigma).toBe(0);
    expect(n0.accelSigma).toBe(0);
    expect(n0.biasRW).toBe(0);
    expect(n0.baroSigma).toBe(0);
    expect(n0.lidarSigma).toBe(0);
    expect(n0.lidarDropout).toBe(0);
    expect(n0.latencyTicks).toBe(0);
    expect(n0.lowLightImu).toBe(1);
    expect(sampleNoise(new RNG(4), 0, true).lowLightImu).toBe(1.5);
    expect(sampleNoise(new RNG(8), 1, true)).toEqual(sampleNoise(new RNG(8), 1, true));
  });
});

describe('sensor noise', () => {
  it('zero noise does not change values or consume rng', () => {
    const probe = new RNG(42);
    const expected = probe.next();
    const rng = new RNG(42);
    const noise = new SensorNoise(zeroNoise(), rng);
    expect(noise.gyro([1, 2, 3], 0.01)).toEqual([1, 2, 3]);
    expect(noise.accel([4, 5, 6], 0.0025)).toEqual([4, 5, 6]);
    expect(noise.baro(1.25)).toBe(1.25);
    expect(noise.lidarMeters(3.5, 8, false, 0)).toBe(3.5);
    expect(noise.lidarMeters(3.5, 8, false, 0.6)).toBe(3.5);
    expect(noise.latency(7)).toBe(7);
    expect(rng.next()).toBe(expected);
  });

  it('matches across two instances with the same seed', () => {
    const seq = (seed: number): number[] => {
      const params = sampleNoise(new RNG(seed), 1, false);
      const noise = new SensorNoise(params, new RNG(seed + 1));
      const out: number[] = [];
      for (let i = 0; i < 6; i++) {
        const g = noise.gyro([0.1, -0.2, 0.3], 0.01);
        const a = noise.accel([0, 0, -9.81], 0.01);
        out.push(g[0], g[1], g[2], a[0], a[1], a[2], noise.baro(2), noise.lidarMeters(4, 8, i % 2 === 0, 0.8));
        out.push(noise.latency(i));
      }
      return out;
    };
    expect(seq(5)).toEqual(seq(5));
  });

  it('returns max range when lidar dropout is 1', () => {
    const params = zeroNoise();
    params.lidarDropout = 1;
    const noise = new SensorNoise(params, new RNG(3));
    let misses = 0;
    for (let i = 0; i < 10; i++) {
      if (noise.lidarMeters(2, 8, false, 0) === 8) misses += 1;
    }
    expect(misses).toBeGreaterThan(0);
  });
});

describe('euler and quaternion', () => {
  it('identity is zero and yaw pi/2 maps body +X to world +Y', () => {
    const e0 = quatToEuler([0, 0, 0, 1]);
    expect(e0.roll).toBe(0);
    expect(e0.pitch).toBe(0);
    expect(e0.yaw).toBe(0);

    const q: [number, number, number, number] = [0, 0, 0, 1];
    quatFromEulerZXY(0, 0, Math.PI / 2, q);
    const v: [number, number, number] = [0, 0, 0];
    rotateBodyToWorld(q, [1, 0, 0], v);
    expect(Math.abs(v[0] - 0)).toBeLessThan(1e-3);
    expect(Math.abs(v[1] - 1)).toBeLessThan(1e-3);
    expect(Math.abs(v[2] - 0)).toBeLessThan(1e-3);
    const e = quatToEuler(q);
    expect(Math.abs(e.yaw - Math.PI / 2)).toBeLessThan(1e-3);
    expect(Math.abs(e.roll)).toBeLessThan(1e-3);
    expect(Math.abs(e.pitch)).toBeLessThan(1e-3);
  });
});

import { describe, expect, it } from 'vitest';
import { LIDAR_MAX } from '../../src/shared/constants';
import { createBattery, DEFAULT_BATTERY } from '../../src/mission/autonomy/battery';
import type { Phase } from '../../src/mission/autonomy/contracts';
import { createFailsafe, longestArc, type HealthInput } from '../../src/mission/autonomy/failsafe';
import { FAULT_SCENARIOS } from '../../src/mission/autonomy/faultScenarios';
import {
  authorityLimits,
  FaultInjector,
  headwindCostScale,
  sectorBeams,
  THRUST_TO_WEIGHT,
  type Fault,
} from '../../src/mission/autonomy/faults';
import { OUTDOOR_LIMITS } from '../../src/mission/autonomy/vehicle';

const DT = 0.05;

function run(inj: FaultInjector, untilSec: number, phase: (t: number) => Phase = () => 'coverage', each?: (t: number) => void): void {
  for (let k = inj.transitions().length || inj.tSec > 0 ? Math.round(inj.tSec / DT) + 1 : 0; k * DT <= untilSec + 1e-9; k++) {
    const t = k * DT;
    inj.advance(t, DT, phase(t));
    each?.(t);
  }
}

describe('fault injector', () => {
  it('starts and ends faults on their triggers and logs the edges', () => {
    const inj = new FaultInjector(
      [
        { id: 'link', kind: 'comms_loss', trigger: { atSec: 2 }, durationSec: 3 },
        { id: 'cam', kind: 'camera_dropout', trigger: { phase: 'indoor_explore', afterSec: 1 }, durationSec: Infinity },
      ],
      1,
    );
    const linkAt: boolean[] = [];
    run(inj, 10, (t) => (t < 6 ? 'coverage' : 'indoor_explore'), () => linkAt.push(inj.linkUp()));
    expect(linkAt[Math.round(1.9 / DT)]).toBe(true);
    expect(linkAt[Math.round(2.1 / DT)]).toBe(false);
    expect(linkAt[Math.round(5.1 / DT)]).toBe(true);
    expect(inj.cameraUp()).toBe(false);
    expect(inj.startOf('cam')).toBeCloseTo(7, 5);
    const edges = inj.transitions().map((e) => `${e.id}:${e.edge}`);
    expect(edges).toEqual(['link:start', 'link:end', 'cam:start']);
  });

  it('never fires a phase trigger for a phase the drone never enters', () => {
    const inj = new FaultInjector([{ id: 'cam', kind: 'camera_dropout', trigger: { phase: 'confirm', afterSec: 0 }, durationSec: 5 }], 1);
    run(inj, 20);
    expect(inj.startOf('cam')).toBeNull();
    expect(inj.cameraUp()).toBe(true);
  });

  it('rejects duplicate fault ids', () => {
    const f: Fault = { id: 'x', kind: 'comms_loss', trigger: { atSec: 0 }, durationSec: 1 };
    expect(() => new FaultInjector([f, f], 1)).toThrow(/Duplicate/);
  });

  it('is deterministic per seed and independent across fault kinds', () => {
    const wind: Fault = { id: 'w', kind: 'wind', trigger: { atSec: 0 }, durationSec: Infinity, meanMps: 3, dirDeg: 45, gustMps: 1.5, gustTauSec: 2 };
    const gnss: Fault = { id: 'g', kind: 'gnss_loss', trigger: { atSec: 0 }, durationSec: Infinity, driftMps: 0.5 };
    const trace = (faults: Fault[], seed: number) => {
      const inj = new FaultInjector(faults, seed);
      const out: number[] = [];
      run(inj, 30, undefined, () => out.push(inj.wind()[0]));
      return out;
    };
    expect(trace([wind], 5)).toEqual(trace([wind], 5));
    expect(trace([wind], 5)).not.toEqual(trace([wind], 6));
    // Adding a GNSS fault does not disturb the wind draws.
    expect(trace([wind, gnss], 5)).toEqual(trace([wind], 5));
  });

  it('produces wind with the configured mean and gust spread', () => {
    const inj = new FaultInjector([{ id: 'w', kind: 'wind', trigger: { atSec: 0 }, durationSec: Infinity, meanMps: 4, dirDeg: 90, gustMps: 1, gustTauSec: 2 }], 3);
    const xs: number[] = [];
    const ys: number[] = [];
    run(inj, 1000, undefined, () => {
      const w = inj.wind();
      xs.push(w[0]);
      ys.push(w[1]);
    });
    const mean = (a: number[]) => a.reduce((s, v) => s + v, 0) / a.length;
    const sd = (a: number[]) => Math.sqrt(mean(a.map((v) => (v - mean(a)) ** 2)));
    expect(mean(xs)).toBeCloseTo(0, 0);
    expect(mean(ys)).toBeGreaterThan(3.6);
    expect(mean(ys)).toBeLessThan(4.4);
    expect(sd(xs)).toBeGreaterThan(0.75);
    expect(sd(xs)).toBeLessThan(1.25);
  });

  it('applies flagged and silent lidar sectors in the body frame', () => {
    expect(sectorBeams(32, 0, 90)).toEqual([0, 1, 2, 3, 4, 28, 29, 30, 31]);
    expect(sectorBeams(32, 90, 22.5)).toEqual([7, 8, 9]);
    const inj = new FaultInjector(
      [
        { id: 'a', kind: 'lidar_sector', trigger: { atSec: 0 }, durationSec: Infinity, centerDeg: 0, widthDeg: 45, mode: 'flagged' },
        { id: 'b', kind: 'lidar_sector', trigger: { atSec: 0 }, durationSec: Infinity, centerDeg: 180, widthDeg: 45, mode: 'silent' },
      ],
      1,
    );
    inj.advance(0, DT, 'indoor_explore');
    const ring = inj.lidarRing(new Float64Array(32).fill(2));
    expect(Number.isNaN(ring[0]!)).toBe(true);
    expect(Number.isNaN(ring[2]!)).toBe(true);
    expect(ring[16]).toBe(LIDAR_MAX);
    expect(ring[8]).toBe(2);
  });

  it('adds lidar noise and misses at the configured rates', () => {
    const inj = new FaultInjector([{ id: 'n', kind: 'lidar_noise', trigger: { atSec: 0 }, durationSec: Infinity, sigmaM: 0.2, missRate: 0.1 }], 9);
    inj.advance(0, DT, 'indoor_explore');
    let misses = 0;
    const errs: number[] = [];
    for (let k = 0; k < 400; k++) {
      const ring = inj.lidarRing(new Array<number>(32).fill(3));
      for (const r of ring) {
        if (r === LIDAR_MAX) misses += 1;
        else errs.push(r - 3);
      }
    }
    expect(misses / (400 * 32)).toBeGreaterThan(0.08);
    expect(misses / (400 * 32)).toBeLessThan(0.12);
    const sd = Math.sqrt(errs.reduce((s, e) => s + e * e, 0) / errs.length);
    expect(sd).toBeGreaterThan(0.17);
    expect(sd).toBeLessThan(0.23);
  });

  it('freezes or drops odometry and resumes after the fault', () => {
    const inj = new FaultInjector(
      [
        { id: 'f', kind: 'odometry_freeze', trigger: { atSec: 1 }, durationSec: 1, stream: 'flow', mode: 'frozen' },
        { id: 'i', kind: 'odometry_freeze', trigger: { atSec: 3 }, durationSec: 1, stream: 'imu', mode: 'dropped' },
      ],
      1,
    );
    const flow: (number | null)[] = [];
    const imu: (number | null)[] = [];
    let k = 0;
    run(inj, 5, undefined, () => {
      k += 1;
      flow.push(inj.odometry('flow', k));
      imu.push(inj.odometry('imu', k));
    });
    const at = (t: number) => Math.round(t / DT);
    expect(flow[at(1.5)]).toBe(flow[at(1)! - 1]);
    expect(flow[at(2.5)]).toBe(at(2.5) + 1);
    expect(imu[at(3.5)]).toBeNull();
    expect(imu[at(4.5)]).toBe(at(4.5) + 1);
  });

  it('grows GNSS error while the fix is lost and clears it when it returns', () => {
    const inj = new FaultInjector([{ id: 'g', kind: 'gnss_loss', trigger: { atSec: 0 }, durationSec: 30, driftMps: 0.4 }], 2);
    run(inj, 29);
    const lost = inj.gnss();
    expect(lost.fix).toBe(false);
    const err = Math.hypot(lost.errorM[0], lost.errorM[1]);
    expect(err).toBeGreaterThan(4);
    expect(err).toBeLessThan(25);
    run(inj, 31);
    expect(inj.gnss()).toEqual({ fix: true, errorM: [0, 0, 0] });
  });

  it('drains a sagging pack faster while the model still predicts healthy costs', () => {
    const healthy = createBattery();
    const inj = new FaultInjector([{ id: 'p', kind: 'battery_sag', trigger: { atSec: 10 }, durationSec: 1, dropPct: 20, drawScale: 1.5 }], 1);
    const bat = inj.wrapBattery(createBattery());
    run(inj, 60, undefined, () => {
      healthy.draw(150, DT);
      bat.draw(150, DT);
    });
    // Permanent despite durationSec: a weak cell does not heal.
    expect(inj.isActive('p')).toBe(true);
    const usedHealthy = 100 - healthy.remainingPct();
    const usedFaulty = 100 - bat.remainingPct();
    const expected = 20 + usedHealthy * (10 / 60) + usedHealthy * (50 / 60) * 1.5;
    expect(usedFaulty).toBeCloseTo(expected, 0);
    expect(bat.costPct(100, 5)).toBeCloseTo(healthy.costPct(100, 5), 9);
  });
});

describe('control authority', () => {
  it('leaves a healthy airframe untouched', () => {
    const r = authorityLimits(OUTDOOR_LIMITS, [1, 1, 1, 1]);
    expect(r.limits.speedH).toBeCloseTo(OUTDOOR_LIMITS.speedH, 9);
    expect(r.limits.accel).toBeCloseTo(OUTDOOR_LIMITS.accel, 9);
    expect(r.thrustToWeight).toBe(THRUST_TO_WEIGHT);
    expect(r.sinkMps).toBe(0);
  });

  it('shrinks caps monotonically as the weakest motor weakens', () => {
    let prev = authorityLimits(OUTDOOR_LIMITS, [1]);
    for (const e of [0.9, 0.8, 0.7, 0.6, 0.55]) {
      const r = authorityLimits(OUTDOOR_LIMITS, [1, e, 1, 1]);
      expect(r.limits.speedH).toBeLessThan(prev.limits.speedH);
      expect(r.limits.accel).toBeLessThan(prev.limits.accel);
      expect(r.sinkMps).toBe(0);
      prev = r;
    }
  });

  it('cannot hover below half thrust with a 2:1 airframe', () => {
    const r = authorityLimits(OUTDOOR_LIMITS, [0.45, 1, 1, 1]);
    expect(r.thrustToWeight).toBeLessThan(1);
    expect(r.sinkMps).toBeGreaterThan(0);
    expect(r.limits.speedH).toBe(0);
  });

  it('prices headwinds and refuses a wind faster than cruise', () => {
    expect(headwindCostScale(8, 0)).toBe(1);
    expect(headwindCostScale(8, 4)).toBeCloseTo(2, 9);
    expect(headwindCostScale(8, -3)).toBe(1);
    expect(headwindCostScale(8, 8)).toBe(Infinity);
  });
});

function healthy(over: Partial<HealthInput> = {}): HealthInput {
  return {
    tSec: 0,
    dt: DT,
    phase: 'coverage',
    inside: null,
    linkUp: true,
    cameraFrame: true,
    lidar: new Float64Array(32).fill(3),
    lidarExpected: new Float64Array(32).fill(3),
    gnssFix: true,
    poseHealth: null,
    motorRatio: [1, 1, 1, 1],
    batteryPct: 90,
    modelPowerW: 150,
    usableWh: DEFAULT_BATTERY.capacityWh * DEFAULT_BATTERY.usableFraction,
    costHomePct: 5,
    windEstimate: [0, 0, 0],
    toHome: [1, 0],
    cruiseMps: 5.5,
    ...over,
  };
}

/** Feed the failsafe a stream of inputs; returns each verdict. */
function feed(seconds: number, at: (t: number) => Partial<HealthInput>, config = {}) {
  const fs = createFailsafe(config);
  const usable = DEFAULT_BATTERY.capacityWh * DEFAULT_BATTERY.usableFraction;
  let pct = 90;
  const out = [];
  for (let k = 0; k * DT <= seconds + 1e-9; k++) {
    const t = k * DT;
    const over = at(t);
    pct -= ((150 * DT) / 3600 / usable) * 100;
    out.push({ t, v: fs.check(healthy({ tSec: t, batteryPct: pct, ...over })) });
  }
  return { fs, out };
}

describe('failsafe', () => {
  it('stays quiet on a healthy drone', () => {
    const { fs, out } = feed(120, () => ({}));
    expect(fs.events()).toEqual([]);
    expect(out.every((o) => o.v.action.kind === 'none' && o.v.speedScale === 1 && o.v.costScale === 1)).toBe(true);
  });

  it('rides out a short link loss and returns home after a long one', () => {
    const short = feed(30, (t) => ({ linkUp: !(t >= 5 && t < 10) }));
    expect(short.out.every((o) => o.v.action.kind === 'none')).toBe(true);
    expect(short.out.find((o) => o.t >= 6)!.v.denyAsks).toBe(true);
    expect(short.out.find((o) => o.t >= 11)!.v.denyAsks).toBe(false);
    expect(short.fs.events().map((e) => e.kind)).toEqual(['comms_loss', 'comms_loss']);

    const long = feed(40, (t) => ({ linkUp: t < 5 }));
    const first = long.out.find((o) => o.v.action.kind !== 'none')!;
    expect(first.v.action.kind).toBe('return_home');
    expect(first.t).toBeCloseTo(25, 1);
    // Inside a building it waits longer (walls block the link), then exits first.
    const inside = feed(80, (t) => ({ linkUp: t < 5, inside: 'bldg-0', phase: 'indoor_explore' }));
    const firstIn = inside.out.find((o) => o.v.action.kind !== 'none')!;
    expect(firstIn.v.action.kind).toBe('exit_building');
    expect(firstIn.t).toBeCloseTo(65, 1);
  });

  it('treats flagged lidar beams as unknown and leaves the building for a wide blind arc', () => {
    const ring = (t: number) => {
      const r = new Float64Array(32).fill(3);
      if (t >= 2) for (const i of sectorBeams(32, 0, 90)) r[i] = NaN;
      return r;
    };
    const { out, fs } = feed(5, (t) => ({ lidar: ring(t), inside: 'bldg-0', phase: 'indoor_explore' }));
    const hit = out.find((o) => o.v.action.kind !== 'none')!;
    expect(hit.t).toBeGreaterThanOrEqual(2.2 - 1e-6);
    expect(hit.t).toBeLessThan(2.3);
    expect(hit.v.action.kind).toBe('exit_building');
    expect(hit.v.unknownBeams).toEqual(sectorBeams(32, 0, 90).sort((a, b) => a - b));
    expect(hit.v.speedScale).toBeLessThan(1);
    expect(fs.events().filter((e) => e.kind === 'sensor_fault')).toHaveLength(1);
    // A narrow arc only slows the drone down.
    const narrow = feed(5, (t) => {
      const r = new Float64Array(32).fill(3);
      if (t >= 2) r[4] = NaN;
      return { lidar: r, inside: 'bldg-0', phase: 'indoor_explore' };
    });
    expect(narrow.out.every((o) => o.v.action.kind === 'none')).toBe(true);
    expect(narrow.out[narrow.out.length - 1]!.v.speedScale).toBeLessThan(1);
  });

  it('catches a silent lidar arc that reads open where the map has walls, and latches it', () => {
    const beams = sectorBeams(32, 90, 60);
    const { out } = feed(5, (t) => {
      const r = new Float64Array(32).fill(3);
      if (t >= 1 && t < 3) for (const i of beams) r[i] = LIDAR_MAX;
      return { lidar: r, lidarExpected: new Float64Array(32).fill(3), inside: 'bldg-0', phase: 'indoor_explore' };
    });
    const hit = out.find((o) => o.v.flags.has('lidar_silent'))!;
    expect(hit.t).toBeGreaterThan(1);
    // 20 scans at one scan per tick.
    expect(hit.t).toBeLessThan(1 + 21 * DT);
    expect(hit.v.unknownBeams).toEqual(beams);
    expect(out[out.length - 1]!.v.unknownBeams).toEqual(beams);
  });

  it('does not call a long corridor a silent fault', () => {
    // The map predicts nothing down the corridor, so open returns there are expected.
    const exp = new Float64Array(32).fill(3);
    for (const i of sectorBeams(32, 0, 30)) exp[i] = Infinity;
    const { out } = feed(10, () => {
      const r = new Float64Array(32).fill(3);
      for (const i of sectorBeams(32, 0, 30)) r[i] = LIDAR_MAX;
      return { lidar: r, lidarExpected: exp, inside: 'bldg-0', phase: 'indoor_explore' };
    });
    expect(out.every((o) => !o.v.flags.has('lidar_silent'))).toBe(true);
  });

  it('keeps searching through a short camera dropout and gives up on a long one', () => {
    const short = feed(20, (t) => ({ cameraFrame: !(t >= 5 && t < 9) }));
    expect(short.out.every((o) => o.v.action.kind === 'none')).toBe(true);
    expect(short.out.some((o) => o.v.flags.has('camera_lost'))).toBe(true);
    const long = feed(20, (t) => ({ cameraFrame: t < 5 }));
    const hit = long.out.find((o) => o.v.action.kind !== 'none')!;
    expect(hit.v.action.kind).toBe('return_home');
    expect(hit.t).toBeCloseTo(15, 1);
  });

  it('flags the outage, then returns home, then lands as a GNSS outage drags on', () => {
    const { out } = feed(120, (t) => ({ gnssFix: t < 10 }));
    const at = (t: number) => out.find((o) => o.t >= t - 1e-9)!.v.action.kind;
    expect(at(9)).toBe('none');
    // The first seconds are the runner's climb-clear wait, signalled by the flag.
    expect(at(11)).toBe('none');
    expect(out.find((o) => o.t >= 11)!.v.flags.has('gnss_lost')).toBe(true);
    expect(at(16)).toBe('return_home');
    expect(at(101)).toBe('land');
    // Inside a building GNSS is expected to be absent.
    const inside = feed(30, () => ({ gnssFix: false, inside: 'bldg-0', phase: 'indoor_explore' }));
    expect(inside.out.every((o) => o.v.action.kind === 'none')).toBe(true);
  });

  it('leaves a building when the pose estimate degrades and lands when it is lost', () => {
    const { out } = feed(10, (t) => ({
      inside: 'bldg-0',
      phase: 'indoor_explore',
      poseHealth: t < 2 ? { ok: true, sigmaM: 0.05 } : t < 6 ? { ok: false, sigmaM: 0.5 } : { ok: false, sigmaM: 1.4, state: 'lost' as const },
    }));
    const at = (t: number) => out.find((o) => o.t >= t - 1e-9)!.v;
    expect(at(2.5).action.kind).toBe('none');
    expect(at(3.1).action.kind).toBe('exit_building');
    expect(at(3.1).speedScale).toBeLessThan(1);
    expect(at(6.1).action.kind).toBe('land');
  });

  it('goes home on a weak motor and lands on a severe one', () => {
    const weak = feed(10, (t) => ({ motorRatio: t < 3 ? [1, 1, 1, 1] : [1, 0.78, 1, 1] }));
    const w = weak.out.find((o) => o.v.action.kind !== 'none')!;
    expect(w.v.action.kind).toBe('return_home');
    expect(w.t).toBeCloseTo(3.3, 1);
    expect(w.v.speedScale).toBeLessThan(1);
    // Latched: a motor that faltered is not trusted again.
    const blip = feed(10, (t) => ({ motorRatio: t >= 3 && t < 4 ? [0.8, 1, 1, 1] : [1, 1, 1, 1] }));
    expect(blip.out[blip.out.length - 1]!.v.action.kind).toBe('return_home');
    const severe = feed(10, (t) => ({ motorRatio: t < 3 ? [1, 1, 1, 1] : [1, 1, 0.55, 1] }));
    expect(severe.out.find((o) => o.v.action.kind !== 'none')!.v.action.kind).toBe('land');
    // A momentary dip shorter than the detection time is ignored.
    const dip = feed(10, (t) => ({ motorRatio: t >= 3 && t < 3.2 ? [0.7, 1, 1, 1] : [1, 1, 1, 1] }));
    expect(dip.out.every((o) => o.v.action.kind === 'none')).toBe(true);
  });

  it('notices a gauge step and a fast drain, and returns home when the scaled reserve runs out', () => {
    const usable = DEFAULT_BATTERY.capacityWh * DEFAULT_BATTERY.usableFraction;
    const perTick = ((150 * DT) / 3600 / usable) * 100;
    const fs = createFailsafe();
    let pct = 90;
    let stepAt: number | null = null;
    let homeAt: number | null = null;
    for (let k = 0; k * DT <= 600; k++) {
      const t = k * DT;
      pct -= perTick * (t >= 100 ? 1.4 : 1);
      if (Math.abs(t - 100) < 1e-9) pct -= 25;
      const v = fs.check(healthy({ tSec: t, batteryPct: pct, costHomePct: 20 }));
      if (stepAt === null && v.flags.has('battery_anomaly')) stepAt = t;
      if (homeAt === null && v.action.kind === 'return_home') homeAt = t;
    }
    expect(stepAt).toBeCloseTo(100, 1);
    expect(homeAt).not.toBeNull();
    const ev = fs.events().filter((e) => e.kind === 'battery_fault');
    expect(ev.length).toBeGreaterThanOrEqual(2);
    expect(ev.some((e) => /faster/.test(e.message))).toBe(true);
  });

  it('returns home in sustained strong wind and prices a headwind', () => {
    // The limit applies to the 20 s mean: 5 m/s reaches 3 m/s mean after 20 ln 2.5 = 18.3 s, plus 5 s persistence.
    const { out } = feed(40, () => ({ windEstimate: [5, 0, 0], toHome: [1, 0] }));
    const hit = out.find((o) => o.v.action.kind !== 'none')!;
    expect(hit.v.action.kind).toBe('return_home');
    expect(hit.t).toBeCloseTo(23.3, 0);
    // Strong wind doesn't wait for the mean: 2 s over 6 m/s on the short estimate.
    const strong = feed(10, () => ({ windEstimate: [7, 0, 0], toHome: [1, 0] }));
    const fast = strong.out.find((o) => o.v.action.kind !== 'none')!;
    expect(fast.v.action.kind).toBe('return_home');
    expect(fast.t).toBeCloseTo(2, 0);
    expect(fast.v.flags.has('wind_severe')).toBe(true);
    // Gusts alone do not trip it: a 2 m/s mean with 4 m/s peaks every few seconds.
    const gusty = feed(120, (t) => ({ windEstimate: [2 + 2 * Math.sin(t * 2), 0, 0] }));
    expect(gusty.out.every((o) => o.v.action.kind === 'none')).toBe(true);
    // A tailwind home costs nothing extra; a headwind does, once the mean has built up.
    const head = feed(200, () => ({ windEstimate: [-2, 0, 0], toHome: [1, 0] }));
    expect(head.out[head.out.length - 1]!.v.costScale).toBeCloseTo(5.5 / 3.5, 2);
    const tail = feed(200, () => ({ windEstimate: [2, 0, 0], toHome: [1, 0] }));
    expect(tail.out[tail.out.length - 1]!.v.costScale).toBe(1);
  });

  it('lands when it makes no headway home in wind', () => {
    // Going home into 7 m/s and stuck 50 m out: after the 2 s trigger, 30 s without closing 3 m.
    const { out } = feed(45, () => ({ windEstimate: [-7, 0, 0], toHome: [1, 0], phase: 'return_home', homeDistM: 50 }));
    const land = out.find((o) => o.v.action.kind === 'land')!;
    expect(land.t).toBeCloseTo(32, 0);
    // Closing on home, however slowly, keeps it going.
    const slow = feed(60, (t) => ({ windEstimate: [-7, 0, 0], toHome: [1, 0], phase: 'return_home', homeDistM: 50 - t * 0.2 }));
    expect(slow.out.every((o) => o.v.action.kind !== 'land')).toBe(true);
  });

  it('latches severe wind through gust dips and lets it go after a calm spell', () => {
    // 8 m/s with dips to 5 m/s: the short estimate drops under 6 m/s but stays over 4 m/s, so the flag holds.
    const dips = feed(30, (t) => ({ windEstimate: [t > 10 && t < 12 ? 5 : 8, 0, 0], toHome: [1, 0] }));
    const after = dips.out.filter((o) => o.t > 2.5);
    expect(after.every((o) => o.v.flags.has('wind_severe'))).toBe(true);
    // Under 4 m/s for 5 s clears it.
    const calm = feed(30, (t) => ({ windEstimate: [t < 10 ? 8 : 1, 0, 0], toHome: [1, 0] }));
    expect(calm.out.find((o) => o.t > 12 && o.t < 14.5)!.v.flags.has('wind_severe')).toBe(true);
    expect(calm.out.find((o) => o.t > 16)!.v.flags.has('wind_severe')).toBe(false);
  });

  it('does not call a hover over the pad a lack of headway', () => {
    const { out } = feed(60, () => ({ windEstimate: [-7, 0, 0], toHome: [1, 0], phase: 'return_home', homeDistM: 2 }));
    expect(out.every((o) => o.v.action.kind !== 'land')).toBe(true);
  });

  it('suppresses actions once landing', () => {
    const { out } = feed(30, () => ({ linkUp: false, phase: 'land' }));
    expect(out.every((o) => o.v.action.kind === 'none')).toBe(true);
  });

  it('measures blind arcs across the ring seam', () => {
    const m = new Array<boolean>(8).fill(false);
    m[7] = m[0] = m[1] = true;
    expect(longestArc(m)).toBe(3);
    expect(longestArc(new Array<boolean>(8).fill(true))).toBe(8);
    expect(longestArc(new Array<boolean>(8).fill(false))).toBe(0);
  });
});

describe('fault scenarios', () => {
  it('have unique ids, known hazards, and seed-stable plans', () => {
    const ids = FAULT_SCENARIOS.map((s) => s.id);
    expect(new Set(ids).size).toBe(ids.length);
    for (const s of FAULT_SCENARIOS) {
      expect(s.safe).toContain(s.intended);
      for (const h of s.hazards) expect(h).toMatch(/^H\d+$/);
      expect(JSON.stringify(s.faults(9101))).toBe(JSON.stringify(s.faults(9101)));
      // Each plan builds a valid injector.
      expect(() => new FaultInjector(s.faults(9101), 9101)).not.toThrow();
    }
  });
});

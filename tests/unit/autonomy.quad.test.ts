import { beforeAll, describe, expect, it } from 'vitest';
import { createBattery } from '../../src/mission/autonomy/battery';
import type { V3 } from '../../src/mission/autonomy/contracts';
import { MISSION_QUAD_BRAKING, QuadVehicle } from '../../src/mission/autonomy/quadVehicle';
import { INDOOR_LIMITS, OUTDOOR_LIMITS, Vehicle } from '../../src/mission/autonomy/vehicle';
import { buildMissionWorld } from '../../src/mission/autonomy/world';
import { brakingFilter, safeSpeed } from '../../src/sim/sensorPlanner';
import { LIDAR_H, LIDAR_MAX } from '../../src/shared/constants';
import { initOutdoorPhysics, OutdoorPhysics } from '../../src/world/outdoor/physics';
import type { MissionWorld } from '../../src/mission/autonomy/contracts';

let world: MissionWorld;
let physics: OutdoorPhysics;

beforeAll(async () => {
  await initOutdoorPhysics();
  world = buildMissionWorld({ seed: 20 });
  physics = new OutdoorPhysics(world.outdoor);
});

/** A start point 25 m above home: open air, nothing to hit. */
function aloft(): V3 {
  return [world.home[0], world.home[1], world.home[2] + 25];
}

function horizontalSpeed(v: Vehicle): number {
  return Math.hypot(v.vel[0], v.vel[1]);
}

describe('quad mission vehicle', () => {
  it('hovers in place and holds a commanded speed', () => {
    const q = new QuadVehicle(physics, createBattery(), aloft(), 0, world.outdoor);
    const start = [...q.pos];
    for (let i = 0; i < 100; i++) q.step({ vel: [0, 0, 0], yaw: null }, 0.05, INDOOR_LIMITS);
    expect(Math.hypot(q.pos[0] - start[0]!, q.pos[1] - start[1]!, q.pos[2] - start[2]!)).toBeLessThan(0.1);
    for (let i = 0; i < 80; i++) q.step({ vel: [1, 0, 0], yaw: null }, 0.05, INDOOR_LIMITS);
    expect(q.vel[0]).toBeGreaterThan(0.85);
    expect(q.vel[0]).toBeLessThan(1.15);
    expect(Math.abs(q.vel[1])).toBeLessThan(0.1);
    expect(Math.abs(q.pos[2] - start[2]!)).toBeLessThan(0.2);
  });

  it('holds position in strong wind with the outdoor tilt limit, and keeps its tilt inside it', () => {
    // Outdoors the runner sets a tilt limit (phases/base.ts limits()), which also turns on the velocity
    // integral. The default 23° per axis holds only about 5 m/s on the 1.2 kg airframe.
    const drift = (limits: typeof OUTDOOR_LIMITS, wind: V3) => {
      const q = new QuadVehicle(physics, createBattery(), aloft(), 0, world.outdoor);
      for (let i = 0; i < 40; i++) q.step({ vel: [0, 0, 0], yaw: null }, 0.05, limits);
      let tilt = 0;
      for (let i = 0; i < 200; i++) {
        q.step({ vel: [0, 0, 0], yaw: null }, 0.05, limits, { wind });
        tilt = Math.max(tilt, Math.acos(Math.cos(q.attitude.roll) * Math.cos(q.attitude.pitch)));
      }
      return { speed: horizontalSpeed(q), tilt, vz: Math.abs(q.vel[2]) };
    };
    const windy = { ...OUTDOOR_LIMITS, maxTilt: 0.7 };
    expect(drift(OUTDOOR_LIMITS, [8, 0, 0]).speed).toBeGreaterThan(2);
    for (const wind of [[8, 0, 0], [5.7, 5.7, 0]] as V3[]) {
      const r = drift(windy, wind);
      expect(r.speed).toBeLessThan(1);
      // A quartering wind asks for roll and pitch together; the tilt vector stays inside the limit.
      expect(r.tilt).toBeLessThan(0.7 + 0.05);
      expect(r.vz).toBeLessThan(0.2);
    }
    // Below the limit the velocity integral takes out the downwind drift a P-only loop needs to lean.
    expect(drift(windy, [6, 0, 0]).speed).toBeLessThan(0.1);
    // 40° can't hold 10 m/s (gusts on the bench's 8 m/s wind); the 55° outdoor limit can.
    expect(drift(windy, [10, 0, 0]).speed).toBeGreaterThan(1);
    const r = drift({ ...OUTDOOR_LIMITS, maxTilt: 0.96 }, [10, 0, 0]);
    expect(r.speed).toBeLessThan(0.5);
    expect(r.tilt).toBeLessThan(0.96 + 0.05);
    expect(r.vz).toBeLessThan(0.2);
  });

  it('settles on a stop point under the 55° outdoor limit', () => {
    // The waypoint follower's stop loop (phases/base.ts followJob) on a point 1.5 m off. A full-gain
    // velocity loop leaned the whole 55° each way and circled the point at 1 m/s, never inside the
    // 0.12 m stop tolerance: compound sweeps hung at a building's standoff until the time ran out.
    for (const maxTilt of [0.7, 0.96]) {
      const limits = { ...OUTDOOR_LIMITS, maxTilt, shapeTilt: true };
      const q = new QuadVehicle(physics, createBattery(), aloft(), 0, world.outdoor);
      const target: V3 = [q.pos[0] + 1.5, q.pos[1], q.pos[2]];
      let worst = 0;
      for (let i = 0; i < 200; i++) {
        const off = [target[0] - q.pos[0], target[1] - q.pos[1], target[2] - q.pos[2]];
        const d = Math.hypot(off[0]!, off[1]!, off[2]!);
        const speed = Math.min(1.6 * d + 0.03, Math.sqrt(4 * d));
        const k = d > 1e-9 ? speed / d : 0;
        q.step({ vel: [off[0]! * k, off[1]! * k, off[2]! * k], yaw: null }, 0.05, limits);
        if (i >= 100) worst = Math.max(worst, d, Math.hypot(q.vel[0], q.vel[1], q.vel[2]));
      }
      expect(worst).toBeLessThan(0.12);
    }
  });

  it('keeps attitude control when a fast stop meets a full-thrust climb', () => {
    // The GNSS-loss climb: setting off, then asked to stop and climb at once. A mixer that
    // keeps the collective and drops the differential loses the attitude, and the airframe runs off.
    const q = new QuadVehicle(physics, createBattery(), aloft(), 0, world.outdoor);
    // Recorded in a gnss-lost flight: from a hover, two ticks of a turn-and-go, then the climb.
    for (let i = 0; i < 20; i++) q.step({ vel: [0, 0, 0], yaw: 0 }, 0.05, OUTDOOR_LIMITS);
    for (let i = 0; i < 2; i++) q.step({ vel: [2.08, -1.39, -0.04], yaw: -0.588 }, 0.05, OUTDOOR_LIMITS);
    let peak = 0;
    for (let i = 0; i < 40; i++) {
      q.step({ vel: [-0.5, 0, 1.2], yaw: null }, 0.05, OUTDOOR_LIMITS);
      peak = Math.max(peak, horizontalSpeed(q));
    }
    expect(peak).toBeLessThan(3);
    expect(Math.hypot(q.vel[0] + 0.5, q.vel[1])).toBeLessThan(0.3);
  });

  it('stops within the braking model from indoor and outdoor speeds', () => {
    // MISSION_QUAD_BRAKING was fitted on the coverage env's micro quad; check it holds on the 1.2 kg airframe.
    for (const [limits, v0] of [
      [INDOOR_LIMITS, 1],
      [OUTDOOR_LIMITS, 2],
      [OUTDOOR_LIMITS, 4],
    ] as const) {
      const q = new QuadVehicle(physics, createBattery(), aloft(), 0, world.outdoor);
      for (let i = 0; i < 120; i++) q.step({ vel: [v0, 0, 0], yaw: null }, 0.05, limits);
      const v = horizontalSpeed(q);
      expect(v).toBeGreaterThan(v0 * 0.85);
      const x0 = q.pos[0];
      for (let i = 0; i < 100; i++) q.step({ vel: [0, 0, 0], yaw: null }, 0.05, limits);
      const stop = q.pos[0] - x0;
      const m = MISSION_QUAD_BRAKING;
      const modelled = v * m.reaction + (v * v) / (2 * m.decel);
      expect(stop, `from ${v0} m/s`).toBeLessThan(modelled);
      // And the filter's speed at that distance gives the speed back: the model is self-consistent.
      expect(safeSpeed(modelled, m)).toBeCloseTo(v, 6);
    }
  });

  it('moves with the wind: the kinematic airframe flies through the air, the quad holds ground speed', () => {
    const wind: V3 = [3, 0, 0];
    // The kinematic command is an air velocity, so a zero command drifts with the wind; the mission
    // planners steer on position and take the drift out.
    const kin = new Vehicle(physics, createBattery(), aloft(), 0, world.outdoor);
    for (let i = 0; i < 200; i++) kin.step({ vel: [0, 0, 0], yaw: null }, 0.05, OUTDOOR_LIMITS, { wind });
    expect(kin.pos[0] - aloft()[0]).toBeCloseTo(30, 0);
    // The quad's velocity loop works on ground velocity: the gust pushes it, the loop leans into it.
    const quad = new QuadVehicle(physics, createBattery(), aloft(), 0, world.outdoor);
    for (let i = 0; i < 200; i++) quad.step({ vel: [0, 0, 0], yaw: null }, 0.05, OUTDOOR_LIMITS, { wind });
    const drift = quad.pos[0] - aloft()[0];
    expect(drift).toBeGreaterThan(0.05);
    expect(drift).toBeLessThan(5);
    expect(Math.abs(quad.vel[0])).toBeLessThan(0.5);
    expect(quad.attitude.pitch).toBeLessThan(-0.02);
  });

  it('scales thrust authority and reports an IMU reading', () => {
    const full = new QuadVehicle(physics, createBattery(), aloft(), 0, world.outdoor);
    const weak = new QuadVehicle(physics, createBattery(), aloft(), 0, world.outdoor);
    weak.setThrustAuthority(0.55);
    for (let i = 0; i < 60; i++) {
      full.step({ vel: [0, 0, 1.5], yaw: null }, 0.05, OUTDOOR_LIMITS);
      weak.step({ vel: [0, 0, 1.5], yaw: null }, 0.05, OUTDOOR_LIMITS);
    }
    expect(full.pos[2] - aloft()[2]).toBeGreaterThan(weak.pos[2] - aloft()[2] + 0.5);
    const imu = full.imu();
    // At steady climb the accelerometer reads about +1 g along body z.
    expect(imu.accelBody[2]).toBeGreaterThan(8);
    expect(imu.accelBody[2]).toBeLessThan(11.5);
    expect(imu.gyroBody.every(Number.isFinite)).toBe(true);
    const kin = new Vehicle(physics, createBattery(), aloft(), 0, world.outdoor);
    kin.step({ vel: [0, 0, 0], yaw: null }, 0.05, OUTDOOR_LIMITS);
    expect(kin.imu().accelBody[2]).toBeCloseTo(9.81, 1);
  });
});

describe('braking filter', () => {
  it('keeps full speed in the open, stops at the margin, and treats a dropped reading as blocked', () => {
    const open = new Float64Array(LIDAR_H).fill(LIDAR_MAX);
    const [fx] = brakingFilter(1, 0, 0, open, [1, 0], MISSION_QUAD_BRAKING);
    expect(fx).toBeCloseTo(1, 6);
    const wall = new Float64Array(LIDAR_H).fill(LIDAR_MAX);
    wall[0] = 0.4;
    expect(brakingFilter(1, 0, 0, wall, [1, 0], MISSION_QUAD_BRAKING)[0]).toBeLessThan(0.6);
    const dropped = new Float64Array(LIDAR_H).fill(LIDAR_MAX);
    dropped[0] = Number.NaN;
    expect(brakingFilter(1, 0, 0, dropped, [0, 0], MISSION_QUAD_BRAKING)[0]).toBeLessThanOrEqual(0);
  });
});

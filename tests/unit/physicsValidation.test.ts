import { describe, expect, it } from 'vitest';
import { AeroModel, ceilingEffect, groundEffect, legacyGroundEffect } from '../../src/sim/dynamics/aero';
import { GustTable } from '../../src/sim/dynamics/gust';
import { QuadrotorDynamics } from '../../src/sim/dynamics/quad';
import { makeDroneState } from '../../src/sim/drone';
import { missionQuadParams } from '../../src/mission/autonomy/quadVehicle';
import { cellOcv, MISSION_PACK, motorSpeedScale, packCurrent, Px4BatteryEstimator, px4VoltageSoc } from '../../src/mission/autonomy/pack';
import { FrontierPlant, nedQuat, px4Geometry } from '../../src/px4/frontierPlant';
import { PhysicsRig } from '../../src/px4/rig';
import { MANEUVERS, px4Params, sihDrag } from '../../scripts/physics/maneuvers';
import { flyNative } from '../../scripts/physics/native';

describe('near-surface aerodynamics', () => {
  it('matches Cheeseman-Bennett and the published small-rotor anchor', () => {
    const r = 0.127;
    expect(groundEffect(r, r)).toBeCloseTo(1 / (1 - 1 / 16), 6);
    // Sanchez-Cuevas et al. (ROBOT 2017) measured about 1.2 at z/R = 0.5; the classical model gives 1.33 there.
    expect(groundEffect(0.5 * r, r)).toBeCloseTo(4 / 3, 6);
    // Held below the floor distance, and gone far away.
    expect(groundEffect(0.01, r)).toBeCloseTo(groundEffect(0.5 * r, r), 9);
    expect(groundEffect(10, r)).toBeLessThan(1.0001);
  });

  it('ceiling fit: about 1.5x near one radius, held at its floor, 1 without a ceiling', () => {
    const r = 0.1194;
    // K1 = 6.924, K2 = 3.782 cm on the paper's 11.94 cm rotor.
    expect(ceilingEffect(0.03, r)).toBeCloseTo(1 / (1 - (r / (0.03 + 0.03782)) ** 2 / 6.924), 3);
    expect(ceilingEffect(0, r)).toBe(ceilingEffect(0.2 * r, r));
    expect(ceilingEffect(Infinity, r)).toBe(1);
    expect(ceilingEffect(r, r)).toBeGreaterThan(1.05);
  });

  it('legacy ground effect is what QuadrotorDynamics has always applied', () => {
    expect(legacyGroundEffect(1)).toBe(1);
    expect(legacyGroundEffect(0.2)).toBeCloseTo(1.016, 9);
    expect(legacyGroundEffect(-1)).toBeCloseTo(1.032, 9);
  });

  it('an AeroModel lifts a hovering quad near the ground and pulls it up under a ceiling', () => {
    const lift = (aero: AeroModel | null, z: number): number => {
      const dyn = new QuadrotorDynamics();
      const s = makeDroneState([0, 0, z]);
      dyn.reset(s, missionQuadParams());
      dyn.aero = aero;
      const w = Math.sqrt((missionQuadParams().mass * 9.81) / 4 / missionQuadParams().kf);
      dyn.stepPlant([w, w, w, w], 0.0025);
      return s.vel[2];
    };
    expect(Math.abs(lift(null, 3))).toBeLessThan(1e-3);
    expect(lift(new AeroModel({ rotorRadiusM: 0.127 }), 0.1)).toBeGreaterThan(0);
    expect(lift(new AeroModel({ rotorRadiusM: 0.127, clearance: () => ({ below: 3, above: 0.1 }) }), 3)).toBeGreaterThan(0);
  });
});

describe('LiPo pack and PX4 battery gauge', () => {
  it('open-circuit voltage follows Chen & Rincón-Mora', () => {
    expect(cellOcv(1)).toBeCloseTo(4.103, 3);
    expect(cellOcv(0)).toBeCloseTo(2.654, 3);
    expect(cellOcv(0.5)).toBeGreaterThan(cellOcv(0.2));
  });

  it('sags under load and lowers the motor ceiling', () => {
    const i = packCurrent(MISSION_PACK, 0.5, 400);
    expect(i).toBeGreaterThan(400 / (4 * cellOcv(0.5)));
    expect(motorSpeedScale(MISSION_PACK, 0.1, 30)).toBeLessThan(motorSpeedScale(MISSION_PACK, 1, 30));
  });

  it("PX4's voltage estimate: linear between 3.6 and 4.05 V per cell after load compensation", () => {
    expect(px4VoltageSoc(4, 4 * 3.825, 0)).toBeCloseTo(0.5, 6);
    expect(px4VoltageSoc(4, 4 * 3.8, 10)).toBeCloseTo((3.85 - 3.6) / 0.45, 6);
    const est = new Px4BatteryEstimator(4, 5000);
    expect(est.update(4 * 4.0, 8, 0.1)).toBeCloseTo((4 - 3.6 + 0.04) / 0.45, 3);
  });
});

describe('PX4 on the frontier-fly plant', () => {
  it('frame conversion: level and heading north is the identity in NED', () => {
    const p = new FrontierPlant(missionQuadParams());
    const [w, x, y, z] = p.s.q;
    expect(Math.abs(w)).toBeCloseTo(1, 9);
    expect(Math.hypot(x, y, z)).toBeLessThan(1e-9);
    // Heading east (ENU yaw 0) is NED yaw +90°.
    const q = nedQuat([0, 0, 0, 1]);
    expect(2 * Math.atan2(q[3], q[0])).toBeCloseTo(Math.PI / 2, 9);
  });

  it('control allocation signs: what PX4 asks for is what the plant does', () => {
    const params = missionQuadParams();
    const geo = px4Geometry(params);
    const hover = Math.sqrt((params.mass * 9.81) / 4 / params.kf) / 1100;
    const fly = (u: number[]): [number, number, number] => {
      const p = new FrontierPlant(params);
      p.state.pos[2] = 10;
      for (let k = 0; k < 20; k++) p.step(u, 0.004);
      return p.s.omega;
    };
    // More thrust on the left rotors (negative PX4 y) rolls right: positive FRD roll rate.
    const left = geo.map((r) => hover * (r.py < 0 ? 1.05 : 0.95));
    expect(fly(left)[0]).toBeGreaterThan(0.1);
    // More thrust in front pitches nose up: positive FRD pitch rate.
    const front = geo.map((r) => hover * (r.px > 0 ? 1.05 : 0.95));
    expect(fly(front)[1]).toBeGreaterThan(0.1);
    // More on the rotors PX4 believes spin counter-clockwise (KM > 0) yaws right: positive FRD yaw rate.
    const ccw = geo.map((r) => hover * (r.km > 0 ? 1.05 : 0.95));
    expect(fly(ccw)[2]).toBeGreaterThan(0.01);
  });

  it('rests on the ground disarmed and reads gravity on the accelerometer', () => {
    const p = new FrontierPlant(missionQuadParams());
    for (let k = 0; k < 100; k++) p.step([0, 0, 0, 0], 0.004);
    expect(p.s.onGround).toBe(true);
    expect(p.s.pos[2]).toBe(0);
    expect(p.specificForce[2]).toBeCloseTo(-9.81, 6);
  });

  it('the rig plays the gust table from the maneuver start and logs on PX4 time', () => {
    const rig = new PhysicsRig({ wind: { meanMps: 4, dirDeg: 90, gustMps: 1, gustTauSec: 3, seed: 3 } });
    rig.beforeStep(null);
    expect(rig.plant.wind[0]).toBeCloseTo(4, 9);
    rig.beforeStep(2.5);
    const g = new GustTable({ meanMps: 4, dirDeg: 90, gustMps: 1, gustTauSec: 3, seed: 3 }, 10).at(2.5);
    expect(rig.plant.wind[0]).toBeCloseTo(g[1], 9);
    rig.record(1_000_000, [0.5, 0.5, 0.5, 0.5]);
    rig.record(1_010_000, [0.5, 0.5, 0.5, 0.5]);
    rig.record(1_020_000, [0.5, 0.5, 0.5, 0.5]);
    expect(rig.log().rows).toHaveLength(2);
  });
});

describe('validation maneuvers', () => {
  it('PX4 parameters describe the same airframe', () => {
    const p = missionQuadParams();
    const sih = px4Params('sih');
    expect(sih.SIH_MASS).toBe(p.mass);
    expect(sih.SIH_T_MAX).toBeCloseTo(p.kf * 1100 * 1100, 3);
    expect(sihDrag(p)).toBeGreaterThan(p.dragLin[0]);
    const bridge = px4Params('bridge');
    expect(bridge.THR_MDL_FAC).toBe(1);
    expect(bridge.CA_ROTOR0_PX).toBeGreaterThan(0);
  });

  it('every PX4 setpoint pairs its horizontal axes, as PX4 requires', () => {
    for (const m of MANEUVERS)
      for (const s of m.segments) {
        const pos = s.pos ?? [null, null, null];
        const vel = s.vel ?? [null, null, null];
        expect(pos[0] === null).toBe(pos[1] === null);
        expect(vel[0] === null).toBe(vel[1] === null);
        for (let i = 0; i < 3; i++) expect(pos[i] !== null || vel[i] !== null).toBe(true);
      }
  });

  it("frontier-fly's controller flies the forward step and holds height", () => {
    const log = flyNative(MANEUVERS.find((m) => m.id === 'step-forward')!);
    const last = log.rows[log.rows.length - 1]!;
    expect(last[1]).toBeGreaterThan(15);
    expect(Math.abs(last[3]! + 5)).toBeLessThan(0.2);
  });
});

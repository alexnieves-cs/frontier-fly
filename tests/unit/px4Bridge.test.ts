import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { GEO_ORIGINS } from '../../src/mission/export/geo';
import {
  decodeActuatorControls,
  HIL_SENSOR_FIELDS,
  MavlinkParser,
  MavlinkWriter,
  MSG,
  type Frame,
} from '../../src/px4/mavlink';
import { eulerOf, GRAVITY, IRIS, Px4Quad, quatFromYaw, rotate } from '../../src/px4/quad';
import { earthField, isaPressureHpa, Px4SimSession, wmmLookup } from '../../src/px4/session';
import { DECLINATION, INCLINATION, STRENGTH } from '../../src/px4/wmmTables';

interface FixtureFrame {
  kind: string;
  seq: number;
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  values: Record<string, any>;
  hex: string;
}

const fixtures = JSON.parse(readFileSync(join(__dirname, '../fixtures/px4/mavlink-frames.json'), 'utf8')) as {
  frames: FixtureFrame[];
};
const fixture = (kind: string): FixtureFrame => fixtures.frames.find((f) => f.kind === kind)!;
const hex = (bytes: Uint8Array): string => Buffer.from(bytes).toString('hex');
const fromHex = (s: string): Uint8Array => new Uint8Array(Buffer.from(s, 'hex'));

/** A writer whose next sequence number is `seq`. */
function writerAt(seq: number, sysid = 1, compid = 51): MavlinkWriter {
  const w = new MavlinkWriter(sysid, compid);
  for (let i = 0; i < seq; i++) w.frame(MSG.HEARTBEAT, new Uint8Array(9));
  return w;
}

describe('MAVLink codec against pymavlink', () => {
  it('HIL_SENSOR matches pymavlink byte for byte, trailing zeros trimmed', () => {
    const f = fixture('hilSensor');
    const v = f.values;
    const bytes = writerAt(f.seq).hilSensor({
      timeUsec: v.time_usec,
      xacc: v.xacc,
      yacc: v.yacc,
      zacc: v.zacc,
      xgyro: v.xgyro,
      ygyro: v.ygyro,
      zgyro: v.zgyro,
      xmag: v.xmag,
      ymag: v.ymag,
      zmag: v.zmag,
      absPressure: v.abs_pressure,
      diffPressure: v.diff_pressure,
      pressureAlt: v.pressure_alt,
      temperature: v.temperature,
      fieldsUpdated: v.fields_updated,
      id: v.id,
    });
    expect(hex(bytes)).toBe(f.hex);
  });

  it('HIL_GPS matches pymavlink, with speed and course derived from the NED velocity', () => {
    const f = fixture('hilGps');
    const v = f.values;
    const bytes = writerAt(f.seq).hilGps({
      timeUsec: v.time_usec,
      latDeg: v.lat / 1e7,
      lonDeg: v.lon / 1e7,
      altM: v.alt / 1000,
      ephCm: v.eph,
      epvCm: v.epv,
      vn: v.vn / 100,
      ve: v.ve / 100,
      vd: v.vd / 100,
      fixType: v.fix_type,
      satellites: v.satellites_visible,
    });
    expect(hex(bytes)).toBe(f.hex);
  });

  it('HEARTBEAT matches pymavlink', () => {
    const f = fixture('heartbeat');
    const v = f.values;
    const bytes = writerAt(f.seq).heartbeat({
      type: v.type,
      autopilot: v.autopilot,
      baseMode: v.base_mode,
      customMode: v.custom_mode,
      systemStatus: v.system_status,
    });
    expect(hex(bytes)).toBe(f.hex);
  });

  it('decodes PX4 motor outputs, including a trimmed all-zero frame, from any chunking', () => {
    const armed = fixture('actuatorControls');
    const idle = fixture('actuatorControlsDisarmed');
    const stream = new Uint8Array([...fromHex('00ff13'), ...fromHex(armed.hex), ...fromHex(idle.hex)]);
    for (const chunk of [1, 3, 7, stream.length]) {
      const parser = new MavlinkParser();
      const frames: Frame[] = [];
      for (let i = 0; i < stream.length; i += chunk) frames.push(...parser.push(stream.subarray(i, i + chunk)));
      expect(frames.map((f) => f.msgid)).toEqual([MSG.HIL_ACTUATOR_CONTROLS, MSG.HIL_ACTUATOR_CONTROLS]);
      const a = decodeActuatorControls(frames[0]!);
      expect(a.timeUsec).toBe(armed.values.time_usec);
      expect(a.mode).toBe(129);
      expect(a.flags).toBe(1);
      expect(Array.from(a.controls.slice(0, 4), (x) => +x.toFixed(4))).toEqual([0.69, 0.7, 0.71, 0.72]);
      const b = decodeActuatorControls(frames[1]!);
      expect(b.mode).toBe(1);
      expect(Array.from(b.controls).every((x) => x === 0)).toBe(true);
    }
  });

  it('drops a frame with a bad checksum and recovers on the next one', () => {
    const good = fromHex(fixture('actuatorControls').hex);
    const bad = good.slice();
    bad[20] = (bad[20] ?? 0) ^ 0xff;
    const parser = new MavlinkParser();
    const frames = parser.push(new Uint8Array([...bad, ...good]));
    expect(frames).toHaveLength(1);
    expect(parser.badCrc).toBe(1);
  });
});

describe('Earth model', () => {
  it("looks up PX4's WMM tables as PX4 does (values from PX4's own lookup tests)", () => {
    const deg = (rad1e4: number): number => (rad1e4 * 1e-4 * 180) / Math.PI;
    for (const [lat, lon, dec, inc, nT] of [
      [40, 10, 3.4, 55.8, 45954],
      [50, 10, 3.9, 65.9, 49136],
      [-30, 150, 11.0, -60.8, 55113],
    ] as const) {
      expect(deg(wmmLookup(DECLINATION, lat, lon))).toBeCloseTo(dec, 0);
      expect(Math.abs(deg(wmmLookup(INCLINATION, lat, lon)) - inc)).toBeLessThan(1.5);
      expect(Math.abs(wmmLookup(STRENGTH, lat, lon) * 1e-4 * 1e5 - nT)).toBeLessThan(700);
    }
  });

  it('points the field north and down at the PX4 SITL home', () => {
    const h = GEO_ORIGINS['px4-sitl'];
    const [n, e, d] = earthField(h.latDeg, h.lonDeg);
    expect(n).toBeGreaterThan(0.15);
    expect(Math.abs(e)).toBeLessThan(0.05);
    expect(d).toBeGreaterThan(0.35);
  });

  it('gives standard sea-level pressure and the right lapse', () => {
    expect(isaPressureHpa(0)).toBeCloseTo(1013.25, 2);
    // About 0.12 hPa per meter near the ground.
    expect(isaPressureHpa(0) - isaPressureHpa(10)).toBeCloseTo(1.2, 1);
  });
});

describe('motor-level quadrotor', () => {
  const hover = (): number => {
    const w = Math.sqrt((IRIS.massKg * GRAVITY) / 4 / IRIS.kf);
    return (w - IRIS.omegaIdle) / IRIS.omegaScale;
  };

  function airborne(): Px4Quad {
    const q = new Px4Quad();
    q.s.pos = [0, 0, -10];
    q.s.onGround = false;
    const u = hover();
    q.s.rotor = q.s.rotor.map(() => IRIS.omegaIdle + IRIS.omegaScale * u);
    return q;
  }

  it('hovers at the thrust PX4 settles on (about 0.69 for the Iris)', () => {
    expect(hover()).toBeGreaterThan(0.6);
    expect(hover()).toBeLessThan(0.75);
    const q = airborne();
    const u = [hover(), hover(), hover(), hover()];
    for (let i = 0; i < 250; i++) q.step(u, 0.004);
    expect(Math.abs(q.s.pos[2] + 10)).toBeLessThan(0.01);
    expect(Math.hypot(...q.s.omega)).toBeLessThan(1e-9);
    // An accelerometer in hover reads one g up the body z axis.
    expect(q.specificForce[2]).toBeCloseTo(-GRAVITY, 3);
  });

  it('turns the way PX4 mixes: left rotors up rolls right, front rotors up pitches up, counter-clockwise rotors up yaws right', () => {
    const h = hover();
    const cases: [number[], 'roll' | 'pitch' | 'yaw'][] = [
      // Rotors 1 and 2 are on the left (y < 0).
      [[h, h + 0.05, h + 0.05, h], 'roll'],
      // Rotors 0 and 2 are in front.
      [[h + 0.05, h, h + 0.05, h], 'pitch'],
      // Rotors 0 and 1 spin counter-clockwise (positive CA_ROTOR_KM).
      [[h + 0.05, h + 0.05, h, h], 'yaw'],
    ];
    for (const [u, axis] of cases) {
      const q = airborne();
      for (let i = 0; i < 25; i++) q.step(u, 0.004);
      const e = eulerOf(q.s.q);
      expect(e[axis], axis).toBeGreaterThan(0.001);
      // The Iris's rear arms are shorter than its front arms, so raising one diagonal pair also rolls a little.
      if (axis === 'yaw') continue;
      for (const other of ['roll', 'pitch', 'yaw'] as const) {
        if (other !== axis) expect(Math.abs(e[other]), `${axis} -> ${other}`).toBeLessThan(e[axis] * 0.2);
      }
    }
  });

  it('rests on the ground disarmed and reads one g; falls freely with the rotors stopped', () => {
    const ground = new Px4Quad(IRIS, 0.7);
    for (let i = 0; i < 100; i++) ground.step([0, 0, 0, 0], 0.004);
    expect(ground.s.pos[2]).toBe(0);
    expect(ground.s.onGround).toBe(true);
    expect(ground.specificForce[2]).toBeCloseTo(-GRAVITY, 6);
    expect(eulerOf(ground.s.q).yaw).toBeCloseTo(0.7, 6);

    const falling = airborne();
    falling.s.rotor = [0, 0, 0, 0];
    for (let i = 0; i < 50; i++) falling.step([0, 0, 0, 0], 0.004);
    expect(falling.s.vel[2]).toBeGreaterThan(1.5);
    expect(Math.hypot(...falling.specificForce)).toBeLessThan(0.5);
  });

  it('stands on terrain below the pad height', () => {
    const q = new Px4Quad();
    q.groundDown = () => 2;
    q.s.onGround = false;
    for (let i = 0; i < 500; i++) q.step([0, 0, 0, 0], 0.004);
    expect(q.s.pos[2]).toBe(2);
  });

  it('rotates body vectors into NED with yaw measured clockwise from north', () => {
    const v = rotate(quatFromYaw(Math.PI / 2), [1, 0, 0]);
    expect(v[0]).toBeCloseTo(0, 9);
    expect(v[1]).toBeCloseTo(1, 9);
  });
});

describe('lockstep session', () => {
  it('stamps sensors on its own clock, waits for PX4, and sends GNSS at 10 Hz', async () => {
    const sent: Frame[] = [];
    const parser = new MavlinkParser();
    // A stand-in PX4: answers every HIL_SENSOR with hover motor outputs, armed.
    const px4 = writerAt(0, 1, 1);
    let session: Px4SimSession | null = null;
    const controls = new Uint8Array(81);
    const view = new DataView(controls.buffer);
    for (let i = 0; i < 4; i++) view.setFloat32(16 + 4 * i, 0.69, true);
    controls[80] = 129;
    session = new Px4SimSession(
      (bytes) => {
        const frames = parser.push(bytes);
        sent.push(...frames);
        if (frames.some((f) => f.msgid === MSG.HIL_SENSOR)) {
          queueMicrotask(() => session!.receive(px4.frame(MSG.HIL_ACTUATOR_CONTROLS, controls)));
        }
      },
      { home: GEO_ORIGINS['px4-sitl'], lockstepTimeoutMs: 1000 },
    );
    for (let i = 0; i < 50; i++) await session.step();
    const sensors = sent.filter((f) => f.msgid === MSG.HIL_SENSOR);
    const gps = sent.filter((f) => f.msgid === MSG.HIL_GPS);
    expect(sensors).toHaveLength(50);
    expect(gps).toHaveLength(2);
    const times = sensors.map((f) => Number(f.payload.getBigUint64(0, true)));
    expect(times.every((t, i) => i === 0 || t - times[i - 1]! === 4000)).toBe(true);
    const fields = sensors.map((f) => f.payload.getUint32(60, true));
    expect(fields[0]! & HIL_SENSOR_FIELDS.MAG).toBe(HIL_SENSOR_FIELDS.MAG);
    expect(fields[1]! & HIL_SENSOR_FIELDS.MAG).toBe(0);
    expect(session.stats.timeouts).toBe(0);
    expect(session.armed).toBe(true);
    expect(session.motors[0]).toBeCloseTo(0.69, 5);
    // GNSS starts on the pad, where the export puts the plan's home.
    const lat = gps[0]!.payload.getInt32(8, true) / 1e7;
    expect(lat).toBeCloseTo(GEO_ORIGINS['px4-sitl'].latDeg, 6);
  });

  it('steps on without PX4 after the lockstep timeout, and counts it', async () => {
    const session = new Px4SimSession(() => {}, { home: GEO_ORIGINS['px4-sitl'], lockstepTimeoutMs: 1 });
    await session.step();
    expect(session.stats.timeouts).toBe(1);
    expect(session.simTimeSec).toBeCloseTo(0.004, 9);
  });
});

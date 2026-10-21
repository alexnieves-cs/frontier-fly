#!/usr/bin/env python3
"""Regenerate tests/fixtures/px4/mavlink-frames.json with pymavlink's own encoder.

    python3 scripts/px4/mavlink_fixtures.py > tests/fixtures/px4/mavlink-frames.json

The unit tests encode the same values with src/px4/mavlink.ts and compare
bytes, and decode pymavlink's HIL_ACTUATOR_CONTROLS frames.
"""
import json

from pymavlink.dialects.v20 import common as mavlink2


class Sink:
    def write(self, _b):
        pass


def encoder(sysid, compid):
    m = mavlink2.MAVLink(Sink(), srcSystem=sysid, srcComponent=compid)
    m.robust_parsing = True
    return m


out = {"frames": []}
m = encoder(1, 51)
sensor = dict(time_usec=1004000, xacc=0.125, yacc=-0.25, zacc=-9.80665, xgyro=0.001, ygyro=-0.002, zgyro=0.5,
              xmag=0.21, ymag=0.01, zmag=0.43, abs_pressure=955.5, diff_pressure=0.0, pressure_alt=488.25,
              temperature=11.8, fields_updated=0b1101111111111, id=0)
out["frames"].append(dict(kind="hilSensor", seq=0, values=sensor,
                          hex=m.hil_sensor_encode(**sensor).pack(m).hex()))
m.seq = 1
gps = dict(time_usec=1004000, fix_type=3, lat=473977420, lon=85455940, alt=488000, eph=30, epv=40, vel=354,
           vn=250, ve=-250, vd=-10, cog=31500, satellites_visible=12, id=0, yaw=0)
out["frames"].append(dict(kind="hilGps", seq=1, values=gps, hex=m.hil_gps_encode(**gps).pack(m).hex()))
m.seq = 2
hb = dict(type=0, autopilot=8, base_mode=0, custom_mode=0, system_status=4, mavlink_version=3)
out["frames"].append(dict(kind="heartbeat", seq=2, values=hb, hex=m.heartbeat_encode(**hb).pack(m).hex()))

px4 = encoder(1, 1)
controls = [0.69, 0.7, 0.71, 0.72] + [0.0] * 12
act = dict(time_usec=1008000, controls=controls, mode=129, flags=1)
out["frames"].append(dict(kind="actuatorControls", seq=0, values=act, hex=px4.hil_actuator_controls_encode(**act).pack(px4).hex()))
px4.seq = 1
idle = dict(time_usec=1012000, controls=[0.0] * 16, mode=1, flags=1)
out["frames"].append(dict(kind="actuatorControlsDisarmed", seq=1, values=idle, hex=px4.hil_actuator_controls_encode(**idle).pack(px4).hex()))
print(json.dumps(out, indent=1))

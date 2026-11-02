import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from guard import Command, Guard, Limits, Telemetry, arena_to_body, estimate_to_arena  # noqa: E402


def tel(**kw):
    base = dict(t=1.0, x=2.0, y=2.0, z=0.5, yaw_deg=0.0, vbat=3.9)
    base.update(kw)
    return Telemetry(**base)


def g():
    return Guard(4.0, 4.0, Limits())


def test_clamps_speed_and_yaw():
    v = g().check(tel(), Command(2.0, 0.0, 500.0, 0.5), 0.0)
    assert v.action == 'fly'
    assert math.isclose(v.command.vx, 0.5)
    assert v.command.yaw_rate == 90.0


def test_height_setpoint_clamped():
    assert g().check(tel(), Command(0, 0, 0, 3.0), 0.0).command.z == 1.0
    assert g().check(tel(), Command(0, 0, 0, 0.0), 0.0).command.z == 0.25


def test_fence_blocks_outward_velocity_only():
    v = g().check(tel(x=0.1), Command(-0.3, 0.2, 0, 0.5), 0.0)
    assert v.command.vx == 0.0 and math.isclose(v.command.vy, 0.2)
    v = g().check(tel(x=3.9), Command(-0.3, 0.0, 0, 0.5), 0.0)
    assert math.isclose(v.command.vx, -0.3)


def test_fence_breach_lands():
    assert g().check(tel(x=-0.3), Command(), 0.0).action == 'land'


def test_ranger_blocks_velocity_toward_close_obstacle():
    # Nose along +x, wall 10 cm ahead: forward velocity removed, sideways kept.
    v = g().check(tel(front=0.1), Command(0.3, 0.2, 0, 0.5), 0.0)
    assert abs(v.command.vx) < 1e-9 and math.isclose(v.command.vy, 0.2)
    # Rotated 90 deg: the left ranger now looks along -x.
    v = g().check(tel(yaw_deg=90.0, left=0.1), Command(-0.3, 0, 0, 0.5), 0.0)
    assert abs(v.command.vx) < 1e-9


def test_battery_lands_after_hold_time():
    gd = g()
    assert gd.check(tel(t=1.0, vbat=3.3), Command(), 0.0).action == 'fly'
    assert gd.check(tel(t=1.5, vbat=3.3), Command(), 0.0).action == 'fly'
    v = gd.check(tel(t=2.1, vbat=3.3), Command(), 0.0)
    assert v.action == 'land' and 'battery' in v.reason


def test_battery_sag_that_recovers_does_not_land():
    gd = g()
    gd.check(tel(t=1.0, vbat=3.3), Command(), 0.0)
    gd.check(tel(t=1.5, vbat=3.7), Command(), 0.0)
    assert gd.check(tel(t=2.2, vbat=3.3), Command(), 0.0).action == 'fly'


def test_stale_planner_hovers_then_lands():
    assert g().check(tel(), Command(0.3, 0, 0, 0.5), 0.8).action == 'hover'
    assert g().check(tel(), Command(0.3, 0, 0, 0.5), 2.5).action == 'land'
    assert g().check(tel(), None, 0.0).action == 'land'


def test_lost_telemetry_lands():
    assert g().check(tel(), Command(), 0.0, tel_age=0.8).action == 'land'


def test_estimate_jump_lands():
    gd = g()
    gd.check(tel(t=1.0), Command(), 0.0)
    assert gd.check(tel(t=1.05, x=2.7), Command(), 0.0).action == 'land'


def test_tilt_and_flyaway():
    assert g().check(tel(roll_deg=40), Command(), 0.0).action == 'land'
    assert g().check(tel(pitch_deg=80), Command(), 0.0).action == 'kill'
    assert g().check(tel(z=1.5), Command(), 0.0).action == 'land'
    assert g().check(tel(z=2.5), Command(), 0.0).action == 'kill'


def test_flight_time_limit():
    gd = Guard(4, 4, Limits(max_flight_sec=10))
    gd.check(tel(t=0.0), Command(), 0.0)
    assert gd.check(tel(t=11.0), Command(), 0.0).action == 'land'


def test_non_finite_command_hovers():
    assert g().check(tel(), Command(float('nan'), 0, 0, 0.5), 0.0).action == 'hover'


def test_frames():
    # Nose along +y: a world +y velocity is body forward.
    bx, by = arena_to_body(0.0, 0.3, 90.0)
    assert math.isclose(bx, 0.3, abs_tol=1e-9) and math.isclose(by, 0.0, abs_tol=1e-9)
    # World +x with nose along +y is body right (-y).
    bx, by = arena_to_body(0.3, 0.0, 90.0)
    assert math.isclose(by, -0.3, abs_tol=1e-9)
    # Took off at (1, 2) facing +y: estimate +x (forward) is arena +y.
    x, y, yaw = estimate_to_arena(1.0, 0.0, 10.0, {'x': 1.0, 'y': 2.0, 'yaw': math.pi / 2})
    assert math.isclose(x, 1.0, abs_tol=1e-9) and math.isclose(y, 3.0) and math.isclose(yaw, 100.0)

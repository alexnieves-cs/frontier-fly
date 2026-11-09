"""Safety guard between the planner and the Crazyflie.

Every command the planner sends passes through Guard.check before it reaches
the radio. The guard knows only the drone's own telemetry and the arena box;
it never trusts the planner. It is pure Python with no cflib import, so the
cloud tests exercise exactly the code that flies.

Actions, most to least severe:
  kill   cut the motors now (the drone falls; used only for a flip or fly-away)
  land   stop, descend and disarm
  hover  hold position this step
  fly    send the (clamped) command
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class Limits:
    """Every number the guard enforces. docs/CRAZYFLIE.md lists them with reasons."""
    max_speed: float = 0.5           # m/s, horizontal
    max_yaw_rate: float = 90.0       # deg/s
    min_height: float = 0.25         # m, commanded
    max_height: float = 1.0          # m, commanded
    ceiling: float = 1.4             # m, measured: land above this
    flyaway: float = 2.0             # m, measured: kill above this
    fence_margin: float = 0.3        # m inside the arena box: no velocity outward
    fence_breach: float = 0.15       # m outside the arena box: land
    land_voltage: float = 3.4        # V under load, held for low_volt_sec
    low_volt_sec: float = 1.0
    max_flight_sec: float = 240.0
    stale_hover_sec: float = 0.5     # no planner command: hover
    stale_land_sec: float = 2.0      # ... then land
    telemetry_land_sec: float = 0.5  # no telemetry: land (the firmware watchdog cuts motors at 2 s)
    jump_m: float = 0.5              # estimate moved more than this in one step: land
    too_close_m: float = 0.2         # a horizontal ranger closer than this: no velocity toward it
    tilt_land_deg: float = 30.0
    tilt_kill_deg: float = 60.0


@dataclass
class Telemetry:
    """One sample, arena frame. Ranges in meters, None = no return, NaN = invalid."""
    t: float
    x: float
    y: float
    z: float
    yaw_deg: float
    vx: float = 0.0
    vy: float = 0.0
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    vbat: float = 4.0
    front: float | None = None
    back: float | None = None
    left: float | None = None
    right: float | None = None
    up: float | None = None


@dataclass
class Command:
    """World-frame (arena) velocity in m/s, yaw rate in deg/s counter-clockwise, height in m."""
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    z: float = 0.5


@dataclass
class Verdict:
    action: str  # 'fly' | 'hover' | 'land' | 'kill'
    command: Command
    reason: str = ''


@dataclass
class Guard:
    arena_w: float
    arena_d: float
    limits: Limits = field(default_factory=Limits)
    takeoff_t: float | None = None
    _low_volt_since: float | None = None
    _last: Telemetry | None = None
    events: list[str] = field(default_factory=list)

    def _stop(self, action: str, reason: str, z: float) -> Verdict:
        self.events.append(f'{action}: {reason}')
        return Verdict(action, Command(0.0, 0.0, 0.0, z), reason)

    def check(self, tel: Telemetry, cmd: Command | None, cmd_age: float, tel_age: float = 0.0) -> Verdict:
        """Decide what to send this step. cmd_age: seconds since the planner's last command."""
        L = self.limits
        if self.takeoff_t is None:
            self.takeoff_t = tel.t
        # NaN makes every comparison below false, so a non-finite sample would pass the fence,
        # ceiling, fly-away and jump checks. Sanitise the hold height first (max(nan, x) is nan).
        want_z = cmd.z if cmd is not None and isinstance(cmd.z, (int, float)) and math.isfinite(cmd.z) else tel.z
        if not (isinstance(want_z, (int, float)) and math.isfinite(want_z)):
            want_z = L.min_height
        hold_z = min(max(want_z, L.min_height), L.max_height)
        state = (tel.x, tel.y, tel.z, tel.yaw_deg, tel.vx, tel.vy, tel.roll_deg, tel.pitch_deg, tel.vbat)
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in state):
            return self._stop('land', 'non-finite telemetry', hold_z)

        tilt = max(abs(tel.roll_deg), abs(tel.pitch_deg))
        if tilt > L.tilt_kill_deg:
            return self._stop('kill', f'tilt {tilt:.0f} deg', 0.0)
        if tel.z > L.flyaway:
            return self._stop('kill', f'height {tel.z:.2f} m (fly-away)', 0.0)
        if tel_age > L.telemetry_land_sec:
            return self._stop('land', f'no telemetry for {tel_age:.1f} s', hold_z)
        if tilt > L.tilt_land_deg:
            return self._stop('land', f'tilt {tilt:.0f} deg', hold_z)
        if tel.z > L.ceiling:
            return self._stop('land', f'height {tel.z:.2f} m over the {L.ceiling} m ceiling', hold_z)
        out = max(-tel.x, -tel.y, tel.x - self.arena_w, tel.y - self.arena_d)
        if out > L.fence_breach:
            return self._stop('land', f'{out:.2f} m outside the arena', hold_z)
        if self._last is not None:
            jump = math.hypot(tel.x - self._last.x, tel.y - self._last.y)
            if jump > L.jump_m:
                self._last = tel
                return self._stop('land', f'position estimate jumped {jump:.2f} m (flow lost?)', hold_z)
        self._last = tel
        if tel.vbat < L.land_voltage:
            if self._low_volt_since is None:
                self._low_volt_since = tel.t
            if tel.t - self._low_volt_since >= L.low_volt_sec:
                return self._stop('land', f'battery {tel.vbat:.2f} V', hold_z)
        else:
            self._low_volt_since = None
        if tel.t - self.takeoff_t > L.max_flight_sec:
            return self._stop('land', f'flight time over {L.max_flight_sec:.0f} s', hold_z)
        if cmd is None or cmd_age > L.stale_land_sec:
            return self._stop('land', 'planner silent', hold_z)
        if cmd_age > L.stale_hover_sec:
            return Verdict('hover', Command(0.0, 0.0, 0.0, hold_z), 'planner late')

        vx, vy = cmd.vx, cmd.vy
        if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (vx, vy, cmd.yaw_rate, cmd.z)):
            return Verdict('hover', Command(0.0, 0.0, 0.0, hold_z), 'non-finite command')
        sp = math.hypot(vx, vy)
        if sp > L.max_speed:
            vx, vy = vx * L.max_speed / sp, vy * L.max_speed / sp
        # Fence: inside the margin, drop the velocity component that points out of the box.
        if tel.x < L.fence_margin:
            vx = max(vx, 0.0)
        if tel.y < L.fence_margin:
            vy = max(vy, 0.0)
        if tel.x > self.arena_w - L.fence_margin:
            vx = min(vx, 0.0)
        if tel.y > self.arena_d - L.fence_margin:
            vy = min(vy, 0.0)
        # Rangers: no velocity toward anything closer than too_close_m.
        yaw = math.radians(tel.yaw_deg)
        for rng, off in ((tel.front, 0.0), (tel.left, 90.0), (tel.back, 180.0), (tel.right, -90.0)):
            if rng is None or math.isnan(rng) or rng >= L.too_close_m:
                continue
            a = yaw + math.radians(off)
            ux, uy = math.cos(a), math.sin(a)
            toward = vx * ux + vy * uy
            if toward > 0:
                vx -= toward * ux
                vy -= toward * uy
        yr = max(-L.max_yaw_rate, min(L.max_yaw_rate, cmd.yaw_rate))
        return Verdict('fly', Command(vx, vy, yr, hold_z))


def arena_to_body(vx: float, vy: float, yaw_deg: float) -> tuple[float, float]:
    """World (arena) velocity to the body frame send_hover_setpoint expects (x forward, y left)."""
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return c * vx + s * vy, -s * vx + c * vy


def estimate_to_arena(ex: float, ey: float, eyaw_deg: float, start: dict) -> tuple[float, float, float]:
    """The Crazyflie's estimate starts at (0, 0, yaw 0) where it took off; rotate it into the arena."""
    c, s = math.cos(start['yaw']), math.sin(start['yaw'])
    return start['x'] + c * ex - s * ey, start['y'] + s * ex + c * ey, eyaw_deg + math.degrees(start['yaw'])

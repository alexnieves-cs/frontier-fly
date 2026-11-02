"""Two ways to reach a Crazyflie with one interface.

CflibLink talks to the real drone over a Crazyradio with cflib.
SimLink is a small simulated Crazyflie in the arena, used by the cloud tests
and as a rehearsal on the laptop before the props go on. Both report
telemetry in the estimator's own frame (origin where the drone took off,
yaw 0 along its nose then), exactly as the Crazyflie does, so fly.py's frame
handling is exercised by the sim too.
"""
from __future__ import annotations

import math
import random
import threading
import time
from pathlib import Path
from dataclasses import dataclass


@dataclass
class RawTelemetry:
    """Estimator frame. Ranges in mm as the firmware logs them (>= 4000 or 0 means no return)."""
    t: float
    x: float
    y: float
    z: float
    yaw_deg: float
    vx: float
    vy: float
    roll_deg: float
    pitch_deg: float
    vbat: float
    front_mm: int
    back_mm: int
    left_mm: int
    right_mm: int
    up_mm: int
    zrange_mm: int


def mm_to_m(v: int) -> float | None:
    """Multi-ranger reading to meters; None for no return. VL53L1x reports 8000+ or 0 out of range."""
    if v <= 0 or v >= 4000:
        return None
    return v / 1000.0


class Link:
    def now(self) -> float: ...
    def sleep(self, dt: float) -> None: ...
    def read(self) -> tuple[RawTelemetry | None, float]:
        """Latest telemetry and its age in seconds."""
        ...
    def decks(self) -> dict[str, bool]: ...
    def reset_estimator(self) -> None: ...
    def arm(self) -> None: ...
    def send_hover(self, vx_body: float, vy_body: float, yaw_rate_deg: float, z: float) -> None: ...
    def stop_motors(self) -> None: ...
    def kill(self) -> None: ...
    def close(self) -> None: ...


# ---------------------------------------------------------------- simulation

def arena_segments(a: dict) -> list[tuple[float, float, float, float]]:
    w, d = a['w'], a['d']
    s = [(0, 0, w, 0), (w, 0, w, d), (w, d, 0, d), (0, d, 0, 0)]
    for wl in a.get('walls', []):
        s.append((wl['ax'], wl['ay'], wl['bx'], wl['by']))
    for b in a.get('boxes', []):
        x, y, bw, bd = b['x'], b['y'], b['w'], b['d']
        s += [(x, y, x + bw, y), (x + bw, y, x + bw, y + bd), (x + bw, y + bd, x, y + bd), (x, y + bd, x, y)]
    return s


def cast_ray(segs, x: float, y: float, ang: float) -> float:
    dx, dy = math.cos(ang), math.sin(ang)
    best = math.inf
    for ax, ay, bx, by in segs:
        ex, ey = bx - ax, by - ay
        den = dx * ey - dy * ex
        if abs(den) < 1e-12:
            continue
        t = ((ax - x) * ey - (ay - y) * ex) / den
        u = ((ax - x) * dy - (ay - y) * dx) / den
        if t > 0 and 0 <= u <= 1 and t < best:
            best = t
    return best


def clearance(segs, x: float, y: float) -> float:
    best = math.inf
    for ax, ay, bx, by in segs:
        ex, ey = bx - ax, by - ay
        l2 = ex * ex + ey * ey
        t = max(0.0, min(1.0, ((x - ax) * ex + (y - ay) * ey) / l2)) if l2 > 0 else 0.0
        best = min(best, math.hypot(x - ax - t * ex, y - ay - t * ey))
    return best


class SimLink(Link):
    """A Crazyflie flying the stock velocity loop in an arena, on virtual time.

    faults: optional dict with any of
      'vbat_drop_at': s     battery sags to 3.3 V from then on
      'flow_jump_at': s     the position estimate jumps 0.8 m (flow deck lost texture)
      'telemetry_stop_at': s  the radio stops delivering telemetry
      'decks_missing': True  the Multi-ranger deck is not detected
    """

    DT = 0.05
    PROP_RADIUS = 0.07

    def __init__(self, arena: dict, seed: int = 1, faults: dict | None = None):
        self.arena = arena
        self.segs = arena_segments(arena)
        self.rng = random.Random(seed)
        self.faults = faults or {}
        self.t = 0.0
        s = arena['start']
        self.x, self.y, self.yaw = s['x'], s['y'], s['yaw']
        self.z = 0.02
        self.vx = self.vy = self.wz = 0.0
        self.cmd = (0.0, 0.0, 0.0, 0.0)
        self.motors = False
        self.armed = False
        self.min_clearance = math.inf
        self.crashed = False
        self.jump = (0.0, 0.0)
        self.track: list[tuple[float, float]] = []

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        steps = max(1, round(dt / self.DT))
        for _ in range(steps):
            self._step()

    def _step(self) -> None:
        dt = self.DT
        self.t += dt
        if not self.motors or self.crashed:
            self.vx = self.vy = self.wz = 0.0
            self.z = max(0.0, self.z - 1.0 * dt)
            return
        bvx, bvy, wdeg, zt = self.cmd
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        wvx, wvy = c * bvx - s * bvy, s * bvx + c * bvy
        tau = 0.35
        self.vx += max(-2.0, min(2.0, (wvx - self.vx) / tau)) * dt
        self.vy += max(-2.0, min(2.0, (wvy - self.vy) / tau)) * dt
        self.wz += (math.radians(wdeg) - self.wz) / 0.15 * dt
        self.z += max(-0.5, min(0.5, (zt - self.z) * 2.0)) * dt
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.yaw += self.wz * dt
        if self.z > 0.1:
            cl = clearance(self.segs, self.x, self.y) - self.PROP_RADIUS
            self.min_clearance = min(self.min_clearance, cl)
            if cl < 0:
                self.crashed = True
        if abs(self.t * 2 - round(self.t * 2)) < 1e-6:
            self.track.append((round(self.x, 2), round(self.y, 2)))

    def _range_mm(self, ang: float) -> int:
        r = cast_ray(self.segs, self.x, self.y, ang) + self.rng.gauss(0, 0.02)
        return 8190 if r >= 4.0 else max(30, int(r * 1000))

    def read(self):
        stop = self.faults.get('telemetry_stop_at')
        if stop is not None and self.t >= stop:
            return None, self.t - stop
        s = self.arena['start']
        dx, dy = self.x - s['x'], self.y - s['y']
        c, sn = math.cos(-s['yaw']), math.sin(-s['yaw'])
        ex, ey = c * dx - sn * dy, sn * dx + c * dy
        evx, evy = c * self.vx - sn * self.vy, sn * self.vx + c * self.vy
        fj = self.faults.get('flow_jump_at')
        if fj is not None and self.t >= fj:
            ex += 0.8
        vbat = 3.3 if self.faults.get('vbat_drop_at') is not None and self.t >= self.faults['vbat_drop_at'] else (4.1 if not self.motors else 3.8)
        tel = RawTelemetry(
            t=self.t, x=ex, y=ey, z=self.z, yaw_deg=math.degrees(self.yaw - s['yaw']), vx=evx, vy=evy,
            roll_deg=0.0, pitch_deg=0.0, vbat=vbat,
            front_mm=self._range_mm(self.yaw), back_mm=self._range_mm(self.yaw + math.pi),
            left_mm=self._range_mm(self.yaw + math.pi / 2), right_mm=self._range_mm(self.yaw - math.pi / 2),
            up_mm=8190, zrange_mm=int(self.z * 1000),
        )
        return tel, 0.0

    def decks(self):
        return {'flow2': True, 'multiranger': not self.faults.get('decks_missing', False)}

    def reset_estimator(self):
        pass

    def arm(self):
        self.armed = True

    def send_hover(self, vx_body, vy_body, yaw_rate_deg, z):
        self.motors = True
        self.cmd = (vx_body, vy_body, yaw_rate_deg, z)

    def stop_motors(self):
        self.motors = False

    def kill(self):
        self.motors = False

    def close(self):
        pass


# ---------------------------------------------------------------- real drone

class CflibLink(Link):
    """The real Crazyflie through cflib (pip install cflib==0.1.34) and a Crazyradio."""

    def __init__(self, uri: str):
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.log import LogConfig
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

        cflib.crtp.init_drivers()
        self._lock = threading.Lock()
        self._a: dict = {}
        self._b: dict = {}
        self._ta = self._tb = 0.0
        self.scf = SyncCrazyflie(uri, cf=Crazyflie(rw_cache=str(Path.home() / '.cache' / 'frontier-fly-cflib')))
        self.scf.open_link()
        self.cf = self.scf.cf
        # A log packet holds 26 bytes, so two blocks: pose (6 floats), then rangers + battery + attitude.
        a = LogConfig(name='pose', period_in_ms=50)
        for v in ('stateEstimate.x', 'stateEstimate.y', 'stateEstimate.z', 'stateEstimate.yaw', 'stateEstimate.vx', 'stateEstimate.vy'):
            a.add_variable(v, 'float')
        b = LogConfig(name='rangers', period_in_ms=50)
        for v in ('range.front', 'range.back', 'range.left', 'range.right', 'range.up', 'range.zrange'):
            b.add_variable(v, 'uint16_t')
        b.add_variable('pm.vbat', 'float')
        b.add_variable('stabilizer.roll', 'float')
        b.add_variable('stabilizer.pitch', 'float')
        self.cf.log.add_config(a)
        self.cf.log.add_config(b)
        a.data_received_cb.add_callback(self._on_a)
        b.data_received_cb.add_callback(self._on_b)
        a.start()
        b.start()
        self._logs = (a, b)

    def _on_a(self, _ts, data, _conf):
        with self._lock:
            self._a, self._ta = data, time.monotonic()

    def _on_b(self, _ts, data, _conf):
        with self._lock:
            self._b, self._tb = data, time.monotonic()

    def now(self):
        return time.monotonic()

    def sleep(self, dt):
        time.sleep(dt)

    def read(self):
        with self._lock:
            a, b, ta, tb = self._a, self._b, self._ta, self._tb
        if not a or not b:
            return None, math.inf
        now = time.monotonic()
        tel = RawTelemetry(
            t=now, x=a['stateEstimate.x'], y=a['stateEstimate.y'], z=a['stateEstimate.z'],
            yaw_deg=a['stateEstimate.yaw'], vx=a['stateEstimate.vx'], vy=a['stateEstimate.vy'],
            roll_deg=b['stabilizer.roll'], pitch_deg=b['stabilizer.pitch'], vbat=b['pm.vbat'],
            front_mm=b['range.front'], back_mm=b['range.back'], left_mm=b['range.left'],
            right_mm=b['range.right'], up_mm=b['range.up'], zrange_mm=b['range.zrange'],
        )
        return tel, now - min(ta, tb)

    def decks(self):
        def present(name):
            try:
                return int(self.cf.param.get_value(name)) == 1
            except Exception:
                return False
        return {'flow2': present('deck.bcFlow2'), 'multiranger': present('deck.bcMultiranger')}

    def reset_estimator(self):
        from cflib.utils.reset_estimator import reset_estimator
        reset_estimator(self.scf)

    def arm(self):
        # Newer firmware needs an arming request before the motors spin; older firmware ignores it.
        try:
            self.cf.platform.send_arming_request(True)
            time.sleep(1.0)
        except Exception:
            pass

    def send_hover(self, vx_body, vy_body, yaw_rate_deg, z):
        # cflib >= 0.1.2x with current firmware: yaw rate in deg/s, counter-clockwise positive (TYPE_HOVER).
        self.cf.commander.send_hover_setpoint(vx_body, vy_body, yaw_rate_deg, z)

    def stop_motors(self):
        self.cf.commander.send_stop_setpoint()
        self.cf.commander.send_notify_setpoint_stop()

    def kill(self):
        try:
            self.cf.commander.send_stop_setpoint()
        finally:
            self.cf.loc.send_emergency_stop()

    def close(self):
        for log in self._logs:
            try:
                log.stop()
            except Exception:
                pass
        self.scf.close_link()

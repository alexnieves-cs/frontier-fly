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


def _dist_version(name: str) -> str | None:
    try:
        from importlib.metadata import version
        return version(name)
    except Exception:
        return None


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


# How the simulated Crazyflie behaves. 'ideal' is the original rehearsal sim: commands act on the next
# step and the position estimate is perfect. 'bench' matches src/crazyflie/sim.ts, the model behind
# npm run crazyflie:bench: 0.1 s of command latency (log + radio + planner) and a flow-deck estimate
# with a fixed ~4% scale error per axis and a heading misalignment. compare.py replays real flights
# with 'bench' so the two simulators and the real drone can sit in one table.
SIM_PROFILES: dict[str, dict[str, float]] = {
    'ideal': {'latency': 0.0, 'drift': 0.0, 'tau': 0.35, 'sigma': 0.02},
    'bench': {'latency': 0.1, 'drift': 0.04, 'tau': 0.35, 'sigma': 0.02},
}


class SimLink(Link):
    """A Crazyflie flying the stock velocity loop in an arena, on virtual time.

    faults: optional dict with any of
      'vbat_drop_at': s     battery sags to 3.3 V from then on
      'flow_jump_at': s     the position estimate jumps 0.8 m (flow deck lost texture)
      'telemetry_stop_at': s  the radio stops delivering telemetry
      'decks_missing': True  the Multi-ranger deck is not detected

    profile: a SIM_PROFILES key; latency, drift, tau and sigma override single values.
    """

    DT = 0.05
    PROP_RADIUS = 0.07

    def __init__(self, arena: dict, seed: int = 1, faults: dict | None = None, profile: str = 'ideal', **overrides: float):
        if profile not in SIM_PROFILES:
            raise ValueError(f'unknown sim profile {profile!r}: use one of {", ".join(SIM_PROFILES)}')
        unknown = set(overrides) - set(SIM_PROFILES[profile])
        if unknown:
            raise ValueError(f'unknown sim setting {", ".join(sorted(unknown))}')
        self.settings = {**SIM_PROFILES[profile], **{k: float(v) for k, v in overrides.items()}}
        self.profile = profile
        self.arena = arena
        self.segs = arena_segments(arena)
        self.rng = random.Random(seed)
        self.faults = faults or {}
        self.t = 0.0
        self.k = 0
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
        # Commands reach the motors delay steps after the step they were sent in (0 = the next step).
        self.delay = max(0, round(self.settings['latency'] / self.DT))
        self.pending: list[tuple[int, tuple | None]] = []
        # Flow-deck error, fixed for the flight, drawn from its own stream so the beam noise is unchanged.
        d = self.settings['drift']
        drng = random.Random(seed * 7919 + 17)
        self.sx, self.sy, self.th = drng.gauss(0, 1) * d, drng.gauss(0, 1) * d, drng.gauss(0, 1) * d * 0.5

    def now(self) -> float:
        return self.t

    def sleep(self, dt: float) -> None:
        steps = max(1, round(dt / self.DT))
        for _ in range(steps):
            self._step()

    def _step(self) -> None:
        dt = self.DT
        self.t += dt
        self.k += 1
        while self.pending and self.pending[0][0] <= self.k:
            _, c = self.pending.pop(0)
            if c is None:
                self.motors = False
            else:
                self.motors = True
                self.cmd = c
        if not self.motors or self.crashed:
            self.vx = self.vy = self.wz = 0.0
            self.z = max(0.0, self.z - 1.0 * dt)
            return
        bvx, bvy, wdeg, zt = self.cmd
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        wvx, wvy = c * bvx - s * bvy, s * bvx + c * bvy
        tau = self.settings['tau']
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
        r = cast_ray(self.segs, self.x, self.y, ang) + self.rng.gauss(0, 0.02) * (self.settings['sigma'] / 0.02)
        return 8190 if r >= 4.0 else max(30, int(r * 1000))

    def read(self):
        stop = self.faults.get('telemetry_stop_at')
        if stop is not None and self.t >= stop:
            return None, self.t - stop
        s = self.arena['start']
        # The flow estimate, in the arena frame: scaled per axis, then turned by the misalignment.
        ax, ay = (1 + self.sx) * (self.x - s['x']), (1 + self.sy) * (self.y - s['y'])
        ct, st = math.cos(self.th), math.sin(self.th)
        dx, dy = ct * ax - st * ay, st * ax + ct * ay
        wvx, wvy = ct * self.vx - st * self.vy, st * self.vx + ct * self.vy
        # ... then into the estimator's own frame (origin and heading where it took off).
        c, sn = math.cos(-s['yaw']), math.sin(-s['yaw'])
        ex, ey = c * dx - sn * dy, sn * dx + c * dy
        evx, evy = c * wvx - sn * wvy, sn * wvx + c * wvy
        fj = self.faults.get('flow_jump_at')
        if fj is not None and self.t >= fj:
            ex += 0.8
        vbat = 3.3 if self.faults.get('vbat_drop_at') is not None and self.t >= self.faults['vbat_drop_at'] else (4.1 if not self.motors else 3.8)
        tel = RawTelemetry(
            t=self.t, x=ex, y=ey, z=self.z, yaw_deg=math.degrees(self.yaw + self.th - s['yaw']), vx=evx, vy=evy,
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
        self._send((vx_body, vy_body, yaw_rate_deg, z))

    def stop_motors(self):
        self._send(None)

    def kill(self):
        self._send(None)

    def _send(self, c: tuple | None) -> None:
        if self.delay == 0:
            # The original behaviour: the setpoint holds from the next step on.
            if c is None:
                self.motors = False
            else:
                self.motors = True
                self.cmd = c
            return
        self.pending.append((self.k + 1 + self.delay, c))

    def truth(self) -> dict:
        """The simulator's real pose in the arena frame, for the log. A real flight has no such thing."""
        return {'x': self.x, 'y': self.y, 'z': self.z, 'yaw_deg': math.degrees(self.yaw)}

    def describe(self) -> dict:
        return {'profile': self.profile, **self.settings, 'flow_error': {'scale_x': self.sx, 'scale_y': self.sy, 'heading_rad': self.th}}

    def close(self):
        pass


# ---------------------------------------------------------------- mock radio

class MockRadioLink(Link):
    """A Crazyradio between fly.py and a simulated Crazyflie, so link trouble can be rehearsed in the cloud.

    The inner link (a SimLink) is the drone. This class is the radio: telemetry packets come down at
    20 Hz and commands go up, and either can be lost, delayed or blacked out. It keeps the firmware's
    commander watchdog too: with no setpoint for 0.5 s the drone stops flying its last velocity and
    holds still (the real firmware levels out, which is close enough at these speeds), and after 2 s
    it cuts the motors. read() reports the age of the newest packet that arrived, like CflibLink.

      drop        share of telemetry packets lost (0 to 1)
      cmd_drop    share of command packets lost
      latency     one-way delay, s, both directions
      jitter      extra random delay, s, uniform 0 to jitter
      outage_at   start of a total blackout, s (optional)
      outage_sec  how long it lasts, s
    """

    PERIOD = 0.05
    WDT_LEVEL = 0.5
    WDT_STOP = 2.0
    KEYS = ('drop', 'cmd_drop', 'latency', 'jitter', 'outage_at', 'outage_sec')

    def __init__(self, inner: Link, seed: int = 1, **radio: float):
        unknown = set(radio) - set(self.KEYS)
        if unknown:
            raise ValueError(f'unknown mock radio setting {", ".join(sorted(unknown))}')
        self.inner = inner
        self.radio = {'drop': 0.0, 'cmd_drop': 0.0, 'latency': 0.0, 'jitter': 0.0, 'outage_at': math.inf, 'outage_sec': 0.0}
        self.radio.update({k: float(v) for k, v in radio.items()})
        self.rng = random.Random(seed * 104729 + 3)
        self.down: list[tuple[float, RawTelemetry]] = []   # (arrival time, packet), in flight
        self.up: list[tuple[float, str, tuple]] = []        # (arrival time, kind, args)
        self.latest: RawTelemetry | None = None
        self.latest_at = -math.inf
        self.last_cmd_at: float | None = None
        self.last_z = 0.0
        self.watchdog = ''
        self.counts = {'tel_sent': 0, 'tel_received': 0, 'cmd_sent': 0, 'cmd_delivered': 0}
        self.max_gap = 0.0
        self._next_tel = 0.0

    def _blackout(self, t: float) -> bool:
        return self.radio['outage_at'] <= t < self.radio['outage_at'] + self.radio['outage_sec']

    def _delay(self) -> float:
        return self.radio['latency'] + self.rng.random() * self.radio['jitter']

    def now(self) -> float:
        return self.inner.now()

    def _tick(self) -> None:
        """Advance the radio to the inner link's clock: deliver what has arrived, sample telemetry."""
        t = self.inner.now()
        while t + 1e-9 >= self._next_tel:
            self._next_tel += self.PERIOD
            pkt, _ = self.inner.read()
            if pkt is None:
                continue
            self.counts['tel_sent'] += 1
            if self._blackout(t) or self.rng.random() < self.radio['drop']:
                continue
            self.down.append((t + self._delay(), pkt))
        arrived = [p for p in self.down if p[0] <= t + 1e-9]
        if arrived:
            self.down = [p for p in self.down if p[0] > t + 1e-9]
            at, pkt = max(arrived, key=lambda p: p[1].t)
            if self.latest is None or pkt.t >= self.latest.t:
                if self.latest is not None:
                    self.max_gap = max(self.max_gap, at - self.latest_at)
                self.latest, self.latest_at = pkt, at
            self.counts['tel_received'] += len(arrived)
        ready = [c for c in self.up if c[0] <= t + 1e-9]
        if ready:
            self.up = [c for c in self.up if c[0] > t + 1e-9]
            for _, kind, args in ready:
                self.counts['cmd_delivered'] += 1
                # A stop disarms the watchdog: there is nothing left for it to stop.
                self.last_cmd_at = t if kind == 'send_hover' else None
                self.watchdog = ''
                getattr(self.inner, kind)(*args)
        if self.last_cmd_at is not None:
            quiet = t - self.last_cmd_at
            if quiet > self.WDT_STOP and self.watchdog != 'stop':
                self.watchdog = 'stop'
                self.inner.stop_motors()
            elif quiet > self.WDT_LEVEL and self.watchdog == '':
                self.watchdog = 'level'
                self.inner.send_hover(0.0, 0.0, 0.0, self.last_z)

    def sleep(self, dt: float) -> None:
        steps = max(1, round(dt / SimLink.DT))
        for _ in range(steps):
            self.inner.sleep(SimLink.DT)
            self._tick()

    def read(self):
        self._tick()
        if self.latest is None:
            return None, math.inf
        now = self.inner.now()
        pkt = self.latest
        # Like CflibLink: the sample is stamped when it is read, and age is the time since it arrived.
        tel = RawTelemetry(**{**pkt.__dict__, 't': now})
        return tel, now - self.latest_at

    def _uplink(self, kind: str, *args) -> None:
        self.counts['cmd_sent'] += 1
        t = self.inner.now()
        if self._blackout(t) or self.rng.random() < self.radio['cmd_drop']:
            return
        self.up.append((t + self._delay(), kind, args))
        if self.radio['latency'] == 0 and self.radio['jitter'] == 0:
            self._tick()

    def decks(self):
        return self.inner.decks()

    def reset_estimator(self):
        self.inner.reset_estimator()

    def arm(self):
        self.inner.arm()

    def send_hover(self, vx_body, vy_body, yaw_rate_deg, z):
        self.last_z = z
        self._uplink('send_hover', vx_body, vy_body, yaw_rate_deg, z)

    def stop_motors(self):
        self._uplink('stop_motors')

    def kill(self):
        self._uplink('kill')

    def stats(self) -> dict:
        sent = max(1, self.counts['tel_sent'])
        return {
            **self.counts,
            'tel_loss': round(1 - self.counts['tel_received'] / sent, 4),
            'max_gap_s': round(self.max_gap, 3),
            'latency_ms': round(2000 * self.radio['latency'] + 1000 * self.radio['jitter'], 1),
            'link_quality': round(100 * (1 - self.radio['drop']), 1),
        }

    def truth(self) -> dict:
        return self.inner.truth()

    def describe(self) -> dict:
        inner = self.inner.describe() if hasattr(self.inner, 'describe') else {}
        return {**inner, 'radio': {k: v for k, v in self.radio.items() if math.isfinite(v)}}

    def close(self):
        self.inner.close()

    def __getattr__(self, name):
        # crashed, min_clearance, track, t: the drone's, not the radio's.
        return getattr(self.inner, name)


# ---------------------------------------------------------------- real drone

class CflibLink(Link):
    """The real Crazyflie through cflib (pip install cflib==0.1.34) and a Crazyradio."""

    def __init__(self, uri: str):
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.log import LogConfig
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

        cflib.crtp.init_drivers()
        self.uri = uri
        self._lock = threading.Lock()
        self._a: dict = {}
        self._b: dict = {}
        self._ta = self._tb = 0.0
        # Link health for bringup.py and the flight log: packets per log block, the longest gap
        # between two packets of a block, and what cflib's link statistics report.
        self._count = {'pose': 0, 'rangers': 0}
        self._gap = {'pose': 0.0, 'rangers': 0.0}
        self._opened = time.monotonic()
        self._radio: dict[str, float] = {}
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
        self._opened = time.monotonic()
        # cflib 0.1.34 pings the Crazyflie every 0.1 s and reports a rolling p95 round trip in ms, plus
        # the radio's link quality (0-100) and uplink RSSI. Older cflib or a stand-in may not have them.
        stats = getattr(self.cf, 'link_statistics', None)
        for name, key in (('latency_updated', 'latency_ms'), ('link_quality_updated', 'link_quality'), ('uplink_rssi_updated', 'rssi')):
            caller = getattr(stats, name, None)
            if caller is not None:
                try:
                    caller.add_callback(lambda v, key=key: self._on_radio(key, v))
                except Exception:
                    pass

    def _on_radio(self, key: str, value) -> None:
        with self._lock:
            self._radio[key] = float(value)

    def _on_a(self, _ts, data, _conf):
        with self._lock:
            now = time.monotonic()
            if self._count['pose']:
                self._gap['pose'] = max(self._gap['pose'], now - self._ta)
            self._count['pose'] += 1
            self._a, self._ta = data, now

    def _on_b(self, _ts, data, _conf):
        with self._lock:
            now = time.monotonic()
            if self._count['rangers']:
                self._gap['rangers'] = max(self._gap['rangers'], now - self._tb)
            self._count['rangers'] += 1
            self._b, self._tb = data, now

    def stats(self) -> dict:
        """Packets per log block since the link opened, the longest gap, and cflib's radio numbers."""
        with self._lock:
            up = max(1e-9, time.monotonic() - self._opened)
            expected = up / 0.05
            out = {
                'pose_packets': self._count['pose'], 'rangers_packets': self._count['rangers'],
                'tel_loss': round(max(0.0, 1 - min(self._count.values()) / expected), 4) if expected >= 1 else 0.0,
                'max_gap_s': round(max(self._gap.values()), 3),
            }
            out.update({k: round(v, 1) for k, v in self._radio.items()})
        return out

    def describe(self) -> dict:
        """What the flight log records about the link and the drone. Best effort: missing values are left out."""
        info: dict = {'uri': self.uri}
        try:
            import cflib
            info['cflib'] = getattr(cflib, '__version__', None) or _dist_version('cflib')
        except Exception:
            pass
        try:
            info['protocol_version'] = self.cf.platform.get_protocol_version()
        except Exception:
            pass
        for name in ('firmware.revision0', 'firmware.revision1', 'firmware.modified'):
            try:
                info[name] = self.cf.param.get_value(name)
            except Exception:
                pass
        info['decks'] = self.decks()
        return info

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

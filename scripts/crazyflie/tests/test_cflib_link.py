"""CflibLink against a stand-in Crazyflie: the real cflib package, no radio."""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
cflib = pytest.importorskip('cflib')
import links  # noqa: E402
from cflib.crazyflie.log import LogTocElement  # noqa: E402

SIZES = {'float': 4, 'uint16_t': 2}


class FakeCommander:
    def __init__(self):
        self.sent = []

    def send_hover_setpoint(self, *a):
        self.sent.append(('hover', a))

    def send_stop_setpoint(self):
        self.sent.append(('stop',))

    def send_notify_setpoint_stop(self, *a):
        self.sent.append(('notify',))


class FakeLog:
    def __init__(self):
        self.configs = []

    def add_config(self, c):
        self.configs.append(c)


class FakeCf:
    def __init__(self, *a, **k):
        self.commander = FakeCommander()
        self.log = FakeLog()
        self.param = SimpleNamespace(get_value=lambda name: '1')
        self.platform = SimpleNamespace(send_arming_request=lambda on: None)
        self.loc = SimpleNamespace(send_emergency_stop=lambda: self.commander.sent.append(('estop',)))


class FakeScf:
    def __init__(self, uri, cf):
        self.cf = cf

    def open_link(self):
        pass

    def close_link(self):
        pass


@pytest.fixture
def link(monkeypatch):
    import cflib.crazyflie
    import cflib.crazyflie.log as log
    import cflib.crazyflie.syncCrazyflie as sync
    import cflib.crtp
    monkeypatch.setattr(cflib.crtp, 'init_drivers', lambda: None)
    monkeypatch.setattr(cflib.crazyflie, 'Crazyflie', FakeCf)
    monkeypatch.setattr(sync, 'SyncCrazyflie', FakeScf)
    monkeypatch.setattr(log.LogConfig, 'start', lambda self: None)
    monkeypatch.setattr(log.LogConfig, 'stop', lambda self: None)
    return links.CflibLink('radio://0/80/2M/E7E7E7E7E7')


def test_log_blocks_fit_one_packet(link):
    # A Crazyflie log packet carries at most 26 bytes of variables.
    for c in link.cf.log.configs:
        size = sum(SIZES[v.fetch_as] if isinstance(v.fetch_as, str) else LogTocElement.get_size_from_id(v.fetch_as) for v in c.variables)
        assert size <= 26, (c.name, size)
        assert c.period_in_ms == 50


def test_telemetry_and_units(link):
    pose = {'stateEstimate.x': 0.5, 'stateEstimate.y': -0.2, 'stateEstimate.z': 0.4, 'stateEstimate.yaw': 30.0,
            'stateEstimate.vx': 0.1, 'stateEstimate.vy': 0.0}
    rng = {'range.front': 1234, 'range.back': 8190, 'range.left': 0, 'range.right': 350, 'range.up': 8190,
           'range.zrange': 400, 'pm.vbat': 3.95, 'stabilizer.roll': 1.0, 'stabilizer.pitch': -2.0}
    assert link.read()[0] is None
    link._on_a(0, pose, None)
    link._on_b(0, rng, None)
    tel, age = link.read()
    assert tel.x == 0.5 and tel.yaw_deg == 30.0 and age < 0.5
    assert links.mm_to_m(tel.front_mm) == 1.234
    assert links.mm_to_m(tel.back_mm) is None and links.mm_to_m(tel.left_mm) is None
    assert links.mm_to_m(tel.right_mm) == 0.35


def test_commands_reach_cflib(link):
    link.send_hover(0.2, -0.1, 30.0, 0.5)
    link.stop_motors()
    link.kill()
    sent = link.cf.commander.sent
    assert sent[0] == ('hover', (0.2, -0.1, 30.0, 0.5))
    assert ('stop',) in sent and ('notify',) in sent and sent[-1] == ('estop',)
    assert link.decks() == {'flow2': True, 'multiranger': True}

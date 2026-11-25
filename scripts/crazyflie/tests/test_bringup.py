"""bringup.py: the radio scan and the props-off link check."""
import json
import sys
from pathlib import Path
from types import ModuleType

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import bringup  # noqa: E402
from flightlog import read_log, validate  # noqa: E402
from links import MockRadioLink, SimLink  # noqa: E402

ARENA = json.loads(bringup.REHEARSAL_ARENA.read_text())


class NoCommands:
    """Wraps a link and fails the test if anything could start the motors."""

    def __init__(self, inner):
        self.inner = inner

    def __getattr__(self, name):
        if name in ('arm', 'send_hover', 'stop_motors', 'kill'):
            raise AssertionError(f'link check called {name}()')
        return getattr(self.inner, name)


def test_clean_link_passes_and_never_commands():
    r = bringup.link_check(NoCommands(MockRadioLink(SimLink(ARENA), drop=0.01, latency=0.005)), seconds=5)
    assert r.ok, r.problems
    assert r.fresh_share > 0.98 and r.vbat == 4.1
    assert r.decks == {'flow2': True, 'multiranger': True}


def test_lossy_link_fails():
    r = bringup.link_check(NoCommands(MockRadioLink(SimLink(ARENA), drop=0.3)), seconds=20)
    assert not r.ok
    assert any('fresh' in p for p in r.problems) and any('link quality' in p for p in r.problems)


def test_missing_deck_and_no_telemetry():
    r = bringup.link_check(NoCommands(SimLink(ARENA, faults={'decks_missing': True, 'telemetry_stop_at': 0})), seconds=1)
    assert not r.ok
    assert 'Multi-ranger deck not detected' in r.problems
    assert any('no telemetry' in p for p in r.problems)


def test_cli_rehearsal_writes_a_valid_log(tmp_path):
    log, report = tmp_path / 'link.jsonl', tmp_path / 'link.json'
    code = bringup.main(['link', '--sim', '--seconds', '3', '--log', str(log), '--report', str(report)])
    assert code == 0
    assert validate(log) == []
    assert read_log(log).mode == 'link'
    assert json.loads(report.read_text())['outcome'] == 'link-ok'
    assert bringup.main(['link', '--mock-radio', 'drop=0.1', '--seconds', '1', '--log', str(tmp_path / 'x.jsonl')]) == 2


def fake_cflib(monkeypatch, found=None, error=None):
    crtp = ModuleType('cflib.crtp')
    crtp.init_drivers = lambda: None
    crtp.get_interfaces_status = lambda: {'radio': 'Crazyradio 2.0 version 5.0'}

    def scan_interfaces(addr=None):
        if error:
            raise error
        return found or []
    crtp.scan_interfaces = scan_interfaces
    pkg = ModuleType('cflib')
    pkg.crtp = crtp
    monkeypatch.setitem(sys.modules, 'cflib', pkg)
    monkeypatch.setitem(sys.modules, 'cflib.crtp', crtp)


def test_scan_lists_uris(monkeypatch, capsys):
    fake_cflib(monkeypatch, found=[['radio://0/80/2M/E7E7E7E7E7', '']])
    assert bringup.scan() == 0
    assert 'found radio://0/80/2M/E7E7E7E7E7' in capsys.readouterr().out


def test_scan_explains_nothing_found_and_errors(monkeypatch, capsys):
    fake_cflib(monkeypatch)
    assert bringup.scan() == 1
    assert 'udev' in capsys.readouterr().err
    fake_cflib(monkeypatch, error=OSError('USB permission denied'))
    assert bringup.scan() == 1
    assert 'USB permission denied' in capsys.readouterr().err


def test_scan_without_cflib(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, 'cflib', None)
    monkeypatch.setitem(sys.modules, 'cflib.crtp', None)
    assert bringup.scan() == 2
    assert 'pip install' in capsys.readouterr().err


def test_link_without_a_radio_explains(monkeypatch, capsys, tmp_path):
    def no_radio(uri):
        raise Exception('Cannot find a Crazyradio Dongle')
    monkeypatch.setattr(bringup, 'CflibLink', no_radio)
    assert bringup.main(['link', '--uri', 'radio://0/80/2M/E7E7E7E7E7', '--log', str(tmp_path / 'x.jsonl')]) == 1
    assert 'bringup.py scan' in capsys.readouterr().err

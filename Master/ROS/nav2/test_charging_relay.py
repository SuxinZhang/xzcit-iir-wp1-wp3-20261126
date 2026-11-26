"""Verify charging ownership and stale-command stops without vehicle hardware.
在没有车辆硬件时验证回充速度所有权与过期命令停车。
"""
import importlib.util
from pathlib import Path
from types import SimpleNamespace


spec = importlib.util.spec_from_file_location('charging_relay', Path(__file__).with_name('nav_relay.py'))
relay_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay_module)


def test_charging_timeout_never_restores_navigation(monkeypatch):
    now = [1.0]
    monkeypatch.setattr(relay_module.time, 'monotonic', lambda: now[0])
    relay = relay_module.NavRelay.__new__(relay_module.NavRelay)
    relay.args = SimpleNamespace(max_out=0.15)
    relay.charging_hold = False
    relay.charging_hold_rx = relay.charging_cmd_rx = None
    relay.last_rx = 0.0
    outputs = []
    relay._publish = lambda vx, wz: outputs.append((vx, wz))
    relay.publish_zero = lambda why: outputs.append((0.0, 0.0))
    command = SimpleNamespace(linear=SimpleNamespace(x=0.05), angular=SimpleNamespace(z=0.0))
    relay.on_charging_hold(SimpleNamespace(data=True))
    relay.on_cmd(command)
    assert outputs == [(0.0, 0.0)]
    relay.on_charging_cmd(command)
    assert outputs[-1] == (0.05, 0.0)
    now[0] = 1.31
    relay.watchdog()
    assert outputs[-1] == (0.0, 0.0)
    relay.on_cmd(command)
    relay.on_charging_cmd(command)
    assert outputs[-1] == (0.0, 0.0)
    assert relay.charging_hold
    relay.on_charging_hold(SimpleNamespace(data=False))
    assert relay.last_rx is None and relay.charging_cmd_rx is None


def test_undocking_rejects_reverse_turn_and_nonfinite_speed(monkeypatch):
    monkeypatch.setattr(relay_module.time, 'monotonic', lambda: 1.0)
    relay = relay_module.NavRelay.__new__(relay_module.NavRelay)
    relay.args = SimpleNamespace(max_out=0.15)
    relay.charging_hold = True
    relay.charging_hold_rx = 1.0
    outputs = []
    relay._publish = lambda vx, wz: outputs.append((vx, wz))
    relay.publish_zero = lambda why: outputs.append((0.0, 0.0))
    for vx, wz in ((-0.05, 0.0), (0.05, 0.1), (float('nan'), 0.0), (0.16, 0.0)):
        relay.on_charging_cmd(SimpleNamespace(linear=SimpleNamespace(x=vx), angular=SimpleNamespace(z=wz)))
        assert outputs[-1] == (0.0, 0.0)
        assert relay.charging_cmd_rx is None


def test_configured_radius_rejects_tight_turn_and_pivot():
    for vx, wz in ((0.1, 0.2), (0.0, 0.1)):
        assert relay_module.convert(vx, wz, max_ang=0.65, min_turning_radius=0.8)[2]
    assert relay_module.convert(0.1, 0.1, max_ang=0.65, min_turning_radius=0.8)[2] is None

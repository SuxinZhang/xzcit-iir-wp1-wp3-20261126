"""Match dock permission to a fresh, unchanged target.
将回充许可与新鲜且未改变的目标匹配。
"""
import json

from c50c_uart3.dock_safety import DockSafety
from c50c_uart3.protocol_v3_ext import DockTarget
from c50c_uart3.chassis_cmd.arbiter import Arbiter
from c50c_uart3.chassis_cmd.handshake import SessionHandshake, session_frame_for
from c50c_uart3.chassis_cmd.chain import StraightCommandChain
from c50c_uart3.chassis_cmd import check_bcc
from test_protocol_v3_ext import _load_uart3_node_with_stubs
from test_protocol_v3_ext import fw_v3_harness
from test_wp3_uplink_stream import target_frame
from types import SimpleNamespace


def permission(generation, allowed=True, session=123):
    return json.dumps({'v': 1, 'allowed': allowed, 'ttl_s': 0.4,
                       'dock': {'active': True, 'generation': generation,
                                'session': session}})


def test_only_matching_current_target_gets_permission():
    gate = DockSafety()
    target = DockTarget(True, 17, -0.1, 0.2)
    gate.on_target(target, 1.0, 123)
    gate.on_permission(permission(16), 1.0)
    assert gate.permitted_generation(1.0) is None
    gate.on_permission(permission(17), 1.0)
    assert gate.permitted_generation(1.1) == 17
    gate.on_target(DockTarget(True, 18, -0.1, -0.2), 1.1, 123)
    assert gate.permitted_generation(1.1) is None


def test_denial_timeout_and_reset_revoke_permission():
    gate = DockSafety()
    target = DockTarget(True, 17, -0.1, 0.2)
    gate.on_target(target, 1.0, 123)
    gate.on_permission(permission(17), 1.0)
    assert gate.permitted_generation(1.31) is None
    gate.on_target(target, 2.0, 123)
    gate.on_permission(permission(17), 2.0)
    gate.on_permission(permission(17, allowed=False), 2.01)
    assert gate.permitted_generation(2.01) is None
    gate.on_permission(permission(17), 2.02)
    gate.reset()
    assert gate.permitted_generation(2.02) is None


def test_old_session_permission_cannot_authorize_new_session():
    gate = DockSafety()
    gate.on_target(DockTarget(True, 17, -0.1, 0.2), 1.0, 124)
    gate.on_permission(permission(17, session=123), 1.0)
    assert gate.permitted_generation(1.0) is None
    gate.on_permission(permission(17, session=124), 1.0)
    assert gate.permitted_generation(1.0) == 17


def test_dock_enable_requires_live_session_safety_and_released_stop():
    arbiter = Arbiter()
    arbiter.mark_receiver_clean(simulated=True)
    assert not arbiter.dock_enable_allowed(1.0)
    SessionHandshake(arbiter, clock=lambda: 1.0).feed(session_frame_for(1), write=len)
    assert not arbiter.dock_enable_allowed(1.0)
    arbiter.on_safety_permission(True, 'ok', 0.4, 1.0)
    assert arbiter.dock_enable_allowed(1.0)
    arbiter.on_guard_stop(True)
    assert not arbiter.dock_enable_allowed(1.0)
    arbiter.on_guard_stop(False)
    assert not arbiter.dock_enable_allowed(1.5)


def test_dock_permission_uses_shared_motion_writer_and_sequence():
    arbiter = Arbiter()
    arbiter.mark_receiver_clean(simulated=True)
    SessionHandshake(arbiter, clock=lambda: 1.0).feed(session_frame_for(1), write=len)
    arbiter.on_safety_permission(True, 'ok', 0.4, 1.0)
    chain = StraightCommandChain(arbiter)
    first = chain.tick(1.0)
    assert first.write_ok and first.frame[1] == 0x00
    assert chain.send_dock_permission(17, 1.0)
    frame = chain._transport.writes[-1]
    assert frame[1:7] == bytes([3, 0x81, 0, 0, 0, 17])
    assert frame[9] == check_bcc(frame[:9])
    arbiter.on_guard_stop(True)
    assert not chain.send_dock_permission(17, 1.0)


def test_uart_dispatch_publishes_target_with_observed_session(monkeypatch):
    mod = _load_uart3_node_with_stubs(monkeypatch)
    node = mod.C50CUart3Node.__new__(mod.C50CUart3Node)
    node._drive_lock = mod.threading.RLock()
    node._dock_safety = DockSafety()
    arbiter = Arbiter()
    SessionHandshake(arbiter, clock=lambda: mod.time.monotonic()).feed(
        session_frame_for(123), write=len)
    node._drive_bridge = SimpleNamespace(chain=SimpleNamespace(_arbiter=arbiter))
    messages = []
    node._dock_target_publisher = SimpleNamespace(publish=messages.append)
    monkeypatch.setattr(mod, 'String', lambda data: SimpleNamespace(data=data), raising=False)
    node._handle_dock_target(target_frame(generation=17))
    payload = json.loads(messages[-1].data)
    assert payload == {'active': True, 'generation': 17, 'session': 123,
                       'vx': -0.1, 'wz': 0.2}


def test_python_stop_frame_reaches_real_c_parser():
    from c50c_uart3.protocol_v3_ext import encode_auto_dock_frame
    fw = fw_v3_harness.V3Firmware.start()
    try:
        fw.feed_bytes(session_frame_for(fw.session))
        fw.feed_bytes(encode_auto_dock_frame(True, seq=0))
        assert fw.ext_state.dock_enable
        arbiter = Arbiter()
        arbiter.mark_receiver_clean(simulated=True)
        SessionHandshake(arbiter, clock=lambda: 1.0).feed(
            session_frame_for(fw.session), write=len)
        arbiter.frame_seq.take()
        arbiter.on_safety_permission(True, 'ok', 0.4, 1.0)
        arbiter.on_guard_stop(True)
        fw.feed_bytes(arbiter.evaluate(1.0).frame)
        assert not fw.ext_state.dock_enable
        assert fw.state.safe_stop == 1
    finally:
        fw.close()

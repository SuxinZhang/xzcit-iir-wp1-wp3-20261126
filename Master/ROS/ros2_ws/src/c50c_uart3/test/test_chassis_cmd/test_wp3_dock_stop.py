"""A safety stop must be distinct from the zero used for docking handoff.
安全锁停帧必须区别于用于移交红外对接的普通零速帧。
"""

from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason
from c50c_uart3.chassis_cmd.handshake import SessionHandshake, session_frame_for


def ready_arbiter():
    arbiter = Arbiter(safety_max_age_s=1.0)
    arbiter.mark_receiver_clean(simulated=True)
    SessionHandshake(arbiter, clock=lambda: 10.0).feed(
        session_frame_for(123), write=len)
    arbiter.on_safety_permission(True, 'clear', 1.0, 10.0)
    return arbiter


def test_guard_stop_sends_stop_command_instead_of_ordinary_zero():
    arbiter = ready_arbiter()
    arbiter.on_guard_stop(True)
    decision = arbiter.evaluate(10.0)
    assert decision.zero_reason is ZeroReason.STOP_REQUESTED
    assert decision.frame[1] == 0x02


def test_safety_denial_sends_stop_command():
    arbiter = ready_arbiter()
    arbiter.on_safety_permission(False, 'obstacle', 1.0, 10.0)
    decision = arbiter.evaluate(10.0)
    assert decision.zero_reason is ZeroReason.SAFETY_DENIED
    assert decision.frame[1] == 0x02


def test_no_demand_sends_ordinary_zero_for_docking_handoff():
    arbiter = ready_arbiter()
    decision = arbiter.evaluate(10.0)
    assert decision.zero_reason is ZeroReason.NO_NAV_COMMAND
    assert decision.frame[1] == 0x00

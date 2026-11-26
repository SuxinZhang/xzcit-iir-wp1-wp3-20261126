"""A new firmware boot must invalidate old commands and lock motion."""

from c50c_uart3.chassis_cmd.arbiter import Arbiter, ZeroReason
from c50c_uart3.chassis_cmd.handshake import SessionHandshake, session_frame_for
from c50c_uart3.chassis_cmd.guard_wiring import GuardWiring
from types import SimpleNamespace


def test_new_firmware_session_locks_and_discards_old_command():
    clock = [100.0]
    arbiter = Arbiter(safety_max_age_s=10.0)
    arbiter.mark_receiver_clean(simulated=True)
    arbiter.on_publisher_count(0, 0, 0, 1)
    arbiter.on_safety_permission(True, 'clear', 10.0, clock[0])
    handshake = SessionHandshake(arbiter, clock=lambda: clock[0])

    handshake.feed(session_frame_for(1), write=len)
    arbiter.evaluate(clock[0])
    arbiter.on_auto_cmd(0.1, 0.0, 0.0, clock[0])
    assert not arbiter.evaluate(clock[0]).is_zero

    clock[0] += 0.02
    handshake.feed(session_frame_for(2), write=len)
    stopped = arbiter.evaluate(clock[0])
    assert stopped.is_zero
    assert stopped.zero_reason is ZeroReason.STOP_REQUESTED

    arbiter.on_guard_stop(False)
    assert arbiter.evaluate(clock[0]).is_zero
    arbiter.on_auto_cmd(0.1, 0.0, 0.0, clock[0])
    assert not arbiter.evaluate(clock[0]).is_zero


def test_new_session_latches_guard_until_explicit_rearm():
    clock = [100.0]
    arbiter = Arbiter(safety_max_age_s=10.0)
    arbiter.mark_receiver_clean(simulated=True)
    arbiter.on_safety_permission(True, 'clear', 10.0, clock[0])
    handshake = SessionHandshake(arbiter, clock=lambda: clock[0])
    handshake.feed(session_frame_for(1), write=len)

    class Guard:
        stopped = False

        def stop(self, cause):
            self.stopped = True

    guard = Guard()
    wiring = GuardWiring(SimpleNamespace(), port=None, device='test',
                         firmware_protocol='v3', baseline_protocol='baseline',
                         coexist=True)
    wiring.bridge = SimpleNamespace(guard=guard)
    chain = SimpleNamespace(_arbiter=arbiter)

    handshake.feed(session_frame_for(2), write=len)
    wiring.forward_stop_to_arbiter(chain)
    assert guard.stopped
    assert arbiter.evaluate(clock[0]).zero_reason is ZeroReason.STOP_REQUESTED

    guard.stopped = False
    wiring.forward_stop_to_arbiter(chain)
    assert arbiter.evaluate(clock[0]).zero_reason is not ZeroReason.STOP_REQUESTED

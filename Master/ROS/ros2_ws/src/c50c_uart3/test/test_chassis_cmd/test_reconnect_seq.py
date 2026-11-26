"""定向复现：主机重连而固件未重启时，两侧序号与授权如何变化。

问题（任务书原话）：
"核对实际重连路径：主机重连但固件未重启时，双方序号和授权如何变化。
只在现有代码或定向复现证实会影响本任务时修复；否则记录证据与限制。"

复现对象是**真实固件逻辑**（``fw_v3_harness``：真实 ``chassis_ack.c`` 的
会话门 + 序号门 + 位图 + 7 个计数器）。固件模型进程在整条用例里**不重启**
（同一个 ``V3Firmware``、同一会话 ID、位图不清），只有主机侧状态按
``Arbiter.on_serial_reconnect()`` 重置。

回归要求：同一会话重连后保留序号，首帧零速可由固件模型接受。
接收端状态另由主机验证，本测试在读取新会话帧后构造接收端干净证据。

证据边界（照抄 ``fw_v3_harness`` 模块文档）：计数器与目标速度是真实固件
逻辑的判定结果；电机、PWM、真实串口、中断时序、DTR/RTS 都**没有**验证。
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'firmware_harness'))

import fw_v3_harness                                    # noqa: E402

from c50c_uart3.chassis_cmd import encode_speed_frame  # noqa: E402
from c50c_uart3.chassis_cmd.arbiter import Arbiter
from sim_confirm import TEST_SAFETY_MAX_AGE_S, grant_safety      # noqa: E402
from c50c_uart3.chassis_cmd.seq import SEQ_MASK, SEQ_WINDOW  # noqa: E402
from c50c_uart3.chassis_cmd.session import (            # noqa: E402
    SessionExtractor, encode_session_ack,
)

VX = 0.1
DT = 0.02


@pytest.fixture()
def fw():
    model = fw_v3_harness.V3Firmware.start()
    try:
        yield model
    finally:
        model.close()


class Clock:
    def __init__(self, start: float = 100.0) -> None:
        self.t = float(start)

    def __call__(self) -> float:
        return self.t

    def tick(self, dt: float) -> float:
        self.t += float(dt)
        return self.t


class Host:
    """主机侧最小驱动：仲裁器 + 直接写入固件模型的传输。

    只做真实桥做的事：取 ``Arbiter.evaluate`` 给出的帧原样写入固件
    （相当于"串口写成功"），并把自身计数器读回来。
    """

    def __init__(self, fw_model) -> None:
        self.fw = fw_model
        self.clock = Clock()
        self.arb = Arbiter(nav_timeout_s=0.3, nav_cmd_max_age_s=0.5,
                           safety_max_age_s=TEST_SAFETY_MAX_AGE_S)
        grant_safety(self.arb)
        self.arb.on_publisher_count(1, 0)
        self.arb.mark_receiver_clean(simulated=True)   # 测试设施入口
        self.written: list[bytes] = []

    def observe_session(self) -> None:
        """线上读到会话帧 → 回填确认帧（与 ``SessionHandshake.feed`` 一致）。"""
        self.fw.feed_bytes(encode_session_ack(self.fw.session))
        self.arb.on_session_observed(self.fw.session, self.clock(), accepted=True)

    def run(self, seconds: float, vx: float = VX) -> None:
        for _ in range(int(round(seconds / DT))):
            self.clock.tick(DT)
            self.arb.on_nav_cmd(vx, 0.0, 0.0, self.clock())
            d = self.arb.evaluate(self.clock())
            if d.frame is None:                        # 流毒/接收端未知：静默
                continue
            self.fw.feed_bytes(d.frame)
            self.written.append(d.frame)

    def reconnect(self) -> None:
        """真实调用链：uart3_node._on_serial_reconnected → Arbiter。"""
        self.arb.on_serial_reconnect()
        self.observe_session()
        self.arb.mark_receiver_clean(simulated=True)


def make_host(fw_model) -> Host:
    host = Host(fw_model)
    host.observe_session()
    return host


def speed_seq(frame: bytes) -> int:
    """速度帧 buffer[2] 的 bit0..6 是逐帧序号。"""
    return frame[2] & SEQ_MASK


# ---------------------------------------------------------------- 场景 1
def test_clean_reconnect_keeps_sequence_and_firmware_accepts_it(fw):
    """固件会话不变时继续使用原序号，固件模型不拒绝新帧。"""
    host = make_host(fw)
    host.run(1.0)
    before = fw.state
    assert before.accept == 50
    assert before.stale_seq == 0 and before.vx == pytest.approx(VX, abs=1e-4)
    last = max(speed_seq(f) for f in host.written)
    assert last < SEQ_WINDOW

    host.reconnect()
    assert host.arb._seq.next_value == last + 1  # noqa: SLF001
    print(f'[clean] 重连后序号 next_value={host.arb._seq.next_value}')  # noqa: SLF001
    start = len(host.written)
    host.run(1.0)

    after = fw.state
    sent = [speed_seq(f) for f in host.written[start:]]
    accepted = after.accept - before.accept
    print(f'\n[clean] preamble last={last} accept={before.accept}\n'
          f'[clean] after reconnect: accept={after.accept} '
          f'(新增 {accepted}) stale_seq={after.stale_seq} '
          f'stale={after.stale} no_session={after.no_session} '
          f'no_seq={after.no_seq} vx={after.vx:.4f}')
    print(f'[clean] 重连后主机发出的序号前 8 个: {sent[:8]}')
    assert sent[0] == last + 1
    assert after.stale_seq == before.stale_seq
    assert accepted == len(sent)
    assert after.stale == 0 and after.no_session == 0


# ---------------------------------------------------------------- 场景 2
def test_first_zero_frame_after_reconnect_is_accepted(fw):
    """重连后首帧零速沿用下一个序号，可被固件模型接收。"""
    host = make_host(fw)
    host.run(0.4)
    last = max(speed_seq(f) for f in host.written)
    assert 1 <= last < SEQ_WINDOW
    before = fw.state

    host.reconnect()
    host.arb.on_nav_cmd(VX, 0.0, 0.0, host.clock())
    host.clock.tick(DT)
    d = host.arb.evaluate(host.clock())
    first = d.frame
    assert d.is_zero                      # 首帧零速规则
    assert speed_seq(first) == last + 1
    host.fw.feed_bytes(first)

    st = fw.state
    print(f'\n[zero-first] last={last} first_seq={speed_seq(first)} data={first[4:8].hex()} '
          f'accept={st.accept} stale_seq={st.stale_seq} armed={st.armed}')
    assert st.stale_seq == before.stale_seq
    assert st.accept == before.accept + 1
    assert st.vx == pytest.approx(0.0)


# ---------------------------------------------------------------- 场景 3
def test_zero_after_reconnect_clears_previous_target_in_firmware_model(fw):
    """同会话重连后，固件模型接受零速并清除旧目标。"""
    host = make_host(fw)
    host.run(0.4)
    vx_before = fw.state.vx
    assert vx_before == pytest.approx(VX, abs=1e-4)
    host.reconnect()
    host.run(0.4, vx=0.0)                 # 主机只想发零速
    st = fw.state
    print(f'\n[freeze] vx_before={vx_before:.4f} vx_after={st.vx:.4f} '
          f'stale_seq={st.stale_seq} accept={st.accept}')
    assert st.vx == pytest.approx(0.0, abs=1e-4)
    assert st.stale_seq == 0


# ---------------------------------------------------------------- 场景 4
def test_not_resetting_seq_on_same_session_leaves_no_stale_frames(fw):
    """对照：同一会话 ID 下**不**归零，固件零拒绝。

    这就是"只让 ``on_session_observed``（session_id 变化时才 reset）决定
    序号"的行为，也是 ``on_serial_reconnect`` 删除 ``_seq.reset()`` 后的
    效果。说明不需要改协议、不需要协议级重试。
    """
    host = make_host(fw)
    host.run(0.4)
    before = fw.state
    reserve = host.arb._seq.next_value            # noqa: SLF001

    host.reconnect()
    assert host.arb._seq.next_value == reserve               # noqa: SLF001
    host.run(0.4)
    after = fw.state
    assert after.stale_seq == 0
    assert after.accept > before.accept


# ---------------------------------------------------------------- 场景 5
def test_new_session_id_resets_seq_on_host_side():
    """对照：会话 ID 变化时主机必须重置序号（现有语义，保持不动）。

    harness 的熵源是常量桩，两个模型进程的会话 ID 恒等，因此这一条只断言
    主机侧语义。"固件重启后首帧接受任意序号"由 ``check_seq`` 的
    ``s_seq_have == 0`` 分支保证，已由 ``test_protocol_v3_integration`` 覆盖。
    """
    clock = Clock()
    a = Arbiter()
    a.on_session_observed(0x11111111, clock(), accepted=True)
    for _ in range(5):
        a._seq.take()                              # noqa: SLF001
    assert a._seq.next_value == 5                  # noqa: SLF001
    a.on_session_observed(0x11111111, clock(), accepted=True)
    assert a._seq.next_value == 5                  # noqa: SLF001——未变化不动
    a.on_serial_reconnect()
    a.on_session_observed(0x22222222, clock(), accepted=True)
    assert a._seq.next_value == 0, '会话 ID 变化必须重置序号'  # noqa: SLF001


# ---------------------------------------------------------------- 场景 6
def test_wire_session_frame_after_reconnect_carries_same_id(fw):
    """重连后线上会话帧仍可提取，且会话 ID 与重连前相同。

    这是"同一会话 ID 时不应 reset 序号"能落地的前提。会话帧节拍：每
    ``DATA_TASK_RATE``（20）拍发一帧，真实 ``Chassis_Ack_EncodeSessionFrame``。
    """
    fw.clear_wire()
    fw.tick(20)
    ids = SessionExtractor().feed(fw.wire)
    print(f'\n[wire] session_frames={len(ids)} '
          f'id={ids[-1] if ids else None:#010x} fw={fw.session:#010x}')
    assert ids and ids[-1] == fw.session

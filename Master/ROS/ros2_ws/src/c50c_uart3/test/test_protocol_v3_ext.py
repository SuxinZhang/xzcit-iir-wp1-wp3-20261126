#!/usr/bin/env python3
"""Integration and unit tests for WP2 protocol_v3 extensions and c50c_uart3 node bindings.
针对 WP2 protocol_v3 扩展帧编解码、实时固件互操作及 c50c_uart3 节点绑定的集成与单元测试。
"""
from __future__ import annotations

import pathlib
import struct
import sys
import types

import pytest

_TEST_DIR = pathlib.Path(__file__).resolve().parent
_HARNESS_DIR = _TEST_DIR / 'firmware_harness'
if str(_HARNESS_DIR) not in sys.path:
    sys.path.insert(0, str(_HARNESS_DIR))

import fw_v3_harness  # noqa: E402
from c50c_uart3.chassis_cmd import FRAME_HEADER, FRAME_TAIL, check_bcc  # noqa: E402
from c50c_uart3.chassis_cmd.session import PROTO_VERSION, SESSION_SUB  # noqa: E402
from c50c_uart3.frame_parser import FRAME_LEN as STATUS_FRAME_LEN, StatusFrameParser  # noqa: E402
from c50c_uart3.protocol_v3_ext import (  # noqa: E402
    CMD_EXT_STATUS,
    EXT_FRAME_LEN,
    EXT_HEADER,
    EXT_TAIL,
    ExtStatusFrameParser,
    encode_auto_dock_frame,
    encode_led_beep_frame,
    encode_sos_frame,
)


def _encode_session_ack(session_id: int) -> bytes:
    buf = bytearray(11)
    buf[0] = FRAME_HEADER
    buf[1] = 0x7F
    buf[2] = SESSION_SUB
    buf[3] = PROTO_VERSION
    buf[4:8] = int(session_id).to_bytes(4, 'big', signed=False)
    buf[8] = 0
    buf[9] = check_bcc(bytes(buf[:9]))
    buf[10] = FRAME_TAIL
    return bytes(buf)


def _build_50b_status_frame(
    *,
    stop_flag: int = 0,
    vx_mm_s: int = 250,
    ultrasonic_mm: int = 1234,
    voltage_mv: int = 24200,
    accel_x: int = 0x7C01,
) -> bytes:
    buf = bytearray(STATUS_FRAME_LEN)
    buf[0] = FRAME_HEADER
    buf[1] = stop_flag & 0xFF
    struct.pack_into('>hhh', buf, 2, vx_mm_s, 0, 0)
    struct.pack_into('>hhh', buf, 8, accel_x, -200, 16384)
    struct.pack_into('>hhh', buf, 14, 10, -20, 30)
    struct.pack_into('>H', buf, 20, voltage_mv)
    struct.pack_into('>iiii', buf, 22, 100, 200, 0, 0)
    struct.pack_into('>HHHH', buf, 38, 10, 20, 0, 0)
    struct.pack_into('>H', buf, 46, ultrasonic_mm)
    buf[48] = check_bcc(bytes(buf[:48]))
    buf[49] = FRAME_TAIL
    return bytes(buf)


def _build_ext_status_frame(
    *,
    charging_state: int,
    charging_current_ma: int,
    ir_mask: int,
    safety_flags: int,
    d50a_alarm_mask: int,
) -> bytes:
    buf = bytearray(EXT_FRAME_LEN)
    buf[0] = EXT_HEADER
    buf[1] = CMD_EXT_STATUS
    buf[2] = PROTO_VERSION
    buf[3] = charging_state & 0xFF
    buf[4:6] = int(charging_current_ma).to_bytes(2, 'big', signed=True)
    buf[6] = ir_mask & 0xFF
    buf[7] = safety_flags & 0xFF
    buf[8] = d50a_alarm_mask & 0xFF
    buf[9] = check_bcc(bytes(buf[:9]))
    buf[10] = EXT_TAIL
    return bytes(buf)


def test_extended_status_exposes_host_stop():
    raw = _build_ext_status_frame(charging_state=0, charging_current_ma=0,
                                  ir_mask=0, safety_flags=0x20, d50a_alarm_mask=0)
    status = ExtStatusFrameParser().feed(raw)[0]
    assert status.host_stopped is True


def test_extended_commands_blocked_before_session():
    """Extended commands must fail closed before session handshake completes.
    在 v3 会话握手完成之前，扩展控制帧必须按无会话拒绝（Fail-Closed）。
    """
    fw = fw_v3_harness.V3Firmware.start()
    try:
        fw.feed_bytes(encode_auto_dock_frame(True, -0.15, 0.25, seq=0))
        fw.feed_bytes(encode_sos_frame(True, seq=None))
        st = fw.state
        assert st.no_session == 2
        assert st.accept == 0
        assert fw.ext_state.dock_enable is False
        assert fw.ext_state.remote_sos is False
    finally:
        fw.close()


def test_auto_dock_and_sos_and_led_beep_firmware_interop():
    """Verify byte-for-byte decoding of WP2 extended commands in real protocol_v3.c.
    验证真实 C 固件协议模块对 WP2 自动回充、远程 SOS 与声光控制帧的逐字节解码。
    """
    fw = fw_v3_harness.V3Firmware.start()
    try:
        fw.feed_bytes(_encode_session_ack(fw.session))
        assert fw.state.armed is True

        fw.feed_bytes(encode_auto_dock_frame(True, -0.15, 0.25, seq=0))
        ext = fw.ext_state
        assert fw.state.accept == 1
        assert ext.dock_enable is True
        assert ext.dock_vx == pytest.approx(-0.15, abs=1e-3)
        assert ext.dock_vz == pytest.approx(0.25, abs=1e-3)

        # Unsequenced service command (byte[2] == SESSION_SUB) after session ack
        fw.feed_bytes(encode_auto_dock_frame(False, 0.0, 0.0, seq=None))
        assert fw.state.accept == 2
        assert fw.ext_state.dock_enable is False

        fw.feed_bytes(encode_sos_frame(True, seq=1))
        assert fw.state.accept == 3
        assert fw.state.safe_stop == 1
        assert fw.ext_state.remote_sos is True

        fw.feed_bytes(encode_sos_frame(False, seq=None))
        assert fw.state.accept == 4
        assert fw.ext_state.remote_sos is False

        fw.feed_bytes(encode_led_beep_frame(5, True, seq=2))
        assert fw.state.accept == 5
        assert fw.ext_state.led_mode == 5
        assert fw.ext_state.beep_enable is True
    finally:
        fw.close()


def test_ext_status_parser_extracts_interleaved_stream_safely():
    """ExtStatusFrameParser must extract 0x7C..0x7E frames even when 0x7C appears inside
    50-byte status payloads, leaving the 50-byte telemetry stream intact.
    验证混合串口流中包含负载内 0x7C 伪帧头时，扩展帧剥离器能无损提取扩展帧并保持 50 字节主帧完好。
    """
    status_bytes = _build_50b_status_frame(
        stop_flag=0x03,
        vx_mm_s=310,
        ultrasonic_mm=850,
        voltage_mv=23800,
        accel_x=0x7C01,  # Injects 0x7C inside 50-byte payload
    )
    ext_bytes = _build_ext_status_frame(
        charging_state=0x07,  # charging + allow_recharge + dock_hw_present
        charging_current_ma=1450,
        ir_mask=0x3A,         # red_state=3, channels active
        safety_flags=0x11,    # sos_active + d50a_fault
        d50a_alarm_mask=0x82, # OT + offline
    )

    ext_parser = ExtStatusFrameParser()
    status_parser = StatusFrameParser()

    mixed = status_bytes + ext_bytes[:4]
    ext1, pass1 = ext_parser.extract_and_strip(mixed)
    assert ext1 == []

    # Feed remainder split in the middle of ext frame + another status frame
    status2 = _build_50b_status_frame(ultrasonic_mm=0xFFFF, accel_x=-100)
    ext2, pass2 = ext_parser.extract_and_strip(ext_bytes[4:] + status2)

    assert len(ext2) == 1
    ef = ext2[0]
    assert ef.charging is True
    assert ef.allow_recharge is True
    assert ef.dock_hw_present is True
    assert ef.nav_walk is False
    assert ef.charging_current_ma == 1450
    assert ef.red_state == 3
    assert ef.sos_active is True
    assert ef.d50a_fault is True
    assert ef.d50a_alarm_mask == 0x82

    decoded_status = status_parser.feed(pass1 + pass2)
    assert len(decoded_status) == 2
    assert decoded_status[0].ultrasonic_front_mm == 850
    assert decoded_status[0].chassis_velocity_mm_s[0] == 310
    assert decoded_status[1].ultrasonic_front_mm == 0xFFFF


def _load_uart3_node_with_stubs(monkeypatch):
    """Install minimal temporary ROS 2 message/rclpy stubs cleanly using monkeypatch.
    仅在测试作用域内通过 monkeypatch 注入临时 ROS 2 桩模块，测试结束后自动清理。
    """
    class _FakeTime:
        def to_msg(self):
            return types.SimpleNamespace(sec=1, nanosec=0)

    class _FakeClock:
        def now(self):
            return _FakeTime()

    class _FakeLogger:
        def info(self, *_args):
            pass

        def warn(self, *_args):
            pass

        def warning(self, *_args):
            pass

        def error(self, *_args):
            pass

    class _FakeNode:
        def get_clock(self):
            return _FakeClock()

        def get_logger(self):
            return _FakeLogger()

    class _FakeRange:
        ULTRASOUND = 0

        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id='')
            self.radiation_type = 0
            self.field_of_view = 0.0
            self.min_range = 0.0
            self.max_range = 0.0
            self.range = 0.0

    class _FakeDiagnosticStatus:
        OK = b'\x00'
        WARN = b'\x01'
        ERROR = b'\x02'

        def __init__(self):
            self.name = ''
            self.hardware_id = ''
            self.level = self.OK
            self.message = ''
            self.values = []

    class _FakeKeyValue:
        def __init__(self, key: str = '', value: str = ''):
            self.key = key
            self.value = value

    def _mk_stub(name: str, attrs: dict | None = None):
        mod = types.ModuleType(name)
        for k, v in (attrs or {}).items():
            setattr(mod, k, v)
        monkeypatch.setitem(sys.modules, name, mod)
        return mod

    _mk_stub('rclpy', {'ok': lambda: False, 'init': lambda **_kw: None, 'try_shutdown': lambda: None})
    _mk_stub('rclpy.node', {'Node': _FakeNode})
    _mk_stub('rclpy.qos', {'qos_profile_sensor_data': object()})
    _mk_stub('rclpy.signals', {'SignalHandlerOptions': types.SimpleNamespace(NO=0)})
    _mk_stub('diagnostic_msgs', {})
    _mk_stub('diagnostic_msgs.msg', {
        'DiagnosticArray': type('DiagnosticArray', (), {}),
        'DiagnosticStatus': _FakeDiagnosticStatus,
        'KeyValue': _FakeKeyValue,
    })
    _mk_stub('geometry_msgs', {})
    _mk_stub('geometry_msgs.msg', {
        'Quaternion': type('Quaternion', (), {}),
        'TransformStamped': type('TransformStamped', (), {}),
        'Twist': type('Twist', (), {}),
    })
    _mk_stub('nav_msgs', {})
    _mk_stub('nav_msgs.msg', {'Odometry': type('Odometry', (), {})})
    _mk_stub('rcl_interfaces', {})
    _mk_stub('rcl_interfaces.msg', {'ParameterDescriptor': type('ParameterDescriptor', (), {})})
    _mk_stub('sensor_msgs', {})
    _mk_stub('sensor_msgs.msg', {'Imu': type('Imu', (), {}), 'Range': _FakeRange})
    _mk_stub('std_msgs', {})
    _mk_stub('std_msgs.msg', {'Bool': type('Bool', (), {}), 'String': type('String', (), {})})
    _mk_stub('std_srvs', {})
    _mk_stub('std_srvs.srv', {'SetBool': type('SetBool', (), {})})
    _mk_stub('tf2_msgs', {})
    _mk_stub('tf2_msgs.msg', {'TFMessage': type('TFMessage', (), {})})
    _mk_stub('tf2_ros', {'TransformBroadcaster': type('TransformBroadcaster', (), {})})

    monkeypatch.delitem(sys.modules, 'c50c_uart3.uart3_node', raising=False)
    import c50c_uart3.uart3_node as uart3_mod
    return uart3_mod


def test_uart3_node_wp2_topics_and_services(monkeypatch):
    """Verify ultrasonic publishing, safety/dock diagnostics, and service TX framing.
    验证 uart3_node 对超声波标准话题、安全与回充诊断话题以及服务下行帧的发布与下发行为。
    """
    try:
        uart3_mod = _load_uart3_node_with_stubs(monkeypatch)
        _run_uart3_node_wp2_assertions(uart3_mod)
    finally:
        sys.modules.pop('c50c_uart3.uart3_node', None)


def _run_uart3_node_wp2_assertions(uart3_mod) -> None:
    C50CUart3Node = uart3_mod.C50CUart3Node

    class _RecPub:
        def __init__(self):
            self.msgs = []

        def publish(self, msg):
            self.msgs.append(msg)

    class _RecPort:
        def __init__(self):
            self.writes = []

        def write_bytes(self, frame: bytes) -> int:
            self.writes.append(bytes(frame))
            return len(frame)

    class _FakeSeq:
        def __init__(self):
            self.val = 7

        def take(self) -> int:
            cur = self.val
            self.val = (self.val + 1) & 0x7F
            return cur

    node = C50CUart3Node.__new__(C50CUart3Node)
    node._ultrasonic_frame_id = 'ultrasonic_front'
    node._ultrasonic_max_range_m = 4.5
    node._ultrasonic_publisher = _RecPub()
    node._ultrasonic_std_publisher = _RecPub()
    node._safety_status_publisher = _RecPub()
    node._dock_status_publisher = _RecPub()
    node._drive_enabled = True
    node._drive_port = _RecPort()
    node._fd = 7
    node._drive_lock = uart3_mod.threading.RLock()
    node._drive_wiring = types.SimpleNamespace()
    node._guard_wiring = types.SimpleNamespace(
        bridge=types.SimpleNamespace(guard=types.SimpleNamespace(stopped=False)))
    node._drive_bridge = types.SimpleNamespace(
        chain=types.SimpleNamespace(
            _arbiter=types.SimpleNamespace(
                session_acknowledged=True,
                dock_enable_allowed=lambda now: True,
                frame_seq=_FakeSeq(),
            )
        )
    )

    parser = StatusFrameParser()
    frame = parser.feed(
        _build_50b_status_frame(stop_flag=0x02, vx_mm_s=0, ultrasonic_mm=850, voltage_mv=24100)
    )[0]
    node._publish_ultrasonic(frame, stamp=types.SimpleNamespace(sec=1, nanosec=0))
    assert len(node._ultrasonic_publisher.msgs) == 1
    assert len(node._ultrasonic_std_publisher.msgs) == 1
    rng = node._ultrasonic_std_publisher.msgs[0]
    assert rng.field_of_view == pytest.approx(0.2618)
    assert rng.min_range == pytest.approx(0.02)
    assert rng.max_range == pytest.approx(4.5)
    assert rng.range == pytest.approx(0.85)

    invalid_frame = parser.feed(
        _build_50b_status_frame(stop_flag=0x02, vx_mm_s=0, ultrasonic_mm=0xFFFF, voltage_mv=24100)
    )[0]
    node._publish_ultrasonic(invalid_frame, stamp=types.SimpleNamespace(sec=2, nanosec=0))
    assert node._ultrasonic_std_publisher.msgs[-1].range == pytest.approx(4.5)

    node._publish_safety_from_status(frame)
    assert node._safety_status_publisher.msgs[-1].message == 'SOS active'

    ext_parser = ExtStatusFrameParser()
    ext_list = ext_parser.feed(
        _build_ext_status_frame(
            charging_state=0x06,
            charging_current_ma=-120,
            ir_mask=0x23,
            safety_flags=0x28,
            d50a_alarm_mask=0x00,
        )
    )
    assert len(ext_list) == 1
    node._handle_ext_frame(ext_list[0])
    assert {kv.key: kv.value for kv in node._safety_status_publisher.msgs[-1].values}['host_stopped'] == 'true'
    dock_msg = node._dock_status_publisher.msgs[-1]
    assert dock_msg.message == 'Auto-docking'
    assert {kv.key: kv.value for kv in dock_msg.values}['red_state'] == '2'

    # Service & command encoders
    req = types.SimpleNamespace(data=True)
    resp = types.SimpleNamespace(success=False, message='')
    node._on_set_auto_dock_srv(req, resp)
    assert resp.success is True
    assert node.send_sos_command(True) is True
    assert node.send_led_beep_command(2, True) is True

    assert node._drive_port.writes[0] == encode_auto_dock_frame(True, seq=7)
    assert node._drive_port.writes[1] == encode_sos_frame(True, seq=8)
    assert node._drive_port.writes[2] == encode_led_beep_frame(2, True, seq=9)

#!/usr/bin/env python3
"""Byte-level interoperability tests between C protocol_v3 and Master/ROS c50c_uart3.
固件 C 语言 protocol_v3 模块与 Master/ROS c50c_uart3 真实 Python 解析/编码器的逐字节联调测试。
"""

from __future__ import annotations

import pathlib
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
C50C_UART3_ROOT = REPO_ROOT / "Master" / "ROS" / "ros2_ws" / "src" / "c50c_uart3"
sys.path.insert(0, str(C50C_UART3_ROOT))

# Import the real host ROS 2 protocol codec without modifying Master/.
# 直接导入 Master/ 下的真实主机端协议编解码模块，验证零退化逐字节兼容。
from c50c_uart3.chassis_cmd import encode_speed_frame  # noqa: E402
from c50c_uart3.chassis_cmd.baseline import encode_baseline_speed_frame  # noqa: E402
from c50c_uart3.chassis_cmd.seq import FrameSeq  # noqa: E402
from c50c_uart3.chassis_cmd.session import (  # noqa: E402
    encode_session_ack,
    parse_session_frame,
)
from c50c_uart3.frame_parser import StatusFrameParser  # noqa: E402
from c50c_uart3.protocol_v3_ext import (  # noqa: E402
    ExtStatusFrameParser,
    encode_auto_dock_frame,
    encode_led_beep_frame,
    encode_sos_frame,
)


def run_cli(cli_path: str, *args: str) -> list[str]:
    """Invoke proto_bridge_cli and return non-empty output lines.
    调用 proto_bridge_cli 并返回非空标准输出行列表。
    """
    proc = subprocess.run(
        [cli_path, *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return [line.strip() for line in proc.stdout.splitlines() if line.strip()]


def test_firmware_to_ros_session(cli_path: str) -> None:
    """Verify C-encoded session frames decode cleanly via c50c_uart3.parse_session_frame.
    验证 C 端编码的会话通告帧可被 c50c_uart3.parse_session_frame 精确解码。
    """
    session_id = 0x5A0F1234
    hex_out = run_cli(cli_path, "encode_session", f"{session_id:08x}")[0]
    raw = bytes.fromhex(hex_out)
    assert len(raw) == 11
    parsed_id = parse_session_frame(raw)
    assert parsed_id == session_id, f"expected {session_id:#x}, got {parsed_id!r}"


def test_firmware_to_ros_status(cli_path: str) -> None:
    """Verify C-encoded 50-byte status frames decode cleanly via StatusFrameParser.
    验证 C 端编码的 50 字节遥测帧可被 StatusFrameParser 精确解码且各字段一致。
    """
    parser = StatusFrameParser()

    # Case 1: active motion with valid ultrasonic reading (1250 mm).
    # 用例 1：使能运动状态及有效超声波测距（1250 mm）。
    lines = run_cli(
        cli_path, "encode_status",
        "0", "650", "0", "-420",
        "120", "-340", "16384",
        "-18", "25", "740",
        "24350",
        "10000", "-10000", "20000", "-20000",
        "1500", "1500", "1450", "1450",
        "1250",
    )
    frames = parser.feed(bytes.fromhex(lines[0]))
    assert len(frames) == 1
    f = frames[0]
    assert f.stop_flag == 0
    assert f.chassis_velocity_mm_s == (650, 0)
    assert f.chassis_yaw_rate_mrad_s == -420
    assert f.accel_raw_lsb == (120, -340, 16384)
    assert f.gyro_raw_lsb == (-18, 25, 740)
    assert f.voltage_mv == 24350
    assert f.encoder_counts == (10000, -10000, 20000, -20000)
    assert f.pwm_counts == (1500, 1500, 1450, 1450)
    assert f.ultrasonic_front_mm == 1250

    # Case 2: stopped state suppresses gyro_z and reports 0xFFFF sentinel ultrasonic.
    # 用例 2：停机状态强制清零 Z 轴陀螺仪并上报 0xFFFF 超声波无效哨兵值。
    lines2 = run_cli(
        cli_path, "encode_status",
        "1", "0", "0", "0",
        "0", "0", "16384",
        "5", "-6", "999",
        "23800",
        "0", "0", "0", "0",
        "0", "0", "0", "0",
        "65535",
    )
    frames2 = parser.feed(bytes.fromhex(lines2[0]))
    assert len(frames2) == 1
    f2 = frames2[0]
    assert f2.stop_flag == 1
    assert f2.gyro_raw_lsb == (5, -6, 0)
    assert f2.ultrasonic_front_mm == 0xFFFF


def test_ros_to_firmware_v3_and_baseline(cli_path: str) -> None:
    """Verify c50c_uart3-encoded ack/speed frames decode and gate properly in C RxContext.
    验证 c50c_uart3 编码的会话确认帧、v3 速度帧与基线速度帧在 C 端解析与门控一致。
    """
    session_id = 0xCAFEBABE
    seq_gen = FrameSeq()

    # Send speed before handshake -> ERR -2 (NO_SESSION), then ack -> SESSION_ACK,
    # then two valid v3 speed frames, then replay of seq=1 -> ERR -4 (SEQ_REPLAY).
    # 握手前发送速度帧应拒绝（ERR -2），随后完成握手并校验正常序号推进与重放拦截（ERR -4）。
    seq0 = seq_gen.take()
    seq1 = seq_gen.take()
    pre_ack_speed = encode_speed_frame(0.25, -0.10, seq=seq0)
    ack_frame = encode_session_ack(session_id)
    v3_speed_0 = encode_speed_frame(0.85, 0.50, seq=seq0)
    v3_speed_1 = encode_speed_frame(-0.40, -0.75, seq=seq1)
    v3_replay = encode_speed_frame(0.10, 0.00, seq=seq1)

    wire = pre_ack_speed + ack_frame + v3_speed_0 + v3_speed_1 + v3_replay
    events = run_cli(cli_path, "decode_rx", "1", f"{session_id:08x}", wire.hex())
    assert len(events) == 5, f"unexpected events: {events}"
    assert events[0] == "ERR -2"
    assert events[1] == f"SESSION_ACK {session_id:08x}"
    assert events[2].startswith("SPEED_CMD seq=0 vx=0.850000 vy=0.000000 vz=0.500000")
    assert events[3].startswith("SPEED_CMD seq=1 vx=-0.400000 vy=0.000000 vz=-0.750000")
    assert events[4] == "ERR -4"

    # Baseline dialect speed frame accepted when require_v3_session=0.
    # 在兼容基线模式下（require_v3_session=0），直接接收 c50c_uart3 基线速度帧。
    base_frame = encode_baseline_speed_frame(1.20, -0.35)
    base_events = run_cli(cli_path, "decode_rx", "0", "00000000", base_frame.hex())
    assert len(base_events) == 1
    assert base_events[0].startswith("SPEED_CMD seq=0 vx=1.200000 vy=0.000000 vz=-0.350000")


def test_wp2_ext_status_and_control_interop(cli_path: str) -> None:
    """Verify WP2 extended telemetry and service commands match byte-for-byte with C firmware.
    验证 WP2 扩展遥测帧（0x7C..0x7E）及回充/SOS/声光指令与 C 固件逐字节完全一致。
    """
    ext_hex = run_cli(cli_path, "encode_ext_status", "7", "-900", "41", "10", "129")[0]
    ext_parser = ExtStatusFrameParser()
    frames = ext_parser.feed(bytes.fromhex(ext_hex))
    assert len(frames) == 1
    ef = frames[0]
    assert ef.charging_state == 7 and ef.charging and ef.allow_recharge and ef.dock_hw_present
    assert ef.charging_current_ma == -900
    assert ef.ir_mask == 41 and ef.red_state == 2
    assert ef.safety_flags == 10 and ef.pe4_estop and ef.undervoltage
    assert ef.d50a_alarm_mask == 129

    session_id = 0xAABB1122
    seq_gen = FrameSeq()
    s0 = seq_gen.take()
    s1 = seq_gen.take()
    wire = (
        encode_session_ack(session_id)
        + encode_auto_dock_frame(True, -0.10, 0.25)
        + encode_sos_frame(True, seq=s0)
        + encode_led_beep_frame(3, True, seq=s1)
    )
    events = run_cli(cli_path, "decode_rx", "1", f"{session_id:08x}", wire.hex())
    assert len(events) == 4, f"unexpected ext events: {events}"
    assert events[0] == f"SESSION_ACK {session_id:08x}"
    assert events[1] == "AUTO_DOCK_CMD seq=0 enable=1 vx=-0.100000 vz=0.250000"
    assert events[2] == "SOS_CMD seq=0 active=1"
    assert events[3] == "LED_BEEP_CMD seq=1 mode=3 beep=1"


def main() -> int:
    """Execute all firmware <-> c50c_uart3 interoperability checks.
    运行全部固件与 c50c_uart3 逐字节互操作测试。
    """
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <path/to/proto_bridge_cli>", file=sys.stderr)
        return 2

    cli_path = sys.argv[1]
    test_firmware_to_ros_session(cli_path)
    test_firmware_to_ros_status(cli_path)
    test_ros_to_firmware_v3_and_baseline(cli_path)
    test_wp2_ext_status_and_control_interop(cli_path)
    print("test_c50c_uart3_interop: ALL PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())

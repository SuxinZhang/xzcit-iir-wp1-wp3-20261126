#!/usr/bin/env python3
"""Protocol v3 extended wire codec and telemetry parser for XZCIT-IIR WP2.
XZCIT-IIR WP2 协议 v3 扩展下行指令编码器与上行安全/回充遥测帧解析器。
"""
from __future__ import annotations

from dataclasses import dataclass

from .chassis_cmd import FRAME_HEADER, FRAME_LENGTH, FRAME_TAIL, check_bcc
from .chassis_cmd.seq import CAP_HOST_ACK, SEQ_MASK
from .chassis_cmd.session import PROTO_VERSION, SESSION_SUB

CMD_AUTO_DOCK = 0x01
CMD_DOCK_PERMISSION = 0x03
CMD_SET_SOS = 0x04
CMD_SET_LED_BEEP = 0x05
CMD_VERSION = 0x06

EXT_HEADER = 0x7C
CMD_EXT_STATUS = 0x7E
EXT_TAIL = 0x7E
EXT_FRAME_LEN = 11
VERSION_FRAME_LEN = 18


def _to_int16_be(value: int, field_name: str) -> bytes:
    """Convert a signed 16-bit integer to big-endian two's-complement bytes.
    将 16 位有符号整数按大端二进制补码编码为 2 字节序列。
    """
    if not -32768 <= value <= 32767:
        raise ValueError(f'{field_name}={value} exceeds signed int16 range [-32768, 32767]')
    return int(value).to_bytes(2, 'big', signed=True)


def _encode_sub_byte(seq: int | None) -> int:
    """Encode byte[2] using v3 7-bit sequence bit when seq is given, else SESSION_SUB (0x01).
    当指定 seq 时写入 v3 7 位序号标志字节，否则使用无序号服务子字节 SESSION_SUB (0x01)。
    """
    if seq is None:
        return SESSION_SUB
    if not isinstance(seq, int) or not 0 <= seq <= SEQ_MASK:
        raise ValueError(f'seq={seq!r} must be in range 0..{SEQ_MASK}')
    return CAP_HOST_ACK | (seq & SEQ_MASK)


def encode_auto_dock_frame(enable: bool,
                           dock_vx_m_s: float = -0.10,
                           dock_vz_rad_s: float = 0.20,
                           *,
                           seq: int | None = None) -> bytes:
    """Encode an 11-byte SET_AUTO_DOCK control command frame (cmd=0x01).
    编码 11 字节下行 SET_AUTO_DOCK 自动回充启停与对接速度配置帧（命令字 0x01）。
    """
    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_HEADER
    frame[1] = CMD_AUTO_DOCK
    frame[2] = _encode_sub_byte(seq)
    frame[3] = 1 if enable else 0
    vx_mm = int(round(float(dock_vx_m_s) * 1000.0))
    vz_mrad = int(round(float(dock_vz_rad_s) * 1000.0))
    frame[4:6] = _to_int16_be(vx_mm, 'dock_vx_m_s')
    frame[6:8] = _to_int16_be(vz_mrad, 'dock_vz_rad_s')
    frame[8] = 0
    frame[9] = check_bcc(bytes(frame[:9]))
    frame[10] = FRAME_TAIL
    return bytes(frame)


def encode_dock_permission_frame(generation: int, *, seq: int) -> bytes:
    """Encode permission for the exact infrared target generation.
    编码针对指定红外目标代次的运动许可。
    """
    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_HEADER
    frame[1] = CMD_DOCK_PERMISSION
    frame[2] = _encode_sub_byte(seq)
    frame[3:7] = generation.to_bytes(4, 'big')
    frame[9] = check_bcc(frame[:9])
    frame[10] = FRAME_TAIL
    return bytes(frame)


@dataclass(frozen=True)
class DockTarget:
    active: bool
    generation: int
    vx_m_s: float
    wz_rad_s: float


def decode_dock_target(frame: bytes) -> DockTarget:
    """Decode a complete, validated infrared target frame.
    解码已完成校验的红外目标帧。
    """
    return DockTarget(bool(frame[3]), int.from_bytes(frame[4:8], 'big'),
                      int.from_bytes(frame[8:10], 'big', signed=True) / 1000.0,
                      int.from_bytes(frame[10:12], 'big', signed=True) / 1000.0)


def encode_sos_frame(active: bool, *, seq: int | None = None) -> bytes:
    """Encode an 11-byte SET_SOS remote emergency trigger/reset command frame (cmd=0x04).
    编码 11 字节下行 SET_SOS 上位机远程 SOS 紧急报警触发/复位帧（命令字 0x04）。
    """
    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_HEADER
    frame[1] = CMD_SET_SOS
    frame[2] = _encode_sub_byte(seq)
    frame[3] = 1 if active else 0
    frame[9] = check_bcc(bytes(frame[:9]))
    frame[10] = FRAME_TAIL
    return bytes(frame)


def encode_led_beep_frame(mode: int,
                          beep_enable: bool = False,
                          *,
                          seq: int | None = None) -> bytes:
    """Encode an 11-byte SET_LED_BEEP audio/visual indicator command frame (cmd=0x05).
    编码 11 字节下行 SET_LED_BEEP 远程 RGB 灯效与蜂鸣器控制帧（命令字 0x05）。
    """
    if not isinstance(mode, int) or not 0 <= mode <= 255:
        raise ValueError(f'mode={mode!r} must be a uint8 integer 0..255')
    frame = bytearray(FRAME_LENGTH)
    frame[0] = FRAME_HEADER
    frame[1] = CMD_SET_LED_BEEP
    frame[2] = _encode_sub_byte(seq)
    frame[3] = mode & 0xFF
    frame[4] = 1 if beep_enable else 0
    frame[9] = check_bcc(bytes(frame[:9]))
    frame[10] = FRAME_TAIL
    return bytes(frame)


@dataclass(frozen=True)
class ExtTelemetryFrame:
    """Decoded 11-byte uplink extended safety and auto-dock telemetry frame (0x7C..0x7E).
    解码后的 11 字节上行扩展安全与自动回充状态遥测帧数据结构（0x7C..0x7E）。
    """

    charging_state: int
    charging_current_ma: int
    ir_mask: int
    safety_flags: int
    d50a_alarm_mask: int
    frame_bytes: bytes = b''

    @property
    def charging(self) -> bool:
        """True while battery charging is active.
        正在充电接触状态时为 true。
        """
        return bool(self.charging_state & 0x01)

    @property
    def allow_recharge(self) -> bool:
        """True when auto-dock mode is active.
        开启自动回充模式时为 true。
        """
        return bool(self.charging_state & 0x02)

    @property
    def dock_hw_present(self) -> bool:
        """True once CAN1 auto-dock hardware responded to probe.
        CAN1 回充对接硬件在线时为 true。
        """
        return bool(self.charging_state & 0x04)

    @property
    def nav_walk(self) -> bool:
        """True during host Nav2 pre-dock approach phase before IR capture.
        在捕获红外信号前的 Nav2 接近阶段为 true。
        """
        return bool(self.charging_state & 0x08)

    @property
    def red_state(self) -> int:
        """Number of docking IR channels currently receiving signal (0..4).
        当前接收到有效红外对管信号的通道总数（0..4）。
        """
        return (self.ir_mask >> 4) & 0x07

    @property
    def sos_active(self) -> bool:
        """True when physical PE5 or remote SOS emergency stop is active.
        物理 PE5 或远程 SOS 紧急报警触发时为 true。
        """
        return bool(self.safety_flags & 0x01)

    @property
    def pe4_estop(self) -> bool:
        """True when PE4 hardware E-stop switch is asserted LOW.
        硬件急停开关 PE4 拉低触发急停时为 true。
        """
        return bool(self.safety_flags & 0x02)

    @property
    def uart_timeout(self) -> bool:
        """True when chassis USART3 command watchdog timed out (> 500 ms).
        底盘 USART3 指令看门狗超时（> 500 ms）时为 true。
        """
        return bool(self.safety_flags & 0x04)

    @property
    def undervoltage(self) -> bool:
        """True when battery pack voltage dropped below lockout threshold.
        电池电压低于严重欠压停机保护阈值时为 true。
        """
        return bool(self.safety_flags & 0x08)

    @property
    def d50a_fault(self) -> bool:
        """True when D50A RS485 motor driver reported active protection alarms.
        D50A RS485 电机驱动器上报保护故障报警时为 true。
        """
        return bool(self.safety_flags & 0x10)

    @property
    def host_stopped(self) -> bool:
        """True when the host stop command has disabled motor output.
        上位机锁停命令切断电机输出时为 true。
        """
        return bool(self.safety_flags & 0x20)


@dataclass(frozen=True)
class VersionTelemetry:
    """Decoded 18-byte firmware version telemetry frame.
    解码后的 18 字节固件版本遥测帧。
    """

    major: int
    minor: int
    patch: int
    chassis_type: int
    git_sha_u32: int
    frame_bytes: bytes = b''

    @property
    def version(self) -> str:
        """Return semantic version string.
        返回语义化版本字符串。
        """
        return f'{self.major}.{self.minor}.{self.patch}'


def decode_version_frame(frame: bytes) -> VersionTelemetry:
    """Validate and decode one 18-byte version telemetry frame.
    校验并解码一帧 18 字节版本遥测帧。
    """
    if len(frame) != VERSION_FRAME_LEN:
        raise ValueError('version frame length must be 18')
    if (frame[0] != EXT_HEADER or frame[1] != CMD_VERSION
            or frame[2] != PROTO_VERSION or frame[17] != EXT_TAIL):
        raise ValueError('invalid version frame header/tail')
    if frame[16] != check_bcc(frame[:16]):
        raise ValueError('invalid version frame BCC')
    return VersionTelemetry(
        major=frame[3], minor=frame[4], patch=frame[5],
        chassis_type=frame[6],
        git_sha_u32=int.from_bytes(frame[7:11], 'big'),
        frame_bytes=frame,
    )


class ExtStatusFrameParser:
    """Streaming parser and byte-stream extractor for 11-byte 0x7C..0x7E extended frames.
    支持流式分段同步的 11 字节扩展安全与回充遥测帧（0x7C..0x7E）解析与剥离器。
    """

    def __init__(self) -> None:
        self._buf = bytearray()
        self.frames_accepted = 0
        self.candidates_rejected = 0

    def reset(self) -> None:
        """Discard partial candidate bytes upon serial reconnect.
        串口重连时清空半帧残留缓冲（诊断计数予以保留）。
        """
        self._buf.clear()

    @staticmethod
    def _decode(frame: bytes) -> ExtTelemetryFrame:
        return ExtTelemetryFrame(
            charging_state=frame[3],
            charging_current_ma=int.from_bytes(frame[4:6], 'big', signed=True),
            ir_mask=frame[6],
            safety_flags=frame[7],
            d50a_alarm_mask=frame[8],
            frame_bytes=frame,
        )

    def _consume(self,
                 data: bytes | bytearray | memoryview,
                 *,
                 emit_passthrough: bool) -> tuple[list[ExtTelemetryFrame], bytes]:
        """Process incoming bytes, extracting valid extended frames and optionally returning
        non-extended bytes untouched for primary StatusFrameParser consumption.
        处理串口输入字节流：提取合法扩展帧并可选输出剩余非扩展字节供主解析器消费。
        """
        frames: list[ExtTelemetryFrame] = []
        passthrough = bytearray()

        for b in bytes(data):
            if not self._buf:
                if b == EXT_HEADER:
                    self._buf.append(b)
                elif emit_passthrough:
                    passthrough.append(b)
                continue

            self._buf.append(b)

            # Fail fast on signature header mismatches [0x7C, 0x7E, 0x03] so ordinary bytes
            # inside primary 0x7B telemetry frames pass through without stalling.
            # 若前 3 字节未匹配 [0x7C, 0x7E, 0x03]，立即退回并放行普通数据字节以免滞留主状态帧。
            if (len(self._buf) == 2 and self._buf[1] != CMD_EXT_STATUS) or (
                len(self._buf) == 3 and self._buf[2] != PROTO_VERSION
            ):
                self.candidates_rejected += 1
                self._requeue_tail(passthrough if emit_passthrough else None)
                continue

            if len(self._buf) < EXT_FRAME_LEN:
                continue

            candidate = bytes(self._buf[:EXT_FRAME_LEN])
            if (candidate[10] == EXT_TAIL
                    and candidate[9] == check_bcc(candidate[:9])):
                frames.append(self._decode(candidate))
                self.frames_accepted += 1
                self._buf.clear()
            else:
                self.candidates_rejected += 1
                self._requeue_tail(passthrough if emit_passthrough else None)

        return frames, bytes(passthrough)

    def _requeue_tail(self, passthrough: bytearray | None) -> None:
        """Drop leading false 0x7C header, forward swept bytes, and resync on any next 0x7C.
        丢弃首个误触发的 0x7C 帧头，放行扫过字节并重同步至下一 0x7C 候选位置。
        """
        dropped_first = self._buf[0]
        if passthrough is not None:
            passthrough.append(dropped_first)
        tail = bytes(self._buf[1:])
        self._buf.clear()
        for idx, val in enumerate(tail):
            if val == EXT_HEADER:
                # Re-anchor on trailing 0x7C only if the remaining prefix can match header.
                # 重同步至尾部下一个 0x7C，并将之前非扩展字节写入放行队列。
                if passthrough is not None:
                    passthrough.extend(tail[:idx])
                rem = tail[idx:]
                self._buf.extend(rem)
                if ((len(self._buf) >= 2 and self._buf[1] != CMD_EXT_STATUS) or
                        (len(self._buf) >= 3 and self._buf[2] != PROTO_VERSION)):
                    self._requeue_tail(passthrough)
                return
        if passthrough is not None:
            passthrough.extend(tail)

    def feed(self, data: bytes | bytearray | memoryview) -> list[ExtTelemetryFrame]:
        """Parse any complete 11-byte extended telemetry frames in the input byte stream.
        从输入串口字节流中解码全部完整且校验通过的 11 字节扩展遥测帧。
        """
        frames, _ = self._consume(data, emit_passthrough=False)
        return frames

    def extract_and_strip(
        self, data: bytes | bytearray | memoryview
    ) -> tuple[list[ExtTelemetryFrame], bytes]:
        """Extract extended telemetry frames and return remaining bytes for primary parsers.
        从混合串口流中剥离并解码扩展遥测帧，返回剩余字节以供主 50 字节状态解析器处理。
        """
        return self._consume(data, emit_passthrough=True)


__all__ = [
    'CMD_AUTO_DOCK',
    'CMD_EXT_STATUS',
    'CMD_SET_LED_BEEP',
    'CMD_SET_SOS',
    'CMD_VERSION',
    'EXT_FRAME_LEN',
    'EXT_HEADER',
    'EXT_TAIL',
    'VERSION_FRAME_LEN',
    'ExtStatusFrameParser',
    'ExtTelemetryFrame',
    'VersionTelemetry',
    'decode_version_frame',
    'encode_auto_dock_frame',
    'encode_led_beep_frame',
    'encode_sos_frame',
]

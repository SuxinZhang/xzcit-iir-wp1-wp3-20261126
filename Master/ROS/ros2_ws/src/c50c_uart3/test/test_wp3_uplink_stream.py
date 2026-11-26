"""Parse complete uplink frames without extracting embedded byte patterns.
解析完整上行帧，不从帧载荷中误提取字节序列。
"""

from c50c_uart3.chassis_cmd import check_bcc
from c50c_uart3.chassis_cmd.session import encode_session_ack
from c50c_uart3.uplink_stream import UplinkStreamParser
from c50c_uart3.protocol_v3_ext import decode_dock_target, encode_dock_permission_frame
from test_protocol_v3_ext import _build_50b_status_frame, _build_ext_status_frame


def target_frame(generation=0x12345678, vx=-100, wz=200):
    frame = bytearray([0x7c, 0x7d, 3, 1])
    frame.extend(generation.to_bytes(4, 'big'))
    frame.extend(vx.to_bytes(2, 'big', signed=True))
    frame.extend(wz.to_bytes(2, 'big', signed=True))
    frame.extend((check_bcc(frame), 0x7e))
    return bytes(frame)


def test_fragmented_mixed_stream_preserves_embedded_valid_patterns():
    status = bytearray(_build_50b_status_frame())
    embedded = encode_session_ack(123)
    status[22:33] = embedded
    status[48] = check_bcc(status[:48])
    target = target_frame()
    ext = _build_ext_status_frame(charging_state=0, charging_current_ma=0,
                                  ir_mask=0, safety_flags=0, d50a_alarm_mask=0)
    session = encode_session_ack(456)
    parser = UplinkStreamParser()
    events = []
    stream = bytes(status) + target + ext + session
    for byte in stream:
        events.extend(parser.feed(bytes([byte])))
    assert [kind for kind, _ in events] == ['status', 'dock_target', 'ext_status', 'session']
    assert events[0][1] == bytes(status)
    assert events[1][1] == target
    assert events[-1][1] == session


def test_reset_discards_incomplete_frame():
    parser = UplinkStreamParser()
    assert parser.feed(target_frame()[:6]) == []
    parser.reset()
    assert parser.feed(encode_session_ack(456)) == [('session', encode_session_ack(456))]


def test_status_stop_flag_7f_remains_a_status_frame():
    status = bytearray(_build_50b_status_frame())
    status[1] = 0x7f
    status[48] = check_bcc(status[:48])
    parser = UplinkStreamParser()
    events = []
    for byte in status:
        events.extend(parser.feed(bytes([byte])))
    assert events == [('status', bytes(status))]


def test_session_subtype_high_bit_matches_existing_session_parser():
    session = bytearray(encode_session_ack(456))
    session[2] |= 0x80
    session[9] = check_bcc(session[:9])
    assert UplinkStreamParser().feed(session) == [('session', bytes(session))]


def test_target_snapshot_and_permission_generation_are_encoded():
    target = decode_dock_target(target_frame())
    assert (target.active, target.generation, target.vx_m_s, target.wz_rad_s) == (
        True, 0x12345678, -0.1, 0.2)
    frame = encode_dock_permission_frame(target.generation, seq=9)
    assert frame[:7] == bytes([0x7b, 0x03, 0x89, 0x12, 0x34, 0x56, 0x78])
    assert frame[9] == check_bcc(frame[:9]) and frame[10] == 0x7d

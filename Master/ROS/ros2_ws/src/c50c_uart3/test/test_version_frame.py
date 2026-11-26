"""Unit tests for the 18-byte firmware version telemetry frame.
18 字节固件版本遥测帧的单元测试。
"""

from c50c_uart3.chassis_cmd import check_bcc
from c50c_uart3.protocol_v3_ext import (
    CMD_VERSION,
    EXT_HEADER,
    EXT_TAIL,
    VERSION_FRAME_LEN,
    decode_version_frame,
)


def test_decode_version_frame_round_trip():
    """Decode a valid version frame with semantic version and git hash.
    解码包含语义版本与 Git 哈希的合法版本帧。
    """
    frame = bytearray(VERSION_FRAME_LEN)
    frame[0] = EXT_HEADER
    frame[1] = CMD_VERSION
    frame[2] = 3
    frame[3:6] = bytes((1, 2, 0))
    frame[6] = 0
    frame[7:11] = (0x1234ABCD).to_bytes(4, 'big')
    frame[16] = check_bcc(frame[:16])
    frame[17] = EXT_TAIL
    version = decode_version_frame(bytes(frame))
    assert version.version == '1.2.0'
    assert version.chassis_type == 0
    assert version.git_sha_u32 == 0x1234ABCD


def test_decode_version_frame_rejects_bad_bcc():
    """Reject a version frame with a corrupted checksum.
    拒绝校验和损坏的版本帧。
    """
    frame = bytearray(VERSION_FRAME_LEN)
    frame[0] = EXT_HEADER
    frame[1] = CMD_VERSION
    frame[2] = 3
    frame[17] = EXT_TAIL
    try:
        decode_version_frame(bytes(frame))
    except ValueError as exc:
        assert 'BCC' in str(exc)
    else:
        raise AssertionError('expected ValueError')

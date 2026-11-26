"""Check the Nav2 angular command against the current firmware contract.
验证 Nav2 角速度指令符合当前固件契约。
"""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'nav2'))
sys.path.insert(0, str(Path(__file__).resolve().parent / 'firmware_harness'))

import fw_v3_harness  # noqa: E402
from nav_relay import convert  # noqa: E402
from c50c_uart3.chassis_cmd import encode_speed_frame  # noqa: E402
from c50c_uart3.chassis_cmd.session import encode_session_ack  # noqa: E402
from c50c_uart3.motion_guard import direction_sectors  # noqa: E402


@pytest.mark.parametrize(('nav_wz', 'sector'), [(0.25, 90.0), (-0.25, -90.0)])
def test_angular_command_keeps_direction_and_units_to_firmware(nav_wz, sector):
    vx, wz, reason = convert(0.0, nav_wz, max_ang=0.65)
    assert reason is None
    assert wz == pytest.approx(nav_wz)
    assert direction_sectors(vx, wz) == [sector]

    firmware = fw_v3_harness.V3Firmware.start()
    try:
        firmware.feed_bytes(encode_session_ack(firmware.session))
        firmware.feed_bytes(encode_speed_frame(vx, wz, seq=0))
        assert firmware.state.vz == pytest.approx(nav_wz, abs=0.001)
    finally:
        firmware.close()

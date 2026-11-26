"""Serial DTR/RTS safe-configuration tests on a pseudo-terminal.
伪终端上的串口 DTR/RTS 安全配置测试。
"""

import os
import pty
import termios

from c50c_uart3.chassis_cmd.serial_no_reset import configure_no_reset


def test_configure_no_reset_clears_hupcl():
    """Clear HUPCL on an open pty without raising.
    在打开的伪终端上清除 HUPCL 且不抛异常。
    """
    master, slave = pty.openpty()
    try:
        configure_no_reset(slave)
        attrs = termios.tcgetattr(slave)
        assert not (attrs[2] & termios.HUPCL)
    finally:
        os.close(master)
        os.close(slave)

"""Safe DTR/RTS and HUPCL configuration for the CH9102F STM32 auto-download circuit.
用于 CH9102F STM32 一键下载电路的安全 DTR/RTS 与 HUPCL 配置。

The C50C-V4.3 CH9102F maps DTR/RTS to BOOT0/NRST. Linux tty open/close may
assert HUPCL and toggle DTR/RTS; clearing HUPCL and setting explicit idle
modem lines prevents accidental reset or ISP entry during normal operation.
C50C-V4.3 的 CH9102F 将 DTR/RTS 映射到 BOOT0/NRST；Linux tty 在 open/close 时
可能置位 HUPCL 并拉动 DTR/RTS，清除 HUPCL 并显式设置空闲调制解调器线可避免普通
运行期间误复位或误入 ISP。
"""

from __future__ import annotations

import struct

#: Sentinel meaning "leave this modem line unchanged".
#: 表示保持该调制解调器线不变的哨兵值。
UNCHANGED = -1


def configure_no_reset(fd: int, idle_dtr: int = UNCHANGED,
                       idle_rts: int = UNCHANGED) -> None:
    """Clear HUPCL/flow control and apply safe DTR/RTS idle levels.
    清除 HUPCL/硬件流控并设置安全的 DTR/RTS 空闲电平。

    Args:
        fd: Open tty file descriptor.
            已打开的 tty 文件描述符。
        idle_dtr: 1 drives DTR high, 0 drives it low, -1 leaves it unchanged.
            DTR=1 拉高、0 拉低、-1 保持不变。
        idle_rts: Same semantics for RTS.
            RTS 使用相同语义。
    """
    import termios

    attrs = termios.tcgetattr(fd)
    # Keep raw 8-bit, no parity, one stop bit; clear HUPCL and CRTSCTS.
    # 保持 8N1；清除 HUPCL 与 CRTSCTS。
    attrs[2] &= ~termios.HUPCL
    attrs[2] &= ~getattr(termios, 'CRTSCTS', 0)
    attrs[2] |= termios.CREAD | termios.CLOCAL
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    _set_modem_line(fd, termios.TIOCM_DTR, idle_dtr,
                    termios.TIOCMBIS, termios.TIOCMBIC)
    _set_modem_line(fd, termios.TIOCM_RTS, idle_rts,
                    termios.TIOCMBIS, termios.TIOCMBIC)


def _set_modem_line(fd: int, flag: int, value: int, set_ioctl: int,
                    clear_ioctl: int) -> None:
    """Apply one DTR/RTS line value through termios modem ioctls.
    通过 termios 调制解调器 ioctl 设置一路 DTR/RTS 电平。
    """
    if value == UNCHANGED:
        return
    import fcntl

    request = set_ioctl if value else clear_ioctl
    fcntl.ioctl(fd, request, struct.pack('I', flag))


__all__ = ['UNCHANGED', 'configure_no_reset']

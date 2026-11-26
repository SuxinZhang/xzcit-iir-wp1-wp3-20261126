"""Frame the mixed v3 uplink before dispatching complete frames.
先按完整帧划分 v3 上行混合字节流，再分发给对应解析器。
"""
from .chassis_cmd import check_bcc


class UplinkStreamParser:
    def __init__(self):
        self._buf = bytearray()

    def reset(self):
        self._buf.clear()

    def feed(self, data):
        self._buf.extend(data)
        frames = []
        while self._buf:
            if self._buf[0] not in (0x7b, 0x7c):
                del self._buf[0]
                continue
            if len(self._buf) < 2:
                break
            if self._buf[0] == 0x7b:
                if self._buf[1] == 0x7f:
                    if len(self._buf) < 11:
                        break
                    candidate = self._buf[:11]
                    is_session = (candidate[2] & 0x7f == 1 and candidate[3] == 3
                                  and candidate[8] == 0 and candidate[10] == 0x7d
                                  and candidate[9] == check_bcc(candidate[:9]))
                    kind, length, tail = (('session', 11, 0x7d) if is_session
                                          else ('status', 50, 0x7d))
                else:
                    kind, length, tail = 'status', 50, 0x7d
            elif self._buf[1] == 0x7e:
                kind, length, tail = 'ext_status', 11, 0x7e
            elif self._buf[1] == 0x7d:
                kind, length, tail = 'dock_target', 14, 0x7e
            else:
                del self._buf[0]
                continue
            if len(self._buf) < length:
                break
            frame = bytes(self._buf[:length])
            if kind == 'status':
                version_ok = True
            elif kind == 'session':
                version_ok = frame[2] & 0x7f == 1 and frame[3] == 3
            else:
                version_ok = frame[2] == 3
            if kind == 'dock_target' and frame[3] not in (0, 1):
                version_ok = False
            if (frame[-1] != tail or frame[-2] != check_bcc(frame[:-2])
                    or not version_ok or (kind == 'session' and frame[8] != 0)):
                del self._buf[0]
                continue
            frames.append((kind, frame))
            del self._buf[:length]
        return frames

#!/usr/bin/env python3
"""Host-side test harness compiling and driving real App/protocol/protocol_v3.c.
在宿主机上编译并驱动真实固件协议源码 App/protocol/protocol_v3.c 的测试夹具。
"""
from __future__ import annotations

import hashlib
import os
import pathlib
import re
import subprocess
import tempfile
from typing import NamedTuple

_THIS_FILE = pathlib.Path(__file__).resolve()
REPO_ROOT = _THIS_FILE.parents[7]
PROTOCOL_C = REPO_ROOT / 'App/protocol/protocol_v3.c'
PROTOCOL_H = REPO_ROOT / 'App/protocol/protocol_v3.h'

RECORDED_SHA256: dict[str, str] = {
    'App/protocol/protocol_v3.c': (
        'bedda286d5d82db97e76ae681d8b46f5aa74c284fb05bf15b24f4be54647f994'
    ),
    'App/protocol/protocol_v3.h': (
        'acaf2dab3fa8979012858262b4835f39b04ab0053d0104abfa659d3f983951de'
    ),
}

_DRIVER_C = r"""
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "protocol_v3.h"

static IIR_Proto_RxContext g_rx;
static IIR_Proto_Watchdog g_wd;
static uint32_t g_session = 0u;
static uint32_t g_tick_ms = 0u;
static uint32_t g_tick_count = 0u;
static uint32_t g_age_ticks = 0u;
static uint32_t g_epoch = 0u;
static bool g_armed = false;
static uint8_t g_safe_stop = 1u;
static float g_vx = 0.0f;
static float g_vy = 0.0f;
static float g_vz = 0.0f;

static bool g_dock_enable = false;
static uint32_t g_dock_generation = 0u;
static float g_dock_vx = 0.0f;
static float g_dock_vz = 0.0f;
static bool g_remote_sos = false;
static uint8_t g_led_mode = 0u;
static bool g_beep_enable = false;

static void emit_hex(const uint8_t *buf, size_t len) {
    for (size_t i = 0u; i < len; ++i) {
        printf("%02x", buf[i]);
    }
}

static int hex_val(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

static void handle_byte(uint8_t b) {
    IIR_ProtoRxEvent ev;
    IIR_ProtoRxStatus st = IIR_Proto_RxFeedByte(&g_rx, b, &ev);
    if (st == IIR_PROTO_RX_SESSION_ACK) {
        g_armed = true;
        g_safe_stop = 0u;
        IIR_Proto_WatchdogKick(&g_wd, g_tick_ms);
    } else if (st == IIR_PROTO_RX_SPEED_CMD) {
        g_armed = true;
        g_safe_stop = g_remote_sos ? 1u : 0u;
        g_vx = g_remote_sos ? 0.0f : ev.vx_mps;
        g_vy = g_remote_sos ? 0.0f : ev.vy_mps;
        g_vz = g_remote_sos ? 0.0f : ev.vz_rad_s;
        g_age_ticks = 0u;
        IIR_Proto_WatchdogKick(&g_wd, g_tick_ms);
    } else if (st == IIR_PROTO_RX_AUTO_DOCK_CMD) {
        g_dock_enable = ev.auto_dock_enable;
        g_dock_vx = ev.dock_vx_mps;
        g_dock_vz = ev.dock_vz_rad_s;
        g_age_ticks = 0u;
    } else if (st == IIR_PROTO_RX_STOP_CMD) {
        g_dock_enable = false;
        g_safe_stop = 1u;
        g_vx = 0.0f;
        g_vy = 0.0f;
        g_vz = 0.0f;
        IIR_Proto_WatchdogKick(&g_wd, g_tick_ms);
    } else if (st == IIR_PROTO_RX_DOCK_PERMISSION) {
        g_dock_generation = ev.dock_generation;
    } else if (st == IIR_PROTO_RX_SOS_CMD) {
        g_remote_sos = ev.sos_active;
        if (g_remote_sos) {
            g_safe_stop = 1u;
            g_vx = 0.0f;
            g_vy = 0.0f;
            g_vz = 0.0f;
        }
        g_age_ticks = 0u;
    } else if (st == IIR_PROTO_RX_LED_BEEP_CMD) {
        g_led_mode = ev.led_beep_mode;
        g_beep_enable = ev.beep_enable;
        g_age_ticks = 0u;
    }
}

static void step_one_tick(void) {
    g_tick_ms += 50u;
    g_tick_count++;
    g_age_ticks++;

    bool timed_out = IIR_Proto_WatchdogIsTimedOut(&g_wd, g_tick_ms);
    if (g_armed && timed_out) {
        g_armed = false;
        g_safe_stop = 1u;
        g_vx = 0.0f;
        g_vy = 0.0f;
        g_vz = 0.0f;
        g_epoch++;
        g_dock_enable = false;
    }

    IIR_StatusFrame status;
    memset(&status, 0, sizeof(status));
    status.stop_flag = g_safe_stop;
    status.vx_mm_s = (int16_t)(g_vx * 1000.0f);
    status.vy_mm_s = (int16_t)(g_vy * 1000.0f);
    status.vz_mrad_s = (int16_t)(g_vz * 1000.0f);
    /* Non-zero accel signature ensures normal zero-speed frames do not match echo. */
    status.accel_lsb[0] = 0x1122;
    status.accel_lsb[1] = 0x3344;
    status.accel_lsb[2] = 16384;
    status.voltage_mv = 24000u;
    status.ultrasonic_front_mm = IIR_PROTO_ULTRASONIC_INVALID_MM;

    uint8_t status_buf[IIR_PROTO_STATUS_FRAME_LEN];
    if (IIR_Proto_EncodeStatusFrame(&status, status_buf)) {
        IIR_Proto_RxSetEchoSignature(&g_rx, status_buf[1], &status_buf[3]);
        emit_hex(status_buf, sizeof(status_buf));
    }

    if ((g_tick_count % 20u) == 0u) {
        uint8_t sess_buf[IIR_PROTO_CMD_FRAME_LEN];
        if (IIR_Proto_EncodeSessionFrame(g_session, sess_buf)) {
            emit_hex(sess_buf, sizeof(sess_buf));
        }
    }
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "missing session id\n");
        return 2;
    }
    g_session = (uint32_t)strtoul(argv[1], NULL, 10);
    IIR_Proto_RxInit(&g_rx, g_session, true);
    IIR_Proto_WatchdogInit(&g_wd, IIR_PROTO_DEFAULT_WATCHDOG_MS);

    char line[65536];
    while (fgets(line, sizeof(line), stdin) != NULL) {
        size_t len = strlen(line);
        while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
            line[--len] = '\0';
        }
        if (strncmp(line, "FEED ", 5) == 0) {
            const char *hex = line + 5;
            size_t n = strlen(hex);
            uint32_t count = 0u;
            for (size_t i = 0; i + 1 < n; i += 2) {
                int hi = hex_val(hex[i]);
                int lo = hex_val(hex[i + 1]);
                if (hi >= 0 && lo >= 0) {
                    handle_byte((uint8_t)((hi << 4) | lo));
                    count++;
                }
            }
            printf("FEED %" PRIu32 "\n", count);
            fflush(stdout);
        } else if (strncmp(line, "TICK ", 5) == 0) {
            unsigned long n = strtoul(line + 5, NULL, 10);
            printf("WIRE ");
            for (unsigned long i = 0; i < n; ++i) {
                step_one_tick();
            }
            printf("\n");
            fflush(stdout);
        } else if (strcmp(line, "STATE") == 0) {
            printf("STATE %d %" PRIu32 " %" PRIu32 " %.6f %.6f %.6f "
                   "%" PRIu32 " %" PRIu32 " %" PRIu32 " %" PRIu32 " "
                   "%" PRIu32 " %" PRIu32 " %" PRIu32 " %u\n",
                   g_armed ? 1 : 0,
                   g_age_ticks,
                   g_epoch,
                   (double)g_vx,
                   (double)g_vy,
                   (double)g_vz,
                   g_rx.frames_accepted,
                   g_rx.frames_bad_session,
                   g_rx.frames_echo,
                   g_rx.frames_no_session,
                   g_rx.frames_bad_type,
                   g_rx.frames_stale_seq,
                   g_rx.frames_no_seq,
                   (unsigned)g_safe_stop);
            fflush(stdout);
        } else if (strcmp(line, "EXT") == 0) {
            printf("EXT %d %.6f %.6f %d %u %d\n",
                   g_dock_enable ? 1 : 0,
                   (double)g_dock_vx,
                   (double)g_dock_vz,
                   g_remote_sos ? 1 : 0,
                   (unsigned)g_led_mode,
                   g_beep_enable ? 1 : 0);
            fflush(stdout);
        } else if (strcmp(line, "DOCKGEN") == 0) {
            printf("DOCKGEN %" PRIu32 "\n", g_dock_generation);
            fflush(stdout);
        } else if (strcmp(line, "QUIT") == 0) {
            break;
        } else {
            printf("ERR unknown\n");
            fflush(stdout);
        }
    }
    return 0;
}
"""


class V3State(NamedTuple):
    """Snapshot of the protocol_v3 receiver and watchdog state.
    protocol_v3 接收器与看门狗状态快照。
    """

    armed: bool
    age: int
    epoch: int
    vx: float
    vy: float
    vz: float
    accept: int
    stale: int
    echo: int
    no_session: int
    bad_type: int
    stale_seq: int
    no_seq: int
    safe_stop: int


class ExtV3State(NamedTuple):
    """Snapshot of WP2 extended command state decoded by protocol_v3.c.
    protocol_v3.c 解码的 WP2 扩展命令状态快照。
    """

    dock_enable: bool
    dock_vx: float
    dock_vz: float
    remote_sos: bool
    led_mode: int
    beep_enable: bool


def firmware_hashes() -> dict[str, str]:
    """Compute SHA-256 hashes of the real firmware protocol source files.
    计算真实固件协议源码文件的 SHA-256 哈希值。
    """
    out: dict[str, str] = {}
    for rel in RECORDED_SHA256:
        path = REPO_ROOT / rel
        out[rel] = hashlib.sha256(path.read_bytes()).hexdigest()
    return out


def firmware_contract() -> dict[str, int]:
    """Parse protocol_v3 constants directly from App/protocol/protocol_v3.h.
    直接从 App/protocol/protocol_v3.h 提取 protocol_v3 协议契约宏常量。
    """
    text = PROTOCOL_H.read_text(encoding='utf-8')
    mapping = {
        'IIR_PROTO_VERSION': 'CHASSIS_PROTO_VERSION',
        'IIR_PROTO_SESSION_SUB': 'CHASSIS_SESSION_SUB',
        'IIR_PROTO_CAP_HOST_ACK': 'CHASSIS_CAP_HOST_ACK',
        'IIR_PROTO_SEQ_MASK': 'CHASSIS_SEQ_MASK',
        'IIR_PROTO_SEQ_WINDOW': 'CHASSIS_SEQ_WINDOW',
    }
    result: dict[str, int] = {}
    for c_macro, legacy_key in mapping.items():
        m = re.search(
            rf'^\s*#define\s+{c_macro}\s+(0x[0-9A-Fa-f]+|\d+)[uUlL]*',
            text,
            flags=re.MULTILINE,
        )
        if m is None:
            raise AssertionError(f'Missing macro {c_macro} in {PROTOCOL_H}')
        result[legacy_key] = int(m.group(1), 0)
    return result


_COMPILED_BIN: pathlib.Path | None = None


def _ensure_compiled() -> pathlib.Path:
    """Compile App/protocol/protocol_v3.c with the host C11 test driver.
    使用宿主机 C11 编译器联合编译真实协议实现与测试驱动二进制。
    """
    global _COMPILED_BIN
    digest_src = (
        PROTOCOL_C.read_bytes()
        + PROTOCOL_H.read_bytes()
        + _DRIVER_C.encode('utf-8')
    )
    key = hashlib.sha256(digest_src).hexdigest()[:16]
    bin_path = pathlib.Path(tempfile.gettempdir()) / f'iir_fw_v3_harness_{key}'
    if _COMPILED_BIN == bin_path and bin_path.exists():
        return bin_path

    if not bin_path.exists():
        with tempfile.TemporaryDirectory() as tmpdir:
            driver_c = pathlib.Path(tmpdir) / 'driver.c'
            driver_c.write_text(_DRIVER_C, encoding='utf-8')
            tmp_bin = pathlib.Path(tmpdir) / 'harness'
            cmd = [
                'gcc',
                '-std=c11',
                '-O2',
                '-Wall',
                '-Wextra',
                '-Werror',
                f'-I{PROTOCOL_H.parent}',
                str(PROTOCOL_C),
                str(driver_c),
                '-o',
                str(tmp_bin),
            ]
            res = subprocess.run(cmd, capture_output=True, text=True, check=False)
            if res.returncode != 0:
                raise RuntimeError(
                    f'Failed to compile fw_v3_harness C driver:\n{res.stderr}'
                )
            os.replace(tmp_bin, bin_path)

    _COMPILED_BIN = bin_path
    return bin_path


class V3Firmware:
    """Subprocess wrapper driving the compiled App/protocol/protocol_v3.c C code.
    通过子进程管道驱动编译后 App/protocol/protocol_v3.c 的 Python 封装类。
    """

    def __init__(self, proc: subprocess.Popen[str], session: int) -> None:
        self._proc = proc
        self.session = session
        self._wire = bytearray()
        self._closed = False

    @classmethod
    def start(cls) -> 'V3Firmware':
        """Launch a fresh firmware protocol instance with a random non-zero session ID.
        启动带有随机非零 32 位会话 ID 的独立固件协议实例。
        """
        bin_path = _ensure_compiled()
        rand_int = int.from_bytes(os.urandom(4), 'big')
        if rand_int == 0:
            rand_int = 0x12345678
        proc = subprocess.Popen(
            [str(bin_path), str(rand_int)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        return cls(proc, rand_int)

    def _cmd(self, line: str) -> str:
        if self._closed or self._proc.stdin is None or self._proc.stdout is None:
            raise RuntimeError('V3Firmware instance already closed')
        self._proc.stdin.write(line + '\n')
        self._proc.stdin.flush()
        resp = self._proc.stdout.readline()
        if not resp:
            stderr = self._proc.stderr.read() if self._proc.stderr else ''
            raise RuntimeError(f'fw_v3_harness EOF (stderr={stderr!r})')
        return resp.rstrip('\r\n')

    def feed_bytes(self, data: bytes | bytearray | memoryview) -> int:
        """Feed raw serial bytes into IIR_Proto_RxFeedByte.
        向底层 IIR_Proto_RxFeedByte 喂入原始串口字节流。
        """
        raw = bytes(data)
        if not raw:
            return 0
        resp = self._cmd('FEED ' + raw.hex())
        parts = resp.split()
        if len(parts) != 2 or parts[0] != 'FEED':
            raise RuntimeError(f'Unexpected FEED response: {resp!r}')
        return int(parts[1])

    def tick(self, n: int = 1) -> None:
        """Advance simulated ticks (50 ms/tick), updating watchdog and emitting wire frames.
        推进仿真周期（每拍 50 ms），维护看门狗并生成上行遥测与会话帧。
        """
        if n <= 0:
            return
        resp = self._cmd(f'TICK {int(n)}')
        if not resp.startswith('WIRE '):
            raise RuntimeError(f'Unexpected TICK response: {resp!r}')
        hex_payload = resp[5:].strip()
        if hex_payload:
            self._wire.extend(bytes.fromhex(hex_payload))

    @property
    def wire(self) -> bytes:
        """All un-cleared bytes transmitted on TX wire by the firmware.
        固件在 TX 线上发出且尚未清除的全部字节序列。
        """
        return bytes(self._wire)

    def clear_wire(self) -> None:
        """Clear buffered TX wire bytes after consumer read.
        读取后清空已消费的 TX 线缓冲字节。
        """
        self._wire.clear()

    @property
    def state(self) -> V3State:
        """Read current V3State snapshot from the C receiver context.
        读取底层 C 接收器上下文的当前 V3State 状态快照。
        """
        resp = self._cmd('STATE')
        out = resp.split()
        if len(out) != 15 or out[0] != 'STATE':
            raise RuntimeError(f'Unexpected STATE response: {resp!r}')
        return V3State(
            armed=int(out[1]) == 1,
            age=int(out[2]),
            epoch=int(out[3]),
            vx=float(out[4]),
            vy=float(out[5]),
            vz=float(out[6]),
            accept=int(out[7]),
            stale=int(out[8]),
            echo=int(out[9]),
            no_session=int(out[10]),
            bad_type=int(out[11]),
            stale_seq=int(out[12]),
            no_seq=int(out[13]),
            safe_stop=int(out[14]),
        )

    @property
    def ext_state(self) -> ExtV3State:
        """Read current ExtV3State snapshot of decoded WP2 extended commands.
        读取底层已解码 WP2 扩展指令的当前 ExtV3State 状态快照。
        """
        resp = self._cmd('EXT')
        out = resp.split()
        if len(out) != 7 or out[0] != 'EXT':
            raise RuntimeError(f'Unexpected EXT response: {resp!r}')
        return ExtV3State(
            dock_enable=int(out[1]) == 1,
            dock_vx=float(out[2]),
            dock_vz=float(out[3]),
            remote_sos=int(out[4]) == 1,
            led_mode=int(out[5]),
            beep_enable=int(out[6]) == 1,
        )

    def close(self) -> None:
        """Terminate the compiled firmware harness subprocess cleanly.
        干净终止编译后的固件测试夹具子进程。
        """
        if self._closed:
            return
        self._closed = True
        try:
            if self._proc.stdin and not self._proc.stdin.closed:
                self._proc.stdin.write('QUIT\n')
                self._proc.stdin.flush()
                self._proc.stdin.close()
        except OSError:
            pass
        try:
            self._proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._proc.kill()
            self._proc.wait()

    def __del__(self) -> None:
        self.close()

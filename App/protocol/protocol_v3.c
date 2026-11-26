/**
 * @file protocol_v3.c
 * @brief Implementation of protocol_v3 framing codec, stream resync parser,
 *        v3 session/sequence gating, and millisecond command watchdog.
 *        protocol_v3 帧编解码、字节流滑动重同步、v3 会话/序号门控及指令看门狗实现。
 */

#include "protocol_v3.h"

#include <string.h>

/* Write a signed 16-bit value in big-endian network byte order.
 * 以大端字节序向缓冲区写入 16 位有符号整数。 */
static void write_i16_be(uint8_t *dst, int16_t val)
{
    const uint16_t u = (uint16_t)val;
    dst[0] = (uint8_t)((u >> 8u) & 0xFFu);
    dst[1] = (uint8_t)(u & 0xFFu);
}

/* Write an unsigned 16-bit value in big-endian network byte order.
 * 以大端字节序向缓冲区写入 16 位无符号整数。 */
static void write_u16_be(uint8_t *dst, uint16_t val)
{
    dst[0] = (uint8_t)((val >> 8u) & 0xFFu);
    dst[1] = (uint8_t)(val & 0xFFu);
}

/* Write a signed 32-bit value in big-endian network byte order.
 * 以大端字节序向缓冲区写入 32 位有符号整数。 */
static void write_i32_be(uint8_t *dst, int32_t val)
{
    const uint32_t u = (uint32_t)val;
    dst[0] = (uint8_t)((u >> 24u) & 0xFFu);
    dst[1] = (uint8_t)((u >> 16u) & 0xFFu);
    dst[2] = (uint8_t)((u >> 8u) & 0xFFu);
    dst[3] = (uint8_t)(u & 0xFFu);
}

/* Write an unsigned 32-bit value in big-endian network byte order.
 * 以大端字节序向缓冲区写入 32 位无符号整数。 */
static void write_u32_be(uint8_t *dst, uint32_t val)
{
    dst[0] = (uint8_t)((val >> 24u) & 0xFFu);
    dst[1] = (uint8_t)((val >> 16u) & 0xFFu);
    dst[2] = (uint8_t)((val >> 8u) & 0xFFu);
    dst[3] = (uint8_t)(val & 0xFFu);
}

/* Read a signed 16-bit integer from two big-endian bytes.
 * 从两个大端字节解码 16 位有符号整数。 */
static int16_t read_i16_be(const uint8_t *src)
{
    const uint16_t u = (uint16_t)(((uint16_t)src[0] << 8u) | (uint16_t)src[1]);
    return (int16_t)u;
}

/* Read an unsigned 32-bit integer from four big-endian bytes.
 * 从四个大端字节解码 32 位无符号整数。 */
static uint32_t read_u32_be(const uint8_t *src)
{
    return ((uint32_t)src[0] << 24u) |
           ((uint32_t)src[1] << 16u) |
           ((uint32_t)src[2] << 8u) |
           (uint32_t)src[3];
}

/* Slide buffer past byte[0] and advance to the next 0x7B frame header candidate.
 * 丢弃缓冲区首字节并滑动至下一个 0x7B 帧头候选位置，实现流式重同步。 */
static void resync_shift_buffer(IIR_Proto_RxContext *ctx)
{
    uint8_t next_hdr = ctx->len;
    for (uint8_t i = 1u; i < ctx->len; ++i) {
        if (ctx->buf[i] == IIR_PROTO_FRAME_HEADER) {
            next_hdr = i;
            break;
        }
    }

    if (next_hdr >= ctx->len) {
        ctx->len = 0u;
        return;
    }

    const uint8_t remaining = (uint8_t)(ctx->len - next_hdr);
    memmove(ctx->buf, &ctx->buf[next_hdr], remaining);
    ctx->len = remaining;
}

uint8_t IIR_Proto_CheckBcc(const uint8_t *data, size_t len)
{
    if (data == NULL) {
        return 0u;
    }
    uint8_t bcc = 0u;
    for (size_t i = 0u; i < len; ++i) {
        bcc ^= data[i];
    }
    return bcc;
}

bool IIR_Proto_EncodeStatusFrame(const IIR_StatusFrame *status,
                                 uint8_t out_buf[IIR_PROTO_STATUS_FRAME_LEN])
{
    if (status == NULL || out_buf == NULL) {
        return false;
    }

    out_buf[0] = IIR_PROTO_FRAME_HEADER;
    out_buf[1] = status->stop_flag;

    write_i16_be(&out_buf[2], status->vx_mm_s);
    write_i16_be(&out_buf[4], status->vy_mm_s);
    write_i16_be(&out_buf[6], status->vz_mrad_s);

    write_i16_be(&out_buf[8], status->accel_lsb[0]);
    write_i16_be(&out_buf[10], status->accel_lsb[1]);
    write_i16_be(&out_buf[12], status->accel_lsb[2]);

    write_i16_be(&out_buf[14], status->gyro_lsb[0]);
    write_i16_be(&out_buf[16], status->gyro_lsb[1]);
    /* Suppress Z-axis gyro output when stopped to match c50c_uart3 conversions.py contract.
     * 在停机状态下强制将 Z 轴陀螺仪置零，以对齐 c50c_uart3 conversions.py 的协议约定。 */
    const int16_t gz = (status->stop_flag == 0u) ? status->gyro_lsb[2] : (int16_t)0;
    write_i16_be(&out_buf[18], gz);

    write_u16_be(&out_buf[20], status->voltage_mv);

    for (size_t i = 0u; i < 4u; ++i) {
        write_i32_be(&out_buf[22u + i * 4u], status->encoder_counts[i]);
    }
    for (size_t i = 0u; i < 4u; ++i) {
        write_u16_be(&out_buf[38u + i * 2u], status->pwm_counts[i]);
    }

    write_u16_be(&out_buf[46], status->ultrasonic_front_mm);
    out_buf[48] = IIR_Proto_CheckBcc(out_buf, 48u);
    out_buf[49] = IIR_PROTO_FRAME_TAIL;
    return true;
}

bool IIR_Proto_EncodeSessionFrame(uint32_t session_id,
                                  uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN])
{
    if (out_buf == NULL) {
        return false;
    }

    out_buf[0] = IIR_PROTO_FRAME_HEADER;
    out_buf[1] = IIR_PROTO_CMD_SESSION;
    out_buf[2] = IIR_PROTO_SESSION_SUB;
    out_buf[3] = IIR_PROTO_VERSION;
    write_u32_be(&out_buf[4], session_id);
    out_buf[8] = 0x00u;
    out_buf[9] = IIR_Proto_CheckBcc(out_buf, 9u);
    out_buf[10] = IIR_PROTO_FRAME_TAIL;
    return true;
}

bool IIR_Proto_EncodeExtStatusFrame(const IIR_ExtStatusFrame *frame,
                                     uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN])
{
    if (frame == NULL || out_buf == NULL) {
        return false;
    }

    out_buf[0] = IIR_PROTO_EXT_HEADER;
    out_buf[1] = IIR_PROTO_CMD_EXT_STATUS;
    out_buf[2] = IIR_PROTO_VERSION;
    out_buf[3] = frame->charging_state;
    write_i16_be(&out_buf[4], frame->charging_current_ma);
    out_buf[6] = frame->ir_mask;
    out_buf[7] = frame->safety_flags;
    out_buf[8] = frame->d50a_alarm_mask;
    out_buf[9] = IIR_Proto_CheckBcc(out_buf, 9u);
    out_buf[10] = IIR_PROTO_EXT_TAIL;
    return true;
}

bool IIR_Proto_EncodeVersionFrame(const IIR_FwVersionFrame *version,
                                  uint8_t out_buf[IIR_PROTO_VERSION_FRAME_LEN])
{
    if (version == NULL || out_buf == NULL) {
        return false;
    }
    out_buf[0] = IIR_PROTO_EXT_HEADER;
    out_buf[1] = IIR_PROTO_CMD_VERSION;
    out_buf[2] = IIR_PROTO_VERSION;
    out_buf[3] = version->major;
    out_buf[4] = version->minor;
    out_buf[5] = version->patch;
    out_buf[6] = version->chassis_type;
    write_u32_be(&out_buf[7], version->git_sha_u32);
    out_buf[16] = IIR_Proto_CheckBcc(out_buf, 16u);
    out_buf[17] = IIR_PROTO_EXT_TAIL;
    return true;
}

bool IIR_Proto_EncodeDockTargetFrame(const IIR_DockTargetFrame *target,
                                    uint8_t out_buf[IIR_PROTO_DOCK_TARGET_FRAME_LEN])
{
    if (target == NULL || out_buf == NULL) {
        return false;
    }
    out_buf[0] = IIR_PROTO_EXT_HEADER;
    out_buf[1] = IIR_PROTO_CMD_DOCK_TARGET;
    out_buf[2] = IIR_PROTO_VERSION;
    out_buf[3] = target->active ? 1u : 0u;
    write_u32_be(&out_buf[4], target->generation);
    write_i16_be(&out_buf[8], target->vx_mm_s);
    write_i16_be(&out_buf[10], target->wz_mrad_s);
    out_buf[12] = IIR_Proto_CheckBcc(out_buf, 12u);
    out_buf[13] = IIR_PROTO_EXT_TAIL;
    return true;
}

bool IIR_Proto_DecodeExtStatusFrame(const uint8_t in_buf[IIR_PROTO_CMD_FRAME_LEN],
                                     IIR_ExtStatusFrame *out_frame)
{
    if (in_buf == NULL || out_frame == NULL) {
        return false;
    }

    if (in_buf[0] != IIR_PROTO_EXT_HEADER ||
        in_buf[1] != IIR_PROTO_CMD_EXT_STATUS ||
        in_buf[2] != IIR_PROTO_VERSION ||
        in_buf[10] != IIR_PROTO_EXT_TAIL) {
        return false;
    }

    if (in_buf[9] != IIR_Proto_CheckBcc(in_buf, 9u)) {
        return false;
    }

    out_frame->charging_state = in_buf[3];
    out_frame->charging_current_ma = read_i16_be(&in_buf[4]);
    out_frame->ir_mask = in_buf[6];
    out_frame->safety_flags = in_buf[7];
    out_frame->d50a_alarm_mask = in_buf[8];
    return true;
}

bool IIR_Proto_EncodeAutoDockCmd(bool enable,
                                 int16_t dock_vx_mm_s,
                                 int16_t dock_vz_mrad_s,
                                 uint8_t seq,
                                 bool use_v3_seq,
                                 uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN])
{
    if (out_buf == NULL) {
        return false;
    }

    out_buf[0] = IIR_PROTO_FRAME_HEADER;
    out_buf[1] = IIR_PROTO_CMD_AUTO_DOCK;
    out_buf[2] = use_v3_seq ? (uint8_t)(IIR_PROTO_CAP_HOST_ACK | (seq & IIR_PROTO_SEQ_MASK))
                            : IIR_PROTO_SESSION_SUB;
    out_buf[3] = enable ? 1u : 0u;
    write_i16_be(&out_buf[4], dock_vx_mm_s);
    write_i16_be(&out_buf[6], dock_vz_mrad_s);
    out_buf[8] = 0x00u;
    out_buf[9] = IIR_Proto_CheckBcc(out_buf, 9u);
    out_buf[10] = IIR_PROTO_FRAME_TAIL;
    return true;
}

bool IIR_Proto_EncodeSetSosCmd(bool active,
                               uint8_t seq,
                               bool use_v3_seq,
                               uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN])
{
    if (out_buf == NULL) {
        return false;
    }

    out_buf[0] = IIR_PROTO_FRAME_HEADER;
    out_buf[1] = IIR_PROTO_CMD_SET_SOS;
    out_buf[2] = use_v3_seq ? (uint8_t)(IIR_PROTO_CAP_HOST_ACK | (seq & IIR_PROTO_SEQ_MASK))
                            : IIR_PROTO_SESSION_SUB;
    out_buf[3] = active ? 1u : 0u;
    out_buf[4] = 0x00u;
    out_buf[5] = 0x00u;
    out_buf[6] = 0x00u;
    out_buf[7] = 0x00u;
    out_buf[8] = 0x00u;
    out_buf[9] = IIR_Proto_CheckBcc(out_buf, 9u);
    out_buf[10] = IIR_PROTO_FRAME_TAIL;
    return true;
}

bool IIR_Proto_EncodeSetLedBeepCmd(uint8_t mode,
                                   bool beep_enable,
                                   uint8_t seq,
                                   bool use_v3_seq,
                                   uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN])
{
    if (out_buf == NULL) {
        return false;
    }

    out_buf[0] = IIR_PROTO_FRAME_HEADER;
    out_buf[1] = IIR_PROTO_CMD_SET_LED_BEEP;
    out_buf[2] = use_v3_seq ? (uint8_t)(IIR_PROTO_CAP_HOST_ACK | (seq & IIR_PROTO_SEQ_MASK))
                            : IIR_PROTO_SESSION_SUB;
    out_buf[3] = mode;
    out_buf[4] = beep_enable ? 1u : 0u;
    out_buf[5] = 0x00u;
    out_buf[6] = 0x00u;
    out_buf[7] = 0x00u;
    out_buf[8] = 0x00u;
    out_buf[9] = IIR_Proto_CheckBcc(out_buf, 9u);
    out_buf[10] = IIR_PROTO_FRAME_TAIL;
    return true;
}

void IIR_Proto_RxSetEchoSignature(IIR_Proto_RxContext *ctx,
                                  uint8_t cmd_byte,
                                  const uint8_t payload6[6])
{
    if (ctx == NULL || payload6 == NULL) {
        return;
    }
    ctx->echo_cmd_byte = cmd_byte;
    memcpy(ctx->echo_payload6, payload6, 6u);
    ctx->has_echo_sig = true;
}

void IIR_Proto_RxInit(IIR_Proto_RxContext *ctx,
                      uint32_t session_id,
                      bool require_v3_session)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->session_id = session_id;
    ctx->require_v3_session = require_v3_session;
}

/* Validate 7-bit sliding-window sequence number against the active session.
 * 在当前已激活会话内校验 7 位回绕帧序号是否满足防重放与前向窗口（64）约束。 */
static IIR_ProtoRxStatus check_v3_sequence(IIR_Proto_RxContext *ctx, uint8_t seq)
{
    if (!ctx->has_seq) {
        /* First valid v3 frame in an acknowledged session establishes the sequence anchor.
         * 已确认会话内的首帧合法序号指令可从任意序号起步，并建立基准序号锚点。 */
        ctx->has_seq = true;
        ctx->last_seq = seq;
        return IIR_PROTO_RX_SPEED_CMD;
    }

    const uint8_t diff = (uint8_t)(((uint32_t)seq - (uint32_t)ctx->last_seq) &
                                   IIR_PROTO_SEQ_MASK);
    if (diff == 0u) {
        ctx->frames_rejected++;
        ctx->frames_stale_seq++;
        return IIR_PROTO_RX_ERR_SEQ_REPLAY;
    }
    if (diff >= IIR_PROTO_SEQ_WINDOW) {
        ctx->frames_rejected++;
        ctx->frames_stale_seq++;
        return IIR_PROTO_RX_ERR_SEQ_WINDOW;
    }
    ctx->last_seq = seq;
    return IIR_PROTO_RX_SPEED_CMD;
}

/* Validate v3 session and sequence gate for extended command frames (0x01/0x04/0x05).
 * 对扩展控制帧（0x01/0x04/0x05）执行会话与序号或无序号服务字节检查。 */
static IIR_ProtoRxStatus gate_extended_cmd(IIR_Proto_RxContext *ctx,
                                           uint8_t seq_byte,
                                           uint8_t *out_seq)
{
    const bool has_seq_bit = ((seq_byte & IIR_PROTO_CAP_HOST_ACK) != 0u);
    const uint8_t seq = (uint8_t)(seq_byte & IIR_PROTO_SEQ_MASK);
    *out_seq = has_seq_bit ? seq : 0u;

    if (!ctx->require_v3_session) {
        return IIR_PROTO_RX_SPEED_CMD;
    }

    if (!ctx->session_acked) {
        ctx->frames_rejected++;
        ctx->frames_no_session++;
        return IIR_PROTO_RX_ERR_NO_SESSION;
    }

    if (has_seq_bit) {
        return check_v3_sequence(ctx, seq);
    }

    if (seq_byte == IIR_PROTO_SESSION_SUB) {
        /* Unsequenced service command (byte[2] == 0x01) allowed after session handshake.
         * 完成会话握手后允许通过固定子字节 0x01 下发无序号服务控制指令。 */
        return IIR_PROTO_RX_SPEED_CMD;
    }

    ctx->frames_rejected++;
    ctx->frames_no_seq++;
    return IIR_PROTO_RX_ERR_NO_SEQ;
}

/* Process a framing- and BCC-verified 11-byte command/session frame.
 * 处理已通过帧头/帧尾和 BCC 校验的 11 字节控制帧或会话帧。 */
static IIR_ProtoRxStatus process_verified_frame(IIR_Proto_RxContext *ctx,
                                                const uint8_t frame[IIR_PROTO_CMD_FRAME_LEN],
                                                IIR_ProtoRxEvent *ev)
{
    const uint8_t cmd = frame[1];

    /* Reject frames matching recorded uplink telemetry signature as local serial TX echo.
     * 若接收帧与本端记录的上行遥测字节签名一致，判定为本地回显并丢弃。 */
    if (ctx->has_echo_sig &&
        cmd == ctx->echo_cmd_byte &&
        memcmp(&frame[3], ctx->echo_payload6, 6u) == 0) {
        ctx->frames_rejected++;
        ctx->frames_echo++;
        return IIR_PROTO_RX_ERR_ECHO;
    }

    if (cmd == IIR_PROTO_CMD_SESSION) {
        /* Session ack requires strict sub-byte 0x01, version 3, byte[8]==0, and exact ID.
         * 会话确认帧严格要求子字节为 0x01、版本号为 3、byte[8]==0 且会话 ID 完全匹配。 */
        const uint32_t ack_id = read_u32_be(&frame[4]);
        ev->session_id = ack_id;
        if (frame[2] != IIR_PROTO_SESSION_SUB ||
            frame[3] != IIR_PROTO_VERSION ||
            frame[8] != 0x00u ||
            ack_id != ctx->session_id) {
            ctx->frames_rejected++;
            ctx->frames_bad_session++;
            return IIR_PROTO_RX_ERR_BAD_SESSION;
        }
        if (!ctx->session_acked) {
            ctx->session_acked = true;
            ctx->has_seq = false;
            ctx->last_seq = 0u;
        }
        return IIR_PROTO_RX_SESSION_ACK;
    }

    if (cmd == IIR_PROTO_CMD_SPEED || cmd == IIR_PROTO_CMD_STOP ||
        cmd == IIR_PROTO_CMD_DOCK_PERMISSION) {
        const uint8_t seq_byte = frame[2];
        const uint8_t seq = (uint8_t)(seq_byte & IIR_PROTO_SEQ_MASK);
        ev->seq = seq;

        if (ctx->require_v3_session) {
            if (!ctx->session_acked) {
                ctx->frames_rejected++;
                ctx->frames_no_session++;
                return IIR_PROTO_RX_ERR_NO_SESSION;
            }
            if ((seq_byte & IIR_PROTO_CAP_HOST_ACK) == 0u) {
                ctx->frames_rejected++;
                ctx->frames_no_seq++;
                return IIR_PROTO_RX_ERR_NO_SEQ;
            }
            const IIR_ProtoRxStatus seq_st = check_v3_sequence(ctx, seq);
            if (seq_st < 0) {
                return seq_st;
            }
        }

        if (cmd == IIR_PROTO_CMD_STOP) {
            ctx->frames_accepted++;
            return IIR_PROTO_RX_STOP_CMD;
        }
        if (cmd == IIR_PROTO_CMD_DOCK_PERMISSION) {
            ev->dock_generation = read_u32_be(&frame[3]);
            ctx->frames_accepted++;
            return IIR_PROTO_RX_DOCK_PERMISSION;
        }
        ev->vx_mps = (float)read_i16_be(&frame[3]) / 1000.0f;
        ev->vy_mps = (float)read_i16_be(&frame[5]) / 1000.0f;
        ev->vz_rad_s = (float)read_i16_be(&frame[7]) / 1000.0f;
        ctx->frames_accepted++;
        return IIR_PROTO_RX_SPEED_CMD;
    }

    if (cmd == IIR_PROTO_CMD_AUTO_DOCK) {
        const IIR_ProtoRxStatus gate = gate_extended_cmd(ctx, frame[2], &ev->seq);
        if (gate < 0) {
            return gate;
        }
        ev->auto_dock_enable = (frame[3] != 0u);
        ev->dock_vx_mps = (float)read_i16_be(&frame[4]) / 1000.0f;
        ev->dock_vz_rad_s = (float)read_i16_be(&frame[6]) / 1000.0f;
        ctx->frames_accepted++;
        return IIR_PROTO_RX_AUTO_DOCK_CMD;
    }

    if (cmd == IIR_PROTO_CMD_SET_SOS) {
        const IIR_ProtoRxStatus gate = gate_extended_cmd(ctx, frame[2], &ev->seq);
        if (gate < 0) {
            return gate;
        }
        ev->sos_active = (frame[3] != 0u);
        ctx->frames_accepted++;
        return IIR_PROTO_RX_SOS_CMD;
    }

    if (cmd == IIR_PROTO_CMD_SET_LED_BEEP) {
        const IIR_ProtoRxStatus gate = gate_extended_cmd(ctx, frame[2], &ev->seq);
        if (gate < 0) {
            return gate;
        }
        ev->led_beep_mode = frame[3];
        ev->beep_enable = (frame[4] != 0u);
        ctx->frames_accepted++;
        return IIR_PROTO_RX_LED_BEEP_CMD;
    }

    ctx->frames_rejected++;
    ctx->frames_bad_type++;
    return IIR_PROTO_RX_ERR_UNKNOWN_CMD;
}

IIR_ProtoRxStatus IIR_Proto_RxFeedByte(IIR_Proto_RxContext *ctx,
                                       uint8_t byte,
                                       IIR_ProtoRxEvent *out_event)
{
    IIR_ProtoRxEvent local_ev;
    memset(&local_ev, 0, sizeof(local_ev));
    local_ev.status = IIR_PROTO_RX_INCOMPLETE;

    if (ctx == NULL) {
        if (out_event != NULL) {
            *out_event = local_ev;
        }
        return IIR_PROTO_RX_INCOMPLETE;
    }

    if (ctx->len == 0u) {
        if (byte == IIR_PROTO_FRAME_HEADER) {
            ctx->buf[0] = byte;
            ctx->len = 1u;
        }
        if (out_event != NULL) {
            *out_event = local_ev;
        }
        return IIR_PROTO_RX_INCOMPLETE;
    }

    ctx->buf[ctx->len++] = byte;
    if (ctx->len < (uint8_t)IIR_PROTO_CMD_FRAME_LEN) {
        if (out_event != NULL) {
            *out_event = local_ev;
        }
        return IIR_PROTO_RX_INCOMPLETE;
    }

    const uint8_t expected_bcc = IIR_Proto_CheckBcc(ctx->buf, 9u);
    if (ctx->buf[10] != IIR_PROTO_FRAME_TAIL || ctx->buf[9] != expected_bcc) {
        /* Reject corrupted frame and shift stream to the next 0x7B candidate.
         * 校验失败时拒绝当前候选帧，并将流窗口滑动至下一个 0x7B 起始字节。 */
        ctx->frames_rejected++;
        resync_shift_buffer(ctx);
        local_ev.status = IIR_PROTO_RX_ERR_BAD_FRAME;
        if (out_event != NULL) {
            *out_event = local_ev;
        }
        return local_ev.status;
    }

    uint8_t verified[IIR_PROTO_CMD_FRAME_LEN];
    memcpy(verified, ctx->buf, sizeof(verified));
    ctx->len = 0u;

    local_ev.status = process_verified_frame(ctx, verified, &local_ev);
    if (out_event != NULL) {
        *out_event = local_ev;
    }
    return local_ev.status;
}

void IIR_Proto_WatchdogInit(IIR_Proto_Watchdog *wd, uint32_t timeout_ms)
{
    if (wd == NULL) {
        return;
    }
    wd->timeout_ms = (timeout_ms > 0u) ? timeout_ms : IIR_PROTO_DEFAULT_WATCHDOG_MS;
    wd->last_kick_ms = 0u;
    wd->armed = false;
}

void IIR_Proto_WatchdogKick(IIR_Proto_Watchdog *wd, uint32_t now_ms)
{
    if (wd == NULL) {
        return;
    }
    wd->last_kick_ms = now_ms;
    wd->armed = true;
}

bool IIR_Proto_WatchdogIsTimedOut(const IIR_Proto_Watchdog *wd, uint32_t now_ms)
{
    if (wd == NULL || !wd->armed) {
        return true;
    }
    /* Unsigned subtraction remains exact across 32-bit millisecond tick wrap-around.
     * 无符号减法可正确处理 32 位毫秒计数器的溢出回绕。 */
    const uint32_t elapsed_ms = (uint32_t)(now_ms - wd->last_kick_ms);
    return elapsed_ms > wd->timeout_ms;
}

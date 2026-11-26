/**
 * @file test_protocol_v3.c
 * @brief Host unit tests for App/protocol/protocol_v3 framing, session/sequence
 *        gating, stream resync, and Fail-Closed millisecond watchdog.
 *        protocol_v3 帧编解码、会话/序号门控、流重同步及看门狗的宿主机单元测试。
 */

#include "protocol_v3.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void build_speed_frame(uint8_t out[IIR_PROTO_CMD_FRAME_LEN],
                              uint8_t seq_byte,
                              int16_t vx_mm_s,
                              int16_t vy_mm_s,
                              int16_t vz_mrad_s)
{
    out[0] = IIR_PROTO_FRAME_HEADER;
    out[1] = IIR_PROTO_CMD_SPEED;
    out[2] = seq_byte;
    out[3] = (uint8_t)(((uint16_t)vx_mm_s >> 8u) & 0xFFu);
    out[4] = (uint8_t)((uint16_t)vx_mm_s & 0xFFu);
    out[5] = (uint8_t)(((uint16_t)vy_mm_s >> 8u) & 0xFFu);
    out[6] = (uint8_t)((uint16_t)vy_mm_s & 0xFFu);
    out[7] = (uint8_t)(((uint16_t)vz_mrad_s >> 8u) & 0xFFu);
    out[8] = (uint8_t)((uint16_t)vz_mrad_s & 0xFFu);
    out[9] = IIR_Proto_CheckBcc(out, 9u);
    out[10] = IIR_PROTO_FRAME_TAIL;
}

static IIR_ProtoRxStatus feed_frame(IIR_Proto_RxContext *ctx,
                                    const uint8_t *bytes,
                                    size_t count,
                                    IIR_ProtoRxEvent *last_ev)
{
    IIR_ProtoRxStatus st = IIR_PROTO_RX_INCOMPLETE;
    for (size_t i = 0u; i < count; ++i) {
        st = IIR_Proto_RxFeedByte(ctx, bytes[i], last_ev);
    }
    return st;
}

static void test_status_frame_encoding(void)
{
    IIR_StatusFrame frame = {
        .stop_flag = 0u,
        .vx_mm_s = 450,
        .vy_mm_s = 0,
        .vz_mrad_s = -320,
        .accel_lsb = { 100, -200, 16384 },
        .gyro_lsb = { -15, 22, 650 },
        .voltage_mv = 24600u,
        .encoder_counts = { 123456, -789, 42, -99999 },
        .pwm_counts = { 1200u, 1200u, 1180u, 1180u },
        .ultrasonic_front_mm = 875u,
    };

    uint8_t raw[IIR_PROTO_STATUS_FRAME_LEN] = {0};
    assert(IIR_Proto_EncodeStatusFrame(&frame, raw));
    assert(raw[0] == 0x7Bu);
    assert(raw[1] == 0u);
    assert(raw[46] == 0x03u && raw[47] == 0x6Bu); /* 875 = 0x036B */
    assert(raw[48] == IIR_Proto_CheckBcc(raw, 48u));
    assert(raw[49] == 0x7Du);

    /* When stop_flag != 0, gyro_z bytes [18..19] must be suppressed to zero.
     * 当 stop_flag 非零时，第 18..19 字节的 Z 轴陀螺仪输出必须强制清零。 */
    frame.stop_flag = 1u;
    frame.ultrasonic_front_mm = IIR_PROTO_ULTRASONIC_INVALID_MM;
    assert(IIR_Proto_EncodeStatusFrame(&frame, raw));
    assert(raw[18] == 0u && raw[19] == 0u);
    assert(raw[46] == 0xFFu && raw[47] == 0xFFu);
    assert(raw[48] == IIR_Proto_CheckBcc(raw, 48u));
}

static void test_v3_session_and_sequence_gate(void)
{
    const uint32_t sid = 0x1234ABCDu;
    IIR_Proto_RxContext ctx;
    IIR_Proto_RxInit(&ctx, sid, true);

    IIR_ProtoRxEvent ev;
    uint8_t speed[IIR_PROTO_CMD_FRAME_LEN];
    build_speed_frame(speed, (uint8_t)(IIR_PROTO_CAP_HOST_ACK | 0u), 300, 0, 0);

    /* Must reject speed frames prior to session handshake.
     * 完成会话握手之前必须拒绝所有速度控制帧。 */
    assert(feed_frame(&ctx, speed, sizeof(speed), &ev) == IIR_PROTO_RX_ERR_NO_SESSION);

    /* Session ack with wrong sub-byte or wrong ID must fail.
     * 子字节或会话 ID 不匹配的确认帧必须被拒绝。 */
    uint8_t bad_sess[IIR_PROTO_CMD_FRAME_LEN];
    assert(IIR_Proto_EncodeSessionFrame(sid, bad_sess));
    bad_sess[2] = 0x81u; /* Capability bit forbidden in session sub-byte. */
    bad_sess[9] = IIR_Proto_CheckBcc(bad_sess, 9u);
    assert(feed_frame(&ctx, bad_sess, sizeof(bad_sess), &ev) == IIR_PROTO_RX_ERR_BAD_SESSION);

    /* Valid session ack unlocks speed command reception.
     * 合法会话确认帧解锁速度控制指令接收通道。 */
    uint8_t good_sess[IIR_PROTO_CMD_FRAME_LEN];
    assert(IIR_Proto_EncodeSessionFrame(sid, good_sess));
    assert(feed_frame(&ctx, good_sess, sizeof(good_sess), &ev) == IIR_PROTO_RX_SESSION_ACK);
    assert(ev.session_id == sid);

    /* Missing CAP_HOST_ACK bit must be rejected in v3 mode.
     * v3 模式下缺少 CAP_HOST_ACK 标志位必须被拒绝。 */
    build_speed_frame(speed, 0x05u, 300, 0, 0);
    assert(feed_frame(&ctx, speed, sizeof(speed), &ev) == IIR_PROTO_RX_ERR_NO_SEQ);

    /* First valid speed frame can start at seq 125, then wrap cleanly 126 -> 127 -> 0 -> 1.
     * 会话首帧可从任意序号（如 125）起步，并正确支持 7 位回绕 126->127->0->1。 */
    build_speed_frame(speed, (uint8_t)(IIR_PROTO_CAP_HOST_ACK | 125u), 500, 0, -250);
    assert(feed_frame(&ctx, speed, sizeof(speed), &ev) == IIR_PROTO_RX_SPEED_CMD);
    assert(fabsf(ev.vx_mps - 0.5f) < 1e-4f);
    assert(fabsf(ev.vz_rad_s - (-0.25f)) < 1e-4f);

    /* Replay of same seq 125 must fail.
     * 重复序号 125 重放必须被拒绝。 */
    assert(feed_frame(&ctx, speed, sizeof(speed), &ev) == IIR_PROTO_RX_ERR_SEQ_REPLAY);

    /* Wrap across 127 -> 1 (diff = 4 <= 64) succeeds.
     * 跨回绕边界推进至序号 1（差值 4 <= 64）应接收成功。 */
    build_speed_frame(speed, (uint8_t)(IIR_PROTO_CAP_HOST_ACK | 1u), -300, 0, 100);
    assert(feed_frame(&ctx, speed, sizeof(speed), &ev) == IIR_PROTO_RX_SPEED_CMD);
    assert(fabsf(ev.vx_mps - (-0.3f)) < 1e-4f);

    /* Backward/out-of-window seq (0 has diff=127 > 64 from last_seq=1) is rejected.
     * 回退或越出 64 前向窗口的序号（从 1 到 0 差值为 127 > 64）必须被拒绝。 */
    build_speed_frame(speed, (uint8_t)(IIR_PROTO_CAP_HOST_ACK | 0u), 0, 0, 0);
    assert(feed_frame(&ctx, speed, sizeof(speed), &ev) == IIR_PROTO_RX_ERR_SEQ_WINDOW);
}

static void test_stop_frame_requires_session_and_sequence(void)
{
    IIR_Proto_RxContext ctx;
    IIR_ProtoRxEvent ev;
    uint8_t frame[IIR_PROTO_CMD_FRAME_LEN];
    IIR_Proto_RxInit(&ctx, 123u, true);
    build_speed_frame(frame, IIR_PROTO_CAP_HOST_ACK, 0, 0, 0);
    frame[1] = 0x02u;
    frame[9] = IIR_Proto_CheckBcc(frame, 9u);
    assert(feed_frame(&ctx, frame, sizeof(frame), &ev) == IIR_PROTO_RX_ERR_NO_SESSION);
    assert(IIR_Proto_EncodeSessionFrame(123u, frame));
    assert(feed_frame(&ctx, frame, sizeof(frame), &ev) == IIR_PROTO_RX_SESSION_ACK);
    build_speed_frame(frame, IIR_PROTO_CAP_HOST_ACK, 0, 0, 0);
    frame[1] = 0x02u;
    frame[9] = IIR_Proto_CheckBcc(frame, 9u);
    assert(feed_frame(&ctx, frame, sizeof(frame), &ev) == IIR_PROTO_RX_STOP_CMD);
    assert(feed_frame(&ctx, frame, sizeof(frame), &ev) == IIR_PROTO_RX_ERR_SEQ_REPLAY);
}

static void test_dock_permission_decodes_generation(void)
{
    IIR_Proto_RxContext ctx;
    IIR_ProtoRxEvent ev;
    uint8_t frame[IIR_PROTO_CMD_FRAME_LEN];
    IIR_Proto_RxInit(&ctx, 123u, true);
    assert(IIR_Proto_EncodeSessionFrame(123u, frame));
    assert(feed_frame(&ctx, frame, sizeof(frame), &ev) == IIR_PROTO_RX_SESSION_ACK);
    build_speed_frame(frame, IIR_PROTO_CAP_HOST_ACK, 0, 0, 0);
    frame[1] = 0x03u;
    frame[3] = 0x12u;
    frame[4] = 0x34u;
    frame[5] = 0x56u;
    frame[6] = 0x78u;
    frame[9] = IIR_Proto_CheckBcc(frame, 9u);
    assert(feed_frame(&ctx, frame, sizeof(frame), &ev) == IIR_PROTO_RX_DOCK_PERMISSION);
    assert(ev.dock_generation == 0x12345678u);
}

static void test_dock_target_frame_contains_one_snapshot(void)
{
    const IIR_DockTargetFrame target = {
        .generation = 0x12345678u,
        .vx_mm_s = -100,
        .wz_mrad_s = 200,
        .active = true,
    };
    uint8_t frame[IIR_PROTO_DOCK_TARGET_FRAME_LEN];
    assert(IIR_Proto_EncodeDockTargetFrame(&target, frame));
    assert(frame[0] == IIR_PROTO_EXT_HEADER && frame[1] == IIR_PROTO_CMD_DOCK_TARGET);
    assert(frame[3] == 1u && frame[4] == 0x12u && frame[7] == 0x78u);
    assert(frame[8] == 0xffu && frame[9] == 0x9cu);
    assert(frame[10] == 0u && frame[11] == 200u);
    assert(frame[12] == IIR_Proto_CheckBcc(frame, 12u));
    assert(frame[13] == IIR_PROTO_EXT_TAIL);
}

static void test_stream_resync_and_baseline(void)
{
    IIR_Proto_RxContext ctx;
    IIR_Proto_RxInit(&ctx, 0u, false);

    uint8_t valid[IIR_PROTO_CMD_FRAME_LEN];
    build_speed_frame(valid, 0x00u, 250, 0, 0);

    /* Prepend noise plus a fake 0x7B header before the real frame to test sliding resync.
     * 在真实帧前插入噪声字节及伪造 0x7B 帧头，验证流解析器滑动重同步能力。 */
    uint8_t stream[20];
    stream[0] = 0x11u;
    stream[1] = 0x22u;
    stream[2] = 0x7Bu; /* False header 9 bytes before real frame. */
    stream[3] = 0xAAu;
    stream[4] = 0xBBu;
    stream[5] = 0xCCu;
    memcpy(&stream[6], valid, sizeof(valid));

    int speed_ok = 0;
    IIR_ProtoRxEvent ev;
    for (size_t i = 0u; i < 6u + sizeof(valid); ++i) {
        if (IIR_Proto_RxFeedByte(&ctx, stream[i], &ev) == IIR_PROTO_RX_SPEED_CMD) {
            speed_ok++;
        }
    }
    assert(speed_ok == 1);
    assert(fabsf(ev.vx_mps - 0.25f) < 1e-4f);
    assert(ctx.frames_rejected >= 1u);
}

static void test_command_watchdog(void)
{
    IIR_Proto_Watchdog wd;
    IIR_Proto_WatchdogInit(&wd, 0u);
    assert(wd.timeout_ms == IIR_PROTO_DEFAULT_WATCHDOG_MS);

    /* Fail-Closed before any command is received.
     * 收到首帧有效指令前，看门狗必须保持超时停机态（故障安全）。 */
    assert(IIR_Proto_WatchdogIsTimedOut(&wd, 0u));

    IIR_Proto_WatchdogKick(&wd, 1000u);
    assert(!IIR_Proto_WatchdogIsTimedOut(&wd, 1500u));
    assert(IIR_Proto_WatchdogIsTimedOut(&wd, 1501u));

    /* Wrap-around test across 32-bit tick overflow.
     * 验证 32 位毫秒计数器溢出回绕时的看门狗正确性。 */
    IIR_Proto_WatchdogKick(&wd, 0xFFFFFF00u);
    assert(!IIR_Proto_WatchdogIsTimedOut(&wd, 0x00000050u)); /* +336 ms */
    assert(IIR_Proto_WatchdogIsTimedOut(&wd, 0x00000150u));  /* +592 ms */
}

int main(void)
{
    test_status_frame_encoding();
    test_v3_session_and_sequence_gate();
    test_stop_frame_requires_session_and_sequence();
    test_dock_permission_decodes_generation();
    test_dock_target_frame_contains_one_snapshot();
    test_stream_resync_and_baseline();
    test_command_watchdog();
    printf("test_protocol_v3: ALL PASSED\n");
    return 0;
}

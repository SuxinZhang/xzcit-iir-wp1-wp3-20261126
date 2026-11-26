/**
 * @file test_safety_and_wp2.c
 * @brief Host unit tests for WP2 safety manager, HC-SR04 median/pulse conversion,
 *        ICM-20948 axis rotation, D50A RS485 frame codec, CAN1 auto-docking solver,
 *        and protocol_v3 extended commands/telemetry.
 *        WP2 安全状态机、HC-SR04 中值滤波/脉宽换算、ICM-20948 坐标映射、D50A RS485 编解码、
 *        CAN1 自动回充逻辑及 protocol_v3 扩展帧的宿主机单元测试。
 */

#include "can_auto_dock/bsp_can_dock.h"
#include "d50a_rs485/bsp_d50a.h"
#include "hc_sr04/bsp_hc_sr04.h"
#include "icm20948/bsp_icm20948.h"
#include "protocol_v3.h"
#include "safety_manager.h"

#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void test_safety_state_machine(void)
{
    IIR_SafetyState st;
    IIR_Safety_Init(&st, 0u);
    assert(st.undervolt_mv == IIR_SAFETY_DEFAULT_UNDERVOLT_MV);

    IIR_SafetyInputs in = {
        .hw_enable_pin_high = true,
        .physical_sos_pressed = false,
        .uart_timed_out = false,
        .battery_mv = 24000u,
        .d50a_alarm_mask = 0u,
        .docking_active = false,
        .now_ms = 0u,
    };

    /* Healthy nominal operation: motor allowed, solid green indicator.
     * 全部安全条件正常时允许电机输出，指示灯为绿色常亮。 */
    IIR_SafetyDecision dec = IIR_Safety_Evaluate(&st, &in);
    assert(dec.allow_motor_output);
    assert(dec.stop_reason_mask == (uint8_t)IIR_SAFETY_STOP_NONE);
    assert(!dec.sos_active);
    assert(dec.rgb_mode == IIR_RGB_MODE_GREEN_SOLID);
    assert(dec.rgb_green && !dec.rgb_red && !dec.rgb_blue);
    assert(!dec.buzzer_active);

    in.host_stopped = true;
    dec = IIR_Safety_Evaluate(&st, &in);
    assert(!dec.allow_motor_output);
    assert((dec.stop_reason_mask & IIR_SAFETY_STOP_HOST) != 0u);
    in.host_stopped = false;

    /* Remote SET_LED_BEEP override works only when healthy and not overridden by SOS.
     * 在安全无故障状态下响应远程声光覆盖指令，设置蓝色灯与蜂鸣器。 */
    IIR_Safety_SetLedBeepOverride(&st, 4u, true);
    dec = IIR_Safety_Evaluate(&st, &in);
    assert(dec.allow_motor_output);
    assert(dec.rgb_mode == IIR_RGB_MODE_BLUE_DOCK);
    assert(dec.rgb_blue && dec.buzzer_active);

    /* Physical PE5 SOS press and latch immediately cut PWM and override to SOS alarm.
     * 物理 PE5 SOS 按下及锁存后立即切断电机并强制切入最高优先级声光爆闪报警。 */
    IIR_Safety_LatchPhysicalSos(&st);
    in.physical_sos_pressed = true;
    in.now_ms = 0u;
    dec = IIR_Safety_Evaluate(&st, &in);
    assert(!dec.allow_motor_output);
    assert(dec.sos_active);
    assert((dec.stop_reason_mask & IIR_SAFETY_STOP_SOS) != 0u);
    assert((dec.safety_flags_byte & (1u << 0)) != 0u);
    assert(dec.rgb_mode == IIR_RGB_MODE_SOS_ALARM);
    assert(dec.rgb_red && !dec.rgb_green && !dec.rgb_blue);
    assert(dec.buzzer_active);

    /* Verify SOS strobe toggle cadence at half-period 150 ms.
     * 验证 150 ms 半周期红灯爆闪与蜂鸣器鸣响节拍切换。 */
    in.now_ms = 150u;
    dec = IIR_Safety_Evaluate(&st, &in);
    assert(!dec.rgb_red && !dec.buzzer_active);

    /* Cannot clear physical SOS latch while PE5 button remains pressed.
     * 当 PE5 按钮仍处于按压状态时拒绝清除 SOS 锁存。 */
    assert(!IIR_Safety_ClearPhysicalSos(&st, true));
    IIR_Safety_SetRemoteSos(&st, false, true);
    assert(st.physical_sos_latched);

    /* Releasing PE5 button + remote SET_SOS(0) restores normal healthy gate.
     * 松开 PE5 按钮并下发 SET_SOS(0) 后方可合法解除 SOS 并恢复正常使能。 */
    in.physical_sos_pressed = false;
    IIR_Safety_SetRemoteSos(&st, false, false);
    IIR_Safety_SetLedBeepOverride(&st, 0u, false);
    dec = IIR_Safety_Evaluate(&st, &in);
    assert(dec.allow_motor_output);
    assert(!dec.sos_active);

    /* Remote SET_SOS(1) locks out motor independently of PE5.
     * 远程 SET_SOS(1) 同样在 STM32 本地立即切断电机输出。 */
    IIR_Safety_SetRemoteSos(&st, true, false);
    dec = IIR_Safety_Evaluate(&st, &in);
    assert(!dec.allow_motor_output && dec.sos_active);
    IIR_Safety_SetRemoteSos(&st, false, false);

    /* D50A offline bit (0x80) alone warns telemetry without locking out differential motors;
     * real driver faults (bits 0..3) cut PWM immediately.
     * D50A 仅离线（0x80）时允许无 RS485 台架调试，一旦发生过流/过温等真实故障立即锁停。 */
    in.d50a_alarm_mask = (uint8_t)BSP_D50A_FAULT_OFFLINE;
    dec = IIR_Safety_Evaluate(&st, &in);
    assert(dec.allow_motor_output);

    in.d50a_alarm_mask = (uint8_t)BSP_D50A_FAULT_OVERCURRENT;
    dec = IIR_Safety_Evaluate(&st, &in);
    assert(!dec.allow_motor_output);
    assert((dec.stop_reason_mask & IIR_SAFETY_STOP_D50A_FAULT) != 0u);
    assert(dec.rgb_mode == IIR_RGB_MODE_WARN_YELLOW);
    in.d50a_alarm_mask = 0u;

    /* PE4 E-stop, UART watchdog timeout, and battery undervoltage Fail-Closed checks.
     * PE4 急停、USART3 看门狗超时与电池严重欠压故障安全停机校验。 */
    in.hw_enable_pin_high = false;
    in.uart_timed_out = true;
    in.battery_mv = 19800u;
    dec = IIR_Safety_Evaluate(&st, &in);
    assert(!dec.allow_motor_output);
    assert((dec.stop_reason_mask & IIR_SAFETY_STOP_PE4_ESTOP) != 0u);
    assert((dec.stop_reason_mask & IIR_SAFETY_STOP_UART_TIMEOUT) != 0u);
    assert((dec.stop_reason_mask & IIR_SAFETY_STOP_UNDERVOLT) != 0u);
}

static void test_hcsr04_math_and_median(void)
{
    assert(BSP_HcSr04_PulseUsToMm(0u) == BSP_HCSR04_INVALID_MM);
    assert(BSP_HcSr04_PulseUsToMm(116u) == BSP_HCSR04_INVALID_MM);
    assert(BSP_HcSr04_PulseUsToMm(117u) == 20u);
    assert(BSP_HcSr04_PulseUsToMm(5831u) == 1000u);
    assert(BSP_HcSr04_PulseUsToMm(26240u) == 4500u);
    assert(BSP_HcSr04_PulseUsToMm(26241u) == BSP_HCSR04_INVALID_MM);
    assert(BSP_HcSr04_PulseUsToMm(28000u) == BSP_HCSR04_INVALID_MM);

    const uint16_t win_valid[5] = {1200u, 300u, 1210u, 4500u, 1190u};
    assert(BSP_HcSr04_MedianFilter5(win_valid) == 1200u);

    const uint16_t win_mixed[5] = {BSP_HCSR04_INVALID_MM, 800u, 820u, 810u, BSP_HCSR04_INVALID_MM};
    assert(BSP_HcSr04_MedianFilter5(win_mixed) == 810u);

    const uint16_t win_too_few[5] = {BSP_HCSR04_INVALID_MM, 500u, BSP_HCSR04_INVALID_MM, 510u, 0xFFFFu};
    assert(BSP_HcSr04_MedianFilter5(win_too_few) == BSP_HCSR04_INVALID_MM);
}

static void test_icm20948_axis_mapping(void)
{
    const int16_t chip_a[3] = {100, -250, 16384};
    int16_t ros_a[3] = {0};
    BSP_Icm20948_RotateChipToRos(chip_a, ros_a);
    assert(ros_a[0] == -250);
    assert(ros_a[1] == -100);
    assert(ros_a[2] == 16384);

    /* Verify INT16_MIN inversion safety guard.
     * 验证 INT16_MIN（-32768）取反映射时的防溢出保护。 */
    const int16_t chip_min[3] = {INT16_MIN, 42, -99};
    int16_t ros_min[3] = {0};
    BSP_Icm20948_RotateChipToRos(chip_min, ros_min);
    assert(ros_min[0] == 42);
    assert(ros_min[1] == INT16_MAX);
    assert(ros_min[2] == -99);
}

static void test_d50a_rs485_protocol(void)
{
    uint8_t req[BSP_D50A_FRAME_LEN];
    BSP_D50A_BuildFrame(req, 0x01u, BSP_D50A_FUNC_READ_STATUS, 0u, 0u, 0u, 0u);
    assert(req[0] == 0xAAu && req[1] == 0xAAu && req[2] == 0xAAu);
    assert(req[3] == 0x01u && req[4] == BSP_D50A_FUNC_READ_STATUS);
    assert(req[9] == 0xBBu && req[12] == 0xBBu);

    /* Construct 0xA0 status reply: alarm=0x0E (bit0 overcurrent active-LOW), M1=125 (1.25A), M2=-50 (-0.50A).
     * 构造 0xA0 应答帧：alarm=0x0E（过流保护触发），M1=125（1.25 A），M2=-50（-0.50 A）。 */
    const int16_t m2_raw = (int16_t)-50;
    uint8_t reply[BSP_D50A_FRAME_LEN];
    BSP_D50A_BuildFrame(reply,
                        0x01u,
                        0x0Eu,
                        0x00u,
                        125u,
                        (uint8_t)(((uint16_t)m2_raw >> 8u) & 0xFFu),
                        (uint8_t)((uint16_t)m2_raw & 0xFFu));

    BSP_D50A_RxParser parser;
    memset(&parser, 0, sizeof(parser));
    uint8_t parsed_frame[BSP_D50A_FRAME_LEN];
    assert(!BSP_D50A_FeedRxByte(&parser, 0x55u, parsed_frame));
    assert(!BSP_D50A_FeedRxByte(&parser, 0xAAu, parsed_frame));
    assert(!BSP_D50A_FeedRxByte(&parser, 0x11u, parsed_frame)); /* False partial sync resets */
    bool got = false;
    for (size_t i = 0u; i < sizeof(reply); ++i) {
        got = BSP_D50A_FeedRxByte(&parser, reply[i], parsed_frame);
    }
    assert(got);

    BSP_D50A_Telemetry tel;
    memset(&tel, 0, sizeof(tel));
    assert(BSP_D50A_ParseResponse(parsed_frame, 0x01u, true, &tel));
    assert(tel.online);
    assert(tel.fault_mask == (uint8_t)BSP_D50A_FAULT_OVERCURRENT);
    assert(tel.m1_current_ma == 1250);
    assert(tel.m2_current_ma == -500);

    /* Parse 0xA3 temperature response: M1=42 C, M2=45 C.
     * 解析 0xA3 温度查询应答帧：M1=42 摄氏度，M2=45 摄氏度。 */
    uint8_t temp_reply[BSP_D50A_FRAME_LEN];
    BSP_D50A_BuildFrame(temp_reply, 0x01u, BSP_D50A_FUNC_READ_TEMP, 0u, 42u, 0u, 45u);
    assert(BSP_D50A_ParseResponse(temp_reply, 0x01u, false, &tel));
    assert(tel.m1_temp_c == 42u && tel.m2_temp_c == 45u);
}

static void test_can_auto_dock_logic(void)
{
    BSP_CanDock_State st;
    memset(&st, 0, sizeof(st));

    /* Enable auto-dock with no IR signal yet -> host pre-dock nav twist passes through.
     * 开启自动回充且未收到红外信号时，透传上位机 Nav2 预接近阶段速度指令。 */
    BSP_CanDock_SetEnabled(&st, true, -0.10f, 0.20f);
    float out_vx = 0.0f;
    float out_wz = 0.0f;
    BSP_CanDock_ComputeTwist(&st, 0.25f, -0.15f, true, &out_vx, &out_wz);
    assert(fabsf(out_vx - 0.25f) < 1e-5f && fabsf(out_wz - (-0.15f)) < 1e-5f);

    /* Receive CAN 0x182 frame with L_A + R_A active (bits 5 & 2 -> 0x24), not charging yet,
     * vx=-100 mm/s, vz=200 mrad/s, current=0 mA -> switches immediately to IR guidance.
     * 收到 CAN 0x182 红外有效报文后自动切换为 STM32 红外闭环对接速度。 */
    const uint16_t raw_vx = (uint16_t)((int16_t)-100);
    const uint16_t raw_vz = (uint16_t)((int16_t)200);
    const uint8_t can_ir[8] = {
        (uint8_t)(raw_vx >> 8u), (uint8_t)(raw_vx & 0xFFu),
        0u, 0u,
        (uint8_t)(raw_vz >> 8u), (uint8_t)(raw_vz & 0xFFu),
        0x24u, /* R_A (bit2) + L_A (bit5) */
        10u    /* 300 mA */
    };
    BSP_CanDock_ParseFrame182(can_ir, &st);
    assert(!st.charging);
    assert(st.red_state == 2u);
    assert((st.ir_mask & 0x0Fu) == 0x09u); /* bit0=R_A, bit3=L_A */
    assert(st.charging_current_ma == 300);

    BSP_CanDock_ComputeTwist(&st, 0.25f, -0.15f, true, &out_vx, &out_wz);
    assert(fabsf(out_vx - (-0.10f)) < 1e-5f && fabsf(out_wz - 0.20f) < 1e-5f);

    /* Receive charging bit set (status bit0 = 1), 1500 mA -> stops immediately.
     * 检测到充电触片接通（bit0 = 1）与充电电流后立即输出零速停稳。 */
    const uint8_t can_chg[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0x25u, 50u};
    BSP_CanDock_ParseFrame182(can_chg, &st);
    assert(st.charging);
    assert(st.charging_current_ma == 1500);
    BSP_CanDock_ComputeTwist(&st, 0.25f, -0.15f, true, &out_vx, &out_wz);
    assert(fabsf(out_vx) < 1e-6f && fabsf(out_wz) < 1e-6f);
}

static void test_dock_motion_requires_permission(void)
{
    BSP_CanDock_State st = {0};
    st.allow_recharge = true;
    st.red_state = 1u;
    st.red_vx_mps = -0.1f;
    st.red_vz_rad_s = 0.2f;
    float vx = 1.0f;
    float wz = 1.0f;
    BSP_CanDock_ComputeTwist(&st, 0.0f, 0.0f, false, &vx, &wz);
    assert(vx == 0.0f && wz == 0.0f);
    BSP_CanDock_ComputeTwist(&st, 0.0f, 0.0f, true, &vx, &wz);
    assert(fabsf(vx + 0.1f) < 1e-5f && fabsf(wz - 0.2f) < 1e-5f);
}

static void test_ir_permission_tracks_target_and_expiry(void)
{
    BSP_CanDock_State st = {0};
    st.allow_recharge = true;
    const uint8_t target[8] = {0xffu, 0x9cu, 0u, 0u, 0u, 0xc8u, 0x04u, 0u};
    BSP_CanDock_ParseFrame182(target, &st);
    const uint32_t first = st.target_generation;
    BSP_CanDock_PermitTarget(&st, first, 100u);
    assert(BSP_CanDock_TargetPermitted(&st, 100u));
    BSP_CanDock_ParseFrame182(target, &st);
    assert(st.target_generation == first);
    assert(BSP_CanDock_TargetPermitted(&st, 300u));
    assert(!BSP_CanDock_TargetPermitted(&st, 401u));
    uint8_t changed[8];
    memcpy(changed, target, sizeof(changed));
    changed[5] = 0x9cu;
    BSP_CanDock_ParseFrame182(changed, &st);
    assert(st.target_generation != first);
    BSP_CanDock_PermitTarget(&st, first, 500u);
    assert(!BSP_CanDock_TargetPermitted(&st, 500u));
}

static void test_ir_target_is_invalidated_when_can_status_expires(void)
{
    BSP_CanDock_State st = {0};
    const uint8_t target[8] = {0xffu, 0x9cu, 0u, 0u, 0u, 0xc8u, 0x04u, 0u};
    BSP_CanDock_SetEnabled(&st, true, -0.1f, 0.2f);
    BSP_CanDock_ParseFrame182(target, &st);
    BSP_CanDock_PermitTarget(&st, st.target_generation, 10u);
    BSP_CanDock_InvalidateTarget(&st);
    assert(st.red_state == 0u && !st.allow_recharge && !st.permission_valid);
    assert(st.red_vx_mps == 0.0f && st.red_vz_rad_s == 0.0f);
}

static void test_revoked_ir_target_remains_a_forced_stop(void)
{
    BSP_CanDock_State st = {0};
    const uint8_t first[8] = {0xffu, 0x9cu, 0u, 0u, 0u, 0xc8u, 0x04u, 0u};
    uint8_t changed[8];
    memcpy(changed, first, sizeof(changed));
    changed[5] = 0x9cu;
    BSP_CanDock_SetEnabled(&st, true, -0.1f, 0.2f);
    BSP_CanDock_ParseFrame182(first, &st);
    BSP_CanDock_PermitTarget(&st, st.target_generation, 10u);
    BSP_CanDock_ParseFrame182(changed, &st);
    float vx = 1.0f;
    float wz = 1.0f;
    BSP_CanDock_ComputeTwist(&st, 0.2f, 0.1f, false, &vx, &wz);
    assert(vx == 0.0f && wz == 0.0f);
    assert(st.allow_recharge);
    BSP_CanDock_PermitTarget(&st, st.target_generation, 20u);
    BSP_CanDock_ComputeTwist(&st, 0.2f, 0.1f,
                              BSP_CanDock_TargetPermitted(&st, 20u), &vx, &wz);
    assert(fabsf(vx + 0.1f) < 1e-5f && fabsf(wz - 0.156f) < 1e-5f);
}

static void test_expired_ir_permission_keeps_host_motion_stopped(void)
{
    BSP_CanDock_State st = {0};
    const uint8_t target[8] = {0xffu, 0x9cu, 0u, 0u, 0u, 0xc8u, 0x04u, 0u};
    BSP_CanDock_SetEnabled(&st, true, -0.1f, 0.2f);
    BSP_CanDock_ParseFrame182(target, &st);
    BSP_CanDock_PermitTarget(&st, st.target_generation, 10u);
    assert(BSP_CanDock_ExpirePermission(&st, 311u));

    float vx = 1.0f;
    float wz = 1.0f;
    BSP_CanDock_ComputeTwist(&st, 0.2f, 0.1f, false, &vx, &wz);
    assert(vx == 0.0f && wz == 0.0f);
    assert(!BSP_CanDock_ExpirePermission(&st, 321u));
    BSP_CanDock_ComputeTwist(&st, 0.2f, 0.1f, false, &vx, &wz);
    assert(vx == 0.0f && wz == 0.0f);
}

static void test_changed_ir_target_invalidates_pending_motor_output(void)
{
    BSP_CanDock_State live = {0};
    const uint8_t target[8] = {0xffu, 0x9cu, 0u, 0u, 0u, 0xc8u, 0x04u, 0u};
    uint8_t changed[8];
    memcpy(changed, target, sizeof(changed));
    changed[5] = 0x9cu;
    BSP_CanDock_SetEnabled(&live, true, -0.1f, 0.2f);
    BSP_CanDock_ParseFrame182(target, &live);
    BSP_CanDock_PermitTarget(&live, live.target_generation, 10u);
    const BSP_CanDock_State snapshot = live;
    assert(BSP_CanDock_OutputCurrent(&snapshot, &live, 10u));
    assert(!BSP_CanDock_OutputCurrent(&snapshot, &live, 311u));
    BSP_CanDock_ParseFrame182(changed, &live);
    assert(!BSP_CanDock_OutputCurrent(&snapshot, &live, 10u));
}

static void test_reenabling_dock_changes_target_generation(void)
{
    BSP_CanDock_State st = {0};
    const uint8_t target[8] = {0xffu, 0x9cu, 0u, 0u, 0u, 0xc8u, 0x04u, 0u};
    BSP_CanDock_ParseFrame182(target, &st);
    BSP_CanDock_SetEnabled(&st, true, -0.1f, 0.2f);
    const uint32_t first = st.target_generation;
    BSP_CanDock_PermitTarget(&st, first, 10u);
    BSP_CanDock_SetEnabled(&st, false, 0.0f, 0.0f);
    BSP_CanDock_SetEnabled(&st, true, -0.1f, 0.2f);
    assert(st.target_generation != first);
    BSP_CanDock_PermitTarget(&st, first, 20u);
    assert(!BSP_CanDock_TargetPermitted(&st, 20u));
}

static void test_protocol_v3_extensions(void)
{
    const IIR_ExtStatusFrame ext_in = {
        .charging_state = 0x07u,
        .charging_current_ma = -1200,
        .ir_mask = 0x29u,
        .safety_flags = 0x0Au,
        .d50a_alarm_mask = 0x81u,
    };
    uint8_t raw[IIR_PROTO_CMD_FRAME_LEN] = {0};
    assert(IIR_Proto_EncodeExtStatusFrame(&ext_in, raw));
    assert(raw[0] == IIR_PROTO_EXT_HEADER && raw[1] == IIR_PROTO_CMD_EXT_STATUS);
    assert(raw[2] == IIR_PROTO_VERSION && raw[10] == IIR_PROTO_EXT_TAIL);

    IIR_ExtStatusFrame ext_out;
    assert(IIR_Proto_DecodeExtStatusFrame(raw, &ext_out));
    assert(ext_out.charging_state == ext_in.charging_state);
    assert(ext_out.charging_current_ma == ext_in.charging_current_ma);
    assert(ext_out.ir_mask == ext_in.ir_mask);
    assert(ext_out.safety_flags == ext_in.safety_flags);
    assert(ext_out.d50a_alarm_mask == ext_in.d50a_alarm_mask);

    /* Corrupt BCC byte must fail decode.
     * 破坏 BCC 校验位后必须拒绝解码扩展状态帧。 */
    raw[9] ^= 0xFFu;
    assert(!IIR_Proto_DecodeExtStatusFrame(raw, &ext_out));

    const uint32_t sid = 0x99887766u;
    IIR_Proto_RxContext ctx;
    IIR_Proto_RxInit(&ctx, sid, true);

    uint8_t frame[IIR_PROTO_CMD_FRAME_LEN];
    IIR_ProtoRxEvent ev;

    /* Reject extended service command before session handshake.
     * 完成会话握手之前必须拒绝扩展控制指令。 */
    assert(IIR_Proto_EncodeAutoDockCmd(true, -120, 250, 0u, false, frame));
    for (size_t i = 0u; i + 1u < sizeof(frame); ++i) {
        assert(IIR_Proto_RxFeedByte(&ctx, frame[i], &ev) == IIR_PROTO_RX_INCOMPLETE);
    }
    assert(IIR_Proto_RxFeedByte(&ctx, frame[10], &ev) == IIR_PROTO_RX_ERR_NO_SESSION);

    /* Establish v3 session.
     * 建立 v3 会话。 */
    assert(IIR_Proto_EncodeSessionFrame(sid, frame));
    for (size_t i = 0u; i + 1u < sizeof(frame); ++i) {
        (void)IIR_Proto_RxFeedByte(&ctx, frame[i], &ev);
    }
    assert(IIR_Proto_RxFeedByte(&ctx, frame[10], &ev) == IIR_PROTO_RX_SESSION_ACK);

    /* Unsequenced SET_AUTO_DOCK (sub=0x01) accepted after session.
     * 会话确认后允许通过固定子字节 0x01 发送无序号自动回充指令。 */
    assert(IIR_Proto_EncodeAutoDockCmd(true, -120, 250, 0u, false, frame));
    for (size_t i = 0u; i + 1u < sizeof(frame); ++i) {
        (void)IIR_Proto_RxFeedByte(&ctx, frame[i], &ev);
    }
    assert(IIR_Proto_RxFeedByte(&ctx, frame[10], &ev) == IIR_PROTO_RX_AUTO_DOCK_CMD);
    assert(ev.auto_dock_enable);
    assert(fabsf(ev.dock_vx_mps - (-0.12f)) < 1e-4f);
    assert(fabsf(ev.dock_vz_rad_s - 0.25f) < 1e-4f);

    /* Sequenced SET_SOS(true) with seq=5 accepted and advances seq anchor.
     * 带序号 seq=5 的远程 SET_SOS(1) 被接收并更新序列锚点。 */
    assert(IIR_Proto_EncodeSetSosCmd(true, 5u, true, frame));
    for (size_t i = 0u; i + 1u < sizeof(frame); ++i) {
        (void)IIR_Proto_RxFeedByte(&ctx, frame[i], &ev);
    }
    assert(IIR_Proto_RxFeedByte(&ctx, frame[10], &ev) == IIR_PROTO_RX_SOS_CMD);
    assert(ev.sos_active && ev.seq == 5u);

    /* Sequenced SET_LED_BEEP with seq=6 accepted cleanly.
     * 带序号 seq=6 的 SET_LED_BEEP 指令被正确解析接收。 */
    assert(IIR_Proto_EncodeSetLedBeepCmd(2u, true, 6u, true, frame));
    for (size_t i = 0u; i + 1u < sizeof(frame); ++i) {
        (void)IIR_Proto_RxFeedByte(&ctx, frame[i], &ev);
    }
    assert(IIR_Proto_RxFeedByte(&ctx, frame[10], &ev) == IIR_PROTO_RX_LED_BEEP_CMD);
    assert(ev.led_beep_mode == 2u && ev.beep_enable && ev.seq == 6u);

    /* Echo signature filter drops matching loopback probes.
     * 登记本地遥测签名后，回显探测器帧应触发 IIR_PROTO_RX_ERR_ECHO 拦截。 */
    const uint8_t echo_sig[6] = {0x43u, 0x44u, 0x45u, 0x46u, 0x47u, 0x48u};
    IIR_Proto_RxSetEchoSignature(&ctx, 0x00u, echo_sig);
    frame[0] = IIR_PROTO_FRAME_HEADER;
    frame[1] = 0x00u;
    frame[2] = IIR_PROTO_CAP_HOST_ACK | 7u;
    memcpy(&frame[3], echo_sig, 6u);
    frame[9] = IIR_Proto_CheckBcc(frame, 9u);
    frame[10] = IIR_PROTO_FRAME_TAIL;
    for (size_t i = 0u; i + 1u < sizeof(frame); ++i) {
        (void)IIR_Proto_RxFeedByte(&ctx, frame[i], &ev);
    }
    assert(IIR_Proto_RxFeedByte(&ctx, frame[10], &ev) == IIR_PROTO_RX_ERR_ECHO);
    assert(ctx.frames_echo == 1u);
}

int main(void)
{
    test_safety_state_machine();
    test_hcsr04_math_and_median();
    test_icm20948_axis_mapping();
    test_d50a_rs485_protocol();
    test_can_auto_dock_logic();
    test_dock_motion_requires_permission();
    test_ir_permission_tracks_target_and_expiry();
    test_ir_target_is_invalidated_when_can_status_expires();
    test_revoked_ir_target_remains_a_forced_stop();
    test_expired_ir_permission_keeps_host_motion_stopped();
    test_changed_ir_target_invalidates_pending_motor_output();
    test_reenabling_dock_changes_target_generation();
    test_protocol_v3_extensions();
    printf("test_safety_and_wp2: ALL PASSED\n");
    return 0;
}

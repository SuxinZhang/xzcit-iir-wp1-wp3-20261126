/**
 * @file app_tasks.c
 * @brief FreeRTOS ControlTask (100 Hz), ImuTask (100 Hz), D50ATask (10 Hz), and
 *        TelemetryTask (20 Hz) orchestration for XZCIT-IIR differential chassis.
 *        XZCIT-IIR 差速底盘控制（100 Hz）、IMU 与超声波（100 Hz）、D50A 监控（10 Hz）
 *        及遥测/会话通告（20 Hz）的 FreeRTOS 任务编排实现。
 */

#include "app_tasks.h"

#include "FreeRTOS.h"
#include "task.h"

#include "adc/bsp_adc.h"
#include "bsp_RGBLight/bsp_rgb.h"
#include "buzzer/bsp_buzzer.h"
#include "can_auto_dock/bsp_can_dock.h"
#include "d50a_rs485/bsp_d50a.h"
#include "diff_kinematics.h"
#include "encoder/bsp_encoder.h"
#include "hc_sr04/bsp_hc_sr04.h"
#include "icm20948/bsp_icm20948.h"
#include "motor/bsp_motor.h"
#include "motor_control.h"
#include "oled/bsp_oled.h"
#include "protocol_v3.h"
#include "fw_version.h"
#include "safety_manager.h"
#include "sos_button/bsp_sos.h"
#include "uartx/bsp_uartx.h"

#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Task periods and safety thresholds aligned to AI_EXECUTION_PLAN Invariants 2 and 3.
 * 任务调度周期与本地安全阈值，严格对齐执行规划书红线第 2 条与第 3 条。 */
#define IIR_CONTROL_PERIOD_MS       10u
#define IIR_IMU_PERIOD_MS           10u
#define IIR_D50A_PERIOD_MS          100u
#define IIR_TELEMETRY_PERIOD_MS     50u
#define IIR_SESSION_BEACON_TICKS    20u
#define IIR_MOTOR_MAX_PWM           7200
#define IIR_BATTERY_CRIT_MV         IIR_SAFETY_DEFAULT_UNDERVOLT_MV

static IIR_KinematicsConfig s_kin_cfg;
static IIR_PiConfig s_pi_cfg;
static IIR_LowSpeedCompConfig s_low_speed_cfg;
static IIR_PiState s_pi_states[IIR_MOTOR_CHANNEL_COUNT];
static IIR_SmootherState s_vx_smoother;
static IIR_SmootherState s_wz_smoother;

static IIR_Proto_RxContext s_rx_ctx;
static IIR_Proto_Watchdog s_cmd_wd;
static IIR_SafetyState s_safety_state;
static uint32_t s_session_id;

/* Shared telemetry snapshots protected by FreeRTOS critical sections.
 * 受 FreeRTOS 临界区保护的共享主状态遥测与扩展安全回充遥测快照。 */
static IIR_StatusFrame s_telemetry_snapshot;
static IIR_ExtStatusFrame s_ext_snapshot;
static IIR_DockTargetFrame s_dock_target_snapshot;
static volatile bool s_motion_allowed = false;

static int16_t round_clamp_i16(float value)
{
    if (!isfinite(value)) {
        return 0;
    }
    const float r = roundf(value);
    if (r >= 32767.0f) {
        return 32767;
    }
    if (r <= -32768.0f) {
        return -32768;
    }
    return (int16_t)r;
}

/* Pack BSP_CanDock_State booleans into the protocol_v3_ext charging_state bitmask.
 * 将 CAN 自动回充状态标志编码为 protocol_v3_ext 契约定义的 charging_state 位掩码。 */
static uint8_t pack_charging_state(const BSP_CanDock_State *dock)
{
    uint8_t val = 0u;
    if (dock->charging) {
        val |= (1u << 0);
    }
    if (dock->allow_recharge) {
        val |= (1u << 1);
    }
    if (dock->hw_present) {
        val |= (1u << 2);
    }
    if (dock->nav_walk) {
        val |= (1u << 3);
    }
    return val;
}

/* 100 Hz dedicated IMU + ultrasonic periodic trigger task (Invariant 3).
 * 100 Hz 独立 IMU 软件 I2C 采集与 HC-SR04 周期触发任务（工程红线第 3 条）。 */
static void imu_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IIR_IMU_PERIOD_MS));

        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        (void)BSP_Icm20948_Sample();
        BSP_HcSr04_PeriodicTick(now_ms);
    }
}

/* 10 Hz single-task queue-exclusive D50A RS485 monitoring task on USART2.
 * 10 Hz 运行的 USART2 RS485 单任务队列独占 D50A 驱动器健康状态轮询任务。 */
static void d50a_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IIR_D50A_PERIOD_MS));

        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        BSP_D50A_PollTick(now_ms);
    }
}

/* 100 Hz motion control loop: UART RX -> SOS/Safety -> Dock/Host -> PI -> Motor_ApplyOutput.
 * 100 Hz 运动控制闭环：串口收帧 -> 安全仲裁 -> 回充/上位机指令融合 -> PI 闭环 -> 唯一 PWM 出口。 */
static void control_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    IIR_BodyTwist host_twist = {0.0f, 0.0f, 0.0f};
    bool host_stopped = true;
    const float dt_s = (float)IIR_CONTROL_PERIOD_MS / 1000.0f;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IIR_CONTROL_PERIOD_MS));

        const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        /* Drain non-blocking USART3 RX ring buffer and handle speed & WP2 service commands.
         * 排空非阻塞 USART3 接收环形缓冲区，分别处理速度指令与 WP2 扩展控制命令。 */
        uint8_t rx_byte = 0u;
        while (BSP_Uart3_ReadByte(&rx_byte)) {
            IIR_ProtoRxEvent ev;
            const IIR_ProtoRxStatus st = IIR_Proto_RxFeedByte(&s_rx_ctx, rx_byte, &ev);
            if (st == IIR_PROTO_RX_SPEED_CMD) {
                host_stopped = false;
                host_twist.vx_mps = ev.vx_mps;
                host_twist.vy_mps = 0.0f;
                host_twist.wz_rad_s = ev.vz_rad_s;
                IIR_Proto_WatchdogKick(&s_cmd_wd, now_ms);
            } else if (st == IIR_PROTO_RX_STOP_CMD) {
                host_stopped = true;
                host_twist.vx_mps = 0.0f;
                host_twist.vy_mps = 0.0f;
                host_twist.wz_rad_s = 0.0f;
                BSP_CanDock_ApplyHostCommand(false, 0.0f, 0.0f);
                IIR_Proto_WatchdogKick(&s_cmd_wd, now_ms);
            } else if (st == IIR_PROTO_RX_AUTO_DOCK_CMD) {
                BSP_CanDock_ApplyHostCommand(ev.auto_dock_enable,
                                             ev.dock_vx_mps,
                                             ev.dock_vz_rad_s);
            } else if (st == IIR_PROTO_RX_DOCK_PERMISSION && !host_stopped) {
                BSP_CanDock_ApplyTargetPermission(ev.dock_generation, now_ms);
            } else if (st == IIR_PROTO_RX_SOS_CMD) {
                const bool sos_pressed = BSP_Sos_IsPressed();
                IIR_Safety_SetRemoteSos(&s_safety_state, ev.sos_active, sos_pressed);
                if (!ev.sos_active) {
                    (void)BSP_Sos_ClearLatch();
                }
            } else if (st == IIR_PROTO_RX_LED_BEEP_CMD) {
                IIR_Safety_SetLedBeepOverride(&s_safety_state,
                                              ev.led_beep_mode,
                                              ev.beep_enable);
            }
        }

        /* Debounce PE5 physical SOS input and latch into pure-C safety state machine.
         * 对 PE5 物理 SOS 按键进行 20 ms 消抖并锁存至纯 C 安全状态机。 */
        BSP_Sos_Update(now_ms);
        const bool sos_pressed_now = BSP_Sos_IsPressed();
        if (BSP_Sos_IsLatched()) {
            IIR_Safety_LatchPhysicalSos(&s_safety_state);
        }

        /* Advance CAN1 auto-dock monitor and fetch atomic snapshots for all WP2 sensors.
         * 更新 CAN1 自动回充在线监测，并原子读取 D50A、IMU 与超声波最新传感器快照。 */
        BSP_CanDock_PeriodicTick(now_ms);
        if (IIR_Proto_WatchdogIsTimedOut(&s_cmd_wd, now_ms)) {
            host_stopped = true;
            BSP_CanDock_ApplyHostCommand(false, 0.0f, 0.0f);
        }
        BSP_CanDock_ExpireGlobalPermission(now_ms);
        BSP_CanDock_State dock_snap;
        BSP_CanDock_GetState(&dock_snap);
        const bool ir_target_unpermitted = dock_snap.red_state != 0u &&
                                            !BSP_CanDock_TargetPermitted(&dock_snap, now_ms);

        BSP_D50A_Telemetry d50a_snap;
        BSP_D50A_GetTelemetry(&d50a_snap);

        BSP_Icm20948_Data imu_snap;
        BSP_Icm20948_GetLatest(&imu_snap);

        const uint16_t bat_mv = BSP_Adc_GetBatteryMillivolts();
        const uint16_t ultrasonic_mm = BSP_HcSr04_GetDistanceMm();

        const IIR_SafetyInputs safety_in = {
            .hw_enable_pin_high = BSP_Motor_IsHwEnabled(),
            .physical_sos_pressed = sos_pressed_now,
            .uart_timed_out = IIR_Proto_WatchdogIsTimedOut(&s_cmd_wd, now_ms),
            .host_stopped = host_stopped,
            .battery_mv = bat_mv,
            .d50a_alarm_mask = d50a_snap.fault_mask,
            .docking_active = dock_snap.allow_recharge && !dock_snap.charging,
            .now_ms = now_ms,
        };
        const IIR_SafetyDecision safety_dec = IIR_Safety_Evaluate(&s_safety_state, &safety_in);

        /* Apply audio/visual outputs arbitrated strictly by IIR_Safety_Evaluate.
         * 按照安全管理器裁决的最高优先级结果更新 RGB 灯带与 PD11 蜂鸣器输出。 */
        BSP_Rgb_Set(safety_dec.rgb_red, safety_dec.rgb_green, safety_dec.rgb_blue);
        BSP_Buzzer_Set(safety_dec.buzzer_active);

        /* Read 4-wheel encoders and reconstruct actual body twist via forward kinematics.
         * 采集四轮正交编码器脉冲增量并通过正运动学重构车体实测速度。 */
        int32_t deltas[IIR_MOTOR_CHANNEL_COUNT] = {0, 0, 0, 0};
        BSP_Encoder_ReadDeltas(deltas);

        const IIR_WheelSpeeds measured_wheels = {
            .fl_a = IIR_Kinematics_EncoderDeltaToMps(&s_kin_cfg, deltas[0], dt_s),
            .rl_b = IIR_Kinematics_EncoderDeltaToMps(&s_kin_cfg, deltas[1], dt_s),
            .rr_c = IIR_Kinematics_EncoderDeltaToMps(&s_kin_cfg, deltas[2], dt_s),
            .fr_d = IIR_Kinematics_EncoderDeltaToMps(&s_kin_cfg, deltas[3], dt_s),
        };
        IIR_BodyTwist measured_twist = {0.0f, 0.0f, 0.0f};
        (void)IIR_Kinematics_Forward(&s_kin_cfg, measured_wheels, &measured_twist);

        int16_t cmd_pwm[IIR_MOTOR_CHANNEL_COUNT] = {0, 0, 0, 0};
        if (safety_dec.allow_motor_output) {
            float target_vx = 0.0f;
            float target_wz = 0.0f;
            BSP_CanDock_ComputeTwist(&dock_snap,
                                     host_twist.vx_mps,
                                     host_twist.wz_rad_s,
                                     BSP_CanDock_TargetPermitted(&dock_snap, now_ms),
                                     &target_vx,
                                     &target_wz);
            const bool ir_dock_active = ir_target_unpermitted || dock_snap.motion_revoked ||
                                        (dock_snap.allow_recharge && dock_snap.red_state != 0u);
            const IIR_BodyTwist smoothed_cmd = {
                .vx_mps = IIR_Smoother_Step(&s_vx_smoother, target_vx, 0.02f, ir_dock_active),
                .vy_mps = 0.0f,
                .wz_rad_s = IIR_Smoother_Step(&s_wz_smoother, target_wz, 0.04f, ir_dock_active),
            };
            IIR_WheelSpeeds target_wheels = {0};
            (void)IIR_Kinematics_Inverse(&s_kin_cfg, smoothed_cmd, &target_wheels);

            const float targets[IIR_MOTOR_CHANNEL_COUNT] = {
                target_wheels.fl_a, target_wheels.rl_b, target_wheels.rr_c, target_wheels.fr_d
            };
            const float meas[IIR_MOTOR_CHANNEL_COUNT] = {
                measured_wheels.fl_a, measured_wheels.rl_b, measured_wheels.rr_c, measured_wheels.fr_d
            };

            for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
                const float raw_pi = IIR_Pi_Update(&s_pi_cfg, &s_pi_states[i], targets[i], meas[i]);
                const float comp_pwm = IIR_Control_CompensateLowSpeed(&s_low_speed_cfg,
                                                                      targets[i],
                                                                      raw_pi);
                cmd_pwm[i] = round_clamp_i16(comp_pwm);
            }
            /* Keep the CAN target check and hardware PWM write in one critical section.
             * 在同一临界区内检查 CAN 目标并写入硬件 PWM。 */
            taskENTER_CRITICAL();
            const uint32_t output_now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            const bool dock_output_current = BSP_CanDock_GlobalOutputCurrent(&dock_snap, output_now_ms);
            Motor_ApplyOutput(cmd_pwm, dock_output_current);
            taskEXIT_CRITICAL();
            if (!dock_output_current) {
                for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
                    IIR_Pi_Reset(&s_pi_states[i]);
                }
                IIR_Smoother_Reset(&s_vx_smoother);
                IIR_Smoother_Reset(&s_wz_smoother);
            }
        } else {
            /* Clear PI accumulators and slew limiters on safety stop to prevent jump on re-arm.
             * 安全停机时清零 PI 积分器与速度平滑状态，确保恢复使能时不发生突跳。 */
            for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
                IIR_Pi_Reset(&s_pi_states[i]);
            }
            IIR_Smoother_Reset(&s_vx_smoother);
            IIR_Smoother_Reset(&s_wz_smoother);
            host_twist.vx_mps = 0.0f;
            host_twist.wz_rad_s = 0.0f;
            Motor_ApplyOutput(cmd_pwm, false);
        }

        uint16_t actual_ccr[4] = {0u, 0u, 0u, 0u};
        BSP_Motor_ReadCcr(actual_ccr);

        /* Publish atomic snapshots for 20 Hz primary status & 10 Hz extended telemetry.
         * 在临界区内原子更新 20 Hz 主遥测帧与 10 Hz 扩展安全/回充遥测快照。 */
        taskENTER_CRITICAL();
        s_motion_allowed = safety_dec.allow_motor_output;
        s_telemetry_snapshot.stop_flag = safety_dec.stop_reason_mask;
        s_telemetry_snapshot.vx_mm_s = round_clamp_i16(measured_twist.vx_mps * 1000.0f);
        s_telemetry_snapshot.vy_mm_s = 0;
        s_telemetry_snapshot.vz_mrad_s = round_clamp_i16(measured_twist.wz_rad_s * 1000.0f);
        for (uint32_t i = 0u; i < 3u; ++i) {
            s_telemetry_snapshot.accel_lsb[i] = imu_snap.accel_lsb[i];
            s_telemetry_snapshot.gyro_lsb[i] = imu_snap.gyro_lsb[i];
        }
        s_telemetry_snapshot.voltage_mv = bat_mv;
        s_telemetry_snapshot.encoder_counts[0] = (int32_t)roundf(measured_wheels.fl_a * 1000.0f);
        s_telemetry_snapshot.encoder_counts[1] = (int32_t)roundf(measured_wheels.rl_b * 1000.0f);
        s_telemetry_snapshot.encoder_counts[2] = (int32_t)roundf(measured_wheels.rr_c * 1000.0f);
        s_telemetry_snapshot.encoder_counts[3] = (int32_t)roundf(measured_wheels.fr_d * 1000.0f);
        for (uint32_t i = 0u; i < 4u; ++i) {
            s_telemetry_snapshot.pwm_counts[i] = actual_ccr[i];
        }
        s_telemetry_snapshot.ultrasonic_front_mm = ultrasonic_mm;

        s_ext_snapshot.charging_state = pack_charging_state(&dock_snap);
        s_ext_snapshot.charging_current_ma = dock_snap.charging_current_ma;
        s_ext_snapshot.ir_mask = dock_snap.ir_mask;
        s_ext_snapshot.safety_flags = safety_dec.safety_flags_byte;
        s_ext_snapshot.d50a_alarm_mask = d50a_snap.fault_mask;
        s_dock_target_snapshot.generation = dock_snap.target_generation;
        s_dock_target_snapshot.vx_mm_s = round_clamp_i16(dock_snap.red_vx_mps * 1000.0f);
        s_dock_target_snapshot.wz_mrad_s = round_clamp_i16(dock_snap.red_vz_rad_s * 1000.0f);
        s_dock_target_snapshot.active = dock_snap.allow_recharge && dock_snap.red_state != 0u &&
                                        !dock_snap.charging;
        taskEXIT_CRITICAL();
    }
}

/* 20 Hz telemetry task: 50-byte status frame (20 Hz), 11-byte ext status (10 Hz), and 1 Hz beacon.
 * 20 Hz 遥测任务：20 Hz 主状态帧、10 Hz 扩展安全/回充帧与 1 Hz v3 会话通告帧时分复用发送。 */
static void telemetry_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint8_t slot_tick = 0u;

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IIR_TELEMETRY_PERIOD_MS));

        BSP_Adc_Update();

        IIR_StatusFrame snap;
        IIR_ExtStatusFrame ext_snap;
        IIR_DockTargetFrame dock_target_snap;
        bool allowed;
        taskENTER_CRITICAL();
        snap = s_telemetry_snapshot;
        ext_snap = s_ext_snapshot;
        dock_target_snap = s_dock_target_snapshot;
        allowed = s_motion_allowed;
        taskEXIT_CRITICAL();

        uint8_t status_buf[IIR_PROTO_STATUS_FRAME_LEN];
        if (IIR_Proto_EncodeStatusFrame(&snap, status_buf)) {
            (void)BSP_Uart3_SendAsync(status_buf, (uint16_t)sizeof(status_buf));
        }

        /* Wait 6 ms (> 4.4 ms DMA duration at 115200 bps) before sending secondary 11-byte frame.
         * 等待前序 50 字节遥测 DMA 发送完成（115200 bps 下约 4.4 ms）后再错峰发送 11 字节辅助帧。 */
        vTaskDelay(pdMS_TO_TICKS(6u));

        if (slot_tick == (IIR_SESSION_BEACON_TICKS - 1u)) {
            uint8_t sess_buf[IIR_PROTO_CMD_FRAME_LEN];
            if (IIR_Proto_EncodeSessionFrame(s_session_id, sess_buf)) {
                (void)BSP_Uart3_SendAsync(sess_buf, (uint16_t)sizeof(sess_buf));
            }
            BSP_Oled_ShowStatus(snap.voltage_mv, allowed, snap.vx_mm_s);
        } else if (slot_tick == (IIR_SESSION_BEACON_TICKS - 2u)) {
            /* Send version telemetry once per 20 ticks (1 s) for OTA verification.
             * 每 20 个 tick（1 秒）发送一次版本遥测，供 OTA 校验。 */
            IIR_FwVersionFrame version = {
                .major = (uint8_t)IIR_FW_VERSION_MAJOR,
                .minor = (uint8_t)IIR_FW_VERSION_MINOR,
                .patch = (uint8_t)IIR_FW_VERSION_PATCH,
                .chassis_type = (uint8_t)(strcmp(IIR_CHASSIS_TYPE, "DIFF") == 0 ? 0u : 1u),
                .git_sha_u32 = (uint32_t)strtoul(IIR_GIT_SHA, NULL, 16),
            };
            uint8_t version_buf[IIR_PROTO_VERSION_FRAME_LEN];
            if (IIR_Proto_EncodeVersionFrame(&version, version_buf)) {
                (void)BSP_Uart3_SendAsync(version_buf, (uint16_t)sizeof(version_buf));
            }
        } else if ((slot_tick % 2u) == 0u) {
            uint8_t ext_buf[IIR_PROTO_CMD_FRAME_LEN];
            if (IIR_Proto_EncodeExtStatusFrame(&ext_snap, ext_buf)) {
                (void)BSP_Uart3_SendAsync(ext_buf, (uint16_t)sizeof(ext_buf));
            }
        } else {
            uint8_t target_buf[IIR_PROTO_DOCK_TARGET_FRAME_LEN];
            if (IIR_Proto_EncodeDockTargetFrame(&dock_target_snap, target_buf)) {
                (void)BSP_Uart3_SendAsync(target_buf, (uint16_t)sizeof(target_buf));
            }
        }

        slot_tick++;
        if (slot_tick >= IIR_SESSION_BEACON_TICKS) {
            slot_tick = 0u;
        }
    }
}

bool IIR_AppTasks_Start(const IIR_AppHwHandles *hw)
{
    if (hw == NULL) {
        return false;
    }

    /* A new hardware-generated session distinguishes this boot from the previous one.
     * Failure to obtain a valid RNG word prevents the control tasks from starting.
     * 硬件随机数区分每次启动；随机数发生器失败时不启动控制任务。 */
    __HAL_RCC_RNG_CLK_ENABLE();
    RNG->CR |= RNG_CR_RNGEN;
    const uint32_t rng_start_ms = HAL_GetTick();
    while ((RNG->SR & RNG_SR_DRDY) == 0u) {
        if ((RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) != 0u ||
            (uint32_t)(HAL_GetTick() - rng_start_ms) > 100u) {
            RNG->CR &= ~RNG_CR_RNGEN;
            __HAL_RCC_RNG_CLK_DISABLE();
            return false;
        }
    }
    if ((RNG->SR & (RNG_SR_CECS | RNG_SR_SECS)) != 0u) {
        RNG->CR &= ~RNG_CR_RNGEN;
        __HAL_RCC_RNG_CLK_DISABLE();
        return false;
    }
    s_session_id = RNG->DR;
    RNG->CR &= ~RNG_CR_RNGEN;
    __HAL_RCC_RNG_CLK_DISABLE();

    s_kin_cfg = IIR_Kinematics_DefaultConfig();
    s_pi_cfg = IIR_Pi_DefaultConfig();
    s_low_speed_cfg = IIR_Control_DefaultLowSpeedConfig();

    for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
        IIR_Pi_Init(&s_pi_states[i]);
    }
    IIR_Smoother_Reset(&s_vx_smoother);
    IIR_Smoother_Reset(&s_wz_smoother);
    memset(&s_telemetry_snapshot, 0, sizeof(s_telemetry_snapshot));
    memset(&s_ext_snapshot, 0, sizeof(s_ext_snapshot));
    s_telemetry_snapshot.stop_flag = (uint8_t)IIR_SAFETY_STOP_UART_TIMEOUT;
    s_telemetry_snapshot.ultrasonic_front_mm = IIR_PROTO_ULTRASONIC_INVALID_MM;

    IIR_Safety_Init(&s_safety_state, IIR_BATTERY_CRIT_MV);
    IIR_Proto_RxInit(&s_rx_ctx, s_session_id, true);
    IIR_Proto_WatchdogInit(&s_cmd_wd, IIR_PROTO_DEFAULT_WATCHDOG_MS);

    if (!BSP_Motor_Init(hw->htim8_pwm, IIR_MOTOR_MAX_PWM) ||
        !BSP_Encoder_Init(hw->htim_enc_a, hw->htim_enc_b, hw->htim_enc_c, hw->htim_enc_d) ||
        !BSP_Uart3_Init(hw->huart3_host) ||
        !BSP_Adc_Init(hw->hadc1_bat) ||
        !BSP_HcSr04_Init(hw->htim1_echo) ||
        !BSP_D50A_Init(hw->huart2_d50a)) {
        return false;
    }

    /* Optional external hardware buses (ICM-20948, CAN dock) degrade safely without halting boot.
     * 外部扩展传感器总线（ICM-20948、CAN1 回充）在离线时安全降级标记，不阻塞底盘核心启动。 */
    (void)BSP_Icm20948_Init();
    (void)BSP_CanDock_Init();
    BSP_Sos_Init();
    BSP_Buzzer_Set(false);
    BSP_Rgb_Set(false, true, false);
    BSP_Oled_Init();

    if (xTaskCreate(control_task,
                    "ControlTask",
                    512u,
                    NULL,
                    (UBaseType_t)(tskIDLE_PRIORITY + 3u),
                    NULL) != pdPASS) {
        return false;
    }

    if (xTaskCreate(imu_task,
                    "ImuTask",
                    384u,
                    NULL,
                    (UBaseType_t)(tskIDLE_PRIORITY + 3u),
                    NULL) != pdPASS) {
        return false;
    }

    if (xTaskCreate(d50a_task,
                    "D50ATask",
                    384u,
                    NULL,
                    (UBaseType_t)(tskIDLE_PRIORITY + 1u),
                    NULL) != pdPASS) {
        return false;
    }

    if (xTaskCreate(telemetry_task,
                    "TelemetryTask",
                    512u,
                    NULL,
                    (UBaseType_t)(tskIDLE_PRIORITY + 2u),
                    NULL) != pdPASS) {
        return false;
    }

    return true;
}

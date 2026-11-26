/**
 * @file safety_manager.c
 * @brief Implementation of the pure-C Fail-Closed safety state machine and
 *        SOS audio/visual priority arbiter.
 *        纯 C 故障安全（Fail-Closed）底盘安全状态机与 SOS 声光优先级仲裁实现。
 */

#include "safety_manager.h"

#include <stddef.h>
#include <string.h>

#define IIR_SAFETY_DEFAULT_BLINK_MS 150u

void IIR_Safety_Init(IIR_SafetyState *state, uint16_t undervolt_mv)
{
    if (state == NULL) {
        return;
    }

    memset(state, 0, sizeof(*state));
    state->undervolt_mv = (undervolt_mv > 0u) ? undervolt_mv : IIR_SAFETY_DEFAULT_UNDERVOLT_MV;
    state->alarm_blink_period_ms = IIR_SAFETY_DEFAULT_BLINK_MS;
}

void IIR_Safety_LatchPhysicalSos(IIR_SafetyState *state)
{
    if (state == NULL) {
        return;
    }
    state->physical_sos_latched = true;
}

bool IIR_Safety_ClearPhysicalSos(IIR_SafetyState *state, bool physical_sos_still_pressed)
{
    if (state == NULL) {
        return false;
    }

    /* Fail-Closed: refuse clearing the SOS latch while PE5 button remains held LOW.
     * 故障安全约束：当 PE5 物理 SOS 按键仍被按下时，拒绝解除 SOS 锁存态。 */
    if (physical_sos_still_pressed) {
        state->physical_sos_latched = true;
        return false;
    }

    state->physical_sos_latched = false;
    return true;
}

void IIR_Safety_SetRemoteSos(IIR_SafetyState *state,
                             bool active,
                             bool physical_sos_still_pressed)
{
    if (state == NULL) {
        return;
    }

    if (active) {
        state->remote_sos_active = true;
        return;
    }

    /* Remote SET_SOS(0) resets remote SOS and clears physical latch once button is released.
     * 远程复位 SET_SOS(0) 会清除远程 SOS 标志，并在物理按键松开后同步清除本地锁存。 */
    state->remote_sos_active = false;
    if (!physical_sos_still_pressed) {
        state->physical_sos_latched = false;
    }
}

void IIR_Safety_SetLedBeepOverride(IIR_SafetyState *state, uint8_t mode, bool beep_enable)
{
    if (state == NULL) {
        return;
    }
    state->remote_led_mode = mode;
    state->remote_beep_active = beep_enable;
}

IIR_SafetyDecision IIR_Safety_Evaluate(const IIR_SafetyState *state,
                                       const IIR_SafetyInputs *inputs)
{
    IIR_SafetyDecision dec;
    memset(&dec, 0, sizeof(dec));

    /* Fail-Closed if caller passes invalid context pointers.
     * 若调用方传入空指针，立即按故障安全停机策略返回禁止输出。 */
    if (state == NULL || inputs == NULL) {
        dec.allow_motor_output = false;
        dec.stop_reason_mask = (uint8_t)IIR_SAFETY_STOP_PE4_ESTOP;
        dec.rgb_mode = IIR_RGB_MODE_WARN_YELLOW;
        dec.rgb_red = true;
        dec.rgb_green = true;
        dec.rgb_blue = false;
        return dec;
    }

    const bool pe4_estop = !inputs->hw_enable_pin_high;
    const bool sos_active = state->physical_sos_latched ||
                            state->remote_sos_active ||
                            inputs->physical_sos_pressed;
    const bool undervolt = (inputs->battery_mv < state->undervolt_mv);
    const bool d50a_fault = ((inputs->d50a_alarm_mask & IIR_SAFETY_D50A_FAULT_MASK) != 0u);
    const bool uart_timeout = inputs->uart_timed_out;

    uint8_t stop_mask = (uint8_t)IIR_SAFETY_STOP_NONE;
    if (pe4_estop) {
        stop_mask |= (uint8_t)IIR_SAFETY_STOP_PE4_ESTOP;
    }
    if (sos_active) {
        stop_mask |= (uint8_t)IIR_SAFETY_STOP_SOS;
    }
    if (uart_timeout) {
        stop_mask |= (uint8_t)IIR_SAFETY_STOP_UART_TIMEOUT;
    }
    if (undervolt) {
        stop_mask |= (uint8_t)IIR_SAFETY_STOP_UNDERVOLT;
    }
    if (d50a_fault) {
        stop_mask |= (uint8_t)IIR_SAFETY_STOP_D50A_FAULT;
    }
    if (inputs->host_stopped) {
        stop_mask |= (uint8_t)IIR_SAFETY_STOP_HOST;
    }

    dec.stop_reason_mask = stop_mask;
    dec.sos_active = sos_active;
    dec.allow_motor_output = (stop_mask == (uint8_t)IIR_SAFETY_STOP_NONE);

    /* Encode protocol_v3_ext safety_flags bitmask (bit0=SOS, bit1=PE4, bit2=UART, bit3=UV, bit4=D50A).
     * 按照 protocol_v3_ext 契约编码 safety_flags 字段，供 11 字节扩展遥测帧上报。 */
    uint8_t flags = 0u;
    if (sos_active) {
        flags |= (1u << 0);
    }
    if (pe4_estop) {
        flags |= (1u << 1);
    }
    if (uart_timeout) {
        flags |= (1u << 2);
    }
    if (undervolt) {
        flags |= (1u << 3);
    }
    if (d50a_fault) {
        flags |= (1u << 4);
    }
    if (inputs->host_stopped) {
        flags |= (1u << 5);
    }
    dec.safety_flags_byte = flags;

    const uint32_t period_ms =
        (state->alarm_blink_period_ms > 0u) ? state->alarm_blink_period_ms : IIR_SAFETY_DEFAULT_BLINK_MS;
    const bool strobe_high = (((inputs->now_ms / period_ms) & 1u) == 0u);

    /* Priority 1 (Highest): SOS active triggers immediate high-frequency red strobe & siren.
     * 最高优先级：SOS 报警激活时强制进入红灯高频爆闪与急促蜂鸣，覆盖一切远程灯效。 */
    if (sos_active) {
        dec.rgb_mode = IIR_RGB_MODE_SOS_ALARM;
        dec.rgb_red = strobe_high;
        dec.rgb_green = false;
        dec.rgb_blue = false;
        dec.buzzer_active = strobe_high;
        return dec;
    }

    /* Priority 2: Hardware faults (PE4 E-stop, undervoltage, D50A protection alarm) warn yellow.
     * 次高优先级：硬件故障（PE4 急停、严重欠压、D50A 保护故障）常亮/指示黄色告警。 */
    if (pe4_estop || undervolt || d50a_fault) {
        dec.rgb_mode = IIR_RGB_MODE_WARN_YELLOW;
        dec.rgb_red = true;
        dec.rgb_green = true;
        dec.rgb_blue = false;
        dec.buzzer_active = state->remote_beep_active || undervolt || d50a_fault;
        return dec;
    }

    /* Priority 3: Explicit remote SET_LED_BEEP override when local hardware is safe.
     * 第三优先级：本地硬件安全无故障时，响应上位机显式下发的 SET_LED_BEEP 声光模式。 */
    if (state->remote_led_mode != 0u) {
        switch (state->remote_led_mode) {
        case 1u:
            dec.rgb_mode = IIR_RGB_MODE_SOS_ALARM;
            dec.rgb_red = strobe_high;
            dec.rgb_green = false;
            dec.rgb_blue = false;
            break;
        case 2u:
            dec.rgb_mode = IIR_RGB_MODE_WARN_YELLOW;
            dec.rgb_red = true;
            dec.rgb_green = true;
            dec.rgb_blue = false;
            break;
        case 3u:
            dec.rgb_mode = IIR_RGB_MODE_GREEN_SOLID;
            dec.rgb_red = false;
            dec.rgb_green = true;
            dec.rgb_blue = false;
            break;
        case 4u:
            dec.rgb_mode = IIR_RGB_MODE_BLUE_DOCK;
            dec.rgb_red = false;
            dec.rgb_green = false;
            dec.rgb_blue = true;
            break;
        case 5u:
        default:
            dec.rgb_mode = IIR_RGB_MODE_ALL_OFF;
            dec.rgb_red = false;
            dec.rgb_green = false;
            dec.rgb_blue = false;
            break;
        }
        dec.buzzer_active = state->remote_beep_active;
        return dec;
    }

    /* Priority 4: Auto-docking indicator (blue) -> Active ROS motion (green) -> Standby heartbeat.
     * 第四优先级：自主回充指示（蓝灯） -> 正常运行使能（绿灯） -> 串口待机心跳。 */
    if (inputs->docking_active) {
        dec.rgb_mode = IIR_RGB_MODE_BLUE_DOCK;
        dec.rgb_red = false;
        dec.rgb_green = false;
        dec.rgb_blue = true;
    } else if (dec.allow_motor_output) {
        dec.rgb_mode = IIR_RGB_MODE_GREEN_SOLID;
        dec.rgb_red = false;
        dec.rgb_green = true;
        dec.rgb_blue = false;
    } else {
        dec.rgb_mode = IIR_RGB_MODE_NORMAL_HEARTBEAT;
        const bool heartbeat_on = (((inputs->now_ms / 500u) & 1u) == 0u);
        dec.rgb_red = false;
        dec.rgb_green = heartbeat_on;
        dec.rgb_blue = false;
    }

    dec.buzzer_active = state->remote_beep_active;
    return dec;
}

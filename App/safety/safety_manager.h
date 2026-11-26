/**
 * @file safety_manager.h
 * @brief Pure-C Fail-Closed chassis safety state machine and audio/visual priority
 *        arbiter (PE4 E-stop, PE5 SOS, UART watchdog, battery undervoltage, D50A fault).
 *        零 HAL 依赖的纯 C 故障安全（Fail-Closed）底盘安全状态机与声光报警优先级仲裁器。
 */

#ifndef IIR_APP_SAFETY_SAFETY_MANAGER_H
#define IIR_APP_SAFETY_SAFETY_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Default severe battery undervoltage lockout threshold in millivolts (20.5 V).
 *  默认电池严重欠压停机保护阈值（20500 mV）。 */
#define IIR_SAFETY_DEFAULT_UNDERVOLT_MV 20500u

/** Active-HIGH D50A RS485 protection fault mask bits (bits 0..3).
 *  D50A 驱动器 RS485 保护故障掩码（高电平有效：过流/过温/过压/欠压）。 */
#define IIR_SAFETY_D50A_FAULT_MASK 0x0Fu

/**
 * @brief Bitmask flags written to status_frame.stop_flag to report local lockout causes.
 *        写入上行状态帧 stop_flag 字段的本地安全停机原因位掩码。
 */
typedef enum {
    /** All local safety invariants satisfied; motor output permitted.
     *  全部本地安全条件满足，允许电机输出。 */
    IIR_SAFETY_STOP_NONE = 0u,

    /** Hardware enable switch (PE4) switched LOW (E-stop active).
     *  硬件使能开关（PE4）拉低触发急停。 */
    IIR_SAFETY_STOP_PE4_ESTOP = (1u << 0),

    /** Physical SOS button (PE5) latched or remote SET_SOS(1) active.
     *  物理 SOS 按钮（PE5）锁存或上位机远程 SET_SOS(1) 报警激活。 */
    IIR_SAFETY_STOP_SOS = (1u << 1),

    /** Host USART3 speed command watchdog exceeded 500 ms timeout.
     *  上位机 USART3 速度指令看门狗超时（> 500 ms）。 */
    IIR_SAFETY_STOP_UART_TIMEOUT = (1u << 2),

    /** Battery voltage dropped below the severe undervoltage lockout threshold.
     *  电池组电压低于严重欠压保护阈值。 */
    IIR_SAFETY_STOP_UNDERVOLT = (1u << 3),

    /** D50A RS485 motor driver reported overcurrent, overtemp, overvoltage, or undervoltage.
     *  D50A 电机驱动器通过 RS485 上报过流、过温、过压或欠压故障。 */
    IIR_SAFETY_STOP_D50A_FAULT = (1u << 4),

    /** Host requested an immediate motor stop.
     *  上位机请求立即切断电机输出。 */
    IIR_SAFETY_STOP_HOST = (1u << 5)
} IIR_SafetyStopReason;

/**
 * @brief RGB indicator display modes arbitrated by the safety manager.
 *        安全管理器仲裁的 RGB 状态指示灯显示模式枚举。
 */
typedef enum {
    /** Normal standby / UART timeout slow heartbeat indicator.
     *  正常待机或串口等待时的常规心跳模式。 */
    IIR_RGB_MODE_NORMAL_HEARTBEAT = 0,

    /** Motion enabled and all safety checks green.
     *  运动使能且全部本地安全门通过时的绿色常亮模式。 */
    IIR_RGB_MODE_GREEN_SOLID = 1,

    /** Recoverable warning (PE4 estop, UART timeout, undervoltage, D50A fault, or remote yellow).
     *  可恢复安全告警（PE4 急停、串口超时、欠压、D50A 故障或远程黄色告警）。 */
    IIR_RGB_MODE_WARN_YELLOW = 2,

    /** Highest-priority SOS high-frequency red strobe alarm.
     *  最高优先级 SOS 报警红灯高频爆闪模式。 */
    IIR_RGB_MODE_SOS_ALARM = 3,

    /** Auto-docking indicator (blue solid/pulse when healthy).
     *  自主回充对接状态蓝色指示模式。 */
    IIR_RGB_MODE_BLUE_DOCK = 4,

    /** Explicit RGB off override commanded via SET_LED_BEEP.
     *  经由 SET_LED_BEEP 显式关闭灯带模式。 */
    IIR_RGB_MODE_ALL_OFF = 5
} IIR_RgbMode;

/**
 * @brief Persistent local safety latch and remote indicator override state.
 *        本地安全锁存状态及上位机声光覆盖配置结构体。
 */
typedef struct {
    /** True while physical PE5 SOS alarm is latched until reset.
     *  物理 PE5 SOS 按键触发后锁存为 true，直到上位机/手动合法复位。 */
    bool physical_sos_latched;

    /** True while host SET_SOS(1) emergency stop is active.
     *  上位机下发 SET_SOS(1) 远程紧急报警期间为 true。 */
    bool remote_sos_active;

    /** Remote LED mode override (0 = automatic arbitration, 1..5 explicit override).
     *  上位机远程灯效覆盖模式（0 表示自动仲裁，1..5 为显式指定模式）。 */
    uint8_t remote_led_mode;

    /** Remote buzzer request when no higher-priority SOS alarm is active.
     *  上位机请求开启蜂鸣器（仅在未触发更高优先级 SOS 时生效）。 */
    bool remote_beep_active;

    /** Severe undervoltage lockout threshold in mV.
     *  电池严重欠压停机保护阈值（mV）。 */
    uint16_t undervolt_mv;

    /** Strobe/beep toggle half-period in milliseconds for SOS alarm cadence.
     *  SOS 高频声光报警的亮灭闪烁半周期（毫秒）。 */
    uint32_t alarm_blink_period_ms;
} IIR_SafetyState;

/**
 * @brief Per-cycle sampled hardware and link inputs evaluated by the safety manager.
 *        每个控制周期采集的硬件与通信链路安全输入快照。
 */
typedef struct {
    /** Hardware enable switch state on PE4 (true = HIGH/enabled, false = LOW/E-stop).
     *  PE4 硬件使能开关电平状态（true 表示高电平使能，false 表示急停）。 */
    bool hw_enable_pin_high;

    /** Instantaneous physical SOS button state on PE5 (true = pressed/LOW).
     *  PE5 物理 SOS 按键瞬时按压状态（true 表示按键正在被按下/低电平）。 */
    bool physical_sos_pressed;

    /** True when USART3 command watchdog has exceeded its timeout (> 500 ms).
     *  USART3 上位机指令看门狗已超时（> 500 ms）时为 true。 */
    bool uart_timed_out;

    /** Most recent host motion command is a stop frame.
     *  最近一帧上位机运动命令为锁停帧。 */
    bool host_stopped;

    /** Filtered battery pack voltage in millivolts (mV).
     *  经滑动滤波的当前电池电压，单位为毫伏（mV）。 */
    uint16_t battery_mv;

    /** Active-HIGH D50A RS485 protection alarm mask (bits 0..3).
     *  D50A 驱动器 RS485 保护报警掩码（位 0..3 高电平有效）。 */
    uint8_t d50a_alarm_mask;

    /** True when auto-docking IR control is actively executing.
     *  当前正处于 CAN 自动回充红外对接控制态时为 true。 */
    bool docking_active;

    /** Current monotonic tick in milliseconds used to drive alarm cadence.
     *  当前单调毫秒时间戳，用于驱动声光报警闪烁节拍。 */
    uint32_t now_ms;
} IIR_SafetyInputs;

/**
 * @brief Deterministic safety decision output consumed by Motor_ApplyOutput and BSP indicators.
 *        供唯一 PWM 出口 Motor_ApplyOutput 及声光外设消费的确定性安全决策结果。
 */
typedef struct {
    /** True ONLY when every local Fail-Closed invariant passes.
     *  仅当全部本地故障安全红线均满足时为 true；否则必须立即切断电机 PWM。 */
    bool allow_motor_output;

    /** Non-zero bitmask of IIR_SafetyStopReason written directly into stop_flag.
     *  对应 IIR_SafetyStopReason 的停机原因位掩码，直接填入状态帧 stop_flag。 */
    uint8_t stop_reason_mask;

    /** True if physical or remote SOS alarm is currently active.
     *  物理 SOS 或远程 SOS 任一处于激活态时为 true。 */
    bool sos_active;

    /** Normalized safety_flags byte transmitted in the 11-byte extended telemetry frame.
     *  用于 11 字节扩展遥测帧传输的归一化 safety_flags 字节。 */
    uint8_t safety_flags_byte;

    /** Arbitrated RGB indicator mode.
     *  经优先级仲裁后的目标 RGB 指示灯模式。 */
    IIR_RgbMode rgb_mode;

    /** Final RGB red channel output state after strobe modulation.
     *  经过爆闪节拍调制后的红色通道开关输出状态。 */
    bool rgb_red;

    /** Final RGB green channel output state.
     *  经过仲裁与调制后的绿色通道开关输出状态。 */
    bool rgb_green;

    /** Final RGB blue channel output state.
     *  经过仲裁与调制后的蓝色通道开关输出状态。 */
    bool rgb_blue;

    /** Final buzzer output state on PD11 (SOS siren has highest priority).
     *  PD11 蜂鸣器最终输出状态（SOS 报警声音具有最高优先级）。 */
    bool buzzer_active;
} IIR_SafetyDecision;

/**
 * @brief Initialize the safety manager state with Fail-Closed defaults.
 *        以故障安全默认值初始化安全管理器状态。
 */
void IIR_Safety_Init(IIR_SafetyState *state, uint16_t undervolt_mv);

/**
 * @brief Latch the physical PE5 SOS emergency stop state.
 *        锁存 PE5 物理按键触发的 SOS 紧急停机报警态。
 */
void IIR_Safety_LatchPhysicalSos(IIR_SafetyState *state);

/**
 * @brief Clear the physical SOS latch only if the button is physically released.
 *        仅当 PE5 物理按键已松开时才允许清除物理 SOS 锁存标志。
 *
 * @return true if physical_sos_latched is now false; false if still held pressed.
 *         若成功清除返回 true；若按键仍被按住维持锁存则返回 false。
 */
bool IIR_Safety_ClearPhysicalSos(IIR_SafetyState *state, bool physical_sos_still_pressed);

/**
 * @brief Apply remote SET_SOS(active) command from host bridge.
 *        处理上位机下发的远程 SET_SOS(active) 报警置位与复位指令。
 */
void IIR_Safety_SetRemoteSos(IIR_SafetyState *state,
                             bool active,
                             bool physical_sos_still_pressed);

/**
 * @brief Set remote LED and buzzer indicator override via SET_LED_BEEP.
 *        通过上位机 SET_LED_BEEP 指令设置声光提示覆盖模式（优先级低于 SOS）。
 */
void IIR_Safety_SetLedBeepOverride(IIR_SafetyState *state, uint8_t mode, bool beep_enable);

/**
 * @brief Evaluate all local safety invariants and audio/visual outputs for one tick.
 *        评估单周期全部本地安全红线并生成电机使能与声光控制输出。
 */
IIR_SafetyDecision IIR_Safety_Evaluate(const IIR_SafetyState *state,
                                       const IIR_SafetyInputs *inputs);

#ifdef __cplusplus
}
#endif

#endif /* IIR_APP_SAFETY_SAFETY_MANAGER_H */

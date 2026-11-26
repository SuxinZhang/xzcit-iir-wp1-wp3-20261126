/**
 * @file motor_control.h
 * @brief Pure-C incremental PI controller, velocity smoother, low-speed compensation,
 *        and the unique Fail-Closed Motor_ApplyOutput PWM gate.
 *        零 HAL 依赖的纯 C 增量式 PI 控制器、速度平滑器、低速非线性补偿及唯一电机 PWM 出口门禁。
 */

#ifndef IIR_APP_CONTROL_MOTOR_CONTROL_H
#define IIR_APP_CONTROL_MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Number of independent drive motor channels (A, B, C, D).
 *        独立驱动电机通道数量（A、B、C、D 四路）。
 */
#define IIR_MOTOR_CHANNEL_COUNT 4u

/**
 * @brief Incremental PI controller parameters.
 *        增量式 PI 速度闭环控制器参数。
 */
typedef struct {
    /** Proportional gain mapping m/s error difference to PWM counts.
     *  比例增益，将误差变化量（m/s）映射为 PWM 计数值增量。 */
    float kp;

    /** Integral gain mapping m/s error per step to PWM counts.
     *  积分增益，将单步速度误差（m/s）映射为 PWM 计数值增量。 */
    float ki;

    /** Symmetric output saturation limit in PWM counts (must be > 0).
     *  对称输出饱和限幅（PWM 计数值，必须大于 0）。 */
    float output_limit;
} IIR_PiConfig;

/**
 * @brief Runtime state for one incremental PI loop instance.
 *        单个增量式 PI 控制回路的运行时状态。
 */
typedef struct {
    /** Previous control cycle velocity error (target - measured) in m/s.
     *  上一控制周期的速度误差（目标减实测），单位为 m/s。 */
    float prev_error;

    /** Accumulated PWM output before hardware clamping.
     *  送往硬件限幅前的累积 PWM 输出值。 */
    float output;
} IIR_PiState;

/**
 * @brief Runtime state for a rate-limited scalar velocity smoother.
 *        带斜率限幅的标量速度平滑器运行时状态。
 */
typedef struct {
    /** Current smoothed velocity value in m/s or rad/s.
     *  当前平滑后的速度值，单位为 m/s 或 rad/s。 */
    float current;
} IIR_SmootherState;

/**
 * @brief Low-speed nonlinear static-friction feedforward configuration (0.01 to 1.3 m/s).
 *        覆盖 0.01 至 1.3 m/s 范围的低速静摩擦非线性补偿配置。
 */
typedef struct {
    /** Absolute velocity below which motor command is forced to zero (deadband) in m/s.
     *  低于该绝对速度阈值时强制输出零（死区），单位为 m/s。 */
    float stop_deadband_mps;

    /** Minimum controllable cruising speed in m/s (Tender Clause 7.1.3: 0.01 m/s).
     *  最小可控巡航线速度，单位为 m/s（标书第 7.1.3 条：0.01 m/s）。 */
    float min_linear_speed_mps;

    /** Upper speed boundary where low-speed stiction feedforward tapers to zero in m/s.
     *  低速静摩擦前馈补偿渐变归零的速度上限，单位为 m/s。 */
    float low_speed_threshold_mps;

    /** Feedforward PWM magnitude applied at min_linear_speed_mps to overcome stiction.
     *  在 min_linear_speed_mps 处施加的静摩擦克服前馈 PWM 幅值。 */
    float static_ff_pwm;

    /** Maximum absolute PWM magnitude allowed after compensation.
     *  补偿后允许的最大 PWM 绝对值上限。 */
    float output_limit;
} IIR_LowSpeedCompConfig;

/**
 * @brief Hardware PWM sink callback invoked exclusively by Motor_ApplyOutput.
 *        仅由唯一出口 Motor_ApplyOutput 调用的底层硬件 PWM 写入回调类型。
 *
 * @param[in] pwm Four clamped PWM commands for channels A/B/C/D.
 *                经过安全门控与限幅后的 A/B/C/D 四路电机 PWM 值。
 */
typedef void (*IIR_MotorHwApplyFn)(const int16_t pwm[IIR_MOTOR_CHANNEL_COUNT]);

/**
 * @brief Return default incremental PI parameters tuned for the C50C differential chassis.
 *        返回适配 C50C 差速底盘的默认增量式 PI 参数。
 */
IIR_PiConfig IIR_Pi_DefaultConfig(void);

/**
 * @brief Initialize an incremental PI state structure to zero.
 *        初始化增量式 PI 状态结构体并清零。
 */
void IIR_Pi_Init(IIR_PiState *state);

/**
 * @brief Reset an incremental PI state structure on safety stop or mode switch.
 *        在安全停机或模式切换时复位增量式 PI 状态以消除积分累积。
 */
void IIR_Pi_Reset(IIR_PiState *state);

/**
 * @brief Advance an incremental PI controller by one control tick.
 *        推进一步增量式 PI 速度闭环控制计算。
 *
 * Non-finite inputs reset the state and return 0.0f (Fail-Closed).
 * 检测到非有限输入（NaN/Inf）时自动复位状态并返回 0.0f（故障安全）。
 *
 * @param[in]     cfg          PI parameters.
 *                             PI 控制器参数配置。
 * @param[in,out] state        PI state updated in place.
 *                             原地更新的 PI 状态指针。
 * @param[in]     target_mps   Desired wheel speed in m/s.
 *                             目标车轮线速度，单位为 m/s。
 * @param[in]     measured_mps Measured wheel speed in m/s.
 *                             实测车轮线速度，单位为 m/s。
 * @return Saturated PWM command in [-output_limit, +output_limit].
 *         限幅在 [-output_limit, +output_limit] 内的 PWM 控制量。
 */
float IIR_Pi_Update(const IIR_PiConfig *cfg,
                    IIR_PiState *state,
                    float target_mps,
                    float measured_mps);

/**
 * @brief Reset a velocity smoother back to 0.0f.
 *        将速度平滑器当前输出复位为 0.0f。
 */
void IIR_Smoother_Reset(IIR_SmootherState *state);

/**
 * @brief Rate-limit a scalar target command toward target by at most max_step per cycle.
 *        按每周期最大步长 max_step 对标量目标速度执行斜率平滑。
 *
 * @param[in,out] state    Smoother state.
 *                         平滑器状态指针。
 * @param[in]     target   Desired velocity command.
 *                         期望目标速度。
 * @param[in]     max_step Maximum change permitted in one tick (must be > 0).
 *                         单周期允许的最大速度变化量（必须大于 0）。
 * @param[in]     bypass   When true, jump directly to target (e.g. IR precision docking).
 *                         为 true 时旁路平滑直接跟踪目标（如红外精对接场景）。
 * @return Updated smoothed velocity.
 *         更新后的平滑速度值。
 */
float IIR_Smoother_Step(IIR_SmootherState *state,
                        float target,
                        float max_step,
                        bool bypass);

/**
 * @brief Return default low-speed nonlinear compensation parameters.
 *        返回默认的低速非线性静摩擦补偿参数配置。
 */
IIR_LowSpeedCompConfig IIR_Control_DefaultLowSpeedConfig(void);

/**
 * @brief Apply directional static-friction feedforward compensation at ultra-low speeds.
 *        在极低速段叠加方向性静摩擦前馈补偿，改善低速起步与防抖性能。
 *
 * @param[in] cfg        Low-speed compensation parameters.
 *                       低速补偿配置参数。
 * @param[in] target_mps Target wheel velocity in m/s.
 *                       目标车轮线速度，单位为 m/s。
 * @param[in] raw_pwm    Raw PI output in PWM counts.
 *                       原始 PI 输出的 PWM 值。
 * @return Compensated and saturated PWM command.
 *         叠加补偿并限幅后的 PWM 值。
 */
float IIR_Control_CompensateLowSpeed(const IIR_LowSpeedCompConfig *cfg,
                                     float target_mps,
                                     float raw_pwm);

/**
 * @brief Register the single hardware PWM sink driver and global absolute PWM ceiling.
 *        注册唯一底层电机 PWM 输出驱动回调与全局绝对值上限。
 *
 * @param[in] fn          Hardware callback invoked by Motor_ApplyOutput (may be NULL in tests).
 *                        由 Motor_ApplyOutput 调用的硬件输出回调（单测时可为 NULL）。
 * @param[in] max_pwm_abs Maximum allowed absolute PWM counts (<= 0 disables output).
 *                        允许的最大 PWM 绝对值（<= 0 时禁止输出非零值）。
 */
void Motor_RegisterOutputDriver(IIR_MotorHwApplyFn fn, int16_t max_pwm_abs);

/**
 * @brief Unique motor PWM output gate enforcing Fail-Closed safety state machine checks.
 *        全固件唯一的电机 PWM 输出出口，强制执行故障安全门控。
 *
 * No task or module may bypass this function to write timer CCR registers directly
 * (AI_EXECUTION_PLAN Invariant 2).
 * 任何任务或模块严禁绕过本函数直接写定时器 CCR 寄存器（工程红线第 2 条）。
 *
 * @param[in] target_pwm   Requested PWM counts for channels A/B/C/D.
 *                         请求输出的 A/B/C/D 四路电机 PWM 值。
 * @param[in] safety_allow True only when all local safety invariants permit motion.
 *                         仅当本地所有安全条件均满足时为 true；否则强制清零停机。
 */
void Motor_ApplyOutput(const int16_t target_pwm[IIR_MOTOR_CHANNEL_COUNT], bool safety_allow);

/**
 * @brief Read back the latest PWM counts passed through Motor_ApplyOutput.
 *        回读最近一次经由 Motor_ApplyOutput 下发的四路 PWM 输出值（用于遥测与测试）。
 *
 * @param[out] out_pwm Output array receiving the 4 latest applied PWM counts.
 *                     接收最近一次实际下发 PWM 值的 4 元素数组。
 */
void Motor_GetLastAppliedOutput(int16_t out_pwm[IIR_MOTOR_CHANNEL_COUNT]);

#ifdef __cplusplus
}
#endif

#endif /* IIR_APP_CONTROL_MOTOR_CONTROL_H */

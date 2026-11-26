/**
 * @file motor_control.c
 * @brief Implementation of incremental PI control, velocity smoothing, low-speed
 *        compensation, and the unique Motor_ApplyOutput safety gate.
 *        增量式 PI 控制、速度平滑、低速非线性补偿与唯一电机 PWM 出口门禁实现。
 */

#include "motor_control.h"

#include <math.h>
#include <stddef.h>

/* Module-private state for the single motor PWM output gate (Invariant 2).
 * 唯一电机 PWM 输出门禁的模块私有状态（工程红线第 2 条）。 */
static IIR_MotorHwApplyFn s_hw_apply_fn = NULL;
static int16_t s_max_pwm_abs = 0;
static int16_t s_last_applied_pwm[IIR_MOTOR_CHANNEL_COUNT] = {0, 0, 0, 0};

/* Clamp a float value within [-limit, +limit] and fail-closed to 0 on NaN/Inf.
 * 将浮点数限制在 [-limit, +limit] 区间内，遇到 NaN/Inf 时故障安全归零。 */
static float clamp_float_sym(float value, float limit)
{
    if (!isfinite(value) || !isfinite(limit) || limit <= 0.0f) {
        return 0.0f;
    }
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

IIR_PiConfig IIR_Pi_DefaultConfig(void)
{
    IIR_PiConfig cfg = {
        .kp = 300.0f,
        .ki = 300.0f,
        .output_limit = 7200.0f,
    };
    return cfg;
}

void IIR_Pi_Init(IIR_PiState *state)
{
    if (state == NULL) {
        return;
    }
    state->prev_error = 0.0f;
    state->output = 0.0f;
}

void IIR_Pi_Reset(IIR_PiState *state)
{
    IIR_Pi_Init(state);
}

float IIR_Pi_Update(const IIR_PiConfig *cfg,
                    IIR_PiState *state,
                    float target_mps,
                    float measured_mps)
{
    if (state == NULL) {
        return 0.0f;
    }

    if (cfg == NULL ||
        !isfinite(cfg->kp) || !isfinite(cfg->ki) ||
        !isfinite(cfg->output_limit) || cfg->output_limit <= 0.0f ||
        !isfinite(target_mps) || !isfinite(measured_mps)) {
        /* Fail-closed: clear integrator state if configuration or inputs are corrupted.
         * 故障安全：若配置或输入出现非有限数值则清零积分状态并停机。 */
        IIR_Pi_Reset(state);
        return 0.0f;
    }

    const float error = target_mps - measured_mps;
    const float delta_u = cfg->kp * (error - state->prev_error) + cfg->ki * error;
    state->prev_error = error;

    /* Clamp accumulated output to prevent windup beyond hardware PWM bounds.
     * 对累积输出执行饱和限幅，防止超出硬件 PWM 量程产生积分饱和。 */
    state->output = clamp_float_sym(state->output + delta_u, cfg->output_limit);
    return state->output;
}

void IIR_Smoother_Reset(IIR_SmootherState *state)
{
    if (state == NULL) {
        return;
    }
    state->current = 0.0f;
}

float IIR_Smoother_Step(IIR_SmootherState *state,
                        float target,
                        float max_step,
                        bool bypass)
{
    if (state == NULL) {
        return 0.0f;
    }

    if (!isfinite(target) || !isfinite(state->current)) {
        state->current = 0.0f;
        return 0.0f;
    }

    if (bypass) {
        state->current = target;
        return state->current;
    }

    if (!isfinite(max_step) || max_step <= 0.0f) {
        return state->current;
    }

    const float diff = target - state->current;
    if (diff > max_step) {
        state->current += max_step;
    } else if (diff < -max_step) {
        state->current -= max_step;
    } else {
        state->current = target;
    }
    return state->current;
}

IIR_LowSpeedCompConfig IIR_Control_DefaultLowSpeedConfig(void)
{
    IIR_LowSpeedCompConfig cfg = {
        .stop_deadband_mps = 0.005f,
        .min_linear_speed_mps = 0.01f,
        .low_speed_threshold_mps = 0.15f,
        .static_ff_pwm = 320.0f,
        .output_limit = 7200.0f,
    };
    return cfg;
}

float IIR_Control_CompensateLowSpeed(const IIR_LowSpeedCompConfig *cfg,
                                     float target_mps,
                                     float raw_pwm)
{
    if (cfg == NULL || !isfinite(target_mps) || !isfinite(raw_pwm) ||
        !isfinite(cfg->output_limit) || cfg->output_limit <= 0.0f) {
        return 0.0f;
    }

    const float speed_abs = fabsf(target_mps);

    /* Force zero output inside stop deadband so stationary chassis does not creep or hum.
     * 在静止死区内强制输出零，防止底盘停驻时发生蠕动或电机啸叫。 */
    if (speed_abs == 0.0f || speed_abs < cfg->stop_deadband_mps) {
        return 0.0f;
    }

    float compensated = raw_pwm;
    if (cfg->static_ff_pwm > 0.0f &&
        cfg->low_speed_threshold_mps > cfg->min_linear_speed_mps &&
        speed_abs <= cfg->low_speed_threshold_mps) {
        /* Apply full stiction feedforward at/below min_linear_speed_mps and taper linearly.
         * 在最小可控速度及以下施加全额静摩擦前馈，并随速度升高线性衰减至零。 */
        float ratio = 1.0f;
        if (speed_abs > cfg->min_linear_speed_mps) {
            const float span = cfg->low_speed_threshold_mps - cfg->min_linear_speed_mps;
            ratio = (cfg->low_speed_threshold_mps - speed_abs) / span;
        }
        const float dir = (target_mps > 0.0f) ? 1.0f : -1.0f;
        compensated += dir * (cfg->static_ff_pwm * ratio);
    }

    return clamp_float_sym(compensated, cfg->output_limit);
}

void Motor_RegisterOutputDriver(IIR_MotorHwApplyFn fn, int16_t max_pwm_abs)
{
    s_hw_apply_fn = fn;
    s_max_pwm_abs = (max_pwm_abs > 0) ? max_pwm_abs : 0;
}

void Motor_ApplyOutput(const int16_t target_pwm[IIR_MOTOR_CHANNEL_COUNT], bool safety_allow)
{
    int16_t final_pwm[IIR_MOTOR_CHANNEL_COUNT] = {0, 0, 0, 0};

    if (safety_allow && target_pwm != NULL && s_max_pwm_abs > 0) {
        const int32_t limit = (int32_t)s_max_pwm_abs;
        for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
            int32_t v = (int32_t)target_pwm[i];
            if (v > limit) {
                v = limit;
            } else if (v < -limit) {
                v = -limit;
            }
            final_pwm[i] = (int16_t)v;
        }
    }

    for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
        s_last_applied_pwm[i] = final_pwm[i];
    }

    if (s_hw_apply_fn != NULL) {
        s_hw_apply_fn(final_pwm);
    }
}

void Motor_GetLastAppliedOutput(int16_t out_pwm[IIR_MOTOR_CHANNEL_COUNT])
{
    if (out_pwm == NULL) {
        return;
    }
    for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
        out_pwm[i] = s_last_applied_pwm[i];
    }
}

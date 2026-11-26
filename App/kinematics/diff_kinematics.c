/**
 * @file diff_kinematics.c
 * @brief Implementation of pure-C four-wheel differential drive kinematics.
 *        纯 C 四轮差速底盘正逆运动学实现。
 */

#include "diff_kinematics.h"

#include <math.h>
#include <stddef.h>

/* Single-precision Pi constant to avoid double-promotion warnings under -Wconversion.
 * 单精度圆周率常量，避免在开启 -Wconversion 时触发双精度隐式转换告警。 */
#define IIR_PI_F 3.14159265358979323846f

/* Helper to clamp a finite float symmetrically within [-limit, +limit].
 * 将有限浮点数对称限幅在 [-limit, +limit] 区间内的辅助函数。 */
static float clamp_sym(float value, float limit)
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

/* Validate core geometric parameters before solving kinematics.
 * 在执行运动学解算前校验核心几何与限幅参数的有效性。 */
static bool is_valid_config(const IIR_KinematicsConfig *cfg)
{
    if (cfg == NULL) {
        return false;
    }
    if (!isfinite(cfg->effective_half_track_m) || cfg->effective_half_track_m <= 0.0f) {
        return false;
    }
    if (!isfinite(cfg->max_linear_speed_mps) || cfg->max_linear_speed_mps <= 0.0f) {
        return false;
    }
    if (!isfinite(cfg->max_angular_speed_rad_s) || cfg->max_angular_speed_rad_s <= 0.0f) {
        return false;
    }
    if (!isfinite(cfg->max_wheel_speed_mps) || cfg->max_wheel_speed_mps <= 0.0f) {
        return false;
    }
    return true;
}

IIR_KinematicsConfig IIR_Kinematics_DefaultConfig(void)
{
    /* Defaults derived from WHEELTEC C50C-V4.3 differential swing-arm spec and Tender 7.1.3.
     * 默认值对齐 WHEELTEC C50C-V4.3 摆式四轮差速规格及标书第 7.1.3 条指标。 */
    IIR_KinematicsConfig cfg = {
        .effective_half_track_m = 0.2865f,
        .wheel_diameter_m = 0.254f,
        .gear_ratio = 18.0f,
        .encoder_lines = 13u,
        .encoder_quadrature = 4u,
        .max_linear_speed_mps = 1.3f,
        .min_linear_speed_mps = 0.01f,
        .max_angular_speed_rad_s = 2.0f,
        .max_wheel_speed_mps = 1.3f,
    };
    return cfg;
}

IIR_BodyTwist IIR_Kinematics_ClampTwist(const IIR_KinematicsConfig *cfg, IIR_BodyTwist in)
{
    IIR_KinematicsConfig fallback = IIR_Kinematics_DefaultConfig();
    const IIR_KinematicsConfig *active = is_valid_config(cfg) ? cfg : &fallback;

    IIR_BodyTwist out = {
        .vx_mps = clamp_sym(in.vx_mps, active->max_linear_speed_mps),
        /* Differential chassis cannot strafe; always discard lateral velocity.
         * 差速底盘不具备横移自由度，始终将横向速度清零。 */
        .vy_mps = 0.0f,
        .wz_rad_s = clamp_sym(in.wz_rad_s, active->max_angular_speed_rad_s),
    };
    return out;
}

bool IIR_Kinematics_Inverse(const IIR_KinematicsConfig *cfg,
                            IIR_BodyTwist twist,
                            IIR_WheelSpeeds *out)
{
    if (out == NULL) {
        return false;
    }

    if (!is_valid_config(cfg)) {
        out->fl_a = 0.0f;
        out->rl_b = 0.0f;
        out->rr_c = 0.0f;
        out->fr_d = 0.0f;
        return false;
    }

    const IIR_BodyTwist clamped = IIR_Kinematics_ClampTwist(cfg, twist);
    float v_left = clamped.vx_mps - clamped.wz_rad_s * cfg->effective_half_track_m;
    float v_right = clamped.vx_mps + clamped.wz_rad_s * cfg->effective_half_track_m;

    /* Preserve turning radius by scaling left and right wheels uniformly on saturation.
     * 当单侧轮速饱和时等比例缩放左右轮速，以维持原始转弯半径不发生畸变。 */
    const float peak = fmaxf(fabsf(v_left), fabsf(v_right));
    if (peak > cfg->max_wheel_speed_mps) {
        const float scale = cfg->max_wheel_speed_mps / peak;
        v_left *= scale;
        v_right *= scale;
    }

    /* Motors A/B form the left side; Motors C/D form the right side.
     * 电机 A/B 位于车体左侧，电机 C/D 位于车体右侧。 */
    out->fl_a = v_left;
    out->rl_b = v_left;
    out->rr_c = v_right;
    out->fr_d = v_right;
    return true;
}

bool IIR_Kinematics_Forward(const IIR_KinematicsConfig *cfg,
                            IIR_WheelSpeeds wheels,
                            IIR_BodyTwist *out)
{
    if (out == NULL) {
        return false;
    }

    if (!is_valid_config(cfg) ||
        !isfinite(wheels.fl_a) || !isfinite(wheels.rl_b) ||
        !isfinite(wheels.rr_c) || !isfinite(wheels.fr_d)) {
        out->vx_mps = 0.0f;
        out->vy_mps = 0.0f;
        out->wz_rad_s = 0.0f;
        return false;
    }

    const float v_left = 0.5f * (wheels.fl_a + wheels.rl_b);
    const float v_right = 0.5f * (wheels.rr_c + wheels.fr_d);

    out->vx_mps = 0.5f * (v_left + v_right);
    out->vy_mps = 0.0f;
    out->wz_rad_s = (v_right - v_left) / (2.0f * cfg->effective_half_track_m);
    return true;
}

float IIR_Kinematics_EncoderDeltaToMps(const IIR_KinematicsConfig *cfg,
                                       int32_t delta_ticks,
                                       float dt_s)
{
    if (cfg == NULL || !isfinite(dt_s) || dt_s <= 0.0f) {
        return 0.0f;
    }
    if (!isfinite(cfg->wheel_diameter_m) || cfg->wheel_diameter_m <= 0.0f ||
        !isfinite(cfg->gear_ratio) || cfg->gear_ratio <= 0.0f ||
        cfg->encoder_lines == 0u || cfg->encoder_quadrature == 0u) {
        return 0.0f;
    }

    const float ticks_per_rev = (float)cfg->encoder_lines *
                                (float)cfg->encoder_quadrature *
                                cfg->gear_ratio;
    const float wheel_circ_m = IIR_PI_F * cfg->wheel_diameter_m;
    return ((float)delta_ticks / ticks_per_rev) * wheel_circ_m / dt_s;
}

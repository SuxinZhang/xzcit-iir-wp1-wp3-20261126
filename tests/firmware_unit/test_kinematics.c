/**
 * @file test_kinematics.c
 * @brief Host unit tests for App/kinematics/diff_kinematics.
 *        App/kinematics/diff_kinematics 模块的宿主机单元测试。
 */

#include "diff_kinematics.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Tolerance for single-precision kinematics round-trip comparisons.
 * 单精度浮点数运动学正逆解往返校验的误差容限。 */
#define EPS 1e-4f

/* Helper comparing two floats within EPS tolerance.
 * 在 EPS 容差范围内比较两个单精度浮点数的辅助函数。 */
static int approx_eq(float a, float b)
{
    return fabsf(a - b) <= EPS;
}

static void test_defaults_and_clamping(void)
{
    const IIR_KinematicsConfig cfg = IIR_Kinematics_DefaultConfig();
    assert(approx_eq(cfg.max_linear_speed_mps, 1.3f));
    assert(approx_eq(cfg.min_linear_speed_mps, 0.01f));

    /* Enforce Tender Clause 7.1.3 upper linear velocity clamp and zero lateral speed.
     * 强制执行标书第 7.1.3 条线速度上限幅，并将差速底盘横向速度清零。 */
    IIR_BodyTwist over = { .vx_mps = 2.5f, .vy_mps = 0.8f, .wz_rad_s = -5.0f };
    IIR_BodyTwist clamped = IIR_Kinematics_ClampTwist(&cfg, over);
    assert(approx_eq(clamped.vx_mps, 1.3f));
    assert(approx_eq(clamped.vy_mps, 0.0f));
    assert(approx_eq(clamped.wz_rad_s, -cfg.max_angular_speed_rad_s));

    /* Non-finite inputs (NaN / Inf) must fail-closed to zero.
     * 非有限输入（NaN / Inf）必须按故障安全原则归零。 */
    IIR_BodyTwist bad = { .vx_mps = NAN, .vy_mps = 0.0f, .wz_rad_s = INFINITY };
    IIR_BodyTwist safe = IIR_Kinematics_ClampTwist(&cfg, bad);
    assert(approx_eq(safe.vx_mps, 0.0f));
    assert(approx_eq(safe.wz_rad_s, 0.0f));
}

static void test_inverse_and_forward_roundtrip(void)
{
    const IIR_KinematicsConfig cfg = IIR_Kinematics_DefaultConfig();
    IIR_WheelSpeeds w = {0};
    IIR_BodyTwist out = {0};

    /* Straight-line drive at 0.50 m/s produces four equal wheel targets.
     * 0.50 m/s 直线行驶时四个车轮的目标线速度应完全相等。 */
    IIR_BodyTwist cmd_straight = { .vx_mps = 0.5f, .vy_mps = 0.0f, .wz_rad_s = 0.0f };
    assert(IIR_Kinematics_Inverse(&cfg, cmd_straight, &w));
    assert(approx_eq(w.fl_a, 0.5f) && approx_eq(w.rl_b, 0.5f));
    assert(approx_eq(w.rr_c, 0.5f) && approx_eq(w.fr_d, 0.5f));
    assert(IIR_Kinematics_Forward(&cfg, w, &out));
    assert(approx_eq(out.vx_mps, 0.5f) && approx_eq(out.wz_rad_s, 0.0f));

    /* Combined linear + CCW angular velocity makes right side faster than left side.
     * 前进叠加逆时针旋转时，右侧轮速应大于左侧轮速且正逆解往返一致。 */
    IIR_BodyTwist cmd_arc = { .vx_mps = 0.6f, .vy_mps = 0.0f, .wz_rad_s = 1.0f };
    assert(IIR_Kinematics_Inverse(&cfg, cmd_arc, &w));
    assert(w.rr_c > w.fl_a);
    assert(IIR_Kinematics_Forward(&cfg, w, &out));
    assert(approx_eq(out.vx_mps, 0.6f));
    assert(approx_eq(out.wz_rad_s, 1.0f));

    /* Minimum tender speed 0.01 m/s round-trip accuracy check.
     * 标书最小线速度 0.01 m/s 正逆解往返精度验证。 */
    IIR_BodyTwist cmd_min = { .vx_mps = 0.01f, .vy_mps = 0.0f, .wz_rad_s = 0.0f };
    assert(IIR_Kinematics_Inverse(&cfg, cmd_min, &w));
    assert(IIR_Kinematics_Forward(&cfg, w, &out));
    assert(approx_eq(out.vx_mps, 0.01f));
}

static void test_wheel_saturation_preserves_curvature(void)
{
    const IIR_KinematicsConfig cfg = IIR_Kinematics_DefaultConfig();
    IIR_WheelSpeeds w = {0};
    IIR_BodyTwist out = {0};

    /* Requesting vx=1.2 m/s and wz=1.5 rad/s exceeds max_wheel_speed_mps (1.3 m/s).
     * 请求 vx=1.2 m/s 与 wz=1.5 rad/s 将使单轮超过 1.3 m/s 上限，应按比例缩放以保曲率。 */
    IIR_BodyTwist cmd = { .vx_mps = 1.2f, .vy_mps = 0.0f, .wz_rad_s = 1.5f };
    assert(IIR_Kinematics_Inverse(&cfg, cmd, &w));
    assert(fabsf(w.rr_c) <= cfg.max_wheel_speed_mps + EPS);
    assert(approx_eq(w.rr_c, cfg.max_wheel_speed_mps));

    /* Verify turning curvature (wz / vx) matches the unclamped command ratio.
     * 验证限幅后实际转弯曲率 (wz / vx) 与原始指令比例保持一致。 */
    assert(IIR_Kinematics_Forward(&cfg, w, &out));
    const float expected_curvature = cmd.wz_rad_s / cmd.vx_mps;
    const float actual_curvature = out.wz_rad_s / out.vx_mps;
    assert(approx_eq(actual_curvature, expected_curvature));
}

static void test_encoder_conversion(void)
{
    const IIR_KinematicsConfig cfg = IIR_Kinematics_DefaultConfig();
    /* One full wheel revolution: 13 * 4 * 18 = 936 ticks in 1.0 s => pi * 0.254 m/s.
     * 车轮完整旋转一圈：13 * 4 * 18 = 936 脉冲 / 1.0 s 对应 pi * 0.254 m/s。 */
    const int32_t one_rev_ticks = 13 * 4 * 18;
    const float expected_mps = 3.14159265f * cfg.wheel_diameter_m;
    const float actual_mps = IIR_Kinematics_EncoderDeltaToMps(&cfg, one_rev_ticks, 1.0f);
    assert(approx_eq(actual_mps, expected_mps));

    /* Guard against zero or negative dt_s.
     * 防止时间间隔 dt_s <= 0 引发除零异常。 */
    assert(approx_eq(IIR_Kinematics_EncoderDeltaToMps(&cfg, 100, 0.0f), 0.0f));
    assert(approx_eq(IIR_Kinematics_EncoderDeltaToMps(&cfg, 100, -0.01f), 0.0f));
}

int main(void)
{
    test_defaults_and_clamping();
    test_inverse_and_forward_roundtrip();
    test_wheel_saturation_preserves_curvature();
    test_encoder_conversion();
    printf("test_kinematics: ALL PASSED\n");
    return 0;
}

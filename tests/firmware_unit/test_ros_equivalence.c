/**
 * @file test_ros_equivalence.c
 * @brief Equivalent ROS speed-to-wheel and safety-gate regression tests for the
 *        removed legacy fw_ros_drive_model/fw_ros_mode_model suites.
 *        针对已移除的 fw_ros_drive_model/fw_ros_mode_model 的 ROS 速度到四轮输出
 *        与安全门等效回归测试。
 */

#include "diff_kinematics.h"
#include "motor_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define EPS 1e-3f

static int16_t s_hw_pwm[IIR_MOTOR_CHANNEL_COUNT];

static void mock_hw_driver(const int16_t pwm[IIR_MOTOR_CHANNEL_COUNT])
{
    for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
        s_hw_pwm[i] = pwm[i];
    }
}

static void test_ros_speed_to_wheels(void)
{
    const IIR_KinematicsConfig cfg = IIR_Kinematics_DefaultConfig();

    /* 1.3 m/s upper bound from Tender Clause 7.1.3 must survive clamp and inverse kinematics.
     * 标书 7.1.3 的 1.3 m/s 上限必须同时通过限幅与逆运动学。 */
    IIR_BodyTwist over = { .vx_mps = 5.0f, .vy_mps = 0.0f, .wz_rad_s = 0.0f };
    const IIR_BodyTwist clamped = IIR_Kinematics_ClampTwist(&cfg, over);
    assert(fabsf(clamped.vx_mps - 1.3f) < EPS);
    assert(fabsf(clamped.vy_mps) < EPS);

    IIR_WheelSpeeds wheels;
    assert(IIR_Kinematics_Inverse(&cfg, clamped, &wheels));
    assert(fabsf(wheels.fl_a - 1.3f) < EPS);
    assert(fabsf(wheels.rl_b - 1.3f) < EPS);
    assert(fabsf(wheels.rr_c - 1.3f) < EPS);
    assert(fabsf(wheels.fr_d - 1.3f) < EPS);

    /* 0.01 m/s minimum controllable speed must produce non-zero wheel targets.
     * 0.01 m/s 最小可控速度必须产生非零轮速目标。 */
    IIR_BodyTwist slow = { .vx_mps = 0.01f, .vy_mps = 0.0f, .wz_rad_s = 0.0f };
    assert(IIR_Kinematics_Inverse(&cfg, slow, &wheels));
    assert(fabsf(wheels.fl_a - 0.01f) < EPS);
    assert(fabsf(wheels.fr_d - 0.01f) < EPS);
}

static void test_mode_and_motor_gate(void)
{
    Motor_RegisterOutputDriver(mock_hw_driver, 6900);
    const int16_t request[IIR_MOTOR_CHANNEL_COUNT] = {1000, -2000, 3000, -4000};

    /* safety_allow=false models PE4/PE5/500ms/undervoltage mode gate: all outputs must be zero.
     * safety_allow=false 模拟 PE4/PE5/500ms/欠压模式门：四路输出必须全为零。 */
    Motor_ApplyOutput(request, false);
    for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
        assert(s_hw_pwm[i] == 0);
    }

    /* Authorized output must be clamped and passed through the single gate.
     * 授权输出必须经唯一出口限幅并下发。 */
    const int16_t extreme[IIR_MOTOR_CHANNEL_COUNT] = {10000, -10000, 10000, -10000};
    Motor_ApplyOutput(extreme, true);
    for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
        assert(s_hw_pwm[i] == (i % 2u == 0u ? 6900 : -6900));
    }
}

int main(void)
{
    test_ros_speed_to_wheels();
    test_mode_and_motor_gate();
    printf("test_ros_equivalence: ALL PASSED\n");
    return 0;
}

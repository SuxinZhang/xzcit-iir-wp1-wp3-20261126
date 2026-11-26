/**
 * @file test_control.c
 * @brief Host unit tests for incremental PI, velocity smoother, low-speed
 *        compensation, and the unique Motor_ApplyOutput safety gate.
 *        增量式 PI、速度平滑器、低速非线性补偿及唯一电机 PWM 出口门禁单元测试。
 */

#include "motor_control.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define EPS 1e-3f

/* Capture buffer written by the mock hardware PWM driver callback.
 * 模拟硬件 PWM 驱动回调写入的捕获缓冲区与调用计数。 */
static int16_t s_mock_hw_pwm[IIR_MOTOR_CHANNEL_COUNT] = {0, 0, 0, 0};
static int s_mock_hw_calls = 0;

static void mock_hw_driver(const int16_t pwm[IIR_MOTOR_CHANNEL_COUNT])
{
    s_mock_hw_calls++;
    for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
        s_mock_hw_pwm[i] = pwm[i];
    }
}

static void test_incremental_pi(void)
{
    const IIR_PiConfig cfg = IIR_Pi_DefaultConfig();
    IIR_PiState st;
    IIR_Pi_Init(&st);

    /* Positive velocity error drives positive PWM and saturates cleanly at output_limit.
     * 正向速度误差驱动正向 PWM 输出，并在 output_limit 处平稳饱和。 */
    float pwm = 0.0f;
    for (int i = 0; i < 40; ++i) {
        pwm = IIR_Pi_Update(&cfg, &st, 1.0f, 0.0f);
    }
    assert(fabsf(pwm - cfg.output_limit) <= EPS);

    /* Reset clears accumulated output and previous error.
     * 复位操作必须清除累积输出与上一拍误差记忆。 */
    IIR_Pi_Reset(&st);
    assert(fabsf(st.output) <= EPS && fabsf(st.prev_error) <= EPS);

    /* Negative target drives negative PWM and NaN fails closed to zero.
     * 反向目标产生负向 PWM；遇到 NaN 输入时必须故障安全归零。 */
    pwm = IIR_Pi_Update(&cfg, &st, -0.5f, 0.0f);
    assert(pwm < 0.0f);
    pwm = IIR_Pi_Update(&cfg, &st, NAN, 0.0f);
    assert(fabsf(pwm) <= EPS && fabsf(st.output) <= EPS);
}

static void test_velocity_smoother(void)
{
    IIR_SmootherState sm;
    IIR_Smoother_Reset(&sm);

    /* Rate-limited stepping moves toward target by at most max_step per call.
     * 斜率限幅模式下每次步进不得超过 max_step。 */
    assert(fabsf(IIR_Smoother_Step(&sm, 1.0f, 0.25f, false) - 0.25f) <= EPS);
    assert(fabsf(IIR_Smoother_Step(&sm, 1.0f, 0.25f, false) - 0.50f) <= EPS);
    assert(fabsf(IIR_Smoother_Step(&sm, -1.0f, 0.25f, false) - 0.25f) <= EPS);

    /* Bypass mode jumps directly to target (used during precision IR docking).
     * 旁路模式直接追踪目标速度（用于红外自动回充精对接阶段）。 */
    assert(fabsf(IIR_Smoother_Step(&sm, 0.42f, 0.05f, true) - 0.42f) <= EPS);
}

static void test_low_speed_compensation(void)
{
    const IIR_LowSpeedCompConfig cfg = IIR_Control_DefaultLowSpeedConfig();

    /* Inside stop deadband must return 0.0f.
     * 处于静止死区内时输出必须严格为零。 */
    assert(fabsf(IIR_Control_CompensateLowSpeed(&cfg, 0.0f, 200.0f)) <= EPS);
    assert(fabsf(IIR_Control_CompensateLowSpeed(&cfg, 0.002f, 200.0f)) <= EPS);

    /* Ultra-low speed 0.01 m/s receives full directional stiction feedforward.
     * 0.01 m/s 极低速段应叠加完整的正向静摩擦前馈。 */
    const float pos_comp = IIR_Control_CompensateLowSpeed(&cfg, 0.01f, 100.0f);
    assert(fabsf(pos_comp - (100.0f + cfg.static_ff_pwm)) <= EPS);

    const float neg_comp = IIR_Control_CompensateLowSpeed(&cfg, -0.01f, -100.0f);
    assert(fabsf(neg_comp - (-100.0f - cfg.static_ff_pwm)) <= EPS);

    /* High cruising speed above low_speed_threshold_mps receives zero feedforward offset.
     * 高于低速补偿阈值的巡航速度段不叠加额外前馈。 */
    const float high_comp = IIR_Control_CompensateLowSpeed(&cfg, 0.8f, 2500.0f);
    assert(fabsf(high_comp - 2500.0f) <= EPS);

    /* Saturated output never exceeds output_limit.
     * 补偿叠加后仍必须受限于全局 output_limit。 */
    const float clamped = IIR_Control_CompensateLowSpeed(&cfg, 0.01f, 7100.0f);
    assert(fabsf(clamped - cfg.output_limit) <= EPS);
}

static void test_motor_apply_output_gate(void)
{
    Motor_RegisterOutputDriver(mock_hw_driver, 6900);
    s_mock_hw_calls = 0;

    const int16_t req[IIR_MOTOR_CHANNEL_COUNT] = { 1200, -8000, 9000, -450 };
    int16_t readback[IIR_MOTOR_CHANNEL_COUNT] = { 0 };

    /* Fail-Closed check: safety_allow == false must force all 4 PWM channels to 0.
     * 故障安全门控测试：safety_allow 为 false 时必须强制四路 PWM 全为零。 */
    Motor_ApplyOutput(req, false);
    assert(s_mock_hw_calls == 1);
    Motor_GetLastAppliedOutput(readback);
    for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
        assert(s_mock_hw_pwm[i] == 0);
        assert(readback[i] == 0);
    }

    /* NULL pointer check also fails closed to zero.
     * 传入空指针同样必须故障安全归零。 */
    Motor_ApplyOutput(NULL, true);
    assert(s_mock_hw_calls == 2);
    for (uint32_t i = 0u; i < IIR_MOTOR_CHANNEL_COUNT; ++i) {
        assert(s_mock_hw_pwm[i] == 0);
    }

    /* Authorized output clamps symmetrically within [-6900, +6900].
     * 授权输出时四路 PWM 必须对称限幅在 [-6900, +6900] 范围内。 */
    Motor_ApplyOutput(req, true);
    assert(s_mock_hw_calls == 3);
    assert(s_mock_hw_pwm[0] == 1200);
    assert(s_mock_hw_pwm[1] == -6900);
    assert(s_mock_hw_pwm[2] == 6900);
    assert(s_mock_hw_pwm[3] == -450);

    Motor_GetLastAppliedOutput(readback);
    assert(readback[1] == -6900 && readback[2] == 6900);
}

int main(void)
{
    test_incremental_pi();
    test_velocity_smoother();
    test_low_speed_compensation();
    test_motor_apply_output_gate();
    printf("test_control: ALL PASSED\n");
    return 0;
}

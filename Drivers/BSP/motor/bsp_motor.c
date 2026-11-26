/**
 * @file bsp_motor.c
 * @brief Implementation of C50C-V4.3 TIM8 motor PWM & direction BSP driver.
 *        C50C-V4.3 TIM8 四路电机 PWM 与方向引脚板级驱动实现。
 */

#include "motor/bsp_motor.h"

#include "main.h"
#include "motor_control.h"

#include <stddef.h>

/* Cached TIM8 handle used exclusively by the private hardware sink callback.
 * 仅由私有硬件写入回调使用的 TIM8 句柄缓存。 */
static TIM_HandleTypeDef *s_htim8 = NULL;

/* Apply direction pins and CCR duty cycle for a single motor channel.
 * 为单个电机通道设置方向引脚电平与定时器 CCR 占空比。 */
static void set_channel_output(GPIO_TypeDef *in1_port, uint16_t in1_pin,
                               GPIO_TypeDef *in2_port, uint16_t in2_pin,
                               uint32_t tim_channel,
                               int16_t signed_pwm,
                               bool invert_dir)
{
    if (signed_pwm == 0) {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(s_htim8, tim_channel, 0u);
        return;
    }

    const bool forward = (signed_pwm > 0) ^ invert_dir;
    const uint32_t mag = (signed_pwm > 0)
                             ? (uint32_t)signed_pwm
                             : (uint32_t)(-(int32_t)signed_pwm);

    HAL_GPIO_WritePin(in1_port, in1_pin, forward ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(in2_port, in2_pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(s_htim8, tim_channel, mag);
}

/* Single hardware PWM write sink registered with Motor_RegisterOutputDriver (Invariant 2).
 * 向 Motor_RegisterOutputDriver 注册的唯一硬件 PWM 写入回调（工程红线第 2 条）。 */
static void motor_hw_apply_sink(const int16_t pwm[IIR_MOTOR_CHANNEL_COUNT])
{
    if (s_htim8 == NULL || pwm == NULL) {
        return;
    }

    /* Left-side motors (A, B) are mirrored relative to right-side motors (C, D) on C50C chassis.
     * C50C 底盘左侧电机（A、B）与右侧电机（C、D）物理对称安装，故左右侧方向极性互补。
     * TIM8 channels: A=CH4 (PC9), B=CH3 (PC8), C=CH2 (PC7), D=CH1 (PC6).
     * TIM8 通道映射：A=CH4 (PC9), B=CH3 (PC8), C=CH2 (PC7), D=CH1 (PC6)。 */
    set_channel_output(IIR_AIN1_GPIO_Port, IIR_AIN1_Pin,
                       IIR_AIN2_GPIO_Port, IIR_AIN2_Pin,
                       TIM_CHANNEL_4, pwm[0], true);

    set_channel_output(IIR_BIN1_GPIO_Port, IIR_BIN1_Pin,
                       IIR_BIN2_GPIO_Port, IIR_BIN2_Pin,
                       TIM_CHANNEL_3, pwm[1], true);

    set_channel_output(IIR_CIN1_GPIO_Port, IIR_CIN1_Pin,
                       IIR_CIN2_GPIO_Port, IIR_CIN2_Pin,
                       TIM_CHANNEL_2, pwm[2], false);

    set_channel_output(IIR_DIN1_GPIO_Port, IIR_DIN1_Pin,
                       IIR_DIN2_GPIO_Port, IIR_DIN2_Pin,
                       TIM_CHANNEL_1, pwm[3], false);
}

bool BSP_Motor_Init(TIM_HandleTypeDef *htim_pwm, int16_t max_pwm)
{
    if (htim_pwm == NULL || max_pwm <= 0) {
        return false;
    }

    s_htim8 = htim_pwm;

    /* Clamp registered limit to TIM8 ARR period to avoid saturation wrap-around.
     * 将注册上限钳位到 TIM8 ARR 周期以内，防止比较值超出周期发生异常翻转。 */
    const uint32_t arr = __HAL_TIM_GET_AUTORELOAD(s_htim8);
    const int16_t bounded_max = ((uint32_t)max_pwm > arr) ? (int16_t)arr : max_pwm;

    if (HAL_TIM_PWM_Start(s_htim8, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(s_htim8, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(s_htim8, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(s_htim8, TIM_CHANNEL_4) != HAL_OK) {
        s_htim8 = NULL;
        return false;
    }

    Motor_RegisterOutputDriver(motor_hw_apply_sink, bounded_max);

    /* Ensure all four channels start safely at zero PWM (Fail-Closed).
     * 上电初始化时通过唯一出口强制将四路 PWM 清零（故障安全）。 */
    const int16_t zero_pwm[IIR_MOTOR_CHANNEL_COUNT] = {0, 0, 0, 0};
    Motor_ApplyOutput(zero_pwm, false);
    return true;
}

bool BSP_Motor_IsHwEnabled(void)
{
    /* PE4 is pulled up and reads SET when the enable switch allows motion.
     * PE4 上拉输入，使能开关处于运行档时为高电平，急停按下时拉低为低电平。 */
    return HAL_GPIO_ReadPin(IIR_EN_GPIO_Port, IIR_EN_Pin) == GPIO_PIN_SET;
}

void BSP_Motor_ReadCcr(uint16_t out_ccr[4])
{
    if (out_ccr == NULL) {
        return;
    }
    if (s_htim8 == NULL) {
        out_ccr[0] = 0u;
        out_ccr[1] = 0u;
        out_ccr[2] = 0u;
        out_ccr[3] = 0u;
        return;
    }

    out_ccr[0] = (uint16_t)(s_htim8->Instance->CCR1 & 0xFFFFu);
    out_ccr[1] = (uint16_t)(s_htim8->Instance->CCR2 & 0xFFFFu);
    out_ccr[2] = (uint16_t)(s_htim8->Instance->CCR3 & 0xFFFFu);
    out_ccr[3] = (uint16_t)(s_htim8->Instance->CCR4 & 0xFFFFu);
}

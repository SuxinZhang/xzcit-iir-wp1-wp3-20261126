/**
 * @file bsp_motor.h
 * @brief STM32 HAL motor PWM and direction BSP driver for C50C-V4.3 (TIM8 CH1..CH4).
 *        适配 C50C-V4.3 的 STM32 HAL 电机 PWM 与方向板级驱动（TIM8 CH1..CH4）。
 *
 * Registers the single hardware PWM output callback with App/control/motor_control
 * so Motor_ApplyOutput remains the sole PWM write gate (Invariant 2).
 * 向 App/control/motor_control 注册唯一底层硬件输出回调，确保 Motor_ApplyOutput 为全固件唯一 PWM 出口（红线第 2 条）。
 */

#ifndef IIR_BSP_MOTOR_H
#define IIR_BSP_MOTOR_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize TIM8 CH1..CH4 PWM outputs and register the single PWM hardware sink.
 *        初始化 TIM8 CH1..CH4 PWM 输出，并向控制层注册唯一的硬件 PWM 写入回调。
 *
 * @param[in] htim_pwm Initialized HAL TIM8 handle (Period = 16799 at 10 kHz).
 *                     已初始化的 HAL TIM8 句柄（10 kHz，Period = 16799）。
 * @param[in] max_pwm  Maximum allowed absolute PWM compare value passed to Motor_RegisterOutputDriver.
 *                     允许下发至定时器的最大 PWM 绝对值上限。
 * @return true on success; false if htim_pwm is NULL or TIM_PWM_Start fails.
 *         初始化成功返回 true；句柄为空或启动 PWM 失败则返回 false。
 */
bool BSP_Motor_Init(TIM_HandleTypeDef *htim_pwm, int16_t max_pwm);

/**
 * @brief Read hardware emergency-stop / enable switch state on PE4.
 *        读取 PE4 硬件急停/使能开关状态。
 *
 * @return true when PE4 is HIGH (run enabled); false when PE4 is LOW (E-stop active).
 *         PE4 为高电平时返回 true（允许运行）；低电平时返回 false（急停触发）。
 */
bool BSP_Motor_IsHwEnabled(void);

/**
 * @brief Read actual TIM8 CCR1..CCR4 compare register values (for telemetry frame bytes [38..45]).
 *        读取 TIM8 CCR1..CCR4 实际寄存器比较值（用于填充 50 字节遥测帧 [38..45]）。
 *
 * @param[out] out_ccr Array of 4 uint16_t receiving CCR1..CCR4 counts.
 *                     接收 TIM8 CCR1..CCR4 数值的 4 元素数组。
 */
void BSP_Motor_ReadCcr(uint16_t out_ccr[4]);

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_MOTOR_H */

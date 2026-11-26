/**
 * @file bsp_encoder.h
 * @brief STM32 HAL quadrature encoder BSP driver for TIM2/TIM3/TIM4/TIM5 (Motors A..D).
 *        适配 TIM2/TIM3/TIM4/TIM5 四轮正交编码器（电机 A..D）的 STM32 HAL 板级驱动。
 */

#ifndef IIR_BSP_ENCODER_H
#define IIR_BSP_ENCODER_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Start quadrature encoder hardware timers for motors A, B, C, and D.
 *        启动电机 A、B、C、D 对应的四路硬件正交编码器定时器。
 *
 * @param[in] htim_a TIM2 handle (PA15/PB3).
 *                   电机 A 编码器 TIM2 句柄（PA15/PB3）。
 * @param[in] htim_b TIM3 handle (PA6/PA7).
 *                   电机 B 编码器 TIM3 句柄（PA6/PA7）。
 * @param[in] htim_c TIM4 handle (PB6/PB7).
 *                   电机 C 编码器 TIM4 句柄（PB6/PB7）。
 * @param[in] htim_d TIM5 handle (PA0/PA1).
 *                   电机 D 编码器 TIM5 句柄（PA0/PA1）。
 * @return true when all four encoder timers started cleanly.
 *         全部四个编码器定时器成功启动时返回 true。
 */
bool BSP_Encoder_Init(TIM_HandleTypeDef *htim_a,
                      TIM_HandleTypeDef *htim_b,
                      TIM_HandleTypeDef *htim_c,
                      TIM_HandleTypeDef *htim_d);

/**
 * @brief Atomically read and reset signed quadrature tick deltas for motors A..D.
 *        原子读取并清零电机 A..D 在上一采样周期内的有符号正交编码器脉冲增量。
 *
 * @param[out] deltas Four signed tick deltas [A, B, C, D] (right-side polarity normalized).
 *                    输出四轮有符号脉冲增量 [A, B, C, D]（右侧已按向前为正对齐符号）。
 */
void BSP_Encoder_ReadDeltas(int32_t deltas[4]);

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_ENCODER_H */

/**
 * @file bsp_adc.h
 * @brief Non-blocking battery voltage ADC BSP driver on PB0 (ADC1_IN8) with 16-sample moving average.
 *        PB0（ADC1_IN8）非阻塞电池电压 ADC 采样驱动，带 16 点滑动均值滤波。
 */

#ifndef IIR_BSP_ADC_H
#define IIR_BSP_ADC_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize battery voltage sampler and trigger the first ADC1 conversion.
 *        初始化电池电压采样滤波器并启动首轮 ADC1 转换。
 *
 * @param[in] hadc1 Initialized HAL ADC1 handle configured for ADC_CHANNEL_8 (PB0).
 *                  已配置为通道 8（PB0）的 HAL ADC1 句柄。
 * @return true on success; false if hadc1 is NULL.
 *         初始化成功返回 true；句柄为空返回 false。
 */
bool BSP_Adc_Init(ADC_HandleTypeDef *hadc1);

/**
 * @brief Sample any completed conversion without blocking and trigger the next conversion.
 *        非阻塞采集已完成的 ADC1 转换结果，更新滑动均值并立即触发下一次采样。
 */
void BSP_Adc_Update(void);

/**
 * @brief Return filtered battery pack voltage in millivolts (11:1 divider, 3.3 V Vref).
 *        返回经过滑动平均滤波后的电池组电压，单位为毫伏（11:1 电阻分压，3.3 V 基准）。
 */
uint16_t BSP_Adc_GetBatteryMillivolts(void);

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_ADC_H */

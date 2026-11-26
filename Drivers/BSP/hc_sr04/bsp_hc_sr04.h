/**
 * @file bsp_hc_sr04.h
 * @brief Wide-voltage HC-SR04 ultrasonic BSP driver using TIM1_CH1 (PE9 ECHO)
 *        hardware input capture and PE11 TRIG output.
 *        基于 PE9（TIM1_CH1 硬件输入捕获）与 PE11（TRIG）的宽压版 HC-SR04 超声波驱动。
 */

#ifndef IIR_BSP_HC_SR04_H
#define IIR_BSP_HC_SR04_H

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Sentinel distance returned when echo times out or falls outside 20..4500 mm.
 *  当回波超时或超出 20..4500 mm 有效量程时返回的哨兵无效值（0xFFFF）。 */
#define BSP_HCSR04_INVALID_MM 0xFFFFu

/** Minimum valid ultrasonic range in millimeters (2 cm).
 *  超声波传感器最小有效测距下限，单位为毫米（20 mm）。 */
#define BSP_HCSR04_MIN_MM 20u

/** Maximum valid ultrasonic range in millimeters (4.50 m).
 *  超声波传感器最大有效测距上限，单位为毫米（4500 mm）。 */
#define BSP_HCSR04_MAX_MM 4500u

/** Hardware echo timeout window in milliseconds (covers > 4.50 m round trip).
 *  覆盖 4.50 m 满量程往返声时的硬件超时保护窗口（28 ms）。 */
#define BSP_HCSR04_TIMEOUT_MS 28u

/** Trigger repetition period in milliseconds preventing multi-echo crosstalk.
 *  防止多次回波串扰的周期性触发间隔（60 ms）。 */
#define BSP_HCSR04_TRIG_PERIOD_MS 60u

/** Window size used by the 5-point median range filter.
 *  5 点中值滤波窗口样本长度。 */
#define BSP_HCSR04_MEDIAN_LEN 5u

/**
 * @brief Convert a 1 MHz timer echo pulse width in microseconds to millimeters.
 *        将 1 MHz 定时器测得的回波脉宽（微秒）换算为有效量程内的毫米距离。
 *
 * @param[in] pulse_us Echo HIGH pulse duration in microseconds.
 *                     回波高电平持续时间，单位为微秒（us）。
 * @return Distance clamped to [20, 4500] mm, or BSP_HCSR04_INVALID_MM if invalid/timeout.
 *         有效测量返回 20..4500 mm，超出量程或超时则返回 0xFFFF。
 */
uint16_t BSP_HcSr04_PulseUsToMm(uint32_t pulse_us);

/**
 * @brief Compute a median-filtered range over up to 5 recent ultrasonic samples.
 *        对最近 5 次超声波测距窗口执行有效值筛选与中值滤波。
 *
 * @param[in] window Array of 5 recent sample values in millimeters.
 *                   包含 5 个近期毫米级测距样本的滑动窗口数组。
 * @return Filtered distance in millimeters, or BSP_HCSR04_INVALID_MM if fewer than 3 valid samples exist.
 *         若有效样本不少于 3 个则返回中位值；否则返回无效哨兵 0xFFFF。
 */
uint16_t BSP_HcSr04_MedianFilter5(const uint16_t window[BSP_HCSR04_MEDIAN_LEN]);

#ifdef USE_HAL_DRIVER
/**
 * @brief Initialize HC-SR04 TIM1_CH1 input capture (PE9) and PE11 TRIG output.
 *        初始化 HC-SR04 超声波 PE9（TIM1_CH1 输入捕获）与 PE11 触发输出。
 */
bool BSP_HcSr04_Init(TIM_HandleTypeDef *htim_echo);

/**
 * @brief Check echo timeout and emit a non-blocking 12 us trigger pulse every 60 ms.
 *        检查回波超时，并按 60 ms 周期非阻塞发出 12 us 超声波触发脉冲。
 */
void BSP_HcSr04_PeriodicTick(uint32_t now_ms);

/**
 * @brief Process TIM1_CH1 rising/falling input-capture edge events from ISR.
 *        在 TIM1 输入捕获中断回调中处理上升沿与下降沿脉宽计算。
 */
void BSP_HcSr04_OnCaptureIrq(TIM_HandleTypeDef *htim);

/**
 * @brief Return the latest median-filtered ultrasonic distance in millimeters.
 *        读取经中值滤波后的当前前置超声波距离（毫米），无效或超时返回 0xFFFF。
 */
uint16_t BSP_HcSr04_GetDistanceMm(void);
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_HC_SR04_H */

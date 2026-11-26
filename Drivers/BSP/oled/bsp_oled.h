/**
 * @file bsp_oled.h
 * @brief Lightweight 4-wire software SPI SSD1306 OLED status display BSP driver.
 *        轻量级四线软件 SPI SSD1306 OLED 状态显示屏板级驱动。
 *
 * Pin verification note (C50C-V4.3):
 * Schematic J4/U11 and WHEELTEC oled_pins_v1_2 use PD7(SCL), PD4(SDA), PE1(RST), PE0(DC),
 * whereas the resource summary table lists PE10. Verify against board silkscreen before field deploy.
 * 引脚核验说明（C50C-V4.3）：
 * 原理图 J4/U11 与出厂 oled_pins_v1_2 使用 PD7(SCL)/PD4(SDA)/PE1(RST)/PE0(DC)，资源表误标为 PE10，
 * 实车部署前请对照实物核对 RST 引脚定义。
 */

#ifndef IIR_BSP_OLED_H
#define IIR_BSP_OLED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the SSD1306 OLED controller over 4-wire software SPI.
 *        通过四线软件 SPI 初始化 SSD1306 OLED 控制器并清屏。
 */
void BSP_Oled_Init(void);

/**
 * @brief Render compact chassis battery, safety state, and forward speed onto OLED.
 *        在 OLED 屏上刷新底盘电池电压、运行使能状态与当前线速度摘要。
 *
 * @param[in] battery_mv Filtered battery pack voltage in millivolts.
 *                       滤波后的电池电压（单位 mV）。
 * @param[in] enabled    True when chassis motion safety gate is armed.
 *                       底盘安全门控允许运动时为 true。
 * @param[in] vx_mm_s    Current chassis X velocity in mm/s.
 *                       当前底盘 X 轴线速度（单位 mm/s）。
 */
void BSP_Oled_ShowStatus(uint16_t battery_mv, bool enabled, int16_t vx_mm_s);

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_OLED_H */

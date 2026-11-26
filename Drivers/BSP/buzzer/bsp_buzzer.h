/**
 * @file bsp_buzzer.h
 * @brief Buzzer GPIO BSP driver on PD11 (active-HIGH transistor driver Q2 S8050).
 *        位于 PD11 的蜂鸣器 GPIO 板级驱动（Q2 S8050 三极管高电平导通鸣响）。
 */

#ifndef IIR_BSP_BUZZER_H
#define IIR_BSP_BUZZER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enable or silence the onboard buzzer on PD11.
 *        控制 PD11 板载蜂鸣器开启或静音。
 *
 * @param[in] on true drives PD11 HIGH (beep); false pulls PD11 LOW (silent).
 *               true 输出高电平鸣响；false 拉低静音。
 */
void BSP_Buzzer_Set(bool on);

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_BUZZER_H */

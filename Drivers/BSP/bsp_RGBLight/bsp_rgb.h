/**
 * @file bsp_rgb.h
 * @brief RGB status indicator BSP driver on PB15 (R), PE6 (G), and PB14 (B).
 *        位于 PB15(R)、PE6(G)、PB14(B) 的 RGB 状态指示灯板级驱动。
 */

#ifndef IIR_BSP_RGB_H
#define IIR_BSP_RGB_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Set onboard common-anode RGB LED1 color channels.
 *        设置板载共阳极 RGB 指示灯 LED1 的三色通道开关状态。
 *
 * Note: Onboard LED1 (TC3838RGBE07) is active-LOW; external AT8236 strip drivers
 * share the same GPIO lines for status/SOS signaling.
 * 说明：板载 LED1（TC3838RGBE07）为共阳极低电平点亮；外接 AT8236 灯带驱动共用同组引脚。
 */
void BSP_Rgb_Set(bool red, bool green, bool blue);

/**
 * @brief Toggle the green status channel (PE6) for periodic task heartbeat.
 *        翻转绿色状态灯通道（PE6），用于指示控制或遥测任务心跳。
 */
void BSP_Rgb_ToggleGreen(void);

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_RGB_H */

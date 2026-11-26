/**
 * @file bsp_rgb.c
 * @brief Implementation of PB15(R)/PE6(G)/PB14(B) RGB indicator BSP driver.
 *        PB15(R)/PE6(G)/PB14(B) RGB 状态指示灯板级驱动实现。
 */

#include "bsp_RGBLight/bsp_rgb.h"

#include "main.h"

void BSP_Rgb_Set(bool red, bool green, bool blue)
{
    /* Common-anode onboard LED1 illuminates when pin is driven LOW.
     * 板载共阳极 LED1 在引脚拉低（GPIO_PIN_RESET）时点亮。 */
    HAL_GPIO_WritePin(IIR_LED_R_GPIO_Port,
                      IIR_LED_R_Pin,
                      red ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIR_LED_G_GPIO_Port,
                      IIR_LED_G_Pin,
                      green ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(IIR_LED_B_GPIO_Port,
                      IIR_LED_B_Pin,
                      blue ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void BSP_Rgb_ToggleGreen(void)
{
    HAL_GPIO_TogglePin(IIR_LED_G_GPIO_Port, IIR_LED_G_Pin);
}

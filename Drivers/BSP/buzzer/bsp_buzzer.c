/**
 * @file bsp_buzzer.c
 * @brief Implementation of PD11 buzzer GPIO BSP driver.
 *        PD11 蜂鸣器 GPIO 板级驱动实现。
 */

#include "buzzer/bsp_buzzer.h"

#include "main.h"

void BSP_Buzzer_Set(bool on)
{
    HAL_GPIO_WritePin(IIR_BUZZER_GPIO_Port,
                      IIR_BUZZER_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

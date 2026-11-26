/**
 * @file bsp_sos.c
 * @brief Implementation of PE5 EXTI5 physical SOS button driver with 20 ms debounce.
 *        带 20 ms 消抖确认的 PE5（EXTI5）物理 SOS 一键呼救按键板级驱动实现。
 */

#include "sos_button/bsp_sos.h"

#ifdef USE_HAL_DRIVER
#include "main.h"
#endif

#include <stddef.h>

static volatile bool s_exti_pending = false;
static volatile bool s_latched = false;
static bool s_press_tracking = false;
static uint32_t s_press_start_ms = 0u;

void BSP_Sos_Init(void)
{
    s_exti_pending = false;
    s_latched = false;
    s_press_tracking = false;
    s_press_start_ms = 0u;
}

void BSP_Sos_OnExtiIrq(uint16_t gpio_pin)
{
#ifdef USE_HAL_DRIVER
    if (gpio_pin == IIR_SOS_Pin) {
        s_exti_pending = true;
    }
#else
    (void)gpio_pin;
#endif
}

bool BSP_Sos_IsPressed(void)
{
#ifdef USE_HAL_DRIVER
    /* PE5 is pulled up; active button press pulls the line LOW.
     * PE5 配置为上拉输入，物理 SOS 按键按下时将引脚拉至低电平（GPIO_PIN_RESET）。 */
    return HAL_GPIO_ReadPin(IIR_SOS_GPIO_Port, IIR_SOS_Pin) == GPIO_PIN_RESET;
#else
    return false;
#endif
}

void BSP_Sos_Update(uint32_t now_ms)
{
    const bool pressed = BSP_Sos_IsPressed();

    if (pressed) {
        if (!s_press_tracking) {
            s_press_tracking = true;
            s_press_start_ms = now_ms;
        } else if ((uint32_t)(now_ms - s_press_start_ms) >= BSP_SOS_DEBOUNCE_MS) {
            s_latched = true;
            s_exti_pending = false;
        }
    } else if (s_exti_pending) {
        /* Clear transient bounce flag once button returns HIGH within debounce window.
         * 若消抖窗口内电平已回弹至高电平，则视作抖动干扰并清除挂起标志。 */
        s_exti_pending = false;
        s_press_tracking = false;
    } else {
        s_press_tracking = false;
    }
}

bool BSP_Sos_IsLatched(void)
{
    return s_latched;
}

bool BSP_Sos_ClearLatch(void)
{
    if (BSP_Sos_IsPressed()) {
        return false;
    }
    s_latched = false;
    s_exti_pending = false;
    return true;
}

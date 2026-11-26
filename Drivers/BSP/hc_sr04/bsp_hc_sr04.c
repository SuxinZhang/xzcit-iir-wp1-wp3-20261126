/**
 * @file bsp_hc_sr04.c
 * @brief Implementation of wide-voltage HC-SR04 ultrasonic BSP driver using
 *        TIM1_CH1 hardware input capture on PE9 and DWT-paced 12 us PE11 TRIG pulse.
 *        基于 PE9 TIM1_CH1 硬件输入捕获与 PE11 短脉冲触发的宽压版 HC-SR04 驱动实现。
 */

#include "hc_sr04/bsp_hc_sr04.h"

#ifdef USE_HAL_DRIVER
#include "main.h"
#endif

#include <stddef.h>

/* Sound speed round-trip bounds for 20..4500 mm (~343 m/s at 20 C).
 * 20 C 下 20..4500 mm 有效声波往返脉宽阈值（117..26240 us）。 */
#define HCSR04_MIN_PULSE_US 117u
#define HCSR04_MAX_PULSE_US 26240u
#define HCSR04_STALE_MS     300u

uint16_t BSP_HcSr04_PulseUsToMm(uint32_t pulse_us)
{
    if (pulse_us < HCSR04_MIN_PULSE_US || pulse_us > HCSR04_MAX_PULSE_US) {
        return BSP_HCSR04_INVALID_MM;
    }

    const uint32_t dist_mm = (pulse_us * 343u + 1000u) / 2000u;
    if (dist_mm < BSP_HCSR04_MIN_MM || dist_mm > BSP_HCSR04_MAX_MM) {
        return BSP_HCSR04_INVALID_MM;
    }
    return (uint16_t)dist_mm;
}

uint16_t BSP_HcSr04_MedianFilter5(const uint16_t window[BSP_HCSR04_MEDIAN_LEN])
{
    if (window == NULL) {
        return BSP_HCSR04_INVALID_MM;
    }

    uint16_t valid[BSP_HCSR04_MEDIAN_LEN];
    uint8_t count = 0u;
    for (uint8_t i = 0u; i < BSP_HCSR04_MEDIAN_LEN; ++i) {
        const uint16_t v = window[i];
        if (v != BSP_HCSR04_INVALID_MM &&
            v >= BSP_HCSR04_MIN_MM &&
            v <= BSP_HCSR04_MAX_MM) {
            valid[count++] = v;
        }
    }

    /* Require at least 3 valid echoes inside the 5-sample window to reject spikes.
     * 在 5 点滑动窗口内要求至少包含 3 个有效回波，抑制偶发杂波干扰。 */
    if (count < 3u) {
        return BSP_HCSR04_INVALID_MM;
    }

    /* Insertion sort over small fixed buffer (<= 5 elements).
     * 对不超过 5 个元素的定长数组执行插入排序。 */
    for (uint8_t i = 1u; i < count; ++i) {
        const uint16_t key = valid[i];
        int8_t j = (int8_t)(i - 1u);
        while (j >= 0 && valid[j] > key) {
            valid[j + 1] = valid[j];
            j--;
        }
        valid[j + 1] = key;
    }

    if ((count & 1u) != 0u) {
        return valid[count / 2u];
    }
    return (uint16_t)(((uint32_t)valid[count / 2u - 1u] + (uint32_t)valid[count / 2u]) / 2u);
}

#ifdef USE_HAL_DRIVER
static TIM_HandleTypeDef *s_htim1 = NULL;
static uint16_t s_window[BSP_HCSR04_MEDIAN_LEN];
static uint8_t s_win_idx = 0u;
static volatile uint16_t s_rising_ccr = 0u;
static volatile bool s_waiting_falling = false;
static volatile bool s_cycle_pending = false;
static volatile uint16_t s_latest_filtered_mm = BSP_HCSR04_INVALID_MM;
static volatile uint32_t s_last_trig_ms = 0u;
static volatile uint32_t s_last_valid_ms = 0u;

/* Record a newly acquired range sample into the 5-point circular filter window.
 * 将新测得的距离样本压入 5 点环形中值滤波窗口并刷新输出快照。 */
static void push_sample_and_update(uint16_t sample_mm, uint32_t now_ms)
{
    s_window[s_win_idx] = sample_mm;
    s_win_idx = (uint8_t)((s_win_idx + 1u) % BSP_HCSR04_MEDIAN_LEN);
    const uint16_t filtered = BSP_HcSr04_MedianFilter5(s_window);
    s_latest_filtered_mm = filtered;
    if (filtered != BSP_HCSR04_INVALID_MM) {
        s_last_valid_ms = now_ms;
    }
}

/* Emit a short 12 us trigger pulse on PE11 timed via Cortex-M4 DWT cycle counter.
 * 利用 Cortex-M4 DWT 周期计数器在 PE11 上发出精准的 12 us 触发脉冲。 */
static void trig_pulse_12us(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t cycles_12us = (SystemCoreClock / 1000000u) * 12u;
    HAL_GPIO_WritePin(IIR_HC_TRIG_GPIO_Port, IIR_HC_TRIG_Pin, GPIO_PIN_SET);
    const uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles_12us) {
        /* Brief 12 us pulse width timing; global interrupts remain enabled.
         * 仅维持 12 us 触发脉冲宽度，期间保持全局中断开启。 */
    }
    HAL_GPIO_WritePin(IIR_HC_TRIG_GPIO_Port, IIR_HC_TRIG_Pin, GPIO_PIN_RESET);
}

bool BSP_HcSr04_Init(TIM_HandleTypeDef *htim_echo)
{
    if (htim_echo == NULL) {
        return false;
    }

    s_htim1 = htim_echo;
    for (uint8_t i = 0u; i < BSP_HCSR04_MEDIAN_LEN; ++i) {
        s_window[i] = BSP_HCSR04_INVALID_MM;
    }
    s_win_idx = 0u;
    s_rising_ccr = 0u;
    s_waiting_falling = false;
    s_cycle_pending = false;
    s_latest_filtered_mm = BSP_HCSR04_INVALID_MM;
    s_last_trig_ms = 0u;
    s_last_valid_ms = 0u;

    HAL_GPIO_WritePin(IIR_HC_TRIG_GPIO_Port, IIR_HC_TRIG_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_CAPTUREPOLARITY(s_htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
    return HAL_TIM_IC_Start_IT(s_htim1, TIM_CHANNEL_1) == HAL_OK;
}

void BSP_HcSr04_PeriodicTick(uint32_t now_ms)
{
    if (s_htim1 == NULL) {
        return;
    }

    /* Enforce 28 ms hardware capture timeout to reset stuck polarity and push 0xFFFF.
     * 执行 28 ms 硬件捕获超时保护，及时复位极性状态并写入 0xFFFF 无效标志。 */
    if (s_cycle_pending && (uint32_t)(now_ms - s_last_trig_ms) >= BSP_HCSR04_TIMEOUT_MS) {
        __disable_irq();
        s_waiting_falling = false;
        s_cycle_pending = false;
        __HAL_TIM_SET_CAPTUREPOLARITY(s_htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        __enable_irq();
        push_sample_and_update(BSP_HCSR04_INVALID_MM, now_ms);
    }

    if ((uint32_t)(now_ms - s_last_trig_ms) >= BSP_HCSR04_TRIG_PERIOD_MS) {
        __disable_irq();
        s_last_trig_ms = now_ms;
        s_waiting_falling = false;
        s_cycle_pending = true;
        __HAL_TIM_SET_CAPTUREPOLARITY(s_htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
        __enable_irq();

        trig_pulse_12us();
    }
}

void BSP_HcSr04_OnCaptureIrq(TIM_HandleTypeDef *htim)
{
    if (s_htim1 == NULL || htim != s_htim1 || htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) {
        return;
    }

    const uint16_t captured = (uint16_t)(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1) & 0xFFFFu);
    if (!s_waiting_falling) {
        s_rising_ccr = captured;
        s_waiting_falling = true;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
        return;
    }

    /* Unsigned 16-bit subtraction accurately handles single counter wrap-around (65536 us).
     * 无符号 16 位减法可精确处理 65536 us 定时器单次溢出回绕。 */
    const uint16_t pulse_us = (uint16_t)(captured - s_rising_ccr);
    s_waiting_falling = false;
    s_cycle_pending = false;
    __HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);

    const uint16_t sample_mm = BSP_HcSr04_PulseUsToMm(pulse_us);
    push_sample_and_update(sample_mm, HAL_GetTick());
}

uint16_t BSP_HcSr04_GetDistanceMm(void)
{
    const uint32_t now_ms = HAL_GetTick();
    if (s_latest_filtered_mm == BSP_HCSR04_INVALID_MM) {
        return BSP_HCSR04_INVALID_MM;
    }
    if ((uint32_t)(now_ms - s_last_valid_ms) > HCSR04_STALE_MS) {
        return BSP_HCSR04_INVALID_MM;
    }
    return s_latest_filtered_mm;
}
#endif /* USE_HAL_DRIVER */

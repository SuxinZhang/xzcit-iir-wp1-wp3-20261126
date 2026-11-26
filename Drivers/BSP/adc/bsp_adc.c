/**
 * @file bsp_adc.c
 * @brief Implementation of non-blocking PB0 battery voltage ADC1 BSP driver.
 *        PB0 电池电压非阻塞 ADC1 板级驱动实现。
 */

#include "adc/bsp_adc.h"

#include <stddef.h>

#define BAT_FILTER_LEN 16u

static ADC_HandleTypeDef *s_hadc1 = NULL;
static uint16_t s_samples[BAT_FILTER_LEN];
static uint8_t s_idx = 0u;
static uint8_t s_count = 0u;
static uint32_t s_sum = 0u;
static uint16_t s_battery_mv = 24000u;

bool BSP_Adc_Init(ADC_HandleTypeDef *hadc1)
{
    if (hadc1 == NULL) {
        return false;
    }

    s_hadc1 = hadc1;
    s_idx = 0u;
    s_count = 0u;
    s_sum = 0u;
    for (uint8_t i = 0u; i < BAT_FILTER_LEN; ++i) {
        s_samples[i] = 0u;
    }

    (void)HAL_ADC_Start(s_hadc1);
    return true;
}

void BSP_Adc_Update(void)
{
    if (s_hadc1 == NULL) {
        return;
    }

    if (__HAL_ADC_GET_FLAG(s_hadc1, ADC_FLAG_EOC) != RESET) {
        const uint16_t raw = (uint16_t)(HAL_ADC_GetValue(s_hadc1) & 0x0FFFu);

        s_sum -= s_samples[s_idx];
        s_samples[s_idx] = raw;
        s_sum += raw;
        s_idx = (uint8_t)((s_idx + 1u) % BAT_FILTER_LEN);
        if (s_count < BAT_FILTER_LEN) {
            s_count++;
        }

        const uint32_t avg_raw = s_sum / (uint32_t)s_count;
        /* Schematic divider R9=100k / R38=10k gives 11:1 ratio with 3300 mV 12-bit Vref.
         * 根据原理图 R9=100k / R38=10k 的 11:1 分压及 3300 mV 12 位基准换算电池毫伏数。 */
        const uint32_t mv = (avg_raw * 3300u * 11u) / 4096u;
        s_battery_mv = (mv > 65535u) ? 65535u : (uint16_t)mv;

        (void)HAL_ADC_Start(s_hadc1);
    }
}

uint16_t BSP_Adc_GetBatteryMillivolts(void)
{
    return s_battery_mv;
}

/**
 * @file bsp_encoder.c
 * @brief Implementation of TIM2/TIM3/TIM4/TIM5 quadrature encoder reader.
 *        TIM2/TIM3/TIM4/TIM5 四倍频正交编码器增量读取实现。
 */

#include "encoder/bsp_encoder.h"

#include <stddef.h>

static TIM_HandleTypeDef *s_enc_timers[4] = {NULL, NULL, NULL, NULL};

bool BSP_Encoder_Init(TIM_HandleTypeDef *htim_a,
                      TIM_HandleTypeDef *htim_b,
                      TIM_HandleTypeDef *htim_c,
                      TIM_HandleTypeDef *htim_d)
{
    if (htim_a == NULL || htim_b == NULL || htim_c == NULL || htim_d == NULL) {
        return false;
    }

    s_enc_timers[0] = htim_a;
    s_enc_timers[1] = htim_b;
    s_enc_timers[2] = htim_c;
    s_enc_timers[3] = htim_d;

    for (uint32_t i = 0u; i < 4u; ++i) {
        __HAL_TIM_SET_COUNTER(s_enc_timers[i], 0u);
        if (HAL_TIM_Encoder_Start(s_enc_timers[i], TIM_CHANNEL_ALL) != HAL_OK) {
            return false;
        }
    }
    return true;
}

void BSP_Encoder_ReadDeltas(int32_t deltas[4])
{
    if (deltas == NULL) {
        return;
    }

    for (uint32_t i = 0u; i < 4u; ++i) {
        if (s_enc_timers[i] == NULL) {
            deltas[i] = 0;
            continue;
        }
        /* Cast low 16 bits to signed int16_t so 16-bit and 32-bit counters handle reverse wrap uniformly.
         * 将计数器低 16 位转为有符号 int16_t，使 16 位与 32 位定时器反转计数均统一回绕。 */
        const uint16_t raw = (uint16_t)(__HAL_TIM_GET_COUNTER(s_enc_timers[i]) & 0xFFFFu);
        __HAL_TIM_SET_COUNTER(s_enc_timers[i], 0u);
        int32_t signed_delta = (int32_t)(int16_t)raw;

        /* Right-side motors C/D are mounted mirrored on C50C; invert sign so forward motion is positive.
         * C50C 底盘右侧电机 C/D 为镜像安装，对其读数取反以统一定义车体向前转动为正。 */
        if (i >= 2u) {
            signed_delta = -signed_delta;
        }
        deltas[i] = signed_delta;
    }
}

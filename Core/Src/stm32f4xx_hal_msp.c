/**
 * @file stm32f4xx_hal_msp.c
 * @brief HAL MSP low-level clock, GPIO alternate-function, DMA, and NVIC setup
 *        aligned strictly to the C50C-V4.3 schematic.
 *        严格对齐 C50C-V4.3 原理图的 HAL MSP 底层时钟、复用引脚、DMA 与中断初始化。
 */

#include "main.h"

void HAL_MspInit(void)
{
    /* Enable SYSCFG and PWR peripheral clocks; set NVIC Group 4 for FreeRTOS.
     * 使能 SYSCFG 与 PWR 时钟，并配置 FreeRTOS 所需的 NVIC 优先级分组 4。 */
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
    GPIO_InitTypeDef gpio = {0};
    if (hadc->Instance == ADC1) {
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* Configure PB0 as ADC1_IN8 analog battery voltage divider input.
         * 将 PB0 配置为 ADC1_IN8 模拟输入，用于采集电池分压电压。 */
        gpio.Pin = GPIO_PIN_0;
        gpio.Mode = GPIO_MODE_ANALOG;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &gpio);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};
    if (htim->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* Configure PC6..PC9 as TIM8 CH1..CH4 PWM outputs (Motors D, C, B, A).
         * 将 PC6..PC9 配置为 TIM8 CH1..CH4 复用 PWM 输出（对应电机 D、C、B、A）。 */
        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLDOWN;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &gpio);
    }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};
    if (htim->Instance == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();

        /* Configure PE9 as TIM1_CH1 AF1 input capture for HC-SR04 ultrasonic ECHO.
         * 将 PE9 配置为 TIM1_CH1（AF1）硬件输入捕获，用于测绘 HC-SR04 超声波回波脉宽。 */
        gpio.Pin = IIR_HC_ECHO_Pin;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLDOWN;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(IIR_HC_ECHO_GPIO_Port, &gpio);

        HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};

    if (htim->Instance == TIM2) {
        /* Motor A quadrature encoder: PA15 (TIM2_CH1) and PB3 (TIM2_CH2).
         * 电机 A 正交编码器：PA15（TIM2_CH1）与 PB3（TIM2_CH2）。 */
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF1_TIM2;

        gpio.Pin = GPIO_PIN_15;
        HAL_GPIO_Init(GPIOA, &gpio);
        gpio.Pin = GPIO_PIN_3;
        HAL_GPIO_Init(GPIOB, &gpio);
    } else if (htim->Instance == TIM3) {
        /* Motor B quadrature encoder: PA6 (TIM3_CH1) and PA7 (TIM3_CH2).
         * 电机 B 正交编码器：PA6（TIM3_CH1）与 PA7（TIM3_CH2）。 */
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOA, &gpio);
    } else if (htim->Instance == TIM4) {
        /* Motor C quadrature encoder: PB6 (TIM4_CH1) and PB7 (TIM4_CH2).
         * 电机 C 正交编码器：PB6（TIM4_CH1）与 PB7（TIM4_CH2）。 */
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &gpio);
    } else if (htim->Instance == TIM5) {
        /* Motor D quadrature encoder: PA0 (TIM5_CH1) and PA1 (TIM5_CH2).
         * 电机 D 正交编码器：PA0（TIM5_CH1）与 PA1（TIM5_CH2）。 */
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_HIGH;
        gpio.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOA, &gpio);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio = {0};

    if (huart->Instance == USART3) {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        /* Configure PB10 (USART3_TX) and PB11 (USART3_RX) connected to CH9102F Type-C.
         * 配置连接至 CH9102F Type-C 桥接芯片的 PB10（TX）与 PB11（RX）。 */
        gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &gpio);

        /* Configure circular DMA1 Stream 1 Channel 4 for non-blocking USART3 RX + IDLE.
         * 配置循环模式的 DMA1 Stream 1 通道 4，用于非阻塞 USART3 DMA+IDLE 接收。 */
        hdma_usart3_rx.Instance = DMA1_Stream1;
        hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

        /* Configure normal DMA1 Stream 3 Channel 4 for non-blocking USART3 TX.
         * 配置普通模式的 DMA1 Stream 3 通道 4，用于非阻塞 USART3 遥测发送。 */
        hdma_usart3_tx.Instance = DMA1_Stream3;
        hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_tx.Init.Mode = DMA_NORMAL;
        hdma_usart3_tx.Init.Priority = DMA_PRIORITY_MEDIUM;
        hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

        /* Priority 5 matches configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
         * 中断抢占优先级设为 5，满足 FreeRTOS 系统可屏蔽最高中断优先级约束。 */
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    } else if (huart->Instance == USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* Configure PD5 (USART2_TX) and PD6 (USART2_RX) wired to SP485EEN for D50A telemetry.
         * 配置连接至 SP485EEN 芯片的 PD5（TX）与 PD6（RX），用于 D50A 驱动器监控。 */
        gpio.Pin = IIR_D50A_TX_Pin | IIR_D50A_RX_Pin;
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pull = GPIO_PULLUP;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(IIR_D50A_TX_GPIO_Port, &gpio);

        HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}

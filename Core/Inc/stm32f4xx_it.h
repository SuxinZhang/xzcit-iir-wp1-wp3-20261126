/**
 * @file stm32f4xx_it.h
 * @brief Cortex-M4 exception and peripheral interrupt handler declarations.
 *        Cortex-M4 系统异常与外设中断服务函数声明头文件。
 */

#ifndef STM32F4XX_IT_H
#define STM32F4XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
void SysTick_Handler(void);
void EXTI3_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream3_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_IT_H */

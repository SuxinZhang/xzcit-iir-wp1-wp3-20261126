/**
 * @file stm32f4xx_it.c
 * @brief Interrupt service routines for Cortex-M4 core, SysTick/FreeRTOS, and USART3 DMA.
 *        Cortex-M4 内核异常、SysTick/FreeRTOS 时基及 USART3 DMA 收发中断服务实现。
 */

#include "stm32f4xx_it.h"
#include "main.h"

#include "can_auto_dock/bsp_can_dock.h"
#include "d50a_rs485/bsp_d50a.h"

#include "FreeRTOS.h"
#include "task.h"

/* Declared by FreeRTOS GCC ARM_CM4F port.c.
 * 由 FreeRTOS GCC ARM_CM4F port.c 提供的系统节拍中断处理函数。 */
extern void xPortSysTickHandler(void);

void NMI_Handler(void)
{
    /* Halt on non-maskable interrupt fault.
     * 发生不可屏蔽中断异常时停驻等待调试。 */
    while (1) {
    }
}

void HardFault_Handler(void)
{
    /* Halt safely on hard fault (PWM stays off when motors are not driven).
     * 发生硬件错误异常时安全停驻死循环。 */
    while (1) {
    }
}

void MemManage_Handler(void)
{
    while (1) {
    }
}

void BusFault_Handler(void)
{
    while (1) {
    }
}

void UsageFault_Handler(void)
{
    while (1) {
    }
}

void DebugMon_Handler(void)
{
}

void SysTick_Handler(void)
{
    /* Advance STM32 HAL millisecond tick required by HAL timeouts.
     * 驱动 STM32 HAL 毫秒时基，供各外设超时逻辑使用。 */
    HAL_IncTick();

    /* Forward 1 kHz tick to FreeRTOS only after scheduler startup.
     * 仅在 FreeRTOS 调度器启动后向内核转发 1 kHz 时钟节拍。 */
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void DMA1_Stream1_IRQHandler(void)
{
    /* Handle USART3 RX DMA interrupts (Transfer Complete / Error).
     * 处理 USART3 接收 DMA（DMA1 Stream 1）中断。 */
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

void DMA1_Stream3_IRQHandler(void)
{
    /* Handle USART3 TX DMA interrupts (Transfer Complete / Error).
     * 处理 USART3 发送 DMA（DMA1 Stream 3）中断。 */
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

void USART3_IRQHandler(void)
{
    /* Handle USART3 IDLE line detection and TX completion callbacks.
     * 处理 USART3 空闲（IDLE）检测与非阻塞发送完成回调。 */
    HAL_UART_IRQHandler(&huart3);
}

void USART2_IRQHandler(void)
{
    /* Handle USART2 RS485 D50A reply byte reception and error recovery.
     * 处理 USART2 RS485 接收中断并将 D50A 返回字节喂入帧状态机。 */
    BSP_D50A_OnUartIrq(&huart2);
}

void TIM1_CC_IRQHandler(void)
{
    /* Handle TIM1_CH1 (PE9) input capture edges for HC-SR04 echo timing.
     * 处理 PE9（TIM1_CH1）超声波回波边沿输入捕获中断。 */
    HAL_TIM_IRQHandler(&htim1);
}

void CAN1_RX0_IRQHandler(void)
{
    /* Process CAN1 FIFO0 auto-docking infrared & charger frames.
     * 读取 CAN1 FIFO0 邮箱中的红外回充状态与充电电流报文。 */
    BSP_CanDock_OnRxIrq();
}

void EXTI3_IRQHandler(void)
{
    /* Handle PD3 ICM-20948 data-ready interrupt flag.
     * 响应 PD3 引脚上的 ICM-20948 数据就绪外部中断。 */
    HAL_GPIO_EXTI_IRQHandler(IIR_IMU_INT_Pin);
}

void EXTI9_5_IRQHandler(void)
{
    /* Handle PE5 physical SOS button falling-edge external interrupt.
     * 响应 PE5 物理一键呼救（SOS）按钮下降沿外部中断。 */
    HAL_GPIO_EXTI_IRQHandler(IIR_SOS_Pin);
}

/**
 * @file bsp_uartx.h
 * @brief Non-blocking USART3 DMA + IDLE RX and DMA TX BSP driver for host communications.
 *        用于上位机通信的非阻塞 USART3 DMA + 空闲中断（IDLE）收发板级驱动。
 */

#ifndef IIR_BSP_UARTX_H
#define IIR_BSP_UARTX_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize USART3 circular DMA + IDLE reception and non-blocking DMA transmission.
 *        初始化 USART3 循环 DMA + 空闲中断（IDLE）接收及非阻塞 DMA 发送通道。
 *
 * @param[in] huart3 Initialized HAL UART handle for USART3 (PB10/PB11).
 *                   已初始化的 USART3 HAL 句柄（PB10/PB11）。
 * @return true on success; false on NULL handle or HAL_UARTEx_ReceiveToIdle_DMA error.
 *         初始化成功返回 true；句柄为空或启动 DMA 接收失败返回 false。
 */
bool BSP_Uart3_Init(UART_HandleTypeDef *huart3);

/**
 * @brief Pop one received byte from the lock-free RX ring buffer (called by ControlTask).
 *        从无锁接收环形缓冲区中取出一个已收字节（由 ControlTask 调用）。
 *
 * @param[out] out_byte Destination pointer for the read byte.
 *                      接收读出字节的目标指针。
 * @return true if a byte was available; false when the ring buffer is empty.
 *         成功读到字节返回 true；缓冲区为空返回 false。
 */
bool BSP_Uart3_ReadByte(uint8_t *out_byte);

/**
 * @brief Queue a non-blocking DMA transmission on USART3 (drops cleanly if DMA is busy).
 *        发起一次非阻塞 USART3 DMA 发送（若上一次 DMA 仍在发送则安全跳过，绝不阻塞任务）。
 *
 * @param[in] data Payload bytes to transmit.
 *                 待发送的数据缓冲指针。
 * @param[in] len  Number of bytes to transmit (must fit static TX buffer <= 128 bytes).
 *                 待发送字节数（需不超过静态发送缓存容量 128 字节）。
 * @return true if DMA transfer started; false if busy, invalid args, or HAL error.
 *         成功启动 DMA 传输返回 true；忙状态、参数越界或 HAL 返回异常时返回 false。
 */
bool BSP_Uart3_SendAsync(const uint8_t *data, uint16_t len);

/**
 * @brief HAL RxEvent callback hook invoked on USART3 IDLE, Half-Transfer, or Full-Transfer.
 *        在 USART3 触发 IDLE、半传输或全传输事件时由 HAL_UARTEx_RxEventCallback 调用的钩子函数。
 */
void BSP_Uart3_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size);

/**
 * @brief HAL TxCplt callback hook releasing USART3 DMA TX ownership.
 *        在 USART3 DMA 发送完成后由 HAL_UART_TxCpltCallback 调用的释放钩子函数。
 */
void BSP_Uart3_OnTxCplt(UART_HandleTypeDef *huart);

/**
 * @brief HAL Error callback hook clearing USART3 UART errors and restarting DMA+IDLE RX.
 *        在 USART3 发生溢出/帧错误时清除状态并自动重启 DMA+IDLE 接收的错误恢复钩子。
 */
void BSP_Uart3_OnError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_UARTX_H */

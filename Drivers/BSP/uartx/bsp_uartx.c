/**
 * @file bsp_uartx.c
 * @brief Implementation of non-blocking USART3 DMA + IDLE ring-buffered host link.
 *        基于 DMA + 空闲中断（IDLE）与环形缓冲区的非阻塞 USART3 上位机通信驱动实现。
 */

#include "uartx/bsp_uartx.h"

#include <stddef.h>
#include <string.h>

#define UART3_RX_DMA_SIZE  128u
#define UART3_RX_RING_SIZE 512u
#define UART3_TX_DMA_SIZE  128u

static UART_HandleTypeDef *s_huart3 = NULL;

/* Circular DMA reception buffer and SPSC software ring buffer.
 * 循环模式 DMA 接收缓存及单生产者单消费者软件环形缓冲区。 */
static uint8_t s_rx_dma_buf[UART3_RX_DMA_SIZE];
static volatile uint16_t s_rx_dma_old_pos = 0u;

static uint8_t s_rx_ring[UART3_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0u;
static volatile uint16_t s_rx_tail = 0u;

/* Static DMA transmission buffer and busy guard preventing overlapping transfers.
 * 静态 DMA 发送缓冲区及防重入忙标志。 */
static uint8_t s_tx_dma_buf[UART3_TX_DMA_SIZE];
static volatile bool s_tx_busy = false;

/* Push a contiguous slice into the SPSC ring buffer (drops oldest excess safely if full).
 * 向单生产者单消费者环形缓冲区写入一段连续字节（满时安全丢弃溢出字节避免阻塞中断）。 */
static void ring_push_slice(const uint8_t *data, uint16_t count)
{
    uint16_t head = s_rx_head;
    const uint16_t tail = s_rx_tail;

    for (uint16_t i = 0u; i < count; ++i) {
        const uint16_t next = (uint16_t)((head + 1u) % UART3_RX_RING_SIZE);
        if (next == tail) {
            break;
        }
        s_rx_ring[head] = data[i];
        head = next;
    }
    s_rx_head = head;
}

bool BSP_Uart3_Init(UART_HandleTypeDef *huart3)
{
    if (huart3 == NULL) {
        return false;
    }

    s_huart3 = huart3;
    s_rx_dma_old_pos = 0u;
    s_rx_head = 0u;
    s_rx_tail = 0u;
    s_tx_busy = false;

    return HAL_UARTEx_ReceiveToIdle_DMA(s_huart3, s_rx_dma_buf, UART3_RX_DMA_SIZE) == HAL_OK;
}

bool BSP_Uart3_ReadByte(uint8_t *out_byte)
{
    if (out_byte == NULL) {
        return false;
    }

    const uint16_t tail = s_rx_tail;
    if (tail == s_rx_head) {
        return false;
    }

    *out_byte = s_rx_ring[tail];
    s_rx_tail = (uint16_t)((tail + 1u) % UART3_RX_RING_SIZE);
    return true;
}

bool BSP_Uart3_SendAsync(const uint8_t *data, uint16_t len)
{
    if (s_huart3 == NULL || data == NULL || len == 0u || len > UART3_TX_DMA_SIZE) {
        return false;
    }

    /* Briefly guard s_tx_busy check-and-set against ISR completion race.
     * 短暂临界保护 s_tx_busy 判忙与置位，避免与发送完成中断产生竞态。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_tx_busy) {
        __set_PRIMASK(primask);
        return false;
    }
    s_tx_busy = true;
    __set_PRIMASK(primask);

    memcpy(s_tx_dma_buf, data, len);
    if (HAL_UART_Transmit_DMA(s_huart3, s_tx_dma_buf, len) != HAL_OK) {
        s_tx_busy = false;
        return false;
    }
    return true;
}

void BSP_Uart3_OnRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart != s_huart3 || size > UART3_RX_DMA_SIZE) {
        return;
    }

    const uint16_t old_pos = s_rx_dma_old_pos;
    if (size != old_pos) {
        if (size > old_pos) {
            ring_push_slice(&s_rx_dma_buf[old_pos], (uint16_t)(size - old_pos));
        } else {
            /* Handle circular DMA wrap-around from tail of buffer back to index 0.
             * 处理循环 DMA 缓冲区从尾部回绕至起始位置 0 的分段拷贝。 */
            ring_push_slice(&s_rx_dma_buf[old_pos], (uint16_t)(UART3_RX_DMA_SIZE - old_pos));
            if (size > 0u) {
                ring_push_slice(&s_rx_dma_buf[0], size);
            }
        }
        s_rx_dma_old_pos = (size == UART3_RX_DMA_SIZE) ? 0u : size;
    }
}

void BSP_Uart3_OnTxCplt(UART_HandleTypeDef *huart)
{
    if (huart == s_huart3) {
        s_tx_busy = false;
    }
}

void BSP_Uart3_OnError(UART_HandleTypeDef *huart)
{
    if (huart != s_huart3) {
        return;
    }

    s_tx_busy = false;
    s_rx_dma_old_pos = 0u;
    (void)HAL_UART_AbortReceive(s_huart3);
    (void)HAL_UARTEx_ReceiveToIdle_DMA(s_huart3, s_rx_dma_buf, UART3_RX_DMA_SIZE);
}

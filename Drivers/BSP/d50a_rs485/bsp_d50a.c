/**
 * @file bsp_d50a.c
 * @brief Implementation of D50A RS485 motor driver telemetry on USART2 (PD5/PD6).
 *        基于 USART2（PD5/PD6）的 D50A RS485 电机驱动器健康监控与协议解析实现。
 */

#include "d50a_rs485/bsp_d50a.h"

#ifdef USE_HAL_DRIVER
#include "main.h"
#endif

#include <stddef.h>
#include <string.h>

#define D50A_HEAD_BYTE 0xAAu
#define D50A_TAIL_BYTE 0xBBu
#define D50A_ONLINE_TIMEOUT_MS 500u

void BSP_D50A_BuildFrame(uint8_t out_frame[BSP_D50A_FRAME_LEN],
                         uint8_t addr,
                         uint8_t func,
                         uint8_t d0,
                         uint8_t d1,
                         uint8_t d2,
                         uint8_t d3)
{
    if (out_frame == NULL) {
        return;
    }

    out_frame[0] = D50A_HEAD_BYTE;
    out_frame[1] = D50A_HEAD_BYTE;
    out_frame[2] = D50A_HEAD_BYTE;
    out_frame[3] = addr;
    out_frame[4] = func;
    out_frame[5] = d0;
    out_frame[6] = d1;
    out_frame[7] = d2;
    out_frame[8] = d3;
    out_frame[9] = D50A_TAIL_BYTE;
    out_frame[10] = D50A_TAIL_BYTE;
    out_frame[11] = D50A_TAIL_BYTE;
    out_frame[12] = D50A_TAIL_BYTE;
}

uint8_t BSP_D50A_ConvertRawAlarmToFaultMask(uint8_t raw_alarm, bool online)
{
    /* In D50A protocol bits 0..3 are 1=OK and 0=FAULT; invert them to active-HIGH faults.
     * D50A 协议原始报警位（bit0..3）为 1 正常、0 故障；此处反相转为高电平有效故障位。 */
    uint8_t mask = (uint8_t)((uint8_t)(~raw_alarm) & 0x0Fu);
    if (!online) {
        mask |= (uint8_t)BSP_D50A_FAULT_OFFLINE;
    }
    return mask;
}

bool BSP_D50A_FeedRxByte(BSP_D50A_RxParser *parser,
                         uint8_t byte,
                         uint8_t out_frame[BSP_D50A_FRAME_LEN])
{
    if (parser == NULL || out_frame == NULL) {
        return false;
    }

    if (parser->idx < 3u) {
        if (byte == D50A_HEAD_BYTE) {
            parser->buf[parser->idx++] = byte;
        } else {
            parser->idx = 0u;
        }
        return false;
    }

    parser->buf[parser->idx++] = byte;
    if (parser->idx < BSP_D50A_FRAME_LEN) {
        return false;
    }

    parser->idx = 0u;
    if (parser->buf[9] == D50A_TAIL_BYTE &&
        parser->buf[10] == D50A_TAIL_BYTE &&
        parser->buf[11] == D50A_TAIL_BYTE &&
        parser->buf[12] == D50A_TAIL_BYTE) {
        memcpy(out_frame, parser->buf, BSP_D50A_FRAME_LEN);
        return true;
    }
    return false;
}

bool BSP_D50A_ParseResponse(const uint8_t frame[BSP_D50A_FRAME_LEN],
                            uint8_t expected_addr,
                            bool is_status_query,
                            BSP_D50A_Telemetry *io_telemetry)
{
    if (frame == NULL || io_telemetry == NULL) {
        return false;
    }

    if (frame[0] != D50A_HEAD_BYTE || frame[1] != D50A_HEAD_BYTE || frame[2] != D50A_HEAD_BYTE ||
        frame[9] != D50A_TAIL_BYTE || frame[10] != D50A_TAIL_BYTE ||
        frame[11] != D50A_TAIL_BYTE || frame[12] != D50A_TAIL_BYTE) {
        return false;
    }

    /* Exception frame returned by D50A on unsupported commands: AA AA AA AA FF FF FF FF FF BB..BB.
     * D50A 返回异常帧 AA FF FF FF FF FF 时直接丢弃并不覆盖有效遥测。 */
    if (frame[3] == D50A_HEAD_BYTE && frame[4] == 0xFFu) {
        return false;
    }

    if (frame[3] != expected_addr) {
        return false;
    }

    if (is_status_query) {
        /* 0xA0 response replaces byte[4] directly with the raw alarm status byte.
         * 0xA0 状态应答帧第 4 字节为原始报警码，第 5..8 字节为 M1/M2 电流（0.01 A）。 */
        const int16_t raw_m1 = (int16_t)(((uint16_t)frame[5] << 8u) | (uint16_t)frame[6]);
        const int16_t raw_m2 = (int16_t)(((uint16_t)frame[7] << 8u) | (uint16_t)frame[8]);
        io_telemetry->online = true;
        io_telemetry->raw_alarm = frame[4];
        io_telemetry->fault_mask = BSP_D50A_ConvertRawAlarmToFaultMask(frame[4], true);
        io_telemetry->m1_current_ma = (int16_t)((int32_t)raw_m1 * 10);
        io_telemetry->m2_current_ma = (int16_t)((int32_t)raw_m2 * 10);
        return true;
    }

    if (frame[4] != BSP_D50A_FUNC_READ_TEMP) {
        return false;
    }

    io_telemetry->online = true;
    io_telemetry->m1_temp_c = frame[6];
    io_telemetry->m2_temp_c = frame[8];
    return true;
}

#ifdef USE_HAL_DRIVER
static UART_HandleTypeDef *s_huart2 = NULL;
static BSP_D50A_RxParser s_parser;
static volatile bool s_frame_ready = false;
static uint8_t s_isr_frame[BSP_D50A_FRAME_LEN];
static BSP_D50A_Telemetry s_telemetry;
static bool s_pending_is_status = true;
static uint8_t s_poll_step = 0u;
static uint32_t s_last_rx_ms = 0u;

bool BSP_D50A_Init(UART_HandleTypeDef *huart_rs485)
{
    if (huart_rs485 == NULL) {
        return false;
    }

    s_huart2 = huart_rs485;
    memset(&s_parser, 0, sizeof(s_parser));
    memset(&s_telemetry, 0, sizeof(s_telemetry));
    s_telemetry.raw_alarm = 0x0Fu;
    s_telemetry.fault_mask = (uint8_t)BSP_D50A_FAULT_OFFLINE;
    s_frame_ready = false;
    s_pending_is_status = true;
    s_poll_step = 0u;
    s_last_rx_ms = 0u;

    /* Enable RXNE interrupt on USART2 so incoming bytes feed the ISR frame state machine.
     * 使能 USART2 接收非空中断（RXNE），在中断中将字节流喂入 D50A 收帧状态机。 */
    __HAL_UART_ENABLE_IT(s_huart2, UART_IT_RXNE);
    return true;
}

void BSP_D50A_OnRxByte(uint8_t byte)
{
    uint8_t complete[BSP_D50A_FRAME_LEN];
    if (BSP_D50A_FeedRxByte(&s_parser, byte, complete)) {
        memcpy(s_isr_frame, complete, BSP_D50A_FRAME_LEN);
        s_frame_ready = true;
    }
}

void BSP_D50A_OnUartIrq(UART_HandleTypeDef *huart)
{
    if (s_huart2 == NULL || huart != s_huart2) {
        return;
    }

    const uint32_t sr = huart->Instance->SR;
    if ((sr & USART_SR_RXNE) != 0u) {
        const uint8_t b = (uint8_t)(huart->Instance->DR & 0xFFu);
        BSP_D50A_OnRxByte(b);
        return;
    }

    if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0u) {
        (void)huart->Instance->DR;
    }
}

void BSP_D50A_PollTick(uint32_t now_ms)
{
    if (s_huart2 == NULL) {
        return;
    }

    /* Consume completed asynchronous RS485 reply captured since the previous 10 Hz tick.
     * 先处理自上一 10 Hz 周期以来异步中断接收完成的 RS485 应答帧。 */
    if (s_frame_ready) {
        uint8_t local_frame[BSP_D50A_FRAME_LEN];
        __disable_irq();
        memcpy(local_frame, s_isr_frame, BSP_D50A_FRAME_LEN);
        s_frame_ready = false;
        __enable_irq();

        if (BSP_D50A_ParseResponse(local_frame,
                                   BSP_D50A_DEFAULT_ADDR,
                                   s_pending_is_status,
                                   &s_telemetry)) {
            s_last_rx_ms = now_ms;
        }
    }

    if ((uint32_t)(now_ms - s_last_rx_ms) > D50A_ONLINE_TIMEOUT_MS) {
        s_telemetry.online = false;
        s_telemetry.fault_mask = BSP_D50A_ConvertRawAlarmToFaultMask(s_telemetry.raw_alarm, false);
    }

    /* Reset parser before issuing next request and alternate 4x 0xA0 status + 1x 0xA3 temp.
     * 发出下一帧查询前复位接收状态机，并以 4:1 节奏交替轮询状态/电流与温度。 */
    __disable_irq();
    s_parser.idx = 0u;
    s_frame_ready = false;
    __enable_irq();

    s_pending_is_status = ((s_poll_step % 5u) != 4u);
    s_poll_step++;

    uint8_t tx_frame[BSP_D50A_FRAME_LEN];
    const uint8_t func = s_pending_is_status ? BSP_D50A_FUNC_READ_STATUS : BSP_D50A_FUNC_READ_TEMP;
    BSP_D50A_BuildFrame(tx_frame, BSP_D50A_DEFAULT_ADDR, func, 0u, 0u, 0u, 0u);
    (void)HAL_UART_Transmit(s_huart2, tx_frame, BSP_D50A_FRAME_LEN, 5u);
}

void BSP_D50A_GetTelemetry(BSP_D50A_Telemetry *out_telemetry)
{
    if (out_telemetry == NULL) {
        return;
    }
    __disable_irq();
    *out_telemetry = s_telemetry;
    __enable_irq();
}
#endif /* USE_HAL_DRIVER */

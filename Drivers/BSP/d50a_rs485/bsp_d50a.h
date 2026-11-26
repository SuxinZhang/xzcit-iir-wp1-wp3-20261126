/**
 * @file bsp_d50a.h
 * @brief D50A RS485 motor driver telemetry BSP on USART2 (PD5 TX / PD6 RX).
 *        基于 USART2（PD5 TX / PD6 RX）单任务队列独占架构的 D50A RS485 驱动器监控驱动。
 */

#ifndef IIR_BSP_D50A_H
#define IIR_BSP_D50A_H

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Fixed D50A RS485 frame length in bytes (3x 0xAA, ADDR, FUNC, D0..D3, 4x 0xBB).
 *  D50A RS485 通信固定帧长（13 字节）。 */
#define BSP_D50A_FRAME_LEN 13u

/** Default RS485 slave node address monitored on C50C chassis.
 *  C50C 底盘默认监控的 D50A 驱动器从站地址（0x01）。 */
#define BSP_D50A_DEFAULT_ADDR 0x01u

/** D50A protocol function codes for status/current (0xA0) and temperature (0xA3).
 *  D50A 协议读取报警/电流（0xA0）与读取温度（0xA3）功能码。 */
#define BSP_D50A_FUNC_READ_STATUS 0xA0u
#define BSP_D50A_FUNC_READ_TEMP   0xA3u

/**
 * @brief Normalized active-HIGH D50A fault mask exported to safety manager & telemetry.
 *        归一化高电平有效 D50A 保护报警掩码位定义（用于安全状态机与遥测上报）。
 */
typedef enum {
    BSP_D50A_FAULT_NONE = 0u,
    BSP_D50A_FAULT_OVERCURRENT = (1u << 0),
    BSP_D50A_FAULT_OVERTEMP = (1u << 1),
    BSP_D50A_FAULT_OVERVOLTAGE = (1u << 2),
    BSP_D50A_FAULT_UNDERVOLTAGE = (1u << 3),
    BSP_D50A_FAULT_OFFLINE = (1u << 7)
} BSP_D50A_FaultBits;

/**
 * @brief Stream accumulator for D50A 13-byte RS485 response frames.
 *        D50A 13 字节 RS485 响应帧字节流接收解析器状态结构。
 */
typedef struct {
    uint8_t buf[BSP_D50A_FRAME_LEN];
    uint8_t idx;
} BSP_D50A_RxParser;

/**
 * @brief Parsed D50A motor driver health, current, and temperature telemetry.
 *        解析后的 D50A 电机驱动器在线状态、电流、温度与保护故障快照。
 */
typedef struct {
    bool online;
    uint8_t raw_alarm;
    uint8_t fault_mask;
    int16_t m1_current_ma;
    int16_t m2_current_ma;
    uint8_t m1_temp_c;
    uint8_t m2_temp_c;
} BSP_D50A_Telemetry;

/**
 * @brief Build a 13-byte D50A request frame with 0xAA header and 0xBB trailer.
 *        构造带 0xAA 帧头与 0xBB 帧尾的 13 字节 D50A RS485 请求帧。
 */
void BSP_D50A_BuildFrame(uint8_t out_frame[BSP_D50A_FRAME_LEN],
                         uint8_t addr,
                         uint8_t func,
                         uint8_t d0,
                         uint8_t d1,
                         uint8_t d2,
                         uint8_t d3);

/**
 * @brief Convert D50A active-LOW alarm byte into active-HIGH BSP_D50A_FaultBits mask.
 *        将 D50A 协议低电平有效报警字节反相转换为高有效标准故障位掩码。
 */
uint8_t BSP_D50A_ConvertRawAlarmToFaultMask(uint8_t raw_alarm, bool online);

/**
 * @brief Feed one received RS485 byte into the 13-byte frame resynchronizer.
 *        向 D50A 字节流解析器喂入单个串口字节，收满合法帧时输出 13 字节副本。
 */
bool BSP_D50A_FeedRxByte(BSP_D50A_RxParser *parser,
                         uint8_t byte,
                         uint8_t out_frame[BSP_D50A_FRAME_LEN]);

/**
 * @brief Parse a verified 13-byte D50A 0xA0 (status/current) or 0xA3 (temp) response.
 *        解析通过帧头/帧尾校验的 13 字节 D50A 状态电流（0xA0）或温度（0xA3）应答帧。
 */
bool BSP_D50A_ParseResponse(const uint8_t frame[BSP_D50A_FRAME_LEN],
                            uint8_t expected_addr,
                            bool is_status_query,
                            BSP_D50A_Telemetry *io_telemetry);

#ifdef USE_HAL_DRIVER
/**
 * @brief Initialize USART2 RS485 D50A monitor driver (single-task exclusive bus access).
 *        初始化 USART2 D50A RS485 状态监控驱动（单任务队列独占总线架构）。
 */
bool BSP_D50A_Init(UART_HandleTypeDef *huart_rs485);

/**
 * @brief Non-blocking 10 Hz bus poll tick invoked exclusively from D50ATask.
 *        仅由 D50ATask 调用的 10 Hz 周期性 RS485 状态与温度轮询状态机。
 */
void BSP_D50A_PollTick(uint32_t now_ms);

/**
 * @brief Push one received USART2 byte into D50A ISR receive queue.
 *        在 USART2 接收中断中将单字节压入 D50A 收帧状态机。
 */
void BSP_D50A_OnRxByte(uint8_t byte);

/**
 * @brief Handle USART2 RXNE and line-error flags safely from USART2_IRQHandler.
 *        在 USART2_IRQHandler 中安全读取 RXNE 数据并清除总线错误标志。
 */
void BSP_D50A_OnUartIrq(UART_HandleTypeDef *huart);

/**
 * @brief Copy the latest atomic D50A telemetry snapshot safely.
 *        原子拷贝最近一次更新的 D50A 驱动器遥测快照。
 */
void BSP_D50A_GetTelemetry(BSP_D50A_Telemetry *out_telemetry);
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_D50A_H */

/**
 * @file bsp_sos.h
 * @brief Physical SOS button BSP driver on PE5 (EXTI5 falling-edge + 20 ms debounce).
 *        位于 PE5（EXTI5 下降沿外部中断 + 20 ms 消抖）的物理 SOS 一键呼救按钮驱动。
 */

#ifndef IIR_BSP_SOS_H
#define IIR_BSP_SOS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Debounce qualification window in milliseconds for PE5 physical SOS input.
 *  PE5 物理 SOS 按键按下状态的消抖确认时间窗（20 ms）。 */
#define BSP_SOS_DEBOUNCE_MS 20u

/**
 * @brief Initialize PE5 physical SOS button debounce and latch state.
 *        初始化 PE5 物理 SOS 按键消抖与紧急锁存状态。
 */
void BSP_Sos_Init(void);

/**
 * @brief Interrupt service notification invoked from EXTI9_5 when PE5 triggers.
 *        PE5 下降沿外部中断触发时调用的中断事件通知函数。
 */
void BSP_Sos_OnExtiIrq(uint16_t gpio_pin);

/**
 * @brief Poll and debounce physical PE5 input state at control loop cadence.
 *        在控制周期内更新 PE5 物理电平消抖与 SOS 紧急停机锁存状态。
 */
void BSP_Sos_Update(uint32_t now_ms);

/**
 * @brief Return true when physical PE5 SOS emergency stop is latched.
 *        当 PE5 物理 SOS 按键已锁存紧急报警态时返回 true。
 */
bool BSP_Sos_IsLatched(void);

/**
 * @brief Return instantaneous debounced/raw pressed state (true = PE5 held LOW).
 *        返回 PE5 按键当前是否处于按压低电平状态（按下返回 true）。
 */
bool BSP_Sos_IsPressed(void);

/**
 * @brief Clear hardware latch flag once PE5 button has been released.
 *        在确认 PE5 按键松开后清除底层硬件锁存标志。
 */
bool BSP_Sos_ClearLatch(void);

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_SOS_H */

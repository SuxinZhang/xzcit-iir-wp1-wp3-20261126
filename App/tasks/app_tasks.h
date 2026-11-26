/**
 * @file app_tasks.h
 * @brief FreeRTOS task orchestration header connecting BSP drivers and pure-C App modules.
 *        连接板级驱动（BSP）与纯 C 应用核心模块的 FreeRTOS 任务编排头文件。
 */

#ifndef IIR_APP_TASKS_APP_TASKS_H
#define IIR_APP_TASKS_APP_TASKS_H

#include "stm32f4xx_hal.h"

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Hardware peripheral handle bundle passed from Core/main.c into App/tasks/.
 *        由 Core/main.c 传入 App/tasks/ 的硬件外设句柄集合。
 */
typedef struct {
    /** TIM8 handle driving 4-channel motor PWM (PC6..PC9).
     *  驱动四路电机 PWM（PC6..PC9）的 TIM8 句柄。 */
    TIM_HandleTypeDef *htim8_pwm;

    /** TIM1 handle capturing front HC-SR04 echo pulse width on PE9.
     *  在 PE9 上捕获前置 HC-SR04 超声波回波脉宽的 TIM1 句柄。 */
    TIM_HandleTypeDef *htim1_echo;

    /** TIM2 quadrature encoder handle for Motor A.
     *  电机 A 正交编码器 TIM2 句柄。 */
    TIM_HandleTypeDef *htim_enc_a;

    /** TIM3 quadrature encoder handle for Motor B.
     *  电机 B 正交编码器 TIM3 句柄。 */
    TIM_HandleTypeDef *htim_enc_b;

    /** TIM4 quadrature encoder handle for Motor C.
     *  电机 C 正交编码器 TIM4 句柄。 */
    TIM_HandleTypeDef *htim_enc_c;

    /** TIM5 quadrature encoder handle for Motor D.
     *  电机 D 正交编码器 TIM5 句柄。 */
    TIM_HandleTypeDef *htim_enc_d;

    /** USART2 handle wired to SP485EEN for D50A driver monitoring (PD5/PD6).
     *  连接 SP485EEN 监控 D50A 电机驱动器的 USART2 句柄（PD5/PD6）。 */
    UART_HandleTypeDef *huart2_d50a;

    /** USART3 handle connected to Orange Pi host bridge (c50c_uart3).
     *  连接上位机 Orange Pi（c50c_uart3）的 USART3 句柄。 */
    UART_HandleTypeDef *huart3_host;

    /** ADC1 handle sampling battery voltage on PB0.
     *  采样 PB0 电池电压的 ADC1 句柄。 */
    ADC_HandleTypeDef *hadc1_bat;
} IIR_AppHwHandles;

/**
 * @brief Initialize BSP peripherals, pure-C state machines, and create FreeRTOS tasks.
 *        初始化板级外设、纯 C 控制/协议状态机，并创建 FreeRTOS 控制与遥测任务。
 *
 * @param[in] hw Pointer to initialized HAL peripheral handle bundle.
 *               已初始化的 HAL 外设句柄集合指针。
 * @return true when all BSP modules and FreeRTOS tasks were created cleanly.
 *         全部板级驱动初始化及 FreeRTOS 任务创建成功时返回 true。
 */
bool IIR_AppTasks_Start(const IIR_AppHwHandles *hw);

#ifdef __cplusplus
}
#endif

#endif /* IIR_APP_TASKS_APP_TASKS_H */

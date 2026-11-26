/**
 * @file main.h
 * @brief Top-level HAL peripheral handle exports and C50C-V4.3 pin definitions.
 *        顶层 HAL 外设句柄导出与 C50C-V4.3 硬件引脚定义头文件。
 */

#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Exported peripheral handles shared with Core interrupt handlers and App/tasks/.
 * 与 Core 中断服务及 App/tasks/ 共享的 HAL 外设句柄声明。 */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

/* Motor direction, enable switch, buzzer, RGB, and OLED pin mappings (C50C-V4.3).
 * 电机方向、使能开关、蜂鸣器、RGB 灯及 OLED 引脚映射定义（依据 C50C-V4.3 原理图）。 */
#define IIR_AIN1_GPIO_Port GPIOB
#define IIR_AIN1_Pin       GPIO_PIN_13
#define IIR_AIN2_GPIO_Port GPIOB
#define IIR_AIN2_Pin       GPIO_PIN_12

#define IIR_BIN1_GPIO_Port GPIOC
#define IIR_BIN1_Pin       GPIO_PIN_0
#define IIR_BIN2_GPIO_Port GPIOE
#define IIR_BIN2_Pin       GPIO_PIN_15

#define IIR_CIN1_GPIO_Port GPIOD
#define IIR_CIN1_Pin       GPIO_PIN_10
#define IIR_CIN2_GPIO_Port GPIOD
#define IIR_CIN2_Pin       GPIO_PIN_12

#define IIR_DIN1_GPIO_Port GPIOC
#define IIR_DIN1_Pin       GPIO_PIN_12
#define IIR_DIN2_GPIO_Port GPIOA
#define IIR_DIN2_Pin       GPIO_PIN_8

#define IIR_EN_GPIO_Port   GPIOE
#define IIR_EN_Pin         GPIO_PIN_4

#define IIR_KEY_GPIO_Port  GPIOC
#define IIR_KEY_Pin        GPIO_PIN_5

#define IIR_BUZZER_GPIO_Port GPIOD
#define IIR_BUZZER_Pin       GPIO_PIN_11

#define IIR_LED_R_GPIO_Port GPIOB
#define IIR_LED_R_Pin       GPIO_PIN_15
#define IIR_LED_G_GPIO_Port GPIOE
#define IIR_LED_G_Pin       GPIO_PIN_6
#define IIR_LED_B_GPIO_Port GPIOB
#define IIR_LED_B_Pin       GPIO_PIN_14

#define IIR_OLED_SCL_GPIO_Port GPIOD
#define IIR_OLED_SCL_Pin       GPIO_PIN_7
#define IIR_OLED_SDA_GPIO_Port GPIOD
#define IIR_OLED_SDA_Pin       GPIO_PIN_4
#define IIR_OLED_RST_GPIO_Port GPIOE
#define IIR_OLED_RST_Pin       GPIO_PIN_1
#define IIR_OLED_DC_GPIO_Port  GPIOE
#define IIR_OLED_DC_Pin        GPIO_PIN_0

/* WP2 peripheral pin mappings (HC-SR04, SOS EXTI5, ICM-20948 SW-I2C, D50A USART2, CAN1).
 * WP2 新增外设引脚映射定义（超声波、SOS 按键、ICM-20948 软件 I2C、D50A RS485、CAN1 回充）。 */
#define IIR_HC_ECHO_GPIO_Port  GPIOE
#define IIR_HC_ECHO_Pin        GPIO_PIN_9
#define IIR_HC_TRIG_GPIO_Port  GPIOE
#define IIR_HC_TRIG_Pin        GPIO_PIN_11

#define IIR_SOS_GPIO_Port      GPIOE
#define IIR_SOS_Pin            GPIO_PIN_5

#define IIR_IMU_SDA_GPIO_Port  GPIOB
#define IIR_IMU_SDA_Pin        GPIO_PIN_8
#define IIR_IMU_SCL_GPIO_Port  GPIOB
#define IIR_IMU_SCL_Pin        GPIO_PIN_9
#define IIR_IMU_INT_GPIO_Port  GPIOD
#define IIR_IMU_INT_Pin        GPIO_PIN_3

#define IIR_D50A_TX_GPIO_Port  GPIOD
#define IIR_D50A_TX_Pin        GPIO_PIN_5
#define IIR_D50A_RX_GPIO_Port  GPIOD
#define IIR_D50A_RX_Pin        GPIO_PIN_6

#define IIR_CAN1_GPIO_Port     GPIOD
#define IIR_CAN1_RX_Pin        GPIO_PIN_0
#define IIR_CAN1_TX_Pin        GPIO_PIN_1

/**
 * @brief Enter Fail-Closed halt state when fatal HAL initialization fails.
 *        当底层 HAL 初始化发生致命错误时进入故障安全停机状态。
 */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */

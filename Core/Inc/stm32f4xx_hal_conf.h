/**
 * @file stm32f4xx_hal_conf.h
 * @brief Clean STM32F4xx HAL module configuration for XZCIT-IIR (STM32F407VET6).
 *        面向 XZCIT-IIR（STM32F407VET6）的洁净版 STM32F4xx HAL 模块裁剪配置头文件。
 */

#ifndef STM32F4XX_HAL_CONF_H
#define STM32F4XX_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Enable only the HAL peripheral modules required by the clean WP1 firmware.
 * 仅启用 WP1 洁净固件所需的 HAL 外设模块，避免冗余耦合与引脚冲突。 */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
/* The CAN auto-dock BSP drives bxCAN registers directly; no HAL CAN source file is required.
 * CAN 自动回充 BSP 直接操作 bxCAN 寄存器，因此不需要 HAL CAN 源文件。 */
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED

/* Note: HAL_I2C_MODULE_ENABLED is intentionally disabled on WP1/WP2 because C50C-V4.3
 * swaps PB9(SCL)/PB8(SDA) relative to STM32F407 hardware I2C1 AF4.
 * 说明：因 C50C-V4.3 原理图将 PB9(SCL)/PB8(SDA) 与硬件 I2C1 AF4 反接，此处不启用硬件 I2C。 */

/* Board external high-speed crystal oscillator frequency in Hz (8 MHz on C50C-V4.3).
 * 板载外部高速晶振频率（C50C-V4.3 为 8 MHz）。 */
#if !defined(HSE_VALUE)
  #define HSE_VALUE    8000000U
#endif

#if !defined(HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    100U
#endif

#if !defined(HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U)
#endif

#if !defined(LSI_VALUE)
  #define LSI_VALUE    32000U
#endif

#if !defined(LSE_VALUE)
  #define LSE_VALUE    32768U
#endif

#if !defined(LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT    5000U
#endif

#if !defined(EXTERNAL_CLOCK_VALUE)
  #define EXTERNAL_CLOCK_VALUE    12288000U
#endif

/* HAL system configuration: 3.3 V supply, lowest SysTick priority for FreeRTOS compatibility.
 * HAL 系统配置：3.3 V 供电，SysTick 中断置为最低抢占优先级 15 以兼容 FreeRTOS。 */
#define VDD_VALUE                    3300U
#define TICK_INT_PRIORITY            15U
#define USE_RTOS                     0U
#define PREFETCH_ENABLE              1U
#define INSTRUCTION_CACHE_ENABLE     1U
#define DATA_CACHE_ENABLE            1U

/* Disable dynamic HAL callback registration to save RAM and simplify static dispatch.
 * 禁用动态 HAL 回调注册以节省 SRAM 并使用确定性静态回调分发。 */
#define USE_HAL_ADC_REGISTER_CALLBACKS         0U
#define USE_HAL_CAN_REGISTER_CALLBACKS         0U
#define USE_HAL_CEC_REGISTER_CALLBACKS         0U
#define USE_HAL_CRYP_REGISTER_CALLBACKS        0U
#define USE_HAL_DAC_REGISTER_CALLBACKS         0U
#define USE_HAL_DCMI_REGISTER_CALLBACKS        0U
#define USE_HAL_DFSDM_REGISTER_CALLBACKS       0U
#define USE_HAL_DMA2D_REGISTER_CALLBACKS       0U
#define USE_HAL_DSI_REGISTER_CALLBACKS         0U
#define USE_HAL_ETH_REGISTER_CALLBACKS         0U
#define USE_HAL_HASH_REGISTER_CALLBACKS        0U
#define USE_HAL_I2C_REGISTER_CALLBACKS         0U
#define USE_HAL_FMPI2C_REGISTER_CALLBACKS      0U
#define USE_HAL_FMPSMBUS_REGISTER_CALLBACKS    0U
#define USE_HAL_I2S_REGISTER_CALLBACKS         0U
#define USE_HAL_IRDA_REGISTER_CALLBACKS        0U
#define USE_HAL_LPTIM_REGISTER_CALLBACKS       0U
#define USE_HAL_LTDC_REGISTER_CALLBACKS        0U
#define USE_HAL_MMC_REGISTER_CALLBACKS         0U
#define USE_HAL_NAND_REGISTER_CALLBACKS        0U
#define USE_HAL_NOR_REGISTER_CALLBACKS         0U
#define USE_HAL_PCCARD_REGISTER_CALLBACKS      0U
#define USE_HAL_PCD_REGISTER_CALLBACKS         0U
#define USE_HAL_QSPI_REGISTER_CALLBACKS        0U
#define USE_HAL_RNG_REGISTER_CALLBACKS         0U
#define USE_HAL_RTC_REGISTER_CALLBACKS         0U
#define USE_HAL_SAI_REGISTER_CALLBACKS         0U
#define USE_HAL_SD_REGISTER_CALLBACKS          0U
#define USE_HAL_SMARTCARD_REGISTER_CALLBACKS   0U
#define USE_HAL_SDRAM_REGISTER_CALLBACKS       0U
#define USE_HAL_SRAM_REGISTER_CALLBACKS        0U
#define USE_HAL_SPDIFRX_REGISTER_CALLBACKS     0U
#define USE_HAL_SMBUS_REGISTER_CALLBACKS       0U
#define USE_HAL_SPI_REGISTER_CALLBACKS         0U
#define USE_HAL_TIM_REGISTER_CALLBACKS         0U
#define USE_HAL_UART_REGISTER_CALLBACKS        0U
#define USE_HAL_USART_REGISTER_CALLBACKS       0U
#define USE_HAL_WWDG_REGISTER_CALLBACKS        0U

#define USE_SPI_CRC                            0U

/* Include enabled HAL driver headers.
 * 包含已启用的 HAL 驱动头文件。 */
#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f4xx_hal_exti.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32f4xx_hal_pwr.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32f4xx_hal_tim.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32f4xx_hal_uart.h"
#endif

/* Stub out assert_param in release/minimal builds without stdio dependency.
 * 在不依赖 stdio 的固件构建中将 assert_param 展开为空检查。 */
#ifdef USE_FULL_ASSERT
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((uint8_t *)__FILE__, __LINE__))
  void assert_failed(uint8_t *file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* STM32F4XX_HAL_CONF_H */

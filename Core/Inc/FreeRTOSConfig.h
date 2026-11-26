/**
 * @file FreeRTOSConfig.h
 * @brief Clean FreeRTOS kernel configuration for STM32F407VET6 (GCC ARM_CM4F port).
 *        面向 STM32F407VET6（GCC ARM_CM4F 移植层）的洁净 FreeRTOS 内核配置。
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>

/* Reference CMSIS SystemCoreClock updated to 168 MHz by SystemInit().
 * 引用由 SystemInit() 更新为 168 MHz 的 CMSIS 系统主频变量。 */
extern uint32_t SystemCoreClock;

/* Scheduler and timing behavior: preemptive 1 kHz (1 ms) tick on Cortex-M4F.
 * 调度与时基配置：Cortex-M4F 抢占式调度，1 kHz（1 ms）系统节拍。 */
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      ( SystemCoreClock )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 32 )
#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 128 )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1

/* Memory allocation: dynamic heap_4 (32 KiB inside 128 KiB SRAM).
 * 内存分配配置：使用 heap_4 动态堆（在 128 KiB SRAM 中分配 32 KiB）。 */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 32 * 1024 ) )

/* Hook and diagnostic options (stack overflow detection disabled for initial bring-up).
 * 钩子函数与诊断开关。 */
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_APPLICATION_TASK_TAG          0
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0

/* Software timers configuration.
 * 软件定时器配置。 */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                5
#define configTIMER_TASK_STACK_DEPTH            ( configMINIMAL_STACK_SIZE * 2 )

/* Co-routines disabled.
 * 禁用协程。 */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/* Optional API functions enabled for App/tasks/.
 * 为 App/tasks/ 启用的 FreeRTOS 可选 API 函数。 */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskDelayUntil                 1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1

/* Cortex-M4 NVIC priority configuration (4 priority bits, lowest=15, syscall<=5).
 * Cortex-M4 NVIC 中断优先级配置（4 位抢占优先级，内核最低 15，可调 API 最高 5）。 */
#ifdef __NVIC_PRIO_BITS
  #define configPRIO_BITS                       __NVIC_PRIO_BITS
#else
  #define configPRIO_BITS                       4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY       15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/* Fail-Closed assert trap without blocking UART printf dependency.
 * 无阻塞串口 printf 依赖的故障安全断言捕获宏。 */
#define configASSERT( x ) \
    if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ) {} }

/* Map FreeRTOS port handlers onto standard CMSIS vector names.
 * 将 FreeRTOS 移植层中断处理函数映射至标准 CMSIS 向量名。 */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

#endif /* FREERTOS_CONFIG_H */

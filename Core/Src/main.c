/**
 * @file main.c
 * @brief Clean STM32CubeMX HAL entry point for XZCIT-IIR differential chassis firmware.
 *        XZCIT-IIR 差速底盘固件的洁净 STM32CubeMX HAL 主程序入口。
 */

#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

#include "app_tasks.h"
#include "hc_sr04/bsp_hc_sr04.h"
#include "icm20948/bsp_icm20948.h"
#include "sos_button/bsp_sos.h"
#include "uartx/bsp_uartx.h"

/* Global HAL peripheral handles.
 * 全局 HAL 外设句柄实例。 */
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_Encoder_Init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance);

int main(void)
{
    /* SystemInit() has already executed HAL_Init() and configured 168 MHz PLL clocks.
     * SystemInit() 已于启动阶段执行 HAL_Init() 并完成 168 MHz 主频时钟树配置。 */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM8_Init();
    MX_USART2_UART_Init();
    MX_USART3_UART_Init();

    /* Hardware notice (C50C-V4.3):
     * 1. Legacy RC capture on PE9/PE11 is permanently removed (Invariant 4); PE9 uses TIM1_CH1 IC.
     * 2. Hardware I2C1 on PB8/PB9 is omitted because PCB swaps PB9=SCL / PB8=SDA (400 kHz SW I2C).
     * 3. Manual controller init, control, and disconnect gate are permanently removed.
     * 硬件约束说明（C50C-V4.3）：
     * 1. 已彻底禁用原航模遥控捕获（红线第 4 条），PE9 配置为 TIM1_CH1 超声波硬件输入捕获。
     * 2. 不启用硬件 I2C1，因原理图将 PB9(SCL)/PB8(SDA) 物理反接（采用 400 kHz 开漏软件 I2C）。
     * 3. 已永久取消无线/有线操控接收机初始化、控制逻辑及断链安全门。 */

    /* Construct BSP handles and start FreeRTOS chassis control, IMU, D50A, and telemetry tasks.
     * 初始化板级驱动句柄并创建 FreeRTOS 控制、IMU 采集、D50A 监控与遥测任务。 */
    IIR_AppHwHandles hw = {
        .htim8_pwm = &htim8,
        .htim1_echo = &htim1,
        .htim_enc_a = &htim2,
        .htim_enc_b = &htim3,
        .htim_enc_c = &htim4,
        .htim_enc_d = &htim5,
        .huart2_d50a = &huart2,
        .huart3_host = &huart3,
        .hadc1_bat = &hadc1,
    };

    if (!IIR_AppTasks_Start(&hw)) {
        Error_Handler();
    }

    vTaskStartScheduler();

    /* Should never reach here unless FreeRTOS heap is exhausted.
     * 正常情况下调度器接管运行不会返回；返回则说明堆空间不足。 */
    Error_Handler();
    return 0;
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Initialize all motor direction outputs LOW before enabling outputs.
     * 在开启驱动之前将所有电机方向引脚默认置低电平，防止上电误动作。 */
    HAL_GPIO_WritePin(IIR_AIN1_GPIO_Port, IIR_AIN1_Pin | IIR_AIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IIR_BIN1_GPIO_Port, IIR_BIN1_Pin | IIR_DIN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IIR_BIN2_GPIO_Port, IIR_BIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IIR_CIN1_GPIO_Port, IIR_CIN1_Pin | IIR_CIN2_Pin | IIR_BUZZER_Pin |
                      IIR_OLED_SCL_Pin | IIR_OLED_SDA_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IIR_DIN2_GPIO_Port, IIR_DIN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IIR_OLED_RST_GPIO_Port, IIR_OLED_RST_Pin | IIR_OLED_DC_Pin, GPIO_PIN_RESET);

    /* Common-anode onboard RGB LED1 is active-LOW; initialize HIGH (off).
     * 板载共阳极 RGB 灯珠 LED1 为低电平有效；上电默认置高电平熄灭。 */
    HAL_GPIO_WritePin(GPIOB, IIR_LED_R_Pin | IIR_LED_B_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, IIR_LED_G_Pin, GPIO_PIN_SET);

    /* Output pins on GPIOA: PA8 (DIN2).
     * GPIOA 输出引脚：PA8（电机 D 方向 2）。 */
    gpio.Pin = IIR_DIN2_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Output pins on GPIOB: PB12 (AIN2), PB13 (AIN1), PB14 (LED_B), PB15 (LED_R).
     * GPIOB 输出引脚：电机 A 方向及 RGB 红/蓝控制引脚。 */
    gpio.Pin = IIR_AIN1_Pin | IIR_AIN2_Pin | IIR_LED_B_Pin | IIR_LED_R_Pin;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Output pins on GPIOC: PC0 (BIN1), PC12 (DIN1).
     * GPIOC 输出引脚：电机 B/D 方向控制引脚。 */
    gpio.Pin = IIR_BIN1_Pin | IIR_DIN1_Pin;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Output pins on GPIOD: PD4/PD7 (OLED), PD10/PD12 (CIN1/CIN2), PD11 (Buzzer).
     * GPIOD 输出引脚：OLED、电机 C 方向与蜂鸣器。 */
    gpio.Pin = IIR_OLED_SDA_Pin | IIR_OLED_SCL_Pin | IIR_CIN1_Pin | IIR_BUZZER_Pin | IIR_CIN2_Pin;
    HAL_GPIO_Init(GPIOD, &gpio);

    /* Output pins on GPIOE: PE0/PE1 (OLED), PE6 (LED_G), PE11 (HC_TRIG), PE15 (BIN2).
     * GPIOE 输出引脚：OLED、RGB 绿灯、HC-SR04 超声波触发与电机 B 方向。 */
    HAL_GPIO_WritePin(IIR_HC_TRIG_GPIO_Port, IIR_HC_TRIG_Pin, GPIO_PIN_RESET);
    gpio.Pin = IIR_OLED_DC_Pin | IIR_OLED_RST_Pin | IIR_LED_G_Pin |
               IIR_HC_TRIG_Pin | IIR_BIN2_Pin;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* Open-drain pull-up HIGH pins on GPIOB: PB8 (SDA), PB9 (SCL) for 400 kHz ICM-20948 SW I2C.
     * GPIOB 开漏上拉软件 I2C 引脚：PB8（SDA）与 PB9（SCL），默认释放为高电平。 */
    HAL_GPIO_WritePin(GPIOB, IIR_IMU_SDA_Pin | IIR_IMU_SCL_Pin, GPIO_PIN_SET);
    gpio.Pin = IIR_IMU_SDA_Pin | IIR_IMU_SCL_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Hardware enable switch PE4 and user button PC5 as pull-up inputs.
     * 硬件使能开关 PE4 与用户按键 PC5 配置为上拉输入。 */
    gpio.Pin = IIR_EN_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(IIR_EN_GPIO_Port, &gpio);

    gpio.Pin = IIR_KEY_Pin;
    HAL_GPIO_Init(IIR_KEY_GPIO_Port, &gpio);

    /* Physical SOS button PE5 as pull-up falling-edge external interrupt (EXTI5).
     * PE5 物理 SOS 紧急按钮配置为上拉下降沿触发外部中断（EXTI5）。 */
    gpio.Pin = IIR_SOS_Pin;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IIR_SOS_GPIO_Port, &gpio);

    /* ICM-20948 INT1 pin PD3 as rising-edge external interrupt (EXTI3).
     * PD3 配置为 ICM-20948 数据就绪上升沿外部中断（EXTI3）。 */
    gpio.Pin = IIR_IMU_INT_Pin;
    gpio.Mode = GPIO_MODE_IT_RISING;
    gpio.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(IIR_IMU_INT_GPIO_Port, &gpio);

    HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

    HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM1_Init(void)
{
    TIM_IC_InitTypeDef sConfigIC = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* 168 MHz APB2 timer clock / (167 + 1) = 1.000 MHz (1 us resolution per tick).
     * 168 MHz APB2 定时器时钟 168 分频后得到 1 MHz（1 us 分辨率），用于超声波输入捕获。 */
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 167u;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535u;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0u;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 4u;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_Encoder_Init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance)
{
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim->Instance = instance;
    htim->Init.Prescaler = 0;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 65535;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(htim, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM2_Init(void)
{
    MX_Encoder_Init(&htim2, TIM2);
}

static void MX_TIM3_Init(void)
{
    MX_Encoder_Init(&htim3, TIM3);
}

static void MX_TIM4_Init(void)
{
    MX_Encoder_Init(&htim4, TIM4);
}

static void MX_TIM5_Init(void)
{
    MX_Encoder_Init(&htim5, TIM5);
}

static void MX_TIM8_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* 168 MHz APB2 timer clock / 16800 = 10 kHz motor PWM frequency.
     * 168 MHz APB2 定时器时钟 / 16800 = 10 kHz 电机 PWM 开关频率。 */
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim8.Init.Period = 16799;
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter = 0;
    htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
        Error_Handler();
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BSP_Uart3_OnRxEvent(huart, Size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BSP_Uart3_OnTxCplt(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    BSP_Uart3_OnError(huart);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    BSP_HcSr04_OnCaptureIrq(htim);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BSP_Sos_OnExtiIrq(GPIO_Pin);
    BSP_Icm20948_OnExtiIrq(GPIO_Pin);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

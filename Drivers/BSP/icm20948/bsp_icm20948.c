/**
 * @file bsp_icm20948.c
 * @brief Implementation of ICM-20948 IMU BSP driver over 400 kHz open-drain software I2C
 *        (PB9=SCL, PB8=SDA) with DWT cycle timing, 9-clock bus recovery, and ROS axis mapping.
 *        基于 DWT 精确定时、400 kHz 开漏软件 I2C（PB9=SCL, PB8=SDA）及 9 时钟总线恢复的
 *        ICM-20948 六轴传感器驱动实现。
 */

#include "icm20948/bsp_icm20948.h"

#ifdef USE_HAL_DRIVER
#include "main.h"
#endif

#include <limits.h>
#include <stddef.h>
#include <string.h>

/* ICM-20948 register and configuration bit definitions aligned with ICM20948.h.
 * 与原厂手册对齐的 ICM-20948 寄存器地址与配置掩码定义。 */
#define ICM_ADDR_AD0_LOW       0xD0u
#define ICM_ADDR_AD0_HIGH      0xD2u
#define ICM_REG_WHO_AM_I       0x00u
#define ICM_VAL_WHO_AM_I       0xEAu
#define ICM_REG_PWR_MGMT_1     0x06u
#define ICM_REG_PWR_MGMT_2     0x07u
#define ICM_REG_ACCEL_XOUT_H   0x2Du
#define ICM_REG_BANK_SEL       0x7Fu

#define ICM_BANK_0             0x00u
#define ICM_BANK_2             0x20u

#define ICM_REG_GYRO_SMPLRT    0x00u
#define ICM_REG_GYRO_CONFIG_1  0x01u
#define ICM_REG_ACCEL_SMPLRT_2 0x11u
#define ICM_REG_ACCEL_CONFIG   0x14u

/* Bit[5:3]=DLPCFG_6 (0x30), Bit[2:1]=FS_500DPS (0x02), Bit[0]=DLPF (0x01).
 * 陀螺仪配置：+/-500 dps 量程与数字低通滤波器启用（0x33）。 */
#define ICM_VAL_GYRO_500DPS    0x33u
/* Bit[5:3]=DLPCFG_6 (0x30), Bit[2:1]=FS_2G (0x00), Bit[0]=DLPF (0x01).
 * 加速度计配置：+/-2g 量程与数字低通滤波器启用（0x31）。 */
#define ICM_VAL_ACCEL_2G       0x31u

void BSP_Icm20948_RotateChipToRos(const int16_t chip_xyz[3], int16_t ros_xyz[3])
{
    if (chip_xyz == NULL || ros_xyz == NULL) {
        return;
    }

    const int16_t cx = chip_xyz[0];
    const int16_t cy = chip_xyz[1];
    const int16_t cz = chip_xyz[2];

    /* Guard against -(-32768) overflow when inverting chip X onto ROS +Y.
     * 将芯片 -X 映射至 ROS Y 轴时防止 int16_t 最小值 -32768 取反溢出。 */
    ros_xyz[0] = cy;
    ros_xyz[1] = (cx == INT16_MIN) ? INT16_MAX : (int16_t)(-cx);
    ros_xyz[2] = cz;
}

#ifdef USE_HAL_DRIVER
static uint8_t s_dev_addr = ICM_ADDR_AD0_LOW;
static BSP_Icm20948_Data s_latest;
static volatile bool s_data_ready = false;

static bool s_calibrating = true;
static uint16_t s_calib_count = 0u;
static int32_t s_calib_sum_accel[3];
static int32_t s_calib_sum_gyro[3];
static int16_t s_bias_accel[3];
static int16_t s_bias_gyro[3];

static int16_t clamp_i32_to_i16(int32_t val)
{
    if (val > 32767) {
        return 32767;
    }
    if (val < -32768) {
        return -32768;
    }
    return (int16_t)val;
}

/* Enable Cortex-M4 DWT cycle counter for non-locking sub-microsecond I2C pacing.
 * 启用 Cortex-M4 DWT 周期计数器，用于在不开关全局中断的条件下精确定时 400 kHz I2C。 */
static void dwt_enable(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Busy-wait a precise CPU cycle count without masking interrupts (400 kHz target ~210 cycles).
 * 基于 DWT->CYCCNT 的非关中断微秒节拍等待（168 MHz 下 400 kHz 半周期约 210 周期）。 */
static void i2c_half_bit_delay(void)
{
    const uint32_t cycles = (SystemCoreClock / 800000u);
    const uint32_t start = DWT->CYCCNT;
    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
    }
}

static void delay_ms_dwt(uint32_t ms)
{
    const uint32_t cycles_per_ms = SystemCoreClock / 1000u;
    for (uint32_t m = 0u; m < ms; ++m) {
        const uint32_t start = DWT->CYCCNT;
        while ((uint32_t)(DWT->CYCCNT - start) < cycles_per_ms) {
        }
    }
}

static inline void scl_high(void)
{
    HAL_GPIO_WritePin(IIR_IMU_SCL_GPIO_Port, IIR_IMU_SCL_Pin, GPIO_PIN_SET);
}

static inline void scl_low(void)
{
    HAL_GPIO_WritePin(IIR_IMU_SCL_GPIO_Port, IIR_IMU_SCL_Pin, GPIO_PIN_RESET);
}

static inline void sda_high(void)
{
    HAL_GPIO_WritePin(IIR_IMU_SDA_GPIO_Port, IIR_IMU_SDA_Pin, GPIO_PIN_SET);
}

static inline void sda_low(void)
{
    HAL_GPIO_WritePin(IIR_IMU_SDA_GPIO_Port, IIR_IMU_SDA_Pin, GPIO_PIN_RESET);
}

static inline bool sda_read(void)
{
    return HAL_GPIO_ReadPin(IIR_IMU_SDA_GPIO_Port, IIR_IMU_SDA_Pin) == GPIO_PIN_SET;
}

static void sw_i2c_start(void)
{
    sda_high();
    scl_high();
    i2c_half_bit_delay();
    sda_low();
    i2c_half_bit_delay();
    scl_low();
}

static void sw_i2c_stop(void)
{
    scl_low();
    sda_low();
    i2c_half_bit_delay();
    scl_high();
    i2c_half_bit_delay();
    sda_high();
    i2c_half_bit_delay();
}

static bool sw_i2c_write_byte(uint8_t byte)
{
    for (uint8_t bit = 0u; bit < 8u; ++bit) {
        if ((byte & 0x80u) != 0u) {
            sda_high();
        } else {
            sda_low();
        }
        byte <<= 1u;
        i2c_half_bit_delay();
        scl_high();
        i2c_half_bit_delay();
        scl_low();
    }

    /* Release open-drain SDA and sample slave ACK on the 9th clock pulse.
     * 释放开漏 SDA 并在第 9 个时钟高电平采样从机 ACK 应答位。 */
    sda_high();
    i2c_half_bit_delay();
    scl_high();
    i2c_half_bit_delay();
    const bool acked = !sda_read();
    scl_low();
    return acked;
}

static uint8_t sw_i2c_read_byte(bool send_ack)
{
    uint8_t val = 0u;
    sda_high();

    for (uint8_t bit = 0u; bit < 8u; ++bit) {
        scl_low();
        i2c_half_bit_delay();
        scl_high();
        val = (uint8_t)((val << 1u) | (sda_read() ? 1u : 0u));
        i2c_half_bit_delay();
    }

    scl_low();
    if (send_ack) {
        sda_low();
    } else {
        sda_high();
    }
    i2c_half_bit_delay();
    scl_high();
    i2c_half_bit_delay();
    scl_low();
    sda_high();
    return val;
}

bool BSP_Icm20948_BusRecover(void)
{
    sda_high();
    scl_high();
    i2c_half_bit_delay();

    for (uint8_t pulse = 0u; pulse < 9u; ++pulse) {
        if (sda_read()) {
            break;
        }
        scl_low();
        i2c_half_bit_delay();
        scl_high();
        i2c_half_bit_delay();
    }

    sw_i2c_stop();
    return sda_read();
}

static bool icm_write_reg(uint8_t reg, uint8_t value)
{
    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t)(s_dev_addr & 0xFEu)) ||
        !sw_i2c_write_byte(reg) ||
        !sw_i2c_write_byte(value)) {
        sw_i2c_stop();
        return false;
    }
    sw_i2c_stop();
    return true;
}

static bool icm_read_regs(uint8_t reg, uint8_t *dst, size_t len)
{
    if (dst == NULL || len == 0u) {
        return false;
    }

    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t)(s_dev_addr & 0xFEu)) ||
        !sw_i2c_write_byte(reg)) {
        sw_i2c_stop();
        return false;
    }

    /* Repeated START followed by 7-bit read address and multi-byte burst read.
     * 发送重复起始条件与读地址，随后连续突发读取多字节寄存器以将耗时压缩至 0.35 ms 内。 */
    sw_i2c_start();
    if (!sw_i2c_write_byte((uint8_t)(s_dev_addr | 0x01u))) {
        sw_i2c_stop();
        return false;
    }

    for (size_t i = 0u; i < len; ++i) {
        dst[i] = sw_i2c_read_byte((i + 1u) < len);
    }
    sw_i2c_stop();
    return true;
}

void BSP_Icm20948_RestartCalibration(void)
{
    s_calibrating = true;
    s_calib_count = 0u;
    memset(s_calib_sum_accel, 0, sizeof(s_calib_sum_accel));
    memset(s_calib_sum_gyro, 0, sizeof(s_calib_sum_gyro));
    memset(s_bias_accel, 0, sizeof(s_bias_accel));
    memset(s_bias_gyro, 0, sizeof(s_bias_gyro));

    __disable_irq();
    s_latest.calibrated = false;
    __enable_irq();
}

bool BSP_Icm20948_Init(void)
{
    dwt_enable();
    memset(&s_latest, 0, sizeof(s_latest));
    BSP_Icm20948_RestartCalibration();

    if (!BSP_Icm20948_BusRecover()) {
        return false;
    }

    /* Probe primary (0xD0) and alternate (0xD2) AD0 addresses for WHO_AM_I == 0xEA.
     * 依次探测 AD0=0 (0xD0) 与 AD0=1 (0xD2) 地址上的 WHO_AM_I 寄存器（应为 0xEA）。 */
    const uint8_t addrs[2] = {ICM_ADDR_AD0_LOW, ICM_ADDR_AD0_HIGH};
    bool found = false;
    for (uint8_t i = 0u; i < 2u; ++i) {
        s_dev_addr = addrs[i];
        uint8_t wia = 0u;
        (void)icm_write_reg(ICM_REG_BANK_SEL, ICM_BANK_0);
        if (icm_read_regs(ICM_REG_WHO_AM_I, &wia, 1u) && wia == ICM_VAL_WHO_AM_I) {
            found = true;
            break;
        }
    }

    if (!found) {
        s_latest.online = false;
        return false;
    }

    /* Reset device, select best clock (CLKSEL=1), enable all 6 axes, set +/-2g & +/-500dps.
     * 复位器件、配置最优 PLL 时钟与六轴使能，并在 Bank2 设置 +/-2g 与 +/-500 dps 量程。 */
    if (!icm_write_reg(ICM_REG_BANK_SEL, ICM_BANK_0) ||
        !icm_write_reg(ICM_REG_PWR_MGMT_1, 0x80u)) {
        return false;
    }
    delay_ms_dwt(10u);

    if (!icm_write_reg(ICM_REG_PWR_MGMT_1, 0x01u) ||
        !icm_write_reg(ICM_REG_PWR_MGMT_2, 0x00u) ||
        !icm_write_reg(ICM_REG_BANK_SEL, ICM_BANK_2) ||
        !icm_write_reg(ICM_REG_GYRO_SMPLRT, 0x07u) ||
        !icm_write_reg(ICM_REG_GYRO_CONFIG_1, ICM_VAL_GYRO_500DPS) ||
        !icm_write_reg(ICM_REG_ACCEL_SMPLRT_2, 0x07u) ||
        !icm_write_reg(ICM_REG_ACCEL_CONFIG, ICM_VAL_ACCEL_2G) ||
        !icm_write_reg(ICM_REG_BANK_SEL, ICM_BANK_0)) {
        return false;
    }

    s_latest.online = true;
    return true;
}

bool BSP_Icm20948_Sample(void)
{
    uint8_t raw[12] = {0};
    s_data_ready = false;

    if (!icm_read_regs(ICM_REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        (void)BSP_Icm20948_BusRecover();
        __disable_irq();
        s_latest.online = false;
        __enable_irq();
        return false;
    }

    int16_t chip_accel[3];
    int16_t chip_gyro[3];
    for (uint8_t axis = 0u; axis < 3u; ++axis) {
        const size_t a_off = (size_t)axis * 2u;
        const size_t g_off = 6u + (size_t)axis * 2u;
        chip_accel[axis] = (int16_t)(((uint16_t)raw[a_off] << 8u) | (uint16_t)raw[a_off + 1u]);
        chip_gyro[axis] = (int16_t)(((uint16_t)raw[g_off] << 8u) | (uint16_t)raw[g_off + 1u]);
    }

    int16_t ros_accel[3];
    int16_t ros_gyro[3];
    BSP_Icm20948_RotateChipToRos(chip_accel, ros_accel);
    BSP_Icm20948_RotateChipToRos(chip_gyro, ros_gyro);

    if (s_calibrating) {
        for (uint8_t axis = 0u; axis < 3u; ++axis) {
            s_calib_sum_accel[axis] += (int32_t)ros_accel[axis];
            s_calib_sum_gyro[axis] += (int32_t)ros_gyro[axis];
        }
        s_calib_count++;
        if (s_calib_count >= BSP_ICM20948_CALIB_SAMPLES) {
            for (uint8_t axis = 0u; axis < 3u; ++axis) {
                const int32_t avg_a = s_calib_sum_accel[axis] / (int32_t)BSP_ICM20948_CALIB_SAMPLES;
                const int32_t avg_g = s_calib_sum_gyro[axis] / (int32_t)BSP_ICM20948_CALIB_SAMPLES;
                s_bias_accel[axis] = clamp_i32_to_i16(avg_a);
                s_bias_gyro[axis] = clamp_i32_to_i16(avg_g);
            }
            s_calibrating = false;
        }
    }

    int16_t out_accel[3];
    int16_t out_gyro[3];
    if (s_calibrating) {
        memcpy(out_accel, ros_accel, sizeof(out_accel));
        memcpy(out_gyro, ros_gyro, sizeof(out_gyro));
    } else {
        out_accel[0] = clamp_i32_to_i16((int32_t)ros_accel[0] - (int32_t)s_bias_accel[0]);
        out_accel[1] = clamp_i32_to_i16((int32_t)ros_accel[1] - (int32_t)s_bias_accel[1]);
        out_accel[2] = clamp_i32_to_i16((int32_t)ros_accel[2] - (int32_t)s_bias_accel[2] + BSP_ICM20948_1G_LSB);
        out_gyro[0] = clamp_i32_to_i16((int32_t)ros_gyro[0] - (int32_t)s_bias_gyro[0]);
        out_gyro[1] = clamp_i32_to_i16((int32_t)ros_gyro[1] - (int32_t)s_bias_gyro[1]);
        out_gyro[2] = clamp_i32_to_i16((int32_t)ros_gyro[2] - (int32_t)s_bias_gyro[2]);
    }

    __disable_irq();
    memcpy(s_latest.accel_lsb, out_accel, sizeof(out_accel));
    memcpy(s_latest.gyro_lsb, out_gyro, sizeof(out_gyro));
    s_latest.calibrated = !s_calibrating;
    s_latest.online = true;
    __enable_irq();
    return true;
}

void BSP_Icm20948_OnExtiIrq(uint16_t gpio_pin)
{
    if (gpio_pin == IIR_IMU_INT_Pin) {
        s_data_ready = true;
    }
}

void BSP_Icm20948_GetLatest(BSP_Icm20948_Data *out_data)
{
    if (out_data == NULL) {
        return;
    }
    __disable_irq();
    *out_data = s_latest;
    __enable_irq();
}
#endif /* USE_HAL_DRIVER */

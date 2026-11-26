/**
 * @file bsp_icm20948.h
 * @brief ICM-20948 9-axis IMU BSP driver over 400 kHz open-drain software I2C on
 *        PB9(SCL)/PB8(SDA) and PD3(INT1), compensating for C50C-V4.3 I2C1 pin swap.
 *        基于 PB9(SCL)/PB8(SDA) 400 kHz 开漏软件 I2C 与 PD3(INT1) 的 ICM-20948 驱动
 *        （规避 C50C-V4.3 原理图硬件 I2C1 引脚物理反接约束）。
 */

#ifndef IIR_BSP_ICM20948_H
#define IIR_BSP_ICM20948_H

#ifdef USE_HAL_DRIVER
#include "stm32f4xx_hal.h"
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Number of initial stationary samples averaged for gyroscope & accelerometer bias calibration.
 *  开机静止陀螺仪与加速度计零偏校准采集样本数（100 Hz 下约 1.0 s）。 */
#define BSP_ICM20948_CALIB_SAMPLES 100u

/** Target gravity count along +Z axis in raw accelerometer LSB (+2g full-scale).
 *  +/-2g 量程下 +Z 轴标准 1g 重力参考值（16384 LSB）。 */
#define BSP_ICM20948_1G_LSB 16384

/**
 * @brief IMU telemetry sample rotated to ROS REP-103 chassis coordinates.
 *        已旋转对齐至 ROS REP-103 车体坐标系的三轴加速度与角速度采样数据。
 */
typedef struct {
    /** Linear acceleration in ROS axes (X, Y, Z), +/-2g LSB (bias removed +1g on Z).
     *  ROS 坐标系三轴加速度原始 LSB（已扣除静止零偏并保留 Z 轴 +1g 基准）。 */
    int16_t accel_lsb[3];

    /** Angular velocity in ROS axes (X, Y, Z), +/-500 dps LSB (stationary bias removed).
     *  ROS 坐标系三轴角速度原始 LSB（+/-500 dps 量程，已扣除静止零偏）。 */
    int16_t gyro_lsb[3];

    /** True when startup stationary zero-bias calibration has finished.
     *  开机静止零偏校准完成后置为 true。 */
    bool calibrated;

    /** True when ICM-20948 WHO_AM_I (0xEA) probe and recent burst read succeeded.
     *  当 ICM-20948 设备 ID（0xEA）探测成功且近期突发读取正常时为 true。 */
    bool online;
} BSP_Icm20948_Data;

/**
 * @brief Map raw ICM-20948 chip axes into ROS chassis coordinates (chipY->X, -chipX->Y, chipZ->Z).
 *        将 ICM-20948 芯片坐标系原始值旋转映射为 ROS 底盘坐标系（X=+Y, Y=-X, Z=+Z）。
 */
void BSP_Icm20948_RotateChipToRos(const int16_t chip_xyz[3], int16_t ros_xyz[3]);

#ifdef USE_HAL_DRIVER
/**
 * @brief Probe and initialize ICM-20948 via 400 kHz open-drain software I2C (PB9=SCL, PB8=SDA).
 *        通过 400 kHz 开漏软件 I2C（PB9=SCL, PB8=SDA）探测并初始化 ICM-20948 传感器。
 */
bool BSP_Icm20948_Init(void);

/**
 * @brief Execute 9-clock open-drain I2C bus recovery when SDA is held LOW by a stalled slave.
 *        当从机异常拉低 SDA 导致总线卡死时，发送最多 9 个 SCL 时钟脉冲执行总线恢复。
 */
bool BSP_Icm20948_BusRecover(void);

/**
 * @brief Read raw 6-axis burst sample at 100 Hz, apply bias calibration, and update snapshot.
 *        按 100 Hz 频率突发读取 12 字节 6 轴数据、更新零偏校准并刷新遥测快照。
 */
bool BSP_Icm20948_Sample(void);

/**
 * @brief Notify data-ready state from PD3 EXTI3 interrupt callback.
 *        来自 PD3 EXTI3 外部中断的数据就绪事件通知函数。
 */
void BSP_Icm20948_OnExtiIrq(uint16_t gpio_pin);

/**
 * @brief Copy the latest atomic ROS-axis IMU telemetry snapshot safely.
 *        原子拷贝最近一次更新的 ROS 坐标系 IMU 遥测数据快照。
 */
void BSP_Icm20948_GetLatest(BSP_Icm20948_Data *out_data);

/**
 * @brief Trigger asynchronous re-calibration of stationary gyroscope/accelerometer zero-bias.
 *        触发重新采集开机静止状态下的陀螺仪与加速度计零偏基准。
 */
void BSP_Icm20948_RestartCalibration(void);
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_ICM20948_H */

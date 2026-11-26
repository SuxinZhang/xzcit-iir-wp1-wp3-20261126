/**
 * @file diff_kinematics.h
 * @brief Pure-C four-wheel differential drive kinematics with zero HAL dependencies.
 *        零 HAL 依赖的纯 C 四轮差速底盘正逆运动学接口。
 *
 * Aligns with WHEELTEC C50C differential geometry defaults and Tender Clause 7.1.3
 * velocity limits (0.01 to 1.3 m/s).
 * 对齐 WHEELTEC C50C 差速几何默认参数及标书第 7.1.3 条速度范围要求（0.01 至 1.3 m/s）。
 */

#ifndef IIR_APP_KINEMATICS_DIFF_KINEMATICS_H
#define IIR_APP_KINEMATICS_DIFF_KINEMATICS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Physical geometry, encoder scaling, and speed envelope for 4WD differential chassis.
 *        四轮差速底盘的物理几何、编码器换算与速度包络配置参数。
 */
typedef struct {
    /** Effective half-track width in meters (L = track_width / 2).
     *  等效半轮距，单位为米（L = 轮距 / 2）。 */
    float effective_half_track_m;

    /** Drive wheel outer diameter in meters.
     *  驱动轮外径，单位为米。 */
    float wheel_diameter_m;

    /** Motor gearbox reduction ratio.
     *  电机减速箱减速比。 */
    float gear_ratio;

    /** Hall encoder pulses per motor shaft revolution (single channel).
     *  电机轴单圈霍尔编码器线数（单相）。 */
    uint16_t encoder_lines;

    /** Quadrature timer multiplier (4x on STM32 encoder interface).
     *  定时正交解码倍频系数（STM32 编码器模式为 4 倍频）。 */
    uint8_t encoder_quadrature;

    /** Hard upper linear velocity bound in m/s (Tender Clause 7.1.3: 1.3 m/s).
     *  线速度硬上限，单位为 m/s（标书第 7.1.3 条：1.3 m/s）。 */
    float max_linear_speed_mps;

    /** Minimum controllable linear velocity magnitude in m/s (Tender Clause 7.1.3: 0.01 m/s).
     *  最小可控线速度幅值，单位为 m/s（标书第 7.1.3 条：0.01 m/s）。 */
    float min_linear_speed_mps;

    /** Maximum permitted body yaw angular velocity in rad/s.
     *  允许的最大车体偏航角速度，单位为 rad/s。 */
    float max_angular_speed_rad_s;

    /** Maximum permitted linear speed for any individual wheel in m/s.
     *  单个车轮允许的最大线速度，单位为 m/s。 */
    float max_wheel_speed_mps;
} IIR_KinematicsConfig;

/**
 * @brief Chassis body velocity command or odometry estimate in REP-103 frame.
 *        REP-103 车体坐标系下的速度指令或里程计估计值（X 前、Y 左、Z 逆时针为正）。
 */
typedef struct {
    /** Forward linear velocity along body +X axis in m/s.
     *  沿车体 +X 轴向前的线速度，单位为 m/s。 */
    float vx_mps;

    /** Lateral linear velocity along body +Y axis in m/s (always 0 for differential drive).
     *  沿车体 +Y 轴向左的横向速度，单位为 m/s（差速底盘恒为 0）。 */
    float vy_mps;

    /** Yaw angular velocity around body +Z axis in rad/s (counter-clockwise positive).
     *  绕车体 +Z 轴的偏航角速度，单位为 rad/s（逆时针为正）。 */
    float wz_rad_s;
} IIR_BodyTwist;

/**
 * @brief Target or measured wheel linear velocities for motors A/B/C/D in m/s.
 *        四轮电机 A/B/C/D 的目标或实测轮缘线速度，单位为 m/s。
 */
typedef struct {
    /** Front-left wheel (Motor A) linear speed in m/s.
     *  左前轮（电机 A）线速度，单位为 m/s。 */
    float fl_a;

    /** Rear-left wheel (Motor B) linear speed in m/s.
     *  左后轮（电机 B）线速度，单位为 m/s。 */
    float rl_b;

    /** Rear-right wheel (Motor C) linear speed in m/s.
     *  右后轮（电机 C）线速度，单位为 m/s。 */
    float rr_c;

    /** Front-right wheel (Motor D) linear speed in m/s.
     *  右前轮（电机 D）线速度，单位为 m/s。 */
    float fr_d;
} IIR_WheelSpeeds;

/**
 * @brief Return default geometry and speed configuration for the C50C differential chassis.
 *        返回 C50C 四轮差速底盘的默认几何与速度配置参数。
 *
 * @return Initialized IIR_KinematicsConfig structure.
 *         已初始化的 IIR_KinematicsConfig 结构体。
 */
IIR_KinematicsConfig IIR_Kinematics_DefaultConfig(void);

/**
 * @brief Clamp a body twist within configured safety bounds (NaN/Inf fail-closed to zero).
 *        将车体速度指令限制在配置的安全包络内（NaN/Inf 按故障安全原则归零）。
 *
 * @param[in] cfg Kinematics configuration; uses defaults when NULL.
 *                运动学配置指针；传入 NULL 时使用默认配置。
 * @param[in] in  Raw input body twist.
 *                原始输入车体速度。
 * @return Clamped body twist with vy_mps forced to 0.0f.
 *         限幅后的车体速度，且 vy_mps 强制置为 0.0f。
 */
IIR_BodyTwist IIR_Kinematics_ClampTwist(const IIR_KinematicsConfig *cfg, IIR_BodyTwist in);

/**
 * @brief Solve differential inverse kinematics from body twist to four wheel linear speeds.
 *        求解差速逆运动学，将车体速度解算为四个车轮的目标线速度。
 *
 * Applies proportional wheel-speed scaling when |vx +/- wz*L| exceeds max_wheel_speed_mps
 * so turning curvature is preserved.
 * 当 |vx +/- wz*L| 超出单轮上限时按比例缩减左右轮速，以保持转弯曲率不变。
 *
 * @param[in]  cfg   Kinematics configuration.
 *                   运动学配置指针。
 * @param[in]  twist Desired body velocity command.
 *                   期望的车体速度指令。
 * @param[out] out   Computed wheel linear velocities in m/s.
 *                   输出解算后的四轮线速度，单位为 m/s。
 * @return true on success; false when arguments or config values are invalid.
 *         求解成功返回 true；参数或配置非法时返回 false。
 */
bool IIR_Kinematics_Inverse(const IIR_KinematicsConfig *cfg,
                            IIR_BodyTwist twist,
                            IIR_WheelSpeeds *out);

/**
 * @brief Solve differential forward kinematics from four wheel speeds to body twist.
 *        求解差速正运动学，从四轮实测线速度恢复车体速度。
 *
 * @param[in]  cfg    Kinematics configuration.
 *                    运动学配置指针。
 * @param[in]  wheels Measured wheel linear velocities in m/s.
 *                    实测四轮线速度，单位为 m/s。
 * @param[out] out    Reconstructed body twist in m/s and rad/s.
 *                    输出重构的车体线速度（m/s）与角速度（rad/s）。
 * @return true on success; false when arguments or config values are invalid.
 *         求解成功返回 true；参数或配置非法时返回 false。
 */
bool IIR_Kinematics_Forward(const IIR_KinematicsConfig *cfg,
                            IIR_WheelSpeeds wheels,
                            IIR_BodyTwist *out);

/**
 * @brief Convert an encoder tick delta over dt_s seconds into wheel linear speed in m/s.
 *        将采样周期 dt_s 内的编码器脉冲增量换算为车轮线速度（m/s）。
 *
 * @param[in] cfg         Kinematics configuration containing wheel diameter and encoder scaling.
 *                        包含轮胎直径与编码器倍频参数的运动学配置。
 * @param[in] delta_ticks Signed quadrature tick delta during the control period.
 *                        控制周期内的有符号正交编码器脉冲增量。
 * @param[in] dt_s        Elapsed sampling interval in seconds (must be > 0).
 *                        采样时间间隔，单位为秒（必须大于 0）。
 * @return Wheel linear velocity in m/s, or 0.0f on invalid inputs.
 *         车轮线速度（m/s）；输入非法时返回 0.0f。
 */
float IIR_Kinematics_EncoderDeltaToMps(const IIR_KinematicsConfig *cfg,
                                       int32_t delta_ticks,
                                       float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* IIR_APP_KINEMATICS_DIFF_KINEMATICS_H */

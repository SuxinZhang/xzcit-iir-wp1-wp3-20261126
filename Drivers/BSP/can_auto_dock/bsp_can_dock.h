/**
 * @file bsp_can_dock.h
 * @brief CAN1 automatic infrared docking BSP driver on PD0(RX)/PD1(TX) at 1 Mbps.
 *        位于 PD0(RX)/PD1(TX)、波特率 1 Mbps 的 CAN1 四路红外自动回充板级驱动。
 */

#ifndef IIR_BSP_CAN_DOCK_H
#define IIR_BSP_CAN_DOCK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Standard CAN ID carrying charger infrared guidance and charging telemetry.
 *  回充充电桩下发四路红外对接与充电状态的标准 CAN 帧 ID（0x182）。 */
#define BSP_CAN_DOCK_STD_ID_STATUS 0x182u

/** Standard CAN ID used to configure charger infrared speed parameters.
 *  向回充充电桩下发对接速度配置的标准 CAN 帧 ID（0x105）。 */
#define BSP_CAN_DOCK_STD_ID_CONFIG 0x105u

/** Extended CAN ID used to probe auto-dock charger hardware presence.
 *  用于探测回充对接硬件是否在线的扩展 CAN 帧 ID（0x12345678）。 */
#define BSP_CAN_DOCK_EXT_ID_PROBE 0x12345678u

/** Maximum age of an infrared target permission in milliseconds.
 *  红外目标运动许可的最大有效期（毫秒）。 */
#define BSP_CAN_DOCK_PERMISSION_MS 300u

/**
 * @brief Auto-docking state, 4-channel infrared receiver mask, and charger current.
 *        自动回充模式状态、四路红外对接传感器状态与充电电流结构体。
 */
typedef struct {
    bool hw_present;
    bool allow_recharge;
    bool nav_walk;
    bool charging;
    uint8_t ir_mask;
    uint8_t red_state;
    int16_t charging_current_ma;
    float dock_vx_mps;
    float dock_vz_rad_s;
    float red_vx_mps;
    float red_vz_rad_s;
    uint32_t target_generation;
    uint32_t permitted_generation;
    uint32_t permission_at_ms;
    bool permission_valid;
    bool motion_revoked;
} BSP_CanDock_State;

/**
 * @brief Decode a CAN 0x182 8-byte payload into IR flags, charging bit, current, and twist.
 *        将 CAN 0x182 8 字节载荷解码为红外标志位、充电状态、电流与红外导引速度。
 */
void BSP_CanDock_ParseFrame182(const uint8_t data[8], BSP_CanDock_State *io_state);

/**
 * @brief Compute differential-drive target velocity during auto-docking mode.
 *        在自动回充模式下融合上位机接近指令与 CAN 四路红外导引，解算差速控制速度。
 *
 * @param[in]  state          Current auto-docking state structure.
 *                            当前自动回充状态结构体指针。
 * @param[in]  host_vx_mps    Host pre-dock X velocity command (m/s).
 *                            上位机 Nav2 接近阶段期望线速度（m/s）。
 * @param[in]  host_vz_rad_s  Host pre-dock yaw rate command (rad/s).
 *                            上位机 Nav2 接近阶段期望角速度（rad/s）。
 * @param[in]  ir_permitted   Permission for the current infrared target.
 *                            当前红外目标是否已获运动许可。
 * @param[out] out_vx_mps     Selected target X linear velocity (m/s).
 *                            仲裁后输出的控制 X 轴线速度（m/s）。
 * @param[out] out_vz_rad_s   Selected target yaw angular velocity (rad/s).
 *                            仲裁后输出的控制偏航角速度（rad/s）。
 */
void BSP_CanDock_ComputeTwist(const BSP_CanDock_State *state,
                              float host_vx_mps,
                              float host_vz_rad_s,
                              bool ir_permitted,
                              float *out_vx_mps,
                              float *out_vz_rad_s);

/**
 * @brief Update auto-dock enable flag and optional docking approach speeds.
 *        更新自动回充模式开关及对接速度配置（供 SET_AUTO_DOCK 指令调用）。
 */
void BSP_CanDock_SetEnabled(BSP_CanDock_State *io_state,
                            bool enable,
                            float dock_vx_mps,
                            float dock_vz_rad_s);

/** Grant and test permission for a specific infrared target generation.
 *  授予并检查指定红外目标代次的运动许可。 */
void BSP_CanDock_PermitTarget(BSP_CanDock_State *io_state, uint32_t generation,
                             uint32_t now_ms);
bool BSP_CanDock_TargetPermitted(const BSP_CanDock_State *state, uint32_t now_ms);

/** Revoke an expired target permission and keep motion stopped.
 *  撤销过期的目标许可并保持运动停止。 */
bool BSP_CanDock_ExpirePermission(BSP_CanDock_State *io_state, uint32_t now_ms);

/** Check that a pending PWM calculation still matches the live dock state.
 *  检查待输出的 PWM 计算是否仍与实时回充状态一致。 */
bool BSP_CanDock_OutputCurrent(const BSP_CanDock_State *snapshot,
                               const BSP_CanDock_State *live, uint32_t now_ms);

/** Invalidate an expired CAN target and revoke docking.
 *  作废超时的 CAN 目标并撤销回充授权。 */
void BSP_CanDock_InvalidateTarget(BSP_CanDock_State *io_state);

#ifdef USE_HAL_DRIVER
/**
 * @brief Initialize CAN1 peripheral (PD0=RX, PD1=TX, 1 Mbps) and probe dock hardware.
 *        初始化 CAN1 外设（PD0=RX, PD1=TX, 1 Mbps）并发送回充硬件探测报文。
 */
bool BSP_CanDock_Init(void);

/**
 * @brief Process pending CAN1 FIFO0 frames inside CAN1_RX0_IRQHandler.
 *        在 CAN1_RX0_IRQHandler 中读取并解析 FIFO0 接收到的回充状态报文。
 */
void BSP_CanDock_OnRxIrq(void);

/**
 * @brief Periodic 20 Hz auto-dock offline monitor and CAN config refresh.
 *        20 Hz 周期性检查回充装备在线状态并下发配置速度。
 */
void BSP_CanDock_PeriodicTick(uint32_t now_ms);

/**
 * @brief Apply SET_AUTO_DOCK command safely to the active global CAN dock state.
 *        安全地向全局 CAN 自动回充状态写入 SET_AUTO_DOCK 启停与速度参数。
 */
void BSP_CanDock_ApplyHostCommand(bool enable, float dock_vx_mps, float dock_vz_rad_s);

/** Apply a host permission to the global docking state.
 *  向全局回充状态应用上位机目标许可。 */
void BSP_CanDock_ApplyTargetPermission(uint32_t generation, uint32_t now_ms);

/** Expire a permission under the same interrupt lock used by CAN reception.
 *  在与 CAN 接收相同的中断锁内处理许可超时。 */
void BSP_CanDock_ExpireGlobalPermission(uint32_t now_ms);

/** Check the live state while the caller holds the CAN interrupt mask.
 *  调用方屏蔽 CAN 中断时检查实时回充状态。 */
bool BSP_CanDock_GlobalOutputCurrent(const BSP_CanDock_State *snapshot, uint32_t now_ms);

/**
 * @brief Copy the latest atomic auto-dock state snapshot safely.
 *        原子读取当前 CAN 自动回充状态与红外对接速度快照。
 */
void BSP_CanDock_GetState(BSP_CanDock_State *out_state);
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif

#endif /* IIR_BSP_CAN_DOCK_H */

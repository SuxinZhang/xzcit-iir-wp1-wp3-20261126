/**
 * @file bsp_can_dock.c
 * @brief Implementation of CAN1 (PD0/PD1, 1 Mbps) auto-docking BSP driver and
 *        differential infrared docking solver.
 *        基于 STM32F4 CAN1（PD0/PD1, 1 Mbps）的自动回充硬件驱动与差速红外对接逻辑实现。
 */

#include "can_auto_dock/bsp_can_dock.h"

#ifdef USE_HAL_DRIVER
#include "main.h"
#endif

/* CAN1 is initialized and exchanged through direct bxCAN registers below, not
 * HAL CAN APIs. Keep HAL_CAN_MODULE_ENABLED disabled and do not add a HAL CAN
 * source file unless this driver is intentionally migrated to HAL.
 * CAN1 的初始化与收发均在下方直接操作 bxCAN 寄存器，不使用 HAL CAN API；
 * 保持 HAL_CAN_MODULE_ENABLED 关闭，除非有意迁移到 HAL，否则不要添加 HAL CAN 源文件。 */

#include <stddef.h>
#include <string.h>

#define DOCK_DEFAULT_VX_MPS    (-0.10f)
#define DOCK_DEFAULT_VZ_RAD_S  0.20f
#define DOCK_OFFLINE_MS        1000u

void BSP_CanDock_ParseFrame182(const uint8_t data[8], BSP_CanDock_State *io_state)
{
    if (data == NULL || io_state == NULL) {
        return;
    }

    const int16_t raw_vx = (int16_t)(((uint16_t)data[0] << 8u) | (uint16_t)data[1]);
    const int16_t raw_vz = (int16_t)(((uint16_t)data[4] << 8u) | (uint16_t)data[5]);

    const uint8_t status_byte = data[6];
    const uint8_t charging = (uint8_t)(status_byte & 0x01u);
    const uint8_t r_a = (uint8_t)((status_byte >> 2u) & 0x01u);
    const uint8_t r_b = (uint8_t)((status_byte >> 3u) & 0x01u);
    const uint8_t l_b = (uint8_t)((status_byte >> 4u) & 0x01u);
    const uint8_t l_a = (uint8_t)((status_byte >> 5u) & 0x01u);
    const uint8_t red_sum = (uint8_t)(r_a + r_b + l_b + l_a);
    const float target_vx = (red_sum == 0u || charging != 0u) ? 0.0f : (float)raw_vx / 1000.0f;
    const float target_vz = (red_sum == 0u || charging != 0u) ? 0.0f : (float)raw_vz / 1000.0f;
    if (target_vx != io_state->red_vx_mps || target_vz != io_state->red_vz_rad_s ||
        red_sum != io_state->red_state || (charging != 0u) != io_state->charging) {
        io_state->target_generation++;
        if (io_state->permission_valid) {
            io_state->motion_revoked = true;
        }
        io_state->permission_valid = false;
    }

    io_state->charging = (charging != 0u);
    io_state->red_state = red_sum;
    io_state->ir_mask = (uint8_t)((r_a << 0u) | (r_b << 1u) | (l_b << 2u) | (l_a << 3u) |
                                  ((red_sum & 0x07u) << 4u));

    if (red_sum == 0u || charging != 0u) {
        io_state->red_vx_mps = 0.0f;
        io_state->red_vz_rad_s = 0.0f;
    } else {
        io_state->red_vx_mps = (float)raw_vx / 1000.0f;
        io_state->red_vz_rad_s = (float)raw_vz / 1000.0f;
        /* Hand off from host pre-dock approach to precision infrared guidance once IR hits.
         * 一旦检测到有效四路红外对接信号，自动退出上位机巡航接管并移交红外闭环精对接。 */
        io_state->nav_walk = false;
    }

    /* Decode signed 8-bit charging current scaled at 30 mA per LSB (matching auto_recharge.c).
     * 按照 WHEELTEC 协议解码 30 mA/LSB 的有符号充电电流。 */
    if (data[7] > 128u) {
        io_state->charging_current_ma = (int16_t)(-(int32_t)(256u - data[7]) * 30);
    } else {
        io_state->charging_current_ma = (int16_t)((int32_t)data[7] * 30);
    }
}

void BSP_CanDock_ComputeTwist(const BSP_CanDock_State *state,
                              float host_vx_mps,
                              float host_vz_rad_s,
                              bool ir_permitted,
                              float *out_vx_mps,
                              float *out_vz_rad_s)
{
    if (out_vx_mps == NULL || out_vz_rad_s == NULL) {
        return;
    }

    if (state != NULL && state->motion_revoked) {
        *out_vx_mps = 0.0f;
        *out_vz_rad_s = 0.0f;
        return;
    }
    if (state == NULL || !state->allow_recharge) {
        *out_vx_mps = host_vx_mps;
        *out_vz_rad_s = host_vz_rad_s;
        return;
    }

    /* Stop immediately once physical charger contact & active charging begins.
     * 一旦充电桩触片接触并开始充电，立即强制零速停稳。 */
    if (state->charging) {
        *out_vx_mps = 0.0f;
        *out_vz_rad_s = 0.0f;
        return;
    }

    if (state->red_state != 0u && ir_permitted) {
        *out_vx_mps = state->red_vx_mps;
        *out_vz_rad_s = state->red_vz_rad_s;
    } else if (state->red_state == 0u && state->nav_walk) {
        *out_vx_mps = host_vx_mps;
        *out_vz_rad_s = host_vz_rad_s;
    } else {
        *out_vx_mps = 0.0f;
        *out_vz_rad_s = 0.0f;
    }
}

void BSP_CanDock_SetEnabled(BSP_CanDock_State *io_state,
                            bool enable,
                            float dock_vx_mps,
                            float dock_vz_rad_s)
{
    if (io_state == NULL) {
        return;
    }

    if (io_state->dock_vx_mps == 0.0f) {
        io_state->dock_vx_mps = DOCK_DEFAULT_VX_MPS;
    }
    if (io_state->dock_vz_rad_s == 0.0f) {
        io_state->dock_vz_rad_s = DOCK_DEFAULT_VZ_RAD_S;
    }

    if (dock_vx_mps != 0.0f) {
        io_state->dock_vx_mps = dock_vx_mps;
    }
    if (dock_vz_rad_s != 0.0f) {
        io_state->dock_vz_rad_s = dock_vz_rad_s;
    }

    if (enable && !io_state->allow_recharge) {
        io_state->target_generation++;
    }
    io_state->allow_recharge = enable;
    io_state->permission_valid = false;
    io_state->motion_revoked = false;
    if (enable) {
        io_state->nav_walk = (io_state->red_state == 0u);
    } else {
        io_state->nav_walk = false;
    }
}

void BSP_CanDock_PermitTarget(BSP_CanDock_State *io_state, uint32_t generation,
                             uint32_t now_ms)
{
    if (io_state->allow_recharge && io_state->red_state != 0u &&
        generation == io_state->target_generation) {
        io_state->permitted_generation = generation;
        io_state->permission_at_ms = now_ms;
        io_state->permission_valid = true;
        io_state->motion_revoked = false;
    }
}

bool BSP_CanDock_TargetPermitted(const BSP_CanDock_State *state, uint32_t now_ms)
{
    return state->allow_recharge && state->permission_valid &&
           state->permitted_generation == state->target_generation &&
           (uint32_t)(now_ms - state->permission_at_ms) <= BSP_CAN_DOCK_PERMISSION_MS;
}

bool BSP_CanDock_ExpirePermission(BSP_CanDock_State *io_state, uint32_t now_ms)
{
    if (!io_state->permission_valid || BSP_CanDock_TargetPermitted(io_state, now_ms)) {
        return false;
    }
    io_state->allow_recharge = false;
    io_state->permission_valid = false;
    io_state->nav_walk = false;
    io_state->motion_revoked = true;
    return true;
}

bool BSP_CanDock_OutputCurrent(const BSP_CanDock_State *snapshot,
                               const BSP_CanDock_State *live, uint32_t now_ms)
{
    if (live->motion_revoked || snapshot->motion_revoked ||
        snapshot->allow_recharge != live->allow_recharge) {
        return false;
    }
    if (snapshot->allow_recharge) {
        if (snapshot->target_generation != live->target_generation ||
            snapshot->charging != live->charging) {
            return false;
        }
        if (snapshot->red_state != 0u &&
            !BSP_CanDock_TargetPermitted(live, now_ms)) {
            return false;
        }
    }
    return true;
}

void BSP_CanDock_InvalidateTarget(BSP_CanDock_State *io_state)
{
    if (io_state->red_state != 0u || io_state->red_vx_mps != 0.0f ||
        io_state->red_vz_rad_s != 0.0f) {
        io_state->target_generation++;
    }
    if (io_state->allow_recharge || io_state->permission_valid ||
        io_state->red_state != 0u) {
        io_state->motion_revoked = true;
    }
    io_state->red_state = 0u;
    io_state->ir_mask = 0u;
    io_state->allow_recharge = false;
    io_state->nav_walk = false;
    io_state->permission_valid = false;
    io_state->red_vx_mps = 0.0f;
    io_state->red_vz_rad_s = 0.0f;
}

#ifdef USE_HAL_DRIVER
static BSP_CanDock_State s_dock_state;
static volatile uint32_t s_last_182_ms = 0u;
static uint32_t s_last_tick_tx_ms = 0u;

static bool can1_send_frame(uint32_t id, bool is_ext, const uint8_t data[8])
{
    /* Wait briefly for at least one free TX mailbox (up to ~1 ms).
     * 轮询检查 CAN 发送邮箱空位（短时限定以免阻塞控制循环）。 */
    uint32_t wait = 0u;
    while ((CAN1->TSR & CAN_TSR_TME) == 0u && wait < 5000u) {
        wait++;
    }
    if ((CAN1->TSR & CAN_TSR_TME) == 0u) {
        return false;
    }

    const uint32_t mbox = (CAN1->TSR & CAN_TSR_CODE_Msk) >> CAN_TSR_CODE_Pos;
    CAN_TxMailBox_TypeDef *tx = &CAN1->sTxMailBox[mbox];

    if (is_ext) {
        tx->TIR = ((id & 0x1FFFFFFFu) << CAN_TI0R_EXID_Pos) | CAN_TI0R_IDE;
    } else {
        tx->TIR = ((id & 0x7FFu) << CAN_TI0R_STID_Pos);
    }
    tx->TDTR = 8u;
    tx->TDLR = ((uint32_t)data[3] << 24u) |
               ((uint32_t)data[2] << 16u) |
               ((uint32_t)data[1] << 8u) |
               (uint32_t)data[0];
    tx->TDHR = ((uint32_t)data[7] << 24u) |
               ((uint32_t)data[6] << 16u) |
               ((uint32_t)data[5] << 8u) |
               (uint32_t)data[4];
    tx->TIR |= CAN_TI0R_TXRQ;
    return true;
}

static void send_dock_probe(void)
{
    static const uint8_t probe[8] = {1u, 1u, 1u, 1u, 1u, 1u, 1u, 1u};
    (void)can1_send_frame(BSP_CAN_DOCK_EXT_ID_PROBE, true, probe);
}

static void send_dock_config(const BSP_CanDock_State *st)
{
    const int16_t vx_mm = (int16_t)(st->dock_vx_mps * 1000.0f);
    const int16_t vz_mrad = (int16_t)(st->dock_vz_rad_s * 1000.0f);
    uint8_t msg[8] = {
        0u,
        (uint8_t)(((uint16_t)vx_mm >> 8u) & 0xFFu),
        (uint8_t)((uint16_t)vx_mm & 0xFFu),
        0u,
        0u,
        (uint8_t)(((uint16_t)vz_mrad >> 8u) & 0xFFu),
        (uint8_t)((uint16_t)vz_mrad & 0xFFu),
        0u
    };
    (void)can1_send_frame(BSP_CAN_DOCK_STD_ID_CONFIG, false, msg);
}

bool BSP_CanDock_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    memset(&s_dock_state, 0, sizeof(s_dock_state));
    s_dock_state.dock_vx_mps = DOCK_DEFAULT_VX_MPS;
    s_dock_state.dock_vz_rad_s = DOCK_DEFAULT_VZ_RAD_S;
    s_last_182_ms = 0u;
    s_last_tick_tx_ms = 0u;

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* Configure PD0 (CAN1_RX) and PD1 (CAN1_TX) per C50C-V4.3 schematic.
     * 按照 C50C-V4.3 原理图配置 PD0（CAN1_RX）与 PD1（CAN1_TX）复用功能 AF9。 */
    gpio.Pin = IIR_CAN1_RX_Pin | IIR_CAN1_TX_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(IIR_CAN1_GPIO_Port, &gpio);

    /* Request bxCAN initialization mode (exit SLEEP, set INRQ).
     * 退出 CAN 睡眠模式并进入 bxCAN 初始化模式配置波特率与接收滤波器。 */
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    CAN1->MCR |= CAN_MCR_INRQ;
    uint32_t wait = 0u;
    while ((CAN1->MSR & CAN_MSR_INAK) == 0u && wait < 100000u) {
        wait++;
    }
    if ((CAN1->MSR & CAN_MSR_INAK) == 0u) {
        return false;
    }

    CAN1->MCR &= ~(CAN_MCR_TTCM | CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_RFLM | CAN_MCR_TXFP);
    CAN1->MCR |= CAN_MCR_NART;

    /* 42 MHz APB1 / ((1 + 3 + 3) tq * 6) = 1.000 Mbps matching WHEELTEC_C50X CAN timing.
     * 按 42 MHz APB1 配置 1 Mbps 波特率（SJW=1tq, BS1=3tq, BS2=3tq, BRP=6），匹配充电桩。 */
    CAN1->BTR = ((0u) << CAN_BTR_SJW_Pos) |
                ((2u) << CAN_BTR_TS1_Pos) |
                ((2u) << CAN_BTR_TS2_Pos) |
                (5u);

    /* Configure filter bank 0 as 32-bit accept-all routed to FIFO0.
     * 将 0 号过滤器配置为 32 位全通掩码模式并关联至接收 FIFO0。 */
    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R &= ~1u;
    CAN1->FM1R &= ~1u;
    CAN1->FS1R |= 1u;
    CAN1->FFA1R &= ~1u;
    CAN1->sFilterRegister[0].FR1 = 0u;
    CAN1->sFilterRegister[0].FR2 = 0u;
    CAN1->FA1R |= 1u;
    CAN1->FMR &= ~CAN_FMR_FINIT;

    /* Enable FIFO0 message pending interrupt and start normal CAN operation.
     * 使能 CAN1 FIFO0 报文挂起中断并切换至正常总线收发模式。 */
    CAN1->IER |= CAN_IER_FMPIE0;
    CAN1->MCR &= ~CAN_MCR_INRQ;
    wait = 0u;
    while ((CAN1->MSR & CAN_MSR_INAK) != 0u && wait < 100000u) {
        wait++;
    }
    if ((CAN1->MSR & CAN_MSR_INAK) != 0u) {
        return false;
    }

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    send_dock_probe();
    return true;
}

void BSP_CanDock_OnRxIrq(void)
{
    while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0u) {
        const CAN_FIFOMailBox_TypeDef *rx = &CAN1->sFIFOMailBox[0];
        const uint32_t rir = rx->RIR;
        const uint32_t rdlr = rx->RDLR;
        const uint32_t rdhr = rx->RDHR;
        const bool is_ext = ((rir & CAN_RI0R_IDE) != 0u);

        uint8_t payload[8];
        payload[0] = (uint8_t)(rdlr & 0xFFu);
        payload[1] = (uint8_t)((rdlr >> 8u) & 0xFFu);
        payload[2] = (uint8_t)((rdlr >> 16u) & 0xFFu);
        payload[3] = (uint8_t)((rdlr >> 24u) & 0xFFu);
        payload[4] = (uint8_t)(rdhr & 0xFFu);
        payload[5] = (uint8_t)((rdhr >> 8u) & 0xFFu);
        payload[6] = (uint8_t)((rdhr >> 16u) & 0xFFu);
        payload[7] = (uint8_t)((rdhr >> 24u) & 0xFFu);

        /* Release FIFO0 mailbox immediately after latching payload words.
         * 拷贝接收邮箱数据后立即释放 FIFO0 邮箱槽位。 */
        CAN1->RF0R |= CAN_RF0R_RFOM0;

        if (!is_ext) {
            const uint32_t std_id = (rir >> CAN_RI0R_STID_Pos) & 0x7FFu;
            if (std_id == BSP_CAN_DOCK_STD_ID_STATUS) {
                BSP_CanDock_ParseFrame182(payload, &s_dock_state);
                s_dock_state.hw_present = true;
                s_last_182_ms = HAL_GetTick();
            }
        } else {
            const uint32_t ext_id = (rir >> CAN_RI0R_EXID_Pos) & 0x1FFFFFFFu;
            if (ext_id == BSP_CAN_DOCK_EXT_ID_PROBE) {
                uint16_t sum = 0u;
                for (uint8_t i = 0u; i < 8u; ++i) {
                    sum = (uint16_t)(sum + payload[i]);
                }
                if (sum == 8u) {
                    s_dock_state.hw_present = true;
                }
            }
        }
    }
}

void BSP_CanDock_PeriodicTick(uint32_t now_ms)
{
    if (s_dock_state.hw_present &&
        s_last_182_ms != 0u &&
        (uint32_t)(now_ms - s_last_182_ms) > DOCK_OFFLINE_MS) {
        __disable_irq();
        BSP_CanDock_InvalidateTarget(&s_dock_state);
        __enable_irq();
    }

    if ((uint32_t)(now_ms - s_last_tick_tx_ms) >= 50u) {
        s_last_tick_tx_ms = now_ms;
        if (s_dock_state.hw_present) {
            BSP_CanDock_State snap;
            BSP_CanDock_GetState(&snap);
            send_dock_config(&snap);
        } else {
            send_dock_probe();
        }
    }
}

void BSP_CanDock_ApplyHostCommand(bool enable, float dock_vx_mps, float dock_vz_rad_s)
{
    __disable_irq();
    BSP_CanDock_SetEnabled(&s_dock_state, enable, dock_vx_mps, dock_vz_rad_s);
    __enable_irq();

    if (enable && !s_dock_state.hw_present) {
        send_dock_probe();
    }
}

void BSP_CanDock_ApplyTargetPermission(uint32_t generation, uint32_t now_ms)
{
    __disable_irq();
    BSP_CanDock_PermitTarget(&s_dock_state, generation, now_ms);
    __enable_irq();
}

void BSP_CanDock_ExpireGlobalPermission(uint32_t now_ms)
{
    __disable_irq();
    (void)BSP_CanDock_ExpirePermission(&s_dock_state, now_ms);
    __enable_irq();
}

bool BSP_CanDock_GlobalOutputCurrent(const BSP_CanDock_State *snapshot, uint32_t now_ms)
{
    return BSP_CanDock_OutputCurrent(snapshot, &s_dock_state, now_ms);
}

void BSP_CanDock_GetState(BSP_CanDock_State *out_state)
{
    if (out_state == NULL) {
        return;
    }
    __disable_irq();
    *out_state = s_dock_state;
    __enable_irq();
}
#endif /* USE_HAL_DRIVER */

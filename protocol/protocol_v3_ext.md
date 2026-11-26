# XZCIT-IIR Protocol v3 Extended Wire Specification (`protocol_v3_ext`)

> **Single Source of Truth**: This contract specifies all uplink and downlink frames between the STM32F407VET6 (`C50C-V4.3`) chassis firmware and the Orange Pi (`Master/ROS/ros2_ws/src/c50c_uart3/`) host bridge over `USART3` (`115200 8N1`). All WP1/WP2 extensions maintain byte-level backward compatibility with `c50c_uart3`.

---

## 1. Frame Format Overview

| Direction | Frame Type | Length | Header | Cmd / Sub-ID | Tail | Checksum | Rate |
| :--- | :--- | :---: | :---: | :---: | :---: | :--- | :---: |
| Uplink (STM32 -> Host) | Primary Status Telemetry | 50 B | `0x7B` | `stop_flag` at `[1]` | `0x7D` | `BCC = XOR(buf[0..47])` at `[48]` | 20 Hz |
| Uplink (STM32 -> Host) | v3 Session Advertisement | 11 B | `0x7B` | `0x7F`, sub `0x01`, ver `3` | `0x7D` | `BCC = XOR(buf[0..8])` at `[9]` | 1 Hz |
| Uplink (STM32 -> Host) | Extended Safety & Auto-Dock Status | 11 B | `0x7C` | `0x7E`, ver `3` | `0x7E` | `BCC = XOR(buf[0..8])` at `[9]` | 10 Hz |
| Uplink (STM32 -> Host) | Infrared target snapshot | 14 B | `0x7C` | `0x7D`, ver `3` | `0x7E` | `BCC = XOR(buf[0..11])` at `[12]` | 9–10 Hz |
| Downlink (Host -> STM32) | 3-Axis Velocity Command | 11 B | `0x7B` | `0x00`, `0x80\|seq` (v3) | `0x7D` | `BCC = XOR(buf[0..8])` at `[9]` | 20–50 Hz |
| Downlink (Host -> STM32) | v3 Session Acknowledgement | 11 B | `0x7B` | `0x7F`, sub `0x01`, ver `3` | `0x7D` | `BCC = XOR(buf[0..8])` at `[9]` | On connect / 1 Hz |
| Downlink (Host -> STM32) | `SET_AUTO_DOCK(enable)` | 11 B | `0x7B` | `0x01`, `0x80\|seq` (or `0x01`) | `0x7D` | `BCC = XOR(buf[0..8])` at `[9]` | Event / Command |
| Downlink (Host -> STM32) | Host stop | 11 B | `0x7B` | `0x02`, `0x80\|seq` | `0x7D` | `BCC = XOR(buf[0..8])` at `[9]` | While stopped |
| Downlink (Host -> STM32) | Infrared target permission | 11 B | `0x7B` | `0x03`, `0x80\|seq` | `0x7D` | `BCC = XOR(buf[0..8])` at `[9]` | While permitted |
| Downlink (Host -> STM32) | `SET_SOS(active)` | 11 B | `0x7B` | `0x04`, `0x80\|seq` (or `0x01`) | `0x7D` | `BCC = XOR(buf[0..8])` at `[9]` | Event / Command |
| Downlink (Host -> STM32) | `SET_LED_BEEP(mode)` | 11 B | `0x7B` | `0x05`, `0x80\|seq` (or `0x01`) | `0x7D` | `BCC = XOR(buf[0..8])` at `[9]` | Event / Command |

---

## 2. Uplink 50-Byte Primary Status Frame (`0x7B .. BCC .. 0x7D`, 20 Hz)

Multi-byte integers are encoded in **big-endian (network) byte order**.

| Byte Offset | Type | Field | Semantics & Scaling |
| :---: | :---: | :--- | :--- |
| `0` | `uint8` | `frame_header` | Fixed `0x7B`. |
| `1` | `uint8` | `stop_flag` | `0x00` = motion enabled; non-zero = active `IIR_SafetyStopReason` bitmask:<br>• `bit0 (0x01)`: `PE4` hardware E-stop<br>• `bit1 (0x02)`: Physical `PE5` or remote `SET_SOS(1)` active<br>• `bit2 (0x04)`: `USART3` command watchdog timeout (`> 500 ms`)<br>• `bit3 (0x08)`: Severe battery undervoltage (`< 20500 mV`)<br>• `bit4 (0x10)`: `D50A` RS485 motor driver protection fault<br>• `bit5 (0x20)`: Host stop command active |
| `2..3` | `int16 BE` | `vx_mm_s` | Forward kinematics chassis X linear velocity (`mm/s`). |
| `4..5` | `int16 BE` | `vy_mm_s` | Chassis Y linear velocity (`mm/s`; fixed `0` for 4WD differential). |
| `6..7` | `int16 BE` | `vz_mrad_s` | Forward kinematics chassis yaw rate (`mrad/s`, CCW positive). |
| `8..13` | `3 x int16 BE` | `accel_lsb[3]` | `ICM-20948` raw accelerometer LSB ($\pm 2g$, rotated to ROS axes `X=+chipY, Y=-chipX, Z=+chipZ`, startup zero-bias compensated with `+16384` Z gravity reference). |
| `14..19` | `3 x int16 BE` | `gyro_lsb[3]` | `ICM-20948` raw gyroscope LSB ($\pm 500\text{ dps}$, rotated to ROS axes, zero-bias removed). `gyro_lsb[2]` is forced to `0` whenever `stop_flag != 0`. |
| `20..21` | `uint16 BE` | `voltage_mv` | Filtered battery pack voltage (`mV`). |
| `22..37` | `4 x int32 BE` | `encoder_counts[4]` | Per-wheel speed/odometry telemetry integers (`mm/s`) for channels A, B, C, D. |
| `38..45` | `4 x uint16 BE` | `pwm_counts[4]` | Actual `TIM8->CCR4..CCR1` hardware compare values applied through `Motor_ApplyOutput`. |
| `46..47` | `uint16 BE` | `ultrasonic_front_mm` | Front `HC-SR04` (`PE9`/`PE11`) median-filtered distance in `mm` (`20..4500`). `0xFFFF` indicates timeout (`>= 28 ms`), out-of-range, or invalid reading. |
| `48` | `uint8` | `bcc` | `XOR` of bytes `0..47`. |
| `49` | `uint8` | `frame_tail` | Fixed `0x7D`. |

> **Compatibility Note**: Legacy PS2 gamepad control and PS2 disconnect gating are permanently removed. Local chassis safety depends exclusively on `PE4`, `PE5`, `USART3` watchdog, battery voltage, and `D50A` fault flags.

---

## 3. Uplink 11-Byte Extended Safety & Auto-Dock Frame (`0x7C .. BCC .. 0x7E`, 10 Hz)

The host first separates complete frames with `UplinkStreamParser`, so byte patterns inside a valid payload are not treated as separate frames.

| Byte Offset | Type | Field | Semantics |
| :---: | :---: | :--- | :--- |
| `0` | `uint8` | `ext_header` | Fixed `0x7C` (`IIR_PROTO_EXT_HEADER`). |
| `1` | `uint8` | `cmd_id` | Fixed `0x7E` (`IIR_PROTO_CMD_EXT_STATUS`). |
| `2` | `uint8` | `proto_version` | Fixed `0x03` (`IIR_PROTO_VERSION`). |
| `3` | `uint8` | `charging_state` | Bitmask:<br>• `bit0 (0x01)`: `charging` (actively charging)<br>• `bit1 (0x02)`: `allow_recharge` (auto-dock mode enabled)<br>• `bit2 (0x04)`: `dock_hw_present` (`CAN1` `0x12345678` handshake detected)<br>• `bit3 (0x08)`: `nav_walk` (host Nav2 pre-dock approach active) |
| `4..5` | `int16 BE` | `charging_current_ma` | Charging current in `mA` (signed big-endian). |
| `6` | `uint8` | `ir_mask` | CAN infrared docking receivers:<br>• `bit0 (0x01)`: `R_A`<br>• `bit1 (0x02)`: `R_B`<br>• `bit2 (0x04)`: `L_B`<br>• `bit3 (0x08)`: `L_A`<br>• `bits[6:4]`: `red_state` count (`0..4`) |
| `7` | `uint8` | `safety_flags` | Active safety conditions:<br>• `bit0 (0x01)`: `sos_active`<br>• `bit1 (0x02)`: `pe4_estop`<br>• `bit2 (0x04)`: `uart_timeout`<br>• `bit3 (0x08)`: `undervoltage`<br>• `bit4 (0x10)`: `d50a_fault`<br>• `bit5 (0x20)`: `host_stopped` |
| `8` | `uint8` | `d50a_alarm_mask` | Normalized active-HIGH D50A RS485 fault bits:<br>• `bit0 (0x01)`: `overcurrent` (inverted from D50A active-LOW bit0)<br>• `bit1 (0x02)`: `overtemp` (inverted from bit1)<br>• `bit2 (0x04)`: `overvoltage` (inverted from bit2)<br>• `bit3 (0x08)`: `undervoltage` (inverted from bit3)<br>• `bit7 (0x80)`: `offline` (`USART2` D50A poll timeout/unresponsive) |
| `9` | `uint8` | `bcc` | `XOR` of bytes `0..8`. |
| `10` | `uint8` | `ext_tail` | Fixed `0x7E` (`IIR_PROTO_EXT_TAIL`). |

---

### Infrared target snapshot (`0x7C 0x7D`, 14 bytes)

Bytes `2..3` contain version `3` and active (`0` or `1`). Bytes `4..7` contain the unsigned target generation. Bytes `8..9` and `10..11` contain signed big-endian target velocity in mm/s and yaw rate in mrad/s. Byte `12` is the XOR of bytes `0..11`; byte `13` is `0x7E`. The host binds its direction check to this generation and to the observed boot session. A changed target or a new docking enable advances the generation and invalidates prior permission.

## 4. Downlink 11-Byte Extended Control Frames (`0x7B .. BCC .. 0x7D`)

All downlink extended commands reuse the 11-byte `0x7B .. BCC .. 0x7D` envelope:

- `buf[0] = 0x7B`
- `buf[1] = cmd_id`
- `buf[2]`:
  - **v3 Sequenced Mode**: `0x80 | (seq & 0x7F)` — validated through the shared session + 7-bit sliding-window sequence gate (`IIR_PROTO_SEQ_WINDOW = 64`).
  - **Unsequenced Service Mode**: `0x01` (`IIR_PROTO_SESSION_SUB`) — permitted only when `ctx->session_acked` is true (v3 session already established) or `require_v3_session == false` (baseline compatibility mode).
- `buf[9] = XOR(buf[0..8])`
- `buf[10] = 0x7D`

`0x02` is a sequenced host stop. It cuts motor output and revokes docking. An ordinary zero velocity frame remains the docking handoff. `0x03` is a sequenced infrared permission with the checked generation in bytes `3..6`. The firmware accepts it only for the current target and expires it after 300 ms. Communication timeout revokes docking. A successful serial write does not confirm firmware acceptance.

### 4.1 `SET_AUTO_DOCK(enable)` (`cmd_id = 0x01`)

| Byte Offset | Type | Field | Semantics |
| :---: | :---: | :--- | :--- |
| `1` | `uint8` | `cmd_id` | `0x01` (`IIR_PROTO_CMD_AUTO_DOCK`). |
| `3` | `uint8` | `enable` | `1` = enter/enable auto-dock state machine; `0` = exit auto-dock mode. |
| `4..5` | `int16 BE` | `dock_vx_mm_s` | Optional configured docking approach velocity in `mm/s` (`0` keeps firmware default `-100 mm/s`). |
| `6..7` | `int16 BE` | `dock_vz_mrad_s` | Optional configured docking yaw velocity in `mrad/s` (`0` keeps firmware default `200 mrad/s`). |
| `8` | `uint8` | `reserved` | `0x00`. |

### 4.2 `SET_SOS(active)` (`cmd_id = 0x04`)

| Byte Offset | Type | Field | Semantics |
| :---: | :---: | :--- | :--- |
| `1` | `uint8` | `cmd_id` | `0x04` (`IIR_PROTO_CMD_SET_SOS`). |
| `3` | `uint8` | `active` | `1` = trigger remote SOS latch (immediate PWM cutoff + strobe/buzzer); `0` = reset remote SOS and clear physical `PE5` SOS latch if `PE5` button is released. |
| `4..8` | `uint8[5]` | `reserved` | `0x00`. |

### 4.3 `SET_LED_BEEP(mode)` (`cmd_id = 0x05`)

| Byte Offset | Type | Field | Semantics |
| :---: | :---: | :--- | :--- |
| `1` | `uint8` | `cmd_id` | `0x05` (`IIR_PROTO_CMD_SET_LED_BEEP`). |
| `3` | `uint8` | `mode` | LED override mode (`0` = release to automatic safety/heartbeat indicator, `1` = red strobe, `2` = yellow warning, `3` = green solid, `4` = blue dock indicator, `5` = all off). **Physical/remote SOS always overrides LED and buzzer to SOS alarm.** |
| `4` | `uint8` | `beep_enable` | `1` = enable remote buzzer override (when no higher-priority SOS override is active); `0` = release/silence. |
| `5..8` | `uint8[4]` | `reserved` | `0x00`. |

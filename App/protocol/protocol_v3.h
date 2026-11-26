/**
 * @file protocol_v3.h
 * @brief Pure-C protocol_v3 framing codec and command watchdog compatible with
 *        Master/ROS/ros2_ws/src/c50c_uart3.
 *        与 Master/ROS/ros2_ws/src/c50c_uart3 字节级兼容的纯 C protocol_v3 编解码与超时看门狗。
 */

#ifndef IIR_APP_PROTOCOL_PROTOCOL_V3_H
#define IIR_APP_PROTOCOL_PROTOCOL_V3_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Frame start delimiter byte (0x7B).
 *  帧头定界字节（0x7B）。 */
#define IIR_PROTO_FRAME_HEADER 0x7Bu

/** Frame end delimiter byte (0x7D).
 *  帧尾定界字节（0x7D）。 */
#define IIR_PROTO_FRAME_TAIL 0x7Du

/** Fixed byte length of uplink/downlink command and session frames.
 *  上行会话帧与下行控制/会话确认帧的固定字节长度（11 字节）。 */
#define IIR_PROTO_CMD_FRAME_LEN 11u

/** Fixed byte length of uplink chassis status telemetry frames.
 *  上行底盘状态遥测帧的固定字节长度（50 字节）。 */
#define IIR_PROTO_STATUS_FRAME_LEN 50u

/** Command ID for 3-axis target velocity command frames.
 *  三轴目标速度控制帧命令字（0x00）。 */
#define IIR_PROTO_CMD_SPEED 0x00u

/** Command ID for automatic infrared docking control frames.
 *  自主红外回充启停与对接速度配置命令字（0x01）。 */
#define IIR_PROTO_CMD_AUTO_DOCK 0x01u

/** Sequenced host stop command; revokes automatic docking authorization.
 *  带序号的上位机锁停命令，撤销自动回充授权。 */
#define IIR_PROTO_CMD_STOP 0x02u

/** Sequenced permission for one infrared target generation.
 *  针对一个红外目标代次的带序号运动许可。 */
#define IIR_PROTO_CMD_DOCK_PERMISSION 0x03u

/** Command ID for remote software SOS trigger/reset frames.
 *  上位机远程 SOS 紧急报警触发与复位命令字（0x04）。 */
#define IIR_PROTO_CMD_SET_SOS 0x04u

/** Command ID for remote LED and buzzer override frames.
 *  上位机远程 RGB 灯效与蜂鸣器提示控制命令字（0x05）。 */
#define IIR_PROTO_CMD_SET_LED_BEEP 0x05u

/** Frame start delimiter byte (0x7C) for 11-byte extended telemetry frames.
 *  11 字节扩展安全与回充遥测帧的帧头定界字节（0x7C）。 */
#define IIR_PROTO_EXT_HEADER 0x7Cu

/** Command ID for 11-byte extended telemetry frames.
 *  11 字节扩展安全与回充遥测帧命令字（0x7E）。 */
#define IIR_PROTO_CMD_EXT_STATUS 0x7Eu

/** Infrared target telemetry type and frame length.
 *  红外目标遥测类型及帧长度。 */
#define IIR_PROTO_CMD_DOCK_TARGET 0x7Du
#define IIR_PROTO_DOCK_TARGET_FRAME_LEN 14u

/** Version telemetry frame type and fixed 18-byte length for OTA verification.
 *  用于 OTA 版本校验的版本遥测帧类型与固定 18 字节长度。 */
#define IIR_PROTO_CMD_VERSION 0x06u
#define IIR_PROTO_VERSION_FRAME_LEN 18u

/** Frame end delimiter byte (0x7E) for 11-byte extended telemetry frames.
 *  11 字节扩展安全与回充遥测帧的帧尾定界字节（0x7E）。 */
#define IIR_PROTO_EXT_TAIL 0x7Eu

/** Command ID for protocol_v3 session handshake/ack frames.
 *  protocol_v3 会话握手与确认帧命令字（0x7F）。 */
#define IIR_PROTO_CMD_SESSION 0x7Fu

/** Strict sub-command byte required in session handshake frames.
 *  会话握手帧中严格校验的子命令字节（0x01）。 */
#define IIR_PROTO_SESSION_SUB 0x01u

/** Protocol v3 version identifier encoded in session frames.
 *  会话帧中编码的协议版本号（3）。 */
#define IIR_PROTO_VERSION 3u

/** Capability bit in speed frame byte[2] indicating host v3 sequence mode.
 *  速度帧第 2 字节的能力标志位（0x80），表示主机启用 v3 序号模式。 */
#define IIR_PROTO_CAP_HOST_ACK 0x80u

/** Bitmask extracting the 7-bit sequence number (0..127).
 *  提取 7 位逐帧序号（0..127）的掩码。 */
#define IIR_PROTO_SEQ_MASK 0x7Fu

/** Forward acceptance window size for 7-bit wrapping sequence numbers.
 *  7 位回绕序号的前向接收窗口大小（64）。 */
#define IIR_PROTO_SEQ_WINDOW 64u

/** Sentinel ultrasonic distance indicating no echo, timeout, or stale reading.
 *  表示超声波无回波、超时或数据无效的哨兵值（0xFFFF）。 */
#define IIR_PROTO_ULTRASONIC_INVALID_MM 0xFFFFu

/** Default serial command watchdog timeout in milliseconds (Invariant 2: > 500 ms).
 *  默认串口指令看门狗超时时间，单位为毫秒（工程红线第 2 条：> 500 ms）。 */
#define IIR_PROTO_DEFAULT_WATCHDOG_MS 500u

/**
 * @brief Payload used to encode a 50-byte uplink status telemetry frame.
 *        用于编码 50 字节上行状态遥测帧的数据载荷结构体。
 */
typedef struct {
    /** Software/safety stop flag (0 = motion enabled, non-zero = stopped/disarmed).
     *  软件/安全停机标志位（0 表示允许运动，非零表示已停机失能）。 */
    uint8_t stop_flag;

    /** Measured chassis X linear velocity in mm/s (forward positive).
     *  实测底盘 X 轴线速度，单位为 mm/s（向前为正）。 */
    int16_t vx_mm_s;

    /** Measured chassis Y linear velocity in mm/s (0 for differential chassis).
     *  实测底盘 Y 轴线速度，单位为 mm/s（差速底盘为 0）。 */
    int16_t vy_mm_s;

    /** Measured chassis yaw rate from wheel odometry in mrad/s.
     *  轮式里程计解算的底盘偏航角速度，单位为 mrad/s。 */
    int16_t vz_mrad_s;

    /** Raw 3-axis accelerometer readings in IMU LSB (already rotated to ROS frame).
     *  已映射到 ROS 坐标轴的加速度计三轴原始采样值（LSB）。 */
    int16_t accel_lsb[3];

    /** Raw 3-axis gyroscope readings in IMU LSB (Z forced to 0 when stop_flag != 0).
     *  已映射到 ROS 坐标轴的陀螺仪三轴原始采样值（stop_flag != 0 时 Z 轴强制清零）。 */
    int16_t gyro_lsb[3];

    /** Battery pack voltage in millivolts (mV).
     *  电池组电压，单位为毫伏（mV）。 */
    uint16_t voltage_mv;

    /** Per-wheel velocity/encoder integers for motors A/B/C/D.
     *  电机 A/B/C/D 的编码器/轮速整型遥测值。 */
    int32_t encoder_counts[4];

    /** Actual hardware timer PWM compare values (TIM8 CCR1..CCR4).
     *  底层定时器实际输出的 PWM 比较寄存器值（TIM8 CCR1..CCR4）。 */
    uint16_t pwm_counts[4];

    /** Front HC-SR04 ultrasonic range in mm (0xFFFF when invalid or timed out).
     *  前置 HC-SR04 超声波测距值（mm），超时或无效时为 0xFFFF。 */
    uint16_t ultrasonic_front_mm;
} IIR_StatusFrame;

/**
 * @brief Payload used to encode an 11-byte extended safety and auto-dock telemetry frame.
 *        用于编码 11 字节扩展安全与自动回充状态遥测帧的数据载荷结构体。
 */
typedef struct {
    /** Charging state bitmask (bit0=charging, bit1=allow_recharge, bit2=dock_hw, bit3=nav_walk).
     *  回充状态位掩码（bit0=充电中，bit1=开启回充，bit2=回充硬件在线，bit3=导航接近中）。 */
    uint8_t charging_state;

    /** Signed battery charging current in milliamperes (mA).
     *  有符号回充充电电流，单位为毫安（mA）。 */
    int16_t charging_current_ma;

    /** Docking IR sensor bitmask (bit0=R_A, bit1=R_B, bit2=L_B, bit3=L_A, bits[6:4]=red_state).
     *  红外对接传感器掩码（bit0=R_A，bit1=R_B，bit2=L_B，bit3=L_A，高半区为红外总数）。 */
    uint8_t ir_mask;

    /** Active safety flags bitmask (bit0=SOS, bit1=PE4_ESTOP, bit2=UART_TO, bit3=UV, bit4=D50A).
     *  当前激活的安全锁存与停机标志位掩码。 */
    uint8_t safety_flags;

    /** Normalized active-HIGH D50A RS485 fault mask (bit0=OC, bit1=OT, bit2=OV, bit3=UV, bit7=offline).
     *  归一化高有效 D50A RS485 驱动器故障掩码（bit7 表示 RS485 离线）。 */
    uint8_t d50a_alarm_mask;
} IIR_ExtStatusFrame;

/**
 * @brief Firmware version metadata carried by the 18-byte version telemetry frame.
 *        18 字节版本遥测帧承载的固件版本元数据。
 */
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    uint8_t chassis_type;
    uint32_t git_sha_u32;
} IIR_FwVersionFrame;

/** One atomic infrared target snapshot.
 *  一份原子的红外目标快照。 */
typedef struct {
    uint32_t generation;
    int16_t vx_mm_s;
    int16_t wz_mrad_s;
    bool active;
} IIR_DockTargetFrame;

/** Encode an infrared target snapshot for the host.
 *  编码发送给上位机的红外目标快照。 */
bool IIR_Proto_EncodeDockTargetFrame(const IIR_DockTargetFrame *target,
                                    uint8_t out_buf[IIR_PROTO_DOCK_TARGET_FRAME_LEN]);

/**
 * @brief Status returned by IIR_Proto_RxFeedByte after processing one serial byte.
 *        每向 IIR_Proto_RxFeedByte 喂入一个串口字节后返回的解析结果状态码。
 */
typedef enum {
    /** More bytes are needed to complete a frame.
     *  当前尚未收齐完整帧，需要继续接收后续字节。 */
    IIR_PROTO_RX_INCOMPLETE = 0,

    /** Valid 3-axis target speed frame decoded and accepted.
     *  成功解析并接收一帧合法的三轴速度控制指令。 */
    IIR_PROTO_RX_SPEED_CMD = 1,

    /** Valid v3 host session acknowledgement accepted.
     *  成功接收并验证主机发送的 v3 会话握手确认帧。 */
    IIR_PROTO_RX_SESSION_ACK = 2,

    /** Valid SET_AUTO_DOCK command decoded and accepted.
     *  成功解析并接收合法的自动回充启停与对接配置指令（SET_AUTO_DOCK）。 */
    IIR_PROTO_RX_AUTO_DOCK_CMD = 3,

    /** Valid SET_SOS command decoded and accepted.
     *  成功解析并接收合法的远程 SOS 触发/复位指令（SET_SOS）。 */
    IIR_PROTO_RX_SOS_CMD = 4,

    /** Valid SET_LED_BEEP command decoded and accepted.
     *  成功解析并接收合法的远程声光模式控制指令（SET_LED_BEEP）。 */
    IIR_PROTO_RX_LED_BEEP_CMD = 5,

    /** Valid host stop command accepted.
     *  成功接收上位机锁停命令。 */
    IIR_PROTO_RX_STOP_CMD = 6,

    /** Valid infrared target permission accepted.
     *  成功接收红外目标运动许可。 */
    IIR_PROTO_RX_DOCK_PERMISSION = 7,

    /** Candidate frame failed tail byte or BCC XOR verification (resynced).
     *  候选帧未能通过帧尾或 BCC 异或校验（解析器已滑动重同步）。 */
    IIR_PROTO_RX_ERR_BAD_FRAME = -1,

    /** Speed frame rejected because v3 session handshake has not completed.
     *  在未完成 v3 会话握手前拒绝速度控制帧。 */
    IIR_PROTO_RX_ERR_NO_SESSION = -2,

    /** Speed frame rejected because CAP_HOST_ACK bit is missing in v3 mode.
     *  v3 模式下速度帧缺少 CAP_HOST_ACK 标志位而被拒绝。 */
    IIR_PROTO_RX_ERR_NO_SEQ = -3,

    /** Speed frame rejected due to duplicate/replayed sequence number.
     *  速度帧因序号重复（重放）而被拒绝。 */
    IIR_PROTO_RX_ERR_SEQ_REPLAY = -4,

    /** Speed frame rejected because sequence number fell outside forward window (64).
     *  速度帧序号越出前向接收窗口（64）而被拒绝。 */
    IIR_PROTO_RX_ERR_SEQ_WINDOW = -5,

    /** Session ack frame rejected due to mismatched version, sub-byte, or session_id.
     *  会话确认帧因版本号、子命令字节或会话 ID 不匹配而被拒绝。 */
    IIR_PROTO_RX_ERR_BAD_SESSION = -6,

    /** Valid framing/checksum but unrecognized command ID in frame[1].
     *  帧结构与校验合法，但 frame[1] 命令字未识别。 */
    IIR_PROTO_RX_ERR_UNKNOWN_CMD = -7,

    /** Frame rejected as local serial TX echo matching registered telemetry signature.
     *  帧载荷与本端已登记的遥测发送特征完全一致，判定为串口本地回显并丢弃。 */
    IIR_PROTO_RX_ERR_ECHO = -8
} IIR_ProtoRxStatus;

/**
 * @brief Decoded event output produced when a valid command or session frame arrives.
 *        收到合法控制帧或会话帧时输出的解码事件结构。
 */
typedef struct {
    /** Parse status code indicating frame type or rejection reason.
     *  指示帧类型或拒绝原因的解析状态码。 */
    IIR_ProtoRxStatus status;

    /** Decoded target X linear velocity in m/s (valid when status == SPEED_CMD).
     *  解码得到的目标 X 轴线速度（m/s，仅当 status == SPEED_CMD 时有效）。 */
    float vx_mps;

    /** Decoded target Y linear velocity in m/s (valid when status == SPEED_CMD).
     *  解码得到的目标 Y 轴线速度（m/s，仅当 status == SPEED_CMD 时有效）。 */
    float vy_mps;

    /** Decoded target yaw angular velocity in rad/s (valid when status == SPEED_CMD).
     *  解码得到的目标偏航角速度（rad/s，仅当 status == SPEED_CMD 时有效）。 */
    float vz_rad_s;

    /** Sequence number extracted from byte[2] & 0x7F.
     *  从第 2 字节低 7 位提取的逐帧序号。 */
    uint8_t seq;

    /** Session ID decoded from session ack frame.
     *  从会话确认帧解码得到的 32 位会话 ID。 */
    uint32_t session_id;

    /** Decoded auto-dock enable flag (valid when status == AUTO_DOCK_CMD).
     *  解码得到的自动回充使能标志（仅当 status == AUTO_DOCK_CMD 时有效）。 */
    bool auto_dock_enable;

    /** Decoded auto-dock approach X velocity in m/s (AUTO_DOCK_CMD).
     *  解码得到的自动回充红外对接 X 轴目标速度（m/s）。 */
    float dock_vx_mps;

    /** Decoded auto-dock yaw velocity in rad/s (AUTO_DOCK_CMD).
     *  解码得到的自动回充红外对接 Z 轴角速度（rad/s）。 */
    float dock_vz_rad_s;

    /** Infrared target generation covered by the permission.
     *  运动许可对应的红外目标代次。 */
    uint32_t dock_generation;

    /** Decoded remote SOS active state (valid when status == SOS_CMD).
     *  解码得到的远程 SOS 触发/复位状态（仅当 status == SOS_CMD 时有效）。 */
    bool sos_active;

    /** Decoded remote LED override mode (valid when status == LED_BEEP_CMD).
     *  解码得到的远程 RGB 灯效模式值（仅当 status == LED_BEEP_CMD 时有效）。 */
    uint8_t led_beep_mode;

    /** Decoded remote buzzer enable flag (valid when status == LED_BEEP_CMD).
     *  解码得到的远程蜂鸣器使能标志（仅当 status == LED_BEEP_CMD 时有效）。 */
    bool beep_enable;
} IIR_ProtoRxEvent;

/**
 * @brief Stateful stream parser and protocol_v3 session/sequence gate context.
 *        带状态的串口字节流解析器及 protocol_v3 会话与序号门控上下文。
 */
typedef struct {
    /** Partial frame accumulation buffer (11 bytes max).
     *  半帧接收缓存区（最大 11 字节）。 */
    uint8_t buf[IIR_PROTO_CMD_FRAME_LEN];

    /** Current number of valid bytes buffered in buf.
     *  当前缓存在 buf 中的有效字节数。 */
    uint8_t len;

    /** Firmware-generated 32-bit session ID expected from host session acks.
     *  固件端生成的 32 位会话 ID，用于校验主机回传的会话确认帧。 */
    uint32_t session_id;

    /** When true, enforces v3 session handshake and 7-bit sequence window gating.
     *  为 true 时强制要求 v3 会话握手与 7 位序号窗口门控。 */
    bool require_v3_session;

    /** True once a matching session ack frame has been accepted.
     *  收到匹配的会话确认帧后置为 true。 */
    bool session_acked;

    /** True after at least one valid v3 frame established last_seq.
     *  在会话内首帧合法序号指令建立 last_seq 基准后置为 true。 */
    bool has_seq;

    /** Most recently accepted 7-bit frame sequence number.
     *  最近一次合法接收的 7 位帧序号。 */
    uint8_t last_seq;

    /** Diagnostic counter: total valid command frames accepted (excludes session acks).
     *  诊断计数：累计通过校验并接收的控制命令帧总数量（不含会话确认帧）。 */
    uint32_t frames_accepted;

    /** Diagnostic counter: total candidate frames rejected (checksum/gate).
     *  诊断计数：累计被拒绝的候选帧总数量（含校验失败与门控拒绝）。 */
    uint32_t frames_rejected;

    /** Diagnostic counter: command frames rejected due to missing active session.
     *  诊断计数：因会话未建立而被拒绝的指令帧数量。 */
    uint32_t frames_no_session;

    /** Diagnostic counter: session ack frames rejected (version/sub-byte/stale session ID).
     *  诊断计数：因版本、子命令或会话 ID 不匹配而被拒绝的会话帧数量。 */
    uint32_t frames_bad_session;

    /** Diagnostic counter: frames dropped as local TX echo.
     *  诊断计数：被识别为串口本地回显并丢弃的帧数量。 */
    uint32_t frames_echo;

    /** Diagnostic counter: frames rejected due to unrecognized command ID.
     *  诊断计数：因命令字未识别而被拒绝的帧数量。 */
    uint32_t frames_bad_type;

    /** Diagnostic counter: frames rejected by 7-bit sequence replay/window gate.
     *  诊断计数：因 7 位序号重放或越出前向窗口而被拒绝的帧数量。 */
    uint32_t frames_stale_seq;

    /** Diagnostic counter: frames rejected due to missing CAP_HOST_ACK sequence bit.
     *  诊断计数：因缺少 v3 序号能力位或非法子字节而被拒绝的帧数量。 */
    uint32_t frames_no_seq;

    /** True when local TX echo detection signature is armed.
     *  启用本地串口回显检测特征匹配时为 true。 */
    bool has_echo_sig;

    /** Recorded uplink command/stop byte[1] used for echo suppression.
     *  记录的上行帧第 1 字节特征，用于识别本地回显。 */
    uint8_t echo_cmd_byte;

    /** Recorded uplink bytes [3..8] used for local TX echo suppression.
     *  记录的上行帧第 3..8 字节载荷特征，用于识别本地回显。 */
    uint8_t echo_payload6[6];
} IIR_Proto_RxContext;

/**
 * @brief Wrap-safe millisecond command watchdog implementing Fail-Closed timeout.
 *        支持无符号毫秒计数器溢出的指令看门狗，实现失联超时故障安全停机。
 */
typedef struct {
    /** Maximum allowed silence between valid host speed commands in milliseconds.
     *  允许连续两次合法主机速度指令之间的最大间隔（毫秒）。 */
    uint32_t timeout_ms;

    /** Timestamp of the most recent watchdog kick in milliseconds.
     *  最近一次喂狗时的系统毫秒时间戳。 */
    uint32_t last_kick_ms;

    /** True once kicked at least once; false initially so startup is Fail-Closed.
     *  至少收到一次喂狗后置 true；初始为 false 以确保上电默认处于故障安全停机态。 */
    bool armed;
} IIR_Proto_Watchdog;

/**
 * @brief Compute 8-bit XOR block check character (BCC) over data[0..len-1].
 *        计算字节数组 data[0..len-1] 的 8 位异或校验码（BCC）。
 */
uint8_t IIR_Proto_CheckBcc(const uint8_t *data, size_t len);

/**
 * @brief Encode a 50-byte uplink status telemetry frame compatible with StatusFrameParser.
 *        编码 50 字节上行状态遥测帧，与上位机 StatusFrameParser 逐字节兼容。
 *
 * @param[in]  status  Telemetry payload structure.
 *                     待发送的状态遥测数据结构体。
 * @param[out] out_buf Destination buffer of at least IIR_PROTO_STATUS_FRAME_LEN bytes.
 *                     至少为 IIR_PROTO_STATUS_FRAME_LEN 字节的输出缓冲区。
 * @return true on success; false if any pointer argument is NULL.
 *         编码成功返回 true；传入空指针则返回 false。
 */
bool IIR_Proto_EncodeStatusFrame(const IIR_StatusFrame *status,
                                 uint8_t out_buf[IIR_PROTO_STATUS_FRAME_LEN]);

/**
 * @brief Encode an 11-byte protocol_v3 session advertisement frame.
 *        编码 11 字节的 protocol_v3 会话通告帧。
 *
 * @param[in]  session_id Active 32-bit firmware session identifier.
 *                        当前固件活跃的 32 位会话标识符。
 * @param[out] out_buf    Destination buffer of at least IIR_PROTO_CMD_FRAME_LEN bytes.
 *                        至少为 IIR_PROTO_CMD_FRAME_LEN 字节的输出缓冲区。
 * @return true on success; false if out_buf is NULL.
 *         编码成功返回 true；out_buf 为空则返回 false。
 */
bool IIR_Proto_EncodeSessionFrame(uint32_t session_id,
                                  uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN]);

/**
 * @brief Encode an 11-byte uplink extended safety and auto-dock telemetry frame (0x7C..0x7E).
 *        编码 11 字节上行扩展安全与自动回充状态遥测帧（0x7C..0x7E）。
 */
bool IIR_Proto_EncodeExtStatusFrame(const IIR_ExtStatusFrame *frame,
                                     uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN]);

bool IIR_Proto_EncodeVersionFrame(const IIR_FwVersionFrame *version,
                                  uint8_t out_buf[IIR_PROTO_VERSION_FRAME_LEN]);

/**
 * @brief Decode an 11-byte uplink extended safety and auto-dock telemetry frame (0x7C..0x7E).
 *        解码并校验 11 字节上行扩展安全与自动回充状态遥测帧（0x7C..0x7E）。
 */
bool IIR_Proto_DecodeExtStatusFrame(const uint8_t in_buf[IIR_PROTO_CMD_FRAME_LEN],
                                     IIR_ExtStatusFrame *out_frame);

/**
 * @brief Encode a downlink 11-byte SET_AUTO_DOCK control command frame.
 *        编码下行 11 字节 SET_AUTO_DOCK 自动回充控制指令帧。
 */
bool IIR_Proto_EncodeAutoDockCmd(bool enable,
                                 int16_t dock_vx_mm_s,
                                 int16_t dock_vz_mrad_s,
                                 uint8_t seq,
                                 bool use_v3_seq,
                                 uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN]);

/**
 * @brief Encode a downlink 11-byte SET_SOS emergency trigger/reset command frame.
 *        编码下行 11 字节 SET_SOS 远程紧急报警触发/复位指令帧。
 */
bool IIR_Proto_EncodeSetSosCmd(bool active,
                               uint8_t seq,
                               bool use_v3_seq,
                               uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN]);

/**
 * @brief Encode a downlink 11-byte SET_LED_BEEP audio/visual override command frame.
 *        编码下行 11 字节 SET_LED_BEEP 远程声光提示控制指令帧。
 */
bool IIR_Proto_EncodeSetLedBeepCmd(uint8_t mode,
                                   bool beep_enable,
                                   uint8_t seq,
                                   bool use_v3_seq,
                                   uint8_t out_buf[IIR_PROTO_CMD_FRAME_LEN]);

/**
 * @brief Register a local TX telemetry signature (byte[1], bytes[3..8]) to suppress echo loops.
 *        登记本地已发出的遥测帧特征签名（第 1 字节与第 3..8 字节）以屏蔽串口回显环路。
 */
void IIR_Proto_RxSetEchoSignature(IIR_Proto_RxContext *ctx,
                                  uint8_t cmd_byte,
                                  const uint8_t payload6[6]);

/**
 * @brief Initialize or reset a protocol_v3 receiver stream context.
 *        初始化或复位 protocol_v3 接收流解析上下文。
 *
 * @param[out] ctx                Receiver context to initialize.
 *                                待初始化的接收解析上下文。
 * @param[in]  session_id         Expected 32-bit session ID for v3 handshake.
 *                                期望主机回传匹配的 32 位会话 ID。
 * @param[in]  require_v3_session True for strict v3 session+seq gating; false for baseline mode.
 *                                为 true 时强制 v3 会话与序号门控；为 false 时兼容基线模式。
 */
void IIR_Proto_RxInit(IIR_Proto_RxContext *ctx,
                      uint32_t session_id,
                      bool require_v3_session);

/**
 * @brief Feed one received UART byte into the protocol_v3 state machine.
 *        向 protocol_v3 接收状态机送入一个串口字节并尝试解码帧。
 *
 * @param[in,out] ctx       Receiver context.
 *                          接收解析上下文指针。
 * @param[in]     byte      Next incoming byte from UART DMA/ring buffer.
 *                          从串口 DMA 或环形缓冲区读取的下一个字节。
 * @param[out]    out_event Optional event structure receiving status and decoded payload.
 *                          可选输出事件结构体，接收状态码及解码数据（可为 NULL）。
 * @return Parse status code (IIR_ProtoRxStatus).
 *         返回解析状态码（IIR_ProtoRxStatus）。
 */
IIR_ProtoRxStatus IIR_Proto_RxFeedByte(IIR_Proto_RxContext *ctx,
                                       uint8_t byte,
                                       IIR_ProtoRxEvent *out_event);

/**
 * @brief Initialize the command link watchdog in Fail-Closed disarmed state.
 *        初始化串口指令看门狗并置于故障安全未激活状态（默认超时 500 ms）。
 */
void IIR_Proto_WatchdogInit(IIR_Proto_Watchdog *wd, uint32_t timeout_ms);

/**
 * @brief Refresh the command watchdog timestamp upon receiving a valid host speed frame.
 *        在收到合法主机速度帧时喂狗并更新时间戳。
 */
void IIR_Proto_WatchdogKick(IIR_Proto_Watchdog *wd, uint32_t now_ms);

/**
 * @brief Check whether the command link has timed out (or was never kicked).
 *        检查指令链路是否超时失联（或上电后尚未收到首帧有效指令）。
 *
 * @return true if timed out / disarmed (Fail-Closed); false if link is fresh.
 *         超时或未激活返回 true（触发故障安全停机）；链路正常返回 false。
 */
bool IIR_Proto_WatchdogIsTimedOut(const IIR_Proto_Watchdog *wd, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* IIR_APP_PROTOCOL_PROTOCOL_V3_H */

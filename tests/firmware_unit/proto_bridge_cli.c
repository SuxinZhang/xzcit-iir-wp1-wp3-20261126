/**
 * @file proto_bridge_cli.c
 * @brief Host CLI bridge exposing App/protocol/protocol_v3 for byte-level
 *        interoperability testing against Master/ROS/ros2_ws/src/c50c_uart3.
 *        将 App/protocol/protocol_v3 暴露给宿主机的命令行测试桥，用于与
 *        Master/ROS/ros2_ws/src/c50c_uart3 进行逐字节联调校验。
 */

#include "protocol_v3.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Print a byte array as lowercase hexadecimal followed by newline.
 * 将字节数组以小写十六进制字符串形式输出并换行。 */
static void print_hex(const uint8_t *buf, size_t len)
{
    for (size_t i = 0u; i < len; ++i) {
        printf("%02x", (unsigned int)buf[i]);
    }
    printf("\n");
}

/* Parse a single hex nibble; returns -1 on invalid character.
 * 解析单个十六进制字符；遇到非法字符时返回 -1。 */
static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return -1;
}

static int cmd_encode_status(int argc, char **argv)
{
    if (argc != 22) {
        fprintf(stderr, "Usage: encode_status <20 numeric args>\n");
        return 2;
    }

    IIR_StatusFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.stop_flag = (uint8_t)strtoul(argv[2], NULL, 10);
    frame.vx_mm_s = (int16_t)strtol(argv[3], NULL, 10);
    frame.vy_mm_s = (int16_t)strtol(argv[4], NULL, 10);
    frame.vz_mrad_s = (int16_t)strtol(argv[5], NULL, 10);
    frame.accel_lsb[0] = (int16_t)strtol(argv[6], NULL, 10);
    frame.accel_lsb[1] = (int16_t)strtol(argv[7], NULL, 10);
    frame.accel_lsb[2] = (int16_t)strtol(argv[8], NULL, 10);
    frame.gyro_lsb[0] = (int16_t)strtol(argv[9], NULL, 10);
    frame.gyro_lsb[1] = (int16_t)strtol(argv[10], NULL, 10);
    frame.gyro_lsb[2] = (int16_t)strtol(argv[11], NULL, 10);
    frame.voltage_mv = (uint16_t)strtoul(argv[12], NULL, 10);
    frame.encoder_counts[0] = (int32_t)strtol(argv[13], NULL, 10);
    frame.encoder_counts[1] = (int32_t)strtol(argv[14], NULL, 10);
    frame.encoder_counts[2] = (int32_t)strtol(argv[15], NULL, 10);
    frame.encoder_counts[3] = (int32_t)strtol(argv[16], NULL, 10);
    frame.pwm_counts[0] = (uint16_t)strtoul(argv[17], NULL, 10);
    frame.pwm_counts[1] = (uint16_t)strtoul(argv[18], NULL, 10);
    frame.pwm_counts[2] = (uint16_t)strtoul(argv[19], NULL, 10);
    frame.pwm_counts[3] = (uint16_t)strtoul(argv[20], NULL, 10);
    frame.ultrasonic_front_mm = (uint16_t)strtoul(argv[21], NULL, 10);

    uint8_t out[IIR_PROTO_STATUS_FRAME_LEN];
    if (!IIR_Proto_EncodeStatusFrame(&frame, out)) {
        return 1;
    }
    print_hex(out, sizeof(out));
    return 0;
}

static int cmd_encode_session(int argc, char **argv)
{
    if (argc != 3) {
        fprintf(stderr, "Usage: encode_session <session_id_hex>\n");
        return 2;
    }

    const uint32_t sid = (uint32_t)strtoul(argv[2], NULL, 16);
    uint8_t out[IIR_PROTO_CMD_FRAME_LEN];
    if (!IIR_Proto_EncodeSessionFrame(sid, out)) {
        return 1;
    }
    print_hex(out, sizeof(out));
    return 0;
}

static int cmd_encode_ext_status(int argc, char **argv)
{
    if (argc != 7) {
        fprintf(stderr, "Usage: encode_ext_status <chg_state> <chg_ma> <ir_mask> <safety> <d50a>\n");
        return 2;
    }

    IIR_ExtStatusFrame frame;
    frame.charging_state = (uint8_t)strtoul(argv[2], NULL, 10);
    frame.charging_current_ma = (int16_t)strtol(argv[3], NULL, 10);
    frame.ir_mask = (uint8_t)strtoul(argv[4], NULL, 10);
    frame.safety_flags = (uint8_t)strtoul(argv[5], NULL, 10);
    frame.d50a_alarm_mask = (uint8_t)strtoul(argv[6], NULL, 10);

    uint8_t out[IIR_PROTO_CMD_FRAME_LEN];
    if (!IIR_Proto_EncodeExtStatusFrame(&frame, out)) {
        return 1;
    }
    print_hex(out, sizeof(out));
    return 0;
}

static int cmd_decode_rx(int argc, char **argv)
{
    if (argc != 5) {
        fprintf(stderr, "Usage: decode_rx <v3_mode:0|1> <session_id_hex> <hex_stream>\n");
        return 2;
    }

    const bool v3_mode = (strtol(argv[2], NULL, 10) != 0);
    const uint32_t sid = (uint32_t)strtoul(argv[3], NULL, 16);
    const char *hex = argv[4];
    const size_t hex_len = strlen(hex);
    if ((hex_len % 2u) != 0u) {
        fprintf(stderr, "Odd hex stream length\n");
        return 2;
    }

    IIR_Proto_RxContext ctx;
    IIR_Proto_RxInit(&ctx, sid, v3_mode);

    for (size_t i = 0u; i < hex_len; i += 2u) {
        const int hi = hex_nibble(hex[i]);
        const int lo = hex_nibble(hex[i + 1u]);
        if (hi < 0 || lo < 0) {
            fprintf(stderr, "Invalid hex digit\n");
            return 2;
        }
        const uint8_t byte = (uint8_t)(((unsigned int)hi << 4u) | (unsigned int)lo);

        IIR_ProtoRxEvent ev;
        const IIR_ProtoRxStatus st = IIR_Proto_RxFeedByte(&ctx, byte, &ev);
        if (st == IIR_PROTO_RX_SESSION_ACK) {
            printf("SESSION_ACK %08x\n", (unsigned int)ev.session_id);
        } else if (st == IIR_PROTO_RX_SPEED_CMD) {
            printf("SPEED_CMD seq=%u vx=%.6f vy=%.6f vz=%.6f\n",
                   (unsigned int)ev.seq,
                   (double)ev.vx_mps,
                   (double)ev.vy_mps,
                   (double)ev.vz_rad_s);
        } else if (st == IIR_PROTO_RX_AUTO_DOCK_CMD) {
            printf("AUTO_DOCK_CMD seq=%u enable=%u vx=%.6f vz=%.6f\n",
                   (unsigned int)ev.seq,
                   ev.auto_dock_enable ? 1u : 0u,
                   (double)ev.dock_vx_mps,
                   (double)ev.dock_vz_rad_s);
        } else if (st == IIR_PROTO_RX_SOS_CMD) {
            printf("SOS_CMD seq=%u active=%u\n",
                   (unsigned int)ev.seq,
                   ev.sos_active ? 1u : 0u);
        } else if (st == IIR_PROTO_RX_LED_BEEP_CMD) {
            printf("LED_BEEP_CMD seq=%u mode=%u beep=%u\n",
                   (unsigned int)ev.seq,
                   (unsigned int)ev.led_beep_mode,
                   ev.beep_enable ? 1u : 0u);
        } else if (st < 0) {
            printf("ERR %d\n", (int)st);
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "Missing subcommand\n");
        return 2;
    }

    if (strcmp(argv[1], "encode_status") == 0) {
        return cmd_encode_status(argc, argv);
    }
    if (strcmp(argv[1], "encode_session") == 0) {
        return cmd_encode_session(argc, argv);
    }
    if (strcmp(argv[1], "encode_ext_status") == 0) {
        return cmd_encode_ext_status(argc, argv);
    }
    if (strcmp(argv[1], "decode_rx") == 0) {
        return cmd_decode_rx(argc, argv);
    }

    fprintf(stderr, "Unknown subcommand: %s\n", argv[1]);
    return 2;
}

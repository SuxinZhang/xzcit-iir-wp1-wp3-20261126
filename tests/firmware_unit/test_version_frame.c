/**
 * @file test_version_frame.c
 * @brief Host tests for the 18-byte firmware version telemetry frame.
 *        18 字节固件版本遥测帧的宿主机测试。
 */
#include "protocol_v3.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    IIR_FwVersionFrame version = {
        .major = 1u,
        .minor = 2u,
        .patch = 0u,
        .chassis_type = 0u,
        .git_sha_u32 = 0x1234ABCDu,
    };
    uint8_t frame[IIR_PROTO_VERSION_FRAME_LEN] = {0};
    assert(IIR_Proto_EncodeVersionFrame(&version, frame));
    assert(frame[0] == IIR_PROTO_EXT_HEADER);
    assert(frame[1] == IIR_PROTO_CMD_VERSION);
    assert(frame[2] == IIR_PROTO_VERSION);
    assert(frame[3] == 1u && frame[4] == 2u && frame[5] == 0u);
    assert(frame[6] == 0u);
    assert(frame[7] == 0x12u && frame[8] == 0x34u && frame[9] == 0xABu && frame[10] == 0xCDu);
    assert(frame[16] == IIR_Proto_CheckBcc(frame, 16u));
    assert(frame[17] == IIR_PROTO_EXT_TAIL);
    assert(!IIR_Proto_EncodeVersionFrame(NULL, frame));
    printf("test_version_frame: ALL PASSED\n");
    return 0;
}

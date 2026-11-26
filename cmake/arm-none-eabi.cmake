# Cross-compilation toolchain file for STM32F407VET6 (Cortex-M4F).
# 面向 STM32F407VET6（Cortex-M4F）的交叉编译工具链配置文件。

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Use static library mode during CMake compiler checks to avoid linking without a startup script.
# 在 CMake 编译器自检阶段使用静态库模式，避免缺少启动文件导致的链接报错。
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Specify the GNU Arm Embedded toolchain binaries.
# 指定 GNU Arm Embedded 交叉编译工具链可执行程序。
set(TOOLCHAIN_PREFIX arm-none-eabi-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_AR ${TOOLCHAIN_PREFIX}ar)
set(CMAKE_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
set(CMAKE_OBJDUMP ${TOOLCHAIN_PREFIX}objdump)
set(CMAKE_SIZE ${TOOLCHAIN_PREFIX}size)

# Do not search host root filesystem paths for target libraries or headers.
# 禁止在宿主机系统根目录中检索目标平台的库与头文件。
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Configure Cortex-M4F hardware single-precision FPU flags (-mfpu=fpv4-sp-d16 -mfloat-abi=hard).
# 为 Cortex-M4F 配置单精度硬件浮点编译标志（-mfpu=fpv4-sp-d16 -mfloat-abi=hard）。
set(IIR_MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(IIR_COMMON_COMPILE_FLAGS "${IIR_MCU_FLAGS} -ffunction-sections -fdata-sections")

set(CMAKE_C_FLAGS_INIT "${IIR_COMMON_COMPILE_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${IIR_COMMON_COMPILE_FLAGS}")
set(CMAKE_ASM_FLAGS_INIT "${IIR_MCU_FLAGS} -x assembler-with-cpp")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${IIR_MCU_FLAGS} -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs")

# Generate .hex/.bin firmware images and print Flash/RAM footprint after linking.
# 链接完成后自动生成 .hex/.bin 固件镜像并打印 Flash/RAM 占用报告。
function(iir_add_firmware_outputs target_name)
  string(REGEX REPLACE "\\.elf$" "" _iir_fw_stem "${target_name}")
  add_custom_command(TARGET ${target_name} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/${_iir_fw_stem}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/${_iir_fw_stem}.bin
    COMMAND ${CMAKE_SIZE} --format=berkeley $<TARGET_FILE:${target_name}>
    COMMENT "Generating .hex/.bin images and reporting section sizes for ${target_name}"
  )
endfunction()

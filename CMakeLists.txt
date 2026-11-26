# Top-level CMake configuration for XZCIT-IIR / inspection-robot STM32F407 firmware and host tests.
# XZCIT-IIR / inspection-robot STM32F407 固件与宿主机单元测试的顶层 CMake 构建脚本。

cmake_minimum_required(VERSION 3.22)

project(xzcit_iir_firmware VERSION 0.1.0 LANGUAGES C ASM)

# Enforce strict C11 across all pure-C firmware modules.
# 在全部纯 C 固件模块中强制启用标准 C11。
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

# Export compile_commands.json for static analysis and editor IntelliSense.
# 导出 compile_commands.json 供静态分析与编辑器索引使用。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Explicit chassis configuration option (only DIFF is active for XZCIT-IIR).
# 显式车型配置开关（XZCIT-IIR 仅启用差速底盘 DIFF）。
set(CHASSIS_TYPE "DIFF" CACHE STRING "Chassis kinematics type (DIFF)")
option(BUILD_HOST_TESTS "Build pure-C host unit tests with CTest" OFF)

if(NOT CHASSIS_TYPE STREQUAL "DIFF")
  message(FATAL_ERROR "Unsupported CHASSIS_TYPE='${CHASSIS_TYPE}'. Expected 'DIFF'.")
endif()

# Shared strict compiler warnings for App/ modules (zero-HAL portable C11).
# App/ 纯 C11 模块共享的严格编译告警配置（零 HAL 依赖）。
add_library(iir_app_warnings INTERFACE)
target_compile_definitions(iir_app_warnings INTERFACE IIR_CHASSIS_DIFF=1)
target_compile_options(iir_app_warnings INTERFACE
  -Wall
  -Wextra
  -Wpedantic
  -Werror
  -Wshadow
  -Wconversion
)

# Pure-C differential kinematics library (no HAL dependency).
# 纯 C 差速运动学静态库（零 HAL 依赖）。
add_library(iir_kinematics STATIC
  App/kinematics/diff_kinematics.c
)
target_include_directories(iir_kinematics PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/App/kinematics
)
target_link_libraries(iir_kinematics
  PRIVATE iir_app_warnings
)
if(UNIX AND NOT CMAKE_CROSSCOMPILING)
  target_link_libraries(iir_kinematics PUBLIC m)
endif()

# Pure-C incremental PI, velocity smoother, low-speed compensation, and single PWM gate.
# 纯 C 增量式 PI、速度平滑、低速非线性补偿与唯一电机 PWM 出口静态库。
add_library(iir_control STATIC
  App/control/motor_control.c
)
target_include_directories(iir_control PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/App/control
)
target_link_libraries(iir_control
  PUBLIC iir_kinematics
  PRIVATE iir_app_warnings
)

# Pure-C protocol_v3 encoder/decoder and command watchdog (byte-compatible with c50c_uart3).
# 纯 C protocol_v3 帧编解码器与指令看门狗静态库（与 c50c_uart3 字节级兼容）。
add_library(iir_protocol STATIC
  App/protocol/protocol_v3.c
)
target_include_directories(iir_protocol PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/App/protocol
)
target_link_libraries(iir_protocol
  PRIVATE iir_app_warnings
)

# Pure-C Fail-Closed safety state machine and SOS audio/visual arbiter.
# 纯 C 故障安全底盘状态机与 SOS 声光优先级仲裁静态库（零 HAL 依赖）。
add_library(iir_safety STATIC
  App/safety/safety_manager.c
)
target_include_directories(iir_safety PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/App/safety
)
target_link_libraries(iir_safety
  PRIVATE iir_app_warnings
)

# Host unit tests via CTest (requires BUILD_HOST_TESTS=ON and native compiler).
# 通过 CTest 运行宿主机单元测试（需开启 BUILD_HOST_TESTS=ON 并使用宿主机编译器）。
if(BUILD_HOST_TESTS)
  if(CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "BUILD_HOST_TESTS=ON cannot be combined with cross-compilation.")
  endif()
  enable_testing()
  add_subdirectory(tests/firmware_unit)
endif()

# Cross-compilation target for STM32F407VET6 firmware (WP1b once CubeMX HAL SDK is populated).
# 面向 STM32F407VET6 的交叉编译固件目标（待 WP1b 补全 CubeMX HAL SDK 后即可生成镜像）。
if(CMAKE_CROSSCOMPILING)
  set(IIR_HAL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver")
  set(IIR_CORE_MAIN "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c")
  set(IIR_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F407xx_FLASH.ld")

  set(IIR_FREERTOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS/Source")
  if(NOT EXISTS "${IIR_CORE_MAIN}" OR NOT EXISTS "${IIR_HAL_DIR}/Src" OR
     NOT EXISTS "${IIR_FREERTOS_DIR}/tasks.c" OR NOT EXISTS "${IIR_LINKER_SCRIPT}")
    message(FATAL_ERROR
      "STM32 cross-compilation prerequisites are not yet populated in this clean skeleton (WP1b).\n"
      "Missing one or more of:\n"
      "  - ${IIR_CORE_MAIN}\n"
      "  - ${IIR_HAL_DIR}/Src\n"
      "  - ${IIR_FREERTOS_DIR}/tasks.c\n"
      "  - ${IIR_LINKER_SCRIPT}\n"
      "Populate Core/, CMSIS/, STM32F4xx_HAL_Driver/, FreeRTOS/, startup_stm32f407xx.s, and linker script first."
    )
  endif()

  # Vendor ST HAL, CMSIS, FreeRTOS kernel, and ST system/syscall stubs (compiled without -Werror).
  # 原厂 ST HAL、CMSIS、FreeRTOS 内核及 ST 系统/桩文件库（不开启第三方代码的 -Werror）。
  file(GLOB IIR_HAL_SOURCES
    "${IIR_HAL_DIR}/Src/*.c"
  )
  # Exclude ST template files from the HAL driver source list.
  # 从 HAL 驱动源码列表中排除 ST 模板文件。
  list(FILTER IIR_HAL_SOURCES EXCLUDE REGEX ".*_template\\.c$")

  file(GLOB IIR_FREERTOS_CORE_SOURCES
    "${IIR_FREERTOS_DIR}/*.c"
  )

  add_library(iir_hal_freertos STATIC
    ${IIR_HAL_SOURCES}
    ${IIR_FREERTOS_CORE_SOURCES}
    "${IIR_FREERTOS_DIR}/portable/GCC/ARM_CM4F/port.c"
    "${IIR_FREERTOS_DIR}/portable/MemMang/heap_4.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/system_stm32f4xx.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/syscalls.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/sysmem.c"
  )
  target_compile_definitions(iir_hal_freertos PUBLIC
    USE_HAL_DRIVER
    STM32F407xx
    ARM_MATH_CM4
    __FPU_PRESENT=1
    IIR_CHASSIS_DIFF=1
  )
  # Allow GNU inline-asm extensions on hardware-bound STM32/FreeRTOS targets while keeping
  # App/{kinematics,control,protocol} strictly ISO C11.
  # 在绑定的 STM32/FreeRTOS 目标上允许 GNU 内联汇编扩展，同时保持纯 C 模块严格遵循 ISO C11。
  set_target_properties(iir_hal_freertos PROPERTIES C_EXTENSIONS ON)
  target_include_directories(iir_hal_freertos PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc
  )
  # Mark third-party vendor SDK headers as SYSTEM so strict -Wpedantic/-Wconversion stays on our code.
  # 将第三方原厂 SDK 头文件路径设为 SYSTEM，确保严格告警仅约束本仓库自研源码。
  target_include_directories(iir_hal_freertos SYSTEM PUBLIC
    ${IIR_HAL_DIR}/Inc
    ${IIR_HAL_DIR}/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${IIR_FREERTOS_DIR}/include
    ${IIR_FREERTOS_DIR}/portable/GCC/ARM_CM4F
  )
  target_compile_options(iir_hal_freertos PRIVATE
    -Wall
    -Wno-unused-parameter
    -Wno-sign-compare
  )

  # Board Support Package drivers (strictly checked with project C11 warnings).
  # 板级支持包（BSP）驱动静态库（强制通过项目级 C11 严格告警检查）。
  file(GLOB_RECURSE IIR_BSP_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/BSP/*.c"
  )
  add_library(iir_bsp STATIC
    ${IIR_BSP_SOURCES}
  )
  set_target_properties(iir_bsp PROPERTIES C_EXTENSIONS ON)
  target_include_directories(iir_bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/BSP
  )
  target_link_libraries(iir_bsp
    PUBLIC iir_hal_freertos iir_control
    PRIVATE iir_app_warnings
  )

  # FreeRTOS application task orchestration (connects BSP drivers and pure-C App modules).
  # FreeRTOS 应用任务编排静态库（桥接 BSP 驱动与纯 C 应用模块）。
  file(GLOB_RECURSE IIR_APP_TASK_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/App/tasks/*.c"
  )
  add_library(iir_tasks STATIC
    ${IIR_APP_TASK_SOURCES}
  )
  set_target_properties(iir_tasks PROPERTIES C_EXTENSIONS ON)
  target_include_directories(iir_tasks PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/App/tasks
  )
  target_link_libraries(iir_tasks
    PUBLIC iir_bsp iir_kinematics iir_control iir_protocol iir_safety iir_hal_freertos
    PRIVATE iir_app_warnings
  )

  set(IIR_CORE_ENTRY_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_it.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/stm32f4xx_hal_msp.c"
  )
  set(IIR_STARTUP_ASM "${CMAKE_CURRENT_SOURCE_DIR}/startup_stm32f407xx.s")

  add_executable(xzcit_iir_diff.elf
    ${IIR_CORE_ENTRY_SOURCES}
    ${IIR_STARTUP_ASM}
  )
  set_target_properties(xzcit_iir_diff.elf PROPERTIES C_EXTENSIONS ON)

  target_link_libraries(xzcit_iir_diff.elf PRIVATE
    iir_tasks
    iir_bsp
    iir_kinematics
    iir_control
    iir_protocol
    iir_safety
    iir_hal_freertos
    iir_app_warnings
    m
  )

  target_link_options(xzcit_iir_diff.elf PRIVATE
    -T${IIR_LINKER_SCRIPT}
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/xzcit_iir_diff.map,--cref
  )

  iir_add_firmware_outputs(xzcit_iir_diff.elf)
endif()

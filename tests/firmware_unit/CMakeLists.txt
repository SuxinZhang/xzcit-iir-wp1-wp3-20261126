# Host unit test build configuration for pure-C firmware modules and c50c_uart3 interop.
# 纯 C 固件核心模块宿主机单元测试与 c50c_uart3 互操作测试构建配置。

find_package(Python3 REQUIRED COMPONENTS Interpreter)

# Test 1: Four-wheel differential kinematics unit tests.
# 测试 1：四轮差速正逆运动学及编码器换算单元测试。
add_executable(test_kinematics test_kinematics.c)
target_link_libraries(test_kinematics PRIVATE iir_kinematics iir_app_warnings m)
add_test(NAME test_kinematics COMMAND test_kinematics)

# Test 2: Incremental PI, smoother, low-speed compensation, and Motor_ApplyOutput gate tests.
# 测试 2：增量式 PI、速度平滑、低速补偿与唯一电机 PWM 出口门禁单元测试。
add_executable(test_control test_control.c)
target_link_libraries(test_control PRIVATE iir_control iir_app_warnings m)
add_test(NAME test_control COMMAND test_control)

# Test 3: protocol_v3 framing, session/seq gate, resync, and watchdog unit tests.
# 测试 3：protocol_v3 编解码、会话/序号门控、流重同步及指令看门狗单元测试。
add_executable(test_protocol_v3 test_protocol_v3.c)
target_link_libraries(test_protocol_v3 PRIVATE iir_protocol iir_app_warnings)
add_test(NAME test_protocol_v3 COMMAND test_protocol_v3)

# Test 4: WP2 safety state machine, HC-SR04, ICM-20948 axis map, D50A RS485, CAN dock, and protocol ext tests.
# 测试 4：WP2 安全状态机、超声波、IMU 轴映射、D50A 协议、CAN 回充与扩展协议单元测试。
add_executable(test_safety_and_wp2
  test_safety_and_wp2.c
  ${CMAKE_SOURCE_DIR}/Drivers/BSP/hc_sr04/bsp_hc_sr04.c
  ${CMAKE_SOURCE_DIR}/Drivers/BSP/icm20948/bsp_icm20948.c
  ${CMAKE_SOURCE_DIR}/Drivers/BSP/d50a_rs485/bsp_d50a.c
  ${CMAKE_SOURCE_DIR}/Drivers/BSP/can_auto_dock/bsp_can_dock.c
)
target_include_directories(test_safety_and_wp2 PRIVATE
  ${CMAKE_SOURCE_DIR}/Drivers/BSP
)
target_link_libraries(test_safety_and_wp2
  PRIVATE iir_safety iir_protocol iir_app_warnings m
)
add_test(NAME test_safety_and_wp2 COMMAND test_safety_and_wp2)

# Test 5: CLI bridge exercising byte-level interoperability with Master/ROS c50c_uart3.
# 测试 5：与 Master/ROS c50c_uart3 真实 Python 实现进行逐字节联调的 CLI 桥与测试。
add_executable(proto_bridge_cli proto_bridge_cli.c)
target_link_libraries(proto_bridge_cli PRIVATE iir_protocol iir_app_warnings)
add_test(
  NAME test_protocol_interop
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test_c50c_uart3_interop.py
          $<TARGET_FILE:proto_bridge_cli>
)

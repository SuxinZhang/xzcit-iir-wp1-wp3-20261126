# FIX-03 ROS 2 基线缩水核查记录

日期：2026-11-26
对比对象：`025918d`（WP1 末尾基线）与 `cc69617`（WP2 当前基线）

## 数字

- `.github/ros2-test-baseline.txt`：18,950 B → 12,680 B，删除 60 行测试白名单。
- 被删除的测试文件：
  - `Master/ROS/ros2_ws/src/c50c_uart3/test/firmware_harness/fw_ros_drive_model.py`
  - `Master/ROS/ros2_ws/src/c50c_uart3/test/firmware_harness/fw_ros_mode_model.py`
  - `Master/ROS/ros2_ws/src/c50c_uart3/test/test_chassis_cmd/test_ros_mode_output.py`
  - `Master/ROS/ros2_ws/src/c50c_uart3/test/test_chassis_cmd/test_ros_straight_drive.py`

## 删除原因

1. `fw_ros_drive_model.py` 从仓库外的 `2026-11-08-50pct-migrate/BALANCE/` 逐字读取旧固件源码；该目录从未纳入 Git，干净 CI 中必然失败。
2. `fw_ros_mode_model.py` 从 `baseline-src/`（旧 `source.tar.gz` 解包）读取 `balance_task.c`/`uartx_callback.c`，并围绕 PS2 手柄 `GamePadInterface`、`_PS2_Control`、`g_rear_ps2_valid` 建模；新版任务书已永久移除 PS2/USB 手柄，旧模型不再适用。
3. 两个测试文件继承上述外部目录依赖，因此一起删除；基线中的 60 行对应这些旧失败用例及 WP2 重构后已修复的旧用例，不应作为“永久失败白名单”继续保留。

## 等效行为已恢复到新 App 层

| 旧模型覆盖 | 新等效测试 |
| :--- | :--- |
| ROS 速度帧 → 三轴 → 四轮输出 → 停车 | `tests/firmware_unit/test_kinematics.c`：`0.01 m/s` 往返、`1.3 m/s` 限幅、逆解四轮输出；`tests/firmware_unit/test_ros_equivalence.c`：1.3 m/s / 0.01 m/s 端到端 |
| 增量式 PI、速度平滑、低速补偿 | `tests/firmware_unit/test_control.c`：PI 饱和、平滑、低速补偿、`Motor_ApplyOutput` 唯一出口 |
| 模式切换安全门限 | `tests/firmware_unit/test_safety_and_wp2.c`：PE4 急停、PE5 SOS、UART 超时、欠压、D50A；`test_ros_equivalence.c`：安全门全零 |
| 串口协议对齐 | `tests/firmware_unit/test_protocol_v3.c` + `test_protocol_interop.py`：帧字节、会话/序号、CLI 与 c50c_uart3 互操作 |

## 基线策略

- 不恢复旧 60 行白名单：恢复它会把已经由新代码/新测试覆盖的失败重新当作“已知失败”，掩盖回归。
- 若旧测试需要在 ROS 2 工作空间保留，应新建 `test_ros_equivalence.py`，从 `App/` 纯 C 模块读取，而不是从外部旧的 `2026-11-08-50pct-migrate`/`baseline-src` 读取。
- 每次 `colcon test` 后必须执行 `python3 .github/scripts/compare_ros2_tests.py`；基线外失败一律阻断。

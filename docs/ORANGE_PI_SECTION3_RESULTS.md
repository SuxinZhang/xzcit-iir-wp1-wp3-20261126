# Orange Pi 5 Pro：WP_INTEGRATION_FIX 第 3 节执行记录

日期：2026-11-26
目标机：`orangepi5pro` / `aarch64` / Ubuntu 22.04 / ROS 2 Humble
SSH：`orangepi@192.168.31.148`（使用 `~/.ssh/keys/orangepi.pem`）
源码目录（独立，不覆盖现有 `/home/orangepi/iir-wp5/src`）：
`/home/orangepi/iir-wp5/src-integration-20261126`

> 本记录只执行测试与构建，没有发送运动命令、没有烧录 STM32、没有重启服务。

## 1. 编码门禁

命令：

```bash
python3 scripts/check_encoding.py
```

结果：

```text
ENCODING CHECK PASSED: 465 file(s) verified (UTF-8 without BOM, LF only).
EXIT=0
```

## 2. STM32 交叉编译

命令：

```bash
cmake --preset stm32-diff-release
cmake --build --preset stm32-diff-release
```

结果：**目标机不可执行**

- 目标机没有 `arm-none-eabi-gcc`
- 目标机没有 `ninja`（CMake 配置即失败）
- 目标机 `sudo` 需要密码，无法安装工具链

本地开发机同一命令已验证通过并产出：

```text
build/stm32-diff-release/xzcit_iir_diff.elf
build/stm32-diff-release/xzcit_iir_diff.hex
build/stm32-diff-release/xzcit_iir_diff.bin
```

## 3. 固件宿主机测试

目标机无 Ninja，改用等价的 CMake Makefiles 配置执行：

```bash
cmake -S . -B build/host-test-make -DBUILD_HOST_TESTS=ON \
      -DCHASSIS_TYPE=DIFF -G "Unix Makefiles"
cmake --build build/host-test-make --parallel 2
ctest --test-dir build/host-test-make --output-on-failure
```

结果：**6/6 通过**

```text
test_kinematics ................ Passed
test_control ................... Passed
test_protocol_v3 ............... Passed
test_safety_and_wp2 ............ Passed
test_protocol_interop .......... Passed
test_ros_equivalence ........... Passed
100% tests passed, 0 tests failed out of 6
```

## 4. ROS 2 全工作空间

构建：

```bash
cd Master/ROS/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

结果：**8/8 包构建成功**（`c50c_odom`、`c50c_uart3`、`opi_robot`、`iir_navigation`、`iir_navigation_vision`、`iir_interfaces`、`iir_vision_rknn`、`iir_interaction`）。

测试与基线：

```bash
export PYTHONPATH="/tmp/iir-ros-aiohttp:${PYTHONPATH:-}"
colcon test --executor sequential --event-handlers console_cohesion+
python3 .github/scripts/compare_ros2_tests.py
```

结果：

```text
用例总数 1818，skip 9，基线内失败 118，新增失败 0
OK：无新增测试失败
```

说明：

- 目标机系统 Python 为 NumPy 1.21 + cv2 4.5.4，测试时额外注入 `/tmp/iir-ros-aiohttp` 以补齐 `aiohttp`；未使用带 NumPy 2.2 的旧 venv，避免 cv2 的 `_ARRAY_API` 崩溃。
- `test_weaknet_proxy` 的毫秒级用例在并行负载下偶发超时；`--executor sequential` 下全套通过。
- `test_lidar_stale_points` 的 launch 测试在目标机因 `launch_ros` 版本差异被 `pytest.importorskip` 跳过（skip 不计失败）；源码中的 launch 文件本身可被系统 ROS 导入。
- `test_odom_node::test_odom_reconnect_resets_time_base_explicitly` 单独运行通过；全量并行时偶发时序抖动。
- 基线内 118 个既有失败仍然存在（属于 `.github/ros2-test-baseline.txt` 的历史白名单），本次没有新增失败。

## 5. Gateway 与部署测试

目标机使用隔离依赖 `/tmp/iir-gw-deps`（fastapi/uvicorn/httpx）：

```bash
PYTHONPATH=/tmp/iir-gw-deps python3 -m pytest \
  Master/Gateway/tests deploy/test/test_deployment.py \
  deploy/test/test_integration_contract.py -q
```

结果：

```text
33 passed, 1 warning
```

## 6. Web-Next Rust / JS 测试

目标机环境：

```text
cargo: 未安装
rustc: 未安装
node: 未安装
npm: 未安装
```

因此 `cargo test` 与 `node --test` **无法在 Orange Pi 上执行**；应在开发机或已安装 Rust/Node 的构建机执行。本地开发机已验证：

- `cargo test -- --test-threads 1`：53/53 通过
- `node --test web/test/interaction.test.cjs`：10/10 通过
- 全部 `web/js/*.js`：`node --check` 通过

## 结论

- 编码门禁：目标机通过
- 宿主机固件测试：目标机通过（Makefiles 替代缺失 Ninja）
- ROS 2 构建：8/8 包通过
- ROS 2 基线：**新增失败 0**
- Gateway/部署：33/33 通过
- STM32 交叉编译、Web-Next Rust/JS：受目标机工具链缺失限制，需在开发机执行
- 实车烧录、PE4/PE5 物理安全、RKNN NPU、Caddy/MediaMTX 音视频仍属于上机验收，不在本次测试范围

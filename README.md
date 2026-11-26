# XZCIT-IIR WP1–WP3 基础工程快照

本仓库是 XZCIT-IIR 巡检机器人 **WP1–WP3 基础工程** 的本地快照，包含：

- WP1：UTF-8/LF 编码门禁、CMake/CMakePresets、STM32F407 HAL + FreeRTOS 固件底座、`App/kinematics|control|protocol|tasks`
- WP2：C50C-V4.3 硬件驱动（HC-SR04、ICM-20948、D50A、CAN 回充、SOS）、安全状态机、`protocol_v3_ext`
- WP3：Nav2 三传感器代价地图、融合定位配置、两段式自主回充状态机、`wp3_charging.yaml`
- `docs/`：两版任务书、《WP1–WP5 任务书对照与上机验证清单》、`WP3_IMPLEMENTATION.md`

## 为什么拆成三个仓库

XZCIT-IIR 被拆为 **WP1–WP3 基础工程（本仓库）**、**WP4 视觉/Gateway/Web**、**WP5 音频/WebRTC/部署** 三个独立仓库，原因：

1. **运行环境不同**
   - WP1–WP3 面向 STM32 固件、ROS 2 导航与回充，主要依赖 C/CMake/HAL 与 ROS 2。
   - WP4 依赖 RK3588S NPU、RKNN Toolkit Lite2、Python FastAPI、OpenCV 与 Web-Next 前端。
   - WP5 依赖 PulseAudio/WebRTC AEC、MediaMTX、Caddy、systemd/udev 与 Orange Pi 目标机。
   把三者放在一个仓库会让固件构建被 Python/Node/媒体依赖污染，也会让一次小改动触发全部重测。

2. **失败域与回滚边界不同**
   - WP1–WP3 是安全底盘与导航基础，必须保持稳定、可烧录、可回滚。
   - WP4 的模型、NPU 驱动、Web 页面可以单独迭代、单独回滚，不影响固件基线。
   - WP5 的音频/部署是目标机系统层，适合独立版本化与独立安装/卸载。

3. **验收与上机条件不同**
   - WP1–WP3：需要 STM32 烧录、电机/传感器、Nav2/回充实车场景。
   - WP4：需要真实 `.rknn` 模型、NPU 性能与浏览器联动。
   - WP5：需要音频设备/AEC、MediaMTX/WebRTC、整机 systemd 冷启动。
   分开后，每个仓库可以按自己的验收清单独立推进，不互相阻塞。

4. **代码所有权更清晰**
   - 基础仓库只保留 WP1–WP3 的可运行代码；WP4/WP5 以 **overlay** 形式放在各自仓库，避免目录、CI、依赖互相覆盖。

## 仓库关系

```text
WP1–WP3 基础工程（本仓库）
        ↓ overlay
WP4：iir_interfaces + iir_vision_rknn + Master/Gateway + Web-Next inspection/student
        ↓ overlay
WP5：iir_interaction + deploy/MediaMTX/audio + Web-Next interaction
```

- WP4：`https://github.com/SuxinZhang/xzcit-iir-wp4-20261126`
- WP5：`https://github.com/SuxinZhang/xzcit-iir-wp5-20261126`

> WP4/WP5 仓库包含的是 **overlay 文件**，需要合并到本基础工程后才是完整可运行系统；不要在基础仓库中直接加入 WP4/WP5 的共享文件变更。

## 本仓库包含/不包含

包含：

- `App/`、`Core/`、`Drivers/`、`Middlewares/`、`cmake/`、`CMakeLists.txt`、`CMakePresets.json`
- `tests/firmware_unit/`
- `Master/ROS/ros2_ws/src/c50c_uart3`、`c50c_odom`、`opi_robot`
- `Master/ROS/nav2`、`Master/ROS/config`
- `Master/Web-Next` 的基线版本（不含 WP4 巡检页、学生端、interaction）

不包含：

- `Master/Gateway/`
- `Master/ROS/ros2_ws/src/iir_interfaces/`
- `Master/ROS/ros2_ws/src/iir_vision_rknn/`
- `Master/Web-Next/web/js/inspection.js`、`student.js`、`student.html`、`inspection.css`、`student.css`
- WP5 的 `iir_interaction`、`deploy/`、Web-Next interaction

## 本地验证摘要

| 检查 | 结果 |
| :--- | :--- |
| `python3 scripts/check_encoding.py` | PASS（本快照构建时） |
| `cmake --preset host-test` + CTest | 5/5 通过 |
| `cmake --preset stm32-diff-release` | 通过，生成 `.elf/.hex/.bin` |
| WP3 c50c 定向测试 | 19/19 |
| WP3 `opi_robot` 回充/巡逻测试 | 78/78 |
| WP3 Nav2 中继测试 | 3/3 |

## 上机前必读

`docs/WP1-WP5_任务书对照与上机验证清单.md` 中所有带 `[板]`、`[车]`、`[ROS]`、`[模型]`、`[人工]` 的条目仍未完成。重点：

- WP1：烧录、OLED、时钟/FreeRTOS、四轮方向、`c50c_uart3` 串口直驱
- WP2：PE9/PE11 超声波 20–200 cm、PE5 SOS 50 ms 停机、D50A/CAN 回充
- WP3：相机标定/TF、1.3 m/s、0.8 m 转弯半径、`wp3_charging.yaml` 现场参数、完整回充闭环
- 具体逐项见清单，不能以本地单元测试代替实车验收

## 构建命令

```bash
python3 scripts/check_encoding.py
cmake --preset host-test && cmake --build --preset host-test && ctest --preset host-test
cmake --preset stm32-diff-release && cmake --build --preset stm32-diff-release
```

> 本仓库是本地快照，不是原仓库，也未向原仓库推送。

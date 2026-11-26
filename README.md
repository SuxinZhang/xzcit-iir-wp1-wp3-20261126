# XZCIT-IIR 巡检机器人固件

基于 STM32F4（Cortex-M4F）+ FreeRTOS 的巡检机器人下位机固件工程。
构建体系采用 CMake（替代原 Makefile），支持交叉编译与宿主机单元测试两条构建路径。

## C50C_HAL.ioc 冻结说明

`C50C_HAL.ioc` only records the initial CubeMX pin configuration and is not the build source of truth.
Do **not** regenerate or overwrite `Core/` with CubeMX. All pin, clock, peripheral and FreeRTOS changes must be made in the checked-in CMake/C sources and reviewed as normal code.

`C50C_HAL.ioc` 仅记录初始 CubeMX 引脚配置，不是构建的唯一事实来源。**禁止**用 CubeMX 重新生成或覆写 `Core/`；所有引脚、时钟、外设与 FreeRTOS 变更必须在仓库内的 CMake/C 源码中修改并按普通代码评审。

## 目录总览

```text
XZCIT-IIR/
├── CMakeLists.txt               # 顶层固件 CMake（Debug/Release + 内存占用报告）
├── CMakePresets.json            # 一键预设：stm32-diff-release / host-test
├── cmake/                       # 交叉编译工具链
├── Core/                        # CubeMX HAL 生成目录（保持不变）
├── Drivers/
│   ├── STM32F4xx_HAL_Driver/    # ST 官方 HAL 库
│   └── BSP/                     # 板级支持包（一个外设一个子目录）
├── App/                         # 应用层（与硬件解耦的可测试业务逻辑）
└── tests/
    └── firmware_unit/           # 宿主机单元测试
```

## 各目录详细说明

### 根目录

| 文件 | 说明 |
|------|------|
| `CMakeLists.txt` | 顶层固件构建脚本：支持 Debug/Release 双配置，链接完成后输出 Flash/RAM 内存占用报告 |
| `CMakePresets.json` | 预设两条构建路径：`stm32-diff-release`（ARM 交叉编译固件）、`host-test`（宿主机跑单元测试） |

### cmake/

| 文件 | 说明 |
|------|------|
| `arm-none-eabi.cmake` | GCC 交叉编译工具链定义：`arm-none-eabi-gcc`，Cortex-M4F 硬浮点（`-mfpu=fpv4-sp-d16 -mfloat-abi=hard`） |

### Core/

CubeMX 生成的 HAL 工程目录，**已冻结**（禁止重新生成覆写）。包含 `main.c`、`stm32f4xx_it.c`（中断服务函数）、`FreeRTOSConfig.h` 等。用户代码只允许写在 `/* USER CODE BEGIN */` 与 `/* USER CODE END */` 注释区间内；所有引脚/时钟/外设变更必须直接修改仓库源码并走普通代码评审。

### Drivers/STM32F4xx_HAL_Driver/

ST 官方 HAL 库源码，由 CubeMX 复制进工程，**禁止修改**。

### Drivers/BSP/（板级支持包）

每个外设一个子目录，`xxx.c` / `xxx.h` 配对成对放置。BSP 层是唯一允许调用 HAL/操作寄存器的位置。

| 子目录 | 状态 | 说明 |
|--------|------|------|
| `hc_sr04/` | 【新增，PE9/PE11】 | 宽压版 HC-SR04 超声波测距：使用 `TIM1_CH1`（`PE9` ECHO）硬件输入捕获 + `PE11`（TRIG），停用冲突的原航模 `TIM1` 捕获逻辑 |
| `icm20948/` | 【新增，替代 MPU6050】 | IMU 驱动：引脚 `PB8(SDA) / PB9(SCL) / PD3(INT)`；因 C50C-V4.3 原理图与 STM32F407 硬件 `I2C1` 物理反接，采用 400 kHz 开漏软件 I2C（DWT 节拍 + 总线恢复）。放置寄存器定义、初始化、姿态数据读取与融合接口 |
| `d50a_rs485/` | 【迁移自 C50X】 | D50A 驱动器监控：USART2（PD5-TX / PD6-RX）轮询读取电流、温度、报警状态。放置 RS485 帧收发、解析、超时重连逻辑 |
| `can_auto_dock/` | 【迁移自 C50X】 | 自动回充：CAN1（PD0-RX / PD1-TX），接收四路红外对管回充信号与充电电流状态。放置 CAN 报文过滤、回充状态判断 |
| `sos_button/` | 【新增】 | 急停按钮：PE5 外部中断（EXTI）触发，联动声光报警——PD11 蜂鸣器 + AT8236 驱动 RGB。放置消抖、中断回调、报警控制 |
| `motor/` | 【保留】 | 电机 PWM / 方向控制驱动 |
| `encoder/` | 【保留】 | 编码器正交解码与测速 |
| `oled/` | 【保留】 | OLED 屏显示驱动（含字库） |
| `adc/` | 【保留】 | ADC 采集（电池电压等模拟量） |
| `buzzer/` | 【保留】 | 蜂鸣器基础驱动 |
| `bsp_RGBLight/` | 【保留】 | RGB 氛围灯驱动 |
| `uartx/` | 【保留】 | 通用串口收发驱动（DMA + 空闲中断接收） |

### App/（应用层）

与具体硬件解耦的业务逻辑，**禁止直接调用 HAL**，只能通过 BSP 接口访问外设——这是保证能在宿主机上跑单元测试的前提。

| 子目录 | 状态 | 说明 |
|--------|------|------|
| `kinematics/` | 【从 balance_task.c 拆出】 | 纯 C 四轮差速运动学：期望 `vx / vw` → 四轮轮速解算，含 0.01 ~ 1.3 m/s 速度限幅。无任何 HAL 依赖 |
| `control/` | 【从 balance_task.c 拆出】 | 控制链：增量式 PI 闭环、速度平滑滤波、**唯一的电机 PWM 输出口**（其他模块不得绕过它直接操作电机） |
| `safety/` | 【新增】 | 安全状态机：急停 / SOS 按钮、上位机串口失联超时（> 500 ms，已移除 PS2 手柄控制与断链安全门）、电池欠压。本地优先策略——任何安全事件不依赖上位机即可触发停车与报警 |
| `protocol/` | 保持对齐 | 与上位机（Master / ROS / c50c_uart3）对齐的 protocol_v3 帧编解码：帧头 0xAA55、Msg ID、负载、CRC16。纯 C 无硬件依赖 |
| `tasks/` | 【精简】 | FreeRTOS 任务入口与调度：创建任务、设定优先级与栈大小，将 BSP 驱动与 App 模块组装起来 |

### tests/firmware_unit/

宿主机（PC）GCC 编译的单元测试，针对 App 下无硬件依赖的纯 C 模块：`kinematics`、`control`、`protocol`。通过 CMake 预设 `host-test` 一键编译运行，不依赖任何 HAL。

## 分层约定

```text
tasks（调度入口）
   │
App/（kinematics / control / safety / protocol）   ← 纯 C，可单元测试
   │
Drivers/BSP/（外设驱动）                            ← 唯一可调用 HAL 的层
   │
Core/ + STM32F4xx_HAL_Driver/（CubeMX 生成，不改）
```

- **自上而下依赖**：`App` 只调用 `BSP` 接口，不碰 HAL；`BSP` 不反向依赖 `App`。
- **单一输出口**：电机 PWM 只能经 `App/control` 输出；蜂鸣器/RGB 报警联动由 `App/safety` 统一裁决。
- **可测试性**：新增业务逻辑优先写成纯 C 模块放入 `App/`，配套测试放入 `tests/firmware_unit/`。

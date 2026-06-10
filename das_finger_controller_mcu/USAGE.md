# DAS Finger Controller MCU — 使用文档

STM32F103C8T6 + LK MG4005 电机的机械手指（夹爪）控制器固件。本文档讲清楚如何把板子接上电、编译烧录、连上 finger 设备并测试。

> 文档基于代码仓库当前状态生成。版本日志见 [MDK-ARM/Readme.txt](MDK-ARM/Readme.txt)。

## 1. 系统概览

```
+--------------+   USART1@921600 (DAS frame)   +-----------+
|  上位机/SDK  | <===========================> |           |
+--------------+                               |  STM32F1  |   CAN@1Mbps   +----------+
                                               |  (本固件) | <===========> | MG4005   |
+--------------+   I2C1 (PB6/PB7)              |           |               | 电机     |
| 触觉传感器 L | <===========================> |           |               +----------+
+--------------+                               |           |
+--------------+   I2C2 (PB10/PB11)            |           |
| 触觉传感器 R | <===========================> |           |
+--------------+                               +-----------+
```

| 组件 | 接口 | 说明 |
|---|---|---|
| 上位机 | USART1 (PA9 TX / PA10 RX, 921600 8N1) | DMA + IDLE 中断收发，帧头/帧尾 `das\r\n` |
| 电机 MG4005 | CAN1 (PA12 TX / PA11 RX, 1 Mbps) | StdId `0x141`，速度环模式 `0xA2` |
| 触觉左 | I2C1 (Fast Mode) | 502 字节/帧，含 CRC16-Modbus |
| 触觉右 | I2C2 (Fast Mode) | 同上 |
| 编码器 | SPI1 | KTH71xx（当前固件未启用，主循环走 CAN 电机自带编码器） |
| 校准限位 | PB0 (Key1, 输入上拉) | [main.c:230](Core/Src/main.c#L230) 中已硬编码 `Key_State=1`，目前不依赖物理开关 |

时钟：HSE 8 MHz × PLL9 = 72 MHz（[main.c:616-649](Core/Src/main.c#L616-L649)）。
定时器：TIM3 → 50 Hz 主循环节拍，TIM2 → 30 Hz 触觉触发 + CAN 看门狗。

## 2. 硬件接线

| MCU 引脚 | 信号 | 接到 |
|---|---|---|
| PA9 | USART1_TX | USB-TTL RX（**3.3 V** 电平，921600） |
| PA10 | USART1_RX | USB-TTL TX |
| PA11 | CAN_RX | CAN 收发器 RXD（如 TJA1050） |
| PA12 | CAN_TX | CAN 收发器 TXD |
| PB6/PB7 | I2C1 SCL/SDA | 左触觉模组 |
| PB10/PB11 | I2C2 SCL/SDA | 右触觉模组 |
| PA1/PA2/PA3 | R_TRG/C_TRG/L_TRG | 触觉模组采样触发（30 Hz 翻转，见 [main.c:723-725](Core/Src/main.c#L723-L725)） |
| PA4..PA7 | SPI1 CS/SCK/RST/SDA | KTH71 编码器（可选） |
| PB0 | Key1 | 校准限位开关（可不接，固件目前不依赖） |
| SWD | SWDIO/SWCLK | ST-Link |

CAN 总线两端记得各加 120 Ω 终端电阻。MG4005 电机的电源由电机自身的供电系统提供，MCU 只负责 CAN 信号。

## 3. 工具链

| 工具 | 用途 |
|---|---|
| Keil µVision 5（MDK-ARM） | 打开 [MDK-ARM/KTH71_Example.uvprojx](MDK-ARM/KTH71_Example.uvprojx) 编译/烧录 |
| STM32CubeMX | 改外设配置时打开 [KTH71_Example.ioc](KTH71_Example.ioc) |
| ST-Link Utility 或 Keil 内置 | 烧录 |
| 串口工具 | 任意（SSCOM/MobaXterm/`miniterm`），波特率 921600 |
| CAN 工具（可选调试） | 用 USB-CAN 设备时确认电机能独立响应 |

## 4. 编译与烧录

1. Keil µVision 打开 [MDK-ARM/KTH71_Example.uvprojx](MDK-ARM/KTH71_Example.uvprojx)。
2. 确认 Target：`STM32F103C8`，时钟 72 MHz。
3. `Project → Build Target`（F7）。
4. 接好 ST-Link，`Flash → Download`（F8）。
5. 复位上电，PA9 上能观察到 50 Hz 数据上报（见 §6）。

## 5. 通信协议（USART1）

### 5.1 帧格式

所有"带帧结构"的数据都以 5 字节头尾包裹：

```
[ 'd' 'a' 's' 0x0D 0x0A ] [ payload ... ] [ 'd' 'a' 's' 0x0D 0x0A ]
```

定义见 [Core/Inc/usart.h:36-41](Core/Inc/usart.h#L36-L41)。MCU 收到完整帧（IDLE 触发）后才解析。

### 5.2 上位机 → MCU 命令汇总

裸命令（不需要帧头帧尾，直接发字节即可）：

| 命令字节 | 长度 | 作用 | 实现位置 |
|---|---|---|---|
| `1234` | 4 | 进入**手动校准模式**：电机正反扫到限位，自动写入 Flash 最大行程 | [stm32f1xx_it.c:302](Core/Src/stm32f1xx_it.c#L302) |
| `MCUID` | 5 | 读取 MCU 的 96-bit 唯一 ID | [stm32f1xx_it.c:310](Core/Src/stm32f1xx_it.c#L310) |
| `CALIB` | 5 | **自动校准检查**：扫一遍量程，对比理论值并回报偏差 | [stm32f1xx_it.c:315](Core/Src/stm32f1xx_it.c#L315) |
| `camerawl<500B>` | 508 | 写左相机标定数据到 Flash 页 60（`0x0800F000`） | [stm32f1xx_it.c:331](Core/Src/stm32f1xx_it.c#L331) |
| `camerawc<500B>` | 508 | 写中相机数据到 Flash 页 61（`0x0800F400`） | 同上 |
| `camerawr<500B>` | 508 | 写右相机数据到 Flash 页 62（`0x0800F800`） | 同上 |
| `camerarl` / `rc` / `rr` | 8 | 回读对应相机标定数据 | [stm32f1xx_it.c:345](Core/Src/stm32f1xx_it.c#L345) |

带帧的**电机控制包**（帧头 `das\r\n` + payload + 帧尾 `das\r\n`），payload 首字节 `0x02` 表示电机控制：

```
偏移(payload相对位置)  字节   含义
[0]                    0x02   包类型：电机控制
[1]                    -      （保留）
[2..5]                 float  目标夹爪间距 (m)，big-endian (网络字节序)
[6]                    u8     spacing_select: 0=0.103m, 1=0.102m, 2=0.101m, 3=0.100m, 4=0.0998m
[7]                    u8     torque_flag: 0=默认扭矩(150) / 1=使用上位机给的最大扭矩
[8..9]                 int16  最大扭矩值 (100..500)，小端，仅 torque_flag=1 时生效
[10..13]               int32  备用（当前未使用）
[14]                   u8     编码器重置请求：0x01 时禁用电机（保护）
[15]                   u8     电机使能：0x00=失能 / 0x01=使能
```

解析见 [stm32f1xx_it.c:385-433](Core/Src/stm32f1xx_it.c#L385-L433)，扭矩处理在 [main.c:577-595](Core/Src/main.c#L577-L595)。

收到任一控制包后，MCU 会立即回 `TxBuffer_all` 包作为状态镜像。

### 5.3 MCU → 上位机 上报

| 包名 | 触发 | 频率 | 内容 |
|---|---|---|---|
| `TxBuffer_drive` (14 B) | TIM3 50 Hz 中断 | 50 Hz | 电机当前角度（4 B int32）映射为夹爪距离（mm，float） |
| `TxBuffer_all` | 收到控制包后立即回 | 事件触发 | 电机数据 + 触觉数据（触觉部分目前注释，全 0） |
| MCUID 应答 | 收到 `MCUID` | 一次 | `das\r\n<uid96-hex>das\r\n` |
| CALIB 应答 | 收到 `CALIB` 校准完成 | 一次 | `das\r\nCHECK_meas<N>_theo<N>_dev<N>das\r\n` |

注意：触觉数据上报在 [main.c:282-343](Core/Src/main.c#L282-L343) 整段被注释，当前固件仅上报电机距离。如要恢复触觉，需取消注释并确认 I2C 总线和 CRC 都通过。

## 6. 上电后的标准测试流程

下面是把固件烧好之后让 finger 跑起来的步骤。把 USB-TTL 接到 PA9/PA10，串口工具开到 921600。

### 第 1 步：握手

发送 ASCII：`MCUID`（5 字节，无换行）

期望收到（hex）：
```
das\r\n  XX XX XX XX  XX XX XX XX  XX XX XX XX  das\r\n
```
中间 12 字节是 STM32 的 96-bit UID。**收到说明 USART 正常**。

### 第 2 步：电机校准（首次或换板时必做）

> 校准会把电机驱到机械限位，听到/感觉到夹爪到顶后会停下来，全程约 5–10 s。**校准期间不要发其他命令**。

**方法 A（手动校准，需限位开关或人工辨认到顶）**：
1. 发送 `1234`（4 字节）。
2. 电机反向（开爪方向）转动找零位，再正向转直到达到 `ANGLE_MAX_INIT`（[pid.h:7](Application/pid.h#L7) 当前 5267 编码器单位）。
3. 校准成功后串口输出 `Calibration OK !`，最大行程写入 Flash 页 63（`0x0800FC00`）。

**方法 B（自动校准检查，推荐用于产线复检）**：
1. 发送 `CALIB`（5 字节）。
2. 电机自动扫描完整行程，**不写 Flash**，只回报实测/理论/偏差：
   ```
   das\r\nCHECK_meas5230_theo5267_dev-37das\r\n
   ```
   `dev` 绝对值小于几十算正常；偏差大说明机械装配或电机零位异常。

### 第 3 步：电机使能 + 位置控制

构造一帧控制包（伪 Python，二进制）：

```python
import struct
header = b'das\r\n'
target_m = 0.050              # 目标开口 50 mm
payload = bytearray(16)
payload[0]  = 0x02            # 电机控制
payload[2:6] = struct.pack('>f', target_m)   # big-endian float
payload[6]  = 0               # spacing_select=0 → DISTANCE_MAX=0.103
payload[7]  = 0               # torque_flag=0 → 用默认扭矩 150
payload[15] = 0x01            # 电机使能
frame = header + bytes(payload) + header
ser.write(frame)
```

成功的话：
- 电机以速度环 + 自写位置环走到目标位置；
- 每 20 ms 上位机收到一帧 `TxBuffer_drive`，[10..13] 字节是当前夹爪距离 (mm, float, little-endian)。

### 第 4 步：断电保护 / 急停

- **失能电机**：把 `payload[15]` 改成 `0x00` 发一次，电机即停 (`CAN_LKMotor_speed(0x141, 1, 0)`)。
- **过温保护**：电机温度 > 80 °C 时固件主动停转 ([main.c:539](Core/Src/main.c#L539), [main.c:547](Core/Src/main.c#L547))。
- **看门狗**：MCU 4 个 30 Hz tick（约 130 ms）没收到 CAN 反馈会重初始化 CAN ([main.c:711-718](Core/Src/main.c#L711-L718))。

## 7. 关键参数 / 调参点

| 参数 | 位置 | 当前值 | 说明 |
|---|---|---|---|
| `ANGLE_MAX_INIT` | [pid.h:7](Application/pid.h#L7) | 5267 | 电机标定理论行程（编码器增量单位）。不同设备可能不同。 |
| `ANGLE_OFFSET` | [pid.h:8](Application/pid.h#L8) | 50 | 校准后的安全偏移 |
| `DISTANCE_MAX` | [pid.c:13](Application/pid.c#L13) | 0.103 m | 夹爪满程，可被 `spacing_select` 覆盖 |
| `motor_4005_angle[]` | [pid.c:17](Application/pid.c#L17) | Kp=30, Ki=0, Kd=0 | 位置环 PID |
| `motor_4005_speed[]` | [pid.c:18](Application/pid.c#L18) | Kp=25, Ki=0, Kd=0 | 速度环 PID |
| 默认扭矩限制 | [main.c:589](Core/Src/main.c#L589) | 150 (~1 N·m) | 上位机未指定时用 |
| 扭矩允许范围 | [main.c:584-585](Core/Src/main.c#L584-L585) | 100..500 | 上位机指定值会被钳位 |
| UART 波特率 | [usart.c:115](Core/Src/usart.c#L115) | 921600 | |
| CAN 波特率 | [KTH71_Example.ioc](KTH71_Example.ioc) | 1 Mbps | Prescaler=4, BS1=3TQ, BS2=5TQ |

## 8. 常见问题排查

| 现象 | 可能原因 | 排查 |
|---|---|---|
| `MCUID` 无响应 | 串口接反/电平不对/波特率错 | 万用表确认 PA9 有 3.3 V 空闲电平；921600 8N1 不带流控 |
| 收到 `MCUID` 但发 `1234` 电机不动 | CAN 不通 / 电机未上电 / `0x141` 不匹配 | 用 USB-CAN 抓总线，`0x141` 应有周期性反馈帧 |
| 上报的距离一直是 `-66.66` | `MG_Reset` 未置 1，即电机零位未找到 | 重新发 `1234` 完成校准 |
| 校准完一段时间后失能 | 电机过温（>80 °C） | 散热或降低占空比，看 `MG4005Motor.iqControl.temperature` |
| 触觉数据全 0 | 触觉上报段被注释 | 见 §5.3 备注；恢复 [main.c:282-343](Core/Src/main.c#L282-L343) |
| CAN 偶尔断流 | 总线干扰 / 终端电阻缺失 | 看 `ALLflag.MG_Can_RST` 是否周期性触发重初始化 |

## 9. 文件索引

| 路径 | 作用 |
|---|---|
| [Core/Src/main.c](Core/Src/main.c) | 主循环：校准状态机、电机驱动、上报调度 |
| [Core/Src/stm32f1xx_it.c](Core/Src/stm32f1xx_it.c) | USART IDLE 中断里的协议解析（**所有上行命令在这里**） |
| [Core/Src/can.c](Core/Src/can.c) | CAN 收发 + 电机帧编码 + 207 项夹爪查找表 |
| [Core/Src/usart.c](Core/Src/usart.c) | USART1 DMA 收发，`DMA_Send_Packet` 包装帧头帧尾 |
| [Core/Src/tactile.c](Core/Src/tactile.c) | 触觉数据 CRC16-Modbus 校验 + 解包 |
| [Application/pid.c](Application/pid.c) | 位置环 / 速度环 PID + 距离↔电机圈数换算 |
| [Application/RWflash.c](Application/RWflash.c) | Flash 读写（校准值 + 3 颗相机标定数据） |
| [MDK-ARM/KTH71_Example.uvprojx](MDK-ARM/KTH71_Example.uvprojx) | Keil 工程 |
| [KTH71_Example.ioc](KTH71_Example.ioc) | STM32CubeMX 配置 |

# Gen Controller SDK (Python)

新一代灵巧手指 Python SDK。每只夹爪 = 1 个串口 (CH340) + 1 个 USB 相机。

纯 Python 实现，无 ROS 依赖。通过串口与 STM32 MCU 通信，支持夹爪开合控制、力矩控制、编码器反馈、触觉传感和相机采集。

---

## 目录

1. [环境准备](#1-环境准备)
2. [硬件接口配置 (udev)](#2-硬件接口配置-udev)
3. [快速启动](#3-快速启动)
4. [编程接口 API](#4-编程接口-api)
5. [通信协议](#5-通信协议)
6. [设备参数获取](#6-设备参数获取)
7. [故障排查](#7-故障排查)
8. [注意事项](#8-注意事项)

---

## 1. 环境准备

### 1.1 系统依赖

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3-full v4l-utils
```

> USB 接口必须使用 **USB 3.0**。

### 1.2 Python 虚拟环境

```bash
cd gen_con_sdk_python_release-master
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

> 每次开新终端运行 SDK 前都要先 `source venv/bin/activate`，命令行最前面出现 `(venv)` 就说明环境激活成功。

### 1.3 依赖列表

| 包名 | 版本要求 | 用途 |
|------|---------|------|
| `pyserial` | >=3.5 | 串口通信 |
| `opencv-python` | >=4.5.0 | 相机采集与预览 |
| `numpy` | >=1.19.0 | 数组运算 |

---

## 2. 硬件接口配置 (udev)

udev 规则将 USB 设备绑定到固定的软链接名称（如 `/dev/ttyFingerLeft`），避免每次插拔后设备号变化。**配置一次，以后插同一个口就不用再配。**

### 2.1 设备命名约定

| 设备 | 左夹爪 | 右夹爪 |
|------|--------|--------|
| 串口 | `/dev/ttyFingerLeft` | `/dev/ttyFingerRight` |
| 相机 | `/dev/finger_camera_left` | `/dev/finger_camera_right` |

### 2.2 查找 USB 路径

#### 串口路径

插上夹爪，执行：

```bash
cd /dev && ls | grep ttyUSB
udevadm info -a -n /dev/ttyUSB0 | grep -E "KERNELS|DRIVERS"
```

找输出中 `DRIVERS=="ch341"` 那一组对应的 `KERNELS` 值，例如 `1-1.3:1.0`。

#### 相机路径

```bash
v4l2-ctl --list-devices
```

找到夹爪自带相机对应的 `/dev/videoN`，然后：

```bash
udevadm info -a -n /dev/videoN | grep KERNELS | head -1
```

取第一行 `KERNELS` 值，例如 `1-1.4:1.0`。

### 2.3 编辑规则文件

编辑 `config/99-usb-serial.rules`，将 `KERNELS==` 改成你查到的值：

```bash
# 左夹爪串口
SUBSYSTEM=="tty", KERNELS=="1-1.3:1.0", SYMLINK+="ttyFingerLeft", MODE="0666"

# 左夹爪相机
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", KERNELS=="1-1.4:1.0", ATTR{index}=="0", SYMLINK+="finger_camera_left", MODE="0666"

# 右夹爪串口（双夹爪时取消注释）
# SUBSYSTEM=="tty", KERNELS=="1-1.1:1.0", SYMLINK+="ttyFingerRight", MODE="0666"

# 右夹爪相机（双夹爪时取消注释）
# SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", KERNELS=="1-1.2:1.0", ATTR{index}=="0", SYMLINK+="finger_camera_right", MODE="0666"
```

**双夹爪配置**：先配好左爪，拔下左爪插上右爪，再查一遍 KERNELS 值，填入右爪对应行并取消注释。

> udev 注释符是 `#`，不是 `//`。

### 2.4 安装并加载

```bash
sudo cp config/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2.5 验证

```bash
ls -l /dev/ttyFingerLeft /dev/finger_camera_left
# 双夹爪：
ls -l /dev/ttyFingerRight /dev/finger_camera_right
```

每条都应指向某个 `ttyUSB*` 或 `video*`。

---

## 3. 快速启动

### 3.1 命令行参数

```bash
python3 start_finger.py <side> [选项]
```

**必填参数**：

| 参数 | 说明 |
|------|------|
| `side` | `left` 或 `right`，指定哪只夹爪 |

**可选参数**：

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `--distance` | float | 0.05 | 固定开合距离（米），范围 [0.0, 0.2] |
| `--sine-wave` | flag | - | 正弦波开合模式（与 `--distance` 互斥） |
| `--amplitude` | float | 0.025 | 正弦振幅（米） |
| `--center` | float | 0.05 | 正弦中心位置（米） |
| `--frequency` | float | 0.5 | 正弦频率（Hz） |
| `--duration` | float | 10.0 | 正弦持续时间（秒），0 = 永不停止 |
| `--torque-enable` | flag | - | 启用自定义力矩（需搭配 `--torque-value`） |
| `--torque-value` | int | - | 力矩值，范围 [100, 500] |
| `--camera-resolutions` | str | 1600x1296 | 相机分辨率，格式 `宽x高` |
| `--camera-fps` | int | 30 | 相机显示帧率（V4 Controller 需设为 60） |
| `--no-preview` | flag | - | 不显示相机预览窗口 |
| `--print-tactile-info` | flag | - | 终端打印触觉网格 |
| `--tactile-print-hz` | float | 0.0 | 触觉打印频率上限（Hz），0 = 不限 |

### 3.2 运行示例

```bash
# 单夹爪，默认开到 5cm
python3 start_finger.py left

# 固定开到 8cm
python3 start_finger.py left --distance 0.08

# 正弦波开合 10 秒
python3 start_finger.py left --sine-wave

# 自定义正弦参数
python3 start_finger.py left --sine-wave --amplitude 0.03 --center 0.05 --frequency 1.0 --duration 20.0

# 启用力矩控制
python3 start_finger.py left --distance 0.05 --torque-enable --torque-value 250

# V4 Controller 相机帧率设为 60
python3 start_finger.py left --camera-fps 60

# 打印触觉数据（限制 5Hz）
python3 start_finger.py left --print-tactile-info --tactile-print-hz 5.0

# 双夹爪（两个终端分别运行）
python3 start_finger.py left     # 终端 A
python3 start_finger.py right    # 终端 B
```

启动后：
- 弹出相机预览窗口
- 终端打印 `finger distance: X.XXX m`（编码器回传）
- 每 5 秒打印诊断信息 `[DIAG] sent=... recv_bytes=... parsed_pkts=... queue=...`
- 按 **ESC**（预览窗口）或 **Ctrl+C** 退出

---

## 4. 编程接口 API

### 4.1 GripperSystem

主控类，负责初始化串口通信和相机。

```python
from scripts import GripperSystem

system = GripperSystem(
    serial_port="/dev/ttyFingerLeft",     # 串口路径，None 则自动检测
    camera_resolutions="1600x1296",       # 相机分辨率
    show_preview=True,                    # 是否显示 OpenCV 预览
    video_devices=["/dev/finger_camera_left"],  # 相机设备列表
    tactile_callback=my_tactile_cb,       # 触觉回调（可选）
    encoder_callback=my_encoder_cb,       # 编码器回调（可选）
    capture_frames_callback=my_frame_cb,  # 自定义帧采集回调（可选）
    camera_fps=30,                        # 相机显示帧率
)
```

| 方法 | 说明 |
|------|------|
| `start() -> bool` | 启动相机和串口通信（阻塞，直到退出） |
| `stop()` | 停止所有线程，关闭串口 |
| `set_gripper_distance(distance: float)` | 设置目标开合距离，范围 [0.0, 0.2] 米 |

| 属性 | 类型 | 说明 |
|------|------|------|
| `databus` | `DataBus` 或 `None` | 串口通信层（start 后可用） |
| `camera` | `CameraCapture` 或 `None` | 相机采集层 |
| `running` | `bool` | 系统是否运行中 |

### 4.2 DataBus

串口通信层，管理读/写/解析线程。通常通过 `system.databus` 访问。

| 方法 | 说明 |
|------|------|
| `set_target_distance(distance: float)` | 设置夹爪目标距离，范围 [0.0, 0.2] 米 |
| `get_target_distance() -> float` | 获取当前目标距离 |
| `set_torque(enable: bool, value: int)` | 设置力矩，enable=True 时 value 范围 [100, 500]；enable=False 使用 MCU 默认值 |
| `get_torque() -> (bool, int)` | 获取当前力矩设置 |
| `drive_motor(angle_degree: float)` | 直接驱动电机角度 |
| `disable_motor()` | 禁用电机 |
| `calib_encoder()` | 请求编码器标定 |
| `send_camera_calib_cmd(cmd: str) -> bool` | 发送相机标定命令 |
| `register_tactile_callback(cb)` | 注册触觉回调 |
| `register_encoder_callback(cb)` | 注册编码器回调 |
| `register_camera_calib_callback(cb)` | 注册标定回调 |
| `stop()` | 停止线程，发送 DisableDrive，关闭串口 |
| `is_opened() -> bool` | 串口是否成功打开 |
| `get_serial_info() -> dict` | 获取串口配置信息 |

**DataBus 构造参数**（通常由 GripperSystem 内部创建）：

```python
DataBus(
    tty_port="/dev/ttyFingerLeft",  # 串口路径
    baudrate=921600,                # 波特率（固定）
    timeout=0.5,                    # 读超时
    encoder_freq=100,               # 编码器轮询频率（Hz）
    tactile_freq=None,              # 触觉轮询频率（Hz），None 不主动轮询
    tactile_callback=None,          # 触觉回调
    encoder_callback=None,          # 编码器回调
)
```

### 4.3 CameraCapture

相机采集层，通常通过 `system.camera` 访问。

| 方法 | 说明 |
|------|------|
| `capture_frames_callback()` | 默认帧采集循环（阻塞） |
| `stop()` | 停止采集，释放资源 |

| 属性 | 类型 | 说明 |
|------|------|------|
| `cameras` | `list[dict]` | 每个相机的信息字典 |
| `running` | `bool` | 是否运行中 |
| `target_fps` | `int` | 目标帧率 |
| `show_preview` | `bool` | 是否显示预览 |

### 4.4 回调函数签名

#### 编码器回调

```python
def encoder_callback(record_data: bytes):
    """每次收到编码器数据时调用。"""
    import struct
    distance = struct.unpack(">f", record_data)[0]  # 大端 float32，单位：米
    print(f"夹爪距离: {distance:.3f} m")
```

#### 触觉回调

```python
def tactile_callback(record_data: bytes):
    """每次收到触觉数据时调用。record_data 为 448 字节。"""
    from tactile_processing import convert_tactile_448_to_1000
    left_500, right_500 = convert_tactile_448_to_1000(record_data)
    # left_500: 左侧 500 个传感点数值
    # right_500: 右侧 500 个传感点数值
```

#### 相机帧回调

```python
def frame_callback(camera_id: int, frame, timestamp_ns: int):
    """每帧调用。frame 为 OpenCV BGR 图像。"""
    # camera_id: 相机索引
    # frame: numpy.ndarray (BGR)
    # timestamp_ns: 纳秒时间戳
```

#### 自定义帧采集回调

```python
def capture_frames_callback(camera):
    """替代默认帧采集循环。camera 为 CameraCapture 实例。"""
    while camera.running:
        for cam in camera.cameras:
            frame, ts = camera._get_latest(cam)
            if frame is not None:
                # 处理帧...
                pass
        time.sleep(1.0 / camera.target_fps)
```

### 4.5 完整代码示例

#### 自定义控制循环

```python
import time
import struct
from scripts import GripperSystem

def my_encoder_cb(record_data: bytes):
    distance = struct.unpack(">f", record_data)[0]
    print(f"距离: {distance:.3f} m")

system = GripperSystem(
    serial_port="/dev/ttyFingerLeft",
    video_devices=["/dev/finger_camera_left"],
    encoder_callback=my_encoder_cb,
    show_preview=False,
)

# 在后台线程中启动系统
import threading
t = threading.Thread(target=system.start, daemon=True)
t.start()

# 等待 DataBus 初始化完成
while system.databus is None:
    time.sleep(0.1)
time.sleep(0.5)

# 控制夹爪
try:
    system.databus.set_target_distance(0.08)   # 开到 8cm
    time.sleep(3)
    system.databus.set_target_distance(0.02)   # 合到 2cm
    time.sleep(3)
    system.databus.set_target_distance(0.05)   # 回到 5cm
    time.sleep(2)
finally:
    system.stop()
```

#### 力矩控制

```python
# 启用自定义力矩（范围 100~500）
system.databus.set_torque(enable=True, value=200)

# 恢复 MCU 默认力矩
system.databus.set_torque(enable=False)
```

### 4.6 触觉数据处理

```python
from tactile_processing import (
    convert_tactile_448_to_1000,
    set_tactile_grid_print_enabled,
    set_tactile_grid_print_max_hz,
    submit_tactile_1000_grid_print,
    print_tactile_1000_grid,
)
```

| 函数 | 说明 |
|------|------|
| `convert_tactile_448_to_1000(data) -> (left_500, right_500)` | 将 448 字节原始数据转换为左右各 500 个传感点数值 |
| `set_tactile_grid_print_enabled(enabled: bool)` | 开启/关闭终端网格打印 |
| `set_tactile_grid_print_max_hz(hz: float)` | 限制打印频率，0 = 不限 |
| `submit_tactile_1000_grid_print(all_1000)` | 异步提交网格打印（非阻塞） |
| `print_tactile_1000_grid(all_1000)` | 同步打印 50 行 x 20 列网格 |

---

## 5. 通信协议

SDK 通过串口（921600 波特率）与 STM32 MCU 通信，使用自定义的 DAS 协议。

### 5.1 包结构

**Magic 标识**：`das\r\n`（5 字节），作为包头和包尾。

#### 发送包 (CmdPack，主机 → STM32)

```
字节偏移  长度    内容
───────────────────────────────────
0         5B     Magic "das\r\n"（包头）
5         1B     Opcode（操作码）
6         1B     RecordType（记录类型）
7         4B     ContentLength（大端 uint32）
11        1B     MaxDistance（保留）
12        1B     TorqueEnable（0x00=使能, 0x01=失能）
13        2B     TorqueValue（大端 int16, 100~500）
15        NB     RecordData（变长数据）
15+N      1B     Calibration（保留）
16+N      1B     MotorEnable（保留）
17+N      5B     Magic "das\r\n"（包尾）
```

#### 接收包 (MessagePack，STM32 → 主机)

```
字节偏移  长度    内容
───────────────────────────────────
0         5B     Magic "das\r\n"（包头）
5         1B     Opcode
6+        --     Record 序列（可包含多条记录）：
                   1B  RecordType
                   8B  ContentLength（大端 uint64）
                   NB  RecordData
-5        5B     Magic "das\r\n"（包尾）
```

### 5.2 Opcode 操作码

| 名称 | 值 | 方向 | 说明 |
|------|----|------|------|
| ReadSingle | 0x01 | 发送 | 单次读取触觉数据 |
| ReadBatch | 0x02 | 发送 | 批量读取（编码器 + 触觉），100Hz 轮询 |
| WriteDrive | 0x03 | 发送 | 发送电机目标角度 |
| Echo | 0x04 | 双向 | 回声测试 |
| CalibEncoder | 0x05 | 发送 | 请求编码器标定 |
| DisableDrive | 0x06 | 发送 | 禁用电机驱动 |

### 5.3 RecordType 记录类型

| 名称 | 值 | 数据大小 | 格式 |
|------|----|---------|------|
| Tactile | 0x01 | 448 字节 | 原始触觉数据（左 224 + 右 224） |
| Encoder | 0x02 | 4 字节 | 大端 float32，单位：米 (0.0~0.2) |
| Drive | 0x03 | 变长 | 电机控制 |
| Echo | 0x04 | 变长 | 回声测试 |

### 5.4 协议常量

```python
MAGIC = b"das\r\n"       # 包头/包尾标识
MAX_PACKET_SIZE = 4096    # 单包最大长度
MAX_BUFFER_SIZE = 8192    # 解析缓冲区最大长度
```

### 5.5 诊断输出

运行时每 5 秒打印一次诊断信息：

```
[DIAG] sent=500 recv_bytes=12800 parsed_pkts=500 queue=0
```

| 字段 | 含义 |
|------|------|
| `sent` | 已发送的命令数 |
| `recv_bytes` | 已接收的字节数 |
| `parsed_pkts` | 已成功解析的包数 |
| `queue` | 发送队列中待发的命令数 |

`recv_bytes=0` 表示 STM32 完全没有回传数据，参见[故障排查](#7-故障排查)。

---

## 6. 设备参数获取

### 6.1 相机标定

```bash
# 单夹爪
python3 scripts/camera_cmd.py camerarc     # 中间相机标定

# 双夹爪
python3 scripts/camera_cmd.py left camerarc
python3 scripts/camera_cmd.py right camerarc
```

标定结果保存在 `scripts/calib_result/` 目录下（YAML 格式）。

> 新设备只有 1 个相机，`camerarl`/`camerarr` 不再使用。

### 6.2 查询 MCUID

```bash
# 单夹爪
python3 scripts/camera_cmd.py MCUID

# 双夹爪
python3 scripts/camera_cmd.py left MCUID
python3 scripts/camera_cmd.py right MCUID
```

| 命令 | 说明 | 输出文件 |
|------|------|---------|
| `camerarc` | 中间相机标定 | `cam0_sensor_*.yaml` |
| `camerarl` | 左相机标定（已弃用） | `cam1_sensor_*.yaml` |
| `camerarr` | 右相机标定（已弃用） | `cam2_sensor_*.yaml` |
| `MCUID` | 查询设备 ID | 终端打印 |

---

## 7. 故障排查

| 现象 | 原因 | 解决 |
|------|------|------|
| `/dev/ttyFingerLeft` 不存在 | udev 规则未生效 | 重新执行 `sudo udevadm control --reload-rules && sudo udevadm trigger`；检查 `99-usb-serial.rules` 中 KERNELS 值是否正确 |
| `recv_bytes=0`，STM32 无响应 | STM32 协议解析器错位卡死 | 拔插 USB 断电重启 STM32；参见[注意事项](#8-注意事项) |
| 相机打不开 `OpenCV failed to open device` | 上一个进程未释放相机 | `pkill -9 -f start_finger.py`，或拔插 USB |
| `Permission denied` | 串口/相机权限不足 | `sudo chmod 666 /dev/ttyUSB* /dev/video*`（临时）；或将用户加入 `dialout`、`video` 用户组 |
| 启动正常但无 `finger distance:` 打印 | 夹爪电源/电机供电问题 | 检查夹爪硬件供电；用 `sudo minicom -D /dev/ttyFingerLeft -b 921600` 测试是否有原始数据 |
| `[DIAG] queue=1000` 发送队列满 | 串口被占用或设备断开 | 检查是否有其他进程占用串口；检查 USB 连接 |
| `Warning: system init timed out` | DataBus 初始化超时 | 检查串口路径是否正确，STM32 是否上电 |
| `QObject::killTimer` 警告 | OpenCV Qt 后端线程问题 | 无害，可忽略 |

---

## 8. 注意事项

### 8.1 STM32 通信丢失问题

如果 `[DIAG]` 日志持续显示 `recv_bytes=0`（STM32 不回数据），**拔插 USB 断电重启 STM32** 是唯一有效的恢复方式。

**根本原因**：如果退出程序时正在进行的串口写操作被强行中断（如 `cancel_write()`），STM32 会收到不完整的数据包，导致协议解析器永久错位。

**预防措施**：SDK 的 `stop()` 方法确保当前正在发送的包完整写出后再关闭串口。正常使用 Ctrl+C 退出不会导致此问题。

### 8.2 串口波特率

固定为 **921600**，不可更改，需与 STM32 固件一致。

### 8.3 编码器轮询频率

默认 **100Hz**（`system.py` 中 `encoder_freq=100`）。如需调整，修改 `scripts/system.py` 中的 `encoder_freq` 参数。

### 8.4 夹爪距离范围

有效范围 **[0.0, 0.2]** 米（0 ~ 20 cm），超出范围的值会被拒绝。

### 8.5 力矩范围

自定义力矩值范围 **[100, 500]**。设为 `enable=False` 时使用 MCU 内部默认值。

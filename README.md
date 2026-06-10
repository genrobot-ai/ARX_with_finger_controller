# ARX 项目说明

本仓库是 ARX X5 机械臂与 DAS 灵巧夹爪的集成开发目录，包含机械臂 Python SDK、ROS/ROS2 控制工作空间、VR 遥操作 SDK、灵巧夹爪 Python SDK 以及 STM32 夹爪控制器固件。

项目主要面向以下场景：

- 使用 Python 直接控制 ARX X5 单臂或双臂。
- 通过 ROS/ROS2 启动机械臂控制节点和遥操作流程。
- 通过串口和相机控制 DAS 灵巧夹爪，读取编码器、触觉和图像数据。
- 将双机械臂和左右夹爪组合运行，完成联调演示。
- 编译、烧录和调试 DAS 夹爪控制器 MCU 固件。

## 目录结构

```text
ARX/
├── requirement.txt
├── README.md
├── ARX_X5/
│   ├── 00-sh/                         # ROS/ROS2 编译与启动脚本
│   ├── ARX_CAN/                       # CAN 设备配置脚本
│   ├── ARX_VR_SDK/                    # Meta Quest3 VR 遥操作 SDK
│   ├── ROS/X5_ws/                     # ROS1 工作空间
│   ├── ROS2/X5_ws/                    # ROS2 工作空间
│   ├── arx_joy/                       # 手柄和采集相关 ROS2 包
│   ├── py/arx_x5_python/              # 机械臂 Python SDK 与示例
│   └── 旧版-readme/                   # 旧版说明文档和 PDF 手册
├── finger_sdk_python/
│   └── gen_con_sdk_python_release-master/
│       ├── README_CN.md               # 灵巧夹爪中文文档
│       ├── start_finger.py            # 夹爪启动脚本
│       ├── scripts/                   # 串口、相机、协议和系统封装
│       └── config/99-usb-serial.rules # udev 设备名配置模板
└── das_finger_controller_mcu/
    ├── USAGE.md                       # MCU 使用文档
    ├── Core/                          # STM32 主程序源码
    ├── MDK-ARM/                       # Keil 工程
    └── tools/                         # 固件测试工具
```

## 环境准备

### Python 依赖

根目录依赖用于运行双臂和夹爪联调示例：

```bash
cd ~/ARX
pip install -r requirement.txt
```

主要依赖包括：

- `numpy`
- `pyserial`
- `opencv-python`

夹爪 SDK 也提供了独立依赖文件：

```bash
cd ~/ARX/finger_sdk_python/gen_con_sdk_python_release-master
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 机械臂 Python SDK 依赖

机械臂 Python 绑定需要 CAN 工具和 `pybind11`：

```bash
sudo apt update
sudo apt install -y can-utils
```

`pybind11` 可按 `ARX_X5/py/arx_x5_python/README.md` 中的方式从源码编译安装。

编译 Python 接口：

```bash
cd ~/ARX/ARX_X5/py/arx_x5_python
./build.sh
source ./setup.sh
```

`setup.sh` 会设置 `LD_LIBRARY_PATH`，用于加载 `bimanual/api/` 下的动态库。

## 硬件设备配置

### CAN 设备

机械臂通过 CAN 通信。单路 CAN 的基础启动方式如下：

```bash
sudo slcand -o -f -s8 /dev/arxcan0 can0
sudo ifconfig can0 up
```

仓库中还提供了 CAN 配置脚本，位置在：

```text
ARX_X5/ARX_CAN/arx_can/
```

运行双臂示例前，请确保代码中的 `can1`、`can3` 等 CAN 端口已经正确映射并处于 up 状态。

### 夹爪串口和相机

夹爪 SDK 默认使用固定设备名：

- 左夹爪串口：`/dev/ttyFingerLeft`
- 右夹爪串口：`/dev/ttyFingerRight`
- 左夹爪相机：`/dev/finger_camera_left`
- 右夹爪相机：`/dev/finger_camera_right`

配置步骤参考：

```bash
cd ~/ARX/finger_sdk_python/gen_con_sdk_python_release-master
sudo cp config/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

首次使用时需要根据实际 USB 拓扑修改 `config/99-usb-serial.rules` 中的 `KERNELS==` 值。详细步骤见 `finger_sdk_python/gen_con_sdk_python_release-master/README_CN.md`。

## 快速运行

### 双臂和双夹爪联调

入口文件：

```text
ARX_X5/py/arx_x5_python/test.py
```

运行：

```bash
cd ~/ARX/ARX_X5/py/arx_x5_python
source ./setup.sh
python3 test.py
```

该脚本会：

1. 初始化左右机械臂：左臂 `can1`，右臂 `can3`。
2. 初始化左右夹爪：`/dev/ttyFingerLeft` 和 `/dev/ttyFingerRight`。
3. 双臂回零。
4. 控制夹爪张开、闭合。
5. 使用五次样条插值让双臂平滑移动到目标点、抬升并回到零位。

注意：当前 `test.py` 和 `test2.py` 中的 `FINGER_SDK_DIR` 默认写为：

```python
FINGER_SDK_DIR = "/home/arx/ARX/finger_sdk_python/gen_con_sdk_python_release-master"
```

如果你的项目路径是 `~/ARX` 且用户名不是 `arx`，请先将其改为实际路径，例如：

```python
FINGER_SDK_DIR = "/home/ubuntu/ARX/finger_sdk_python/gen_con_sdk_python_release-master"
```

### 夹爪独立测试

入口文件：

```text
ARX_X5/py/arx_x5_python/test2.py
```

运行：

```bash
cd ~/ARX/ARX_X5/py/arx_x5_python
python3 test2.py
```

该脚本只初始化左右夹爪，并依次设置夹爪距离为 `0.15`、`0.02`、`0.08` 米。

### 夹爪 SDK 独立启动

```bash
cd ~/ARX/finger_sdk_python/gen_con_sdk_python_release-master
source venv/bin/activate
python3 start_finger.py left
```

更多串口、相机、触觉、编码器、校准和协议说明见：

```text
finger_sdk_python/gen_con_sdk_python_release-master/README_CN.md
```

## ROS2 使用

机械臂 ROS2 工作空间位于：

```text
ARX_X5/ROS2/X5_ws/
```

基础编译：

```bash
cd ~/ARX/ARX_X5/ROS2/X5_ws
colcon build
source install/setup.bash
```

单臂启动示例：

```bash
ros2 launch arx_x5_controller open_single_arm.launch.py
```

仓库中也提供了一键脚本：

```bash
bash ~/ARX/ARX_X5/00-sh/ROS2/01make.sh
bash ~/ARX/ARX_X5/00-sh/ROS2/04single_arm.sh
bash ~/ARX/ARX_X5/00-sh/ROS2/03single_vr.sh
```

VR 遥操作 SDK 位于：

```text
ARX_X5/ARX_VR_SDK/
```

ROS2 VR 编译可参考：

```bash
cd ~/ARX/ARX_X5/ARX_VR_SDK/ROS2
./port.sh
colcon build
```

VR 使用 Meta Quest3 作为操作端，详细网络配置、按键映射和启动流程见 `ARX_X5/ARX_VR_SDK/readme.md`。

## MCU 固件

DAS 夹爪控制器固件位于：

```text
das_finger_controller_mcu/
```

硬件和通信概览：

- MCU：STM32F103C8T6
- 电机：LK MG4005
- 上位机通信：USART1，921600 8N1
- 控制协议：以 `das\r\n` 作为帧头和帧尾
- 电机通信：CAN1，1 Mbps
- 主循环频率：50 Hz

编译与烧录：

1. 使用 Keil µVision 5 打开 `das_finger_controller_mcu/MDK-ARM/KTH71_Example.uvprojx`。
2. 确认 Target 为 `STM32F103C8`。
3. 执行 `Project -> Build Target`。
4. 连接 ST-Link 后执行 `Flash -> Download`。

固件测试工具：

```bash
python3 ~/ARX/das_finger_controller_mcu/tools/finger_test.py --port /dev/ttyUSB0 demo
```

完整说明见 `das_finger_controller_mcu/USAGE.md`。

## 常见注意事项

- 运行机械臂前先确认 CAN 设备已经正确启动，否则 Python 或 ROS 节点无法连接机械臂。
- 联合调试过程中，偶发 ARX 左臂电机掉线现象。解决办法是杀死 can 通信进程后重启双臂 can 通信，如若没有效果，请插拔控制盒（can 6）后重新配置。
- 运行夹爪前先确认 udev 规则已经加载，并检查 `/dev/ttyFingerLeft`、`/dev/ttyFingerRight` 是否存在。
- `test.py` 和 `test2.py` 当前包含硬编码路径，换机器或换用户名后需要同步修改。
- 夹爪相机要求 USB 3.0 接口，设备名变化时需要重新配置 udev。
- 旧版文档提示不同型号机械臂的 SDK 不一定通用，使用前请确认硬件型号与 SDK 匹配。
- 尽量避免把工程放在包含中文或特殊字符的路径下，减少构建和脚本路径问题。
- 该仓库目前以硬件联调脚本和示例为主，没有统一的自动化测试入口。

## 参考文档

- `ARX_X5/py/arx_x5_python/README.md`：机械臂 Python SDK 编译与运行说明。
- `finger_sdk_python/gen_con_sdk_python_release-master/README_CN.md`：灵巧夹爪 SDK 中文说明。
- `ARX_X5/ARX_VR_SDK/readme.md`：VR 遥操作使用说明。
- `das_finger_controller_mcu/USAGE.md`：DAS 夹爪控制器 MCU 固件说明。
- `ARX_X5/旧版-readme/`：旧版手册和 PDF 文档。


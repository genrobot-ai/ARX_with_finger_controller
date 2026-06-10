# Gen Controller SDK (Python)

Python SDK for the next-generation dexterous finger gripper. Each gripper = 1 serial port (CH340) + 1 USB camera.

Pure Python implementation with no ROS dependency. Communicates with the STM32 MCU over serial, supporting gripper open/close control, torque control, encoder feedback, tactile sensing, and camera capture.

---

## Table of Contents

1. [Environment Setup](#1-environment-setup)
2. [Hardware Interface Configuration (udev)](#2-hardware-interface-configuration-udev)
3. [Quick Start](#3-quick-start)
4. [Programming API](#4-programming-api)
5. [Communication Protocol](#5-communication-protocol)
6. [Device Parameter Retrieval](#6-device-parameter-retrieval)
7. [Troubleshooting](#7-troubleshooting)
8. [Notes](#8-notes)

---

## 1. Environment Setup

### 1.1 System Dependencies

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv python3-full v4l-utils
```

> The USB interface must be **USB 3.0**.

### 1.2 Python Virtual Environment

```bash
cd gen_con_sdk_python_release-master
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

> Run `source venv/bin/activate` before using the SDK in every new terminal. You will see `(venv)` at the beginning of the command prompt when the environment is active.

### 1.3 Dependencies

| Package | Version | Purpose |
|---------|---------|---------|
| `pyserial` | >=3.5 | Serial communication |
| `opencv-python` | >=4.5.0 | Camera capture and preview |
| `numpy` | >=1.19.0 | Array operations |

---

## 2. Hardware Interface Configuration (udev)

udev rules bind USB devices to fixed symlink names (e.g. `/dev/ttyFingerLeft`), preventing device numbers from changing after each plug/unplug. **Configure once, and you never need to reconfigure for the same USB port.**

### 2.1 Device Naming Convention

| Device | Left Gripper | Right Gripper |
|--------|-------------|---------------|
| Serial | `/dev/ttyFingerLeft` | `/dev/ttyFingerRight` |
| Camera | `/dev/finger_camera_left` | `/dev/finger_camera_right` |

### 2.2 Finding USB Paths

#### Serial Path

Plug in the gripper and run:

```bash
cd /dev && ls | grep ttyUSB
udevadm info -a -n /dev/ttyUSB0 | grep -E "KERNELS|DRIVERS"
```

Find the `KERNELS` value in the group where `DRIVERS=="ch341"`, e.g. `1-1.3:1.0`.

#### Camera Path

```bash
v4l2-ctl --list-devices
```

Find the `/dev/videoN` corresponding to the gripper's built-in camera, then:

```bash
udevadm info -a -n /dev/videoN | grep KERNELS | head -1
```

Take the first `KERNELS` value, e.g. `1-1.4:1.0`.

### 2.3 Editing the Rules File

Edit `config/99-usb-serial.rules` and replace the `KERNELS==` values with the ones you found:

```bash
# Left gripper serial
SUBSYSTEM=="tty", KERNELS=="1-1.3:1.0", SYMLINK+="ttyFingerLeft", MODE="0666"

# Left gripper camera
SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", KERNELS=="1-1.4:1.0", ATTR{index}=="0", SYMLINK+="finger_camera_left", MODE="0666"

# Right gripper serial (uncomment for dual gripper)
# SUBSYSTEM=="tty", KERNELS=="1-1.1:1.0", SYMLINK+="ttyFingerRight", MODE="0666"

# Right gripper camera (uncomment for dual gripper)
# SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", KERNELS=="1-1.2:1.0", ATTR{index}=="0", SYMLINK+="finger_camera_right", MODE="0666"
```

**Dual gripper configuration**: First configure the left gripper, then unplug it, plug in the right gripper, look up its KERNELS values, fill in the right gripper lines and uncomment them.

> udev comment character is `#`, not `//`.

### 2.4 Install and Load

```bash
sudo cp config/99-usb-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2.5 Verification

```bash
ls -l /dev/ttyFingerLeft /dev/finger_camera_left
# Dual gripper:
ls -l /dev/ttyFingerRight /dev/finger_camera_right
```

Each entry should point to a `ttyUSB*` or `video*` device.

---

## 3. Quick Start

### 3.1 Command-Line Arguments

```bash
python3 start_finger.py <side> [options]
```

**Required argument**:

| Argument | Description |
|----------|-------------|
| `side` | `left` or `right`, specifies which gripper |

**Optional arguments**:

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--distance` | float | 0.05 | Fixed open/close distance (meters), range [0.0, 0.2] |
| `--sine-wave` | flag | - | Sine wave open/close mode (mutually exclusive with `--distance`) |
| `--amplitude` | float | 0.025 | Sine amplitude (meters) |
| `--center` | float | 0.05 | Sine center position (meters) |
| `--frequency` | float | 0.5 | Sine frequency (Hz) |
| `--duration` | float | 10.0 | Sine duration (seconds), 0 = run forever |
| `--torque-enable` | flag | - | Enable custom torque (use with `--torque-value`) |
| `--torque-value` | int | - | Torque value, range [100, 500] |
| `--camera-resolutions` | str | 1600x1296 | Camera resolution, format `widthxheight` |
| `--camera-fps` | int | 30 | Camera display frame rate (V4 Controller should be set to 60) |
| `--no-preview` | flag | - | Do not show camera preview windows |
| `--print-tactile-info` | flag | - | Print tactile grid in terminal |
| `--tactile-print-hz` | float | 0.0 | Max tactile print rate (Hz), 0 = unlimited |

### 3.2 Examples

```bash
# Single gripper, default open to 5cm
python3 start_finger.py left

# Fixed open to 8cm
python3 start_finger.py left --distance 0.08

# Sine wave open/close for 10 seconds
python3 start_finger.py left --sine-wave

# Custom sine parameters
python3 start_finger.py left --sine-wave --amplitude 0.03 --center 0.05 --frequency 1.0 --duration 20.0

# Enable torque control
python3 start_finger.py left --distance 0.05 --torque-enable --torque-value 250

# V4 Controller camera frame rate set to 60
python3 start_finger.py left --camera-fps 60

# Print tactile data (limited to 5Hz)
python3 start_finger.py left --print-tactile-info --tactile-print-hz 5.0

# Dual gripper (run in two separate terminals)
python3 start_finger.py left     # Terminal A
python3 start_finger.py right    # Terminal B
```

After startup:
- Camera preview windows will appear
- Terminal prints `finger distance: X.XXX m` (encoder feedback)
- Diagnostic info prints every 5 seconds: `[DIAG] sent=... recv_bytes=... parsed_pkts=... queue=...`
- Press **ESC** (in preview window) or **Ctrl+C** to exit

---

## 4. Programming API

### 4.1 GripperSystem

Main controller class responsible for initializing serial communication and camera.

```python
from scripts import GripperSystem

system = GripperSystem(
    serial_port="/dev/ttyFingerLeft",     # Serial port path, None for auto-detect
    camera_resolutions="1600x1296",       # Camera resolution
    show_preview=True,                    # Whether to show OpenCV preview
    video_devices=["/dev/finger_camera_left"],  # Camera device list
    tactile_callback=my_tactile_cb,       # Tactile callback (optional)
    encoder_callback=my_encoder_cb,       # Encoder callback (optional)
    capture_frames_callback=my_frame_cb,  # Custom frame capture callback (optional)
    camera_fps=30,                        # Camera display frame rate
)
```

| Method | Description |
|--------|-------------|
| `start() -> bool` | Start camera and serial communication (blocking, until exit) |
| `stop()` | Stop all threads, close serial port |
| `set_gripper_distance(distance: float)` | Set target open/close distance, range [0.0, 0.2] meters |

| Property | Type | Description |
|----------|------|-------------|
| `databus` | `DataBus` or `None` | Serial communication layer (available after start) |
| `camera` | `CameraCapture` or `None` | Camera capture layer |
| `running` | `bool` | Whether the system is running |

### 4.2 DataBus

Serial communication layer, manages read/write/parse threads. Usually accessed via `system.databus`.

| Method | Description |
|--------|-------------|
| `set_target_distance(distance: float)` | Set gripper target distance, range [0.0, 0.2] meters |
| `get_target_distance() -> float` | Get current target distance |
| `set_torque(enable: bool, value: int)` | Set torque; when enable=True, value range is [100, 500]; when enable=False, uses MCU default |
| `get_torque() -> (bool, int)` | Get current torque setting |
| `drive_motor(angle_degree: float)` | Directly drive motor angle |
| `disable_motor()` | Disable motor |
| `calib_encoder()` | Request encoder calibration |
| `send_camera_calib_cmd(cmd: str) -> bool` | Send camera calibration command |
| `register_tactile_callback(cb)` | Register tactile callback |
| `register_encoder_callback(cb)` | Register encoder callback |
| `register_camera_calib_callback(cb)` | Register calibration callback |
| `stop()` | Stop threads, send DisableDrive, close serial port |
| `is_opened() -> bool` | Whether serial port is successfully opened |
| `get_serial_info() -> dict` | Get serial port configuration info |

**DataBus constructor parameters** (usually created internally by GripperSystem):

```python
DataBus(
    tty_port="/dev/ttyFingerLeft",  # Serial port path
    baudrate=921600,                # Baud rate (fixed)
    timeout=0.5,                    # Read timeout
    encoder_freq=100,               # Encoder polling frequency (Hz)
    tactile_freq=None,              # Tactile polling frequency (Hz), None = no active polling
    tactile_callback=None,          # Tactile callback
    encoder_callback=None,          # Encoder callback
)
```

### 4.3 CameraCapture

Camera capture layer, usually accessed via `system.camera`.

| Method | Description |
|--------|-------------|
| `capture_frames_callback()` | Default frame capture loop (blocking) |
| `stop()` | Stop capture, release resources |

| Property | Type | Description |
|----------|------|-------------|
| `cameras` | `list[dict]` | Info dict for each camera |
| `running` | `bool` | Whether running |
| `target_fps` | `int` | Target frame rate |
| `show_preview` | `bool` | Whether preview is shown |

### 4.4 Callback Signatures

#### Encoder Callback

```python
def encoder_callback(record_data: bytes):
    """Called each time encoder data is received."""
    import struct
    distance = struct.unpack(">f", record_data)[0]  # Big-endian float32, unit: meters
    print(f"Gripper distance: {distance:.3f} m")
```

#### Tactile Callback

```python
def tactile_callback(record_data: bytes):
    """Called each time tactile data is received. record_data is 448 bytes."""
    from tactile_processing import convert_tactile_448_to_1000
    left_500, right_500 = convert_tactile_448_to_1000(record_data)
    # left_500: 500 sensor point values for the left side
    # right_500: 500 sensor point values for the right side
```

#### Camera Frame Callback

```python
def frame_callback(camera_id: int, frame, timestamp_ns: int):
    """Called for each frame. frame is an OpenCV BGR image."""
    # camera_id: camera index
    # frame: numpy.ndarray (BGR)
    # timestamp_ns: nanosecond timestamp
```

#### Custom Frame Capture Callback

```python
def capture_frames_callback(camera):
    """Replaces the default frame capture loop. camera is a CameraCapture instance."""
    while camera.running:
        for cam in camera.cameras:
            frame, ts = camera._get_latest(cam)
            if frame is not None:
                # Process frame...
                pass
        time.sleep(1.0 / camera.target_fps)
```

### 4.5 Full Code Examples

#### Custom Control Loop

```python
import time
import struct
from scripts import GripperSystem

def my_encoder_cb(record_data: bytes):
    distance = struct.unpack(">f", record_data)[0]
    print(f"Distance: {distance:.3f} m")

system = GripperSystem(
    serial_port="/dev/ttyFingerLeft",
    video_devices=["/dev/finger_camera_left"],
    encoder_callback=my_encoder_cb,
    show_preview=False,
)

# Start system in a background thread
import threading
t = threading.Thread(target=system.start, daemon=True)
t.start()

# Wait for DataBus to initialize
while system.databus is None:
    time.sleep(0.1)
time.sleep(0.5)

# Control the gripper
try:
    system.databus.set_target_distance(0.08)   # Open to 8cm
    time.sleep(3)
    system.databus.set_target_distance(0.02)   # Close to 2cm
    time.sleep(3)
    system.databus.set_target_distance(0.05)   # Back to 5cm
    time.sleep(2)
finally:
    system.stop()
```

#### Torque Control

```python
# Enable custom torque (range 100~500)
system.databus.set_torque(enable=True, value=200)

# Restore MCU default torque
system.databus.set_torque(enable=False)
```

### 4.6 Tactile Data Processing

```python
from tactile_processing import (
    convert_tactile_448_to_1000,
    set_tactile_grid_print_enabled,
    set_tactile_grid_print_max_hz,
    submit_tactile_1000_grid_print,
    print_tactile_1000_grid,
)
```

| Function | Description |
|----------|-------------|
| `convert_tactile_448_to_1000(data) -> (left_500, right_500)` | Convert 448-byte raw data to 500 sensor point values per side |
| `set_tactile_grid_print_enabled(enabled: bool)` | Enable/disable terminal grid printing |
| `set_tactile_grid_print_max_hz(hz: float)` | Limit print rate, 0 = unlimited |
| `submit_tactile_1000_grid_print(all_1000)` | Async submit grid print (non-blocking) |
| `print_tactile_1000_grid(all_1000)` | Sync print 50-row x 20-column grid |

---

## 5. Communication Protocol

The SDK communicates with the STM32 MCU over serial (921600 baud) using a custom DAS protocol.

### 5.1 Packet Structure

**Magic identifier**: `das\r\n` (5 bytes), used as packet header and footer.

#### Send Packet (CmdPack, Host -> STM32)

```
Offset    Length    Content
───────────────────────────────────
0         5B       Magic "das\r\n" (header)
5         1B       Opcode
6         1B       RecordType
7         4B       ContentLength (big-endian uint32)
11        1B       MaxDistance (reserved)
12        1B       TorqueEnable (0x00=enable, 0x01=disable)
13        2B       TorqueValue (big-endian int16, 100~500)
15        NB       RecordData (variable length)
15+N      1B       Calibration (reserved)
16+N      1B       MotorEnable (reserved)
17+N      5B       Magic "das\r\n" (footer)
```

#### Receive Packet (MessagePack, STM32 -> Host)

```
Offset    Length    Content
───────────────────────────────────
0         5B       Magic "das\r\n" (header)
5         1B       Opcode
6+        --       Record sequence (may contain multiple records):
                     1B  RecordType
                     8B  ContentLength (big-endian uint64)
                     NB  RecordData
-5        5B       Magic "das\r\n" (footer)
```

### 5.2 Opcodes

| Name | Value | Direction | Description |
|------|-------|-----------|-------------|
| ReadSingle | 0x01 | Send | Single read of tactile data |
| ReadBatch | 0x02 | Send | Batch read (encoder + tactile), 100Hz polling |
| WriteDrive | 0x03 | Send | Send motor target angle |
| Echo | 0x04 | Both | Echo test |
| CalibEncoder | 0x05 | Send | Request encoder calibration |
| DisableDrive | 0x06 | Send | Disable motor drive |

### 5.3 Record Types

| Name | Value | Data Size | Format |
|------|-------|-----------|--------|
| Tactile | 0x01 | 448 bytes | Raw tactile data (left 224 + right 224) |
| Encoder | 0x02 | 4 bytes | Big-endian float32, unit: meters (0.0~0.2) |
| Drive | 0x03 | Variable | Motor control |
| Echo | 0x04 | Variable | Echo test |

### 5.4 Protocol Constants

```python
MAGIC = b"das\r\n"       # Packet header/footer identifier
MAX_PACKET_SIZE = 4096    # Max single packet length
MAX_BUFFER_SIZE = 8192    # Max parse buffer length
```

### 5.5 Diagnostic Output

Diagnostic info is printed every 5 seconds at runtime:

```
[DIAG] sent=500 recv_bytes=12800 parsed_pkts=500 queue=0
```

| Field | Meaning |
|-------|---------|
| `sent` | Number of commands sent |
| `recv_bytes` | Number of bytes received |
| `parsed_pkts` | Number of successfully parsed packets |
| `queue` | Number of commands pending in the send queue |

`recv_bytes=0` means the STM32 is not returning any data at all. See [Troubleshooting](#7-troubleshooting).

---

## 6. Device Parameter Retrieval

### 6.1 Camera Calibration

```bash
# Single gripper
python3 scripts/camera_cmd.py camerarc     # Center camera calibration

# Dual gripper
python3 scripts/camera_cmd.py left camerarc
python3 scripts/camera_cmd.py right camerarc
```

Calibration results are saved in the `scripts/calib_result/` directory (YAML format).

> New devices have only 1 camera; `camerarl`/`camerarr` are no longer used.

### 6.2 Query MCUID

```bash
# Single gripper
python3 scripts/camera_cmd.py MCUID

# Dual gripper
python3 scripts/camera_cmd.py left MCUID
python3 scripts/camera_cmd.py right MCUID
```

| Command | Description | Output |
|---------|-------------|--------|
| `camerarc` | Center camera calibration | `cam0_sensor_*.yaml` |
| `camerarl` | Left camera calibration (deprecated) | `cam1_sensor_*.yaml` |
| `camerarr` | Right camera calibration (deprecated) | `cam2_sensor_*.yaml` |
| `MCUID` | Query device ID | Printed to terminal |

---

## 7. Troubleshooting

| Symptom | Cause | Solution |
|---------|-------|----------|
| `/dev/ttyFingerLeft` does not exist | udev rules not loaded | Re-run `sudo udevadm control --reload-rules && sudo udevadm trigger`; check that KERNELS values in `99-usb-serial.rules` are correct |
| `recv_bytes=0`, STM32 not responding | STM32 protocol parser is stuck in a misaligned state | Unplug and replug USB to power-cycle the STM32; see [Notes](#8-notes) |
| Camera fails to open: `OpenCV failed to open device` | Previous process did not release the camera | `pkill -9 -f start_finger.py`, or unplug and replug USB |
| `Permission denied` | Insufficient serial/camera permissions | `sudo chmod 666 /dev/ttyUSB* /dev/video*` (temporary); or add user to `dialout` and `video` groups |
| Starts normally but no `finger distance:` output | Gripper power / motor power issue | Check gripper hardware power supply; test with `sudo minicom -D /dev/ttyFingerLeft -b 921600` for raw data |
| `[DIAG] queue=1000` send queue full | Serial port occupied or device disconnected | Check if another process is using the serial port; check USB connection |
| `Warning: system init timed out` | DataBus initialization timed out | Check serial port path is correct and STM32 is powered on |
| `QObject::killTimer` warning | OpenCV Qt backend threading issue | Harmless, can be ignored |

---

## 8. Notes

### 8.1 STM32 Communication Loss

If `[DIAG]` logs consistently show `recv_bytes=0` (STM32 not returning data), **unplugging and replugging USB to power-cycle the STM32** is the only effective recovery method.

**Root cause**: If an in-progress serial write is forcibly interrupted during program exit (e.g. `cancel_write()`), the STM32 receives an incomplete data packet, causing its protocol parser to become permanently misaligned.

**Prevention**: The SDK's `stop()` method ensures the current packet being sent is fully written before closing the serial port. Normal exit via Ctrl+C will not cause this issue.

### 8.2 Serial Baud Rate

Fixed at **921600**, cannot be changed. Must match the STM32 firmware.

### 8.3 Encoder Polling Frequency

Default is **100Hz** (`encoder_freq=100` in `system.py`). To adjust, modify the `encoder_freq` parameter in `scripts/system.py`.

### 8.4 Gripper Distance Range

Valid range is **[0.0, 0.2]** meters (0 ~ 20 cm). Values outside this range will be rejected.

### 8.5 Torque Range

Custom torque value range is **[100, 500]**. Setting `enable=False` uses the MCU's internal default value.

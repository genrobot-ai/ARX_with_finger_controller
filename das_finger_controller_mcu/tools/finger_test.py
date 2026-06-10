#!/usr/bin/env python3
"""
DAS Finger Controller test tool.

Usage:
  python3 finger_test.py --port /dev/ttyUSB0 mcuid
  python3 finger_test.py --port /dev/ttyUSB0 calib
  python3 finger_test.py --port /dev/ttyUSB0 manual-calib
  python3 finger_test.py --port /dev/ttyUSB0 goto 50          # 50 mm
  python3 finger_test.py --port /dev/ttyUSB0 goto 50 --torque 200
  python3 finger_test.py --port /dev/ttyUSB0 disable
  python3 finger_test.py --port /dev/ttyUSB0 monitor          # 持续打印上报
  python3 finger_test.py --port /dev/ttyUSB0 demo             # 一键全流程
"""
import argparse
import struct
import sys
import time

import serial

BAUD = 921600
FRAME = b"das\r\n"


def open_port(port: str) -> serial.Serial:
    return serial.Serial(port, BAUD, timeout=0.2)


def send_frame(ser: serial.Serial, payload: bytes) -> None:
    ser.write(FRAME + payload + FRAME)
    ser.flush()


def read_for(ser: serial.Serial, seconds: float) -> bytes:
    end = time.time() + seconds
    buf = bytearray()
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
    return bytes(buf)


def split_frames(buf: bytes):
    """切分 das\r\n...das\r\n 帧，返回 payload 列表 + 剩余字节。"""
    frames = []
    i = 0
    while True:
        start = buf.find(FRAME, i)
        if start < 0:
            return frames, buf[i:]
        end = buf.find(FRAME, start + len(FRAME))
        if end < 0:
            return frames, buf[start:]
        frames.append(buf[start + len(FRAME) : end])
        i = end + len(FRAME)


def cmd_mcuid(ser: serial.Serial) -> None:
    print("[TX] MCUID")
    ser.write(b"MCUID")
    data = read_for(ser, 0.5)
    frames, _ = split_frames(data)
    for p in frames:
        if len(p) == 24:
            print(f"[RX] UID = {p.decode('ascii', errors='replace')}")
            return
    print(f"[RX] {data.hex(' ') if data else '<no response>'}")
    sys.exit(1)


def cmd_manual_calib(ser: serial.Serial) -> None:
    print("[TX] 1234  (手动校准，电机会扫到限位，约 5-10s)")
    ser.write(b"1234")
    print("[..] 等待校准完成，串口会输出 'Calibration OK !'")
    end = time.time() + 30
    seen = bytearray()
    while time.time() < end:
        seen.extend(ser.read(4096))
        if b"Calibration OK" in seen:
            print("[RX] 校准成功")
            return
    print("[!!] 30s 内未看到完成信息，检查 CAN/电机供电")


def cmd_calib(ser: serial.Serial) -> None:
    print("[TX] CALIB  (自动校准检查，不写 Flash)")
    ser.write(b"CALIB")
    end = time.time() + 30
    seen = bytearray()
    while time.time() < end:
        seen.extend(ser.read(4096))
        if b"CHECK_" in seen:
            start = seen.find(b"CHECK_")
            line = bytes(seen[start : start + 80]).split(b"das")[0]
            print(f"[RX] {line.decode('ascii', errors='replace')}")
            return
    print("[!!] 30s 内未收到 CHECK_ 应答")


def build_control(target_mm: float, enable: bool, torque: int | None) -> bytes:
    payload = bytearray(16)
    payload[0] = 0x02
    payload[2:6] = struct.pack(">f", target_mm / 1000.0)  # big-endian float, meters
    payload[6] = 0x00  # spacing_select=0 → DISTANCE_MAX=0.103
    if torque is None:
        payload[7] = 0x00  # 默认 150
    else:
        payload[7] = 0x01
        torque = max(100, min(500, torque))
        payload[8:10] = struct.pack("<h", torque)  # 小端 int16
    payload[14] = 0x00
    payload[15] = 0x01 if enable else 0x00
    return bytes(payload)


def cmd_goto(ser: serial.Serial, target_mm: float, torque: int | None) -> None:
    print(f"[TX] goto {target_mm:.1f} mm  enable=1  torque={torque or 'default(150)'}")
    send_frame(ser, build_control(target_mm, enable=True, torque=torque))
    time.sleep(0.5)
    show_feedback(ser, samples=10)


def cmd_disable(ser: serial.Serial) -> None:
    print("[TX] disable")
    send_frame(ser, build_control(0.0, enable=False, torque=None))


def parse_drive(payload: bytes):
    """TxBuffer_drive: 14 字节, [10..13] = float 距离 (mm)，LE。"""
    if len(payload) != 14 or payload[0] != 0x01 or payload[1] != 0x02:
        return None
    (distance_mm,) = struct.unpack("<f", payload[10:14])
    return distance_mm


def show_feedback(ser: serial.Serial, samples: int = 10) -> None:
    data = read_for(ser, samples * 0.025)  # 50Hz → 20ms 一帧
    frames, _ = split_frames(data)
    shown = 0
    for p in frames:
        d = parse_drive(p)
        if d is not None:
            print(f"[RX] distance = {d:7.3f} mm")
            shown += 1
            if shown >= samples:
                break
    if shown == 0:
        print(f"[!!] 没解析到 drive 包；原始 {len(data)} 字节，帧数 {len(frames)}")


def cmd_monitor(ser: serial.Serial) -> None:
    print("[..] 持续监听 (Ctrl-C 退出)")
    buf = bytearray()
    try:
        while True:
            buf.extend(ser.read(4096))
            frames, rest = split_frames(bytes(buf))
            buf = bytearray(rest)
            for p in frames:
                d = parse_drive(p)
                if d is not None:
                    print(f"distance = {d:7.3f} mm")
    except KeyboardInterrupt:
        print()


def cmd_demo(ser: serial.Serial) -> None:
    print("=== 全流程演示 ===")
    cmd_mcuid(ser)
    print()
    cmd_calib(ser)
    print()
    for pos in [80, 40, 10, 60]:
        cmd_goto(ser, pos, torque=None)
        time.sleep(1.0)
    cmd_disable(ser)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="串口设备，如 /dev/ttyUSB0")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("mcuid")
    sub.add_parser("calib")
    sub.add_parser("manual-calib")
    g = sub.add_parser("goto")
    g.add_argument("mm", type=float, help="目标开口 (mm)，0..103")
    g.add_argument("--torque", type=int, default=None, help="100..500，默认 150")
    sub.add_parser("disable")
    sub.add_parser("monitor")
    sub.add_parser("demo")
    args = ap.parse_args()

    with open_port(args.port) as ser:
        ser.reset_input_buffer()
        if args.cmd == "mcuid":
            cmd_mcuid(ser)
        elif args.cmd == "calib":
            cmd_calib(ser)
        elif args.cmd == "manual-calib":
            cmd_manual_calib(ser)
        elif args.cmd == "goto":
            cmd_goto(ser, args.mm, args.torque)
        elif args.cmd == "disable":
            cmd_disable(ser)
        elif args.cmd == "monitor":
            cmd_monitor(ser)
        elif args.cmd == "demo":
            cmd_demo(ser)


if __name__ == "__main__":
    main()

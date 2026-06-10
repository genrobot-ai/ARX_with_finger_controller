#!/usr/bin/env python3
"""
DataBus - Pure Python implementation for gripper communication
Removed ROS dependencies, using callbacks instead of ROS topics
"""

import serial
import serial.tools.list_ports
import threading
import time
import logging
import queue
import traceback
import struct
import os
import subprocess
from typing import Callable, Optional
from .pack import CmdPack, MessagePack, Opcode, RecordType
from .das_protocol import DASProtocol


# Default callbacks live in user scripts (gripper_controller.py, start_finger.py, camera_cmd.py)
# so they stay editable even if databus.py is shipped obfuscated.


class DataBus:
    def __init__(
        self,
        tty_port="/dev/ttyUSB0",
        baudrate=921600,
        timeout=0.5,
        is_calib_cmd=False,
        encoder_freq: float = None,
        tactile_freq: float = None,
        tactile_callback: Optional[Callable] = None,
        encoder_callback: Optional[Callable] = None,
        camera_calib_callback: Optional[Callable] = None,
    ):
        """
        Initialize DataBus.

        Args:
            tty_port: Serial device path.
            baudrate: Baud rate.
            timeout: Read timeout (seconds).
            is_calib_cmd: Calibration command mode.
            encoder_freq: Encoder poll rate (Hz).
            tactile_freq: Tactile poll rate (Hz).
            tactile_callback: Tactile record handler.
            encoder_callback: Encoder record handler.
            camera_calib_callback: Camera calibration handler.
        """
        self.tty_port = tty_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.is_running = False

        self._open_serial_success = False
        self.protocol: DASProtocol = DASProtocol()
        self.data_buffer: bytes = b""
        self.data_buffer_lock = threading.Lock()
        self.serial_lock = threading.Lock()

        self.cmd_queue = queue.Queue(1000)

        # 诊断计数器
        self._diag_send_count = 0
        self._diag_recv_bytes = 0
        self._diag_parse_count = 0
        self._diag_last_print = time.time()
        # watchdog 状态
        self._watchdog_last_recv = 0
        self._watchdog_stall_start = None
        self._watchdog_recovery_count = 0
        self._WATCHDOG_STALL_THRESHOLD = 10.0  # 连续 10 秒无新数据则触发恢复
        self._WATCHDOG_MAX_RECOVERIES = 3      # 最多自动恢复 3 次，之后只报警

        self.read_thread: threading.Thread = None
        self.parse_thread: threading.Thread = None
        self.send_thread: threading.Thread = None

        self.encoder_freq = encoder_freq
        self.tactile_freq = tactile_freq
        self.encoder_thread: threading.Thread = None
        self.tactile_thread: threading.Thread = None
        
        self.gripper_dis = 0.0
        self.torque_enable = False
        self.torque_value = 0
        self.angle_lock = threading.Lock()
        self.is_calib_cmd = is_calib_cmd
        
        self.tactile_callback = tactile_callback
        self.encoder_callback = encoder_callback
        self.camera_calib_callback = camera_calib_callback

        self._open_serial()
        if not self._open_serial_success:
            raise RuntimeError(f"Failed to open serial port: {tty_port}")

        self._flush_stale_rx_state()

        self.is_running = True
        self._start_reading()
        self._start_parsing()
        self._start_sending()

        if not self._startup_health_check():
            print("[WARN] STM32 did not respond to startup probe, continuing anyway")

        if self.encoder_freq:
            self._start_encoder_loop()
        if self.tactile_freq:
            self._start_tactile_loop()

    def set_target_distance(self, distance: float):
        """
        Set target gripper opening (encoder setpoint).

        Args:
            distance: Meters in [0.0, 0.2] (~20 cm max).
        """
        if distance < 0.0 or distance > 0.2:
            raise ValueError(f"Distance must be in [0.0, 0.2], got: {distance}")
        
        with self.angle_lock:
            self.gripper_dis = distance

    def get_target_distance(self) -> float:
        """Current target distance."""
        with self.angle_lock:
            return self.gripper_dis

    def set_torque(self, enable: bool, value: int = 0):
        """
        Set torque control parameters.

        Args:
            enable: True to use custom torque value, False to let MCU use its own default.
            value: Torque value in range [100, 500], only meaningful when enable=True.
        """
        if enable and (value < 100 or value > 500):
            raise ValueError(f"Torque value must be in [100, 500], got: {value}")
        with self.angle_lock:
            self.torque_enable = enable
            self.torque_value = value

    def get_torque(self) -> tuple:
        """Return (torque_enable, torque_value)."""
        with self.angle_lock:
            return self.torque_enable, self.torque_value

    def drive_motor(self, angle_dgree: float):
        """Send drive command."""
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.WriteDrive,
                record_type=RecordType.Drive,
                record=struct.pack(">f", angle_dgree),
            )
        )

    def disable_motor(self):
        """Disable motor drive."""
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.DisableDrive,
                record_type=RecordType.Drive,
            )
        )
    
    def calib_encoder(self):
        """Request encoder calibration."""
        self.add_cmd(
            CmdPack.pack(
                opcode=Opcode.CalibEncoder,
                record_type=RecordType.Drive,
            )
        )

    def send_camera_calib_cmd(self, camera_cmd: str):
        """Enqueue camera calibration command string."""
        try:
            self.is_calib_cmd = True
            cmd = CmdPack.pack_calib(
                record=camera_cmd.encode('utf-8')
            )
            success = self.add_cmd(cmd)
            if success:
                print(f"Sent camera calibration command: {camera_cmd}")
            else:
                print(f"Failed to queue camera calibration: {camera_cmd}")
            return success
        except Exception as e:
            print(f"Error sending camera calibration command: {e}")
            return False

    def add_cmd(self, cmd: CmdPack) -> bool:
        """Push command to send queue."""
        try:
            self.cmd_queue.put(cmd, block=True, timeout=1)
            return True
        except queue.Full:
            print("Command queue full; drop")
            return False

    def is_opened(self):
        """True if serial opened successfully."""
        return self._open_serial_success

    def register_tactile_callback(self, callback: Callable):
        self.tactile_callback = callback

    def register_encoder_callback(self, callback: Callable):
        self.encoder_callback = callback

    def register_camera_calib_callback(self, callback: Callable):
        self.camera_calib_callback = callback

    def _flush_stale_rx_state(self):
        """Send footer magic to close any half-received packet stuck in STM32 parser."""
        try:
            magic = b"das\r\n"
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    self.ser.reset_input_buffer()
                    for _ in range(10):
                        self.ser.write(magic)
                    self.ser.flush()
                    time.sleep(0.05)
                    self.ser.reset_input_buffer()
            print("[INIT] Flushed STM32 RX parser state")
        except Exception as e:
            print(f"[INIT] Flush failed: {e}")

    def _reset_stm32_via_dtr(self):
        """Toggle DTR/RTS to reset STM32 through USB-serial adapter."""
        try:
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.1)
            self.ser.dtr = True
            self.ser.rts = True
            time.sleep(0.1)
            self.ser.dtr = False
            self.ser.rts = False
            time.sleep(0.05)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
        except Exception as e:
            print(f"DTR/RTS toggle failed: {e}")

    def _serial_break_and_resync(self):
        """Send BREAK signal + flood magic bytes to re-sync STM32 UART parser."""
        try:
            with self.serial_lock:
                if not (self.ser and self.ser.is_open):
                    return
                # BREAK: hold TX low for 250ms — many UART implementations
                # detect this and reset their receive state machine
                print("[RECOVERY] Sending serial BREAK (250ms)...")
                self.ser.send_break(duration=0.25)
                time.sleep(0.1)
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()

                # Flood protocol magic to re-sync packet parser on STM32 side.
                # If STM32 is stuck mid-packet, these headers help it find a
                # new packet boundary.
                print("[RECOVERY] Flooding magic bytes to re-sync protocol...")
                magic = b"das\r\n"
                for _ in range(20):
                    self.ser.write(magic)
                self.ser.flush()
                time.sleep(0.1)
                self.ser.reset_input_buffer()

            with self.data_buffer_lock:
                self.data_buffer = b""
        except Exception as e:
            print(f"[RECOVERY] Serial break/resync error: {e}")

    def _send_probe_and_wait(self, timeout=2.0):
        """Send a ReadBatch probe command and wait for any response bytes."""
        self._diag_recv_bytes = 0
        probe_cmd = CmdPack.pack(
            opcode=Opcode.ReadBatch,
            record_type=RecordType.Encoder,
            record=struct.pack(">f", 0.0),
        )
        self.add_cmd(probe_cmd)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._diag_recv_bytes > 0:
                return True
            time.sleep(0.05)
        return False

    def _find_usb_sysfs_path(self):
        """Walk sysfs to find the USB device (not interface) for this serial port."""
        try:
            real_port = os.path.realpath(self.tty_port)
            tty_name = os.path.basename(real_port)
            sysfs_device = f"/sys/class/tty/{tty_name}/device"
            if not os.path.exists(sysfs_device):
                return None
            device_path = os.path.realpath(sysfs_device)
            while device_path and device_path != "/":
                auth_file = os.path.join(device_path, "authorized")
                id_file = os.path.join(device_path, "idVendor")
                if os.path.exists(auth_file) and os.path.exists(id_file):
                    return device_path
                device_path = os.path.dirname(device_path)
        except Exception:
            pass
        return None

    def _usb_power_cycle(self):
        """Software USB replug: deauthorize then reauthorize the USB device via sysfs."""
        usb_path = self._find_usb_sysfs_path()
        if not usb_path:
            print("[USB-RESET] Cannot locate USB sysfs path, skipping")
            return False

        helper_script = os.path.join(os.path.dirname(__file__), "usb_power_cycle.sh")
        if not os.path.exists(helper_script):
            print(f"[USB-RESET] Helper script not found: {helper_script}")
            return False

        print(f"[USB-RESET] Power-cycling USB device: {usb_path}")

        with self.serial_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
                self._open_serial_success = False

        try:
            subprocess.run(
                ["sudo", "-n", helper_script, usb_path],
                check=True, timeout=10,
            )
        except subprocess.CalledProcessError as e:
            print(f"[USB-RESET] Helper script failed (exit {e.returncode})")
            return False
        except subprocess.TimeoutExpired:
            print("[USB-RESET] Helper script timed out")
            return False
        except Exception as e:
            print(f"[USB-RESET] Error: {e}")
            return False

        for _ in range(50):
            if os.path.exists(self.tty_port):
                break
            time.sleep(0.1)
        else:
            print(f"[USB-RESET] {self.tty_port} did not reappear within 5s")
            return False

        time.sleep(1.0)
        with self.serial_lock:
            self._open_serial()
        if not self._open_serial_success:
            print("[USB-RESET] Failed to reopen serial after USB reset")
            return False

        print("[USB-RESET] USB power cycle complete, serial reopened")
        with self.data_buffer_lock:
            self.data_buffer = b""
        return True

    def _startup_health_check(self):
        """
        Multi-strategy startup probe.
        Strategy 1: direct probe (maybe STM32 is fine)
        Strategy 2: serial BREAK + magic resync
        Strategy 3: USB power cycle
        """
        # Strategy 1: just send a probe
        print("[HEALTH] Probing STM32...")
        if self._send_probe_and_wait(timeout=1.5):
            print("[HEALTH] STM32 responded immediately")
            return True

        # Strategy 2: serial BREAK + protocol resync
        print("[HEALTH] No response. Trying serial BREAK + protocol resync...")
        self._serial_break_and_resync()
        time.sleep(0.3)
        if self._send_probe_and_wait(timeout=2.0):
            print("[HEALTH] STM32 responded after serial BREAK resync!")
            return True

        # Strategy 3: USB device reset
        print("[HEALTH] Still no response. Trying USB power cycle...")
        if self._usb_power_cycle():
            time.sleep(0.5)
            if self._send_probe_and_wait(timeout=3.0):
                print("[HEALTH] STM32 responded after USB power cycle!")
                return True
            print("[HEALTH] Still no response after USB power cycle")

        return False

    def _watchdog_check(self, now):
        """Called every 5s from encoder loop. Detect recv stall and attempt recovery."""
        current_recv = self._diag_recv_bytes
        if current_recv > self._watchdog_last_recv:
            self._watchdog_last_recv = current_recv
            self._watchdog_stall_start = None
            return

        if self._watchdog_stall_start is None:
            self._watchdog_stall_start = now
            return

        stall_duration = now - self._watchdog_stall_start
        if stall_duration < self._WATCHDOG_STALL_THRESHOLD:
            return

        if self._watchdog_recovery_count >= self._WATCHDOG_MAX_RECOVERIES:
            if not getattr(self, '_watchdog_exhausted_printed', False):
                print(f"[WATCHDOG] Auto-recovery exhausted ({self._WATCHDOG_MAX_RECOVERIES} attempts). "
                      f"Please replug USB to power-cycle STM32.")
                self._watchdog_exhausted_printed = True
            return

        self._watchdog_recovery_count += 1
        print(f"[WATCHDOG] No data for {stall_duration:.0f}s, "
              f"attempting recovery ({self._watchdog_recovery_count}/{self._WATCHDOG_MAX_RECOVERIES})...")
        self._attempt_recovery()
        self._watchdog_stall_start = None

    def _attempt_recovery(self):
        """Try serial BREAK + resync first, then USB power cycle as fallback."""
        print("[WATCHDOG] Trying serial BREAK + protocol resync...")
        self._serial_break_and_resync()
        time.sleep(0.5)
        if self._diag_recv_bytes > self._watchdog_last_recv:
            print("[WATCHDOG] STM32 recovered after serial BREAK!")
            return

        print("[WATCHDOG] BREAK ineffective, trying USB power cycle...")
        if self._usb_power_cycle():
            print("[WATCHDOG] USB power cycle done")
        else:
            print("[WATCHDOG] USB power cycle failed, manual replug may be needed")

    def _open_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.tty_port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=1.0,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
            )

            if self.ser.is_open:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                print(f"Serial opened: {self.tty_port}, baudrate: {self.baudrate}")
                self._open_serial_success = True
            else:
                print(f"Serial open failed: {self.tty_port}")
                self._open_serial_success = False
        except Exception as e:
            print(f"Serial open error: {e}")
            self._open_serial_success = False

    def _start_reading(self):
        self.read_thread = threading.Thread(target=self._reading_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        print("Read thread started")
        return True

    def _start_parsing(self):
        self.parse_thread = threading.Thread(target=self._parsing_loop)
        self.parse_thread.daemon = True
        self.parse_thread.start()
        print("Parse thread started")
        return True

    def _start_encoder_loop(self):
        self.encoder_thread = threading.Thread(target=self._send_encoder_loop)
        self.encoder_thread.daemon = True
        self.encoder_thread.start()
        print("Encoder loop thread started")
        return True

    def _start_tactile_loop(self):
        self.tactile_thread = threading.Thread(target=self._send_tactile_loop)
        self.tactile_thread.daemon = True
        self.tactile_thread.start()
        print("Tactile loop thread started")
        return True

    def _start_sending(self):
        self.send_thread = threading.Thread(target=self._sending_loop)
        self.send_thread.daemon = True
        self.send_thread.start()
        print("Send thread started")
        return True

    def _sending_loop(self):
        while self.is_running:
            try:
                cmd: CmdPack = self.cmd_queue.get(block=True, timeout=0.1)
                with self.serial_lock:
                    if self.ser and self.ser.is_open:
                        self.ser.write(cmd.data)
                        self.ser.flush()
                        self._diag_send_count += 1

            except queue.Empty:
                continue
            except serial.SerialTimeoutException:
                print("Serial write timeout, retrying...")
                continue
            except Exception as e:
                if not self.is_running:
                    break
                print(f"Send error: {e}")
                time.sleep(0.01)

    def _reading_loop(self):
        while self.is_running:
            try:
                with self.serial_lock:
                    if self.ser and self.ser.is_open:
                        n = self.ser.inWaiting()
                        if n:
                            read_size = min(n, 16384)
                            data = self.ser.read(read_size)
                            if data:
                                self._diag_recv_bytes += len(data)
                                with self.data_buffer_lock:
                                    self.data_buffer = self.data_buffer + data
                                if n > read_size:
                                    continue

            except Exception as e:
                print(f"Read loop error: {e}")
                time.sleep(0.1)

            time.sleep(0.001)

    def _parsing_loop(self):
        while self.is_running:
            packets_to_process = []

            with self.data_buffer_lock:
                if len(self.data_buffer) > 0:
                    packets, remain = DASProtocol.find_packet(self.data_buffer)
                    self.data_buffer = remain
                    packets_to_process = packets.copy()

            for packet in packets_to_process:
                self._diag_parse_count += 1
                try:
                    if self.is_calib_cmd:
                        camera_pack = MessagePack.unpack_camera_calib(packet)
                        
                        if camera_pack:
                            if self.camera_calib_callback:
                                self.camera_calib_callback(camera_pack)
                            self.is_calib_cmd = False
                    else:
                        pack = MessagePack.unpack(packet)
                        if not pack:
                            continue

                        for record in pack.records_:
                            try:
                                if record.record_type == RecordType.Tactile:
                                    if self.tactile_callback:
                                        self.tactile_callback(record.record_data)
                                elif record.record_type == RecordType.Encoder:
                                    if self.encoder_callback:
                                        self.encoder_callback(record.record_data)
                                elif record.record_type == RecordType.Echo:
                                    pass
                                else:
                                    logging.error(
                                        "record type:{} invalid !".format(record.record_type)
                                    )
                            except Exception as e:
                                logging.error(f"Callback error: {e}")
                                
                except Exception as e:
                    logging.error(f"Packet handling error: {e}")

            if packets_to_process:
                time.sleep(0.001)
            else:
                time.sleep(0.005)

    def _send_encoder_loop(self):
        if not self.encoder_freq:
            return
            
        interval = 1.0 / self.encoder_freq
        print(f"Encoder loop running at {self.encoder_freq} Hz, interval {interval:.3f}s")
        
        while self.is_running:
            start_time = time.time()
            
            with self.angle_lock:
                dis_target = self.gripper_dis
                t_enable = self.torque_enable
                t_value = self.torque_value

            self.add_cmd(
                CmdPack.pack(
                    opcode=Opcode.ReadBatch,
                    record_type=RecordType.Encoder,
                    record=struct.pack(">f", dis_target),
                    torque_enable=t_enable,
                    torque_value=t_value,
                ),
            )

            # 每 5 秒打印一次诊断 + watchdog 检查
            now = time.time()
            if now - self._diag_last_print >= 5.0:
                print(f"[DIAG] sent={self._diag_send_count} recv_bytes={self._diag_recv_bytes} parsed_pkts={self._diag_parse_count} queue={self.cmd_queue.qsize()}")
                self._diag_last_print = now
                self._watchdog_check(now)

            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("Encoder loop thread exiting")

    def _send_tactile_loop(self):
        if not self.tactile_freq:
            return
            
        interval = 1.0 / self.tactile_freq
        print(f"Tactile loop running at {self.tactile_freq} Hz, interval {interval:.3f}s")
        
        while self.is_running:
            start_time = time.time()
            self.add_cmd(
                CmdPack.pack(opcode=Opcode.ReadSingle, record_type=RecordType.Tactile, record=struct.pack(">f", 0.0))
            )
            
            elapsed = time.time() - start_time
            sleep_time = max(0, interval - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        print("Tactile loop thread exiting")

    def stop(self):
        """Stop worker threads and close serial."""
        print("Stopping all threads...")
        self.is_running = False

        # 不要调用 cancel_write()！
        # cancel_write() 会中断正在进行的 serial.write()，导致 STM32 收到
        # 不完整的数据包，使其协议解析器进入死锁状态（表现为下次启动
        # recv_bytes=0，需要拔插 USB 恢复）。
        # write_timeout=1.0 已经保证 write 不会永远阻塞。

        threads_to_join = []
        if self.read_thread and self.read_thread.is_alive():
            threads_to_join.append(self.read_thread)
        if self.send_thread and self.send_thread.is_alive():
            threads_to_join.append(self.send_thread)
        if self.parse_thread and self.parse_thread.is_alive():
            threads_to_join.append(self.parse_thread)
        if self.encoder_thread and self.encoder_thread.is_alive():
            threads_to_join.append(self.encoder_thread)
        if self.tactile_thread and self.tactile_thread.is_alive():
            threads_to_join.append(self.tactile_thread)

        for thread in threads_to_join:
            thread.join(timeout=2)

        with self.serial_lock:
            if self.ser and self.ser.is_open:
                try:
                    disable_cmd = CmdPack.pack(
                        opcode=Opcode.DisableDrive,
                        record_type=RecordType.Drive,
                    )
                    self.ser.write(disable_cmd.data)
                    self.ser.flush()
                except Exception:
                    pass
                self.ser.close()

    def get_serial_info(self):
        if self.ser and self.ser.is_open:
            info = {
                "tty_port": self.tty_port,
                "baudrate": self.ser.baudrate,
                "bytesize": self.ser.bytesize,
                "parity": self.ser.parity,
                "stopbits": self.ser.stopbits,
                "timeout": self.ser.timeout,
                "in_waiting": self.ser.in_waiting,
            }
            return info
        return None


def check_and_fix_permission(port):
    """Ensure current user can read/write the serial node."""
    if not os.path.exists(port):
        return False
    
    if os.access(port, os.R_OK | os.W_OK):
        return True
    
    print(f"Trying to fix permissions on {port}...")
    try:
        subprocess.run(['sudo', 'chmod', '666', port], check=True)
        print(f"Permissions fixed: {port}")
        return True
    except subprocess.CalledProcessError:
        print(f"Permission fix failed; run manually: sudo chmod 666 {port}")
        return False


def find_configured_serial_port():
    """
    Find configured USB serial symlinks under /dev/ttyDevice*.

    Returns:
        First accessible port path, or None.
    """
    import glob
    configured_ports = glob.glob('/dev/ttyDevice*')
    
    if not configured_ports:
        return None
    
    for port in sorted(configured_ports):
        if os.path.exists(port) and check_and_fix_permission(port):
            return port
    
    return sorted(configured_ports)[0] if configured_ports else None


def find_serial_port(pattern="ttyUSB", max_retries=3, retry_interval=2):
    """
    Prefer /dev/ttyDevice* symlinks from udev rules (raw ttyUSB is not used).

    Args:
        pattern: Deprecated, kept for API compatibility.
        max_retries: Deprecated.
        retry_interval: Deprecated.

    Returns:
        Port path or None (prints setup hints).
    """
    configured_port = find_configured_serial_port()
    
    if configured_port:
        print(f"Using configured serial device: {configured_port}")
        return configured_port
    
    print("\n" + "=" * 60)
    print(" No configured USB serial symlink found")
    print("=" * 60)
    print("\nSetup:")
    print("1. See repository docs (e.g. README_En.md) for udev examples")
    print("2. Create a rules file (e.g. 99-usb-serial.rules)")
    print("3. Copy to /etc/udev/rules.d/")
    print("4. Reload:")
    print("   sudo udevadm control --reload-rules")
    print("   sudo udevadm trigger")
    print("\nYou should then see /dev/ttyFinger* symlinks")
    print("e.g. /dev/ttyFingerLeft or your custom names")
    print("=" * 60 + "\n")
    return None

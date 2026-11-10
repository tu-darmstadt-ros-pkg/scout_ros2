#!/usr/bin/env python3
"""Passively reads the AgileX Scout battery state from CAN and publishes it as a diagnostic status.

Read-only on the socketcan interface, so it runs alongside scout_base or without it (sensor-only configs).
AgileX protocol v2, big endian:
- 0x361 BMS basic (Scout 2.0): SOC [%], SOH [%], voltage [0.1 V], current [0.1 A], temp [0.1 degC]
- 0x211 system state (all bases, e.g. Scout Mini without BMS): vehicle state, control mode, voltage [0.1 V], ...
The level is taken from the SOC if BMS frames arrive, otherwise from the system state voltage.
"""

import socket
import struct
import threading

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

SYSTEM_STATE_ID = 0x211
BMS_BASIC_ID = 0x361
CAN_FRAME_FMT = "=IB3x8s"


class ScoutBatteryMonitor(Node):
    def __init__(self):
        super().__init__("scout_battery_monitor")
        self.interface = self.declare_parameter("can_interface", "can0").value
        self.soc_warn = self.declare_parameter("soc_warn", 30).value
        self.soc_error = self.declare_parameter("soc_error", 15).value
        # Fallback for bases without BMS frames (24 V pack)
        self.voltage_warn = self.declare_parameter("voltage_warn", 24.0).value
        self.voltage_error = self.declare_parameter("voltage_error", 23.0).value
        self.timeout = self.declare_parameter("timeout", 3.0).value

        self.lock = threading.Lock()
        self.bms = None  # (soc, soh, voltage, current, temperature)
        self.bms_rx = None
        self.system_voltage = None
        self.system_rx = None

        self.pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0, self.publish)
        threading.Thread(target=self.read_can, daemon=True).start()

    def read_can(self):
        while rclpy.ok():
            try:
                sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            except OSError as e:
                self.get_logger().error(f"Cannot create CAN socket: {e}")
                return
            try:
                sock.setsockopt(
                    socket.SOL_CAN_RAW,
                    socket.CAN_RAW_FILTER,
                    struct.pack(
                        "=IIII",
                        BMS_BASIC_ID,
                        socket.CAN_SFF_MASK,
                        SYSTEM_STATE_ID,
                        socket.CAN_SFF_MASK,
                    ),
                )
                sock.bind((self.interface,))
                sock.settimeout(1.0)
                while rclpy.ok():
                    try:
                        frame = sock.recv(16)
                    except socket.timeout:
                        continue
                    self.handle_frame(*struct.unpack(CAN_FRAME_FMT, frame))
            except OSError as e:
                self.get_logger().warn(
                    f"CAN {self.interface}: {e}, retrying", throttle_duration_sec=30.0
                )
                sock.close()
                threading.Event().wait(2.0)

    def handle_frame(self, can_id, dlc, data):
        can_id &= socket.CAN_SFF_MASK
        if dlc < 8:
            return
        now = self.get_clock().now()
        if can_id == BMS_BASIC_ID:
            soc, soh, voltage, current, temperature = struct.unpack(">BBhhh", data)
            with self.lock:
                self.bms = (
                    soc,
                    soh,
                    voltage * 0.1,
                    current * 0.1,
                    temperature * 0.1,
                )
                self.bms_rx = now
        elif can_id == SYSTEM_STATE_ID:
            (voltage,) = struct.unpack(">h", data[2:4])
            with self.lock:
                self.system_voltage = voltage * 0.1
                self.system_rx = now

    def fresh(self, stamp, now):
        return stamp is not None and (now - stamp).nanoseconds * 1e-9 <= self.timeout

    def publish(self):
        status = DiagnosticStatus(
            name="scout_battery: Base battery",
            hardware_id=f"scout_base@{self.interface}",
        )
        now = self.get_clock().now()
        with self.lock:
            bms = self.bms if self.fresh(self.bms_rx, now) else None
            voltage = (
                self.system_voltage if self.fresh(self.system_rx, now) else None
            )
        if bms is not None:
            soc, soh, bms_voltage, current, temperature = bms
            status.values = [
                KeyValue(key="SOC [%]", value=str(soc)),
                KeyValue(key="SOH [%]", value=str(soh)),
                KeyValue(key="Voltage [V]", value=f"{bms_voltage:.1f}"),
                KeyValue(key="Current [A]", value=f"{current:.1f}"),
                KeyValue(key="Temperature [C]", value=f"{temperature:.1f}"),
            ]
            status.message = f"{soc}% ({bms_voltage:.1f} V)"
            if soc <= self.soc_error:
                status.level = DiagnosticStatus.ERROR
            elif soc <= self.soc_warn:
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.OK
        elif voltage is not None:
            status.values = [
                KeyValue(key="Voltage [V]", value=f"{voltage:.1f}"),
                KeyValue(key="Source", value="system state (no BMS)"),
            ]
            status.message = f"{voltage:.1f} V"
            if voltage <= self.voltage_error:
                status.level = DiagnosticStatus.ERROR
            elif voltage <= self.voltage_warn:
                status.level = DiagnosticStatus.WARN
            else:
                status.level = DiagnosticStatus.OK
        else:
            status.level = DiagnosticStatus.STALE
            status.message = f"No battery frames on {self.interface}"
        msg = DiagnosticArray(status=[status])
        msg.header.stamp = now.to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ScoutBatteryMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

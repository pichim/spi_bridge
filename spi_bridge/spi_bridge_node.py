#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Raspberry Pi 5 (Ubuntu)  ⇄  Nucleo F446RE (Mbed CE)
# ROS 2 Jazzy rclpy node that wraps your original SPI loop with minimal changes.
#
# Keeps your original constants, CRC table, verify function, buffers,
# and the double-transfer timing model. Adds ROS2 pubs/subs, params,
# logging control, and safer error handling.

import os
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Imu, MagneticField
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

try:
    import spidev
except Exception:
    spidev = None

# ---------------------------------------------------------------------------
# DEV “DEFINE” (toggle-able): per-cycle log enabled by default for now.
# You can set this to False later to reduce log spam.
# A runtime param `verbose_cycle_log` also controls this at runtime.
# ---------------------------------------------------------------------------
VERBOSE_CYCLE_LOG = True  # you can set to False later to reduce log spam

# ------------------ protocol constants (UNCHANGED from your script) ---------
SPI_HEADER_MASTER = 0x55  # Raspberry Pi header: PUBLISH (second transfer)
SPI_HEADER_MASTER_ARM = 0x56  # Raspberry Pi header: ARM-ONLY (first transfer)
SPI_HEADER_SLAVE = 0x45  # Nucleo header

SPI_NUM_FLOATS = 120  # Number of float values in each message
SPI_MSG_SIZE = 1 + SPI_NUM_FLOATS * 4 + 1  # header + floats + checksum

# Main task period (like the C++ example)
main_task_period_us_default = 10000

# always double-transfer
ARM_GAP_US_DEFAULT = 100  # small gap so the slave can re-arm/build fresh TX

# ============================ CRC-8 (poly 0x07) ==============================
CRC8_TAB = [
    0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
    0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
    0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
    0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
    0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
    0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
    0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
    0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
    0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
    0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
    0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
    0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
    0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
    0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
    0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
    0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
]


def calculate_crc8(buffer):
    """CRC-8 with polynomial 0x07 (table-based, fast)."""
    crc = 0x00
    for b in buffer:
        crc = CRC8_TAB[crc ^ b]
    return crc


def verify_checksum_seq(seq):
    """
    Verify CRC over all bytes except last, compared to last byte.
    Works with list/bytes/bytearray (same as original).
    """
    crc = 0x00
    last = len(seq) - 1
    for i in range(last):
        crc = CRC8_TAB[crc ^ seq[i]]
    return crc == seq[last]


class SpiData:
    """Data structure for SPI communication (unchanged)."""

    def __init__(self):
        self.data = [0.0] * SPI_NUM_FLOATS
        self.message_count = 0
        self.failed_count = 0
        self.last_delta_time_us = 0


# Your original hook: we drive [0],[1] from /cmd_vel.
def load_tx_frame(tx_list, frame_idx, fwd, turn, t0):
    """Update tx_list in-place for this frame (indices [0],[1] from /cmd_vel)."""
    tx_list[0] = fwd
    tx_list[1] = turn


class SpiBridgeNode(Node):
    """
    ROS 2 wrapper around your SPI loop.
    Subscribes:  /cmd_vel
    Publishes:   /robot/feedback, /imu/data_raw, /imu/mag, /spi/diag
    """

    def __init__(self):
        super().__init__("spi_bridge")

        # ---------------- Parameters ----------------
        # SPI + timing
        self.declare_parameter("spi_bus", 0)
        self.declare_parameter("spi_dev", 0)
        self.declare_parameter("spi_max_speed_hz", 33_333_333)
        self.declare_parameter("main_task_period_us", main_task_period_us_default)
        self.declare_parameter("arm_gap_us", ARM_GAP_US_DEFAULT)
        # Safety / control
        self.declare_parameter("cmd_timeout_ms", 200)
        self.declare_parameter("max_forward", 2.0)
        self.declare_parameter("max_turn", 3.0)
        # Frames
        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("base_frame_id", "base_link")
        # Optional unit scaling
        self.declare_parameter("gyro_scale", 1.0)  # raw -> rad/s
        self.declare_parameter("accel_scale", 1.0)  # raw -> m/s^2
        self.declare_parameter("mag_scale", 1.0)  # raw -> Tesla
        # Verbose logging param (runtime override for VERBOSE_CYCLE_LOG)
        self.declare_parameter("verbose_cycle_log", VERBOSE_CYCLE_LOG)

        spi_bus = int(self.get_parameter("spi_bus").value)
        spi_dev = int(self.get_parameter("spi_dev").value)
        spi_speed = int(self.get_parameter("spi_max_speed_hz").value)
        self.main_task_period_us = int(self.get_parameter("main_task_period_us").value)
        self.arm_gap_us = int(self.get_parameter("arm_gap_us").value)
        self.cmd_timeout_ms = int(self.get_parameter("cmd_timeout_ms").value)
        self.max_forward = float(self.get_parameter("max_forward").value)
        self.max_turn = float(self.get_parameter("max_turn").value)
        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.base_frame_id = str(self.get_parameter("base_frame_id").value)
        self.gyro_scale = float(self.get_parameter("gyro_scale").value)
        self.accel_scale = float(self.get_parameter("accel_scale").value)
        self.mag_scale = float(self.get_parameter("mag_scale").value)
        self.verbose_cycle_log = bool(self.get_parameter("verbose_cycle_log").value)

        # ---------------- Startup sanity checks ----------------
        dev_path = f"/dev/spidev{spi_bus}.{spi_dev}"
        if not os.path.exists(dev_path):
            raise RuntimeError(f"{dev_path} not found. Enable SPI: add 'dtparam=spi=on' to /boot/firmware/config.txt and reboot.")
        if spidev is None:
            raise RuntimeError("python3-spidev not installed or failed to import.")

        # ---------------- SPI setup (same semantics) ----------------
        self.spi = spidev.SpiDev()
        try:
            self.spi.open(spi_bus, spi_dev)  # SPI0.0 (GPIO10/9/11/8)
        except PermissionError as e:
            raise RuntimeError(f"Permission denied opening {dev_path}. Add your user to the 'dialout' group:\n" f"  sudo usermod -aG dialout $USER && newgrp dialout") from e
        self.spi.max_speed_hz = spi_speed
        self.spi.mode = 0b00  # SPI mode 0

        # ---------------- Buffers / state (unchanged) ----------------
        self.transmitted_data = SpiData()
        self.received_data = SpiData()

        # tx1: 0x56 + zeros + CRC
        self.tx1 = bytearray(SPI_MSG_SIZE)
        self.tx1[0] = SPI_HEADER_MASTER_ARM
        self.tx1[1:-1] = b"\x00" * (SPI_NUM_FLOATS * 4)
        self.tx1[-1] = calculate_crc8(memoryview(self.tx1)[:-1])

        # tx2: 0x55 + payload (header fixed)
        self.tx2 = bytearray(SPI_MSG_SIZE)
        self.tx2[0] = SPI_HEADER_MASTER

        # Precompile struct format for pack/unpack (clean + tiny perf gain)
        self._struct_floats = struct.Struct(f"<{SPI_NUM_FLOATS}f")

        # Command state (from /cmd_vel)
        self.cmd_forward = 0.0
        self.cmd_turn = 0.0
        self.last_cmd_time = time.perf_counter()

        # QoS
        qos_cmd = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        qos_sensors = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

        # Subs & pubs
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, qos_cmd)
        self.pub_feedback = self.create_publisher(TwistStamped, "/robot/feedback", qos_sensors)
        self.pub_imu = self.create_publisher(Imu, "/imu/data_raw", qos_sensors)
        self.pub_mag = self.create_publisher(MagneticField, "/imu/mag", qos_sensors)
        self.pub_diag = self.create_publisher(DiagnosticArray, "/spi/diag", qos_cmd)

        # Timing & log throttle
        self.start_time = time.perf_counter()
        self.previous_time = self.start_time
        self._last_log_time = self.start_time

        # Use a ROS timer to drive your original “while True” loop period.
        # No manual sleep inside; the timer schedules cadence.
        self.timer = self.create_timer(self.main_task_period_us / 1_000_000.0, self._loop_once)

        self.get_logger().info(f"SPI opened on {dev_path} @ {spi_speed} Hz, period {self.main_task_period_us/1000:.3f} ms")

    # -------------------- Subscribers --------------------
    def _on_cmd_vel(self, msg: Twist):
        # Clamp for safety
        fwd = max(-self.max_forward, min(self.max_forward, msg.linear.x))
        turn = max(-self.max_turn, min(self.max_turn, msg.angular.z))
        self.cmd_forward = float(fwd)
        self.cmd_turn = float(turn)
        self.last_cmd_time = time.perf_counter()

    # -------------------- Main cycle (your loop body) --------------------
    def _loop_once(self):
        cycle_start_time = time.perf_counter()
        fail_reason = ""

        # 1) ARM-ONLY (0x56 + zeros)
        t_xfer1_start = time.perf_counter()
        try:
            _ = self.spi.xfer2(self.tx1)  # list[int]; unused
        except Exception as e:
            # Treat as a failed cycle; publish diag and return.
            t_xfer1_end = time.perf_counter()
            xfer1_us = (t_xfer1_end - t_xfer1_start) * 1_000_000.0
            self.received_data.failed_count += 1
            fail_reason = f"arm_xfer_error: {type(e).__name__}: {e}"
            self.get_logger().warn(f"SPI ARM xfer error: {e}")
            self._publish_diag(xfer1_us, 0.0, True)
            self._finish_period_and_maybe_log(cycle_start_time, xfer1_us, 0.0)
            return
        t_xfer1_end = time.perf_counter()
        xfer1_us = (t_xfer1_end - t_xfer1_start) * 1_000_000.0

        # short busy gap so the slave can re-arm/build fresh TX
        t0 = time.perf_counter()
        target = t0 + self.arm_gap_us / 1_000_000.0
        while time.perf_counter() < target:
            pass

        # 2) PUBLISH (0x55 + real payload)
        # Deadman timeout to zero commands if stale
        if (time.perf_counter() - self.last_cmd_time) * 1000.0 > self.cmd_timeout_ms:
            fwd, turn = 0.0, 0.0
        else:
            fwd, turn = self.cmd_forward, self.cmd_turn

        # Fill payload (first two floats are your command signals)
        load_tx_frame(self.transmitted_data.data, self.transmitted_data.message_count, fwd, turn, self.start_time)

        # Pack all floats in one go; compute CRC
        self._struct_floats.pack_into(self.tx2, 1, *self.transmitted_data.data)
        self.tx2[-1] = calculate_crc8(memoryview(self.tx2)[:-1])

        t_xfer2_start = time.perf_counter()
        try:
            rx2_list = self.spi.xfer2(self.tx2)  # list[int]
        except Exception as e:
            t_xfer2_end = time.perf_counter()
            xfer2_us = (t_xfer2_end - t_xfer2_start) * 1_000_000.0
            self.received_data.failed_count += 1
            fail_reason = f"publish_xfer_error: {type(e).__name__}: {e}"
            self.get_logger().warn(f"SPI PUBLISH xfer error: {e}")
            self._publish_diag(xfer1_us, xfer2_us, True)
            self._finish_period_and_maybe_log(cycle_start_time, xfer1_us, xfer2_us)
            return
        t_xfer2_end = time.perf_counter()
        xfer2_us = (t_xfer2_end - t_xfer2_start) * 1_000_000.0

        # Prefer the second reply (fresh data). Do NOT fallback to the first.
        if not (len(rx2_list) == SPI_MSG_SIZE and verify_checksum_seq(rx2_list) and rx2_list[0] == SPI_HEADER_SLAVE):
            self.received_data.failed_count += 1
            if len(rx2_list) != SPI_MSG_SIZE:
                fail_reason = f"bad_len:{len(rx2_list)}"
            elif not verify_checksum_seq(rx2_list):
                fail_reason = "crc_fail"
            elif rx2_list[0] != SPI_HEADER_SLAVE:
                fail_reason = f"bad_hdr:0x{rx2_list[0]:02X}"
            self.get_logger().warn(f"SPI frame rejected ({fail_reason}).")
            self._publish_diag(xfer1_us, xfer2_us, True)
            self._finish_period_and_maybe_log(cycle_start_time, xfer1_us, xfer2_us)
            return

        # Process received data (bulk unpack)
        rx_ba = bytearray(rx2_list)  # convert once for struct.unpack
        floats_tuple = self._struct_floats.unpack_from(rx_ba, 1)
        self.received_data.data[:] = floats_tuple

        self.received_data.message_count += 1

        # elapsed time between valid messages
        current_time = time.perf_counter()
        delta_time_us = int((current_time - self.previous_time) * 1_000_000)
        self.previous_time = current_time
        self.received_data.last_delta_time_us = delta_time_us

        self.transmitted_data.message_count += 1

        # -------- Publish to ROS (feedback + IMU + mag) --------
        self._publish_feedback()
        self._publish_imu()
        self._publish_mag()
        self._publish_diag(xfer1_us, xfer2_us, False)

        # Log per-cycle (or throttled)
        self._finish_period_and_maybe_log(cycle_start_time, xfer1_us, xfer2_us)

    # -------------------- Helpers --------------------
    def _finish_period_and_maybe_log(self, cycle_start_time, xfer1_us, xfer2_us):
        main_task_elapsed_time_us = (time.perf_counter() - cycle_start_time) * 1_000_000.0

        # 1) Original-style per-cycle log (can be noisy). Controlled by verbose_cycle_log param.
        # 2) If verbose_cycle_log is False, we throttle to ~1 Hz instead.
        if self.verbose_cycle_log:
            self.get_logger().info(f"Busy: {int(main_task_elapsed_time_us)} us | " f"Xfer1: {int(xfer1_us)} us | Xfer2: {int(xfer2_us)} us | " f"Failed: {self.received_data.failed_count}")
        else:
            now = time.perf_counter()
            if now - self._last_log_time >= 1.0:  # throttle ~1 Hz
                self._last_log_time = now
                self.get_logger().info(f"Busy(avg)~{int(main_task_elapsed_time_us)} us | " f"Xfer1~{int(xfer1_us)} us | Xfer2~{int(xfer2_us)} us | " f"Failed: {self.received_data.failed_count} | " f"Period: {int(self.main_task_period_us)} us")

    # -------------------- Publishers --------------------
    def _publish_feedback(self):
        # Nucleo reply layout you specified:
        # [0] forward_speed, [1] turn_rate, [2..4] gyro, [5..7] acc, [8..10] mag
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame_id
        msg.twist.linear.x = float(self.received_data.data[0])
        msg.twist.angular.z = float(self.received_data.data[1])
        self.pub_feedback.publish(msg)

    def _publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id
        msg.orientation_covariance[0] = -1.0  # orientation unknown

        # Optional scaling; leave params at 1.0 to publish raw.
        msg.angular_velocity.x = float(self.received_data.data[2] * self.gyro_scale)
        msg.angular_velocity.y = float(self.received_data.data[3] * self.gyro_scale)
        msg.angular_velocity.z = float(self.received_data.data[4] * self.gyro_scale)

        msg.linear_acceleration.x = float(self.received_data.data[5] * self.accel_scale)
        msg.linear_acceleration.y = float(self.received_data.data[6] * self.accel_scale)
        msg.linear_acceleration.z = float(self.received_data.data[7] * self.accel_scale)
        self.pub_imu.publish(msg)

    def _publish_mag(self):
        msg = MagneticField()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame_id
        msg.magnetic_field.x = float(self.received_data.data[8] * self.mag_scale)
        msg.magnetic_field.y = float(self.received_data.data[9] * self.mag_scale)
        msg.magnetic_field.z = float(self.received_data.data[10] * self.mag_scale)
        self.pub_mag.publish(msg)

    def _publish_diag(self, xfer1_us: float, xfer2_us: float, failed: bool):
        da = DiagnosticArray()
        da.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = "spi_bridge/status"
        st.level = DiagnosticStatus.WARN if failed else DiagnosticStatus.OK
        st.message = "last cycle failed" if failed else "ok"
        st.values = [
            KeyValue(key="msg_count", value=str(self.received_data.message_count)),
            KeyValue(key="fail_count", value=str(self.received_data.failed_count)),
            KeyValue(key="delta_time_us", value=str(self.received_data.last_delta_time_us)),
            KeyValue(key="xfer1_us", value=str(int(xfer1_us))),
            KeyValue(key="xfer2_us", value=str(int(xfer2_us))),
            KeyValue(key="period_us", value=str(self.main_task_period_us)),
            KeyValue(key="arm_gap_us", value=str(self.arm_gap_us)),
        ]
        da.status = [st]
        self.pub_diag.publish(da)


def main():
    rclpy.init()
    node = SpiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure SPI device is released
        try:
            node.spi.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

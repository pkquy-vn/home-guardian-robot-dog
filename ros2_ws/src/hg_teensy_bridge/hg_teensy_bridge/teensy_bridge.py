# SPDX-License-Identifier: Apache-2.0
"""teensy_bridge: ROS 2 node for the Pi <-> Teensy serial link.

Implements what the firmware supports today:
  - sends HEARTBEAT at heartbeat_rate_hz with an incrementing uint32 sequence;
  - sends MODE for each /teensy/cmd_mode message and CLEAR_FAULT for each /teensy/clear_fault
    service call (the Teensy decides whether to accept them; there is no acknowledgement);
  - parses STATUS and publishes /teensy/mode, /teensy/faults, /teensy/connected and
    /diagnostics;
  - parses JOINT_STATE and publishes /joint_states (sensor_msgs/JointState, radians), with the
    joint names of the hg_description URDF.

Safety: the heartbeat tells the Teensy that the Pi-side software is alive; the firmware raises
HEARTBEAT_LOST after 500 ms without one. The heartbeat is therefore sent from a timer on this
node's executor, in the same (default, mutually exclusive) callback group as every other
callback, and never from a separate thread. If any callback of this node hangs, or the executor
stops spinning, heartbeats stop and the Teensy watchdog trips. On shutdown the heartbeat timer is
cancelled before the port is closed.
"""

from __future__ import annotations

import math
import os
import time
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, UInt8, UInt16
from std_srvs.srv import Trigger

import serial

from hg_teensy_bridge import protocol

READ_PERIOD_S = 0.01  # Serial poll period (non-blocking reads).
RECONNECT_PERIOD_S = 1.0  # Retry period while the port is missing.
CONNECTED_PERIOD_S = 0.5  # /teensy/connected publish period (2 Hz), plus on change.
DIAGNOSTICS_PERIOD_S = 1.0  # /diagnostics publish period.
WRITE_TIMEOUT_S = 0.02  # Upper bound on a heartbeat write if the OS transmit buffer is full.
MAX_READ_BYTES = 4096  # Upper bound on bytes read per poll.

# URDF joint names (hg_description) in JOINT_STATE order: FL hip, FL knee, FR hip, ... RR knee.
# The URDF joint values equal the firmware hip and knee angles (docs/kinematics.md).
JOINT_NAMES = [
    f"{leg}_{joint}_joint" for leg in ("fl", "fr", "rl", "rr") for joint in ("hip", "knee")
]


class TeensyBridge(Node):
    def __init__(self) -> None:
        super().__init__("teensy_bridge")

        self._port_name: str = self.declare_parameter("port", "/dev/ttyACM0").value
        self._baud: int = self.declare_parameter("baud", 115200).value
        heartbeat_rate_hz: float = self.declare_parameter("heartbeat_rate_hz", 20.0).value
        self._status_timeout_s: float = self.declare_parameter("status_timeout_s", 0.5).value
        if heartbeat_rate_hz <= 0.0:
            raise ValueError(f"heartbeat_rate_hz must be > 0, got {heartbeat_rate_hz}")
        if self._status_timeout_s <= 0.0:
            raise ValueError(f"status_timeout_s must be > 0, got {self._status_timeout_s}")

        self._mode_pub = self.create_publisher(UInt8, "/teensy/mode", 10)
        self._faults_pub = self.create_publisher(UInt16, "/teensy/faults", 10)
        self._connected_pub = self.create_publisher(Bool, "/teensy/connected", 10)
        self._diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._cmd_mode_sub = self.create_subscription(
            UInt8, "/teensy/cmd_mode", self._on_cmd_mode, 10
        )
        self._clear_fault_srv = self.create_service(
            Trigger, "/teensy/clear_fault", self._on_clear_fault
        )

        self._serial: Optional[serial.Serial] = None
        self._parser = protocol.FrameParser()
        self._crc_errors_previous = 0  # CRC errors counted by parsers of earlier connections.
        self._unavailable_reported = False

        self._sequence = 0
        self._heartbeats_sent = 0
        self._heartbeats_dropped = 0

        self._last_status_time: Optional[float] = None  # time.monotonic() of the last STATUS
        self._mode: Optional[int] = None
        self._fault_bits: Optional[int] = None
        self._connected = False
        self._status_in_window = 0
        self._window_start = time.monotonic()
        self._stopping = False

        # All timers use the node's default callback group (mutually exclusive) on the executor
        # that spins this node. Do not move the heartbeat to its own thread or callback group.
        self._heartbeat_timer = self.create_timer(1.0 / heartbeat_rate_hz, self._send_heartbeat)
        self._timers = [
            self._heartbeat_timer,
            self.create_timer(READ_PERIOD_S, self._poll_serial),
            self.create_timer(RECONNECT_PERIOD_S, self._ensure_port),
            self.create_timer(CONNECTED_PERIOD_S, self._publish_connected),
            self.create_timer(DIAGNOSTICS_PERIOD_S, self._publish_diagnostics),
        ]

        self.get_logger().info(
            f"port={self._port_name} baud={self._baud} heartbeat={heartbeat_rate_hz:g} Hz "
            f"status_timeout={self._status_timeout_s:g} s"
        )
        self._ensure_port()
        self._publish_connected()

    # ------------------------------------------------------------------------------------------
    # Serial port handling
    # ------------------------------------------------------------------------------------------

    def _ensure_port(self) -> None:
        """Opens the port if it is closed; detects a port that has disappeared."""
        if self._stopping:
            return
        if self._serial is not None:
            if not os.path.exists(self._port_name):
                self._port_lost("device node disappeared")
            return
        try:
            port = serial.Serial(
                self._port_name,
                self._baud,
                timeout=0,  # Non-blocking reads.
                write_timeout=WRITE_TIMEOUT_S,
                exclusive=True,
            )
        except (serial.SerialException, OSError, ValueError) as exc:
            if not self._unavailable_reported:
                self.get_logger().warning(
                    f"cannot open {self._port_name}: {exc}; retrying every "
                    f"{RECONNECT_PERIOD_S:g} s"
                )
                self._unavailable_reported = True
            return
        try:
            port.reset_input_buffer()
        except (serial.SerialException, OSError):
            pass
        self._serial = port
        self._crc_errors_previous += self._parser.crc_errors
        self._parser = protocol.FrameParser()
        self._unavailable_reported = False
        self.get_logger().info(f"opened {self._port_name}")

    def _port_lost(self, reason: object) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
            self._serial = None
        self._last_status_time = None
        if not self._unavailable_reported:
            self.get_logger().warning(
                f"lost {self._port_name}: {reason}; retrying every {RECONNECT_PERIOD_S:g} s"
            )
            self._unavailable_reported = True
        self._update_connected()

    # ------------------------------------------------------------------------------------------
    # Heartbeat (safety-relevant: see module docstring)
    # ------------------------------------------------------------------------------------------

    def _send_heartbeat(self) -> None:
        if self._stopping or self._serial is None:
            return
        frame = protocol.encode_frame(protocol.HEARTBEAT, protocol.pack_heartbeat(self._sequence))
        if self._write_frame(frame):
            self._sequence = (self._sequence + 1) & 0xFFFFFFFF
            self._heartbeats_sent += 1
        else:
            self._heartbeats_dropped += 1  # Skipped, never queued.

    def _write_frame(self, frame: bytes) -> bool:
        """Writes one frame; returns True only if it was written completely."""
        if self._stopping or self._serial is None:
            return False
        try:
            written = self._serial.write(frame)
        except serial.SerialTimeoutException:
            return False  # OS transmit buffer full (the Teensy is not reading).
        except (serial.SerialException, OSError) as exc:
            self._port_lost(exc)
            return False
        return written == len(frame)

    # ------------------------------------------------------------------------------------------
    # Commands
    # ------------------------------------------------------------------------------------------

    def _on_cmd_mode(self, msg: UInt8) -> None:
        mode = msg.data
        name = protocol.mode_name(mode)
        if mode not in protocol.MODE_NAMES:
            self.get_logger().warning(f"cmd_mode {name}: unknown mode; the Teensy will reject it")
        if self._write_frame(protocol.encode_frame(protocol.MODE, protocol.pack_mode(mode))):
            self.get_logger().info(f"sent MODE {name}")
        else:
            self.get_logger().warning(f"MODE {name} not sent: serial port unavailable")

    def _on_clear_fault(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request  # Trigger requests are empty.
        if self._write_frame(protocol.encode_frame(protocol.CLEAR_FAULT)):
            response.success = True
            response.message = (
                "CLEAR_FAULT sent; the Teensy clears the latch only while heartbeats are healthy. "
                "Check /teensy/faults."
            )
        else:
            response.success = False
            response.message = "CLEAR_FAULT not sent: serial port unavailable"
        self.get_logger().info(response.message)
        return response

    # ------------------------------------------------------------------------------------------
    # STATUS
    # ------------------------------------------------------------------------------------------

    def _poll_serial(self) -> None:
        if self._serial is not None:
            data = b""
            try:
                waiting = self._serial.in_waiting
                if waiting:
                    data = self._serial.read(min(waiting, MAX_READ_BYTES))
            except (serial.SerialException, OSError) as exc:
                self._port_lost(exc)
            for msg_type, payload in self._parser.feed(data):
                if msg_type == protocol.STATUS:
                    self._handle_status(payload)
                elif msg_type == protocol.JOINT_STATE:
                    self._handle_joint_state(payload)
        self._update_connected()

    def _handle_joint_state(self, payload: bytes) -> None:
        joints = protocol.unpack_joint_state(payload)
        if joints is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        # Position in 0.01 deg -> rad. Current and temperature (0 until servos exist) are not
        # published: their units are not defined yet.
        msg.position = [math.radians(position_cdeg / 100.0) for position_cdeg, _, _ in joints]
        self._joint_state_pub.publish(msg)

    def _handle_status(self, payload: bytes) -> None:
        status = protocol.unpack_status(payload)
        if status is None:
            return
        mode, _gait_phase, fault_bits = status
        self._last_status_time = time.monotonic()
        self._status_in_window += 1
        self._mode = mode
        self._fault_bits = fault_bits
        self._mode_pub.publish(UInt8(data=mode))
        self._faults_pub.publish(UInt16(data=fault_bits))

    def _is_connected(self) -> bool:
        return (
            self._serial is not None
            and self._last_status_time is not None
            and time.monotonic() - self._last_status_time <= self._status_timeout_s
        )

    def _update_connected(self) -> None:
        """Publishes /teensy/connected immediately when the link state changes."""
        connected = self._is_connected()
        if connected == self._connected:
            return
        self._connected = connected
        if connected:
            self.get_logger().info("link up: receiving STATUS")
        else:
            self.get_logger().warning(f"link down: no STATUS for {self._status_timeout_s:g} s")
        self._connected_pub.publish(Bool(data=connected))

    def _publish_connected(self) -> None:
        """Periodic (2 Hz) publish of the current link state."""
        self._update_connected()  # Logs and publishes first if the state just changed.
        self._connected_pub.publish(Bool(data=self._connected))

    # ------------------------------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------------------------------

    def _publish_diagnostics(self) -> None:
        now = time.monotonic()
        elapsed = now - self._window_start
        status_rate = self._status_in_window / elapsed if elapsed > 0.0 else 0.0
        self._status_in_window = 0
        self._window_start = now

        faults = self._fault_bits
        if self._serial is None:
            level, link_state, message = DiagnosticStatus.ERROR, "no port", "port unavailable"
        elif not self._is_connected():
            level, link_state, message = DiagnosticStatus.ERROR, "no status", "no STATUS received"
        elif faults:
            level, link_state = DiagnosticStatus.WARN, "connected"
            message = f"faults: {protocol.decode_fault_bits(faults)}"
        else:
            level, link_state, message = DiagnosticStatus.OK, "connected", "connected"

        diag = DiagnosticStatus()
        diag.level = level
        diag.name = "teensy_bridge: Teensy link"
        diag.message = message
        diag.hardware_id = self._port_name
        diag.values = [
            KeyValue(key="link_state", value=link_state),
            KeyValue(key="status_rate_hz", value=f"{status_rate:.1f}"),
            KeyValue(key="mode", value="-" if self._mode is None else str(self._mode)),
            KeyValue(
                key="mode_name", value="-" if self._mode is None else protocol.mode_name(self._mode)
            ),
            KeyValue(key="fault_bits", value="-" if faults is None else f"0x{faults:04X}"),
            KeyValue(
                key="faults", value="-" if faults is None else protocol.decode_fault_bits(faults)
            ),
            KeyValue(
                key="crc_errors", value=str(self._crc_errors_previous + self._parser.crc_errors)
            ),
            KeyValue(key="heartbeats_sent", value=str(self._heartbeats_sent)),
            KeyValue(key="heartbeats_dropped", value=str(self._heartbeats_dropped)),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [diag]
        self._diag_pub.publish(array)

    # ------------------------------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------------------------------

    def stop(self) -> None:
        """Stops heartbeats first, then closes the port. Safe to call more than once."""
        self._stopping = True
        for timer in self._timers:
            timer.cancel()
        if self._serial is not None:
            try:
                self._serial.close()
            except (serial.SerialException, OSError):
                pass
            self._serial = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TeensyBridge()
        # Single-threaded executor: every callback, including the heartbeat, runs here.
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

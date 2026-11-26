# SPDX-License-Identifier: Apache-2.0
"""servo: command-line tool for joint calibration and single-joint tests.

It talks to the running teensy_bridge node, which owns the serial port:

    ros2 run hg_teensy_bridge servo show
    ros2 run hg_teensy_bridge servo calibrate fl_hip fl_knee
    ros2 run hg_teensy_bridge servo flip fl_knee
    ros2 run hg_teensy_bridge servo angle fl_knee 100
    ros2 run hg_teensy_bridge servo speed 30
    ros2 run hg_teensy_bridge servo clear-fault
    ros2 run hg_teensy_bridge servo restore
    ros2 run hg_teensy_bridge servo off

Every command prints the Teensy's answer and exits with 0 only if it was accepted. Postures
(LIE, SIT, STAND) keep using the mode command: /teensy/cmd_mode.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Callable, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String, UInt8
from std_srvs.srv import Trigger

from hg_msgs.msg import ServoCommand
from hg_teensy_bridge import protocol

BRIDGE_TIMEOUT_S = 5.0  # Time allowed to find the teensy_bridge node.
ANSWER_TIMEOUT_S = 3.0  # Time allowed for the Teensy's answer to one command.
QUIET_S = 0.6  # RESTORE: stop collecting answers after this long without one.
LINK_STATUS_NAME = "teensy_bridge: Teensy link"

JOINT_HELP = "joint: " + ", ".join(protocol.JOINT_KEYS)


def joint_argument(text: str) -> int:
    """argparse type: a joint name such as fl_knee; returns its index."""
    key = text.lower()
    if key not in protocol.JOINT_KEYS:
        raise argparse.ArgumentTypeError(f"unknown joint '{text}' ({JOINT_HELP})")
    return protocol.JOINT_KEYS.index(key)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="servo", description="Joint calibration and single-joint tests. " + JOINT_HELP
    )
    commands = parser.add_subparsers(dest="command", required=True, metavar="command")
    commands.add_parser("show", help="calibration and state of every joint")
    sub = commands.add_parser(
        "calibrate",
        help="store the current position as the reference pose (hip 0 deg: femur vertical; "
        "knee 90 deg). The joints must be limp and held at that pose",
    )
    sub.add_argument("joints", nargs="+", type=joint_argument, metavar="joint")
    sub = commands.add_parser("flip", help="invert the direction sign of the joints")
    sub.add_argument("joints", nargs="+", type=joint_argument, metavar="joint")
    sub = commands.add_parser(
        "angle", help="drive one joint to an angle in degrees, until the next mode command"
    )
    sub.add_argument("joint", type=joint_argument)
    sub.add_argument("degrees", type=float)
    sub = commands.add_parser("speed", help="set the maximum joint speed in degrees per second")
    sub.add_argument("deg_per_s", type=float)
    commands.add_parser("clear-fault", help="clear latched faults")
    sub = commands.add_parser(
        "restore", help="send the calibration copy saved on the Pi back to the Teensy"
    )
    sub.add_argument("joints", nargs="*", type=joint_argument, metavar="joint")
    commands.add_parser("off", help="torque off on every servo, now")
    return parser


class ServoCli(Node):
    def __init__(self) -> None:
        super().__init__("servo_cli")
        self.answers: list[str] = []
        self.diagnostics: Optional[DiagnosticArray] = None
        self._command_pub = self.create_publisher(ServoCommand, "/teensy/servo_command", 10)
        self._mode_pub = self.create_publisher(UInt8, "/teensy/cmd_mode", 10)
        self._clear_client = self.create_client(Trigger, "/teensy/clear_fault")
        self._answer_sub = self.create_subscription(
            String, "/teensy/command_result", self._on_answer, 10
        )
        self._diagnostics_sub = self.create_subscription(
            DiagnosticArray, "/diagnostics", self._on_diagnostics, 10
        )

    def _on_answer(self, msg: String) -> None:
        self.answers.append(msg.data)

    def _on_diagnostics(self, msg: DiagnosticArray) -> None:
        # Other nodes publish on /diagnostics too: keep only the bridge's arrays.
        if any(status.name == LINK_STATUS_NAME for status in msg.status):
            self.diagnostics = msg

    def spin_until(self, condition: Callable[[], bool], timeout_s: float) -> bool:
        """Spins until condition() is true. Returns False on timeout."""
        deadline = time.monotonic() + timeout_s
        while not condition():
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def wait_for_bridge(self) -> bool:
        """Waits until the bridge is listening and its answers can be received."""

        def ready() -> bool:
            return (
                self._command_pub.get_subscription_count() > 0
                and self._mode_pub.get_subscription_count() > 0
                and self.count_publishers("/teensy/command_result") > 0
            )

        if not self.spin_until(ready, BRIDGE_TIMEOUT_S):
            return False
        self.spin_until(lambda: False, 0.3)  # Let the subscriptions finish matching.
        return True

    def request(self, send: Callable[[], object], collect_all: bool = False) -> bool:
        """Sends one command and prints the answer (collect_all: every answer until they stop).
        Returns True if every answer is OK."""
        self.answers.clear()
        send()
        if not self.spin_until(lambda: bool(self.answers), ANSWER_TIMEOUT_S):
            print("no answer. Is the Teensy connected? Check: ros2 topic echo /teensy/connected")
            return False
        while collect_all:
            count = len(self.answers)
            self.spin_until(lambda: len(self.answers) > count, QUIET_S)
            if len(self.answers) == count:
                break
        for answer in self.answers:
            print(answer)
        return all(answer.startswith("OK") for answer in self.answers)

    def send_servo_command(self, command: int, joint: int = 0, value: float = 0.0) -> None:
        msg = ServoCommand()
        msg.command = command
        msg.joint = joint
        msg.value = float(value)
        self._command_pub.publish(msg)

    def clear_fault(self) -> bool:
        if not self._clear_client.wait_for_service(timeout_sec=BRIDGE_TIMEOUT_S):
            print("service /teensy/clear_fault is not available")
            return False
        return self.request(lambda: self._clear_client.call_async(Trigger.Request()))

    def torque_off(self) -> bool:
        return self.request(lambda: self._mode_pub.publish(UInt8(data=protocol.MODE_TORQUE_OFF)))

    def show(self) -> bool:
        """Prints one line for the Teensy and a table with one row per joint."""
        if not self.spin_until(lambda: self.diagnostics is not None, ANSWER_TIMEOUT_S):
            print("no /diagnostics from teensy_bridge")
            return False
        rows = [
            ("joint", "servo", "present", "calibrated", "reference", "dir", "torque", "driven",
             "target", "measured", "can drive")
        ]
        for status in self.diagnostics.status:
            values = {item.key: item.value for item in status.values}
            if status.name == LINK_STATUS_NAME:
                print(
                    f"Teensy: {values.get('link_state', '?')}, "
                    f"mode {values.get('mode_name', '?')}, faults {values.get('faults', '?')}, "
                    f"speed limit {values.get('speed_limit_deg_s', '?')} deg/s"
                )
                if values.get("fault_causes", "none") != "none":
                    print(f"fault causes: {values['fault_causes']}")
                continue
            fields = status.name.split(" ")
            if len(fields) < 2 or fields[0] != "servo" or not fields[1].isdigit():
                continue
            index = int(fields[1]) - 1
            if not 0 <= index < protocol.NUM_JOINTS:
                continue
            block = values.get("drive_block", "?")
            if values.get("driven") == "yes" or block == "none":
                can_drive = "yes"
            else:
                can_drive = f"no: {block}"
            target = values.get("target_deg", "?")
            if values.get("test_target") == "yes":
                target += " (test)"
            rows.append((
                protocol.JOINT_KEYS[index],
                str(index + 1),
                {"true": "yes", "false": "no"}.get(values.get("present", ""), "?"),
                values.get("calibrated", "?"),
                values.get("reference_raw", "?"),
                values.get("direction", "?"),
                values.get("torque", "?"),
                values.get("driven", "?"),
                target,
                values.get("measured_deg", "?"),
                can_drive,
            ))
        widths = [max(len(row[column]) for row in rows) for column in range(len(rows[0]))]
        for row in rows:
            print("  ".join(cell.ljust(width) for cell, width in zip(row, widths)).rstrip())
        return True


def run(node: ServoCli, options: argparse.Namespace) -> bool:
    if not node.wait_for_bridge():
        print("teensy_bridge is not running. Start it: ros2 launch hg_bringup robot.launch.py")
        return False
    command = options.command
    if command == "show":
        return node.show()
    if command == "clear-fault":
        return node.clear_fault()
    if command == "off":
        return node.torque_off()
    if command == "speed":
        return node.request(
            lambda: node.send_servo_command(ServoCommand.SET_SPEED_LIMIT, 0, options.deg_per_s)
        )
    if command == "angle":
        return node.request(
            lambda: node.send_servo_command(ServoCommand.SET_ANGLE, options.joint, options.degrees)
        )
    if command == "restore":
        joints = options.joints or [ServoCommand.ALL_JOINTS]
        results = [
            node.request(lambda j=j: node.send_servo_command(ServoCommand.RESTORE, j), True)
            for j in joints
        ]
        return all(results)
    code = ServoCommand.CALIBRATE if command == "calibrate" else ServoCommand.FLIP_DIRECTION
    results = [node.request(lambda j=j: node.send_servo_command(code, j)) for j in options.joints]
    return all(results)


def main(args: Optional[list[str]] = None) -> None:
    arguments = remove_ros_args(args=sys.argv if args is None else args)
    options = build_parser().parse_args(arguments[1:])
    rclpy.init(args=args)
    node = ServoCli()
    ok = False
    try:
        ok = run(node, options)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

# SPDX-License-Identifier: Apache-2.0
"""Launches the teensy_bridge node.

Example:
    ros2 launch hg_teensy_bridge bridge.launch.py port:=/dev/ttyACM0 heartbeat_rate_hz:=20.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

ARGUMENTS = [
    # name, default, type, description
    ("port", "/dev/ttyACM0", str, "Serial port of the Teensy"),
    ("baud", "115200", int, "Baud rate (ignored by USB serial)"),
    ("heartbeat_rate_hz", "20.0", float, "HEARTBEAT send rate"),
    ("status_timeout_s", "0.5", float, "Link is 'connected' while STATUS arrived within this time"),
]


def generate_launch_description() -> LaunchDescription:
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=description)
        for name, default, _, description in ARGUMENTS
    ]
    parameters = {
        name: ParameterValue(LaunchConfiguration(name), value_type=value_type)
        for name, _, value_type, _ in ARGUMENTS
    }
    bridge = Node(
        package="hg_teensy_bridge",
        executable="teensy_bridge",
        name="teensy_bridge",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription([*declared, bridge])

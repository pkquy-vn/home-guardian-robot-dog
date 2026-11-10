# SPDX-License-Identifier: Apache-2.0
"""Starts the Teensy bridge and robot_state_publisher with the hg_description model.

Example:
    ros2 launch hg_bringup robot.launch.py port:=/dev/ttyACM0

The bridge's launch arguments (port, baud, heartbeat_rate_hz, status_timeout_s) pass through.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bridge_launch = os.path.join(
        get_package_share_directory("hg_teensy_bridge"), "launch", "bridge.launch.py"
    )
    urdf_xacro = os.path.join(
        get_package_share_directory("hg_description"), "urdf", "hg.urdf.xacro"
    )
    robot_description = ParameterValue(Command(["xacro ", urdf_xacro]), value_type=str)

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
        ]
    )

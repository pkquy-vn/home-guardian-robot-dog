# SPDX-License-Identifier: Apache-2.0
"""Starts thermal_node (MLX90640 driver) and thermal_detector with config/thermal.yaml.

Example:
    ros2 launch hg_perception thermal.launch.py
    ros2 launch hg_perception thermal.launch.py thermal_config:=/path/to/other.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("hg_perception"), "config", "thermal.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "thermal_config",
                default_value=default_config,
                description="Parameter file for thermal_node and thermal_detector",
            ),
            Node(
                package="hg_perception",
                executable="thermal_node",
                name="thermal_node",
                output="screen",
                parameters=[LaunchConfiguration("thermal_config")],
            ),
            Node(
                package="hg_perception",
                executable="thermal_detector",
                name="thermal_detector",
                output="screen",
                parameters=[LaunchConfiguration("thermal_config")],
            ),
        ]
    )

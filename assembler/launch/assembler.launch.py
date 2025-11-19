#!/usr/bin/env python3
"""
ROS 2 Jazzy launch file for ARMS simulator withAssembler node.

Launches:
  - Assembler action server node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import LogInfo, ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """Generate the launch description for Webots and Assembler node."""

    # Assembler action server node - delayed to let Webots driver start
    assembler_node = Node(
        package="assembler",
        executable="assembler_action_server",
        name="assembler_action_server",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )


    # Launch description
    ld = LaunchDescription([
        LogInfo(msg="Starting ARMS simulator with Assembler node..."),
        assembler_node
    ])

    return ld
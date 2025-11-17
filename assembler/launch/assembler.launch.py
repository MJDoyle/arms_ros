#!/usr/bin/env python3
"""
ROS 2 Jazzy launch file for ARMS simulator with Webots and Assembler node.

Launches:
  - Webots simulation environment with custom world
  - Assembler action server node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import LogInfo, ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
import os
from webots_ros2_driver.webots_launcher import WebotsLauncher



def generate_launch_description():
    """Generate the launch description for Webots and Assembler node."""

    # Get package directories
    webots_pkg = get_package_share_directory("webots_controllers")

    # Define path to Webots world file
    world = os.path.join(webots_pkg, "worlds", "assembler_world.wbt")

    webots = WebotsLauncher(world=world, mode="realtime", ros2_supervisor=True)

    # Assembler action server node - delayed to let Webots driver start
    assembler_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="assembler",
                executable="assembler_action_server",
                name="assembler_action_server",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ]
    )

    # Launch description
    ld = LaunchDescription([
        LogInfo(msg="Starting ARMS simulator with Webots and Assembler..."),
        webots,
        webots._supervisor,
        assembler_node
    ])

    return ld
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
    # Background models — supported formats (comma-separated):
    #   "path,r,g,b"                      — origin, default orientation
    #   "path,r,g,b,x,y,z"               — position in mm, default orientation
    #   "path,r,g,b,x,y,z,qx,qy,qz,qw"  — position in mm + explicit quaternion
    # r,g,b in 0–1.  background_y_up (below) rotates entries without an
    # explicit quaternion by 90° around X (Y-up CAD → Z-up ROS).
    background_models = [
        "/home/md/dev/arms_ros2_ws/arms_model/Self-Release Bed - Bed Frame X-1.STL,0.1,0.1,0.1,610,0,-81,0,0.7071,0.7071,0",
        "/home/md/dev/arms_ros2_ws/arms_model/Self-Release Bed - Bed Frame X-2.STL,0.1,0.1,0.1,610,0,-81,0,0.7071,0.7071,0",
        "/home/md/dev/arms_ros2_ws/arms_model/Self-Release Bed - Bed Frame Y-1.STL,0.1,0.1,0.1,610,0,-81,0,0.7071,0.7071,0",
        "/home/md/dev/arms_ros2_ws/arms_model/Self-Release Bed - Bed Frame Y-2.STL,0.1,0.1,0.1,610,0,-81,0,0.7071,0.7071,0",
        "/home/md/dev/arms_ros2_ws/arms_model/Self-Release Bed - Bed Frame Y-3.STL,0.1,0.1,0.1,610,0,-81,0,0.7071,0.7071,0",
        "/home/md/dev/arms_ros2_ws/arms_model/Self-Release Bed - Heat Spreader-1.STL,0.8,0.8,0.8,610,0,-81,0,0.7071,0.7071,0",
        "/home/md/dev/arms_ros2_ws/arms_model/Self-Release Bed - SRB Base Plate-1.STL,0.6,0.6,0.6,610,0,-81,0,0.7071,0.7071,0",
        "/home/md/dev/arms_ros2_ws/arms_model/Self-Release Bed - Touch Probe-1.STL,0.9,0.95,0.95,610,0,-81,0,0.7071,0.7071,0",
        # Example:
        # "/abs/path/to/table.stl,0.6,0.5,0.4",
        # "/abs/path/to/frame.stl,0.3,0.3,0.3",
    ]

    assembler_node = Node(
        package="assembler",
        executable="assembler_action_server",
        name="assembler_action_server",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "background_models": background_models,
            # True = rotate 90° around X to convert Y-up CAD exports to Z-up ROS convention.
            # Set to False if your STLs were already exported in Z-up.
            "background_y_up": True,
        }],
    )


    foxglove_bridge_node = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "port": 8765,
            "address": "0.0.0.0",
            "tls": False,
            "send_buffer_limit": 10000000,
        }],
    )

    # Open Foxglove Studio after a short delay so the bridge is ready to accept connections
    foxglove_studio = TimerAction(
        period=2.0,
        actions=[ExecuteProcess(
            cmd=["foxglove-studio"],
            output="screen",
        )],
    )

    # Launch description
    ld = LaunchDescription([
        LogInfo(msg="Starting ARMS simulator with Assembler node..."),
        assembler_node,
        foxglove_bridge_node,
        foxglove_studio,
    ])

    return ld
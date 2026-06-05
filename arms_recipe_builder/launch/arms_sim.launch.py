#!/usr/bin/env python3
"""
arms_sim.launch.py — Launch the ARMS digital twin + recipe stack.

Starts:
  1. Isaac Sim digital twin  (arms_isaacsim/scripts/run_digital_twin.sh)
  2. Recipe builder node     (provides /arms/execute_assembly)
  3. Recipe UI               (optional tkinter recipe browser)

Launch arguments
----------------
  output_dir      Assembler output directory — absolute or relative to ws root.
                  Default: assembler_output
  headless        Run Isaac Sim without GUI.  Default: false
  launch_ui       Also open the recipe browser GUI.  Default: true
  isaacsim_scripts  Absolute path to arms_isaacsim/scripts/.
                  Default: auto-detected from workspace root.

Usage
-----
  ros2 launch arms_recipe_builder arms_sim.launch.py
  ros2 launch arms_recipe_builder arms_sim.launch.py headless:=true
  ros2 launch arms_recipe_builder arms_sim.launch.py output_dir:=/abs/path/to/assembler_output launch_ui:=false
"""

from __future__ import annotations

import os
import pathlib

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# ---------------------------------------------------------------------------
# Workspace root — derived from the installed share directory.
# install/arms_recipe_builder/share/arms_recipe_builder  →  (up 4) → ws root
# ---------------------------------------------------------------------------
_share = pathlib.Path(get_package_share_directory('arms_recipe_builder'))
_WS_ROOT = _share.parents[3]

_DEFAULT_ISAACSIM_SCRIPTS = str(_WS_ROOT / 'arms_isaacsim' / 'scripts')
_DEFAULT_OUTPUT_DIR       = str(_WS_ROOT / 'assembler_output')


def generate_launch_description():

    # ------------------------------------------------------------------
    # Launch arguments
    # ------------------------------------------------------------------
    arg_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value=_DEFAULT_OUTPUT_DIR,
        description='Path to the assembler_output directory',
    )

    arg_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Isaac Sim without a GUI window',
    )

    arg_launch_ui = DeclareLaunchArgument(
        'launch_ui',
        default_value='true',
        description='Open the recipe browser GUI',
    )

    arg_isaacsim_scripts = DeclareLaunchArgument(
        'isaacsim_scripts',
        default_value=_DEFAULT_ISAACSIM_SCRIPTS,
        description='Absolute path to arms_isaacsim/scripts/',
    )

    # ------------------------------------------------------------------
    # 1. Isaac Sim digital twin
    #    run_digital_twin.sh sources ROS2, writes the Python path file,
    #    then exec's the isaacsim venv python on digital_twin.py.
    #    We pass --headless only when the argument is 'true'.
    # ------------------------------------------------------------------
    # Build a single bash command string so we can conditionally add --headless.
    digital_twin = ExecuteProcess(
        cmd=[PythonExpression([
            "'/bin/bash ",
            LaunchConfiguration('isaacsim_scripts'),
            "/run_digital_twin.sh"
            " --output-dir ",
            LaunchConfiguration('output_dir'),
            " --headless' if '",
            LaunchConfiguration('headless'),
            "' == 'true' else '/bin/bash ",
            LaunchConfiguration('isaacsim_scripts'),
            "/run_digital_twin.sh"
            " --output-dir ",
            LaunchConfiguration('output_dir'),
            "'",
        ])],
        shell=True,
        output='screen',
        name='digital_twin',
    )

    # ------------------------------------------------------------------
    # 2. Recipe builder node
    #    Waits for /arms/machine/* action servers internally, so no delay
    #    needed — but a short one avoids noisy "server not ready" logs.
    # ------------------------------------------------------------------
    recipe_builder = TimerAction(
        period=5.0,
        actions=[Node(
            package='arms_recipe_builder',
            executable='recipe_builder_node',
            name='recipe_builder',
            output='screen',
            parameters=[{
                'output_base_dir': LaunchConfiguration('output_dir'),
            }],
        )],
    )

    # ------------------------------------------------------------------
    # 3. Recipe UI (optional)
    # ------------------------------------------------------------------
    recipe_ui = TimerAction(
        period=5.0,
        actions=[Node(
            package='arms_recipe_builder',
            executable='recipe_ui',
            name='recipe_ui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('launch_ui')),
            parameters=[{
                'output_base_dir': LaunchConfiguration('output_dir'),
            }],
        )],
    )

    return LaunchDescription([
        arg_output_dir,
        arg_headless,
        arg_launch_ui,
        arg_isaacsim_scripts,

        LogInfo(msg=['[ARMS] Workspace root: ', str(_WS_ROOT)]),
        LogInfo(msg=['[ARMS] Output dir:     ', LaunchConfiguration('output_dir')]),
        LogInfo(msg=['[ARMS] IsaacSim:       ',
                     PythonExpression(["'headless' if '", LaunchConfiguration('headless'),
                                       "' == 'true' else 'GUI'"])]),

        digital_twin,
        recipe_builder,
        recipe_ui,
    ])

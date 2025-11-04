#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Path to the Webots world file. Leave empty to use default world.'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Webots'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Webots in headless mode (no GUI)'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug output for all nodes'
    )
    
    # Get package directories
    #assembler_pkg = get_package_share_directory('assembler')
    #webots_controllers_pkg = get_package_share_directory('webots_controllers')
    
    # Default world file path
    default_world = ''
    #default_world = PathJoinSubstitution([
    #    FindPackageShare('webots_controllers'),
    #    'worlds',
    #    'assembly_world.wbt'
    #])
    
    # Assembler action server node
    assembler_node = Node(
        package='assembler',
        executable='assembler_action_server',
        name='assembler_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # Add any specific parameters your assembler needs
            # 'assembly_tolerance': 0.001,
            # 'max_parts': 100,
        }],
        arguments=['--ros-args', '--log-level', 'INFO'] if not LaunchConfiguration('debug') else 
                 ['--ros-args', '--log-level', 'DEBUG']
    )
    
    # Webots model loader service
    model_loader_node = Node(
        package='webots_model_loader',
        executable='webots_model_loader',
        name='webots_model_loader',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            # Model loader specific parameters
            'temp_model_dir': '/tmp/webots_models',
            'default_material': 'steel',
            'cleanup_on_shutdown': True,
        }],
        arguments=['--ros-args', '--log-level', 'INFO'] if not LaunchConfiguration('debug') else 
                 ['--ros-args', '--log-level', 'DEBUG']
    )
    
    # Webots simulator
    webots_process = ExecuteProcess(
        cmd=[
            'webots',
            '--mode=fast' if LaunchConfiguration('headless') == 'true' else '',
            default_world if LaunchConfiguration('world') == '' else LaunchConfiguration('world')
        ],
        output='screen',
        shell=False,
        condition=IfCondition(LaunchConfiguration('use_sim_time'))  # Only if using simulation
    )
    
    # Optional: RViz for visualization (if you have robot models)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('assembler'),
            'config',
            'assembler.rviz'
        ])],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=IfCondition('false')  # Set to 'true' if you want RViz
    )
    
    # Static transform publisher (if needed for coordinate frame alignment)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_webots_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'webots_world'],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Delayed start for model loader (give Webots time to initialize)
    delayed_model_loader = TimerAction(
        period=3.0,  # 3 second delay
        actions=[model_loader_node]
    )
    
    return LaunchDescription([
        # Launch arguments
        world_file_arg,
        use_sim_time_arg,
        headless_arg,
        debug_arg,
        
        # Core nodes
        webots_process,           # Start Webots first
        assembler_node,           # Your main assembler node
        delayed_model_loader,     # Model loader with delay
        static_tf_node,          # Coordinate transforms
        
        # Optional nodes
        # rviz_node,              # Uncomment if you want RViz
    ])
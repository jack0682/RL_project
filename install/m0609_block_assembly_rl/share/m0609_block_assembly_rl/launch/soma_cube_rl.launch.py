#!/usr/bin/env python3
"""
Launch file for SOMA Cube RL Training System
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'max_episodes',
            default_value='1000',
            description='Maximum training episodes'
        ),
        
        DeclareLaunchArgument(
            'virtual_mode',
            default_value='true',
            description='Use virtual robot mode'
        ),
        
        DeclareLaunchArgument(
            'robot_id',
            default_value='dsr01',
            description='Robot identifier'
        ),
        
        DeclareLaunchArgument(
            'log_dir',
            default_value='logs',
            description='Directory for training logs'
        ),
        
        DeclareLaunchArgument(
            'model_dir',
            default_value='models',
            description='Directory for saving models'
        ),
        
        # Log launch info
        LogInfo(
            msg=['Launching SOMA Cube RL Training System with:']
        ),
        LogInfo(
            msg=['  Max Episodes: ', LaunchConfiguration('max_episodes')]
        ),
        LogInfo(
            msg=['  Virtual Mode: ', LaunchConfiguration('virtual_mode')]
        ),
        LogInfo(
            msg=['  Robot ID: ', LaunchConfiguration('robot_id')]
        ),
        
        # Launch the ROS2 node
        Node(
            package='m0609_block_assembly_rl',
            executable='soma_rl_node',
            name='soma_cube_rl_node',
            parameters=[{
                'max_episodes': LaunchConfiguration('max_episodes'),
                'virtual_mode': LaunchConfiguration('virtual_mode'),
                'robot_id': LaunchConfiguration('robot_id'),
                'log_dir': LaunchConfiguration('log_dir'),
                'model_dir': LaunchConfiguration('model_dir'),
            }],
            output='screen',
            emulate_tty=True,
        ),
    ])
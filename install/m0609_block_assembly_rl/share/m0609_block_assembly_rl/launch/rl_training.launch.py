#!/usr/bin/env python3
"""
ROS2 Launch file for RL Training with Metrics Monitoring
Launches training nodes and monitoring nodes together
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    """Generate launch description for RL training system"""
    
    # Declare launch arguments
    max_episodes_arg = DeclareLaunchArgument(
        'max_episodes',
        default_value='10000',
        description='Maximum number of training episodes'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='dsr01',
        description='Robot identifier'
    )
    
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='m0609',
        description='Robot model'
    )
    
    virtual_mode_arg = DeclareLaunchArgument(
        'virtual_mode',
        default_value='true',
        description='Use virtual robot mode'
    )
    
    enable_tensorboard_arg = DeclareLaunchArgument(
        'enable_tensorboard',
        default_value='true',
        description='Enable tensorboard logging'
    )
    
    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value='logs',
        description='Directory for logs'
    )
    
    model_dir_arg = DeclareLaunchArgument(
        'model_dir',
        default_value='models',
        description='Directory for model checkpoints'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start training on launch'
    )
    
    # RL Metrics Publisher Node
    metrics_publisher_node = Node(
        package='m0609_block_assembly_rl',
        executable='rl_metrics_publisher',
        name='rl_metrics_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )
    
    # RL Training Node (main training controller)
    training_node = Node(
        package='m0609_block_assembly_rl',
        executable='rl_training_node',
        name='rl_training_node',
        output='screen',
        parameters=[{
            'max_episodes': LaunchConfiguration('max_episodes'),
            'robot_id': LaunchConfiguration('robot_id'),
            'robot_model': LaunchConfiguration('robot_model'),
            'virtual_mode': LaunchConfiguration('virtual_mode'),
            'use_tensorboard': LaunchConfiguration('enable_tensorboard'),
            'log_dir': LaunchConfiguration('log_dir'),
            'model_dir': LaunchConfiguration('model_dir'),
            'use_sim_time': False,
        }]
    )
    
    # RQT console for monitoring (optional)
    rqt_console_node = ExecuteProcess(
        cmd=['rqt_console'],
        output='screen',
        condition=IfCondition(TextSubstitution(text='false'))  # Disabled by default
    )
    
    # Auto-start training command (delayed to allow nodes to initialize)
    auto_start_training = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '/rl_training/command', 
             'std_msgs/msg/String', 'data: start'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_start'))
    )
    
    # Delayed auto-start (5 seconds after launch)
    delayed_auto_start = TimerAction(
        period=5.0,
        actions=[auto_start_training],
        condition=IfCondition(LaunchConfiguration('auto_start'))
    )
    
    # Tensorboard launcher (optional)
    tensorboard_launcher = ExecuteProcess(
        cmd=['tensorboard', '--logdir', LaunchConfiguration('log_dir'), '--port', '6006'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_tensorboard'))
    )
    
    return LaunchDescription([
        # Launch arguments
        max_episodes_arg,
        robot_id_arg,
        robot_model_arg,
        virtual_mode_arg,
        enable_tensorboard_arg,
        log_dir_arg,
        model_dir_arg,
        auto_start_arg,
        
        # Core nodes
        metrics_publisher_node,
        training_node,
        
        # Optional components
        # rqt_console_node,  # Uncomment to enable RQT console
        delayed_auto_start,
        # tensorboard_launcher,  # Uncomment to auto-launch tensorboard
    ])
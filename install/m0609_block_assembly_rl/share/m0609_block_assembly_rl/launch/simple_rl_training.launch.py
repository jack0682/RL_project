#!/usr/bin/env python3
"""
Simple ROS2 Launch file for RL Training
Simplified version that works reliably
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate simple launch description for RL training"""
    
    # Simple RL Training Node
    simple_training_node = Node(
        package='m0609_block_assembly_rl',
        executable='simple_rl_training_node',
        name='simple_rl_training_node',
        output='screen'
    )
    
    # RL Metrics Publisher Node
    metrics_publisher_node = Node(
        package='m0609_block_assembly_rl',
        executable='rl_metrics_publisher',
        name='rl_metrics_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        simple_training_node,
        metrics_publisher_node,
    ])
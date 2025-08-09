#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('soma_cube_rl_bridge')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true')
    
    declare_sim_cmd = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Run in simulation mode with relaxed constraints')
    
    declare_controller_ip_cmd = DeclareLaunchArgument(
        'controller_ip',
        default_value='192.168.1.150',
        description='IP address of the Doosan controller')
    
    declare_publish_rate_hz_cmd = DeclareLaunchArgument(
        'publish_rate_hz',
        default_value='20.0',
        description='Rate for state publishing in Hz')
    
    declare_vel_limit_deg_s_cmd = DeclareLaunchArgument(
        'vel_limit_deg_s',
        default_value='50.0',
        description='Velocity limit in degrees per second')
    
    declare_acc_limit_deg_s2_cmd = DeclareLaunchArgument(
        'acc_limit_deg_s2',
        default_value='100.0',
        description='Acceleration limit in degrees per second squared')
    
    declare_max_episode_steps_cmd = DeclareLaunchArgument(
        'max_episode_steps',
        default_value='1000',
        description='Maximum steps per RL episode')
    
    declare_monitor_rate_hz_cmd = DeclareLaunchArgument(
        'monitor_rate_hz',
        default_value='10.0',
        description='Safety monitoring rate in Hz')
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim = LaunchConfiguration('sim')
    controller_ip = LaunchConfiguration('controller_ip')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    vel_limit_deg_s = LaunchConfiguration('vel_limit_deg_s')
    acc_limit_deg_s2 = LaunchConfiguration('acc_limit_deg_s2')
    max_episode_steps = LaunchConfiguration('max_episode_steps')
    monitor_rate_hz = LaunchConfiguration('monitor_rate_hz')
    
    # Define parameters
    real_config_file = os.path.join(pkg_dir, 'config', 'soma_cube_rl_real.yaml')
    sim_config_file = os.path.join(pkg_dir, 'config', 'soma_cube_rl_sim.yaml')
    
    def get_config_file(context):
        sim_val = context.launch_configurations['sim']
        if sim_val.lower() == 'true':
            return sim_config_file
        else:
            return real_config_file
    
    # Safety Monitor Node
    safety_monitor_node = Node(
        package='soma_cube_rl_bridge',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'monitor_rate_hz': monitor_rate_hz},
            {'joint_min_deg': [-170.0, -135.0, -169.0, -90.0, -135.0, -360.0]},
            {'joint_max_deg': [170.0, 135.0, 169.0, 90.0, 135.0, 360.0]},
            {'tcp_workspace_min': [-1000.0, -1000.0, 0.0]},
            {'tcp_workspace_max': [1000.0, 1000.0, 2000.0]},
        ],
        remappings=[
            ('/error', '/error'),
            ('/msg/robot_state', '/msg/robot_state'),
            ('/msg/robot_stop', '/msg/robot_stop'),
            ('/system/get_robot_state', '/system/get_robot_state'),
            ('/system/get_robot_mode', '/system/get_robot_mode'),
        ]
    )
    
    # Motion Proxy Node
    motion_proxy_node = Node(
        package='soma_cube_rl_bridge',
        executable='motion_proxy_node', 
        name='motion_proxy_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'publish_rate_hz': publish_rate_hz},
            {'vel_limit_deg_s': [50.0, 50.0, 50.0, 50.0, 50.0, 50.0]},
            {'acc_limit_deg_s2': [100.0, 100.0, 100.0, 100.0, 100.0, 100.0]},
            {'joint_min_deg': [-170.0, -135.0, -169.0, -90.0, -135.0, -360.0]},
            {'joint_max_deg': [170.0, 135.0, 169.0, 90.0, 135.0, 360.0]},
            {'action_timeout_sec': 30.0},
            {'service_wait_sec': 5.0},
        ],
        remappings=[
            ('/joint_states', '/joint_states'),
            ('/msg/current_posx', '/msg/current_posx'),
            ('/msg/robot_state', '/msg/robot_state'),
            ('/error', '/error'),
            ('/motion/move_joint', '/motion/move_joint'),
            ('/motion/move_stop', '/motion/move_stop'),
            ('/aux_control/get_current_posj', '/aux_control/get_current_posj'),
            ('/aux_control/get_current_posx', '/aux_control/get_current_posx'),
        ]
    )
    
    # RL Environment Node
    rl_env_node = Node(
        package='soma_cube_rl_bridge',
        executable='rl_env_node',
        name='rl_env_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'max_episode_steps': max_episode_steps},
            {'action_timeout_sec': 30.0},
            {'service_wait_sec': 5.0},
            {'joint_min_deg': [-170.0, -135.0, -169.0, -90.0, -135.0, -360.0]},
            {'joint_max_deg': [170.0, 135.0, 169.0, 90.0, 135.0, 360.0]},
            {'vel_limit_deg_s': [80.0, 80.0, 80.0, 100.0, 100.0, 120.0]},
            {'acc_limit_deg_s2': [150.0, 150.0, 150.0, 200.0, 200.0, 250.0]},
            {'reward_weights.grasp_success': 100.0},
            {'reward_weights.assembly_alignment': 50.0},
            {'reward_weights.collision_penalty': -100.0},
            {'reward_weights.time_penalty': -0.1},
            {'reward_weights.distance_penalty': -1.0},
            {'reward_weights.velocity_penalty': -0.01},
        ],
        remappings=[
            ('soma_cube_rl_bridge/joint_states', 'soma_cube_rl_bridge/joint_states'),
            ('soma_cube_rl_bridge/tcp_pose', 'soma_cube_rl_bridge/tcp_pose'),
            ('/msg/tool_force', '/msg/tool_force'),
            ('soma_cube_rl_bridge/move_joint', 'soma_cube_rl_bridge/move_joint'),
            ('/aux_control/get_current_posj', '/aux_control/get_current_posj'),
            ('/aux_control/get_current_posx', '/aux_control/get_current_posx'),
            ('/aux_control/get_tool_force', '/aux_control/get_tool_force'),
            ('safety/get_state', 'safety/get_state'),
        ]
    )
    
    # Simulation-specific parameter adjustments
    def add_sim_parameters(context):
        nodes = []
        sim_val = context.launch_configurations['sim']
        if sim_val.lower() == 'true':
            # Add simulation-specific nodes or parameter overrides
            pass
        return nodes
    
    sim_nodes = OpaqueFunction(function=add_sim_parameters)
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time_cmd,
        declare_sim_cmd,
        declare_controller_ip_cmd,
        declare_publish_rate_hz_cmd,
        declare_vel_limit_deg_s_cmd,
        declare_acc_limit_deg_s2_cmd,
        declare_max_episode_steps_cmd,
        declare_monitor_rate_hz_cmd,
        
        # Core nodes
        safety_monitor_node,
        motion_proxy_node,
        rl_env_node,
        
        # Simulation-specific additions
        sim_nodes,
    ])
#!/usr/bin/env python3
"""
SomaCube RL Training Package

This package provides reinforcement learning training capabilities
for the SomaCube assembly task using a Doosan M0609 robot.

Components:
- rl_client: ROS2 client interface for RL environment
- sac_trainer: Soft Actor-Critic training implementation
- train_somacube: Main training script with CLI
- demo_training: Demo script for system verification

Usage:
    ros2 run soma_cube_rl_training demo_training
    ros2 run soma_cube_rl_training train_somacube --mode check
    ros2 run soma_cube_rl_training train_somacube --mode train
"""

__version__ = "1.0.0"
__author__ = "ROS2 User"
__license__ = "MIT"

# Import core modules without external dependencies
try:
    from .rl_client import SomaCubeRLClient
except ImportError:
    pass

# Import training modules only if dependencies are available
try:
    from .sac_trainer import SACTrainer, SomaCubeGymEnv
except ImportError:
    pass
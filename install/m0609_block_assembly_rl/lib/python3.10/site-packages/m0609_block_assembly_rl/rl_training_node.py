#!/usr/bin/env python3
"""
ROS2 Node for RL Training with Enhanced Logging and Metrics
Proper ROS2 integration for SOMA Cube Assembly RL training
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import std_msgs.msg
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import JointState
import json
import os
import sys
import time
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass, asdict
import numpy as np

try:
    from .enhanced_training_logger import EnhancedTrainingLogger, TrainingMetrics
    from .fixed_train import TrainingConfig, train_soma_cube_assembly
    from .fixed_ppo_agent import M0609PPOAgent, OptimizedPPOConfig
except ImportError:
    # Fallback for direct execution
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from enhanced_training_logger import EnhancedTrainingLogger, TrainingMetrics
    from fixed_train import TrainingConfig, train_soma_cube_assembly
    from fixed_ppo_agent import M0609PPOAgent, OptimizedPPOConfig

@dataclass
class RLTrainingStatus:
    """Status message for RL training"""
    is_training: bool = False
    current_episode: int = 0
    total_episodes: int = 0
    success_rate: float = 0.0
    average_reward: float = 0.0
    training_time: float = 0.0
    best_success_rate: float = 0.0
    status_message: str = "Ready"

class RLTrainingNode(Node):
    """
    ROS2 Node for RL Training with Enhanced Metrics and Logging
    
    Features:
    - ROS2 service interface for starting/stopping training
    - Real-time status publishing
    - Parameter-based configuration
    - Comprehensive logging integration
    - ROS2 standard message publishing
    """
    
    def __init__(self):
        super().__init__('rl_training_node')
        
        # Declare ROS2 parameters
        self.declare_ros2_parameters()
        
        # Initialize training state
        self.training_status = RLTrainingStatus()
        self.training_thread = None
        self.training_logger = None
        self.agent = None
        self.training_config = None
        
        # Setup QoS profiles
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.status_publisher = self.create_publisher(
            std_msgs.msg.String, 
            'rl_training/status', 
            qos_profile
        )
        
        self.metrics_publisher = self.create_publisher(
            std_msgs.msg.String,
            'rl_training/metrics',
            qos_profile
        )
        
        # Subscribers
        self.command_subscriber = self.create_subscription(
            std_msgs.msg.String,
            'rl_training/command',
            self.command_callback,
            qos_profile
        )
        
        # Timers
        self.status_timer = self.create_timer(5.0, self.publish_status)  # 5 second status updates
        self.metrics_timer = self.create_timer(30.0, self.publish_metrics)  # 30 second metric updates
        
        # Setup logging
        self.setup_logging()
        
        self.get_logger().info("RL Training Node initialized")
        self.get_logger().info("Available commands: 'start', 'stop', 'pause', 'resume', 'status'")
        self.get_logger().info("Publishers: /rl_training/status, /rl_training/metrics")
        self.get_logger().info("Subscribers: /rl_training/command")
    
    def declare_ros2_parameters(self):
        """Declare ROS2 parameters for training configuration"""
        
        # Training parameters
        self.declare_parameter('max_episodes', 10000)
        self.declare_parameter('max_steps_per_episode', 50)
        self.declare_parameter('eval_interval', 500)
        self.declare_parameter('save_interval', 1000)
        self.declare_parameter('log_interval', 100)
        
        # Robot parameters
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('robot_model', 'm0609')
        self.declare_parameter('virtual_mode', True)
        
        # Performance targets
        self.declare_parameter('target_success_rate', 0.85)
        self.declare_parameter('target_steps_per_success', 20)
        
        # Logging parameters
        self.declare_parameter('log_dir', 'logs')
        self.declare_parameter('model_dir', 'models')
        self.declare_parameter('use_tensorboard', True)
        
        # RL parameters
        self.declare_parameter('learning_rate', 1e-4)
        self.declare_parameter('batch_size', 128)
        self.declare_parameter('ppo_epochs', 4)
        self.declare_parameter('device', 'auto')
    
    def setup_logging(self):
        """Setup ROS2 compatible logging"""
        self.get_logger().info("Setting up training logging system...")
        
        # Create training config from ROS2 parameters
        self.training_config = TrainingConfig()
        self.update_config_from_parameters()
        
        self.get_logger().info(f"Training configuration:")
        self.get_logger().info(f"  Max Episodes: {self.training_config.max_episodes}")
        self.get_logger().info(f"  Device: {self.training_config.device}")
        self.get_logger().info(f"  Robot: {self.training_config.robot_model} ({self.training_config.robot_id})")
        self.get_logger().info(f"  Mode: {'Virtual' if self.training_config.virtual_mode else 'Real Robot'}")
    
    def update_config_from_parameters(self):
        """Update training config from ROS2 parameters"""
        
        self.training_config.max_episodes = self.get_parameter('max_episodes').get_parameter_value().integer_value
        self.training_config.max_steps_per_episode = self.get_parameter('max_steps_per_episode').get_parameter_value().integer_value
        self.training_config.eval_interval = self.get_parameter('eval_interval').get_parameter_value().integer_value
        self.training_config.save_interval = self.get_parameter('save_interval').get_parameter_value().integer_value
        self.training_config.log_interval = self.get_parameter('log_interval').get_parameter_value().integer_value
        
        self.training_config.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        self.training_config.robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        self.training_config.virtual_mode = self.get_parameter('virtual_mode').get_parameter_value().bool_value
        
        self.training_config.target_success_rate = self.get_parameter('target_success_rate').get_parameter_value().double_value
        self.training_config.target_steps_per_success = self.get_parameter('target_steps_per_success').get_parameter_value().integer_value
        
        self.training_config.log_dir = self.get_parameter('log_dir').get_parameter_value().string_value
        self.training_config.model_dir = self.get_parameter('model_dir').get_parameter_value().string_value
        self.training_config.use_tensorboard = self.get_parameter('use_tensorboard').get_parameter_value().bool_value
        
        # Device handling
        device_param = self.get_parameter('device').get_parameter_value().string_value
        if device_param == 'auto':
            self.training_config.device = None  # Will be auto-detected
        else:
            self.training_config.device = device_param
    
    def command_callback(self, msg):
        """Handle training commands"""
        command = msg.data.lower().strip()
        
        self.get_logger().info(f"Received command: {command}")
        
        if command == 'start':
            self.start_training()
        elif command == 'stop':
            self.stop_training()
        elif command == 'pause':
            self.pause_training()
        elif command == 'resume':
            self.resume_training()
        elif command == 'status':
            self.get_training_status()
        else:
            self.get_logger().warn(f"Unknown command: {command}")
            self.training_status.status_message = f"Unknown command: {command}"
    
    def start_training(self):
        """Start RL training in a separate thread"""
        
        if self.training_status.is_training:
            self.get_logger().warn("Training is already running")
            return
        
        self.get_logger().info("Starting RL training...")
        
        # Update config from current parameters
        self.update_config_from_parameters()
        
        # Create enhanced logger
        self.training_logger = EnhancedTrainingLogger(
            experiment_name=f"soma_cube_rl_ros2",
            log_dir=self.training_config.log_dir,
            config=self.training_config.__dict__,
            enable_tensorboard=self.training_config.use_tensorboard
        )
        
        # Update status
        self.training_status = RLTrainingStatus(
            is_training=True,
            current_episode=0,
            total_episodes=self.training_config.max_episodes,
            status_message="Training started"
        )
        
        # Start training thread
        self.training_thread = threading.Thread(target=self.training_worker)
        self.training_thread.daemon = True
        self.training_thread.start()
        
        self.get_logger().info("Training thread started")
    
    def training_worker(self):
        """Training worker function that runs in separate thread"""
        
        try:
            self.get_logger().info("Training worker started")
            
            # Create modified training function that updates ROS2 status
            self.run_training_with_ros2_integration()
            
        except Exception as e:
            self.get_logger().error(f"Training error: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            self.training_status.status_message = f"Training failed: {str(e)}"
        
        finally:
            self.training_status.is_training = False
            self.training_status.status_message = "Training completed"
            self.get_logger().info("Training worker finished")
    
    def run_training_with_ros2_integration(self):
        """Run training with ROS2 status updates"""
        
        # Import training components
        try:
            from .soma_cube_environment import SOMACubeAssemblyEnv
            from .fixed_ppo_agent import create_ppo_agent
        except ImportError:
            from soma_cube_environment import SOMACubeAssemblyEnv
            from fixed_ppo_agent import create_ppo_agent
        
        # Create environment
        env = SOMACubeAssemblyEnv(
            robot_id=self.training_config.robot_id,
            robot_model=self.training_config.robot_model,
            virtual_mode=self.training_config.virtual_mode,
            unit_cube_size=30.0,
            render_mode=None
        )
        
        # Create PPO agent
        ppo_config = OptimizedPPOConfig(
            learning_rate=self.get_parameter('learning_rate').get_parameter_value().double_value,
            batch_size=self.get_parameter('batch_size').get_parameter_value().integer_value,
            update_epochs=self.get_parameter('ppo_epochs').get_parameter_value().integer_value
        )
        
        self.agent = create_ppo_agent(env, ppo_config, self.training_config.device)
        
        self.get_logger().info("Starting training loop...")
        
        # Training loop with ROS2 integration
        episode_rewards = []
        episode_successes = []
        
        for episode in range(1, self.training_config.max_episodes + 1):
            
            if not self.training_status.is_training:  # Check for stop command
                break
            
            # Update episode status
            self.training_status.current_episode = episode
            
            # Reset environment
            obs, info = env.reset()
            episode_reward = 0
            steps = 0
            episode_start_time = time.time()
            
            # Episode loop
            while steps < self.training_config.max_steps_per_episode:
                action, action_info = self.agent.select_action(obs)
                next_obs, reward, terminated, truncated, step_info = env.step(action)
                
                self.agent.store_experience(obs, action, reward, terminated or truncated, action_info)
                
                obs = next_obs
                episode_reward += reward
                steps += 1
                
                if terminated or truncated:
                    break
            
            # Update agent
            training_stats = self.agent.update()
            episode_time = time.time() - episode_start_time
            
            # Log episode
            success = step_info.get('assembly_complete', False)
            pieces_placed = step_info.get('pieces_placed', 0)
            
            episode_metrics = TrainingMetrics(
                episode=episode,
                timestamp=time.time(),
                success=success,
                total_reward=episode_reward,
                steps_taken=steps,
                pieces_placed=pieces_placed,
                completion_rate=pieces_placed / 7.0,
                total_episode_time=episode_time,
                curriculum_stage="BASIC_PLACEMENT"
            )
            
            self.training_logger.log_episode(episode_metrics, training_stats)
            
            # Update ROS2 status
            episode_rewards.append(episode_reward)
            episode_successes.append(success)
            
            # Calculate rolling statistics
            if len(episode_rewards) >= 100:
                recent_rewards = episode_rewards[-100:]
                recent_successes = episode_successes[-100:]
            else:
                recent_rewards = episode_rewards
                recent_successes = episode_successes
            
            self.training_status.success_rate = np.mean(recent_successes) if recent_successes else 0.0
            self.training_status.average_reward = np.mean(recent_rewards) if recent_rewards else 0.0
            self.training_status.best_success_rate = max(self.training_status.best_success_rate, self.training_status.success_rate)
            self.training_status.status_message = f"Episode {episode}: SR={self.training_status.success_rate:.2%}, Reward={episode_reward:.1f}"
            
            # ROS2 logging
            if episode % self.training_config.log_interval == 0:
                self.get_logger().info(
                    f"Episode {episode}: Success={success}, Reward={episode_reward:.2f}, "
                    f"Steps={steps}, Rolling SR={self.training_status.success_rate:.2%}"
                )
            
            # Save model periodically
            if episode % self.training_config.save_interval == 0:
                model_dir = os.path.join(self.training_config.model_dir, "ros2_checkpoints")
                os.makedirs(model_dir, exist_ok=True)
                model_path = os.path.join(model_dir, f"model_ep_{episode}.pth")
                self.agent.save_model(model_path)
                self.get_logger().info(f"Model saved: {model_path}")
        
        # Training completed
        self.training_status.is_training = False
        self.training_status.status_message = "Training completed successfully"
        
        # Final model save
        final_model_path = os.path.join(self.training_config.model_dir, "ros2_final_model.pth")
        os.makedirs(os.path.dirname(final_model_path), exist_ok=True)
        self.agent.save_model(final_model_path)
        
        # Close logger
        if self.training_logger:
            summary = self.training_logger.close()
            if summary and 'experiment_info' in summary:
                info = summary['experiment_info']
                self.get_logger().info(f"Training completed: {info['total_episodes']} episodes, {info['success_rate']:.2%} success rate")
    
    def stop_training(self):
        """Stop training"""
        if not self.training_status.is_training:
            self.get_logger().warn("Training is not running")
            return
        
        self.get_logger().info("Stopping training...")
        self.training_status.is_training = False
        self.training_status.status_message = "Training stopped by user"
        
        if self.training_thread and self.training_thread.is_alive():
            # Wait for thread to finish
            self.training_thread.join(timeout=10.0)
    
    def pause_training(self):
        """Pause training (not implemented - would require more complex state management)"""
        self.get_logger().warn("Pause/resume functionality not yet implemented")
        self.training_status.status_message = "Pause/resume not implemented"
    
    def resume_training(self):
        """Resume training (not implemented)"""
        self.get_logger().warn("Pause/resume functionality not yet implemented")
        self.training_status.status_message = "Pause/resume not implemented"
    
    def get_training_status(self):
        """Get current training status"""
        status_dict = asdict(self.training_status)
        self.get_logger().info(f"Training Status: {json.dumps(status_dict, indent=2)}")
        return status_dict
    
    def publish_status(self):
        """Publish training status to ROS2 topic"""
        status_dict = asdict(self.training_status)
        status_msg = std_msgs.msg.String()
        status_msg.data = json.dumps(status_dict)
        self.status_publisher.publish(status_msg)
    
    def publish_metrics(self):
        """Publish training metrics to ROS2 topic"""
        if not self.training_status.is_training:
            return
        
        metrics_dict = {
            'episode': self.training_status.current_episode,
            'success_rate': self.training_status.success_rate,
            'average_reward': self.training_status.average_reward,
            'best_success_rate': self.training_status.best_success_rate,
            'training_time': self.training_status.training_time,
            'timestamp': time.time()
        }
        
        metrics_msg = std_msgs.msg.String()
        metrics_msg.data = json.dumps(metrics_dict)
        self.metrics_publisher.publish(metrics_msg)

def main(args=None):
    """Main function for ROS2 node"""
    
    rclpy.init(args=args)
    
    try:
        node = RLTrainingNode()
        
        # Log node ready
        node.get_logger().info("=" * 60)
        node.get_logger().info("ROS2 RL TRAINING NODE READY")
        node.get_logger().info("=" * 60)
        node.get_logger().info("Send commands to: /rl_training/command")
        node.get_logger().info("Available commands: start, stop, status")
        node.get_logger().info("Monitor status: /rl_training/status")
        node.get_logger().info("Monitor metrics: /rl_training/metrics")
        node.get_logger().info("=" * 60)
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Node error: {e}")
        import traceback
        node.get_logger().error(f"Traceback: {traceback.format_exc()}")
    finally:
        if 'node' in locals():
            # Stop training if running
            if node.training_status.is_training:
                node.stop_training()
            node.destroy_node()
        
        rclpy.shutdown()

if __name__ == '__main__':
    main()
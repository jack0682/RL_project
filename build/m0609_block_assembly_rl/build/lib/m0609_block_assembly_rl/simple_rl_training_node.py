#!/usr/bin/env python3
"""
Simple ROS2 RL Training Node
Simplified version for reliable operation
"""

import rclpy
from rclpy.node import Node
import std_msgs.msg
import json
import threading
import time
import os
import sys
from pathlib import Path

# Add module path
try:
    from .enhanced_training_logger import EnhancedTrainingLogger, TrainingMetrics
    from .fixed_train import TrainingConfig
except ImportError:
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from enhanced_training_logger import EnhancedTrainingLogger, TrainingMetrics
    from fixed_train import TrainingConfig

class SimpleRLTrainingNode(Node):
    """Simple ROS2 node for RL training control"""
    
    def __init__(self):
        super().__init__('simple_rl_training_node')
        
        # Training state
        self.is_training = False
        self.current_episode = 0
        self.total_episodes = 10000
        self.training_thread = None
        self.logger_instance = None
        
        # Publishers
        self.status_pub = self.create_publisher(
            std_msgs.msg.String, 
            'rl_training/status', 
            10
        )
        
        # Subscribers  
        self.command_sub = self.create_subscription(
            std_msgs.msg.String,
            'rl_training/command',
            self.command_callback,
            10
        )
        
        # Timer for status updates
        self.status_timer = self.create_timer(5.0, self.publish_status)
        
        self.get_logger().info("Simple RL Training Node ready")
        self.get_logger().info("Send commands to: /rl_training/command")
        self.get_logger().info("Monitor status: /rl_training/status")
        self.get_logger().info("Commands: 'start', 'stop', 'status'")
    
    def command_callback(self, msg):
        """Handle commands"""
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received command: {command}")
        
        if command == 'start':
            self.start_training()
        elif command == 'stop':
            self.stop_training()
        elif command == 'status':
            self.publish_status()
        else:
            self.get_logger().warn(f"Unknown command: {command}")
    
    def start_training(self):
        """Start training"""
        if self.is_training:
            self.get_logger().warn("Training already running")
            return
        
        self.get_logger().info("Starting RL training...")
        self.is_training = True
        self.current_episode = 0
        
        # Start training thread
        self.training_thread = threading.Thread(target=self.training_worker)
        self.training_thread.daemon = True
        self.training_thread.start()
    
    def stop_training(self):
        """Stop training"""
        if not self.is_training:
            self.get_logger().warn("Training not running")
            return
        
        self.get_logger().info("Stopping training...")
        self.is_training = False
        
        if self.training_thread and self.training_thread.is_alive():
            self.training_thread.join(timeout=5.0)
    
    def training_worker(self):
        """Training worker function"""
        try:
            self.get_logger().info("Training worker started")
            
            # Create enhanced logger
            self.logger_instance = EnhancedTrainingLogger(
                experiment_name="simple_ros2_training",
                log_dir="logs",
                enable_ros2=True,
                enable_tensorboard=True
            )
            
            # Simulate training loop
            for episode in range(1, self.total_episodes + 1):
                if not self.is_training:
                    break
                
                self.current_episode = episode
                
                # Simulate episode
                import numpy as np
                success = np.random.random() > 0.7  # 30% success rate initially
                reward = np.random.uniform(10, 50) if success else np.random.uniform(-20, 5)
                steps = np.random.randint(15, 35)
                pieces = np.random.randint(0, 7)
                
                # Create metrics
                metrics = TrainingMetrics(
                    episode=episode,
                    timestamp=time.time(),
                    success=success,
                    total_reward=reward,
                    steps_taken=steps,
                    pieces_placed=pieces,
                    completion_rate=pieces / 7.0,
                    curriculum_stage="BASIC_PLACEMENT"
                )
                
                # Log episode
                self.logger_instance.log_episode(metrics)
                
                # Log progress
                if episode % 10 == 0:
                    self.get_logger().info(
                        f"Episode {episode}: Success={success}, "
                        f"Reward={reward:.2f}, Steps={steps}"
                    )
                
                # Small delay
                time.sleep(0.1)
            
            self.get_logger().info("Training completed")
            
        except Exception as e:
            self.get_logger().error(f"Training error: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
        
        finally:
            self.is_training = False
            if self.logger_instance:
                self.logger_instance.close()
    
    def publish_status(self):
        """Publish training status"""
        status = {
            'is_training': self.is_training,
            'current_episode': self.current_episode,
            'total_episodes': self.total_episodes,
            'timestamp': time.time()
        }
        
        msg = std_msgs.msg.String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = SimpleRLTrainingNode()
        
        node.get_logger().info("=" * 50)
        node.get_logger().info("SIMPLE RL TRAINING NODE STARTED")  
        node.get_logger().info("=" * 50)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted")
    except Exception as e:
        print(f"Node error: {e}")
        import traceback
        print(f"Traceback: {traceback.format_exc()}")
    finally:
        if 'node' in locals():
            if node.is_training:
                node.stop_training()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
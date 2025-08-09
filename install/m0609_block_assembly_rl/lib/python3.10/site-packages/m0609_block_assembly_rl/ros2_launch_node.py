#!/usr/bin/env python3
"""
ROS2 Launch Node for SOMA Cube Assembly RL Training
Provides ROS2 integration for the RL training system
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
import json
import threading
import sys
import os

# Import our fixed training modules
from .fixed_train import train_soma_cube_assembly, TrainingConfig
from .validation_test import main as validation_main

class SOMACubeRLNode(Node):
    """ROS2 Node for SOMA Cube RL Training"""

    def __init__(self):
        super().__init__('soma_cube_rl_node')
        
        self.get_logger().info('SOMA Cube RL Node started')
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/soma_cube/status', 10)
        self.progress_pub = self.create_publisher(Float32, '/soma_cube/progress', 10)
        
        # Subscribers
        self.command_sub = self.create_subscription(
            String, '/soma_cube/command', self.command_callback, 10
        )
        
        # Training state
        self.training_active = False
        self.training_thread = None
        
        # Publish initial status
        self.publish_status("Node initialized - Ready for commands")
        
        # Declare parameters
        self.declare_parameter('max_episodes', 1000)
        self.declare_parameter('virtual_mode', True)
        self.declare_parameter('robot_id', 'dsr01')
        self.declare_parameter('log_dir', 'logs')
        self.declare_parameter('model_dir', 'models')
    
    def publish_status(self, message: str):
        """Publish status message"""
        msg = String()
        msg.data = f"[{self.get_clock().now().to_msg().sec}] {message}"
        self.status_pub.publish(msg)
        self.get_logger().info(message)
    
    def command_callback(self, msg):
        """Handle incoming commands"""
        command = msg.data.lower().strip()
        
        self.get_logger().info(f'Received command: {command}')
        
        if command == 'start_training':
            self.start_training()
        elif command == 'stop_training':
            self.stop_training()
        elif command == 'run_validation':
            self.run_validation()
        elif command == 'status':
            self.report_status()
        else:
            self.publish_status(f"Unknown command: {command}")
    
    def start_training(self):
        """Start RL training in separate thread"""
        if self.training_active:
            self.publish_status("Training already active!")
            return
        
        try:
            # Get parameters
            max_episodes = self.get_parameter('max_episodes').value
            virtual_mode = self.get_parameter('virtual_mode').value
            robot_id = self.get_parameter('robot_id').value
            log_dir = self.get_parameter('log_dir').value
            model_dir = self.get_parameter('model_dir').value
            
            # Create training config
            config = TrainingConfig()
            config.max_episodes = max_episodes
            config.virtual_mode = virtual_mode
            config.robot_id = robot_id
            config.log_dir = log_dir
            config.model_dir = model_dir
            
            self.publish_status(f"Starting training: {max_episodes} episodes")
            
            # Start training in separate thread
            self.training_active = True
            self.training_thread = threading.Thread(
                target=self._training_worker, 
                args=(config,)
            )
            self.training_thread.start()
            
        except Exception as e:
            self.publish_status(f"Failed to start training: {e}")
            self.training_active = False
    
    def _training_worker(self, config: TrainingConfig):
        """Training worker thread"""
        try:
            self.publish_status("Training started...")
            
            # Run training
            agent = train_soma_cube_assembly(config)
            
            self.publish_status("Training completed successfully!")
            
        except Exception as e:
            self.publish_status(f"Training failed: {e}")
        
        finally:
            self.training_active = False
    
    def stop_training(self):
        """Stop active training"""
        if not self.training_active:
            self.publish_status("No active training to stop")
            return
        
        self.publish_status("Training stop requested...")
        # Note: Graceful shutdown would require modifying training loop
        self.training_active = False
    
    def run_validation(self):
        """Run validation tests"""
        self.publish_status("Running validation tests...")
        
        try:
            # Run validation in separate thread to avoid blocking
            validation_thread = threading.Thread(target=self._validation_worker)
            validation_thread.start()
            
        except Exception as e:
            self.publish_status(f"Validation failed: {e}")
    
    def _validation_worker(self):
        """Validation worker thread"""
        try:
            # Capture validation output
            import io
            import contextlib
            
            # Redirect stdout to capture validation results
            old_stdout = sys.stdout
            sys.stdout = io.StringIO()
            
            # Run validation
            success = validation_main()
            
            # Get output
            output = sys.stdout.getvalue()
            sys.stdout = old_stdout
            
            # Parse results
            if "100.0%" in output:
                self.publish_status("Validation: ALL TESTS PASSED (100%)")
            elif "PASS" in output:
                self.publish_status("Validation: Tests completed with some passes")
            else:
                self.publish_status("Validation: Some tests failed")
            
        except Exception as e:
            sys.stdout = old_stdout
            self.publish_status(f"Validation error: {e}")
    
    def report_status(self):
        """Report current node status"""
        status = "TRAINING" if self.training_active else "IDLE"
        self.publish_status(f"Node status: {status}")

def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = SOMACubeRLNode()
        
        node.get_logger().info('SOMA Cube RL Node ready')
        node.get_logger().info('Send commands to: /soma_cube/command')
        node.get_logger().info('Available commands: start_training, stop_training, run_validation, status')
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
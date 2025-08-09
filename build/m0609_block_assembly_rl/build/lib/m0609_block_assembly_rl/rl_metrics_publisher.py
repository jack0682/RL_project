#!/usr/bin/env python3
"""
ROS2 Metrics Publisher Node for RL Training
Publishes training metrics and logs to ROS2 topics for monitoring
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import std_msgs.msg
from geometry_msgs.msg import Vector3
import json
import time
from typing import Dict, Any, Optional
from collections import deque
import numpy as np

class RLMetricsPublisher(Node):
    """
    ROS2 Node for publishing RL training metrics
    
    Topics Published:
    - /rl_metrics/episode_stats - Episode-level statistics
    - /rl_metrics/training_progress - Training progress indicators
    - /rl_metrics/performance - Performance metrics
    """
    
    def __init__(self):
        super().__init__('rl_metrics_publisher')
        
        # QoS profile for reliable metrics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100  # Keep more metrics for analysis
        )
        
        # Publishers
        self.episode_stats_pub = self.create_publisher(
            std_msgs.msg.String,
            'rl_metrics/episode_stats',
            qos_profile
        )
        
        self.training_progress_pub = self.create_publisher(
            std_msgs.msg.String,
            'rl_metrics/training_progress', 
            qos_profile
        )
        
        self.performance_pub = self.create_publisher(
            Vector3,  # x=success_rate, y=avg_reward, z=avg_steps
            'rl_metrics/performance',
            qos_profile
        )
        
        # Internal state
        self.episode_count = 0
        self.metrics_buffer = deque(maxlen=1000)
        self.rolling_stats = deque(maxlen=100)  # Last 100 episodes
        
        self.get_logger().info("RL Metrics Publisher Node initialized")
        self.get_logger().info("Publishing to:")
        self.get_logger().info("  - /rl_metrics/episode_stats")
        self.get_logger().info("  - /rl_metrics/training_progress")
        self.get_logger().info("  - /rl_metrics/performance")
    
    def publish_episode_metrics(self, episode_data: Dict[str, Any]):
        """
        Publish episode-level metrics
        
        Args:
            episode_data: Dictionary containing episode metrics
                Required keys: episode, success, reward, steps, pieces_placed
                Optional keys: policy_loss, value_loss, entropy, completion_time
        """
        
        # Validate required fields
        required_fields = ['episode', 'success', 'reward', 'steps', 'pieces_placed']
        for field in required_fields:
            if field not in episode_data:
                self.get_logger().error(f"Missing required field: {field}")
                return
        
        # Add timestamp
        episode_data['timestamp'] = time.time()
        episode_data['node_time'] = self.get_clock().now().to_msg()
        
        # Update internal counters
        self.episode_count = episode_data['episode']
        self.metrics_buffer.append(episode_data)
        self.rolling_stats.append(episode_data)
        
        # Publish episode stats
        episode_msg = std_msgs.msg.String()
        episode_msg.data = json.dumps(episode_data, default=str)
        self.episode_stats_pub.publish(episode_msg)
        
        # Calculate and publish training progress
        self.publish_training_progress()
        
        # Publish performance vector
        self.publish_performance_vector()
        
        # Log to ROS2 console
        if episode_data['episode'] % 10 == 0:  # Log every 10 episodes
            self.get_logger().info(
                f"Episode {episode_data['episode']}: "
                f"Success={episode_data['success']}, "
                f"Reward={episode_data['reward']:.2f}, "
                f"Steps={episode_data['steps']}, "
                f"Pieces={episode_data['pieces_placed']}/7"
            )
    
    def publish_training_progress(self):
        """Publish training progress statistics"""
        
        if len(self.rolling_stats) < 2:
            return
        
        # Calculate statistics
        recent_data = list(self.rolling_stats)
        success_rate = np.mean([d['success'] for d in recent_data])
        avg_reward = np.mean([d['reward'] for d in recent_data])
        avg_steps = np.mean([d['steps'] for d in recent_data])
        avg_completion = np.mean([d['pieces_placed'] / 7.0 for d in recent_data])
        
        # Calculate trends (last 20 vs previous 20)
        if len(recent_data) >= 40:
            early_sr = np.mean([d['success'] for d in recent_data[:20]])
            late_sr = np.mean([d['success'] for d in recent_data[-20:]])
            trend = "improving" if late_sr > early_sr + 0.05 else "declining" if late_sr < early_sr - 0.05 else "stable"
        else:
            trend = "insufficient_data"
        
        progress_data = {
            'episode': self.episode_count,
            'total_episodes_logged': len(self.metrics_buffer),
            'rolling_window_size': len(self.rolling_stats),
            'success_rate': float(success_rate),
            'average_reward': float(avg_reward),
            'average_steps': float(avg_steps), 
            'average_completion_rate': float(avg_completion),
            'performance_trend': trend,
            'timestamp': time.time(),
            'node_time': self.get_clock().now().to_msg()
        }
        
        # Publish training progress
        progress_msg = std_msgs.msg.String()
        progress_msg.data = json.dumps(progress_data, default=str)
        self.training_progress_pub.publish(progress_msg)
    
    def publish_performance_vector(self):
        """Publish performance as Vector3 for easy visualization"""
        
        if len(self.rolling_stats) == 0:
            return
        
        # Calculate current performance metrics
        recent_data = list(self.rolling_stats)
        success_rate = np.mean([d['success'] for d in recent_data])
        avg_reward = np.mean([d['reward'] for d in recent_data])  
        avg_steps = np.mean([d['steps'] for d in recent_data])
        
        # Normalize avg_steps (invert so higher is better, scale to 0-1)
        normalized_steps = max(0, 1 - (avg_steps - 10) / 40)  # Assume 10-50 step range
        
        # Create Vector3 message
        perf_msg = Vector3()
        perf_msg.x = float(success_rate)  # 0-1 range
        perf_msg.y = float(max(-1, min(1, avg_reward / 50)))  # Normalize to -1 to 1
        perf_msg.z = float(normalized_steps)  # 0-1 range (higher is better)
        
        self.performance_pub.publish(perf_msg)
    
    def publish_training_summary(self, summary_data: Dict[str, Any]):
        """
        Publish training session summary
        
        Args:
            summary_data: Training summary statistics
        """
        
        summary_data['timestamp'] = time.time()
        summary_data['node_time'] = self.get_clock().now().to_msg()
        summary_data['message_type'] = 'training_summary'
        
        # Publish to training progress topic
        summary_msg = std_msgs.msg.String()
        summary_msg.data = json.dumps(summary_data, default=str)
        self.training_progress_pub.publish(summary_msg)
        
        self.get_logger().info("Training summary published")
        
        if 'experiment_info' in summary_data:
            info = summary_data['experiment_info']
            self.get_logger().info(
                f"Training Summary - Episodes: {info.get('total_episodes', 0)}, "
                f"Success Rate: {info.get('success_rate', 0):.2%}"
            )
    
    def get_current_stats(self) -> Dict[str, Any]:
        """Get current training statistics"""
        
        if len(self.rolling_stats) == 0:
            return {
                'episode_count': self.episode_count,
                'total_logged': len(self.metrics_buffer),
                'current_stats': 'no_data'
            }
        
        recent_data = list(self.rolling_stats)
        
        return {
            'episode_count': self.episode_count,
            'total_logged': len(self.metrics_buffer),
            'rolling_window': len(self.rolling_stats),
            'success_rate': float(np.mean([d['success'] for d in recent_data])),
            'average_reward': float(np.mean([d['reward'] for d in recent_data])),
            'average_steps': float(np.mean([d['steps'] for d in recent_data])),
            'average_completion': float(np.mean([d['pieces_placed'] / 7.0 for d in recent_data])),
            'latest_episode': recent_data[-1] if recent_data else None
        }

class RLMetricsIntegration:
    """
    Helper class to integrate RL metrics publishing with existing training code
    """
    
    def __init__(self, node_name: str = 'rl_metrics_integration'):
        self.node_name = node_name
        self.publisher_node = None
        self.initialized = False
    
    def initialize(self):
        """Initialize ROS2 if not already done"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.publisher_node = RLMetricsPublisher()
            self.initialized = True
            return True
            
        except Exception as e:
            print(f"Failed to initialize ROS2 metrics: {e}")
            return False
    
    def publish_episode_metrics(self, episode_data: Dict[str, Any]):
        """Publish episode metrics (safe wrapper)"""
        if not self.initialized:
            if not self.initialize():
                return
        
        try:
            self.publisher_node.publish_episode_metrics(episode_data)
            rclpy.spin_once(self.publisher_node, timeout_sec=0.001)  # Process callbacks briefly
        except Exception as e:
            print(f"Failed to publish episode metrics: {e}")
    
    def publish_training_summary(self, summary_data: Dict[str, Any]):
        """Publish training summary (safe wrapper)"""
        if not self.initialized:
            if not self.initialize():
                return
        
        try:
            self.publisher_node.publish_training_summary(summary_data)
            rclpy.spin_once(self.publisher_node, timeout_sec=0.001)
        except Exception as e:
            print(f"Failed to publish training summary: {e}")
    
    def get_current_stats(self) -> Optional[Dict[str, Any]]:
        """Get current stats (safe wrapper)"""
        if not self.initialized:
            return None
        
        try:
            return self.publisher_node.get_current_stats()
        except Exception as e:
            print(f"Failed to get current stats: {e}")
            return None
    
    def cleanup(self):
        """Cleanup resources"""
        if self.publisher_node:
            self.publisher_node.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()

def main(args=None):
    """Main function to run the metrics publisher node standalone"""
    
    rclpy.init(args=args)
    
    try:
        node = RLMetricsPublisher()
        
        node.get_logger().info("=" * 50)
        node.get_logger().info("RL METRICS PUBLISHER NODE READY")
        node.get_logger().info("=" * 50)
        node.get_logger().info("Waiting for training metrics to publish...")
        node.get_logger().info("Monitor topics:")
        node.get_logger().info("  ros2 topic echo /rl_metrics/episode_stats")
        node.get_logger().info("  ros2 topic echo /rl_metrics/training_progress")
        node.get_logger().info("  ros2 topic echo /rl_metrics/performance")
        node.get_logger().info("=" * 50)
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info("Metrics publisher interrupted by user")
    except Exception as e:
        print(f"Node error: {e}")
        import traceback
        print(f"Traceback: {traceback.format_exc()}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
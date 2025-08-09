"""
Enhanced Unified Training Logger for RL Models
Integrates and improves upon existing logging systems with better error handling
"""

import os
import sys
import json
import csv
import time
import logging
import traceback
from pathlib import Path
from typing import Dict, List, Any, Optional, Union
from dataclasses import dataclass, asdict
from collections import deque, defaultdict
from datetime import datetime
import numpy as np

# ROS2 integration
ROS2_AVAILABLE = False
try:
    import rclpy
    from .rl_metrics_publisher import RLMetricsIntegration
    ROS2_AVAILABLE = True
except ImportError:
    try:
        import rclpy
        from rl_metrics_publisher import RLMetricsIntegration
        ROS2_AVAILABLE = True
    except ImportError:
        # ROS2 not available - will work without it
        RLMetricsIntegration = None

try:
    import torch
    import matplotlib.pyplot as plt
    HAS_PYTORCH = True
except ImportError:
    HAS_PYTORCH = False
    print("PyTorch not available - some features will be disabled")

try:
    from torch.utils.tensorboard import SummaryWriter
    HAS_TENSORBOARD = True
except ImportError:
    HAS_TENSORBOARD = False
    print("Tensorboard not available - visualization features disabled")

@dataclass
class TrainingMetrics:
    """Enhanced training metrics container"""
    episode: int
    timestamp: float
    success: bool
    total_reward: float
    steps_taken: int
    pieces_placed: int = 0
    completion_rate: float = 0.0
    
    # RL-specific metrics
    policy_loss: Optional[float] = None
    value_loss: Optional[float] = None  
    entropy: Optional[float] = None
    kl_divergence: Optional[float] = None
    
    # Assembly-specific metrics
    grasp_success_rate: Optional[float] = None
    placement_accuracy: Optional[float] = None
    force_violations: int = 0
    
    # Performance metrics
    planning_time: Optional[float] = None
    execution_time: Optional[float] = None
    total_episode_time: Optional[float] = None
    
    # Environment state
    curriculum_stage: str = "unknown"
    difficulty_level: float = 1.0
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

class EnhancedTrainingLogger:
    """
    Unified training logger with robust error handling and comprehensive metrics
    Includes ROS2 integration for real-time monitoring
    """
    
    def __init__(self, experiment_name: str = None, log_dir: str = "logs", 
                 config: Optional[Dict] = None, enable_tensorboard: bool = True,
                 enable_ros2: bool = True):
        
        # Setup experiment name with timestamp
        if experiment_name is None:
            experiment_name = "rl_training"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.experiment_name = f"{experiment_name}_{timestamp}"
        
        # Setup directories
        self.log_dir = Path(log_dir) / self.experiment_name
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize logging
        self.setup_logging()
        
        # Initialize file handlers
        self.setup_file_handlers()
        
        # Setup tensorboard if available
        self.tensorboard_writer = None
        if enable_tensorboard and HAS_TENSORBOARD:
            self.setup_tensorboard()
        
        # Setup ROS2 integration if available
        self.ros2_publisher = None
        if enable_ros2 and ROS2_AVAILABLE:
            self.setup_ros2_integration()
        
        # Internal state
        self.metrics_history = []
        self.session_start_time = time.time()
        self.episode_count = 0
        
        # Rolling statistics (last 100 episodes)
        self.rolling_metrics = {
            'rewards': deque(maxlen=100),
            'success_rates': deque(maxlen=100),  
            'episode_lengths': deque(maxlen=100),
            'completion_rates': deque(maxlen=100)
        }
        
        # Save configuration
        if config:
            self.save_config(config)
        
        self.logger.info(f"Enhanced Training Logger initialized: {self.log_dir}")
        self.logger.info(f"Tensorboard: {'Enabled' if self.tensorboard_writer else 'Disabled'}")
        self.logger.info(f"ROS2 Integration: {'Enabled' if self.ros2_publisher else 'Disabled'}")
    
    def setup_logging(self):
        """Setup Python logging with file and console handlers"""
        self.logger = logging.getLogger(f"EnhancedTrainingLogger.{self.experiment_name}")
        self.logger.setLevel(logging.INFO)
        
        # Prevent duplicate handlers
        if self.logger.handlers:
            self.logger.handlers.clear()
        
        # Console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_format = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        console_handler.setFormatter(console_format)
        self.logger.addHandler(console_handler)
        
        # File handler
        log_file = self.log_dir / "training.log"
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(logging.DEBUG)
        file_format = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(funcName)s:%(lineno)d - %(message)s'
        )
        file_handler.setFormatter(file_format)
        self.logger.addHandler(file_handler)
    
    def setup_file_handlers(self):
        """Setup CSV and JSON file handlers"""
        
        # CSV file for metrics
        self.metrics_csv_file = self.log_dir / "training_metrics.csv"
        self.metrics_csv_fieldnames = [
            'episode', 'timestamp', 'success', 'total_reward', 'steps_taken',
            'pieces_placed', 'completion_rate', 'policy_loss', 'value_loss',
            'entropy', 'kl_divergence', 'grasp_success_rate', 'placement_accuracy',
            'force_violations', 'planning_time', 'execution_time', 
            'total_episode_time', 'curriculum_stage', 'difficulty_level'
        ]
        
        # Initialize CSV with headers
        if not self.metrics_csv_file.exists():
            with open(self.metrics_csv_file, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.metrics_csv_fieldnames)
                writer.writeheader()
        
        # JSON file for detailed logging
        self.metrics_json_file = self.log_dir / "training_metrics.jsonl"
        
        # Summary statistics file
        self.summary_file = self.log_dir / "training_summary.json"
    
    def setup_tensorboard(self):
        """Setup tensorboard logging"""
        try:
            tensorboard_dir = self.log_dir / "tensorboard"
            self.tensorboard_writer = SummaryWriter(str(tensorboard_dir))
            self.logger.info(f"Tensorboard logging to: {tensorboard_dir}")
            self.logger.info(f"View with: tensorboard --logdir {tensorboard_dir}")
        except Exception as e:
            self.logger.warning(f"Failed to setup tensorboard: {e}")
            self.tensorboard_writer = None
    
    def setup_ros2_integration(self):
        """Setup ROS2 metrics publishing"""
        try:
            self.ros2_publisher = RLMetricsIntegration(f"rl_logger_{self.experiment_name}")
            self.ros2_publisher.initialize()
            self.logger.info("ROS2 metrics integration enabled")
        except Exception as e:
            self.logger.warning(f"Failed to setup ROS2 integration: {e}")
            self.ros2_publisher = None
    
    def log_episode(self, metrics: Union[TrainingMetrics, Dict[str, Any]], 
                   training_stats: Optional[Dict[str, Any]] = None):
        """
        Log training episode with comprehensive error handling
        
        Args:
            metrics: Training metrics (TrainingMetrics object or dict)
            training_stats: Optional additional training statistics
        """
        try:
            # Convert to TrainingMetrics if needed
            if isinstance(metrics, dict):
                # Fill in defaults for missing keys
                metrics_dict = {
                    'episode': self.episode_count,
                    'timestamp': time.time(),
                    'success': False,
                    'total_reward': 0.0,
                    'steps_taken': 0,
                    **metrics
                }
                metrics = TrainingMetrics(**metrics_dict)
            
            # Update episode count
            self.episode_count = max(self.episode_count, metrics.episode + 1)
            
            # Add to history
            self.metrics_history.append(metrics)
            
            # Update rolling statistics
            self.rolling_metrics['rewards'].append(metrics.total_reward)
            self.rolling_metrics['success_rates'].append(1.0 if metrics.success else 0.0)
            self.rolling_metrics['episode_lengths'].append(metrics.steps_taken)
            self.rolling_metrics['completion_rates'].append(metrics.completion_rate)
            
            # Merge training stats if provided
            if training_stats:
                if 'policy_loss' in training_stats:
                    metrics.policy_loss = training_stats['policy_loss']
                if 'value_loss' in training_stats:
                    metrics.value_loss = training_stats['value_loss']
                if 'entropy' in training_stats:
                    metrics.entropy = training_stats['entropy']
                if 'kl_divergence' in training_stats or 'kl_div' in training_stats:
                    metrics.kl_divergence = training_stats.get('kl_divergence', training_stats.get('kl_div'))
            
            # Log to files
            self.write_to_csv(metrics)
            self.write_to_json(metrics)
            
            # Log to tensorboard
            if self.tensorboard_writer:
                self.log_to_tensorboard(metrics)
            
            # Publish to ROS2 topics
            if self.ros2_publisher:
                self.publish_to_ros2(metrics, training_stats)
            
            # Console logging with rolling averages
            if metrics.episode % 10 == 0:  # Log every 10 episodes
                self.log_console_summary(metrics)
            
        except Exception as e:
            self.logger.error(f"Error logging episode {getattr(metrics, 'episode', 'unknown')}: {e}")
            self.logger.error(f"Traceback: {traceback.format_exc()}")
    
    def write_to_csv(self, metrics: TrainingMetrics):
        """Write metrics to CSV file"""
        try:
            metrics_dict = metrics.to_dict()
            
            # Ensure all expected fields are present
            csv_row = {}
            for field in self.metrics_csv_fieldnames:
                csv_row[field] = metrics_dict.get(field, '')
            
            with open(self.metrics_csv_file, 'a', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=self.metrics_csv_fieldnames)
                writer.writerow(csv_row)
                
        except Exception as e:
            self.logger.error(f"Failed to write CSV: {e}")
    
    def write_to_json(self, metrics: TrainingMetrics):
        """Write metrics to JSON lines file"""
        try:
            with open(self.metrics_json_file, 'a') as f:
                json.dump(metrics.to_dict(), f)
                f.write('\n')
        except Exception as e:
            self.logger.error(f"Failed to write JSON: {e}")
    
    def log_to_tensorboard(self, metrics: TrainingMetrics):
        """Log metrics to tensorboard"""
        try:
            episode = metrics.episode
            
            # Basic metrics
            self.tensorboard_writer.add_scalar('Episode/Reward', metrics.total_reward, episode)
            self.tensorboard_writer.add_scalar('Episode/Success', float(metrics.success), episode)
            self.tensorboard_writer.add_scalar('Episode/Steps', metrics.steps_taken, episode)
            self.tensorboard_writer.add_scalar('Episode/Pieces_Placed', metrics.pieces_placed, episode)
            self.tensorboard_writer.add_scalar('Episode/Completion_Rate', metrics.completion_rate, episode)
            
            # RL-specific metrics
            if metrics.policy_loss is not None:
                self.tensorboard_writer.add_scalar('Training/Policy_Loss', metrics.policy_loss, episode)
            if metrics.value_loss is not None:
                self.tensorboard_writer.add_scalar('Training/Value_Loss', metrics.value_loss, episode)
            if metrics.entropy is not None:
                self.tensorboard_writer.add_scalar('Training/Entropy', metrics.entropy, episode)
            if metrics.kl_divergence is not None:
                self.tensorboard_writer.add_scalar('Training/KL_Divergence', metrics.kl_divergence, episode)
            
            # Rolling averages (every 10 episodes)
            if episode % 10 == 0:
                if len(self.rolling_metrics['rewards']) > 0:
                    avg_reward = np.mean(list(self.rolling_metrics['rewards']))
                    avg_success = np.mean(list(self.rolling_metrics['success_rates']))
                    avg_steps = np.mean(list(self.rolling_metrics['episode_lengths']))
                    
                    self.tensorboard_writer.add_scalar('Rolling/Avg_Reward', avg_reward, episode)
                    self.tensorboard_writer.add_scalar('Rolling/Success_Rate', avg_success, episode)
                    self.tensorboard_writer.add_scalar('Rolling/Avg_Steps', avg_steps, episode)
            
            self.tensorboard_writer.flush()
            
        except Exception as e:
            self.logger.error(f"Failed to log to tensorboard: {e}")
    
    def publish_to_ros2(self, metrics: TrainingMetrics, training_stats: Optional[Dict[str, Any]] = None):
        """Publish metrics to ROS2 topics"""
        try:
            episode_data = {
                'episode': metrics.episode,
                'success': metrics.success,
                'reward': metrics.total_reward,
                'steps': metrics.steps_taken,
                'pieces_placed': metrics.pieces_placed,
                'completion_rate': metrics.completion_rate,
                'curriculum_stage': metrics.curriculum_stage,
                'timestamp': metrics.timestamp
            }
            
            # Add training stats if available
            if training_stats:
                episode_data.update({
                    'policy_loss': training_stats.get('policy_loss'),
                    'value_loss': training_stats.get('value_loss'),
                    'entropy': training_stats.get('entropy'),
                    'kl_divergence': training_stats.get('kl_divergence', training_stats.get('kl_div'))
                })
            
            # Add performance timing if available
            if metrics.planning_time is not None:
                episode_data['planning_time'] = metrics.planning_time
            if metrics.execution_time is not None:
                episode_data['execution_time'] = metrics.execution_time
            if metrics.total_episode_time is not None:
                episode_data['total_episode_time'] = metrics.total_episode_time
            
            # Publish to ROS2
            self.ros2_publisher.publish_episode_metrics(episode_data)
            
        except Exception as e:
            self.logger.error(f"Failed to publish to ROS2: {e}")
    
    def log_console_summary(self, metrics: TrainingMetrics):
        """Log summary to console with rolling statistics"""
        try:
            # Calculate rolling averages
            if len(self.rolling_metrics['rewards']) > 0:
                avg_reward = np.mean(list(self.rolling_metrics['rewards']))
                success_rate = np.mean(list(self.rolling_metrics['success_rates']))
                avg_steps = np.mean(list(self.rolling_metrics['episode_lengths']))
                avg_completion = np.mean(list(self.rolling_metrics['completion_rates']))
            else:
                avg_reward = metrics.total_reward
                success_rate = 1.0 if metrics.success else 0.0
                avg_steps = metrics.steps_taken
                avg_completion = metrics.completion_rate
            
            elapsed_time = time.time() - self.session_start_time
            
            # Format training stats
            training_info = ""
            if metrics.policy_loss is not None:
                training_info += f" | Policy Loss: {metrics.policy_loss:.4f}"
            if metrics.value_loss is not None:
                training_info += f" | Value Loss: {metrics.value_loss:.4f}"
            
            # Main log message
            self.logger.info(
                f"Episode {metrics.episode:6d} | "
                f"Success: {metrics.success} | "
                f"Reward: {metrics.total_reward:8.2f} | "
                f"Steps: {metrics.steps_taken:3d} | "
                f"Pieces: {metrics.pieces_placed}/7 | "
                f"Rolling - SR: {success_rate:.2%}, Reward: {avg_reward:6.2f}, Steps: {avg_steps:.1f} | "
                f"Time: {elapsed_time:.0f}s{training_info}"
            )
            
        except Exception as e:
            self.logger.error(f"Failed to log console summary: {e}")
    
    def save_config(self, config: Dict[str, Any]):
        """Save experiment configuration"""
        try:
            config_file = self.log_dir / "experiment_config.json"
            
            # Convert non-serializable objects to strings
            serializable_config = {}
            for key, value in config.items():
                try:
                    json.dumps(value)  # Test if serializable
                    serializable_config[key] = value
                except (TypeError, ValueError):
                    serializable_config[key] = str(value)
            
            with open(config_file, 'w') as f:
                json.dump(serializable_config, f, indent=2, default=str)
                
            self.logger.info(f"Configuration saved to {config_file}")
            
        except Exception as e:
            self.logger.error(f"Failed to save config: {e}")
    
    def generate_summary(self) -> Dict[str, Any]:
        """Generate comprehensive training summary"""
        try:
            if not self.metrics_history:
                return {"error": "No training data available"}
            
            # Basic statistics
            total_episodes = len(self.metrics_history)
            successful_episodes = sum(1 for m in self.metrics_history if m.success)
            total_training_time = time.time() - self.session_start_time
            
            # Performance metrics
            rewards = [m.total_reward for m in self.metrics_history]
            steps = [m.steps_taken for m in self.metrics_history]
            completion_rates = [m.completion_rate for m in self.metrics_history]
            
            summary = {
                'experiment_info': {
                    'name': self.experiment_name,
                    'total_episodes': total_episodes,
                    'successful_episodes': successful_episodes,
                    'success_rate': successful_episodes / total_episodes if total_episodes > 0 else 0.0,
                    'total_training_time_seconds': total_training_time,
                    'total_training_time_hours': total_training_time / 3600
                },
                'performance_metrics': {
                    'average_reward': np.mean(rewards),
                    'std_reward': np.std(rewards),
                    'min_reward': np.min(rewards),
                    'max_reward': np.max(rewards),
                    'average_steps': np.mean(steps),
                    'std_steps': np.std(steps),
                    'average_completion_rate': np.mean(completion_rates),
                },
                'recent_performance': {},
                'training_efficiency': {
                    'episodes_per_hour': total_episodes / (total_training_time / 3600) if total_training_time > 0 else 0,
                    'successful_episodes_per_hour': successful_episodes / (total_training_time / 3600) if total_training_time > 0 else 0
                }
            }
            
            # Recent performance (last 100 episodes)
            if len(self.rolling_metrics['rewards']) > 0:
                summary['recent_performance'] = {
                    'recent_success_rate': np.mean(list(self.rolling_metrics['success_rates'])),
                    'recent_average_reward': np.mean(list(self.rolling_metrics['rewards'])),
                    'recent_average_steps': np.mean(list(self.rolling_metrics['episode_lengths']))
                }
            
            # Save summary to file
            with open(self.summary_file, 'w') as f:
                json.dump(summary, f, indent=2, default=str)
            
            return summary
            
        except Exception as e:
            self.logger.error(f"Failed to generate summary: {e}")
            return {"error": f"Failed to generate summary: {str(e)}"}
    
    def close(self):
        """Close logger and generate final summary"""
        try:
            # Generate and save final summary
            summary = self.generate_summary()
            
            # Publish summary to ROS2
            if self.ros2_publisher:
                self.ros2_publisher.publish_training_summary(summary)
            
            # Close tensorboard
            if self.tensorboard_writer:
                self.tensorboard_writer.close()
            
            # Cleanup ROS2
            if self.ros2_publisher:
                self.ros2_publisher.cleanup()
            
            # Log final statistics
            if 'experiment_info' in summary:
                info = summary['experiment_info']
                self.logger.info("=" * 80)
                self.logger.info("TRAINING SESSION COMPLETED")
                self.logger.info("=" * 80)
                self.logger.info(f"Total Episodes: {info['total_episodes']}")
                self.logger.info(f"Successful Episodes: {info['successful_episodes']}")
                self.logger.info(f"Success Rate: {info['success_rate']:.2%}")
                self.logger.info(f"Training Time: {info['total_training_time_hours']:.2f} hours")
                
                if 'performance_metrics' in summary:
                    perf = summary['performance_metrics']
                    self.logger.info(f"Average Reward: {perf['average_reward']:.2f} ± {perf['std_reward']:.2f}")
                    self.logger.info(f"Average Steps: {perf['average_steps']:.1f} ± {perf['std_steps']:.1f}")
                
                self.logger.info(f"Logs saved to: {self.log_dir}")
                self.logger.info("=" * 80)
            
        except Exception as e:
            self.logger.error(f"Error closing logger: {e}")

def create_training_logger(experiment_name: str = None, **kwargs) -> EnhancedTrainingLogger:
    """
    Factory function to create enhanced training logger
    
    Args:
        experiment_name: Name of the experiment
        **kwargs: Additional arguments for EnhancedTrainingLogger
    
    Returns:
        EnhancedTrainingLogger instance
    """
    return EnhancedTrainingLogger(experiment_name=experiment_name, **kwargs)

# Compatibility functions for existing code
def setup_experiment_logging(experiment_name: str, config: Dict[str, Any]) -> EnhancedTrainingLogger:
    """Setup logging compatible with existing code"""
    return create_training_logger(experiment_name=experiment_name, config=config)

if __name__ == "__main__":
    # Test the logger
    print("Testing Enhanced Training Logger...")
    
    # Create logger
    logger = create_training_logger("test_experiment")
    
    # Test logging some episodes
    for episode in range(5):
        metrics = TrainingMetrics(
            episode=episode,
            timestamp=time.time(),
            success=episode > 2,
            total_reward=np.random.uniform(-10, 50),
            steps_taken=np.random.randint(10, 30),
            pieces_placed=np.random.randint(0, 7),
            completion_rate=np.random.uniform(0, 1),
            policy_loss=np.random.uniform(0.1, 1.0),
            value_loss=np.random.uniform(0.1, 1.0)
        )
        
        logger.log_episode(metrics)
        time.sleep(0.1)
    
    # Generate summary
    summary = logger.generate_summary()
    print(f"Summary: {json.dumps(summary, indent=2, default=str)}")
    
    # Close logger
    logger.close()
    
    print("Test completed successfully!")
"""
Comprehensive Logging Utilities for Hierarchical SOMA Cube Assembly

This module provides logging and metrics tracking capabilities for:
1. Training progress and performance metrics
2. Curriculum learning progression
3. Domain randomization effects
4. Model checkpointing and recovery
5. Real-time monitoring and alerting
"""

import logging
import json
import csv
import pickle
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass, asdict
from collections import defaultdict, deque
import numpy as np
import torch
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
import threading
import queue

# Set up logging format
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

@dataclass
class TrainingMetrics:
    """Container for training metrics"""
    episode: int
    timestamp: float
    success: bool
    total_reward: float
    steps_taken: int
    pieces_placed: int
    curriculum_stage: str
    upper_level_loss: Optional[float] = None
    lower_level_loss: Optional[float] = None
    planning_time: Optional[float] = None
    execution_time: Optional[float] = None
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

@dataclass
class EvaluationMetrics:
    """Container for evaluation metrics"""
    episode: int
    timestamp: float
    success_rate: float
    average_reward: float
    average_steps: float
    std_reward: float
    std_steps: float
    num_eval_episodes: int
    
    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

class TrainingLogger:
    """
    Comprehensive training logger with multiple output formats
    
    Features:
    - JSON logging for structured data
    - CSV logging for tabular data
    - Real-time metrics tracking
    - Automated backup and rotation
    - Progress visualization
    """
    
    def __init__(self, experiment_name: str, log_dir: str = "logs", 
                 config: Optional[Dict] = None):
        self.experiment_name = experiment_name
        self.log_dir = Path(log_dir) / experiment_name
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize Python logger
        self.logger = logging.getLogger(f"TrainingLogger.{experiment_name}")
        self.logger.setLevel(logging.INFO)
        
        # Initialize logging files
        self.setup_logging_files()
        
        # Store experiment configuration
        if config:
            self.save_config(config)
        
        # Internal state
        self.training_metrics = []
        self.evaluation_metrics = []
        self.start_time = time.time()
        
        # Background logging thread
        self.log_queue = queue.Queue()
        self.logging_thread = threading.Thread(target=self._background_logger, daemon=True)
        self.logging_thread.start()
        
        self.logger = logging.getLogger(f"TrainingLogger.{experiment_name}")
        self.logger.info(f"Training logger initialized: {self.log_dir}")
    
    def setup_logging_files(self):
        """Setup various logging files"""
        
        # Training log (JSON)
        self.training_log_file = self.log_dir / "training_log.jsonl"
        
        # Evaluation log (JSON)
        self.evaluation_log_file = self.log_dir / "evaluation_log.jsonl"
        
        # CSV files for easy analysis
        self.training_csv_file = self.log_dir / "training_metrics.csv"
        self.evaluation_csv_file = self.log_dir / "evaluation_metrics.csv"
        
        # System log
        self.system_log_file = self.log_dir / "system.log"
        
        # Setup file handlers
        file_handler = logging.FileHandler(self.system_log_file)
        file_handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        file_handler.setFormatter(formatter)
        
        # Add handler to root logger
        root_logger = logging.getLogger()
        root_logger.addHandler(file_handler)
        
        # Initialize CSV files with headers
        self._initialize_csv_files()
    
    def _initialize_csv_files(self):
        """Initialize CSV files with headers"""
        
        # Training CSV headers
        training_headers = [
            'episode', 'timestamp', 'success', 'total_reward', 'steps_taken',
            'pieces_placed', 'curriculum_stage', 'upper_level_loss', 
            'lower_level_loss', 'planning_time', 'execution_time'
        ]
        
        if not self.training_csv_file.exists():
            with open(self.training_csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(training_headers)
        
        # Evaluation CSV headers
        eval_headers = [
            'episode', 'timestamp', 'success_rate', 'average_reward',
            'average_steps', 'std_reward', 'std_steps', 'num_eval_episodes'
        ]
        
        if not self.evaluation_csv_file.exists():
            with open(self.evaluation_csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(eval_headers)
    
    def log_episode(self, episode: int, episode_result: Dict[str, Any], 
                   curriculum_info: Dict[str, Any], model_losses: Optional[Dict] = None):
        """Log training episode data"""
        
        # Create training metrics
        metrics = TrainingMetrics(
            episode=episode,
            timestamp=time.time(),
            success=episode_result.get('success', False),
            total_reward=episode_result.get('total_reward', 0.0),
            steps_taken=episode_result.get('steps_taken', 0),
            pieces_placed=episode_result.get('pieces_placed', 0),
            curriculum_stage=curriculum_info.get('current_stage', 'unknown'),
            upper_level_loss=model_losses.get('upper_loss') if model_losses else None,
            lower_level_loss=model_losses.get('lower_loss') if model_losses else None,
            planning_time=episode_result.get('planning_time'),
            execution_time=episode_result.get('execution_time')
        )
        
        # Queue for background processing
        self.log_queue.put(('training', metrics))
        self.training_metrics.append(metrics)
        
        # Real-time console logging
        elapsed_time = time.time() - self.start_time
        self.logger.info(
            f"Episode {episode}: Success={metrics.success}, "
            f"Reward={metrics.total_reward:.2f}, "
            f"Steps={metrics.steps_taken}, "
            f"Stage={metrics.curriculum_stage}, "
            f"Elapsed={elapsed_time:.1f}s"
        )
    
    def log_evaluation(self, episode: int, eval_results: Dict[str, Any]):
        """Log evaluation results"""
        
        # Calculate statistics
        rewards = [r['reward'] for r in eval_results.get('episode_results', [])]
        steps = [r['steps'] for r in eval_results.get('episode_results', [])]
        
        metrics = EvaluationMetrics(
            episode=episode,
            timestamp=time.time(),
            success_rate=eval_results.get('success_rate', 0.0),
            average_reward=eval_results.get('average_reward', 0.0),
            average_steps=eval_results.get('average_steps', 0.0),
            std_reward=np.std(rewards) if rewards else 0.0,
            std_steps=np.std(steps) if steps else 0.0,
            num_eval_episodes=eval_results.get('num_episodes', 0)
        )
        
        # Queue for background processing
        self.log_queue.put(('evaluation', metrics))
        self.evaluation_metrics.append(metrics)
        
        # Console logging
        self.logger.info(
            f"Evaluation at episode {episode}: "
            f"Success Rate={metrics.success_rate:.3f}, "
            f"Avg Reward={metrics.average_reward:.2f}±{metrics.std_reward:.2f}, "
            f"Avg Steps={metrics.average_steps:.1f}±{metrics.std_steps:.1f}"
        )
    
    def log_curriculum_progression(self, episode: int, from_stage: str, to_stage: str, 
                                 performance: Dict[str, float]):
        """Log curriculum progression events"""
        
        progression_data = {
            'episode': episode,
            'timestamp': time.time(),
            'from_stage': from_stage,
            'to_stage': to_stage,
            'performance': performance
        }
        
        # Log to separate curriculum file
        curriculum_file = self.log_dir / "curriculum_progression.jsonl"
        with open(curriculum_file, 'a') as f:
            f.write(json.dumps(progression_data) + '\n')
        
        self.logger.info(
            f"Curriculum progression at episode {episode}: "
            f"{from_stage} -> {to_stage} "
            f"(Success Rate: {performance.get('success_rate', 0):.3f})"
        )
    
    def log_domain_randomization(self, episode: int, randomization_status: Dict[str, Any]):
        """Log domain randomization effects"""
        
        dr_data = {
            'episode': episode,
            'timestamp': time.time(),
            'randomization_status': randomization_status
        }
        
        # Log to separate DR file
        dr_file = self.log_dir / "domain_randomization.jsonl"
        with open(dr_file, 'a') as f:
            f.write(json.dumps(dr_data) + '\n')
    
    def _background_logger(self):
        """Background thread for file logging"""
        
        while True:
            try:
                log_type, data = self.log_queue.get(timeout=1.0)
                
                if log_type == 'training':
                    self._write_training_data(data)
                elif log_type == 'evaluation':
                    self._write_evaluation_data(data)
                
                self.log_queue.task_done()
                
            except queue.Empty:
                continue
            except Exception as e:
                self.logger.error(f"Background logging error: {e}")
    
    def _write_training_data(self, metrics: TrainingMetrics):
        """Write training data to files"""
        
        # JSON logging
        with open(self.training_log_file, 'a') as f:
            f.write(json.dumps(metrics.to_dict()) + '\n')
        
        # CSV logging
        with open(self.training_csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                metrics.episode, metrics.timestamp, metrics.success,
                metrics.total_reward, metrics.steps_taken, metrics.pieces_placed,
                metrics.curriculum_stage, metrics.upper_level_loss,
                metrics.lower_level_loss, metrics.planning_time, metrics.execution_time
            ])
    
    def _write_evaluation_data(self, metrics: EvaluationMetrics):
        """Write evaluation data to files"""
        
        # JSON logging
        with open(self.evaluation_log_file, 'a') as f:
            f.write(json.dumps(metrics.to_dict()) + '\n')
        
        # CSV logging
        with open(self.evaluation_csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                metrics.episode, metrics.timestamp, metrics.success_rate,
                metrics.average_reward, metrics.average_steps, metrics.std_reward,
                metrics.std_steps, metrics.num_eval_episodes
            ])
    
    def save_config(self, config: Dict[str, Any]):
        """Save experiment configuration"""
        
        config_file = self.log_dir / "experiment_config.json"
        with open(config_file, 'w') as f:
            json.dump(config, f, indent=2, default=str)
        
        self.logger.info(f"Saved experiment config to {config_file}")
    
    def generate_training_summary(self) -> Dict[str, Any]:
        """Generate comprehensive training summary"""
        
        if not self.training_metrics:
            return {"error": "No training data available"}
        
        # Calculate summary statistics
        total_episodes = len(self.training_metrics)
        successful_episodes = sum(1 for m in self.training_metrics if m.success)
        success_rate = successful_episodes / total_episodes
        
        rewards = [m.total_reward for m in self.training_metrics]
        steps = [m.steps_taken for m in self.training_metrics]
        
        summary = {
            'experiment_name': self.experiment_name,
            'total_episodes': total_episodes,
            'successful_episodes': successful_episodes,
            'overall_success_rate': success_rate,
            'average_reward': np.mean(rewards),
            'std_reward': np.std(rewards),
            'average_steps': np.mean(steps),
            'std_steps': np.std(steps),
            'total_training_time': time.time() - self.start_time,
            'curriculum_stages_reached': len(set(m.curriculum_stage for m in self.training_metrics))
        }
        
        # Add evaluation summary if available
        if self.evaluation_metrics:
            best_eval = max(self.evaluation_metrics, key=lambda x: x.success_rate)
            summary['best_evaluation'] = {
                'episode': best_eval.episode,
                'success_rate': best_eval.success_rate,
                'average_reward': best_eval.average_reward
            }
        
        # Save summary
        summary_file = self.log_dir / "training_summary.json"
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        return summary
    
    def close(self):
        """Close logger and generate final summary"""
        
        # Wait for background thread to finish
        self.log_queue.join()
        
        # Generate final summary
        summary = self.generate_training_summary()
        self.logger.info("Training summary generated")
        
        return summary

class MetricsTracker:
    """
    Real-time metrics tracking and analysis
    
    Features:
    - Sliding window statistics
    - Trend analysis
    - Anomaly detection
    - Performance alerts
    """
    
    def __init__(self, window_size: int = 100):
        self.window_size = window_size
        
        # Metrics storage
        self.metrics = defaultdict(lambda: deque(maxlen=window_size))
        self.all_metrics = defaultdict(list)
        
        # Trend analysis
        self.trend_window = 20
        self.alert_thresholds = {
            'success_rate_drop': 0.2,  # Alert if success rate drops by 20%
            'reward_drop': 0.3,        # Alert if reward drops by 30%
            'steps_increase': 0.5      # Alert if steps increase by 50%
        }
        
        self.logger = logging.getLogger("MetricsTracker")
    
    def add_scalar(self, name: str, value: Union[float, int], step: Optional[int] = None):
        """Add scalar metric"""
        
        timestamp = time.time()
        entry = {'value': float(value), 'timestamp': timestamp, 'step': step}
        
        self.metrics[name].append(entry)
        self.all_metrics[name].append(entry)
        
        # Check for alerts
        self._check_alerts(name)
    
    def add_histogram(self, name: str, values: List[float], step: Optional[int] = None):
        """Add histogram metric"""
        
        histogram_stats = {
            'mean': np.mean(values),
            'std': np.std(values),
            'min': np.min(values),
            'max': np.max(values),
            'median': np.median(values),
            'count': len(values)
        }
        
        self.add_scalar(f"{name}/mean", histogram_stats['mean'], step)
        self.add_scalar(f"{name}/std", histogram_stats['std'], step)
        self.add_scalar(f"{name}/min", histogram_stats['min'], step)
        self.add_scalar(f"{name}/max", histogram_stats['max'], step)
    
    def get_current_stats(self, name: str) -> Dict[str, float]:
        """Get current statistics for a metric"""
        
        if name not in self.metrics or len(self.metrics[name]) == 0:
            return {}
        
        values = [entry['value'] for entry in self.metrics[name]]
        
        return {
            'current': values[-1],
            'mean': np.mean(values),
            'std': np.std(values),
            'min': np.min(values),
            'max': np.max(values),
            'median': np.median(values),
            'trend': self._calculate_trend(values)
        }
    
    def _calculate_trend(self, values: List[float]) -> str:
        """Calculate trend direction"""
        
        if len(values) < self.trend_window:
            return 'insufficient_data'
        
        recent_values = values[-self.trend_window:]
        early_mean = np.mean(recent_values[:len(recent_values)//2])
        late_mean = np.mean(recent_values[len(recent_values)//2:])
        
        change_ratio = (late_mean - early_mean) / abs(early_mean + 1e-8)
        
        if change_ratio > 0.05:
            return 'increasing'
        elif change_ratio < -0.05:
            return 'decreasing'
        else:
            return 'stable'
    
    def _check_alerts(self, name: str):
        """Check for performance alerts"""
        
        if len(self.metrics[name]) < 2:
            return
        
        current = self.metrics[name][-1]['value']
        previous = self.metrics[name][-2]['value']
        
        # Success rate drop alert
        if name == 'success_rate' and current < previous * (1 - self.alert_thresholds['success_rate_drop']):
            self.logger.warning(f"Success rate drop detected: {previous:.3f} -> {current:.3f}")
        
        # Reward drop alert
        if name == 'episode_reward' and current < previous * (1 - self.alert_thresholds['reward_drop']):
            self.logger.warning(f"Reward drop detected: {previous:.2f} -> {current:.2f}")
        
        # Steps increase alert
        if name == 'steps_taken' and current > previous * (1 + self.alert_thresholds['steps_increase']):
            self.logger.warning(f"Steps increase detected: {previous:.1f} -> {current:.1f}")
    
    def export_metrics(self, filepath: str):
        """Export all metrics to file"""
        
        export_data = {
            'metrics': dict(self.all_metrics),
            'export_timestamp': time.time(),
            'window_size': self.window_size
        }
        
        if filepath.endswith('.json'):
            with open(filepath, 'w') as f:
                json.dump(export_data, f, indent=2, default=str)
        else:
            with open(filepath, 'wb') as f:
                pickle.dump(export_data, f)
        
        self.logger.info(f"Metrics exported to {filepath}")

class CheckpointManager:
    """
    Manages model checkpoints and training state recovery
    
    Features:
    - Automatic checkpoint saving
    - Best model tracking
    - Training state recovery
    - Checkpoint cleanup and rotation
    """
    
    def __init__(self, checkpoint_dir: str, experiment_name: str, 
                 max_checkpoints: int = 5):
        
        self.checkpoint_dir = Path(checkpoint_dir) / experiment_name
        self.checkpoint_dir.mkdir(parents=True, exist_ok=True)
        
        self.max_checkpoints = max_checkpoints
        self.best_metric_value = float('-inf')
        self.checkpoint_history = []
        
        self.logger = logging.getLogger("CheckpointManager")
        self.logger.info(f"Checkpoint manager initialized: {self.checkpoint_dir}")
    
    def save_checkpoint(self, episode: int, state_dict: Dict[str, Any], 
                       metrics: Dict[str, float], is_best: bool = False):
        """Save model checkpoint"""
        
        checkpoint_data = {
            'episode': episode,
            'timestamp': time.time(),
            'state_dict': state_dict,
            'metrics': metrics
        }
        
        # Regular checkpoint
        checkpoint_path = self.checkpoint_dir / f"checkpoint_ep_{episode}.pth"
        torch.save(checkpoint_data, checkpoint_path)
        
        self.checkpoint_history.append({
            'episode': episode,
            'path': checkpoint_path,
            'timestamp': time.time(),
            'is_best': is_best
        })
        
        # Best checkpoint
        if is_best:
            best_path = self.checkpoint_dir / "best_checkpoint.pth"
            torch.save(checkpoint_data, best_path)
            self.logger.info(f"New best checkpoint saved (episode {episode})")
        
        # Cleanup old checkpoints
        self._cleanup_old_checkpoints()
        
        self.logger.info(f"Checkpoint saved: episode {episode}")
    
    def load_latest_checkpoint(self) -> Optional[Dict[str, Any]]:
        """Load the latest checkpoint"""
        
        checkpoint_files = list(self.checkpoint_dir.glob("checkpoint_ep_*.pth"))
        
        if not checkpoint_files:
            return None
        
        # Sort by episode number
        latest_file = max(checkpoint_files, 
                         key=lambda x: int(x.stem.split('_')[-1]))
        
        checkpoint = torch.load(latest_file)
        self.logger.info(f"Loaded checkpoint from episode {checkpoint['episode']}")
        
        return checkpoint
    
    def load_best_checkpoint(self) -> Optional[Dict[str, Any]]:
        """Load the best checkpoint"""
        
        best_path = self.checkpoint_dir / "best_checkpoint.pth"
        
        if not best_path.exists():
            return None
        
        checkpoint = torch.load(best_path)
        self.logger.info(f"Loaded best checkpoint from episode {checkpoint['episode']}")
        
        return checkpoint
    
    def _cleanup_old_checkpoints(self):
        """Remove old checkpoints to save space"""
        
        if len(self.checkpoint_history) <= self.max_checkpoints:
            return
        
        # Keep the most recent checkpoints and best checkpoints
        to_remove = self.checkpoint_history[:-self.max_checkpoints]
        
        for checkpoint_info in to_remove:
            if not checkpoint_info['is_best']:
                try:
                    checkpoint_info['path'].unlink()
                    self.logger.debug(f"Removed old checkpoint: {checkpoint_info['path']}")
                except FileNotFoundError:
                    pass
        
        # Update history
        self.checkpoint_history = self.checkpoint_history[-self.max_checkpoints:]

def setup_experiment_logging(experiment_name: str, config: Dict[str, Any]) -> TrainingLogger:
    """
    Setup comprehensive logging for an experiment
    
    Args:
        experiment_name: Name of the experiment
        config: Experiment configuration
        
    Returns:
        TrainingLogger instance
    """
    
    # Create timestamped experiment name
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    full_experiment_name = f"{experiment_name}_{timestamp}"
    
    # Setup logger
    logger = TrainingLogger(full_experiment_name, config=config)
    
    return logger
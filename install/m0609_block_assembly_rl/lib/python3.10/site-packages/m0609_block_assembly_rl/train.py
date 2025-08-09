"""
Training Script for M0609 Block Assembly Reinforcement Learning

This script trains the PPO agent to optimize Doosan M0609 robot block assembly
tasks, targeting 10-minute to 5-minute performance improvement.
"""

import os
import sys
import time
import argparse
import numpy as np
import torch
import matplotlib.pyplot as plt
import logging
from datetime import datetime
from typing import Dict, List, Tuple
import json
import wandb  # For experiment tracking (optional)

# Add current directory to path for imports (for standalone execution)
if __name__ == "__main__":
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from environment import M0609BlockAssemblyEnv
    from ppo_agent import M0609PPOAgent, PPOConfig
else:
    # Use relative imports for ROS2 package
    from .environment import M0609BlockAssemblyEnv
    from .ppo_agent import M0609PPOAgent, PPOConfig

class TrainingConfig:
    """Training configuration"""
    def __init__(self):
        # Training parameters
        self.max_episodes = 50000
        self.max_steps_per_episode = 7
        self.save_interval = 1000
        self.eval_interval = 500
        self.log_interval = 100
        
        # Environment parameters
        self.robot_id = "dsr01"
        self.robot_model = "m0609"
        self.virtual_mode = True
        
        # Target performance metrics
        self.target_assembly_time = 300  # 5 minutes in seconds
        self.target_success_rate = 0.95
        self.baseline_time = 600  # 10 minutes baseline
        
        # Training monitoring
        self.patience = 5000  # Early stopping patience
        self.min_improvement = 0.01
        
        # Hardware settings
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.num_workers = 1  # Number of parallel environments
        
        # Logging
        self.use_wandb = False
        self.wandb_project = "m0609-block-assembly"
        
class MetricsTracker:
    """Training metrics tracking and visualization"""
    
    def __init__(self, config: TrainingConfig):
        self.config = config
        self.metrics = {
            'episode_rewards': [],
            'episode_lengths': [],
            'assembly_times': [],
            'success_rates': [],
            'motion_efficiencies': [],
            'policy_losses': [],
            'value_losses': [],
            'entropies': []
        }
        
        # Performance tracking
        self.best_performance = {
            'avg_time': float('inf'),
            'success_rate': 0.0,
            'episode': 0,
            'model_path': None
        }
        
        # Setup logging
        self.setup_logging()
        
        # Setup experiment tracking
        if self.config.use_wandb:
            self.setup_wandb()
    
    def setup_logging(self):
        """Setup logging configuration"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.join("logs", f"m0609_training_{timestamp}")
        os.makedirs(log_dir, exist_ok=True)
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(os.path.join(log_dir, 'training.log')),
                logging.StreamHandler()
            ]
        )
        
        self.log_dir = log_dir
        self.logger = logging.getLogger(__name__)
    
    def setup_wandb(self):
        """Setup Weights & Biases experiment tracking"""
        try:
            wandb.init(
                project=self.config.wandb_project,
                config=vars(self.config),
                name=f"m0609_ppo_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            )
        except Exception as e:
            self.logger.warning(f"Failed to initialize wandb: {e}")
            self.config.use_wandb = False
    
    def update(self, episode: int, episode_reward: float, episode_length: int,
               assembly_time: float, success: bool, motion_efficiency: float,
               agent_stats: Dict = None):
        """Update metrics with episode results"""
        self.metrics['episode_rewards'].append(episode_reward)
        self.metrics['episode_lengths'].append(episode_length)
        self.metrics['assembly_times'].append(assembly_time)
        self.metrics['success_rates'].append(1.0 if success else 0.0)
        self.metrics['motion_efficiencies'].append(motion_efficiency)
        
        if agent_stats:
            if 'avg_policy_loss' in agent_stats:
                self.metrics['policy_losses'].append(agent_stats['avg_policy_loss'])
            if 'avg_value_loss' in agent_stats:
                self.metrics['value_losses'].append(agent_stats['avg_value_loss'])
            if 'avg_entropy' in agent_stats:
                self.metrics['entropies'].append(agent_stats['avg_entropy'])
        
        # Log to wandb if enabled
        if self.config.use_wandb:
            wandb_metrics = {
                'episode': episode,
                'episode_reward': episode_reward,
                'assembly_time': assembly_time,
                'success': success,
                'motion_efficiency': motion_efficiency
            }
            if agent_stats:
                wandb_metrics.update(agent_stats)
            wandb.log(wandb_metrics)
    
    def get_recent_performance(self, window: int = 100) -> Dict:
        """Get recent performance metrics"""
        if len(self.metrics['episode_rewards']) < window:
            window = len(self.metrics['episode_rewards'])
        
        if window == 0:
            return {}
        
        recent_times = self.metrics['assembly_times'][-window:]
        recent_success = self.metrics['success_rates'][-window:]
        recent_rewards = self.metrics['episode_rewards'][-window:]
        recent_efficiency = self.metrics['motion_efficiencies'][-window:]
        
        return {
            'avg_reward': np.mean(recent_rewards),
            'avg_assembly_time': np.mean(recent_times),
            'success_rate': np.mean(recent_success),
            'avg_motion_efficiency': np.mean(recent_efficiency),
            'episodes_window': window
        }
    
    def is_new_best_performance(self, window: int = 100) -> bool:
        """Check if current performance is the best so far"""
        perf = self.get_recent_performance(window)
        if not perf:
            return False
        
        # Performance improvement criteria
        time_improvement = perf['avg_assembly_time'] < self.best_performance['avg_time']
        success_improvement = perf['success_rate'] > self.best_performance['success_rate']
        
        # Combined score: prioritize success rate, then time
        current_score = perf['success_rate'] - (perf['avg_assembly_time'] / 1000.0)  # Normalize time
        best_score = self.best_performance['success_rate'] - (self.best_performance['avg_time'] / 1000.0)
        
        return current_score > best_score + self.config.min_improvement
    
    def update_best_performance(self, episode: int, model_path: str, window: int = 100):
        """Update best performance record"""
        perf = self.get_recent_performance(window)
        if perf:
            self.best_performance.update({
                'avg_time': perf['avg_assembly_time'],
                'success_rate': perf['success_rate'],
                'episode': episode,
                'model_path': model_path
            })
    
    def plot_metrics(self, save_path: str = None):
        """Plot training metrics"""
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        
        # Episode rewards
        axes[0, 0].plot(self.metrics['episode_rewards'])
        axes[0, 0].set_title('Episode Rewards')
        axes[0, 0].set_xlabel('Episode')
        axes[0, 0].set_ylabel('Reward')
        
        # Assembly times
        axes[0, 1].plot(self.metrics['assembly_times'])
        axes[0, 1].axhline(y=self.config.target_assembly_time, color='r', linestyle='--', label='Target (5min)')
        axes[0, 1].set_title('Assembly Times')
        axes[0, 1].set_xlabel('Episode')
        axes[0, 1].set_ylabel('Time (seconds)')
        axes[0, 1].legend()
        
        # Success rate (running average)
        if len(self.metrics['success_rates']) > 100:
            window = 100
            success_avg = [np.mean(self.metrics['success_rates'][max(0, i-window):i+1]) 
                          for i in range(len(self.metrics['success_rates']))]
            axes[0, 2].plot(success_avg)
        else:
            axes[0, 2].plot(self.metrics['success_rates'])
        axes[0, 2].axhline(y=self.config.target_success_rate, color='r', linestyle='--', label='Target (95%)')
        axes[0, 2].set_title('Success Rate')
        axes[0, 2].set_xlabel('Episode')
        axes[0, 2].set_ylabel('Success Rate')
        axes[0, 2].legend()
        
        # Motion efficiency
        axes[1, 0].plot(self.metrics['motion_efficiencies'])
        axes[1, 0].set_title('Motion Efficiency')
        axes[1, 0].set_xlabel('Episode')
        axes[1, 0].set_ylabel('Efficiency')
        
        # Policy loss
        if self.metrics['policy_losses']:
            axes[1, 1].plot(self.metrics['policy_losses'])
            axes[1, 1].set_title('Policy Loss')
            axes[1, 1].set_xlabel('Update Step')
            axes[1, 1].set_ylabel('Loss')
        
        # Value loss
        if self.metrics['value_losses']:
            axes[1, 2].plot(self.metrics['value_losses'])
            axes[1, 2].set_title('Value Loss')
            axes[1, 2].set_xlabel('Update Step')
            axes[1, 2].set_ylabel('Loss')
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path)
            self.logger.info(f"Metrics plot saved to {save_path}")
        else:
            plt.savefig(os.path.join(self.log_dir, 'training_metrics.png'))
        
        plt.close()
    
    def save_metrics(self, filepath: str = None):
        """Save metrics to file"""
        if filepath is None:
            filepath = os.path.join(self.log_dir, 'metrics.json')
        
        metrics_data = {
            'metrics': self.metrics,
            'best_performance': self.best_performance,
            'config': vars(self.config)
        }
        
        with open(filepath, 'w') as f:
            json.dump(metrics_data, f, indent=2, default=str)
        
        self.logger.info(f"Metrics saved to {filepath}")

class Trainer:
    """Main training class for M0609 Block Assembly RL"""
    
    def __init__(self, config: TrainingConfig):
        self.config = config
        
        # Initialize environment
        self.env = M0609BlockAssemblyEnv(
            robot_id=config.robot_id,
            robot_model=config.robot_model,
            virtual_mode=config.virtual_mode
        )
        
        # Initialize agent
        ppo_config = PPOConfig()
        self.agent = M0609PPOAgent(ppo_config, device=config.device)
        
        # Initialize metrics tracker
        self.metrics = MetricsTracker(config)
        
        # Create model save directory
        self.model_dir = os.path.join("models", f"m0609_ppo_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        os.makedirs(self.model_dir, exist_ok=True)
        
        self.metrics.logger.info(f"Trainer initialized with device: {config.device}")
        self.metrics.logger.info(f"Model directory: {self.model_dir}")
    
    def train(self):
        """Main training loop"""
        self.metrics.logger.info("Starting M0609 Block Assembly RL Training")
        self.metrics.logger.info(f"Target: {self.config.baseline_time}s -> {self.config.target_assembly_time}s")
        self.metrics.logger.info(f"Target success rate: {self.config.target_success_rate}")
        
        best_performance_episodes = 0
        
        for episode in range(self.config.max_episodes):
            episode_start_time = time.time()
            
            # Run episode
            episode_reward, episode_length, assembly_time, success, motion_efficiency = self._run_episode()
            
            episode_duration = time.time() - episode_start_time
            
            # Update metrics
            agent_stats = self.agent.get_training_stats()
            self.metrics.update(episode, episode_reward, episode_length, 
                              assembly_time, success, motion_efficiency, agent_stats)
            
            # Periodic logging
            if episode % self.config.log_interval == 0:
                self._log_progress(episode, episode_reward, assembly_time, 
                                 success, motion_efficiency, episode_duration)
            
            # Periodic evaluation and model saving
            if episode % self.config.eval_interval == 0:
                eval_results = self._evaluate_agent()
                self._log_evaluation(episode, eval_results)
                
                # Check for new best performance
                if self.metrics.is_new_best_performance():
                    model_path = os.path.join(self.model_dir, f"best_model_ep_{episode}.pth")
                    self.agent.save_model(model_path)
                    self.metrics.update_best_performance(episode, model_path)
                    self.metrics.logger.info(f"New best performance! Model saved to {model_path}")
                    best_performance_episodes = 0
                else:
                    best_performance_episodes += self.config.eval_interval
            
            # Regular model saving
            if episode % self.config.save_interval == 0:
                model_path = os.path.join(self.model_dir, f"checkpoint_ep_{episode}.pth")
                self.agent.save_model(model_path)
            
            # Update agent (PPO training step)
            self.agent.update()
            
            # Early stopping check
            if best_performance_episodes >= self.config.patience:
                self.metrics.logger.info(f"Early stopping: No improvement for {self.config.patience} episodes")
                break
            
            # Target performance check
            recent_perf = self.metrics.get_recent_performance()
            if (recent_perf and 
                recent_perf['avg_assembly_time'] <= self.config.target_assembly_time and
                recent_perf['success_rate'] >= self.config.target_success_rate):
                self.metrics.logger.info("Target performance achieved!")
                self.metrics.logger.info(f"Average time: {recent_perf['avg_assembly_time']:.1f}s")
                self.metrics.logger.info(f"Success rate: {recent_perf['success_rate']:.3f}")
                break
        
        # Final model save
        final_model_path = os.path.join(self.model_dir, "final_model.pth")
        self.agent.save_model(final_model_path)
        
        # Save training plots and metrics
        self.metrics.plot_metrics()
        self.metrics.save_metrics()
        
        self.metrics.logger.info("Training completed!")
        self._print_training_summary()
    
    def _run_episode(self) -> Tuple[float, int, float, bool, float]:
        """Run a single training episode"""
        obs, info = self.env.reset()
        episode_reward = 0
        episode_length = 0
        motion_efficiencies = []
        
        for step in range(self.config.max_steps_per_episode):
            # Get available blocks mask
            available_blocks = (~self.env.blocks_placed).astype(np.float32)
            
            # Select action
            action, log_prob, value = self.agent.select_action(obs, available_blocks)
            
            # Execute action
            next_obs, reward, terminated, truncated, next_info = self.env.step(action)
            
            # Store experience
            self.agent.store_experience(obs, action, reward, next_obs, 
                                      terminated or truncated, log_prob, value)
            
            episode_reward += reward
            episode_length += 1
            
            # Track motion efficiency
            if 'motion_efficiency' in next_info:
                motion_efficiencies.append(next_info['motion_efficiency'])
            
            obs = next_obs
            
            if terminated or truncated:
                break
        
        # Calculate episode metrics
        assembly_time = next_info.get('episode_time', 0)
        success = next_info.get('success_rate', 0) == 1.0
        avg_motion_efficiency = np.mean(motion_efficiencies) if motion_efficiencies else 0.0
        
        return episode_reward, episode_length, assembly_time, success, avg_motion_efficiency
    
    def _evaluate_agent(self, num_eval_episodes: int = 10) -> Dict:
        """Evaluate agent performance"""
        eval_rewards = []
        eval_times = []
        eval_successes = []
        eval_efficiencies = []
        
        for _ in range(num_eval_episodes):
            obs, _ = self.env.reset()
            episode_reward = 0
            motion_efficiencies = []
            
            for step in range(self.config.max_steps_per_episode):
                available_blocks = (~self.env.blocks_placed).astype(np.float32)
                
                # Use deterministic policy for evaluation
                action, _, _ = self.agent.select_action(obs, available_blocks, deterministic=True)
                
                next_obs, reward, terminated, truncated, info = self.env.step(action)
                
                episode_reward += reward
                if 'motion_efficiency' in info:
                    motion_efficiencies.append(info['motion_efficiency'])
                
                obs = next_obs
                
                if terminated or truncated:
                    break
            
            eval_rewards.append(episode_reward)
            eval_times.append(info.get('episode_time', 0))
            eval_successes.append(info.get('success_rate', 0) == 1.0)
            eval_efficiencies.append(np.mean(motion_efficiencies) if motion_efficiencies else 0.0)
        
        return {
            'avg_reward': np.mean(eval_rewards),
            'std_reward': np.std(eval_rewards),
            'avg_time': np.mean(eval_times),
            'std_time': np.std(eval_times),
            'success_rate': np.mean(eval_successes),
            'avg_efficiency': np.mean(eval_efficiencies)
        }
    
    def _log_progress(self, episode: int, reward: float, assembly_time: float,
                     success: bool, motion_efficiency: float, episode_duration: float):
        """Log training progress"""
        recent_perf = self.metrics.get_recent_performance()
        
        log_msg = f"Episode {episode:6d} | "
        log_msg += f"Reward: {reward:7.1f} | "
        log_msg += f"Time: {assembly_time:5.1f}s | "
        log_msg += f"Success: {'✓' if success else '✗'} | "
        log_msg += f"Efficiency: {motion_efficiency:.3f} | "
        log_msg += f"Duration: {episode_duration:.1f}s"
        
        if recent_perf:
            log_msg += f" || Avg100: R={recent_perf['avg_reward']:.1f}, "
            log_msg += f"T={recent_perf['avg_assembly_time']:.1f}s, "
            log_msg += f"SR={recent_perf['success_rate']:.3f}"
        
        self.metrics.logger.info(log_msg)
    
    def _log_evaluation(self, episode: int, eval_results: Dict):
        """Log evaluation results"""
        log_msg = f"Evaluation @ Episode {episode} | "
        log_msg += f"Avg Reward: {eval_results['avg_reward']:.1f}±{eval_results['std_reward']:.1f} | "
        log_msg += f"Avg Time: {eval_results['avg_time']:.1f}±{eval_results['std_time']:.1f}s | "
        log_msg += f"Success Rate: {eval_results['success_rate']:.3f} | "
        log_msg += f"Avg Efficiency: {eval_results['avg_efficiency']:.3f}"
        
        self.metrics.logger.info(log_msg)
    
    def _print_training_summary(self):
        """Print final training summary"""
        best_perf = self.metrics.best_performance
        recent_perf = self.metrics.get_recent_performance()
        
        self.metrics.logger.info("="*80)
        self.metrics.logger.info("TRAINING SUMMARY")
        self.metrics.logger.info("="*80)
        
        if best_perf['model_path']:
            self.metrics.logger.info(f"Best Performance (Episode {best_perf['episode']}):")
            self.metrics.logger.info(f"  Average Assembly Time: {best_perf['avg_time']:.1f}s")
            self.metrics.logger.info(f"  Success Rate: {best_perf['success_rate']:.3f}")
            self.metrics.logger.info(f"  Model Path: {best_perf['model_path']}")
        
        if recent_perf:
            self.metrics.logger.info(f"Final Performance (Last 100 episodes):")
            self.metrics.logger.info(f"  Average Reward: {recent_perf['avg_reward']:.1f}")
            self.metrics.logger.info(f"  Average Assembly Time: {recent_perf['avg_assembly_time']:.1f}s")
            self.metrics.logger.info(f"  Success Rate: {recent_perf['success_rate']:.3f}")
            self.metrics.logger.info(f"  Average Motion Efficiency: {recent_perf['avg_motion_efficiency']:.3f}")
        
        # Performance improvement
        if best_perf['avg_time'] > 0:
            improvement = ((self.config.baseline_time - best_perf['avg_time']) / 
                          self.config.baseline_time) * 100
            self.metrics.logger.info(f"Time Improvement: {improvement:.1f}% (Target: 50%)")
        
        self.metrics.logger.info(f"Total Episodes: {len(self.metrics.metrics['episode_rewards'])}")
        self.metrics.logger.info(f"Model Directory: {self.model_dir}")
        self.metrics.logger.info(f"Logs Directory: {self.metrics.log_dir}")

def main():
    """Main training function"""
    parser = argparse.ArgumentParser(description="Train M0609 Block Assembly RL Agent")
    
    parser.add_argument("--episodes", type=int, default=50000,
                       help="Maximum number of training episodes")
    parser.add_argument("--robot-id", type=str, default="dsr01",
                       help="Robot ID")
    parser.add_argument("--robot-model", type=str, default="m0609",
                       help="Robot model")
    parser.add_argument("--virtual", action="store_true", default=True,
                       help="Use virtual mode (simulation)")
    parser.add_argument("--device", type=str, default="auto",
                       choices=["auto", "cpu", "cuda"],
                       help="Device to use for training")
    parser.add_argument("--wandb", action="store_true",
                       help="Use Weights & Biases for experiment tracking")
    parser.add_argument("--save-interval", type=int, default=1000,
                       help="Model save interval")
    parser.add_argument("--eval-interval", type=int, default=500,
                       help="Evaluation interval")
    
    args = parser.parse_args()
    
    # Create training configuration
    config = TrainingConfig()
    config.max_episodes = args.episodes
    config.robot_id = args.robot_id
    config.robot_model = args.robot_model
    config.virtual_mode = args.virtual
    config.save_interval = args.save_interval
    config.eval_interval = args.eval_interval
    config.use_wandb = args.wandb
    
    if args.device == "auto":
        config.device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        config.device = args.device
    
    # Create and run trainer
    trainer = Trainer(config)
    
    try:
        trainer.train()
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
        trainer.metrics.logger.info("Training interrupted by user")
        
        # Save current progress
        final_model_path = os.path.join(trainer.model_dir, "interrupted_model.pth")
        trainer.agent.save_model(final_model_path)
        trainer.metrics.plot_metrics()
        trainer.metrics.save_metrics()
        
        trainer.metrics.logger.info(f"Progress saved to {trainer.model_dir}")
    
    except Exception as e:
        trainer.metrics.logger.error(f"Training failed with error: {e}", exc_info=True)
        raise
    
    finally:
        # Clean up
        trainer.env.close()
        if config.use_wandb:
            try:
                wandb.finish()
            except:
                pass

if __name__ == "__main__":
    main()
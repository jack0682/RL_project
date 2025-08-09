"""
Fixed and Optimized Training Script for SOMA Cube Assembly
Corrects all import issues, algorithm inefficiencies, and dependency problems
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
from typing import Dict, List, Tuple, Optional
import json
from dataclasses import dataclass
from collections import deque

# Correct imports - using the fixed implementations
try:
    # Try relative imports first (for ROS2 package)
    from .soma_cube_environment import SOMACubeAssemblyEnv
    from .fixed_ppo_agent import M0609PPOAgent, OptimizedPPOConfig, create_ppo_agent
    from .enhanced_training_logger import EnhancedTrainingLogger, TrainingMetrics
except ImportError:
    # Fallback to direct imports (for standalone execution)
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from soma_cube_environment import SOMACubeAssemblyEnv
    from fixed_ppo_agent import M0609PPOAgent, OptimizedPPOConfig, create_ppo_agent
    from enhanced_training_logger import EnhancedTrainingLogger, TrainingMetrics

@dataclass
class TrainingConfig:
    """Optimized training configuration for SOMA cube assembly"""
    
    # Training parameters - optimized for SOMA cube complexity
    max_episodes: int = 20000  # Reduced from 50k - SOMA cube doesn't need as many
    max_steps_per_episode: int = 50  # Adequate for 7-piece assembly
    eval_interval: int = 500  # More frequent evaluation
    save_interval: int = 1000
    log_interval: int = 100
    
    # Environment parameters
    robot_id: str = "dsr01"
    robot_model: str = "m0609"
    virtual_mode: bool = True
    
    # Performance targets - realistic for SOMA cube
    target_success_rate: float = 0.85  # 85% is good for complex assembly
    target_steps_per_success: int = 20  # Average steps to complete
    
    # Training monitoring
    patience: int = 2000  # Early stopping patience
    min_improvement: float = 0.02  # Minimum improvement threshold
    
    # Hardware settings
    device: str = None  # Will be auto-detected
    
    # Logging and experiment tracking
    log_dir: str = "logs"
    model_dir: str = "models"
    use_tensorboard: bool = True
    
    def __post_init__(self):
        if self.device is None:
            self.device = "cuda" if torch.cuda.is_available() else "cpu"

class MetricsTracker:
    """Enhanced metrics tracking for SOMA cube assembly"""
    
    def __init__(self, config: TrainingConfig):
        self.config = config
        
        # Core metrics
        self.episode_rewards = deque(maxlen=1000)
        self.episode_lengths = deque(maxlen=1000)
        self.success_rates = deque(maxlen=100)
        self.assembly_completion_times = deque(maxlen=1000)
        
        # RL-specific metrics
        self.policy_losses = deque(maxlen=100)
        self.value_losses = deque(maxlen=100)
        self.entropies = deque(maxlen=100)
        self.kl_divergences = deque(maxlen=100)
        
        # SOMA cube specific metrics
        self.pieces_placed_per_episode = deque(maxlen=1000)
        self.invalid_action_rates = deque(maxlen=100)
        
        # Performance tracking
        self.best_performance = {
            'success_rate': 0.0,
            'avg_steps_to_success': float('inf'),
            'episode': 0,
            'model_path': None
        }
        
        # Setup logging
        self.setup_logging()
        
        # Setup tensorboard if requested
        if self.config.use_tensorboard:
            self.setup_tensorboard()
    
    def setup_logging(self):
        """Setup structured logging"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = os.path.join(self.config.log_dir, f"soma_cube_training_{timestamp}")
        os.makedirs(self.log_dir, exist_ok=True)
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(os.path.join(self.log_dir, 'training.log')),
                logging.StreamHandler()
            ]
        )
        
        self.logger = logging.getLogger(__name__)
        self.logger.info("=== SOMA Cube Assembly Training Started ===")
        self.logger.info(f"Device: {self.config.device}")
        self.logger.info(f"Log directory: {self.log_dir}")
    
    def setup_tensorboard(self):
        """Setup tensorboard logging"""
        try:
            from torch.utils.tensorboard import SummaryWriter
            tensorboard_dir = os.path.join(self.log_dir, "tensorboard")
            self.writer = SummaryWriter(tensorboard_dir)
            self.logger.info(f"Tensorboard logging to: {tensorboard_dir}")
            self.logger.info("Start tensorboard with: tensorboard --logdir " + tensorboard_dir)
        except ImportError:
            self.logger.warning("Tensorboard not available. Install with: pip install tensorboard")
            self.writer = None
    
    def log_episode(self, episode: int, reward: float, steps: int, success: bool, 
                   pieces_placed: int, training_stats: Dict = None):
        """Log episode results"""
        self.episode_rewards.append(reward)
        self.episode_lengths.append(steps)
        self.pieces_placed_per_episode.append(pieces_placed)
        
        # Update success rate (rolling average over last 100 episodes)
        recent_episodes = min(100, len(self.episode_rewards))
        recent_successes = sum(1 for r in list(self.episode_rewards)[-recent_episodes:] if r > 50)
        success_rate = recent_successes / recent_episodes
        self.success_rates.append(success_rate)
        
        # Log to console every log_interval episodes
        if episode % self.config.log_interval == 0:
            avg_reward = np.mean(list(self.episode_rewards)[-100:])
            avg_steps = np.mean(list(self.episode_lengths)[-100:])
            avg_pieces = np.mean(list(self.pieces_placed_per_episode)[-100:])
            
            self.logger.info(
                f"Episode {episode:6d} | "
                f"Reward: {reward:8.2f} | "
                f"Steps: {steps:3d} | "
                f"Success: {success} | "
                f"Pieces: {pieces_placed}/7 | "
                f"Avg100: R={avg_reward:6.2f}, S={avg_steps:5.1f}, SR={success_rate:.2%}"
            )
        
        # Log training stats if provided
        if training_stats:
            for key, value in training_stats.items():
                if hasattr(self, key + 's'):
                    getattr(self, key + 's').append(value)
        
        # Tensorboard logging
        if self.writer and episode % 10 == 0:  # Log every 10 episodes to reduce overhead
            self.writer.add_scalar('Episode/Reward', reward, episode)
            self.writer.add_scalar('Episode/Steps', steps, episode)
            self.writer.add_scalar('Episode/Success_Rate', success_rate, episode)
            self.writer.add_scalar('Episode/Pieces_Placed', pieces_placed, episode)
            
            if training_stats:
                for key, value in training_stats.items():
                    self.writer.add_scalar(f'Training/{key.title()}', value, episode)
    
    def should_stop_early(self, episode: int) -> bool:
        """Check if training should stop early"""
        if len(self.success_rates) < 100:
            return False
        
        current_sr = self.success_rates[-1]
        
        # Check if we've reached target performance
        if current_sr >= self.config.target_success_rate:
            avg_steps = np.mean(list(self.episode_lengths)[-100:])
            if avg_steps <= self.config.target_steps_per_success:
                self.logger.info(
                    f"Early stopping: Reached target performance "
                    f"(SR={current_sr:.2%}, Steps={avg_steps:.1f})"
                )
                return True
        
        # Check for lack of improvement
        if len(self.success_rates) >= self.config.patience:
            old_sr = np.mean(list(self.success_rates)[-self.config.patience:-self.config.patience//2])
            new_sr = np.mean(list(self.success_rates)[-self.config.patience//2:])
            
            if new_sr - old_sr < self.config.min_improvement:
                self.logger.info(
                    f"Early stopping: No improvement in {self.config.patience} episodes "
                    f"(SR: {old_sr:.2%} -> {new_sr:.2%})"
                )
                return True
        
        return False
    
    def save_model_if_best(self, episode: int, agent: M0609PPOAgent) -> bool:
        """Save model if it's the best so far"""
        if len(self.success_rates) < 50:  # Need enough data
            return False
        
        current_sr = self.success_rates[-1]
        avg_steps = np.mean(list(self.episode_lengths)[-50:])  # Recent average
        
        # Check if this is the best performance
        is_best = (current_sr > self.best_performance['success_rate'] or 
                  (current_sr >= self.best_performance['success_rate'] - 0.02 and 
                   avg_steps < self.best_performance['avg_steps_to_success']))
        
        if is_best:
            model_dir = os.path.join(self.config.model_dir, "best")
            os.makedirs(model_dir, exist_ok=True)
            model_path = os.path.join(model_dir, f"best_model_ep_{episode}.pth")
            
            agent.save_model(model_path)
            
            self.best_performance.update({
                'success_rate': current_sr,
                'avg_steps_to_success': avg_steps,
                'episode': episode,
                'model_path': model_path
            })
            
            self.logger.info(
                f"New best model saved: SR={current_sr:.2%}, "
                f"Avg Steps={avg_steps:.1f} -> {model_path}"
            )
            
            return True
        
        return False
    
    def close(self):
        """Close tensorboard writer"""
        if hasattr(self, 'writer') and self.writer:
            self.writer.close()

def create_training_environment(config: TrainingConfig) -> SOMACubeAssemblyEnv:
    """Create optimized training environment"""
    env = SOMACubeAssemblyEnv(
        robot_id=config.robot_id,
        robot_model=config.robot_model,
        virtual_mode=config.virtual_mode,
        unit_cube_size=30.0,  # 30mm cubes
        render_mode=None  # No rendering during training
    )
    
    logging.info(f"Created SOMA Cube environment:")
    logging.info(f"  Observation space: {env.observation_space}")
    logging.info(f"  Action space: {env.action_space}")
    
    return env

def run_evaluation(agent: M0609PPOAgent, env: SOMACubeAssemblyEnv, 
                  num_episodes: int = 10) -> Dict[str, float]:
    """Run evaluation episodes"""
    eval_rewards = []
    eval_steps = []
    eval_successes = []
    eval_pieces_placed = []
    
    for ep in range(num_episodes):
        obs, info = env.reset()
        episode_reward = 0
        steps = 0
        
        while steps < env.max_steps:
            action, action_info = agent.select_action(obs)
            obs, reward, terminated, truncated, info = env.step(action)
            
            episode_reward += reward
            steps += 1
            
            if terminated or truncated:
                break
        
        eval_rewards.append(episode_reward)
        eval_steps.append(steps)
        eval_successes.append(info.get('assembly_complete', False))
        eval_pieces_placed.append(info.get('pieces_placed', 0))
    
    return {
        'avg_reward': np.mean(eval_rewards),
        'avg_steps': np.mean(eval_steps),
        'success_rate': np.mean(eval_successes),
        'avg_pieces_placed': np.mean(eval_pieces_placed)
    }

def train_soma_cube_assembly(config: TrainingConfig = None) -> M0609PPOAgent:
    """
    Main training function for SOMA cube assembly
    
    Returns:
        Trained PPO agent
    """
    if config is None:
        config = TrainingConfig()
    
    # Create environment and agent
    env = create_training_environment(config)
    
    # Create optimized PPO agent
    ppo_config = OptimizedPPOConfig(
        learning_rate=1e-4,  # Conservative for robotics
        gamma=0.99,         # High discount for long sequences
        batch_size=128,     # Moderate batch size
        update_epochs=getattr(config, 'ppo_epochs', 4),  # Use config or default to 4
        entropy_coef=0.02   # Encourage exploration
    )
    
    agent = create_ppo_agent(env, ppo_config, config.device)
    
    # Create enhanced logger
    logger = EnhancedTrainingLogger(
        experiment_name="soma_cube_assembly",
        log_dir=config.log_dir,
        config=config.__dict__,
        enable_tensorboard=config.use_tensorboard
    )
    
    logger.logger.info("=== Starting SOMA Cube Assembly Training ===")
    logger.logger.info(f"Target: {config.target_success_rate:.1%} success rate")
    logger.logger.info(f"Max episodes: {config.max_episodes}")
    
    # Training loop
    try:
        for episode in range(1, config.max_episodes + 1):
            # Reset environment
            obs, info = env.reset()
            episode_reward = 0
            steps = 0
            
            # Episode loop
            while steps < config.max_steps_per_episode:
                # Select action
                action, action_info = agent.select_action(obs)
                
                # Execute action
                next_obs, reward, terminated, truncated, step_info = env.step(action)
                
                # Store experience
                agent.store_experience(obs, action, reward, terminated or truncated, action_info)
                
                # Update state
                obs = next_obs
                episode_reward += reward
                steps += 1
                
                if terminated or truncated:
                    break
            
            # Update agent
            training_stats = agent.update()
            
            # Log episode
            success = step_info.get('assembly_complete', False)
            pieces_placed = step_info.get('pieces_placed', 0)
            completion_rate = pieces_placed / 7.0  # SOMA cube has 7 pieces
            
            # Create training metrics
            episode_metrics = TrainingMetrics(
                episode=episode,
                timestamp=time.time(),
                success=success,
                total_reward=episode_reward,
                steps_taken=steps,
                pieces_placed=pieces_placed,
                completion_rate=completion_rate,
                curriculum_stage="BASIC_PLACEMENT"  # Can be updated based on curriculum
            )
            
            # Log with training stats
            logger.log_episode(episode_metrics, training_stats)
            
            # Save best model
            if episode % config.save_interval == 0:
                model_dir = os.path.join(config.model_dir, "checkpoints")
                os.makedirs(model_dir, exist_ok=True)
                model_path = os.path.join(model_dir, f"model_ep_{episode}.pth")
                agent.save_model(model_path)
                logger.logger.info(f"Model saved: {model_path}")
            
            # Run evaluation
            if episode % config.eval_interval == 0:
                eval_results = run_evaluation(agent, env, num_episodes=5)
                logger.logger.info(
                    f"Evaluation Ep {episode}: "
                    f"SR={eval_results['success_rate']:.2%}, "
                    f"Reward={eval_results['avg_reward']:.2f}, "
                    f"Steps={eval_results['avg_steps']:.1f}"
                )
                
                # Save best model based on success rate
                if eval_results['success_rate'] > getattr(train_soma_cube_assembly, '_best_sr', 0.0):
                    train_soma_cube_assembly._best_sr = eval_results['success_rate']
                    best_model_path = os.path.join(config.model_dir, "best_model.pth")
                    os.makedirs(os.path.dirname(best_model_path), exist_ok=True)
                    agent.save_model(best_model_path)
                    logger.logger.info(f"New best model saved: {best_model_path} (SR: {eval_results['success_rate']:.2%})")
            
            # Simple early stopping based on episode count for now
            # TODO: Implement sophisticated early stopping based on performance metrics
    
    except KeyboardInterrupt:
        logger.logger.info("Training interrupted by user")
    
    except Exception as e:
        logger.logger.error(f"Training error: {e}")
        import traceback
        logger.logger.error(f"Full traceback: {traceback.format_exc()}")
        raise
    
    finally:
        # Final model save
        final_model_path = os.path.join(config.model_dir, "final_model.pth")
        os.makedirs(os.path.dirname(final_model_path), exist_ok=True)
        agent.save_model(final_model_path)
        logger.logger.info(f"Final model saved: {final_model_path}")
        
        # Generate final summary and close logger
        summary = logger.close()
        
        # Display summary information
        if summary and 'experiment_info' in summary:
            info = summary['experiment_info']
            logger.logger.info(f"Training completed. Total episodes: {info['total_episodes']}")
            logger.logger.info(f"Final success rate: {info['success_rate']:.2%}")
            
            if 'performance_metrics' in summary:
                perf = summary['performance_metrics']
                logger.logger.info(f"Average reward: {perf['average_reward']:.2f}")
                logger.logger.info(f"Average steps per episode: {perf['average_steps']:.1f}")
        
        logger.logger.info("All logs and metrics have been saved to: " + str(logger.log_dir))
    
    return agent

def main():
    """Command line interface"""
    parser = argparse.ArgumentParser(description='SOMA Cube Assembly RL Training')
    
    parser.add_argument('--episodes', type=int, default=20000,
                       help='Maximum training episodes')
    parser.add_argument('--ppo-epochs', type=int, default=4,
                       help='PPO update epochs per training step (default: 4, recommended: 4-10)')
    parser.add_argument('--device', choices=['cpu', 'cuda', 'auto'], default='auto',
                       help='Training device')
    parser.add_argument('--robot-id', default='dsr01',
                       help='Robot identifier')
    parser.add_argument('--robot-model', default='m0609',
                       help='Robot model')
    parser.add_argument('--virtual', action='store_true', default=True,
                       help='Use virtual robot mode')
    parser.add_argument('--real', action='store_true',
                       help='Use real robot (overrides --virtual)')
    parser.add_argument('--log-dir', default='logs',
                       help='Logging directory')
    parser.add_argument('--model-dir', default='models',
                       help='Model save directory')
    parser.add_argument('--no-tensorboard', action='store_true',
                       help='Disable tensorboard logging')
    
    args = parser.parse_args()
    
    # Create config
    config = TrainingConfig()
    config.max_episodes = args.episodes
    config.ppo_epochs = args.ppo_epochs  # PPO training epochs
    config.robot_id = args.robot_id
    config.robot_model = args.robot_model
    config.virtual_mode = not args.real  # Real mode overrides virtual
    config.log_dir = args.log_dir
    config.model_dir = args.model_dir
    config.use_tensorboard = not args.no_tensorboard
    
    if args.device != 'auto':
        config.device = args.device
    
    print("=== SOMA Cube Assembly RL Training ===")
    print(f"Episodes: {config.max_episodes}")
    print(f"Device: {config.device}")
    print(f"Robot: {config.robot_model} ({config.robot_id})")
    print(f"Mode: {'Virtual' if config.virtual_mode else 'Real Robot'}")
    print(f"Logs: {config.log_dir}")
    print(f"Models: {config.model_dir}")
    
    # Start training
    agent = train_soma_cube_assembly(config)
    
    print("Training completed successfully!")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
PPO Training Script for SOMA Cube Assembly using Stable-Baselines3
Based on the paper "High-Speed Autonomous Robotic Assembly Using In-Hand Manipulation and Re-Grasping"
"""

import os
import numpy as np
import torch
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.callbacks import (
    BaseCallback, EvalCallback, CheckpointCallback, 
    CallbackList, StopTrainingOnRewardThreshold
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.logger import configure
from stable_baselines3.common.policies import ActorCriticPolicy
import argparse
import time
from pathlib import Path
from typing import Dict, Any

# Import our custom environment
from soma_cube_gym_env import SOMACubeEnv

class SOMAAssemblyCallback(BaseCallback):
    """Custom callback for SOMA cube assembly training"""
    
    def __init__(self, eval_freq: int = 10000, verbose: int = 1):
        super(SOMAAssemblyCallback, self).__init__(verbose)
        self.eval_freq = eval_freq
        self.best_mean_reward = -np.inf
        self.assembly_success_rates = []
        
    def _on_step(self) -> bool:
        if self.n_calls % self.eval_freq == 0:
            # Log training progress
            if hasattr(self.locals, 'infos') and self.locals['infos']:
                infos = self.locals['infos']
                
                # Calculate success rate from recent episodes
                recent_successes = []
                for info in infos:
                    if 'assembly_complete' in info:
                        recent_successes.append(1 if info['assembly_complete'] else 0)
                
                if recent_successes:
                    success_rate = np.mean(recent_successes)
                    self.assembly_success_rates.append(success_rate)
                    
                    if self.verbose > 0:
                        print(f"Step {self.n_calls}: Assembly success rate = {success_rate:.3f}")
                        
                        # Log additional metrics
                        pieces_placed = [info.get('pieces_placed', 0) for info in infos]
                        avg_pieces = np.mean(pieces_placed) if pieces_placed else 0
                        print(f"Average pieces placed: {avg_pieces:.2f}")
        
        return True

class SOMATrainingConfig:
    """Configuration for SOMA cube PPO training"""
    
    def __init__(self):
        # PPO hyperparameters optimized for robotic assembly
        self.learning_rate = 3e-4
        self.n_steps = 2048         # Steps per rollout
        self.batch_size = 64        # Mini-batch size
        self.n_epochs = 100          # Training epochs per rollout
        self.gamma = 0.99           # Discount factor
        self.gae_lambda = 0.95      # GAE parameter
        self.clip_range = 0.2       # PPO clipping parameter
        self.ent_coef = 0.01        # Entropy bonus coefficient
        self.vf_coef = 0.5          # Value function coefficient
        self.max_grad_norm = 0.5    # Gradient clipping
        
        # Training parameters
        self.total_timesteps = 1000000  # 1M timesteps
        self.eval_freq = 25000          # Evaluation frequency
        self.save_freq = 50000          # Checkpoint frequency
        self.n_eval_episodes = 20       # Episodes per evaluation
        
        # Environment parameters
        self.n_envs = 4                 # Number of parallel environments (reduced for stability)
        self.max_episode_steps = 200    # Max steps per episode (shorter for faster training)
        self.enable_re_grasp = True     # Enable re-grasping capability
        
        # Device configuration
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

def create_soma_env(config: SOMATrainingConfig, env_id: int = 0):
    """Create a single SOMA cube environment"""
    def _init():
        env_config = {
            'render_mode': 'rgb_array',  # For headless training
            'max_steps': config.max_episode_steps,
            'enable_re_grasp': config.enable_re_grasp,
            'seed': 42 + env_id  # Different seed for each env
        }
        env = SOMACubeEnv(env_config)
        env = Monitor(env)  # Wrap with Monitor for logging
        return env
    return _init

def setup_logging(experiment_name: str) -> str:
    """Setup training logs directory"""
    log_dir = f"logs/{experiment_name}_{int(time.time())}"
    os.makedirs(log_dir, exist_ok=True)
    return log_dir

def create_custom_policy():
    """Create custom policy network for SOMA assembly"""
    
    class SOMAAssemblyPolicy(ActorCriticPolicy):
        """Custom policy for SOMA cube assembly with attention mechanism"""
        
        def __init__(self, *args, **kwargs):
            # Use a larger network for complex assembly task (fixed for SB3 v1.8.0+)
            kwargs['net_arch'] = {'pi': [256, 128, 64], 'vf': [256, 128, 64]}  # Dict format
            kwargs['activation_fn'] = torch.nn.ReLU
            super(SOMAAssemblyPolicy, self).__init__(*args, **kwargs)
    
    return SOMAAssemblyPolicy

def train_soma_ppo(config: SOMATrainingConfig, experiment_name: str = "soma_ppo"):
    """Main training function for SOMA cube assembly with PPO"""
    
    print("=== SOMA Cube Assembly PPO Training ===")
    print(f"Device: {config.device}")
    print(f"Parallel environments: {config.n_envs}")
    print(f"Total timesteps: {config.total_timesteps:,}")
    print(f"Re-grasping enabled: {config.enable_re_grasp}")
    
    # Setup directories
    log_dir = setup_logging(experiment_name)
    model_dir = f"models/{experiment_name}"
    os.makedirs(model_dir, exist_ok=True)
    
    print(f"Logs: {log_dir}")
    print(f"Models: {model_dir}")
    
    # Create vectorized training environments
    if config.n_envs > 1:
        train_env = SubprocVecEnv([
            create_soma_env(config, i) for i in range(config.n_envs)
        ])
    else:
        train_env = DummyVecEnv([create_soma_env(config)])
    
    # Create evaluation environment  
    eval_env = DummyVecEnv([create_soma_env(config)])
    
    # Setup custom policy
    policy_class = create_custom_policy()
    
    # Create PPO model with optimized hyperparameters for robotic assembly
    model = PPO(
        policy_class,
        train_env,
        learning_rate=config.learning_rate,
        n_steps=config.n_steps,
        batch_size=config.batch_size,
        n_epochs=config.n_epochs,
        gamma=config.gamma,
        gae_lambda=config.gae_lambda,
        clip_range=config.clip_range,
        ent_coef=config.ent_coef,
        vf_coef=config.vf_coef,
        max_grad_norm=config.max_grad_norm,
        device=config.device,
        verbose=1,
        tensorboard_log=log_dir
    )
    
    # Setup callbacks
    callbacks = []
    
    # Custom SOMA assembly callback
    soma_callback = SOMAAssemblyCallback(eval_freq=config.eval_freq//config.n_envs)
    callbacks.append(soma_callback)
    
    # Evaluation callback
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=f"{model_dir}/best_model",
        log_path=f"{log_dir}/eval_logs",
        eval_freq=config.eval_freq//config.n_envs,  # Adjust for vectorized env
        n_eval_episodes=config.n_eval_episodes,
        deterministic=True,
        render=False
    )
    callbacks.append(eval_callback)
    
    # Checkpoint callback
    checkpoint_callback = CheckpointCallback(
        save_freq=config.save_freq//config.n_envs,
        save_path=f"{model_dir}/checkpoints",
        name_prefix="soma_ppo"
    )
    callbacks.append(checkpoint_callback)
    
    # Stop training on reward threshold (successful assembly gives +100 reward)
    stop_callback = StopTrainingOnRewardThreshold(
        reward_threshold=80.0,  # Target 80% success rate
        verbose=1
    )
    callbacks.append(stop_callback)
    
    callback_list = CallbackList(callbacks)
    
    # Configure logger
    model.set_logger(configure(log_dir, ["stdout", "csv", "tensorboard"]))
    
    print("\n=== Starting Training ===")
    start_time = time.time()
    
    try:
        # Train the model (disable progress bar to avoid dependency issues)
        model.learn(
            total_timesteps=config.total_timesteps,
            callback=callback_list,
            progress_bar=False
        )
        
        training_time = time.time() - start_time
        print(f"\n=== Training Completed ===")
        print(f"Training time: {training_time/3600:.2f} hours")
        
        # Save final model
        final_model_path = f"{model_dir}/final_model"
        model.save(final_model_path)
        print(f"Final model saved to: {final_model_path}")
        
        # Final evaluation
        print("\n=== Final Evaluation ===")
        evaluate_soma_model(model, eval_env, n_episodes=50)
        
    except KeyboardInterrupt:
        print("\n=== Training Interrupted ===")
        model.save(f"{model_dir}/interrupted_model")
        print("Model saved before exit")
    
    finally:
        # Clean up environments
        train_env.close()
        eval_env.close()
    
    return model

def evaluate_soma_model(model, env, n_episodes: int = 20) -> Dict[str, float]:
    """Evaluate trained SOMA assembly model"""
    
    print(f"Evaluating model for {n_episodes} episodes...")
    
    episode_rewards = []
    assembly_successes = []
    pieces_placed_list = []
    re_grasp_counts = []
    
    for episode in range(n_episodes):
        obs = env.reset()
        episode_reward = 0
        pieces_placed = 0
        re_grasps = 0
        done = False
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward[0]  # Extract reward from vector
            
            # Track metrics from info
            if len(info) > 0 and info[0]:
                pieces_placed = info[0].get('pieces_placed', 0)
                if info[0].get('re_grasp_attempted', False):
                    re_grasps += 1
        
        # Check if assembly was completed
        assembly_success = pieces_placed == 7
        
        episode_rewards.append(episode_reward)
        assembly_successes.append(assembly_success)
        pieces_placed_list.append(pieces_placed)
        re_grasp_counts.append(re_grasps)
        
        if episode % 5 == 0:
            print(f"Episode {episode}: Reward={episode_reward:.1f}, "
                  f"Pieces={pieces_placed}/7, Success={assembly_success}")
    
    # Calculate statistics
    results = {
        'mean_reward': np.mean(episode_rewards),
        'std_reward': np.std(episode_rewards),
        'success_rate': np.mean(assembly_successes),
        'mean_pieces_placed': np.mean(pieces_placed_list),
        'mean_re_grasps': np.mean(re_grasp_counts),
        'assembly_time_estimate': len(pieces_placed_list) * 10  # Rough estimate in seconds
    }
    
    print("\n=== Evaluation Results ===")
    print(f"Success Rate: {results['success_rate']:.1%}")
    print(f"Average Reward: {results['mean_reward']:.2f} ± {results['std_reward']:.2f}")
    print(f"Average Pieces Placed: {results['mean_pieces_placed']:.1f}/7")
    print(f"Average Re-grasps per Episode: {results['mean_re_grasps']:.1f}")
    print(f"Estimated Assembly Time: {results['assembly_time_estimate']:.0f}s")
    
    # Compare with paper results
    print(f"\n=== Comparison with Paper Results ===")
    print(f"Paper Success Rate: 95%")
    print(f"Our Success Rate: {results['success_rate']:.1%}")
    print(f"Paper Re-grasp Rate: 25%")
    print(f"Our Re-grasp Rate: {results['mean_re_grasps']/7:.1%}")
    
    return results

def main():
    """Main function with command line interface"""
    
    parser = argparse.ArgumentParser(description="Train PPO for SOMA Cube Assembly")
    parser.add_argument('--experiment-name', default='soma_ppo_training', 
                       help='Experiment name')
    parser.add_argument('--timesteps', type=int, default=1000000,
                       help='Total training timesteps')
    parser.add_argument('--n-envs', type=int, default=8,
                       help='Number of parallel environments')
    parser.add_argument('--learning-rate', type=float, default=3e-4,
                       help='Learning rate')
    parser.add_argument('--no-re-grasp', action='store_true',
                       help='Disable re-grasping capability')
    parser.add_argument('--eval-only', type=str, default=None,
                       help='Path to model for evaluation only')
    parser.add_argument('--device', default='auto',
                       choices=['auto', 'cpu', 'cuda'],
                       help='Training device')
    
    args = parser.parse_args()
    
    # Create configuration
    config = SOMATrainingConfig()
    config.total_timesteps = args.timesteps
    config.n_envs = args.n_envs
    config.learning_rate = args.learning_rate
    config.enable_re_grasp = not args.no_re_grasp
    
    if args.device != 'auto':
        config.device = args.device
    
    print(f"Configuration: {vars(config)}")
    
    if args.eval_only:
        # Evaluation only mode
        print(f"Loading model from: {args.eval_only}")
        model = PPO.load(args.eval_only)
        
        # Create evaluation environment
        eval_env = DummyVecEnv([create_soma_env(config)])
        evaluate_soma_model(model, eval_env, n_episodes=50)
        eval_env.close()
    else:
        # Training mode
        model = train_soma_ppo(config, args.experiment_name)

if __name__ == "__main__":
    main()
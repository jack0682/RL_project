#!/usr/bin/env python3
"""
Simplified PPO Training Script for SOMA Cube Assembly
Single environment training to avoid multiprocessing issues
"""

import os
import numpy as np
import torch
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import (
    BaseCallback, EvalCallback, CheckpointCallback, 
    CallbackList
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.logger import configure
import argparse
import time
from pathlib import Path
from typing import Dict, Any

# Import our custom environment
from soma_cube_gym_env import SOMACubeEnv

class SimplifiedSOMACallback(BaseCallback):
    """Simplified callback for SOMA cube assembly training"""
    
    def __init__(self, eval_freq: int = 5000, verbose: int = 1):
        super(SimplifiedSOMACallback, self).__init__(verbose)
        self.eval_freq = eval_freq
        self.episode_rewards = []
        self.success_count = 0
        self.total_episodes = 0
        
    def _on_step(self) -> bool:
        # Log every 1000 steps
        if self.n_calls % 1000 == 0:
            if self.verbose > 0:
                print(f"Step {self.n_calls}: Training in progress...")
        
        # Check for episode completion
        if hasattr(self.locals, 'infos') and self.locals['infos']:
            for info in self.locals['infos']:
                if info is not None and 'assembly_complete' in info:
                    self.total_episodes += 1
                    if info['assembly_complete']:
                        self.success_count += 1
                    
                    # Log success rate every 10 episodes
                    if self.total_episodes % 10 == 0:
                        success_rate = self.success_count / self.total_episodes
                        print(f"Episodes {self.total_episodes}: Success rate = {success_rate:.2%}")
        
        return True

def create_simple_soma_env(config: Dict):
    """Create a single SOMA cube environment"""
    def _init():
        env = SOMACubeEnv(config)
        env = Monitor(env)
        return env
    return _init

def train_soma_simple(experiment_name: str = "simple_soma", timesteps: int = 100000):
    """Simplified training function using single environment"""
    
    print("=== Simplified SOMA Cube Training ===")
    print(f"Device: {'cuda' if torch.cuda.is_available() else 'cpu'}")
    print(f"Total timesteps: {timesteps:,}")
    
    # Setup directories
    log_dir = f"logs/{experiment_name}_{int(time.time())}"
    model_dir = f"models/{experiment_name}"
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(model_dir, exist_ok=True)
    
    print(f"Logs: {log_dir}")
    print(f"Models: {model_dir}")
    
    # Environment configuration
    env_config = {
        'render_mode': 'rgb_array',  # Headless training
        'max_steps': 100,            # Shorter episodes
        'enable_re_grasp': True
    }
    
    # Create single environment (no multiprocessing)
    env = DummyVecEnv([create_simple_soma_env(env_config)])
    eval_env = DummyVecEnv([create_simple_soma_env(env_config)])
    
    # Create PPO model with conservative settings
    model = PPO(
        'MultiInputPolicy',  # Use built-in policy for Dict observations, epoch change
        env,
        learning_rate=3e-4,
        n_steps=512,         # Shorter rollouts for single env
        batch_size=32,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.01,
        vf_coef=0.5,
        max_grad_norm=0.5,
        device='cuda' if torch.cuda.is_available() else 'cpu',
        verbose=1,
        tensorboard_log=log_dir
    )
    
    # Setup callbacks
    callbacks = []
    
    # Custom callback
    soma_callback = SimplifiedSOMACallback(eval_freq=5000)
    callbacks.append(soma_callback)
    
    # Evaluation callback (smaller frequency for single env)
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=f"{model_dir}/best_model",
        log_path=f"{log_dir}/eval_logs",
        eval_freq=2000,  # More frequent evaluation
        n_eval_episodes=10,  # Fewer episodes per eval
        deterministic=True,
        render=False
    )
    callbacks.append(eval_callback)
    
    # Checkpoint callback
    checkpoint_callback = CheckpointCallback(
        save_freq=5000,
        save_path=f"{model_dir}/checkpoints",
        name_prefix="simple_soma"
    )
    callbacks.append(checkpoint_callback)
    
    callback_list = CallbackList(callbacks)
    
    # Configure logger
    model.set_logger(configure(log_dir, ["stdout", "csv", "tensorboard"]))
    
    print("\n=== Starting Training ===")
    start_time = time.time()
    
    try:
        # Train the model (disable progress bar to avoid import issues)
        model.learn(
            total_timesteps=timesteps,
            callback=callback_list,
            progress_bar=False
        )
        
        training_time = time.time() - start_time
        print(f"\n=== Training Completed ===")
        print(f"Training time: {training_time/60:.1f} minutes")
        
        # Save final model
        final_model_path = f"{model_dir}/final_model"
        model.save(final_model_path)
        print(f"Final model saved to: {final_model_path}")
        
        # Quick evaluation
        print("\n=== Quick Evaluation ===")
        evaluate_simple_model(model, eval_env, n_episodes=20)
        
    except KeyboardInterrupt:
        print("\n=== Training Interrupted ===")
        model.save(f"{model_dir}/interrupted_model")
        print("Model saved before exit")
    
    finally:
        # Clean up environments
        env.close()
        eval_env.close()
    
    return model

def evaluate_simple_model(model, env, n_episodes: int = 10) -> Dict[str, float]:
    """Evaluate the trained model"""
    
    print(f"Evaluating for {n_episodes} episodes...")
    
    episode_rewards = []
    assembly_successes = []
    pieces_placed_list = []
    
    for episode in range(n_episodes):
        obs = env.reset()
        episode_reward = 0
        done = False
        pieces_placed = 0
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward[0]
            
            # Track pieces placed
            if len(info) > 0 and info[0]:
                pieces_placed = info[0].get('pieces_placed', 0)
        
        assembly_success = pieces_placed == 7
        
        episode_rewards.append(episode_reward)
        assembly_successes.append(assembly_success)
        pieces_placed_list.append(pieces_placed)
        
        print(f"Episode {episode+1}: Reward={episode_reward:.1f}, "
              f"Pieces={pieces_placed}/7, Success={assembly_success}")
    
    # Calculate statistics
    results = {
        'mean_reward': np.mean(episode_rewards),
        'std_reward': np.std(episode_rewards),
        'success_rate': np.mean(assembly_successes),
        'mean_pieces_placed': np.mean(pieces_placed_list)
    }
    
    print("\n=== Evaluation Results ===")
    print(f"Success Rate: {results['success_rate']:.1%}")
    print(f"Average Reward: {results['mean_reward']:.2f} ± {results['std_reward']:.2f}")
    print(f"Average Pieces Placed: {results['mean_pieces_placed']:.1f}/7")
    
    return results

def main():
    """Main function with simplified command line interface"""
    
    parser = argparse.ArgumentParser(description="Simplified PPO Training for SOMA Cube")
    parser.add_argument('--experiment-name', default='simple_soma', 
                       help='Experiment name')
    parser.add_argument('--timesteps', type=int, default=100000,
                       help='Total training timesteps (default: 100k)')
    parser.add_argument('--eval-only', type=str, default=None,
                       help='Path to model for evaluation only')
    
    args = parser.parse_args()
    
    if args.eval_only:
        # Evaluation only mode
        print(f"Loading model from: {args.eval_only}")
        model = PPO.load(args.eval_only)
        
        # Create evaluation environment
        env_config = {'render_mode': 'rgb_array', 'max_steps': 100, 'enable_re_grasp': True}
        eval_env = DummyVecEnv([create_simple_soma_env(env_config)])
        evaluate_simple_model(model, eval_env, n_episodes=20)
        eval_env.close()
    else:
        # Training mode
        model = train_soma_simple(args.experiment_name, args.timesteps)

if __name__ == "__main__":
    main()
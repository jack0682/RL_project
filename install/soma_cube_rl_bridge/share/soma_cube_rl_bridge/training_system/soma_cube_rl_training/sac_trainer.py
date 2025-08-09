#!/usr/bin/env python3
"""
Soft Actor-Critic (SAC) trainer for SomaCube assembly task
Uses Stable Baselines3 for the RL algorithm implementation
"""

import numpy as np
import torch
import rclpy
from stable_baselines3 import SAC
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
import gym
from gym import spaces
import os
import matplotlib.pyplot as plt
from datetime import datetime
import json
import time
from typing import Optional

import sys
import os

# Add the training system directory to Python path
training_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, training_dir)

from rl_client import SomaCubeRLClient


class SomaCubeGymEnv(gym.Env):
    """
    Gym wrapper for SomaCube RL environment
    Converts ROS2 service calls to standard Gym interface
    """
    
    def __init__(self, max_episode_steps: int = 500, render_mode: Optional[str] = None):
        super().__init__()
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        self.client = SomaCubeRLClient()
        self.max_episode_steps = max_episode_steps
        self.current_step = 0
        self.episode_count = 0
        self.render_mode = render_mode
        
        # Define action and observation spaces
        action_info = self.client.get_action_space_info()
        obs_info = self.client.get_observation_space_info()
        
        self.action_space = spaces.Box(
            low=action_info["low"] * 0.1,  # Conservative action scaling for safety
            high=action_info["high"] * 0.1,
            shape=action_info["shape"],
            dtype=np.float32
        )
        
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=obs_info["shape"],
            dtype=np.float32
        )
        
        print(f"🤖 SomaCube Gym Environment initialized")
        print(f"   Action space: {self.action_space}")
        print(f"   Observation space: {self.observation_space}")
    
    def reset(self, seed=None, **kwargs):
        """Reset the environment"""
        super().reset(seed=seed)
        
        self.current_step = 0
        self.episode_count += 1
        
        try:
            obs = self.client.reset(seed=seed)
            return obs, {}
        except Exception as e:
            print(f"❌ Reset failed: {e}")
            # Return dummy observation if reset fails
            return np.zeros(self.observation_space.shape, dtype=np.float32), {}
    
    def step(self, action):
        """Execute one step"""
        self.current_step += 1
        
        try:
            obs, reward, done, info = self.client.step(action)
            
            # Add truncation handling for max episode steps
            truncated = self.current_step >= self.max_episode_steps
            if truncated and not done:
                done = True
                info["TimeLimit.truncated"] = True
            
            return obs, reward, done, truncated, info
            
        except Exception as e:
            print(f"❌ Step failed: {e}")
            # Return terminal state on error
            obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            return obs, -100.0, True, True, {"error": str(e)}
    
    def render(self):
        """Render the environment (placeholder)"""
        if self.render_mode == "human":
            stats = self.client.get_stats()
            print(f"Episode: {stats['episodes_completed']}, "
                  f"Step: {stats['current_episode_steps']}, "
                  f"Reward: {stats['current_episode_reward']:.2f}")
    
    def close(self):
        """Clean up"""
        self.client.close()


class TrainingCallback(BaseCallback):
    """
    Callback for monitoring training progress
    """
    
    def __init__(self, verbose=1, save_freq=10000, log_dir="./logs/"):
        super().__init__(verbose)
        self.save_freq = save_freq
        self.log_dir = log_dir
        self.episode_rewards = []
        self.episode_lengths = []
        self.safety_violations = []
        
        # Create log directory
        os.makedirs(log_dir, exist_ok=True)
    
    def _on_step(self) -> bool:
        """Called after each step"""
        # Save model periodically
        if self.n_calls % self.save_freq == 0:
            model_path = os.path.join(self.log_dir, f"sac_somacube_{self.n_calls}.zip")
            self.model.save(model_path)
            print(f"💾 Model saved at step {self.n_calls}: {model_path}")
        
        return True
    
    def _on_rollout_end(self) -> None:
        """Called at the end of each rollout"""
        if len(self.logger.name_to_value) > 0:
            # Log training metrics
            ep_reward = self.logger.name_to_value.get("rollout/ep_rew_mean", 0)
            ep_length = self.logger.name_to_value.get("rollout/ep_len_mean", 0)
            
            self.episode_rewards.append(ep_reward)
            self.episode_lengths.append(ep_length)
            
            # Print progress
            if self.verbose > 0:
                print(f"🔄 Step {self.n_calls}: "
                      f"Reward: {ep_reward:.2f}, "
                      f"Length: {ep_length:.1f}")


class SACTrainer:
    """
    SAC trainer for SomaCube assembly
    """
    
    def __init__(self, 
                 log_dir: str = "./training_logs",
                 max_episode_steps: int = 500,
                 device: str = "auto"):
        
        self.log_dir = log_dir
        self.max_episode_steps = max_episode_steps
        self.device = device
        
        # Create log directory
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = os.path.join(log_dir, f"sac_somacube_{timestamp}")
        os.makedirs(self.run_dir, exist_ok=True)
        
        print(f"🎯 SAC Trainer initialized")
        print(f"   Log directory: {self.run_dir}")
        print(f"   Max episode steps: {max_episode_steps}")
        print(f"   Device: {device}")
        
        # Setup environment
        self.setup_environment()
        
        # Setup SAC model
        self.setup_model()
    
    def setup_environment(self):
        """Setup the training environment"""
        print("🌍 Setting up environment...")
        
        # Create environment
        env = SomaCubeGymEnv(max_episode_steps=self.max_episode_steps)
        
        # Wrap with monitor for logging
        env = Monitor(env, self.run_dir)
        
        # Vectorize environment
        self.env = DummyVecEnv([lambda: env])
        
        # Check environment
        try:
            check_env(env)
            print("✅ Environment validation passed")
        except Exception as e:
            print(f"⚠️ Environment validation warning: {e}")
    
    def setup_model(self):
        """Setup SAC model"""
        print("🧠 Setting up SAC model...")
        
        # SAC hyperparameters tuned for robotic control
        self.model = SAC(
            "MlpPolicy",
            self.env,
            learning_rate=3e-4,
            buffer_size=100000,
            learning_starts=10000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            train_freq=1,
            gradient_steps=1,
            ent_coef="auto",
            target_update_interval=1,
            target_entropy="auto",
            use_sde=False,
            sde_sample_freq=-1,
            use_sde_at_warmup=False,
            policy_kwargs=dict(
                net_arch=[400, 300],  # Network architecture
                activation_fn=torch.nn.ReLU,
            ),
            verbose=1,
            device=self.device,
            tensorboard_log=self.run_dir
        )
        
        print("✅ SAC model created")
        print(f"   Policy network: {self.model.policy}")
        print(f"   Buffer size: {self.model.buffer_size}")
    
    def train(self, total_timesteps: int = 500000, save_freq: int = 50000):
        """Train the model"""
        print(f"🚀 Starting training for {total_timesteps} timesteps...")
        
        # Setup callbacks
        callback = TrainingCallback(
            verbose=1,
            save_freq=save_freq,
            log_dir=self.run_dir
        )
        
        # Save training configuration
        config = {
            "algorithm": "SAC",
            "total_timesteps": total_timesteps,
            "max_episode_steps": self.max_episode_steps,
            "learning_rate": self.model.learning_rate,
            "buffer_size": self.model.buffer_size,
            "batch_size": self.model.batch_size,
            "gamma": self.model.gamma,
            "tau": self.model.tau,
            "device": str(self.device),
            "start_time": datetime.now().isoformat()
        }
        
        with open(os.path.join(self.run_dir, "config.json"), "w") as f:
            json.dump(config, f, indent=2)
        
        try:
            # Start training
            start_time = time.time()
            self.model.learn(
                total_timesteps=total_timesteps,
                callback=callback,
                tb_log_name="SAC",
                reset_num_timesteps=True
            )
            
            training_time = time.time() - start_time
            
            # Save final model
            final_model_path = os.path.join(self.run_dir, "final_model.zip")
            self.model.save(final_model_path)
            
            print(f"✅ Training completed in {training_time:.2f} seconds")
            print(f"💾 Final model saved: {final_model_path}")
            
            # Save training summary
            summary = {
                "training_time_seconds": training_time,
                "final_model_path": final_model_path,
                "total_timesteps": total_timesteps,
                "end_time": datetime.now().isoformat()
            }
            
            with open(os.path.join(self.run_dir, "summary.json"), "w") as f:
                json.dump(summary, f, indent=2)
                
        except KeyboardInterrupt:
            print("\n⚠️ Training interrupted by user")
            interrupt_model_path = os.path.join(self.run_dir, "interrupted_model.zip")
            self.model.save(interrupt_model_path)
            print(f"💾 Model saved at interruption: {interrupt_model_path}")
        
        except Exception as e:
            print(f"❌ Training failed: {e}")
            raise
    
    def evaluate(self, n_episodes: int = 10, render: bool = False):
        """Evaluate the trained model"""
        print(f"📊 Evaluating model for {n_episodes} episodes...")
        
        episode_rewards = []
        episode_lengths = []
        
        for episode in range(n_episodes):
            obs = self.env.reset()
            done = False
            episode_reward = 0
            episode_length = 0
            
            while not done:
                action, _ = self.model.predict(obs, deterministic=True)
                obs, reward, done, info = self.env.step(action)
                episode_reward += reward[0]
                episode_length += 1
                
                if render:
                    self.env.render()
            
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            
            print(f"Episode {episode + 1}: Reward: {episode_reward:.2f}, Length: {episode_length}")
        
        # Calculate statistics
        mean_reward = np.mean(episode_rewards)
        std_reward = np.std(episode_rewards)
        mean_length = np.mean(episode_lengths)
        
        print(f"\n📈 Evaluation Results:")
        print(f"   Mean reward: {mean_reward:.2f} ± {std_reward:.2f}")
        print(f"   Mean episode length: {mean_length:.2f}")
        
        return {
            "mean_reward": mean_reward,
            "std_reward": std_reward,
            "mean_length": mean_length,
            "episode_rewards": episode_rewards,
            "episode_lengths": episode_lengths
        }
    
    def close(self):
        """Clean up resources"""
        print("🔚 Closing trainer...")
        if hasattr(self, 'env'):
            self.env.close()


def main():
    """Main training script"""
    print("🎯 SomaCube SAC Training System")
    print("=" * 50)
    
    # Training configuration
    TOTAL_TIMESTEPS = 500000  # Adjust based on your needs
    MAX_EPISODE_STEPS = 500   # Adjust based on task complexity
    SAVE_FREQ = 50000        # Save model every 50k steps
    
    try:
        # Initialize trainer
        trainer = SACTrainer(
            log_dir="./somacube_training",
            max_episode_steps=MAX_EPISODE_STEPS,
            device="cuda" if torch.cuda.is_available() else "cpu"
        )
        
        # Start training
        trainer.train(
            total_timesteps=TOTAL_TIMESTEPS,
            save_freq=SAVE_FREQ
        )
        
        # Evaluate final model
        trainer.evaluate(n_episodes=10, render=True)
        
    except Exception as e:
        print(f"❌ Training failed: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if 'trainer' in locals():
            trainer.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
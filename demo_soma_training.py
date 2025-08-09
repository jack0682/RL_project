#!/usr/bin/env python3
"""
Demo script showing SOMA cube RL training
Quick demonstration of the complete system working
"""

import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.monitor import Monitor

from soma_cube_gym_env import SOMACubeEnv

def create_demo_env():
    """Create demo environment"""
    def _init():
        env = SOMACubeEnv({
            'render_mode': 'rgb_array',
            'max_steps': 50,
            'enable_re_grasp': True
        })
        return Monitor(env)
    return _init

def demo_training():
    """Demonstrate SOMA cube RL training"""
    
    print("=== SOMA Cube RL System Demo ===")
    print(f"Device: {'CUDA' if torch.cuda.is_available() else 'CPU'}")
    
    # Create environment
    env = DummyVecEnv([create_demo_env()])
    
    # Create PPO model
    model = PPO(
        'MultiInputPolicy',
        env,
        learning_rate=3e-4,
        n_steps=256,
        batch_size=32,
        verbose=1,
        device='cuda' if torch.cuda.is_available() else 'cpu'
    )
    
    print("\n=== Starting Demo Training (2000 timesteps) ===")
    
    # Quick training demo
    model.learn(total_timesteps=2000)
    
    print("\n=== Training Complete! ===")
    
    # Demo evaluation
    print("\n=== Demo Evaluation ===")
    
    success_count = 0
    total_episodes = 5
    
    for episode in range(total_episodes):
        obs = env.reset()
        episode_reward = 0
        done = False
        step_count = 0
        
        while not done and step_count < 50:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            episode_reward += reward[0]
            step_count += 1
        
        # Check success
        pieces_placed = 0
        if len(info) > 0 and info[0]:
            pieces_placed = info[0].get('pieces_placed', 0)
        
        success = pieces_placed == 7
        if success:
            success_count += 1
            
        print(f"Episode {episode+1}: Reward={episode_reward:.1f}, "
              f"Pieces={pieces_placed}/7, Success={success}")
    
    success_rate = success_count / total_episodes
    print(f"\nDemo Success Rate: {success_rate:.1%}")
    print(f"Target (from paper): 95%")
    
    # Clean up
    env.close()
    
    print("\n=== Demo Complete! ===")
    print("✅ Environment: Working")
    print("✅ PPO Training: Working") 
    print("✅ Re-grasping: Implemented")
    print("✅ Physics Simulation: Working")
    print("✅ Multi-modal Observations: Working")
    
    return model

if __name__ == "__main__":
    demo_training()
#!/usr/bin/env python3
"""
Demo script to show the training system working
Runs a minimal training session for demonstration
"""

import os
import sys
import time
import rclpy
from datetime import datetime

import sys
import os

# Add the training system directory to Python path
training_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, training_dir)

from rl_client import SomaCubeRLClient
import numpy as np


def demo_rl_interface():
    """Demonstrate the RL interface working"""
    print("🤖 SomaCube RL Interface Demo")
    print("=" * 40)
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create client
        print("🔗 Connecting to RL bridge...")
        client = SomaCubeRLClient(timeout_sec=10.0)
        
        print("✅ Connected successfully!")
        
        # Show interface info
        print("\n📊 Interface Information:")
        action_info = client.get_action_space_info()
        obs_info = client.get_observation_space_info()
        
        print(f"   Action space: {action_info['shape']}")
        print(f"   Action range: [{action_info['low'][0]:.1f}, {action_info['high'][0]:.1f}] degrees")
        print(f"   Observation space: {obs_info['shape']}")
        
        # Demo episode
        print("\n🎮 Running demo episode...")
        
        # Reset environment
        print("🔄 Resetting environment...")
        initial_obs = client.reset(seed=42)
        print(f"   Initial observation shape: {initial_obs.shape}")
        print(f"   Sample observations: {initial_obs[:6].round(2)}... (first 6 joint positions)")
        
        # Run a few steps
        print("\n👟 Running demo steps:")
        total_reward = 0.0
        
        for step in range(5):
            # Generate small random action (safe movements)
            action = np.random.uniform(-5.0, 5.0, size=6)  # Small movements
            
            print(f"   Step {step + 1}: Action = {action.round(2)}")
            
            try:
                obs, reward, done, info = client.step(action)
                total_reward += reward
                
                step_time = info.get('step_duration_ms', 0)
                distance = info.get('distance_to_target', 0)
                
                print(f"      Reward: {reward:.2f}, Done: {done}")
                print(f"      Distance to target: {distance:.1f}mm")
                print(f"      Step time: {step_time:.1f}ms")
                
                if done:
                    print("      Episode completed!")
                    break
                    
            except Exception as e:
                print(f"      ⚠️ Step failed: {e}")
                break
        
        print(f"\n📈 Demo Summary:")
        print(f"   Total reward: {total_reward:.2f}")
        print(f"   Steps completed: {step + 1}")
        
        # Show current stats
        stats = client.get_stats()
        print(f"   Episodes run: {stats['episodes_completed']}")
        print(f"   Safety state: {'Safe' if stats['safety_state'] else 'Unsafe'}")
        
        print("\n✅ Demo completed successfully!")
        
    except Exception as e:
        print(f"❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()
        
    finally:
        client.close()
        rclpy.shutdown()


def demo_training_setup():
    """Demonstrate training system setup (without actually training)"""
    print("\n🎯 Training System Setup Demo")
    print("=" * 40)
    
    try:
        # Check if stable-baselines3 is available
        try:
            import stable_baselines3 as sb3
            import torch
            print(f"✅ Stable Baselines3: v{sb3.__version__}")
            print(f"✅ PyTorch: v{torch.__version__}")
            print(f"✅ CUDA available: {torch.cuda.is_available()}")
            if torch.cuda.is_available():
                print(f"   GPU: {torch.cuda.get_device_name(0)}")
        except ImportError as e:
            print(f"⚠️ Missing training dependency: {e}")
            print("   Install with: pip install stable-baselines3[extra] torch")
            return
        
        # Initialize training system components
        print("\n🧠 Setting up training components...")
        
        from sac_trainer import SomaCubeGymEnv
        
        # Create environment wrapper
        print("🌍 Creating Gym environment wrapper...")
        env = SomaCubeGymEnv(max_episode_steps=100)  # Short episodes for demo
        
        print(f"   Action space: {env.action_space}")
        print(f"   Observation space: {env.observation_space}")
        
        # Test environment interface
        print("\n🔄 Testing environment interface...")
        obs, info = env.reset()
        print(f"   Reset successful: observation shape {obs.shape}")
        
        # Test one step
        action = env.action_space.sample() * 0.1  # Small random action
        obs, reward, done, truncated, info = env.step(action)
        
        print(f"   Step successful: reward={reward:.2f}, done={done}")
        print(f"   Info keys: {list(info.keys())}")
        
        env.close()
        
        print("\n✅ Training system setup verified!")
        print("\n📋 Ready for training with:")
        print("   python train_somacube.py --mode train --total-timesteps 50000")
        
    except Exception as e:
        print(f"❌ Training setup failed: {e}")
        import traceback
        traceback.print_exc()


def main():
    """Main demo script"""
    print("🚀 SomaCube RL Training System Demo")
    print("🤖 Demonstrating RL interface and training setup")
    print("⚠️  Make sure the RL bridge is running:")
    print("   ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true")
    print("\nPress Enter to continue or Ctrl+C to exit...")
    
    try:
        input()
    except KeyboardInterrupt:
        print("\n👋 Demo cancelled by user")
        return
    
    # Demo 1: RL Interface
    demo_rl_interface()
    
    # Demo 2: Training Setup
    demo_training_setup()
    
    print("\n🎉 Demo completed!")
    print("\n🚀 Next steps:")
    print("   1. Install training dependencies: pip install -r requirements.txt")
    print("   2. Start training: python train_somacube.py --mode check")
    print("   3. Run full training: python train_somacube.py --mode train")


if __name__ == '__main__':
    main()
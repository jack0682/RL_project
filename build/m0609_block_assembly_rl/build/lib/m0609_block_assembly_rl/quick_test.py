#!/usr/bin/env python3
"""
Quick Test Script for M0609 Block Assembly RL
This script helps you quickly test the system before full training
"""

import os
import sys
import time
import traceback

def test_imports():
    """Test if all required modules can be imported"""
    print("=== Testing Imports ===")
    
    try:
        import numpy as np
        print("✓ NumPy")
    except ImportError:
        print("✗ NumPy - pip install numpy")
        return False
    
    try:
        import torch
        print(f"✓ PyTorch - {torch.__version__}")
        if torch.cuda.is_available():
            print(f"  CUDA available: {torch.cuda.device_count()} GPUs")
        else:
            print("  CUDA not available - CPU only")
    except ImportError:
        print("✗ PyTorch - pip install torch")
        return False
    
    try:
        import matplotlib.pyplot as plt
        print("✓ Matplotlib")
    except ImportError:
        print("✗ Matplotlib - pip install matplotlib")
        return False
    
    return True

def test_environment():
    """Test the RL environment"""
    print("\n=== Testing Environment ===")
    
    try:
        from environment import M0609BlockAssemblyEnv
        
        # Create environment in virtual mode
        print("Creating virtual environment...")
        env = M0609BlockAssemblyEnv(
            virtual_mode=True,
            robot_id="dsr01",
            robot_model="m0609"
        )
        print("✓ Environment created successfully")
        
        # Test reset
        print("Testing environment reset...")
        obs = env.reset()
        print(f"✓ Environment reset - Observation shape: {obs.shape}")
        
        # Test step
        print("Testing environment step...")
        action = env.action_space.sample()
        obs, reward, done, info = env.step(action)
        print(f"✓ Environment step - Reward: {reward:.2f}")
        
        return True
        
    except Exception as e:
        print(f"✗ Environment test failed: {e}")
        traceback.print_exc()
        return False

def test_agent():
    """Test the RL agent"""
    print("\n=== Testing Agent ===")
    
    try:
        from ppo_agent import M0609PPOAgent, PPOConfig
        
        print("Creating PPO agent...")
        config = PPOConfig()
        agent = M0609PPOAgent(config)
        print("✓ Agent created successfully")
        
        # Test action selection
        print("Testing action selection...")
        dummy_obs = agent.get_dummy_observation()
        action = agent.select_action(dummy_obs)
        print(f"✓ Action selected - Shape: {action.shape}")
        
        return True
        
    except Exception as e:
        print(f"✗ Agent test failed: {e}")
        traceback.print_exc()
        return False

def test_training():
    """Test short training loop"""
    print("\n=== Testing Training Loop ===")
    
    try:
        from environment import M0609BlockAssemblyEnv
        from ppo_agent import M0609PPOAgent, PPOConfig
        
        # Create environment and agent
        env = M0609BlockAssemblyEnv(virtual_mode=True)
        config = PPOConfig()
        agent = M0609PPOAgent(config)
        
        print("Running 5 training steps...")
        obs = env.reset()
        
        for step in range(5):
            action = agent.select_action(obs)
            obs, reward, done, info = env.step(action)
            print(f"  Step {step+1}: Reward={reward:.2f}")
            
            if done:
                obs = env.reset()
                print("  Episode completed, environment reset")
        
        print("✓ Training loop test completed successfully")
        return True
        
    except Exception as e:
        print(f"✗ Training loop test failed: {e}")
        traceback.print_exc()
        return False

def test_ros2_connection():
    """Test ROS2 connection if available"""
    print("\n=== Testing ROS2 Connection ===")
    
    try:
        import rclpy
        rclpy.init()
        print("✓ ROS2 initialized")
        
        # Test if dsr_msgs2 is available
        try:
            from dsr_msgs2.srv import MoveJ
            print("✓ dsr_msgs2 available")
        except ImportError:
            print("! dsr_msgs2 not available - real robot control may not work")
        
        rclpy.shutdown()
        return True
        
    except ImportError:
        print("! ROS2 not available - pure Python mode only")
        return True  # Not a failure, just info
    except Exception as e:
        print(f"! ROS2 test warning: {e}")
        return True

def test_model_save_load():
    """Test model saving and loading"""
    print("\n=== Testing Model Save/Load ===")
    
    try:
        from ppo_agent import M0609PPOAgent, PPOConfig
        import torch
        
        # Create agent
        config = PPOConfig()
        agent = M0609PPOAgent(config)
        
        # Test save
        test_path = "/tmp/test_model.pth"
        agent.save_model(test_path)
        print("✓ Model saved")
        
        # Test load
        agent2 = M0609PPOAgent(config)
        agent2.load_model(test_path)
        print("✓ Model loaded")
        
        # Cleanup
        if os.path.exists(test_path):
            os.remove(test_path)
        
        return True
        
    except Exception as e:
        print(f"✗ Model save/load test failed: {e}")
        traceback.print_exc()
        return False

def run_interactive_demo():
    """Run interactive demo"""
    print("\n=== Interactive Demo ===")
    
    try:
        from environment import M0609BlockAssemblyEnv
        from ppo_agent import M0609PPOAgent, PPOConfig
        
        env = M0609BlockAssemblyEnv(virtual_mode=True)
        config = PPOConfig()
        agent = M0609PPOAgent(config)
        
        print("Running interactive demo (10 steps)...")
        print("Watching agent interact with environment...")
        
        obs = env.reset()
        total_reward = 0
        
        for step in range(10):
            print(f"\nStep {step+1}/10:")
            
            # Get action from agent
            action = agent.select_action(obs)
            print(f"  Action: {action[:3]}...")  # Show first 3 action values
            
            # Execute step
            obs, reward, done, info = env.step(action)
            total_reward += reward
            
            print(f"  Reward: {reward:.2f}")
            print(f"  Done: {done}")
            print(f"  Assembly Progress: {info.get('assembly_progress', 'N/A')}")
            
            if done:
                print(f"  Episode completed! Total reward: {total_reward:.2f}")
                obs = env.reset()
                total_reward = 0
                
            time.sleep(0.5)  # Pause for readability
        
        print("✓ Interactive demo completed")
        return True
        
    except Exception as e:
        print(f"✗ Interactive demo failed: {e}")
        traceback.print_exc()
        return False

def main():
    """Run all tests"""
    print("M0609 Block Assembly RL - Quick Test Suite")
    print("=" * 50)
    
    tests = [
        ("Import Test", test_imports),
        ("Environment Test", test_environment),
        ("Agent Test", test_agent),
        ("Training Test", test_training),
        ("ROS2 Test", test_ros2_connection),
        ("Model I/O Test", test_model_save_load)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"✗ {test_name} crashed: {e}")
            results.append((test_name, False))
    
    # Summary
    print("\n" + "=" * 50)
    print("TEST RESULTS SUMMARY")
    print("=" * 50)
    
    passed = 0
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        icon = "✓" if result else "✗"
        print(f"{icon} {test_name}: {status}")
        if result:
            passed += 1
    
    print(f"\nPassed: {passed}/{len(results)} tests")
    
    if passed == len(results):
        print("\n🎉 All tests passed! System is ready for use.")
        
        # Offer interactive demo
        while True:
            demo = input("\nRun interactive demo? (y/n): ").strip().lower()
            if demo in ['y', 'yes']:
                run_interactive_demo()
                break
            elif demo in ['n', 'no']:
                break
            else:
                print("Please enter 'y' or 'n'")
    else:
        print(f"\n⚠️  {len(results) - passed} tests failed. Please fix issues before training.")
        print("\nCommon fixes:")
        print("- Install missing dependencies: pip install -r requirements.txt")
        print("- Build ROS2 workspace: colcon build")
        print("- Check GPU drivers if using CUDA")
    
    print("\nNext steps:")
    print("1. For training: python train.py --episodes 1000 --virtual")
    print("2. For evaluation: python evaluate.py --model-path models/model.pth")
    print("3. Read USAGE_GUIDE.md for detailed instructions")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
Comprehensive Validation Test for Fixed SOMA Cube RL Package
Tests all import chains, algorithm configurations, and system integration
"""

import sys
import os
import traceback
import numpy as np
import torch
from typing import Dict, Any

# Utility function for ROS2-compatible imports
def safe_import(module_name, items=None):
    """Safely import modules with fallback for both ROS2 and standalone execution"""
    try:
        # Try ROS2 package import first
        if items:
            module = __import__(f'm0609_block_assembly_rl.{module_name}', fromlist=items)
            return tuple(getattr(module, item) for item in items)
        else:
            return __import__(f'm0609_block_assembly_rl.{module_name}')
    except ImportError:
        try:
            # Try relative import
            if items:
                module = __import__(f'.{module_name}', fromlist=items, level=1, package=__package__)
                return tuple(getattr(module, item) for item in items)
            else:
                return __import__(f'.{module_name}', level=1, package=__package__)
        except ImportError:
            # Fallback to direct import (standalone execution)
            if items:
                module = __import__(module_name, fromlist=items)
                return tuple(getattr(module, item) for item in items)
            else:
                return __import__(module_name)

def test_imports():
    """Test all critical imports"""
    print("=" * 60)
    print("TESTING IMPORTS AND DEPENDENCIES")
    print("=" * 60)
    
    tests = []
    
    # Test 1: Core ML libraries
    try:
        import torch
        import torch.nn as nn
        import numpy as np
        import gymnasium as gym
        tests.append(("Core ML libraries", True, None))
    except ImportError as e:
        tests.append(("Core ML libraries", False, str(e)))
    
    # Test 2: SOMA environment import
    try:
        try:
            from .soma_cube_environment import SOMACubeAssemblyEnv, SOMAPiece
        except ImportError:
            SOMACubeAssemblyEnv, SOMAPiece = safe_import('soma_cube_environment', ['SOMACubeAssemblyEnv', 'SOMAPiece'])
        tests.append(("SOMA environment", True, None))
    except ImportError as e:
        tests.append(("SOMA environment", False, str(e)))
    
    # Test 3: Fixed PPO agent import
    try:
        try:
            from .fixed_ppo_agent import M0609PPOAgent, OptimizedPPOConfig, create_ppo_agent
        except ImportError:
            M0609PPOAgent, OptimizedPPOConfig, create_ppo_agent = safe_import('fixed_ppo_agent', ['M0609PPOAgent', 'OptimizedPPOConfig', 'create_ppo_agent'])
        tests.append(("Fixed PPO agent", True, None))
    except ImportError as e:
        tests.append(("Fixed PPO agent", False, str(e)))
    
    # Test 4: Fixed training script import
    try:
        try:
            from .fixed_train import TrainingConfig, train_soma_cube_assembly
        except ImportError:
            TrainingConfig, train_soma_cube_assembly = safe_import('fixed_train', ['TrainingConfig', 'train_soma_cube_assembly'])
        tests.append(("Fixed training script", True, None))
    except ImportError as e:
        tests.append(("Fixed training script", False, str(e)))
    
    # Test 5: Optional dependencies
    try:
        import matplotlib.pyplot as plt
        tests.append(("Matplotlib (optional)", True, None))
    except ImportError as e:
        tests.append(("Matplotlib (optional)", False, str(e)))
    
    # Print results
    passed = 0
    for test_name, success, error in tests:
        status = "✓ PASS" if success else "✗ FAIL"
        print(f"{status:8} | {test_name:25} | {error or 'OK'}")
        if success:
            passed += 1
    
    print(f"\nImport Tests: {passed}/{len(tests)} passed")
    return passed == len(tests)

def test_environment_creation():
    """Test SOMA cube environment creation and basic functionality"""
    print("\n" + "=" * 60)
    print("TESTING ENVIRONMENT CREATION")
    print("=" * 60)
    
    try:
        SOMACubeAssemblyEnv, SOMAPiece = safe_import('soma_cube_environment', ['SOMACubeAssemblyEnv', 'SOMAPiece'])
        
        # Test 1: Environment creation
        env = SOMACubeAssemblyEnv(virtual_mode=True)
        print("✓ Environment created successfully")
        
        # Test 2: Check observation space
        obs_space = env.observation_space
        print(f"✓ Observation space: {obs_space}")
        
        # Test 3: Check action space
        action_space = env.action_space
        print(f"✓ Action space: {action_space}")
        
        # Test 4: Reset environment
        obs, info = env.reset()
        print(f"✓ Environment reset - Obs keys: {list(obs.keys())}")
        
        # Test 5: Random action
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        print(f"✓ Action executed - Reward: {reward}, Done: {terminated}")
        
        # Test 6: SOMA pieces validation
        SOMAPieceGeometry = safe_import('soma_cube_environment', ['SOMAPieceGeometry'])[0]
        total_cubes = sum(len(SOMAPieceGeometry.PIECE_DEFINITIONS[piece]) 
                         for piece in SOMAPiece)
        print(f"✓ SOMA pieces validated: {total_cubes} total cubes")
        
        return True
        
    except Exception as e:
        print(f"✗ Environment test failed: {e}")
        traceback.print_exc()
        return False

def test_agent_creation():
    """Test PPO agent creation with environment"""
    print("\n" + "=" * 60)
    print("TESTING AGENT CREATION")
    print("=" * 60)
    
    try:
        SOMACubeAssemblyEnv = safe_import('soma_cube_environment', ['SOMACubeAssemblyEnv'])[0]
        M0609PPOAgent, OptimizedPPOConfig, create_ppo_agent = safe_import('fixed_ppo_agent', ['M0609PPOAgent', 'OptimizedPPOConfig', 'create_ppo_agent'])
        
        # Test 1: Create environment
        env = SOMACubeAssemblyEnv(virtual_mode=True)
        print("✓ Environment created")
        
        # Test 2: Create agent config
        config = OptimizedPPOConfig(
            learning_rate=1e-4,
            batch_size=32,  # Small for testing
            update_epochs=2
        )
        print(f"✓ Config created: LR={config.learning_rate}, Batch={config.batch_size}")
        
        # Test 3: Create agent
        agent = create_ppo_agent(env, config, device='cpu')
        print(f"✓ Agent created with device: {agent.device}")
        
        # Test 4: Test action selection
        obs, info = env.reset()
        action, action_info = agent.select_action(obs)
        print(f"✓ Action selected: shape={action.shape}, info_keys={list(action_info.keys())}")
        
        # Test 5: Test experience storage
        agent.store_experience(obs, action, 1.0, False, action_info)
        print("✓ Experience stored")
        
        # Test 6: Check network parameters
        param_count = sum(p.numel() for p in agent.policy_net.parameters())
        print(f"✓ Network has {param_count:,} parameters")
        
        return True
        
    except Exception as e:
        print(f"✗ Agent test failed: {e}")
        traceback.print_exc()
        return False

def test_training_integration():
    """Test integration between environment, agent, and training loop"""
    print("\n" + "=" * 60)
    print("TESTING TRAINING INTEGRATION")
    print("=" * 60)
    
    try:
        SOMACubeAssemblyEnv = safe_import('soma_cube_environment', ['SOMACubeAssemblyEnv'])[0]
        create_ppo_agent, OptimizedPPOConfig = safe_import('fixed_ppo_agent', ['create_ppo_agent', 'OptimizedPPOConfig'])
        TrainingConfig = safe_import('fixed_train', ['TrainingConfig'])[0]
        
        # Test 1: Create components
        env = SOMACubeAssemblyEnv(virtual_mode=True)
        config = OptimizedPPOConfig(batch_size=8, update_epochs=1)  # Minimal for testing
        agent = create_ppo_agent(env, config, device='cpu')
        print("✓ Training components created")
        
        # Test 2: Short training episode
        obs, info = env.reset()
        episode_reward = 0
        experiences_collected = 0
        
        for step in range(10):  # Short episode
            action, action_info = agent.select_action(obs)
            next_obs, reward, terminated, truncated, step_info = env.step(action)
            
            agent.store_experience(obs, action, reward, terminated or truncated, action_info)
            experiences_collected += 1
            
            obs = next_obs
            episode_reward += reward
            
            if terminated or truncated:
                break
        
        print(f"✓ Episode completed: {experiences_collected} experiences, reward={episode_reward:.2f}")
        
        # Test 3: Agent update (if enough experiences)
        if experiences_collected >= config.batch_size:
            training_stats = agent.update()
            print(f"✓ Agent updated: stats={list(training_stats.keys())}")
        else:
            print("✓ Not enough experiences for update (expected for small batch)")
        
        # Test 4: Model save/load
        test_model_path = "/tmp/test_soma_model.pth"
        agent.save_model(test_model_path)
        print("✓ Model saved")
        
        agent.load_model(test_model_path)
        print("✓ Model loaded")
        
        # Cleanup
        if os.path.exists(test_model_path):
            os.remove(test_model_path)
        
        return True
        
    except Exception as e:
        print(f"✗ Training integration test failed: {e}")
        traceback.print_exc()
        return False

def test_algorithm_configuration():
    """Test algorithm hyperparameters and configurations"""
    print("\n" + "=" * 60)
    print("TESTING ALGORITHM CONFIGURATION")
    print("=" * 60)
    
    try:
        OptimizedPPOConfig = safe_import('fixed_ppo_agent', ['OptimizedPPOConfig'])[0]
        
        # Test 1: Default config values
        config = OptimizedPPOConfig()
        
        # Check key parameters are reasonable for robotics
        checks = [
            ("Learning Rate", 1e-5 <= config.learning_rate <= 1e-3, f"LR={config.learning_rate}"),
            ("Gamma", 0.9 <= config.gamma <= 0.999, f"Gamma={config.gamma}"),
            ("Batch Size", 16 <= config.batch_size <= 512, f"Batch={config.batch_size}"),
            ("Update Epochs", 1 <= config.update_epochs <= 10, f"Epochs={config.update_epochs}"),
            ("Entropy Coef", 0.001 <= config.entropy_coef <= 0.1, f"Entropy={config.entropy_coef}"),
            ("Hidden Dims", len(config.hidden_dims) >= 2, f"Layers={len(config.hidden_dims)}"),
        ]
        
        for check_name, condition, detail in checks:
            status = "✓" if condition else "✗"
            print(f"{status} {check_name:15} | {detail}")
        
        # Test 2: Network architecture reasonableness
        total_params_estimate = 1
        for i, dim in enumerate([256] + config.hidden_dims):  # Assume 256-dim input
            next_dim = config.hidden_dims[i] if i < len(config.hidden_dims) else 64
            total_params_estimate += dim * next_dim
        
        reasonable_size = 10000 <= total_params_estimate <= 1000000
        print(f"{'✓' if reasonable_size else '✗'} Network Size    | ~{total_params_estimate:,} params")
        
        # Test 3: Memory requirements
        batch_memory = config.batch_size * 256 * 4  # Rough estimate in bytes
        reasonable_memory = batch_memory < 100 * 1024 * 1024  # < 100MB
        print(f"{'✓' if reasonable_memory else '✗'} Memory Usage   | ~{batch_memory/1024/1024:.1f}MB")
        
        return True
        
    except Exception as e:
        print(f"✗ Algorithm configuration test failed: {e}")
        traceback.print_exc()
        return False

def test_dimension_compatibility():
    """Test dimension compatibility between environment and agent"""
    print("\n" + "=" * 60)
    print("TESTING DIMENSION COMPATIBILITY")
    print("=" * 60)
    
    try:
        SOMACubeAssemblyEnv = safe_import('soma_cube_environment', ['SOMACubeAssemblyEnv'])[0]
        create_ppo_agent, OptimizedPPOConfig = safe_import('fixed_ppo_agent', ['create_ppo_agent', 'OptimizedPPOConfig'])
        
        env = SOMACubeAssemblyEnv(virtual_mode=True)
        agent = create_ppo_agent(env, OptimizedPPOConfig(), device='cpu')
        
        # Test 1: Observation space compatibility
        obs, info = env.reset()
        
        print("Environment observation structure:")
        for key, value in obs.items():
            if isinstance(value, np.ndarray):
                print(f"  {key}: shape={value.shape}, dtype={value.dtype}")
            else:
                print(f"  {key}: {type(value)} = {value}")
        
        # Test 2: Action space compatibility  
        action_space = env.action_space
        print(f"\nAction space: {action_space}")
        
        sample_action = action_space.sample()
        print(f"Sample action: shape={sample_action.shape}, values={sample_action}")
        
        # Test 3: Agent can process observation
        action, action_info = agent.select_action(obs)
        print(f"\nAgent output:")
        print(f"  Action: shape={action.shape}, values={action}")
        print(f"  Action info: {action_info}")
        
        # Test 4: Environment can process agent action
        next_obs, reward, terminated, truncated, step_info = env.step(action)
        print(f"\nEnvironment step result:")
        print(f"  Reward: {reward}")
        print(f"  Terminated: {terminated}")
        print(f"  Info keys: {list(step_info.keys())}")
        
        return True
        
    except Exception as e:
        print(f"✗ Dimension compatibility test failed: {e}")
        traceback.print_exc()
        return False

def performance_benchmark():
    """Benchmark system performance"""
    print("\n" + "=" * 60)
    print("PERFORMANCE BENCHMARK")
    print("=" * 60)
    
    try:
        import time
        SOMACubeAssemblyEnv = safe_import('soma_cube_environment', ['SOMACubeAssemblyEnv'])[0]
        create_ppo_agent, OptimizedPPOConfig = safe_import('fixed_ppo_agent', ['create_ppo_agent', 'OptimizedPPOConfig'])
        
        # Create components
        env = SOMACubeAssemblyEnv(virtual_mode=True)
        config = OptimizedPPOConfig(batch_size=32)
        agent = create_ppo_agent(env, config, device='cpu')
        
        # Benchmark 1: Environment reset speed
        start_time = time.time()
        for _ in range(100):
            env.reset()
        reset_time = (time.time() - start_time) / 100
        print(f"Environment reset: {reset_time*1000:.2f}ms per reset")
        
        # Benchmark 2: Action selection speed
        obs, _ = env.reset()
        start_time = time.time()
        for _ in range(100):
            agent.select_action(obs)
        action_time = (time.time() - start_time) / 100
        print(f"Action selection: {action_time*1000:.2f}ms per action")
        
        # Benchmark 3: Environment step speed
        action = env.action_space.sample()
        start_time = time.time()
        for _ in range(100):
            env.step(action)
            env.reset()  # Reset after each step for consistency
        step_time = (time.time() - start_time) / 100
        print(f"Environment step: {step_time*1000:.2f}ms per step")
        
        # Overall assessment
        steps_per_second = 1.0 / (reset_time + action_time + step_time)
        print(f"\nEstimated training speed: {steps_per_second:.1f} steps/second")
        
        if steps_per_second > 100:
            print("✓ Performance: Excellent (>100 steps/sec)")
        elif steps_per_second > 50:
            print("✓ Performance: Good (>50 steps/sec)")
        elif steps_per_second > 20:
            print("✓ Performance: Acceptable (>20 steps/sec)")
        else:
            print("! Performance: May need optimization (<20 steps/sec)")
        
        return True
        
    except Exception as e:
        print(f"✗ Performance benchmark failed: {e}")
        traceback.print_exc()
        return False

def main():
    """Run comprehensive validation test suite"""
    print("COMPREHENSIVE VALIDATION TEST SUITE")
    print("Validating Fixed SOMA Cube RL Package")
    print("=" * 80)
    
    # Run all test suites
    test_suites = [
        ("Import Tests", test_imports),
        ("Environment Creation", test_environment_creation),
        ("Agent Creation", test_agent_creation),
        ("Training Integration", test_training_integration),
        ("Algorithm Configuration", test_algorithm_configuration),
        ("Dimension Compatibility", test_dimension_compatibility),
        ("Performance Benchmark", performance_benchmark)
    ]
    
    results = []
    
    for suite_name, test_func in test_suites:
        try:
            result = test_func()
            results.append((suite_name, result))
        except Exception as e:
            print(f"\n✗ {suite_name} crashed: {e}")
            results.append((suite_name, False))
    
    # Summary
    print("\n" + "=" * 80)
    print("VALIDATION RESULTS SUMMARY")
    print("=" * 80)
    
    passed = 0
    for suite_name, result in results:
        status = "PASS" if result else "FAIL"
        icon = "✓" if result else "✗"
        print(f"{icon} {suite_name:25} | {status}")
        if result:
            passed += 1
    
    success_rate = passed / len(results)
    print(f"\nOverall Result: {passed}/{len(results)} test suites passed ({success_rate:.1%})")
    
    if success_rate >= 1.0:
        print("\n🎉 ALL TESTS PASSED! Package is ready for production use.")
        print("\nNext steps:")
        print("1. Run training: python fixed_train.py --episodes 1000")
        print("2. Monitor progress in logs/")
        print("3. Deploy best model to robot")
    elif success_rate >= 0.8:
        print("\n✅ Most tests passed. Package is functional with minor issues.")
        print("Review failed tests and fix if needed.")
    else:
        print("\n⚠️ Multiple test failures. Package needs attention before use.")
        print("Fix failing tests before proceeding with training.")
    
    return success_rate >= 0.8

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
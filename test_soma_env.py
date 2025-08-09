#!/usr/bin/env python3
"""
Simple test script for SOMA cube environment
"""

import numpy as np
from soma_cube_gym_env import SOMACubeEnv

def test_environment():
    """Test the SOMA cube environment"""
    print("=== Testing SOMA Cube Environment ===")
    
    # Create environment
    env_config = {
        'render_mode': 'rgb_array',  # Use rgb_array to avoid GUI issues
        'max_steps': 50,
        'enable_re_grasp': True
    }
    
    try:
        env = SOMACubeEnv(env_config)
        print("✓ Environment created successfully")
        
        # Test reset
        obs, info = env.reset(seed=42)
        print("✓ Environment reset successfully")
        print(f"Observation keys: {obs.keys()}")
        
        # Test a few steps
        for step in range(5):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            
            print(f"Step {step}: Reward={reward:.2f}, Done={terminated}, Info={info}")
            
            if terminated or truncated:
                print("Episode finished early")
                break
        
        print("✓ Environment steps completed successfully")
        
        # Test multiple resets
        for i in range(3):
            obs, info = env.reset()
            print(f"✓ Reset {i+1} successful")
        
        # Close environment
        env.close()
        print("✓ Environment closed successfully")
        
        return True
        
    except Exception as e:
        print(f"❌ Error testing environment: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_environment()
    
    if success:
        print("\n🎉 Environment test passed! Ready for training.")
    else:
        print("\n❌ Environment test failed. Please check the errors above.")
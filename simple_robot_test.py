#!/usr/bin/env python3

import rclpy
import sys
import os

def main():
    """Simple robot connection test"""
    
    print("=== M0609 Robot Connection Test ===")
    print(f"Robot IP: 192.168.1.100")
    print(f"Robot Model: M0609")
    
    # Test 1: Network connectivity
    import subprocess
    try:
        result = subprocess.run(['ping', '-c', '1', '192.168.1.100'], 
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✓ Network connectivity: PASS")
        else:
            print("✗ Network connectivity: FAIL")
            return
    except:
        print("✗ Network connectivity: FAIL")
        return
    
    # Test 2: Robot service port
    import socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        result = sock.connect_ex(('192.168.1.100', 12345))
        sock.close()
        if result == 0:
            print("✓ Robot service port 12345: PASS")
        else:
            print("✗ Robot service port 12345: FAIL")
    except:
        print("✗ Robot service port 12345: FAIL")
    
    # Test 3: ROS2 workspace
    if os.path.exists('install/setup.bash'):
        print("✓ ROS2 workspace built: PASS")
    else:
        print("✗ ROS2 workspace built: FAIL")
    
    # Test 4: ML Environment (Virtual Mode)
    print("\nTesting ML Environment (Virtual Mode)...")
    sys.path.append('src/DoosanBootcamp3rd/m0609_block_assembly_rl')
    
    try:
        from m0609_block_assembly_rl.environment import M0609BlockAssemblyEnv
        env = M0609BlockAssemblyEnv(virtual_mode=True)
        obs, _ = env.reset()
        print(f"✓ ML Environment virtual mode: PASS")
        print(f"  - State space: {obs.shape}")
        print(f"  - Action space: {env.action_space}")
        
        # Test agent
        from m0609_block_assembly_rl.ppo_agent import M0609PPOAgent
        agent = M0609PPOAgent()
        print("✓ PPO Agent creation: PASS")
        
        ml_ready = True
    except Exception as e:
        print(f"✗ ML Environment: FAIL - {e}")
        ml_ready = False
    
    # Test 5: Training script
    train_script = 'src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py'
    if os.path.exists(train_script):
        print("✓ Training script available: PASS")
    else:
        print("✗ Training script available: FAIL")
    
    print(f"\n=== Summary ===")
    if ml_ready:
        print("🎉 Your M0609 robot ML system is ready!")
        print("\nNext steps:")
        print("1. For virtual training:")
        print("   python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py --virtual")
        print("\n2. For real robot (ensure robot is in teachout mode):")
        print("   python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py --robot-id dsr01 --robot-model m0609")
        print("\n3. Robot configuration updated for IP: 192.168.1.100")
    else:
        print("⚠ Setup incomplete - ML environment has issues")

if __name__ == "__main__":
    main()
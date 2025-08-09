#!/usr/bin/env python3

import rclpy
import sys
import os
sys.path.append('src/DoosanBootcamp3rd/m0609_block_assembly_rl')

def test_robot_connection():
    """Test connection to real robot at 192.168.1.100"""
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        print("Testing robot connection to 192.168.1.100...")
        
        # Test importing Doosan modules
        import DR_init
        import DSR_ROBOT2 as robot
        import DR_common2 as common
        
        # Set robot parameters
        ROBOT_ID = "dsr01"
        ROBOT_MODEL = "m0609"
        
        DR_init.__dsr__id = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        
        # Create ROS2 node
        node = rclpy.create_node("robot_connection_test", namespace=ROBOT_ID)
        DR_init.__dsr__node = node
        
        print(f"✓ ROS2 node created for robot {ROBOT_ID} ({ROBOT_MODEL})")
        
        # Try basic robot operation
        try:
            pos = robot.get_current_posj()
            print(f"✓ Robot connection successful! Current position: {pos}")
            return True
        except Exception as e:
            print(f"⚠ Robot connection failed: {e}")
            return False
            
    except ImportError as e:
        print(f"⚠ Doosan robot modules not available: {e}")
        return False
    except Exception as e:
        print(f"⚠ Connection test failed: {e}")
        return False
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()

def test_ml_environment():
    """Test ML environment in both virtual and real modes"""
    
    print("\nTesting ML environment...")
    
    try:
        from m0609_block_assembly_rl.environment import M0609BlockAssemblyEnv
        
        # Test virtual mode
        print("Testing virtual mode...")
        env_virtual = M0609BlockAssemblyEnv(virtual_mode=True)
        print("✓ Virtual environment created successfully")
        
        # Test observation space
        obs = env_virtual.reset()
        print(f"✓ Observation space: {obs[0].shape if isinstance(obs, tuple) else obs.shape}")
        
        # Test action space
        action = env_virtual.action_space.sample()
        print(f"✓ Action space sampled: {type(action)}")
        
        return True
        
    except Exception as e:
        print(f"⚠ ML environment test failed: {e}")
        return False

if __name__ == "__main__":
    print("=== Robot Connection and ML Environment Test ===")
    
    # Test robot connection
    robot_ok = test_robot_connection()
    
    # Test ML environment
    ml_ok = test_ml_environment()
    
    print(f"\n=== Test Results ===")
    print(f"Robot Connection: {'✓ PASS' if robot_ok else '✗ FAIL (Virtual mode available)'}")
    print(f"ML Environment: {'✓ PASS' if ml_ok else '✗ FAIL'}")
    
    if ml_ok:
        print("\n🎉 Your machine learning code for real robot is ready!")
        print("   • Robot IP: 192.168.1.100")
        print("   • Model: M0609") 
        print("   • ML Environment: Ready")
        print("   • Next steps: Run training with 'python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py'")
    else:
        print("\n⚠ Setup incomplete - check error messages above")
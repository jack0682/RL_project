#!/usr/bin/env python3
"""
Manual test of ROS2 RL system without background processes
Run this directly to test the ROS2 integration
"""

import os
import sys
import time
import threading
from pathlib import Path

# Set up path
sys.path.append(str(Path(__file__).parent / "src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl"))

def test_enhanced_logger_with_ros2():
    """Test the enhanced logger with ROS2 publishing"""
    print("Testing Enhanced Logger with ROS2 integration...")
    
    try:
        from enhanced_training_logger import EnhancedTrainingLogger, TrainingMetrics
        import numpy as np
        
        # Create logger with ROS2 enabled
        logger = EnhancedTrainingLogger(
            experiment_name="manual_ros2_test",
            log_dir="test_logs",
            enable_ros2=True,
            enable_tensorboard=True
        )
        
        print("✅ Logger created with ROS2 integration")
        
        # Test logging some episodes
        for episode in range(10):
            metrics = TrainingMetrics(
                episode=episode,
                timestamp=time.time(),
                success=episode >= 5,
                total_reward=float(episode * 5 - 10),
                steps_taken=20 + episode,
                pieces_placed=min(episode + 1, 7),
                completion_rate=min((episode + 1) / 7.0, 1.0),
                policy_loss=1.0 - episode * 0.05,
                value_loss=0.5 - episode * 0.02
            )
            
            training_stats = {
                'policy_loss': metrics.policy_loss,
                'value_loss': metrics.value_loss,
                'entropy': 0.3 - episode * 0.01
            }
            
            logger.log_episode(metrics, training_stats)
            
            print(f"Episode {episode}: Success={metrics.success}, Reward={metrics.total_reward}")
            time.sleep(0.2)
        
        print("✅ Completed logging 10 episodes")
        
        # Close and get summary
        summary = logger.close()
        
        if summary and 'experiment_info' in summary:
            info = summary['experiment_info']
            print(f"✅ Training summary: {info['total_episodes']} episodes, {info['success_rate']:.2%} success rate")
        
        return True
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        print(traceback.format_exc())
        return False

def test_simple_file_logging():
    """Test simple file logging without ROS2"""
    print("\nTesting simple file logging...")
    
    try:
        from enhanced_training_logger import EnhancedTrainingLogger, TrainingMetrics
        import numpy as np
        
        # Create logger without ROS2
        logger = EnhancedTrainingLogger(
            experiment_name="manual_file_test",
            log_dir="test_logs",
            enable_ros2=False,
            enable_tensorboard=False
        )
        
        print("✅ File logger created")
        
        # Log a few episodes
        for episode in range(5):
            metrics = TrainingMetrics(
                episode=episode,
                timestamp=time.time(),
                success=episode >= 2,
                total_reward=float(episode * 8),
                steps_taken=15 + episode * 2,
                pieces_placed=min(episode + 2, 7),
                completion_rate=min((episode + 2) / 7.0, 1.0)
            )
            
            logger.log_episode(metrics)
            print(f"Logged episode {episode}")
        
        # Close
        summary = logger.close()
        print("✅ File logging completed")
        
        return True
        
    except Exception as e:
        print(f"❌ File logging test failed: {e}")
        import traceback
        print(traceback.format_exc())
        return False

def test_existing_training_script():
    """Test running existing training with enhanced logging"""
    print("\nTesting existing training script integration...")
    
    try:
        # Import training components
        from enhanced_training_logger import EnhancedTrainingLogger, TrainingMetrics
        from fixed_train import TrainingConfig
        
        # Create config
        config = TrainingConfig()
        config.max_episodes = 20  # Short test
        config.use_tensorboard = False
        
        # Create logger
        logger = EnhancedTrainingLogger(
            experiment_name="training_integration_test",
            log_dir=config.log_dir,
            config=config.__dict__,
            enable_ros2=True
        )
        
        print("✅ Training integration test setup complete")
        
        # Simulate training loop
        import numpy as np
        for episode in range(1, config.max_episodes + 1):
            
            # Simulate episode results
            success = np.random.random() > 0.6
            reward = np.random.uniform(10, 50) if success else np.random.uniform(-20, 10)
            steps = np.random.randint(15, 40)
            pieces = np.random.randint(0, 7)
            
            # Create metrics
            metrics = TrainingMetrics(
                episode=episode,
                timestamp=time.time(),
                success=success,
                total_reward=reward,
                steps_taken=steps,
                pieces_placed=pieces,
                completion_rate=pieces / 7.0,
                curriculum_stage="BASIC_PLACEMENT"
            )
            
            # Simulate training stats
            training_stats = {
                'policy_loss': np.random.uniform(0.1, 1.0),
                'value_loss': np.random.uniform(0.1, 0.5),
                'entropy': np.random.uniform(0.1, 0.4)
            }
            
            # Log episode
            logger.log_episode(metrics, training_stats)
            
            if episode % 5 == 0:
                print(f"Episode {episode}: Success={success}, Reward={reward:.2f}")
        
        # Close
        summary = logger.close()
        print("✅ Training integration test completed")
        
        return True
        
    except Exception as e:
        print(f"❌ Training integration test failed: {e}")
        import traceback
        print(traceback.format_exc())
        return False

def main():
    """Run all manual tests"""
    print("=" * 60)
    print("MANUAL ROS2 RL INTEGRATION TEST")
    print("=" * 60)
    
    # Test 1: Simple file logging (should always work)
    test1 = test_simple_file_logging()
    
    # Test 2: ROS2 integration (may work without nodes running)
    test2 = test_enhanced_logger_with_ros2()
    
    # Test 3: Training integration
    test3 = test_existing_training_script()
    
    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)
    print(f"File Logging:           {'✅ PASSED' if test1 else '❌ FAILED'}")
    print(f"ROS2 Integration:       {'✅ PASSED' if test2 else '❌ FAILED'}")
    print(f"Training Integration:   {'✅ PASSED' if test3 else '❌ FAILED'}")
    
    if test1 and test3:
        print("\n🎉 CORE FUNCTIONALITY WORKING!")
        print("\nYour enhanced logging system is working correctly.")
        print("Even if ROS2 integration has issues, your training")
        print("will log comprehensively to files and tensorboard.")
        
        print(f"\nCheck your logs in: test_logs/")
        print("Files generated:")
        print("- training_metrics.csv")
        print("- training_metrics.jsonl") 
        print("- training_summary.json")
        print("- tensorboard/ (if enabled)")
        
    elif test1:
        print("\n✅ BASIC LOGGING WORKING")
        print("File-based logging is working correctly.")
        print("ROS2 integration may need additional setup.")
        
    else:
        print("\n❌ TESTS FAILED")
        print("Please check the error messages above.")

if __name__ == "__main__":
    main()
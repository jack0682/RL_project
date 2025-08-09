#!/usr/bin/env python3
"""
Run the full_training hierarchical RL model with CUDA
"""
import os
import sys
import torch

# Add the ML module to path
sys.path.append('/home/rokey/ros2_ws/src/DoosanBootcamp3rd/m0609_block_assembly_rl')

def main():
    print("=== DOOSAN M0609 FULL TRAINING MODEL ===")
    
    # Check CUDA availability
    if torch.cuda.is_available():
        device = 'cuda'
        print(f"✓ CUDA available: {torch.cuda.get_device_name(0)}")
        print(f"✓ CUDA memory: {torch.cuda.get_device_properties(0).total_memory / 1e9:.1f} GB")
    else:
        device = 'cpu'
        print("⚠ CUDA not available, using CPU")
    
    # Load the hierarchical trainer
    try:
        from m0609_block_assembly_rl.hierarchical_soma_trainer import HierarchicalSOMATrainer
        from m0609_block_assembly_rl.soma_environment import SOMAEnvironment
        
        print("\n=== LOADING FULL TRAINING MODEL ===")
        
        # Initialize environment
        env_config = {
            'observation_mode': 'discrete',
            'curriculum_level': 3,
            'max_steps': 30,
            'sparse_reward': False
        }
        env = SOMAEnvironment(env_config)
        
        # Initialize trainer with CUDA
        trainer = HierarchicalSOMATrainer(
            env=env,
            device=device,
            experiment_name="full_training_cuda"
        )
        
        # Load the trained model
        checkpoint_dir = "/home/rokey/ros2_ws/checkpoints/full_training"
        if os.path.exists(f"{checkpoint_dir}/final_upper_planner.pth"):
            trainer.load_checkpoint(checkpoint_dir)
            print("✓ Loaded full_training model successfully")
            
            print("\n=== MODEL CONFIGURATION ===")
            print(f"Device: {device}")
            print(f"Upper Planner: Loaded")
            print(f"Lower Controller: Loaded")
            print("Environment: SOMA Cube Assembly")
            
            # Run evaluation
            print("\n=== RUNNING EVALUATION ===")
            success_rate = trainer.evaluate(num_episodes=10)
            print(f"Success Rate: {success_rate:.1%}")
            
        else:
            print("❌ Full training model not found!")
            
    except ImportError as e:
        print(f"❌ Import error: {e}")
        print("Make sure the m0609_block_assembly_rl package is properly installed")
    
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
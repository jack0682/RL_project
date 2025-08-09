#!/usr/bin/env python3
"""
Simple training launcher for SOMA Cube RL model
"""
import os
import sys
import torch
import subprocess
import argparse

def main():
    parser = argparse.ArgumentParser(description='Train SOMA Cube RL Model')
    parser.add_argument('--episodes', type=int, default=10000, help='Number of training episodes')
    parser.add_argument('--name', type=str, default='cuda_training', help='Experiment name')
    parser.add_argument('--resume', action='store_true', help='Resume from existing checkpoint')
    parser.add_argument('--quick', action='store_true', help='Quick training (5000 episodes)')
    
    args = parser.parse_args()
    
    print("=== DOOSAN M0609 RL TRAINING ===")
    
    # Check CUDA
    if torch.cuda.is_available():
        print(f"✓ CUDA available: {torch.cuda.get_device_name(0)}")
        device = "cuda"
    else:
        print("⚠ CUDA not available, using CPU")
        device = "cpu"
    
    # Set episode count
    episodes = 5000 if args.quick else args.episodes
    print(f"Training episodes: {episodes}")
    
    # Change to ML directory
    ml_dir = "/home/rokey/ros2_ws/src/DoosanBootcamp3rd/m0609_block_assembly_rl"
    os.chdir(ml_dir)
    
    # Build training command
    cmd = [
        "python3", "fixed_hierarchical_train.py",
        "--device", device,
        "--episodes", str(episodes),
        "--experiment-name", args.name
    ]
    
    # Add resume option if specified
    if args.resume:
        checkpoint_path = "/home/rokey/ros2_ws/checkpoints/full_training"
        if os.path.exists(checkpoint_path):
            cmd.extend(["--resume-from", checkpoint_path])
            print(f"✓ Resuming from: {checkpoint_path}")
        else:
            print("⚠ No checkpoint found, starting fresh training")
    
    print(f"\nRunning command: {' '.join(cmd)}")
    print("=" * 50)
    
    try:
        # Run training
        result = subprocess.run(cmd, check=True)
        print("\n" + "=" * 50)
        print("✅ Training completed successfully!")
        
        # Show where results are saved
        results_dir = f"/home/rokey/ros2_ws/checkpoints/{args.name}"
        print(f"📁 Results saved to: {results_dir}")
        
    except subprocess.CalledProcessError as e:
        print(f"\n❌ Training failed with error code: {e.returncode}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n⚠ Training interrupted by user")
        sys.exit(1)

if __name__ == "__main__":
    main()
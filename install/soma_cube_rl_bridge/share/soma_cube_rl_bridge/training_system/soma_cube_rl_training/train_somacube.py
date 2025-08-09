#!/usr/bin/env python3
"""
Main training script for SomaCube assembly RL
Supports different training modes and configurations
"""

import argparse
import os
import sys
import time
import signal
from datetime import datetime
from typing import Optional

import rclpy
import torch
import numpy as np

import sys
import os

# Add the training system directory to Python path
training_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, training_dir)

from sac_trainer import SACTrainer
from rl_client import SomaCubeRLClient


class TrainingManager:
    """
    Manages the complete training lifecycle
    """
    
    def __init__(self, config: dict):
        self.config = config
        self.trainer: Optional[SACTrainer] = None
        self.start_time = None
        
        # Setup signal handlers for graceful shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
    
    def _signal_handler(self, signum, frame):
        """Handle shutdown signals gracefully"""
        print(f"\n⚠️ Received signal {signum}, shutting down gracefully...")
        if self.trainer:
            self.trainer.close()
        if rclpy.ok():
            rclpy.shutdown()
        sys.exit(0)
    
    def check_system_status(self) -> bool:
        """Check if RL bridge system is ready"""
        print("🔍 Checking system status...")
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.init()
        
        try:
            # Test connection to RL bridge
            client = SomaCubeRLClient()
            
            # Check if services are available
            if not client.is_safe():
                print("⚠️ Robot is not in safe state - training may be limited")
            
            stats = client.get_stats()
            print(f"✅ Connected to RL bridge")
            print(f"   Safety state: {stats.get('safety_state', 'Unknown')}")
            print(f"   Safety reason: {stats.get('safety_reason', 'N/A')}")
            
            client.close()
            return True
            
        except Exception as e:
            print(f"❌ Failed to connect to RL bridge: {e}")
            print("   Make sure the RL bridge is running:")
            print("   ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true")
            return False
    
    def run_training(self):
        """Run the main training loop"""
        if not self.check_system_status():
            return False
        
        print("🚀 Starting SomaCube RL Training")
        print("=" * 60)
        print(f"Algorithm: {self.config['algorithm']}")
        print(f"Total timesteps: {self.config['total_timesteps']:,}")
        print(f"Max episode steps: {self.config['max_episode_steps']}")
        print(f"Device: {self.config['device']}")
        print(f"Log directory: {self.config['log_dir']}")
        print("=" * 60)
        
        self.start_time = time.time()
        
        try:
            # Create trainer
            self.trainer = SACTrainer(
                log_dir=self.config['log_dir'],
                max_episode_steps=self.config['max_episode_steps'],
                device=self.config['device']
            )
            
            # Start training
            self.trainer.train(
                total_timesteps=self.config['total_timesteps'],
                save_freq=self.config['save_freq']
            )
            
            # Run evaluation
            if self.config['evaluate']:
                print("\n📊 Running final evaluation...")
                results = self.trainer.evaluate(
                    n_episodes=self.config['eval_episodes'],
                    render=self.config['render_eval']
                )
                
                print(f"🎯 Final Results:")
                print(f"   Mean reward: {results['mean_reward']:.2f} ± {results['std_reward']:.2f}")
                print(f"   Mean length: {results['mean_length']:.1f}")
            
            training_time = time.time() - self.start_time
            print(f"\n✅ Training completed successfully in {training_time:.1f} seconds")
            return True
            
        except Exception as e:
            print(f"❌ Training failed: {e}")
            import traceback
            traceback.print_exc()
            return False
        
        finally:
            if self.trainer:
                self.trainer.close()
    
    def run_evaluation_only(self, model_path: str):
        """Run evaluation only with a pre-trained model"""
        print(f"📊 Running evaluation with model: {model_path}")
        
        if not os.path.exists(model_path):
            print(f"❌ Model file not found: {model_path}")
            return False
        
        if not self.check_system_status():
            return False
        
        try:
            # Create trainer and load model
            self.trainer = SACTrainer(
                log_dir=self.config['log_dir'],
                max_episode_steps=self.config['max_episode_steps'],
                device=self.config['device']
            )
            
            # Load the model
            from stable_baselines3 import SAC
            self.trainer.model = SAC.load(model_path, env=self.trainer.env)
            
            # Run evaluation
            results = self.trainer.evaluate(
                n_episodes=self.config['eval_episodes'],
                render=self.config['render_eval']
            )
            
            print(f"🎯 Evaluation Results:")
            print(f"   Mean reward: {results['mean_reward']:.2f} ± {results['std_reward']:.2f}")
            print(f"   Mean length: {results['mean_length']:.1f}")
            
            return True
            
        except Exception as e:
            print(f"❌ Evaluation failed: {e}")
            return False
        
        finally:
            if self.trainer:
                self.trainer.close()


def parse_args():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description="SomaCube RL Training System")
    
    # Training mode
    parser.add_argument(
        "--mode", 
        choices=["train", "eval", "check"], 
        default="train",
        help="Training mode: train, eval (evaluation only), or check (system check)"
    )
    
    # Model parameters
    parser.add_argument(
        "--algorithm", 
        choices=["sac"], 
        default="sac",
        help="RL algorithm to use"
    )
    
    parser.add_argument(
        "--total-timesteps", 
        type=int, 
        default=500000,
        help="Total training timesteps"
    )
    
    parser.add_argument(
        "--max-episode-steps", 
        type=int, 
        default=500,
        help="Maximum steps per episode"
    )
    
    parser.add_argument(
        "--save-freq", 
        type=int, 
        default=50000,
        help="Save model every N steps"
    )
    
    # Evaluation parameters
    parser.add_argument(
        "--eval-episodes", 
        type=int, 
        default=10,
        help="Number of episodes for evaluation"
    )
    
    parser.add_argument(
        "--model-path", 
        type=str,
        help="Path to pre-trained model (for evaluation mode)"
    )
    
    parser.add_argument(
        "--no-eval", 
        action="store_true",
        help="Skip evaluation after training"
    )
    
    parser.add_argument(
        "--no-render", 
        action="store_true",
        help="Don't render during evaluation"
    )
    
    # System parameters
    parser.add_argument(
        "--device", 
        choices=["auto", "cpu", "cuda"], 
        default="auto",
        help="Device for training"
    )
    
    parser.add_argument(
        "--log-dir", 
        type=str, 
        default="./somacube_training",
        help="Directory for training logs"
    )
    
    parser.add_argument(
        "--name", 
        type=str,
        help="Run name (appended to log directory)"
    )
    
    return parser.parse_args()


def main():
    """Main entry point"""
    args = parse_args()
    
    # Setup device
    if args.device == "auto":
        device = "cuda" if torch.cuda.is_available() else "cpu"
    else:
        device = args.device
    
    # Setup log directory
    log_dir = args.log_dir
    if args.name:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.join(log_dir, f"{args.name}_{timestamp}")
    
    # Create configuration
    config = {
        "algorithm": args.algorithm,
        "total_timesteps": args.total_timesteps,
        "max_episode_steps": args.max_episode_steps,
        "save_freq": args.save_freq,
        "eval_episodes": args.eval_episodes,
        "evaluate": not args.no_eval,
        "render_eval": not args.no_render,
        "device": device,
        "log_dir": log_dir
    }
    
    # Initialize training manager
    manager = TrainingManager(config)
    
    try:
        if args.mode == "check":
            # System check mode
            if manager.check_system_status():
                print("✅ System check passed - ready for training!")
                return 0
            else:
                print("❌ System check failed")
                return 1
                
        elif args.mode == "eval":
            # Evaluation only mode
            if not args.model_path:
                print("❌ Model path required for evaluation mode")
                return 1
            
            success = manager.run_evaluation_only(args.model_path)
            return 0 if success else 1
            
        else:
            # Training mode
            success = manager.run_training()
            return 0 if success else 1
    
    except KeyboardInterrupt:
        print("\n⚠️ Training interrupted by user")
        return 1
    
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return 1
    
    finally:
        # Ensure ROS2 cleanup
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
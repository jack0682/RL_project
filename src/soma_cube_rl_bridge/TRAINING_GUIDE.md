# 🎯 SomaCube RL Training Guide

Complete guide for training reinforcement learning agents on the SomaCube assembly task using the Doosan M0609 robot.

## 📋 Prerequisites

### 1. System Requirements
- **ROS2 Humble** installed and configured
- **Python 3.8+** with pip
- **CUDA-capable GPU** (recommended) or CPU training
- **DoosanBootcamp3rd** packages built and working
- **soma_cube_rl_bridge** built and tested

### 2. Install Training Dependencies
```bash
# Navigate to training system directory
cd /home/jack/ros2_ws/src/DoosanBootcamp3rd/soma_cube_rl_bridge/training_system

# Install Python dependencies
pip install -r requirements.txt

# For GPU training (optional but recommended)
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### 3. Verify Installation
```bash
python -c "import torch; print('CUDA available:', torch.cuda.is_available())"
python -c "import stable_baselines3; print('SB3 version:', stable_baselines3.__version__)"
```

## 🚀 Quick Start Training

### Step 1: Start the RL Bridge
```bash
# Terminal 1: Start the RL bridge system
cd /home/jack/ros2_ws
source install/setup.bash
ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true
```

### Step 2: Check System Status
```bash
# Terminal 2: Verify everything is working
cd /home/jack/ros2_ws/src/DoosanBootcamp3rd/soma_cube_rl_bridge/training_system
python train_somacube.py --mode check
```

Expected output:
```
🔍 Checking system status...
✅ Connected to RL bridge
   Safety state: False  # Expected without real robot
   Safety reason: Robot not in AUTONOMOUS mode
✅ System check passed - ready for training!
```

### Step 3: Start Training
```bash
# Start SAC training (500K timesteps, ~2-4 hours depending on hardware)
python train_somacube.py --mode train --total-timesteps 500000
```

## 🎮 Training Modes

### 1. Full Training Mode (Default)
```bash
# Standard training with evaluation
python train_somacube.py --mode train

# Custom configuration
python train_somacube.py \
    --mode train \
    --total-timesteps 1000000 \
    --max-episode-steps 750 \
    --save-freq 25000 \
    --name "long_episodes" \
    --device cuda
```

### 2. Evaluation Only Mode
```bash
# Evaluate a pre-trained model
python train_somacube.py \
    --mode eval \
    --model-path "./somacube_training/sac_somacube_20250809_120000/final_model.zip" \
    --eval-episodes 20
```

### 3. System Check Mode
```bash
# Verify system is ready for training
python train_somacube.py --mode check
```

## 🔧 Training Configuration

### Command Line Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `--mode` | `train` | Mode: `train`, `eval`, or `check` |
| `--algorithm` | `sac` | RL algorithm (currently SAC only) |
| `--total-timesteps` | `500000` | Total training steps |
| `--max-episode-steps` | `500` | Max steps per episode |
| `--save-freq` | `50000` | Save model every N steps |
| `--eval-episodes` | `10` | Episodes for evaluation |
| `--device` | `auto` | Device: `auto`, `cpu`, or `cuda` |
| `--log-dir` | `./somacube_training` | Training logs directory |
| `--name` | - | Run name for logging |

### Example Configurations

#### Quick Test Training
```bash
python train_somacube.py \
    --total-timesteps 50000 \
    --max-episode-steps 200 \
    --save-freq 10000 \
    --name "quick_test"
```

#### Long Training Run
```bash
python train_somacube.py \
    --total-timesteps 2000000 \
    --max-episode-steps 1000 \
    --save-freq 100000 \
    --name "long_training" \
    --device cuda
```

#### CPU Training (Slower)
```bash
python train_somacube.py \
    --total-timesteps 200000 \
    --max-episode-steps 300 \
    --device cpu \
    --name "cpu_training"
```

## 📊 Monitoring Training

### 1. Training Logs
```bash
# View training progress
tail -f somacube_training/sac_somacube_*/progress.txt

# Monitor with tensorboard
tensorboard --logdir=somacube_training/
```

### 2. Real-time Monitoring
Training output includes:
- **Step progress**: Current timestep and progress
- **Episode rewards**: Mean reward per episode
- **Episode lengths**: Average episode duration
- **Model saving**: Automatic model checkpoints
- **Safety warnings**: Any safety violations during training

### 3. Training Files
```
somacube_training/
├── sac_somacube_20250809_120000/
│   ├── config.json              # Training configuration
│   ├── final_model.zip          # Final trained model
│   ├── sac_somacube_50000.zip   # Checkpoint at 50k steps
│   ├── sac_somacube_100000.zip  # Checkpoint at 100k steps
│   ├── summary.json             # Training summary
│   ├── monitor.csv              # Episode rewards/lengths
│   └── tensorboard_logs/        # TensorBoard logs
```

## 🎯 Understanding the Learning Process

### Observation Space (28 elements)
- **Elements 0-5**: Joint positions (degrees)
- **Elements 6-11**: Joint velocities (deg/s)
- **Elements 12-14**: TCP position (x, y, z in mm)
- **Elements 15-18**: TCP orientation (quaternion)
- **Elements 19-24**: Tool force/torque (N, Nm)
- **Element 25**: Distance to target (mm)
- **Element 26**: Current step in episode
- **Element 27**: Remaining steps in episode

### Action Space (6 elements)
- **6-DoF joint target positions** in degrees
- **Range**: Conservative limits (±10% of joint limits)
- **Safety**: Actions automatically clamped to safe ranges

### Reward Structure
- **Grasp success**: +100 points for reaching target
- **Assembly alignment**: +50 points for correct orientation
- **Distance penalty**: -1 point per mm from target
- **Time penalty**: -0.1 points per step (encourages efficiency)
- **Collision penalty**: -100 points for safety violations
- **Velocity penalty**: -0.01 points for high velocities (smoothness)

## 🛠️ Troubleshooting

### Common Issues

#### 1. "Service not available" errors
```bash
# Check if RL bridge is running
ros2 service list | grep rl
ros2 node list | grep soma_cube

# Restart RL bridge if needed
ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true
```

#### 2. CUDA out of memory
```bash
# Reduce batch size or use CPU
python train_somacube.py --device cpu

# Or reduce timesteps for testing
python train_somacube.py --total-timesteps 100000
```

#### 3. Training not converging
- **Increase training time**: Use 1M+ timesteps
- **Adjust episode length**: Try longer episodes (750-1000 steps)
- **Check reward function**: Monitor if rewards are reasonable
- **Safety violations**: Ensure robot safety state is correct

#### 4. Low performance/rewards
- **Check target pose**: Ensure target is reachable
- **Reward tuning**: Adjust reward weights in config files
- **Episode length**: Allow more time for task completion
- **Action scaling**: Verify action space is appropriate

### Advanced Debugging

#### Monitor RL Interface Directly
```bash
# Test reset service
ros2 service call /rl/reset soma_cube_rl_bridge/srv/RLReset "{seed: 42}"

# Test step service
ros2 service call /rl/step soma_cube_rl_bridge/srv/RLStep "{action: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Monitor safety state
ros2 topic echo safety/state
```

#### Python Debug Mode
```python
# Add to training script for debugging
import pdb; pdb.set_trace()

# Or use verbose logging
import logging
logging.basicConfig(level=logging.DEBUG)
```

## 🎓 Training Tips and Best Practices

### 1. Training Strategy
- **Start with short episodes** (200-300 steps) to learn basics
- **Gradually increase** episode length as agent improves  
- **Use curriculum learning**: Start with easier targets, increase difficulty
- **Monitor safety**: Training should rarely trigger safety violations

### 2. Hyperparameter Tuning
- **Learning rate**: Start with 3e-4, adjust based on convergence
- **Buffer size**: Larger for more diverse experience (100k-1M)
- **Batch size**: 256-512 for stable learning
- **Network size**: [400, 300] is good starting point

### 3. Hardware Optimization
- **GPU recommended**: 10-50x faster than CPU
- **RAM requirement**: ~8GB minimum for training
- **Storage**: ~1-10GB for logs and models
- **Multi-core CPU**: Helps with environment simulation

### 4. Real Robot Transition
```bash
# Switch from simulation to real robot
ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=false

# Use more conservative settings
python train_somacube.py \
    --max-episode-steps 200 \
    --total-timesteps 100000 \
    --name "real_robot"
```

## 📈 Expected Training Results

### Training Phases
1. **Exploration (0-50k steps)**: Random actions, negative rewards
2. **Learning (50k-200k steps)**: Gradual improvement, reward increase
3. **Refinement (200k+ steps)**: Consistent performance, high success rate

### Success Metrics
- **Mean reward > 50**: Good performance
- **Mean reward > 100**: Excellent performance  
- **Episode length**: Should decrease as efficiency improves
- **Success rate**: >80% for well-trained agent

### Typical Training Time
- **CPU**: 8-24 hours for 500k steps
- **GPU**: 2-6 hours for 500k steps
- **Convergence**: Usually within 200k-500k steps

## 🚀 Next Steps

### 1. Advanced Training
- **Multi-task learning**: Train on different cube configurations
- **Curriculum learning**: Progressive difficulty increase
- **Domain randomization**: Vary physics parameters
- **Imitation learning**: Bootstrap with human demonstrations

### 2. Deployment
- **Model export**: Convert to ONNX or TensorRT for inference
- **Real-time control**: Integrate with robot control loop
- **Safety verification**: Extensive testing before deployment
- **Performance monitoring**: Track success rates in production

### 3. Research Extensions
- **Different algorithms**: Try PPO, TD3, or other methods
- **Multi-agent**: Coordinate multiple robots
- **Vision integration**: Add camera observations
- **Force feedback**: Use torque/force sensors more effectively

---

## 📞 Support

For training issues:
1. Check this guide and troubleshooting section
2. Monitor training logs and TensorBoard
3. Verify RL bridge is running correctly
4. Check hardware resources (GPU memory, disk space)

**Happy Training!** 🤖🎯🚀
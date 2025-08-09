# SomaCube RL Training System - Execution Guide

## Overview
This guide shows how to execute the SomaCube RL training system that is integrated within the soma_cube_rl_bridge package.

## Package Structure
The training system is located within the soma_cube_rl_bridge package at:
```
/home/jack/ros2_ws/src/DoosanBootcamp3rd/soma_cube_rl_bridge/training_system/
```

## Available Methods
After building, you can run the training system in two ways:

### Method 1: Simple Launcher Script (Recommended)
```bash
cd /home/jack/ros2_ws
source install/setup.bash

# Show help
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh

# Test RL client connection
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh rl_client

# Check system status
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh check

# Run training
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh train --steps 50000 --name test_run
```

### Method 2: Direct Python Scripts
```bash
cd /home/jack/ros2_ws
source install/setup.bash

# RL client test
python3 install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/soma_cube_rl_training/rl_client.py

# Training with full options
python3 install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/soma_cube_rl_training/train_somacube.py --mode check

# Demo system
python3 install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/soma_cube_rl_training/demo_training.py
```

## Step-by-Step Execution

### 1. Build the Complete System
```bash
cd /home/jack/ros2_ws
colcon build --packages-select soma_cube_rl_bridge
source install/setup.bash
```

### 2. Install Training Dependencies (if needed for training)
```bash
pip3 install -r /home/jack/ros2_ws/src/DoosanBootcamp3rd/soma_cube_rl_bridge/training_system/requirements.txt
```

### 3. Start the RL Bridge System
In Terminal 1:
```bash
cd /home/jack/ros2_ws
source install/setup.bash
ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true
```

### 4. Test the System
In Terminal 2:
```bash
cd /home/jack/ros2_ws
source install/setup.bash

# Test basic RL client connection (works without gym dependencies)
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh rl_client

# Check system status
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh check
```

### 5. Run Training (requires dependencies)
```bash
cd /home/jack/ros2_ws
source install/setup.bash

# Quick test training (50k steps)
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh train --steps 50000

# Full training (500k steps) 
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh train --steps 500000 --name full_training

# Custom training
python3 install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/soma_cube_rl_training/train_somacube.py \
    --mode train \
    --total-timesteps 100000 \
    --max-episode-steps 300 \
    --save-freq 25000 \
    --name "my_experiment"
```

### 6. Evaluate Trained Model
```bash
cd /home/jack/ros2_ws
source install/setup.bash

./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh eval \
    --model-path /path/to/your/model.zip \
    --eval-episodes 20
```

## Package Integration Benefits
The training system is now properly integrated within the soma_cube_rl_bridge package:

✅ **Integrated Structure**: Training system is part of the main RL bridge package  
✅ **Easy Access**: Simple launcher script for common operations
✅ **Standard Installation**: Installs with the main package build
✅ **Proper Paths**: All imports and paths work correctly within package
✅ **Dependency Management**: Requirements clearly defined
✅ **ROS2 Compatible**: Works with ROS2 environment and services

## Command Reference

### Launcher Script Commands
- `rl_client` - Test RL client connection (no dependencies needed)
- `demo` - Run interactive demo system
- `check` - Check system status and verify setup
- `train` - Start training session with options
- `eval` - Evaluate existing model

### Training Parameters
- `--steps N` - Training steps (launcher script shortcut)
- `--name NAME` - Experiment name
- `--device auto|cpu|cuda` - Training device
- `--total-timesteps N` - Training duration (direct script)
- `--max-episode-steps N` - Episode length (direct script)
- `--save-freq N` - Model save frequency (direct script)

### Evaluation Parameters  
- `--eval-episodes N` - Number of evaluation episodes
- `--model-path PATH` - Path to trained model file
- `--no-render` - Disable rendering during evaluation

## Troubleshooting

### Service Connection Issues
If RL client cannot connect:
1. Ensure RL bridge is running: `ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true`
2. Check services: `ros2 service list | grep /rl/`
3. Verify robot safety state

### Missing Dependencies
If training commands fail with import errors:
```bash
pip3 install stable-baselines3[extra] torch gymnasium numpy matplotlib tensorboard
```

### Build Issues
If colcon build fails:
```bash
cd /home/jack/ros2_ws
colcon build --packages-select soma_cube_rl_bridge --cmake-clean-cache
```

### Path Issues
If scripts don't run, ensure you're in the workspace root:
```bash
cd /home/jack/ros2_ws
source install/setup.bash
```

## File Locations After Build

The training system files are installed to:
```
install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/
├── run_training.sh                 # Main launcher script
├── requirements.txt               # Python dependencies
├── EXECUTION_GUIDE.md            # This guide
└── soma_cube_rl_training/        # Python modules
    ├── rl_client.py              # ROS2 RL client
    ├── train_somacube.py         # Main training script
    ├── sac_trainer.py           # SAC algorithm implementation
    └── demo_training.py         # Demo and testing system
```

## Success Verification
The system is working correctly if:
1. ✅ Package builds: `colcon build --packages-select soma_cube_rl_bridge`  
2. ✅ Launcher shows help: `./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh`
3. ✅ RL client runs (may timeout waiting for bridge): launcher `rl_client`
4. ✅ System check works when bridge is running: launcher `check`

## Quick Start Summary
```bash
# 1. Build
cd /home/jack/ros2_ws
colcon build --packages-select soma_cube_rl_bridge
source install/setup.bash

# 2. Start RL bridge (Terminal 1)
ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true

# 3. Test connection (Terminal 2) 
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh rl_client

# 4. Install dependencies and train (if desired)
pip3 install -r src/DoosanBootcamp3rd/soma_cube_rl_bridge/training_system/requirements.txt
./install/soma_cube_rl_bridge/share/soma_cube_rl_bridge/training_system/run_training.sh train --steps 50000
```

The training system now properly "runs with package" as requested - it's integrated within the soma_cube_rl_bridge package and executes using the package's installed files.
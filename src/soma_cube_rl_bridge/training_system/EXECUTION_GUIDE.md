# SomaCube RL Training System - Execution Guide

## Overview
This guide shows how to execute the SomaCube RL training system that is now properly packaged as a ROS2 package.

## Package Structure
The training system has been converted into a proper ROS2 package located at:
```
/home/jack/ros2_ws/src/soma_cube_rl_training/
```

## Available Commands
After building, the following ROS2 commands are available:

1. **rl_client** - Basic RL interface client for testing
2. **demo_training** - Interactive demo system
3. **train_somacube** - Main training script with full CLI

## Step-by-Step Execution

### 1. Build the Complete System
```bash
cd /home/jack/ros2_ws
colcon build --packages-select soma_cube_rl_bridge soma_cube_rl_training
source install/setup.bash
```

### 2. Install Training Dependencies (if needed)
```bash
cd /home/jack/ros2_ws/src/soma_cube_rl_training
pip3 install -r requirements.txt
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

# Test basic RL client connection
ros2 run soma_cube_rl_training rl_client

# Run interactive demo (if dependencies installed)
ros2 run soma_cube_rl_training demo_training
```

### 5. Check System Status
```bash
cd /home/jack/ros2_ws
source install/setup.bash
ros2 run soma_cube_rl_training train_somacube --mode check
```

### 6. Run Training
```bash
cd /home/jack/ros2_ws
source install/setup.bash

# Quick test training (50k steps)
ros2 run soma_cube_rl_training train_somacube --mode train --total-timesteps 50000

# Full training (500k steps) 
ros2 run soma_cube_rl_training train_somacube --mode train --total-timesteps 500000

# Training with custom settings
ros2 run soma_cube_rl_training train_somacube --mode train \
    --total-timesteps 100000 \
    --max-episode-steps 300 \
    --save-freq 25000 \
    --name "my_experiment"
```

### 7. Evaluate Trained Model
```bash
cd /home/jack/ros2_ws
source install/setup.bash
ros2 run soma_cube_rl_training train_somacube --mode eval \
    --model-path /path/to/your/model.zip \
    --eval-episodes 20
```

## Package Integration Benefits
Now that the training system is a proper ROS2 package:

✅ **ROS2 Package Integration**: Fully integrated with ROS2 ecosystem
✅ **Standard Execution**: Use `ros2 run` commands instead of direct Python scripts  
✅ **Dependency Management**: Proper package.xml dependencies
✅ **Installation**: Installs with colcon build system
✅ **Distribution**: Can be shared and installed on other systems
✅ **Launch Integration**: Can be included in launch files
✅ **Service Discovery**: Automatically discovers ROS2 services

## Command Reference

### Training Modes
- `--mode train` - Full training session
- `--mode eval` - Evaluation only (requires --model-path)
- `--mode check` - System status check

### Training Parameters
- `--total-timesteps N` - Training duration (default: 500000)
- `--max-episode-steps N` - Episode length (default: 500)
- `--save-freq N` - Model save frequency (default: 50000)
- `--device auto|cpu|cuda` - Training device (default: auto)

### Evaluation Parameters  
- `--eval-episodes N` - Number of evaluation episodes (default: 10)
- `--model-path PATH` - Path to trained model file
- `--no-render` - Disable rendering during evaluation

### Output Options
- `--log-dir PATH` - Training log directory (default: ./somacube_training)
- `--name NAME` - Experiment name (appended to log dir)

## Troubleshooting

### Service Connection Issues
If RL client cannot connect:
1. Ensure RL bridge is running: `ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true`
2. Check services: `ros2 service list | grep /rl/`
3. Verify robot safety state

### Missing Dependencies
If training commands fail:
```bash
pip3 install stable-baselines3[extra] torch gymnasium numpy matplotlib
```

### Build Issues
If colcon build fails:
```bash
cd /home/jack/ros2_ws
colcon build --packages-select soma_cube_rl_training --cmake-clean-cache
```

## Success Verification
The system is working correctly if:
1. ✅ Package builds without errors: `colcon build --packages-select soma_cube_rl_training`  
2. ✅ Commands are available: `ros2 pkg executables soma_cube_rl_training`
3. ✅ RL client can connect when bridge is running
4. ✅ Training system check passes: `ros2 run soma_cube_rl_training train_somacube --mode check`

## Next Steps
- Install training dependencies with `pip3 install -r requirements.txt`
- Start with system check: `ros2 run soma_cube_rl_training train_somacube --mode check`
- Begin with short training: `ros2 run soma_cube_rl_training train_somacube --mode train --total-timesteps 50000`
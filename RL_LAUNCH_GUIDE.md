# 🚀 M0609 RL Package Launch Guide

## Prerequisites
Always source your workspace first:
```bash
cd /home/rokey/ros2_ws
source install/setup.bash
```

## 🎯 Launch Options

### 1. **Virtual Training (Safe - Start Here!)**
Perfect for testing and initial development:

```bash
# Basic virtual training
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py --virtual

# Advanced virtual training with options
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py \
    --virtual \
    --episodes 25000 \
    --device cuda \
    --wandb \
    --save-interval 500 \
    --eval-interval 250
```

### 2. **Real Robot Training** 
⚠️ **IMPORTANT**: Ensure robot is in teachout/manual mode first!

```bash
# Step 1: Start robot connection (in separate terminal)
source install/setup.bash
ros2 launch dsr_tests dsr_bringup_without_spawner_test.launch.py

# Step 2: Start RL training (in another terminal)  
source install/setup.bash
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py \
    --robot-id dsr01 \
    --robot-model m0609 \
    --episodes 10000
```

### 3. **Real Robot with MoveIt Integration**
For advanced motion planning:

```bash
# Terminal 1: Start robot with MoveIt
source install/setup.bash
ros2 launch dsr_moveit_config_m0609 start.launch.py

# Terminal 2: Start RL training
source install/setup.bash
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py \
    --robot-id dsr01 \
    --robot-model m0609 \
    --episodes 15000 \
    --device cuda
```

## 🧪 Evaluation and Testing

### Evaluate a Trained Model
```bash
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/evaluate.py \
    --model-path models/best_model_ep_15000.pth \
    --episodes 100 \
    --robot-id dsr01 \
    --virtual
```

### Quick Environment Test
```bash
python3 -c "
from m0609_block_assembly_rl.environment import M0609BlockAssemblyEnv
env = M0609BlockAssemblyEnv(virtual_mode=True)
obs, _ = env.reset()
print('✅ Environment ready! State shape:', obs.shape)
"
```

## 📊 Training Parameters Explained

| Parameter | Description | Default | Recommended |
|-----------|-------------|---------|-------------|
| `--episodes` | Number of training episodes | 50000 | 25000 (virtual), 10000 (real) |
| `--robot-id` | Robot namespace | dsr01 | dsr01 |
| `--robot-model` | Robot model | m0609 | m0609 |
| `--virtual` | Use simulation mode | True | Start with True |
| `--device` | Training device | auto | cuda (if available) |
| `--wandb` | Enable experiment tracking | False | True (for monitoring) |
| `--save-interval` | Model save frequency | 1000 | 500-1000 |
| `--eval-interval` | Evaluation frequency | 500 | 250-500 |

## 🔄 Typical Training Workflow

### Phase 1: Virtual Development (Safe)
```bash
# 1. Start with short virtual training
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py \
    --virtual --episodes 5000

# 2. Monitor progress in logs/ directory
# 3. Evaluate virtual performance
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/evaluate.py \
    --model-path models/best_model.pth --episodes 50 --virtual
```

### Phase 2: Real Robot Transfer
```bash
# 1. Ensure robot is ready
ros2 launch dsr_tests dsr_bringup_without_spawner_test.launch.py

# 2. Start with conservative real robot training
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py \
    --robot-id dsr01 --robot-model m0609 --episodes 2000

# 3. Monitor robot behavior closely
# 4. Scale up if working well
```

## 📁 Output Files

Your training will create:
- `models/` - Saved model checkpoints
- `logs/` - Training logs and metrics
- `evaluation_logs/` - Evaluation results and plots

## 🚨 Safety Reminders

1. **Always start with virtual mode** to test your setup
2. **Robot must be in teachout/manual mode** for real training
3. **Keep emergency stop accessible** during real robot training
4. **Clear workspace** of obstacles before training
5. **Supervise closely** during real robot sessions
6. **Start with few episodes** (1000-2000) for real robot

## 🐛 Troubleshooting

### "No module named 'm0609_block_assembly_rl'"
```bash
# Make sure you're in the right directory and sourced
cd /home/rokey/ros2_ws
source install/setup.bash
```

### "Robot connection failed"
```bash
# Check robot status
ping 192.168.1.100
nc -zv 192.168.1.100 12345

# Restart robot connection
ros2 launch dsr_tests dsr_bringup_without_spawner_test.launch.py
```

### "CUDA out of memory"
```bash
# Use CPU instead
python3 ... --device cpu
```

## 🎯 Performance Goals

- **Assembly Time**: Reduce from 10 min → 5 min
- **Success Rate**: Achieve 95%+
- **Motion Efficiency**: 0.8+ ratio
- **Learning**: Show consistent improvement

## 🎉 Quick Start Command

**For immediate testing:**
```bash
cd /home/rokey/ros2_ws
source install/setup.bash
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py --virtual --episodes 1000
```

**For real robot (after virtual testing):**
```bash
# Terminal 1
source install/setup.bash  
ros2 launch dsr_tests dsr_bringup_without_spawner_test.launch.py

# Terminal 2  
source install/setup.bash
python3 src/DoosanBootcamp3rd/m0609_block_assembly_rl/m0609_block_assembly_rl/train.py --robot-id dsr01 --robot-model m0609 --episodes 2000
```

---

**Ready to start your RL training! 🤖🧠**
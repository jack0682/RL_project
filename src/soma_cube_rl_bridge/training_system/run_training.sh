#!/bin/bash

# SomaCube RL Training System Launcher
# This script makes it easy to run the training system components

# Get the directory of this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TRAINING_DIR="$SCRIPT_DIR/soma_cube_rl_training"

# Source ROS2 setup
source /home/jack/ros2_ws/install/setup.bash

# Function to show usage
show_usage() {
    echo "🤖 SomaCube RL Training System"
    echo "Usage: $0 [COMMAND] [OPTIONS]"
    echo ""
    echo "Commands:"
    echo "  rl_client      - Test RL client connection"
    echo "  demo          - Run interactive demo"
    echo "  check         - Check system status"
    echo "  train         - Start training (requires dependencies)"
    echo ""
    echo "Training options:"
    echo "  --steps N     - Training steps (default: 500000)"
    echo "  --name NAME   - Experiment name"
    echo "  --device auto|cpu|cuda - Device (default: auto)"
    echo ""
    echo "Examples:"
    echo "  $0 rl_client"
    echo "  $0 check"
    echo "  $0 train --steps 50000 --name test_run"
    echo ""
    echo "Note: Install training dependencies first:"
    echo "  pip3 install -r $SCRIPT_DIR/requirements.txt"
}

# Parse command
case "$1" in
    "rl_client")
        echo "🔗 Running RL client test..."
        timeout 10s python3 "$TRAINING_DIR/rl_client.py"
        ;;
    "demo")
        echo "🎮 Running training demo..."
        echo "Press Ctrl+C to cancel, Enter to continue"
        python3 "$TRAINING_DIR/demo_training.py"
        ;;
    "check")
        echo "🔍 Checking system status..."
        python3 "$TRAINING_DIR/train_somacube.py" --mode check
        ;;
    "train")
        shift  # Remove 'train' from arguments
        echo "🚀 Starting training..."
        echo "Arguments: $@"
        python3 "$TRAINING_DIR/train_somacube.py" --mode train "$@"
        ;;
    "eval")
        shift  # Remove 'eval' from arguments
        echo "📊 Running evaluation..."
        python3 "$TRAINING_DIR/train_somacube.py" --mode eval "$@"
        ;;
    *)
        show_usage
        ;;
esac
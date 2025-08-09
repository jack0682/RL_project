# SomaCube Assembly RL Bridge

A reinforcement learning bridge for SomaCube assembly using the Doosan M0609 robot arm. This package provides a Gym-like interface that wraps the existing DoosanBootcamp3rd motion control packages with safety monitoring and RL-friendly endpoints.

## Overview

The `soma_cube_rl_bridge` package consists of three main nodes:

- **safety_monitor_node**: Centralized safety monitoring and state management
- **motion_proxy_node**: Safe proxy layer for robot motion control 
- **rl_env_node**: Gym-like RL environment with `/rl/reset` and `/rl/step` services

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│   RL Trainer    │◄──►│   rl_env_node    │◄──►│  motion_proxy_node  │
│                 │    │                  │    │                     │
│ /rl/reset       │    │ Gym Interface    │    │ Safety Gating       │
│ /rl/step        │    │ Reward Calc      │    │ Limit Enforcement   │
└─────────────────┘    └──────────────────┘    └─────────────────────┘
                                │                          │
                                ▼                          ▼
                       ┌──────────────────┐    ┌─────────────────────┐
                       │ safety_monitor   │    │ DoosanBootcamp3rd   │
                       │                  │    │                     │
                       │ E-Stop Monitor   │    │ /motion/move_joint  │
                       │ Joint Limits     │    │ /motion/move_stop   │
                       │ Workspace Check  │    │ /system/*           │
                       └──────────────────┘    └─────────────────────┘
```

## Interface Mapping

### Existing DoosanBootcamp3rd → RL Bridge

| DoosanBootcamp3rd Interface | Bridge Interface | Purpose |
|----------------------------|------------------|----------|
| `/motion/move_joint` | `soma_cube_rl_bridge/move_joint` | Joint space motion |
| `/motion/move_stop` | `soma_cube_rl_bridge/move_stop` | Emergency stop |
| `/joint_states` | `soma_cube_rl_bridge/joint_states` | Joint state feedback |
| `/msg/current_posx` | `soma_cube_rl_bridge/tcp_pose` | TCP pose feedback |
| `/error` | `safety/state` | Error monitoring |
| `/system/get_robot_state` | `safety/get_state` | Safety state query |

### New RL-Specific Interfaces

| Service/Topic | Type | Description |
|---------------|------|-------------|
| `/rl/reset` | `soma_cube_rl_bridge/srv/RLReset` | Reset environment, return initial observation |
| `/rl/step` | `soma_cube_rl_bridge/srv/RLStep` | Execute action, return obs/reward/done/info |
| `safety/get_state` | `soma_cube_rl_bridge/srv/GetSafetyState` | Query current safety state |
| `safety/state` | `soma_cube_rl_bridge/msg/SafetyState` | Safety state publishing |

## Parameters

### Safety Monitor Node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `monitor_rate_hz` | double | 10.0 | Safety monitoring frequency (Hz) |
| `joint_min_deg` | double[] | [-170,-135,-169,-90,-135,-360] | Joint min limits (degrees) |
| `joint_max_deg` | double[] | [170,135,169,90,135,360] | Joint max limits (degrees) |
| `tcp_workspace_min` | double[] | [-1000,-1000,0] | TCP workspace min (mm) |
| `tcp_workspace_max` | double[] | [1000,1000,2000] | TCP workspace max (mm) |

### Motion Proxy Node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_rate_hz` | double | 20.0 | State publishing frequency (Hz) |
| `vel_limit_deg_s` | double[] | [50,50,50,50,50,50] | Velocity limits (deg/s) |
| `acc_limit_deg_s2` | double[] | [100,100,100,100,100,100] | Acceleration limits (deg/s²) |
| `action_timeout_sec` | double | 30.0 | Action execution timeout (s) |
| `service_wait_sec` | double | 5.0 | Service connection timeout (s) |

### RL Environment Node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_episode_steps` | int | 1000 | Maximum steps per episode |
| `reward_weights.grasp_success` | double | 100.0 | Reward for successful grasp |
| `reward_weights.assembly_alignment` | double | 50.0 | Reward for assembly alignment |
| `reward_weights.collision_penalty` | double | -100.0 | Penalty for collisions |
| `reward_weights.time_penalty` | double | -0.1 | Per-step time penalty |
| `reward_weights.distance_penalty` | double | -1.0 | Distance-based penalty |
| `reward_weights.velocity_penalty` | double | -0.01 | High velocity penalty |

## Usage

### 1. Build the Package

```bash
cd /home/jack/ros2_ws
colcon build --packages-select soma_cube_rl_bridge
source install/setup.bash
```

### 2. Launch the RL Bridge

For real robot:
```bash
ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=false
```

For simulation:
```bash
ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true
```

### 3. RL Interaction Examples

#### Reset Environment
```bash
ros2 service call /rl/reset soma_cube_rl_bridge/srv/RLReset "{seed: 42}"
```

Expected response:
```yaml
initial_obs: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 400.0, 0.0, 300.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 400.0, 0.0, 1000.0]
success: true
message: "Environment reset successfully"
```

#### Execute Step
```bash
ros2 service call /rl/step soma_cube_rl_bridge/srv/RLStep "{action: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"
```

Expected response:
```yaml
obs: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 450.0, 50.0, 350.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 350.0, 1.0, 999.0]
reward: -1.5
done: false
info: '{"step":1,"total_reward":-1.5,"distance_to_target":350.0,"safety_state":"safe","action_clamped":false}'
success: true
```

#### Check Safety State
```bash
ros2 service call safety/get_state soma_cube_rl_bridge/srv/GetSafetyState "{}"
```

## Observation Space

The observation vector contains (28 elements total):

| Elements | Range | Description |
|----------|-------|-------------|
| 0-5 | [-360°, 360°] | Joint positions (degrees) |
| 6-11 | [-∞, ∞] | Joint velocities (deg/s) |
| 12-14 | [-∞, ∞] | TCP position (mm) |
| 15-18 | [-1, 1] | TCP orientation (quaternion) |
| 19-24 | [-∞, ∞] | Tool force/torque (N, Nm) |
| 25 | [0, ∞] | Distance to target (mm) |
| 26 | [0, max_steps] | Current step count |
| 27 | [0, max_steps] | Remaining steps |

## Action Space

The action space consists of 6 joint target positions in degrees:

| Action | Range | Description |
|--------|-------|-------------|
| 0 | [-170°, 170°] | Joint 1 target position |
| 1 | [-135°, 135°] | Joint 2 target position |  
| 2 | [-169°, 169°] | Joint 3 target position |
| 3 | [-90°, 90°] | Joint 4 target position |
| 4 | [-135°, 135°] | Joint 5 target position |
| 5 | [-360°, 360°] | Joint 6 target position |

Actions outside these ranges are automatically clamped.

## Safety Features

### 1. Multi-layer Safety Monitoring
- **Hardware E-Stop**: Emergency stop button monitoring
- **Protective Stop**: Automatic stops on limit violations
- **Joint Limits**: Software joint position limits
- **Workspace Limits**: TCP workspace boundaries
- **Velocity/Acceleration Limits**: Motion constraint enforcement

### 2. Safe Action Rejection
- Motion commands rejected when `safe_to_move=false`
- Detailed safety violation reasons in service responses
- Automatic episode termination on safety breaches

### 3. Real-time Safety State Publishing
- Continuous safety state broadcasting at 10 Hz
- Safety state changes logged with timestamps
- Integration with ROS2 diagnostics system

## Performance Metrics

### Target Performance
- **Step Latency**: ≤ 150ms end-to-end
- **Safety Response**: ≤ 100ms E-stop propagation
- **State Publishing**: 20 Hz joint states, 10 Hz safety monitoring

### Measurement Commands
```bash
# Measure step latency
ros2 run soma_cube_rl_bridge benchmark_step_latency

# Monitor safety response times  
ros2 topic hz safety/state
ros2 topic hz soma_cube_rl_bridge/joint_states
```

## Configuration Files

### Real Robot (`config/soma_cube_rl_real.yaml`)
- Conservative velocity/acceleration limits
- Strict workspace boundaries  
- Higher safety penalties
- Shorter episode limits

### Simulation (`config/soma_cube_rl_sim.yaml`)
- Higher velocity/acceleration limits
- Larger workspace boundaries
- Lower safety penalties  
- Longer episode limits

## Limitations

### Current Version (1.0.0)
- **Motion Types**: Only MoveJ (joint space) implemented
- **Force Control**: Basic force feedback, no impedance control
- **Gripper Integration**: Placeholder gripper status
- **Multi-robot**: Single robot only
- **Real-time Control**: Uses service calls, not RT streams

### Future Enhancements
- MoveL/MoveC support for Cartesian control
- Real-time streaming motion control integration
- Advanced force/impedance control modes
- Multi-robot coordination capabilities
- Gripper control integration

## Troubleshooting

### Common Issues

#### 1. Build Errors
```bash
# Missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Clean build
rm -rf build/ install/
colcon build --packages-select soma_cube_rl_bridge
```

#### 2. Service Connection Failures
```bash
# Check Doosan services
ros2 service list | grep motion
ros2 service list | grep system

# Check service connectivity
ros2 service call /motion/move_joint dsr_msgs2/srv/MoveJoint "{pos: [0,0,0,0,0,0], vel: 10, acc: 10, time: 0, radius: 0, mode: 0, blend_type: 0, sync_type: 0}"
```

#### 3. Safety State Issues  
```bash
# Check robot mode
ros2 service call /system/get_robot_mode dsr_msgs2/srv/GetRobotMode "{}"

# Check robot state
ros2 service call /system/get_robot_state dsr_msgs2/srv/GetRobotState "{}"

# Monitor error messages
ros2 topic echo /error
```

#### 4. Performance Issues
```bash
# Monitor node CPU usage
top -p $(pgrep -f rl_env_node)

# Check topic rates
ros2 topic hz soma_cube_rl_bridge/joint_states
ros2 topic hz safety/state
```

## Integration with External RL Frameworks

### Python Example (using rclpy)
```python
import rclpy
from rclpy.node import Node
from soma_cube_rl_bridge.srv import RLReset, RLStep

class SomaCubeRLClient(Node):
    def __init__(self):
        super().__init__('somacube_rl_client')
        self.reset_client = self.create_client(RLReset, '/rl/reset')
        self.step_client = self.create_client(RLStep, '/rl/step')
    
    def reset(self, seed=None):
        request = RLReset.Request()
        if seed: request.seed = seed
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result().initial_obs
    
    def step(self, action):
        request = RLStep.Request()
        request.action = action
        future = self.step_client.call_async(request)  
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        return result.obs, result.reward, result.done, result.info
```

### Integration Notes
- All services are blocking and synchronous
- Use `rclpy.spin_until_future_complete()` for service calls
- Handle service timeouts appropriately
- Monitor safety state before and after actions
- Implement proper shutdown procedures

## Contributing

### Development Setup
```bash
# Clone and setup workspace
cd /home/jack/ros2_ws/src/DoosanBootcamp3rd
git status

# Make changes and test
colcon build --packages-select soma_cube_rl_bridge
source install/setup.bash
ros2 launch soma_cube_rl_bridge soma_cube_rl.launch.py sim:=true
```

### Testing
```bash
# Run unit tests (when implemented)
colcon test --packages-select soma_cube_rl_bridge

# Integration testing
ros2 run soma_cube_rl_bridge test_rl_interface
```

## License

MIT License - see package.xml for details.

## Support

For issues and questions:
1. Check this README and troubleshooting section
2. Review ROS2 logs: `ros2 log`
3. Monitor safety state: `ros2 topic echo safety/state`
4. Check Doosan controller connectivity
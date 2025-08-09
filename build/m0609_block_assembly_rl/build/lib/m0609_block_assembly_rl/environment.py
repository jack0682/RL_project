"""
M0609 Block Assembly Reinforcement Learning Environment

This module implements the environment for training RL agents to optimize
Doosan M0609 robot block assembly tasks from 10 minutes to 5 minutes.
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import rclpy
from typing import Dict, List, Tuple, Optional, Any
import time

# Try to import Doosan robot modules (optional for virtual mode)
try:
    import DR_init
    from DSR_ROBOT2 import *
    from DR_common2 import *
    DOOSAN_AVAILABLE = True
except ImportError:
    print("Warning: Doosan robot modules not available, running in virtual-only mode")
    DOOSAN_AVAILABLE = False
    
    # Mock functions for virtual mode
    def set_tool(*args, **kwargs):
        pass
    def set_tcp(*args, **kwargs):
        pass
    def movej(*args, **kwargs):
        pass
    def movel(*args, **kwargs):
        pass
    def set_digital_output(*args, **kwargs):
        pass
    def get_digital_input(*args, **kwargs):
        return True
    def get_current_posj(*args, **kwargs):
        return [0, 0, 90, 0, 90, 0]
    def wait(*args, **kwargs):
        time.sleep(args[0] if args else 0.1)
    def posx(coords):
        return coords

class BlockType:
    """Block types with specific characteristics"""
    L_SHAPED = 0      # High rotation constraints
    RECTANGULAR = 1   # Various grasp points
    T_SHAPED = 2      # Order-dependent placement
    STEPPED = 3       # Requires angle adjustment
    CROSS = 4         # Prefers center placement
    PILLAR = 5        # Recommended for last placement
    TRIANGULAR = 6    # Stability required

class M0609BlockAssemblyEnv(gym.Env):
    """
    M0609 Block Assembly Environment for Reinforcement Learning
    
    State Space (179-dimensional):
    - Assembly progress: 7-dim one-hot encoding
    - Remaining blocks: 7-dim binary encoding
    - Current structure: 3x3x3 grid (27-dim)
    - Block positions: 7x3 xyz coordinates (21-dim)
    - Block orientations: 7x3 euler angles (21-dim)
    - Accessibility state: 64-dim
    - Strategic state: 32-dim
    
    Action Space:
    - Strategic choice: discrete (next block selection)
    - Grasp optimization: continuous (position, orientation, force)
    - Fine tuning: continuous (placement offset, rotation adjustment)
    """
    
    metadata = {'render.modes': ['human', 'rgb_array']}
    
    def __init__(self, 
                 robot_id: str = "dsr01",
                 robot_model: str = "m0609",
                 virtual_mode: bool = True,
                 render_mode: Optional[str] = None):
        super().__init__()
        
        self.robot_id = robot_id
        self.robot_model = robot_model
        self.virtual_mode = virtual_mode
        self.render_mode = render_mode
        
        # Initialize ROS2 and robot connection
        self._init_robot_connection()
        
        # Environment parameters
        self.num_blocks = 7
        self.workspace_bounds = np.array([
            [300, 700],   # X bounds (mm)
            [-400, 400],  # Y bounds (mm)
            [0, 500]      # Z bounds (mm)
        ])
        
        # Block specifications
        self.block_specs = self._init_block_specs()
        
        # Target structure (3x3x3 cube)
        self.target_positions = self._init_target_positions()
        
        # State and action space definitions
        self._init_spaces()
        
        # Episode variables
        self.current_step = 0
        self.max_steps = 7  # 7 blocks to place
        self.start_time = None
        self.episode_rewards = []
        
        # State tracking
        self.blocks_placed = np.zeros(self.num_blocks, dtype=bool)
        self.block_positions = np.zeros((self.num_blocks, 3))
        self.block_orientations = np.zeros((self.num_blocks, 3))
        self.assembly_grid = np.zeros((3, 3, 3), dtype=int)
        
    def _init_robot_connection(self):
        """Initialize robot connection and ROS2 node"""
        try:
            if not rclpy.ok():
                rclpy.init()
                
            self.node = rclpy.create_node(
                f"m0609_assembly_env_{int(time.time())}", 
                namespace=self.robot_id
            )
            
            # Initialize Doosan robot connection if available
            if DOOSAN_AVAILABLE:
                DR_init.__dsr__id = self.robot_id
                DR_init.__dsr__model = self.robot_model
                DR_init.__dsr__node = self.node
            
            # Set up tool and TCP (only in non-virtual mode)
            if not self.virtual_mode and DOOSAN_AVAILABLE:
                try:
                    set_tool("Tool Weight_2FG")
                    set_tcp("2FG_TCP")
                    print(f"Robot {self.robot_id} ({self.robot_model}) initialized successfully")
                except Exception as e:
                    print(f"Warning: Could not initialize robot tools: {e}")
            else:
                print(f"Virtual mode: Robot {self.robot_id} ({self.robot_model}) simulation initialized")
                
        except Exception as e:
            print(f"Warning: ROS2 connection failed, running in standalone simulation mode: {e}")
            self.node = None
            self.virtual_mode = True
    
    def _init_block_specs(self) -> Dict[int, Dict]:
        """Initialize block specifications with physical properties"""
        return {
            BlockType.L_SHAPED: {
                'name': 'L-Block',
                'weight': 1.5,  # kg
                'size': [80, 80, 40],  # mm
                'grasp_points': [[40, 20, 20], [20, 40, 20]],
                'rotation_constraint': 0.8,  # High constraint
                'stability_factor': 0.7
            },
            BlockType.RECTANGULAR: {
                'name': 'Rectangle-Block', 
                'weight': 2.0,
                'size': [120, 60, 40],
                'grasp_points': [[60, 30, 20], [30, 60, 20], [90, 30, 20]],
                'rotation_constraint': 0.3,  # Low constraint
                'stability_factor': 0.9
            },
            BlockType.T_SHAPED: {
                'name': 'T-Block',
                'weight': 1.8,
                'size': [100, 80, 40],
                'grasp_points': [[50, 20, 20], [50, 60, 20]],
                'rotation_constraint': 0.6,
                'stability_factor': 0.6
            },
            BlockType.STEPPED: {
                'name': 'Step-Block',
                'weight': 2.2,
                'size': [100, 100, 60],
                'grasp_points': [[25, 50, 30], [75, 50, 30]],
                'rotation_constraint': 0.9,
                'stability_factor': 0.8
            },
            BlockType.CROSS: {
                'name': 'Cross-Block',
                'weight': 1.6,
                'size': [90, 90, 40],
                'grasp_points': [[45, 45, 20]],
                'rotation_constraint': 0.4,
                'stability_factor': 0.5
            },
            BlockType.PILLAR: {
                'name': 'Pillar-Block',
                'weight': 2.5,
                'size': [40, 40, 120],
                'grasp_points': [[20, 20, 60], [20, 20, 90]],
                'rotation_constraint': 0.7,
                'stability_factor': 0.9
            },
            BlockType.TRIANGULAR: {
                'name': 'Triangle-Block',
                'weight': 1.3,
                'size': [80, 80, 40],
                'grasp_points': [[40, 27, 20], [27, 40, 20]],
                'rotation_constraint': 0.8,
                'stability_factor': 0.4
            }
        }
    
    def _init_target_positions(self) -> np.ndarray:
        """Initialize target positions for 3x3x3 assembly structure"""
        positions = []
        base_x, base_y, base_z = 500, 0, 50  # Center of workspace
        block_spacing = 80  # mm between block centers
        
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    x = base_x + (i - 1) * block_spacing
                    y = base_y + (j - 1) * block_spacing
                    z = base_z + k * 40  # Height of each layer
                    positions.append([x, y, z])
        
        return np.array(positions[:self.num_blocks])  # Use first 7 positions
    
    def _init_spaces(self):
        """Initialize observation and action spaces"""
        # Observation space (179-dimensional)
        obs_low = np.full(179, -np.inf)
        obs_high = np.full(179, np.inf)
        self.observation_space = spaces.Box(
            low=obs_low, high=obs_high, dtype=np.float32
        )
        
        # Action space - hierarchical structure
        self.action_space = spaces.Dict({
            # Strategic choice (discrete)
            'next_block': spaces.Discrete(self.num_blocks),
            'approach_strategy': spaces.Discrete(3),  # direct/detour/optimal
            
            # Grasp optimization (continuous)  
            'grasp_position': spaces.Box(
                low=-1.0, high=1.0, shape=(3,), dtype=np.float32
            ),
            'grasp_orientation': spaces.Box(
                low=-np.pi, high=np.pi, shape=(3,), dtype=np.float32
            ),
            'grasp_force': spaces.Box(
                low=0.1, high=1.0, shape=(1,), dtype=np.float32
            ),
            
            # Fine tuning (continuous)
            'placement_offset': spaces.Box(
                low=-0.02, high=0.02, shape=(3,), dtype=np.float32
            ),
            'rotation_adjustment': spaces.Box(
                low=-np.pi/12, high=np.pi/12, shape=(3,), dtype=np.float32
            )
        })
    
    def reset(self, seed: Optional[int] = None, options: Optional[Dict] = None) -> Tuple[np.ndarray, Dict]:
        """Reset environment to initial state"""
        super().reset(seed=seed)
        
        self.current_step = 0
        self.start_time = time.time()
        self.episode_rewards = []
        
        # Reset block states
        self.blocks_placed.fill(False)
        self.block_positions = self._generate_initial_block_positions()
        self.block_orientations = self._generate_initial_block_orientations()
        self.assembly_grid.fill(0)
        
        # Move robot to home position
        self._move_to_home_position()
        
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, info
    
    def step(self, action: Dict) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute one step in the environment"""
        if self.current_step >= self.max_steps:
            raise ValueError("Episode is already terminated")
        
        # Extract action components
        next_block = action['next_block']
        approach_strategy = action['approach_strategy']
        grasp_pos = action['grasp_position']
        grasp_orient = action['grasp_orientation']
        grasp_force = action['grasp_force'][0]
        placement_offset = action['placement_offset']
        rotation_adj = action['rotation_adjustment']
        
        # Check if block is already placed
        if self.blocks_placed[next_block]:
            reward = -100  # Penalty for selecting already placed block
            observation = self._get_observation()
            return observation, reward, False, False, self._get_info()
        
        # Execute assembly action
        success, execution_time, motion_efficiency = self._execute_assembly(
            next_block, approach_strategy, grasp_pos, grasp_orient, 
            grasp_force, placement_offset, rotation_adj
        )
        
        # Calculate reward
        reward = self._calculate_reward(
            next_block, success, execution_time, motion_efficiency
        )
        
        self.episode_rewards.append(reward)
        
        if success:
            self.blocks_placed[next_block] = True
            self._update_assembly_state(next_block, placement_offset, rotation_adj)
        
        self.current_step += 1
        
        # Check termination conditions
        terminated = self.current_step >= self.max_steps or np.all(self.blocks_placed)
        truncated = time.time() - self.start_time > 5400  # 90 minutes timeout
        
        observation = self._get_observation()
        info = self._get_info()
        
        if terminated:
            info['episode_time'] = time.time() - self.start_time
            info['total_reward'] = sum(self.episode_rewards)
            info['success_rate'] = np.sum(self.blocks_placed) / self.num_blocks
        
        return observation, reward, terminated, truncated, info
    
    def _generate_initial_block_positions(self) -> np.ndarray:
        """Generate random initial positions for blocks on workspace"""
        positions = []
        for _ in range(self.num_blocks):
            x = np.random.uniform(self.workspace_bounds[0, 0], self.workspace_bounds[0, 1])
            y = np.random.uniform(self.workspace_bounds[1, 0], self.workspace_bounds[1, 1])
            z = 25  # On table surface
            positions.append([x, y, z])
        return np.array(positions)
    
    def _generate_initial_block_orientations(self) -> np.ndarray:
        """Generate random initial orientations for blocks"""
        orientations = []
        for _ in range(self.num_blocks):
            # Random rotation around Z-axis, limited rotation around X,Y
            rx = np.random.uniform(-np.pi/6, np.pi/6)
            ry = np.random.uniform(-np.pi/6, np.pi/6)  
            rz = np.random.uniform(-np.pi, np.pi)
            orientations.append([rx, ry, rz])
        return np.array(orientations)
    
    def _move_to_home_position(self):
        """Move robot to home position"""
        if self.virtual_mode:
            print("Virtual mode: Moving to home position (simulated)")
            return
        
        try:
            home_joints = [0, 0, 90, 0, 90, 0]
            movej(home_joints, vel=60, acc=60)
        except Exception as e:
            print(f"Warning: Could not move to home position: {e}")
    
    def _execute_assembly(self, block_id: int, approach_strategy: int,
                         grasp_pos: np.ndarray, grasp_orient: np.ndarray,
                         grasp_force: float, placement_offset: np.ndarray,
                         rotation_adj: np.ndarray) -> Tuple[bool, float, float]:
        """
        Execute assembly action using Doosan robot
        
        Returns:
            success: Whether assembly was successful
            execution_time: Time taken for execution
            motion_efficiency: Efficiency score (0-1)
        """
        start_time = time.time()
        
        # In virtual mode, simulate the assembly process
        if self.virtual_mode:
            return self._simulate_assembly_execution(
                block_id, approach_strategy, grasp_pos, grasp_orient, 
                grasp_force, placement_offset, rotation_adj
            )
        
        try:
            # Calculate grasp point
            block_pos = self.block_positions[block_id]
            block_spec = self.block_specs[block_id]
            
            # Select grasp point based on action
            base_grasp = block_spec['grasp_points'][0]  # Use first grasp point as base
            actual_grasp = block_pos + grasp_pos * 50  # Scale relative position
            
            # Calculate target position with offset
            target_pos = self.target_positions[block_id] + placement_offset * 1000  # mm
            target_orientation = grasp_orient + rotation_adj
            
            # Approach strategies
            if approach_strategy == 0:  # Direct
                # Direct path to grasp
                pre_grasp_pos = actual_grasp + [0, 0, 100]  # 10cm above
                trajectory = [pre_grasp_pos, actual_grasp, target_pos]
            elif approach_strategy == 1:  # Detour
                # Detour path to avoid obstacles
                safe_height = max(block_pos[2] + 150, 200)
                detour_pos = [actual_grasp[0], actual_grasp[1], safe_height]
                trajectory = [detour_pos, actual_grasp, target_pos]
            else:  # Optimal (approach_strategy == 2)
                # Optimized path based on current configuration
                trajectory = self._calculate_optimal_trajectory(actual_grasp, target_pos)
            
            # Execute trajectory
            motion_distance = 0
            for i, pos in enumerate(trajectory):
                if i < len(trajectory) - 1:
                    # Convert to pose
                    pose = posx([pos[0], pos[1], pos[2], 
                               target_orientation[0], target_orientation[1], target_orientation[2]])
                    
                    # Move to position
                    if i == 1:  # Grasp position
                        movel(pose, vel=30, acc=30)  # Slower for precision
                        self._execute_grip(True)  # Close gripper
                    else:
                        movel(pose, vel=60, acc=60)
                    
                    # Calculate motion distance for efficiency
                    if i > 0:
                        motion_distance += np.linalg.norm(np.array(pos) - np.array(trajectory[i-1]))
            
            # Place block and release
            final_pose = posx([target_pos[0], target_pos[1], target_pos[2],
                              target_orientation[0], target_orientation[1], target_orientation[2]])
            movel(final_pose, vel=20, acc=20)  # Very slow for placement
            self._execute_grip(False)  # Open gripper
            
            # Move up after placement
            up_pose = posx([target_pos[0], target_pos[1], target_pos[2] + 100,
                           target_orientation[0], target_orientation[1], target_orientation[2]])
            movel(up_pose, vel=40, acc=40)
            
            execution_time = time.time() - start_time
            
            # Calculate motion efficiency (inverse of motion distance)
            optimal_distance = np.linalg.norm(target_pos - actual_grasp)
            motion_efficiency = min(1.0, optimal_distance / max(motion_distance, optimal_distance))
            
            return True, execution_time, motion_efficiency
            
        except Exception as e:
            print(f"Assembly execution failed: {e}")
            execution_time = time.time() - start_time
            return False, execution_time, 0.0
    
    def _simulate_assembly_execution(self, block_id: int, approach_strategy: int,
                                   grasp_pos: np.ndarray, grasp_orient: np.ndarray,
                                   grasp_force: float, placement_offset: np.ndarray,
                                   rotation_adj: np.ndarray) -> Tuple[bool, float, float]:
        """
        Simulate assembly execution for virtual mode
        """
        # Simulate realistic execution time
        base_time = 30 + np.random.normal(0, 5)  # Base 30s ± 5s
        
        # Add complexity based on block characteristics
        block_spec = self.block_specs[block_id]
        complexity_time = block_spec['rotation_constraint'] * 10
        weight_time = (block_spec['weight'] - 1.0) * 5  # Extra time for heavier blocks
        
        # Strategy impact on time
        strategy_multipliers = {0: 1.0, 1: 1.3, 2: 0.8}  # direct, detour, optimal
        strategy_multiplier = strategy_multipliers.get(approach_strategy, 1.0)
        
        execution_time = (base_time + complexity_time + weight_time) * strategy_multiplier
        execution_time = max(10, execution_time)  # Minimum 10 seconds
        
        # Simulate execution time (shortened for training)
        actual_wait = min(0.1, execution_time / 100)  # Scale down for simulation
        time.sleep(actual_wait)
        
        # Calculate success probability based on action quality
        block_pos = self.block_positions[block_id]
        target_pos = self.target_positions[block_id]
        
        # Distance factor (closer is better)
        distance = np.linalg.norm(target_pos - block_pos)
        distance_factor = max(0.3, 1.0 - distance / 1000)  # Normalize to workspace size
        
        # Grasp quality factor
        grasp_quality = 1.0 - np.linalg.norm(grasp_pos)  # Centered grasps are better
        grasp_quality = max(0.1, grasp_quality)
        
        # Force factor (moderate forces are better)
        force_quality = 1.0 - abs(grasp_force - 0.5) * 2  # Optimal around 0.5
        
        # Strategy factor
        strategy_success_rates = {0: 0.8, 1: 0.9, 2: 0.95}  # direct, detour, optimal
        strategy_factor = strategy_success_rates.get(approach_strategy, 0.8)
        
        # Block difficulty factor
        block_difficulty = block_spec['rotation_constraint']
        difficulty_factor = 1.0 - block_difficulty * 0.3
        
        # Combined success probability
        success_prob = (distance_factor * 0.3 + 
                       grasp_quality * 0.2 + 
                       force_quality * 0.1 + 
                       strategy_factor * 0.2 + 
                       difficulty_factor * 0.2)
        
        success = np.random.random() < success_prob
        
        # Motion efficiency (how optimal was the path)
        efficiency_factors = [
            1.0 - np.linalg.norm(placement_offset) * 10,  # Small offsets are better
            1.0 - np.linalg.norm(rotation_adj) * 2,       # Small adjustments are better
            strategy_factor,                               # Strategy choice
            force_quality                                  # Force control
        ]
        
        motion_efficiency = np.mean(efficiency_factors)
        motion_efficiency = max(0.0, min(1.0, motion_efficiency))
        
        return success, execution_time, motion_efficiency
    
    def _execute_grip(self, grip: bool):
        """Execute gripper control"""
        if self.virtual_mode:
            action = "closing" if grip else "opening"
            print(f"Virtual mode: {action} gripper (simulated)")
            time.sleep(0.01)  # Small delay for realism
            return
            
        try:
            if grip:
                # Close gripper
                set_digital_output(1, 1)  # Close signal
                set_digital_output(2, 0)  # Release signal off
                # Wait for grip confirmation
                timeout = 10
                start = time.time()
                while not get_digital_input(1) and (time.time() - start) < timeout:
                    wait(0.1)
            else:
                # Open gripper  
                set_digital_output(2, 1)  # Release signal
                set_digital_output(1, 0)  # Close signal off
                # Wait for release confirmation
                timeout = 10
                start = time.time()
                while not get_digital_input(2) and (time.time() - start) < timeout:
                    wait(0.1)
        except Exception as e:
            print(f"Gripper control failed: {e}")
    
    def _calculate_optimal_trajectory(self, start_pos: np.ndarray, 
                                    end_pos: np.ndarray) -> List[np.ndarray]:
        """Calculate optimal trajectory avoiding collisions"""
        # Simple trajectory planning - can be enhanced with proper path planning
        mid_height = max(start_pos[2], end_pos[2]) + 100
        mid_pos = [(start_pos[0] + end_pos[0])/2, 
                  (start_pos[1] + end_pos[1])/2, 
                  mid_height]
        
        return [
            start_pos + [0, 0, 50],  # Pre-grasp
            start_pos,               # Grasp
            mid_pos,                 # Mid-point
            end_pos                  # Target
        ]
    
    def _update_assembly_state(self, block_id: int, 
                              placement_offset: np.ndarray, 
                              rotation_adj: np.ndarray):
        """Update assembly state after successful placement"""
        # Update block position
        self.block_positions[block_id] = self.target_positions[block_id] + placement_offset * 1000
        
        # Update block orientation
        self.block_orientations[block_id] += rotation_adj
        
        # Update 3D grid (simplified - map to grid coordinates)
        grid_pos = self._world_to_grid_coords(self.block_positions[block_id])
        if self._is_valid_grid_pos(grid_pos):
            self.assembly_grid[grid_pos[0], grid_pos[1], grid_pos[2]] = block_id + 1
    
    def _world_to_grid_coords(self, world_pos: np.ndarray) -> Tuple[int, int, int]:
        """Convert world coordinates to grid coordinates"""
        # Simplified mapping to 3x3x3 grid
        base_x, base_y, base_z = 500, 0, 50
        block_spacing = 80
        
        grid_x = int(round((world_pos[0] - base_x) / block_spacing)) + 1
        grid_y = int(round((world_pos[1] - base_y) / block_spacing)) + 1
        grid_z = int(round((world_pos[2] - base_z) / 40))
        
        return (grid_x, grid_y, grid_z)
    
    def _is_valid_grid_pos(self, grid_pos: Tuple[int, int, int]) -> bool:
        """Check if grid position is valid"""
        return all(0 <= pos < 3 for pos in grid_pos)
    
    def _calculate_reward(self, block_id: int, success: bool, 
                         execution_time: float, motion_efficiency: float) -> float:
        """Calculate reward for current action"""
        reward = 0.0
        
        if success:
            # Base success reward
            time_bonus = max(0, 200 - execution_time)  # Bonus for speed
            reward += 100 + time_bonus
            
            # Motion efficiency reward  
            reward += motion_efficiency * 100
            
            # Grasp quality reward (simplified)
            block_spec = self.block_specs[block_id]
            grasp_quality = 1.0 - block_spec['rotation_constraint']  # Easier grasp = higher reward
            reward += grasp_quality * 50
            
            # Sequence optimization reward
            remaining_blocks = ~self.blocks_placed
            if np.sum(remaining_blocks) > 0:
                sequence_wisdom = self._evaluate_sequence_choice(block_id)
                reward += sequence_wisdom * 80
            
        else:
            # Failure penalties
            reward -= 100  # Base failure penalty
            if execution_time > 180:  # Over 3 minutes
                reward -= 50  # Time penalty
        
        return reward
    
    def _evaluate_sequence_choice(self, block_id: int) -> float:
        """Evaluate wisdom of block selection sequence"""
        # Simplified heuristic - prefer lighter blocks first, complex shapes last
        block_spec = self.block_specs[block_id]
        remaining_count = np.sum(~self.blocks_placed)
        
        # Prefer lighter blocks when many remain
        weight_factor = (3.0 - block_spec['weight']) / 3.0
        
        # Prefer stable blocks for foundation
        stability_bonus = 0.0
        if self.current_step < 3:  # First 3 blocks
            stability_bonus = block_spec['stability_factor']
        
        # Penalty for placing complex blocks early
        complexity_penalty = 0.0
        if self.current_step < 2 and block_spec['rotation_constraint'] > 0.7:
            complexity_penalty = -0.3
        
        return weight_factor + stability_bonus + complexity_penalty
    
    def _get_observation(self) -> np.ndarray:
        """Get current observation vector (179-dimensional)"""
        obs = []
        
        # Assembly progress (7-dim one-hot)
        progress_onehot = np.zeros(7)
        placed_count = np.sum(self.blocks_placed)
        if placed_count < 7:
            progress_onehot[placed_count] = 1.0
        else:
            progress_onehot[6] = 1.0
        obs.extend(progress_onehot)
        
        # Remaining blocks (7-dim binary)
        remaining = (~self.blocks_placed).astype(np.float32)
        obs.extend(remaining)
        
        # Current structure (3x3x3 grid = 27-dim)
        obs.extend(self.assembly_grid.flatten())
        
        # Block positions (7x3 = 21-dim)
        obs.extend(self.block_positions.flatten())
        
        # Block orientations (7x3 = 21-dim)
        obs.extend(self.block_orientations.flatten())
        
        # Accessibility state (64-dim)
        accessibility = self._calculate_accessibility_state()
        obs.extend(accessibility)
        
        # Strategic state (32-dim)
        strategic = self._calculate_strategic_state()
        obs.extend(strategic)
        
        return np.array(obs, dtype=np.float32)
    
    def _calculate_accessibility_state(self) -> List[float]:
        """Calculate accessibility metrics for each block and position"""
        accessibility = []
        
        # Grasp difficulty for each remaining block (7-dim)
        for block_id in range(self.num_blocks):
            if self.blocks_placed[block_id]:
                accessibility.append(0.0)  # Already placed
            else:
                # Calculate based on surrounding obstacles
                difficulty = self._calculate_grasp_difficulty(block_id)
                accessibility.append(difficulty)
        
        # Collision risk for each block (7-dim)
        for block_id in range(self.num_blocks):
            if self.blocks_placed[block_id]:
                accessibility.append(0.0)
            else:
                risk = self._calculate_collision_risk(block_id)
                accessibility.append(risk)
        
        # Placement feasibility for target positions (27-dim)
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    if self.assembly_grid[i, j, k] > 0:
                        accessibility.append(0.0)  # Occupied
                    else:
                        feasibility = self._calculate_placement_feasibility(i, j, k)
                        accessibility.append(feasibility)
        
        # Robot pose optimality (6-dim - joint distances from optimal)
        try:
            if self.virtual_mode:
                # Simulate current joint positions
                current_joints = [0, -10, 85, 5, 95, -5]  # Simulated pose near home
            else:
                current_joints = get_current_posj()
            optimal_joints = self._calculate_optimal_pose()
            joint_diffs = [abs(c - o) for c, o in zip(current_joints, optimal_joints)]
            accessibility.extend(joint_diffs[:6])  # Ensure exactly 6 dimensions
        except:
            accessibility.extend([0.0] * 6)  # Default if can't get current pose
        
        # Pad or truncate to exactly 64 dimensions
        while len(accessibility) < 64:
            accessibility.append(0.0)
        accessibility = accessibility[:64]
        
        return accessibility
    
    def _calculate_grasp_difficulty(self, block_id: int) -> float:
        """Calculate difficulty of grasping a specific block"""
        block_pos = self.block_positions[block_id]
        block_spec = self.block_specs[block_id]
        
        # Base difficulty from block characteristics
        base_difficulty = block_spec['rotation_constraint']
        
        # Increase difficulty based on proximity to other blocks
        proximity_penalty = 0.0
        for other_id in range(self.num_blocks):
            if other_id != block_id and not self.blocks_placed[other_id]:
                distance = np.linalg.norm(block_pos - self.block_positions[other_id])
                if distance < 150:  # 15cm threshold
                    proximity_penalty += (150 - distance) / 150
        
        return min(1.0, base_difficulty + proximity_penalty * 0.3)
    
    def _calculate_collision_risk(self, block_id: int) -> float:
        """Calculate collision risk when moving a block"""
        block_pos = self.block_positions[block_id]
        target_pos = self.target_positions[block_id]
        
        # Check trajectory for obstacles
        trajectory_risk = 0.0
        for other_id in range(self.num_blocks):
            if other_id != block_id and not self.blocks_placed[other_id]:
                other_pos = self.block_positions[other_id]
                # Simplified linear trajectory check
                traj_points = [block_pos + t * (target_pos - block_pos) for t in np.linspace(0, 1, 10)]
                for point in traj_points:
                    distance = np.linalg.norm(point - other_pos)
                    if distance < 100:  # 10cm safety margin
                        trajectory_risk += (100 - distance) / 100
        
        return min(1.0, trajectory_risk / 10)  # Normalize
    
    def _calculate_placement_feasibility(self, i: int, j: int, k: int) -> float:
        """Calculate feasibility of placing block at grid position"""
        # Check if position needs support from below
        if k > 0 and self.assembly_grid[i, j, k-1] == 0:
            return 0.0  # No support below
        
        # Check accessibility - can robot reach this position?
        world_pos = self._grid_to_world_coords(i, j, k)
        distance_from_robot = np.linalg.norm(world_pos[:2])  # Distance in XY plane
        
        if distance_from_robot > 900:  # Beyond robot reach
            return 0.0
        elif distance_from_robot > 700:  # Difficult to reach
            return 0.3
        else:
            return 1.0
    
    def _grid_to_world_coords(self, i: int, j: int, k: int) -> np.ndarray:
        """Convert grid coordinates to world coordinates"""
        base_x, base_y, base_z = 500, 0, 50
        block_spacing = 80
        
        x = base_x + (i - 1) * block_spacing
        y = base_y + (j - 1) * block_spacing
        z = base_z + k * 40
        
        return np.array([x, y, z])
    
    def _calculate_optimal_pose(self) -> List[float]:
        """Calculate optimal robot pose for current task"""
        # Simplified - return a reasonable working pose
        return [0, -20, 90, 0, 110, 0]  # Slightly forward-leaning pose
    
    def _calculate_strategic_state(self) -> List[float]:
        """Calculate strategic state information"""
        strategic = []
        
        # Block interdependencies (simplified 7x7 matrix compressed to 25-dim)
        dependencies = self._get_block_dependencies()
        strategic.extend(dependencies[:25])  # Take first 25 elements
        
        # Efficiency indicators (7-dim)
        remaining_blocks = ~self.blocks_placed
        remaining_count = np.sum(remaining_blocks)
        
        # Estimated remaining time
        avg_time_per_block = 300 if self.current_step == 0 else (time.time() - self.start_time) / max(1, self.current_step)
        estimated_remaining_time = avg_time_per_block * remaining_count
        strategic.append(min(1.0, estimated_remaining_time / 1800))  # Normalize to 30 minutes
        
        # Motion complexity score
        motion_complexity = self._calculate_motion_complexity()
        strategic.append(motion_complexity)
        
        # Assembly stability score
        stability_score = self._calculate_assembly_stability()
        strategic.append(stability_score)
        
        # Sequence optimality score
        sequence_score = self._calculate_sequence_optimality()
        strategic.append(sequence_score)
        
        # Workspace utilization
        workspace_util = self._calculate_workspace_utilization()
        strategic.append(workspace_util)
        
        # Future accessibility preservation
        future_access = self._calculate_future_accessibility()
        strategic.append(future_access)
        
        # Current episode progress ratio
        progress_ratio = self.current_step / self.max_steps
        strategic.append(progress_ratio)
        
        # Ensure exactly 32 dimensions
        while len(strategic) < 32:
            strategic.append(0.0)
        strategic = strategic[:32]
        
        return strategic
    
    def _get_block_dependencies(self) -> List[float]:
        """Get simplified block placement dependencies"""
        # Simplified dependency matrix (flattened)
        dependencies = []
        for i in range(self.num_blocks):
            for j in range(self.num_blocks):
                if i == j:
                    dependencies.append(0.0)
                else:
                    # Simplified dependency score
                    spec_i = self.block_specs[i]
                    spec_j = self.block_specs[j]
                    
                    # Heavy blocks should go before light blocks
                    weight_dep = (spec_i['weight'] - spec_j['weight']) / 3.0
                    
                    # Stable blocks should go first
                    stability_dep = spec_i['stability_factor'] - spec_j['stability_factor']
                    
                    dep_score = (weight_dep + stability_dep) / 2.0
                    dependencies.append(max(-1.0, min(1.0, dep_score)))
        
        return dependencies
    
    def _calculate_motion_complexity(self) -> float:
        """Calculate complexity of remaining motions"""
        remaining_blocks = np.where(~self.blocks_placed)[0]
        if len(remaining_blocks) == 0:
            return 0.0
        
        total_complexity = 0.0
        for block_id in remaining_blocks:
            # Distance to target
            distance = np.linalg.norm(
                self.target_positions[block_id] - self.block_positions[block_id]
            )
            
            # Block complexity
            block_complexity = self.block_specs[block_id]['rotation_constraint']
            
            total_complexity += (distance / 1000.0) * block_complexity
        
        return min(1.0, total_complexity / max(1, len(remaining_blocks)))
    
    def _calculate_assembly_stability(self) -> float:
        """Calculate current assembly stability"""
        placed_blocks = np.where(self.blocks_placed)[0]
        if len(placed_blocks) == 0:
            return 1.0
        
        total_stability = 0.0
        for block_id in placed_blocks:
            block_stability = self.block_specs[block_id]['stability_factor']
            
            # Check if block has support
            grid_pos = self._world_to_grid_coords(self.block_positions[block_id])
            if self._is_valid_grid_pos(grid_pos) and grid_pos[2] > 0:
                # Check support from below
                support_pos = (grid_pos[0], grid_pos[1], grid_pos[2] - 1)
                if self.assembly_grid[support_pos] > 0:
                    block_stability *= 1.2  # Bonus for having support
                else:
                    block_stability *= 0.5  # Penalty for no support
            
            total_stability += block_stability
        
        return min(1.0, total_stability / max(1, len(placed_blocks)))
    
    def _calculate_sequence_optimality(self) -> float:
        """Calculate optimality of current placement sequence"""
        if self.current_step == 0:
            return 1.0
        
        placed_blocks = np.where(self.blocks_placed)[0]
        if len(placed_blocks) == 0:
            return 1.0  # No blocks placed yet
            
        optimality_score = 0.0
        
        for i, block_id in enumerate(placed_blocks):
            block_spec = self.block_specs[block_id]
            
            # Earlier placement of stable, heavy blocks is better
            expected_order = i / len(placed_blocks)
            actual_suitability = (block_spec['stability_factor'] + 
                                 (block_spec['weight'] / 3.0)) / 2.0
            
            order_score = 1.0 - abs(expected_order - actual_suitability)
            optimality_score += order_score
        
        return optimality_score / max(1, len(placed_blocks))
    
    def _calculate_workspace_utilization(self) -> float:
        """Calculate how efficiently workspace is being used"""
        placed_blocks = np.where(self.blocks_placed)[0]
        if len(placed_blocks) == 0:
            return 0.0
        
        # Calculate bounding box of placed blocks
        placed_positions = self.block_positions[placed_blocks]
        min_coords = np.min(placed_positions, axis=0)
        max_coords = np.max(placed_positions, axis=0)
        
        used_volume = np.prod(max_coords - min_coords)
        max_workspace_volume = np.prod([400, 800, 500])  # workspace dimensions
        
        return min(1.0, used_volume / max_workspace_volume)
    
    def _calculate_future_accessibility(self) -> float:
        """Calculate preservation of future accessibility"""
        remaining_blocks = np.where(~self.blocks_placed)[0]
        if len(remaining_blocks) == 0:
            return 1.0
        
        total_accessibility = 0.0
        for block_id in remaining_blocks:
            accessibility = 1.0 - self._calculate_grasp_difficulty(block_id)
            total_accessibility += accessibility
        
        return total_accessibility / max(1, len(remaining_blocks))
    
    def _get_info(self) -> Dict:
        """Get additional information about current state"""
        return {
            'current_step': self.current_step,
            'blocks_placed': np.sum(self.blocks_placed),
            'elapsed_time': time.time() - self.start_time if self.start_time else 0,
            'remaining_blocks': np.sum(~self.blocks_placed),
            'assembly_progress': np.sum(self.blocks_placed) / self.num_blocks
        }
    
    def render(self, mode='human'):
        """Render the environment (placeholder for visualization)"""
        if mode == 'human':
            print(f"Step: {self.current_step}/{self.max_steps}")
            print(f"Blocks placed: {np.sum(self.blocks_placed)}/{self.num_blocks}")
            print(f"Elapsed time: {time.time() - self.start_time if self.start_time else 0:.1f}s")
            print(f"Assembly grid:\n{self.assembly_grid}")
        elif mode == 'rgb_array':
            # Return RGB array representation (placeholder)
            return np.zeros((480, 640, 3), dtype=np.uint8)
    
    def close(self):
        """Clean up resources"""
        try:
            if hasattr(self, 'node'):
                self.node.destroy_node()
        except:
            pass
        
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    # Test environment
    env = M0609BlockAssemblyEnv(virtual_mode=True)
    
    print("Environment created successfully!")
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    
    # Test reset
    obs, info = env.reset()
    print(f"Initial observation shape: {obs.shape}")
    print(f"Initial info: {info}")
    
    # Test step with random action
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    print(f"Step result - Reward: {reward}, Terminated: {terminated}, Truncated: {truncated}")
    
    env.close()
    print("Environment test completed!")
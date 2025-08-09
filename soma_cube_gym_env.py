#!/usr/bin/env python3
"""
OpenAI Gym-compatible SOMA Cube Assembly Environment
Based on the paper "High-Speed Autonomous Robotic Assembly Using In-Hand Manipulation and Re-Grasping"
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
import time
from typing import Dict, List, Tuple, Optional, Any
from enum import Enum
import os

class SOMAPiece(Enum):
    """SOMA Cube pieces (7 pieces total)"""
    L = "L"           # L-shaped piece
    T = "T"           # T-shaped piece  
    Z = "Z"           # Z-shaped piece
    P = "P"           # P-shaped piece
    V = "V"           # V-shaped piece
    Y = "Y"           # Y-shaped piece
    B = "B"           # Bent piece

class GraspDirection(Enum):
    """Grasp directions based on paper's vertical pickup constraint"""
    X_AXIS_VERTICAL = 0    # X-axis grasp with vertical pickup
    Y_AXIS_VERTICAL = 1    # Y-axis grasp with vertical pickup
    Z_AXIS_VERTICAL = 2    # Z-axis grasp with vertical pickup

class SOMACubeEnv(gym.Env):
    """
    OpenAI Gym environment for SOMA Cube assembly with PyBullet physics
    Based on the paper's assembly strategy and re-grasping approach
    """
    
    metadata = {'render_modes': ['human', 'rgb_array']}
    
    def __init__(self, config: Dict[str, Any] = None):
        super(SOMACubeEnv, self).__init__()
        
        # Environment configuration
        self.config = config or {}
        self.render_mode = self.config.get('render_mode', 'human')
        self.max_episode_steps = self.config.get('max_steps', 500)
        self.enable_re_grasp = self.config.get('enable_re_grasp', True)
        
        # SOMA cube target is 3x3x3
        self.cube_size = 3
        self.target_volume = self.cube_size ** 3  # 27 unit cubes
        
        # Physics simulation setup
        self.physics_client = None
        self.robot_id = None
        self.gripper_id = None
        self.piece_ids = {}
        self.table_id = None
        
        # Environment state
        self.current_step = 0
        self.placed_pieces = []
        self.remaining_pieces = list(SOMAPiece)
        self.assembly_complete = False
        self.last_grasp_failed = False
        
        # 3D occupancy matrix for the target cube (3x3x3)
        self.occupancy_matrix = np.zeros((3, 3, 3), dtype=np.int8)
        
        # Define action space: [piece_selection, grasp_direction, target_x, target_y, target_z, orientation]
        self.action_space = spaces.Box(
            low=np.array([0, 0, 0, 0, 0, 0]),           # piece_idx, grasp_dir, x, y, z, orientation
            high=np.array([6, 2, 2, 2, 2, 3.14159]),    # 7 pieces, 3 grasp dirs, 3x3x3 positions, rotation
            dtype=np.float32
        )
        
        # Define observation space
        self.observation_space = spaces.Dict({
            # 3D positions and orientations of all pieces (7 pieces x 7 values each)
            'pieces_state': spaces.Box(low=-10.0, high=10.0, shape=(7, 7), dtype=np.float32),
            
            # 3D occupancy matrix for target cube
            'occupancy_matrix': spaces.Box(low=0, high=1, shape=(3, 3, 3), dtype=np.int8),
            
            # Gripper state [x, y, z, is_grasping, current_piece_id]
            'gripper_state': spaces.Box(low=-10.0, high=10.0, shape=(5,), dtype=np.float32),
            
            # Remaining pieces mask (7 pieces)
            'remaining_pieces': spaces.Box(low=0, high=1, shape=(7,), dtype=np.int8),
            
            # Assembly progress [pieces_placed, total_pieces, completion_ratio]
            'assembly_progress': spaces.Box(low=0.0, high=1.0, shape=(3,), dtype=np.float32),
            
            # Re-grasp state [re_grasp_available, last_grasp_failed]
            're_grasp_state': spaces.Box(low=0, high=1, shape=(2,), dtype=np.int8)
        })
    
    def reset(self, seed=None, options=None) -> Tuple[Dict, Dict]:
        """Reset the environment to initial state"""
        super().reset(seed=seed)
        
        # Initialize physics simulation
        self._init_physics()
        
        # Reset environment state
        self.current_step = 0
        self.placed_pieces = []
        self.remaining_pieces = list(SOMAPiece)
        self.assembly_complete = False
        self.last_grasp_failed = False
        self.occupancy_matrix = np.zeros((3, 3, 3), dtype=np.int8)
        
        # Spawn SOMA pieces randomly on table
        self._spawn_soma_pieces()
        
        # Get initial observation
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, info
    
    def step(self, action: np.ndarray) -> Tuple[Dict, float, bool, bool, Dict]:
        """Execute one step in the environment"""
        self.current_step += 1
        
        # Parse action
        piece_idx = int(np.clip(action[0], 0, 6))
        grasp_dir = int(np.clip(action[1], 0, 2))
        target_pos = np.clip(action[2:5], [0, 0, 0], [2, 2, 2]).astype(int)
        orientation = action[5]
        
        # Get selected piece
        selected_piece = list(SOMAPiece)[piece_idx]
        
        # Execute assembly action
        reward, success, info = self._execute_assembly_action(
            selected_piece, grasp_dir, target_pos, orientation
        )
        
        # Check termination conditions
        terminated = self.assembly_complete or self.current_step >= self.max_episode_steps
        truncated = False
        
        # Get observation
        observation = self._get_observation()
        
        return observation, reward, terminated, truncated, info
    
    def _init_physics(self):
        """Initialize PyBullet physics simulation"""
        try:
            if self.physics_client is not None:
                p.disconnect(self.physics_client)
        except:
            pass  # Ignore disconnect errors
        
        # Connect to physics server
        try:
            if self.render_mode == 'human':
                self.physics_client = p.connect(p.GUI)
            else:
                self.physics_client = p.connect(p.DIRECT)
        except Exception as e:
            print(f"Warning: Could not connect to PyBullet GUI, using DIRECT mode: {e}")
            self.physics_client = p.connect(p.DIRECT)
        
        # Configure simulation
        try:
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            p.setTimeStep(1/240)
            
            # Load environment
            self._load_environment()
        except Exception as e:
            print(f"Error initializing physics: {e}")
            raise
    
    def _load_environment(self):
        """Load the robotic assembly environment with built-in shapes"""
        # Load ground plane
        plane_id = p.loadURDF("plane.urdf")
        
        # Create table using basic shapes
        table_pos = [0, 0, 0.6]
        table_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.6, 0.6, 0.05])
        table_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.6, 0.6, 0.05], 
                                         rgbaColor=[0.8, 0.6, 0.4, 1])
        self.table_id = p.createMultiBody(baseMass=0, 
                                        baseCollisionShapeIndex=table_collision,
                                        baseVisualShapeIndex=table_visual,
                                        basePosition=table_pos)
        
        # Create simplified robot base (as a cylinder)
        robot_pos = [0, -0.8, 0.6]
        robot_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.1, height=0.3)
        robot_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.1, length=0.3,
                                         rgbaColor=[0.2, 0.2, 0.8, 1])
        self.robot_id = p.createMultiBody(baseMass=0,
                                        baseCollisionShapeIndex=robot_collision,
                                        baseVisualShapeIndex=robot_visual,
                                        basePosition=robot_pos)
        
        # Initialize empty robot joints list (simplified robot)
        self.robot_joints = []
        
        # Create 2-DOF gripper (simplified as box)
        gripper_pos = [0, -0.3, 1.2]
        gripper_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.05])
        gripper_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.05], 
                                           rgbaColor=[0.5, 0.5, 0.5, 1])
        self.gripper_id = p.createMultiBody(baseMass=0.1, 
                                          baseCollisionShapeIndex=gripper_collision,
                                          baseVisualShapeIndex=gripper_visual,
                                          basePosition=gripper_pos)
    
    def _spawn_soma_pieces(self):
        """Spawn SOMA cube pieces on the table"""
        self.piece_ids = {}
        
        # SOMA piece configurations (simplified as colored boxes)
        piece_configs = {
            SOMAPiece.L: {'size': [0.03, 0.03, 0.06], 'color': [1, 0, 0, 1]},     # Red L-piece
            SOMAPiece.T: {'size': [0.03, 0.06, 0.03], 'color': [0, 1, 0, 1]},     # Green T-piece
            SOMAPiece.Z: {'size': [0.03, 0.04, 0.03], 'color': [0, 0, 1, 1]},     # Blue Z-piece
            SOMAPiece.P: {'size': [0.03, 0.03, 0.04], 'color': [1, 1, 0, 1]},     # Yellow P-piece
            SOMAPiece.V: {'size': [0.03, 0.04, 0.04], 'color': [1, 0, 1, 1]},     # Magenta V-piece
            SOMAPiece.Y: {'size': [0.04, 0.03, 0.04], 'color': [0, 1, 1, 1]},     # Cyan Y-piece
            SOMAPiece.B: {'size': [0.04, 0.04, 0.03], 'color': [0.5, 0.5, 0.5, 1]} # Gray B-piece
        }
        
        # Spawn pieces randomly on table
        for i, (piece, config) in enumerate(piece_configs.items()):
            # Random position on table surface
            x = np.random.uniform(-0.3, 0.3)
            y = np.random.uniform(-0.3, 0.3)
            z = 0.65 + config['size'][2]  # Above table surface
            
            # Create collision and visual shapes
            collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=config['size'])
            visual_shape = p.createVisualShape(p.GEOM_BOX, halfExtents=config['size'], 
                                             rgbaColor=config['color'])
            
            # Create piece
            piece_id = p.createMultiBody(baseMass=0.1,
                                       baseCollisionShapeIndex=collision_shape,
                                       baseVisualShapeIndex=visual_shape,
                                       basePosition=[x, y, z])
            
            self.piece_ids[piece] = piece_id
    
    def _execute_assembly_action(self, piece: SOMAPiece, grasp_dir: int, target_pos: np.ndarray, 
                               orientation: float) -> Tuple[float, bool, Dict]:
        """Execute assembly action with re-grasping capability"""
        reward = 0.0
        success = False
        info = {}
        
        # Check if piece is available
        if piece not in self.remaining_pieces:
            return -5.0, False, {'error': 'piece_already_used'}
        
        # Check if target position is valid
        if self.occupancy_matrix[target_pos[0], target_pos[1], target_pos[2]] == 1:
            return -5.0, False, {'error': 'position_occupied'}
        
        # Simulate grasp attempt based on paper's vertical pickup constraint
        grasp_success = self._attempt_grasp(piece, grasp_dir)
        
        if not grasp_success and self.enable_re_grasp:
            # Attempt re-grasping as described in paper
            info['re_grasp_attempted'] = True
            grasp_success = self._attempt_re_grasp(piece, grasp_dir)
        
        if grasp_success:
            # Execute placement
            placement_success = self._place_piece(piece, target_pos, orientation)
            
            if placement_success:
                # Successful placement
                reward += 10.0  # Correct placement reward
                success = True
                
                # Update state
                self.remaining_pieces.remove(piece)
                self.placed_pieces.append(piece)
                self.occupancy_matrix[target_pos[0], target_pos[1], target_pos[2]] = 1
                
                # Check assembly completion
                if len(self.placed_pieces) == 7:
                    self.assembly_complete = True
                    reward += 100.0  # Assembly completion bonus
                    info['assembly_complete'] = True
                
                info['placement_success'] = True
                
            else:
                # Placement failed (collision or invalid position)
                reward -= 5.0
                info['placement_failed'] = True
        else:
            # Grasp failed
            reward -= 2.0
            self.last_grasp_failed = True
            info['grasp_failed'] = True
        
        # Step penalty to encourage efficiency
        reward -= 0.1
        
        return reward, success, info
    
    def _attempt_grasp(self, piece: SOMAPiece, grasp_dir: int) -> bool:
        """Attempt to grasp piece following paper's vertical pickup constraint"""
        
        # Get piece position
        piece_id = self.piece_ids[piece]
        piece_pos, piece_orn = p.getBasePositionAndOrientation(piece_id)
        
        # Calculate grasp success probability based on piece orientation and grasp direction
        grasp_directions = list(GraspDirection)
        selected_grasp_dir = grasp_directions[grasp_dir]
        
        # Simulate grasp based on paper's findings (95% success rate overall)
        base_success_prob = 0.85
        
        # Vertical pickup constraint bonus (as emphasized in paper)
        if selected_grasp_dir in [GraspDirection.X_AXIS_VERTICAL, GraspDirection.Y_AXIS_VERTICAL]:
            base_success_prob += 0.1
        
        # Add some randomness
        success = np.random.random() < base_success_prob
        
        if success:
            # Move piece to gripper for visualization
            gripper_pos, _ = p.getBasePositionAndOrientation(self.gripper_id)
            p.resetBasePositionAndOrientation(piece_id, 
                                            [gripper_pos[0], gripper_pos[1], gripper_pos[2] + 0.05], 
                                            piece_orn)
        
        return success
    
    def _attempt_re_grasp(self, piece: SOMAPiece, grasp_dir: int) -> bool:
        """Attempt re-grasping when direct grasp fails (paper reports 25% re-grasp rate)"""
        # Re-grasp has higher success probability as mentioned in paper
        re_grasp_success_prob = 0.75
        
        success = np.random.random() < re_grasp_success_prob
        
        if success:
            # Move piece to gripper
            piece_id = self.piece_ids[piece]
            piece_pos, piece_orn = p.getBasePositionAndOrientation(piece_id)
            gripper_pos, _ = p.getBasePositionAndOrientation(self.gripper_id)
            p.resetBasePositionAndOrientation(piece_id, 
                                            [gripper_pos[0], gripper_pos[1], gripper_pos[2] + 0.05], 
                                            piece_orn)
        
        return success
    
    def _place_piece(self, piece: SOMAPiece, target_pos: np.ndarray, orientation: float) -> bool:
        """Place piece at target position"""
        
        # Calculate world coordinates for target position
        base_x, base_y, base_z = 0.0, 0.0, 0.65  # Table surface
        cube_unit_size = 0.03  # 3cm per unit cube
        
        world_x = base_x + target_pos[0] * cube_unit_size
        world_y = base_y + target_pos[1] * cube_unit_size
        world_z = base_z + target_pos[2] * cube_unit_size + cube_unit_size/2  # Half-height offset
        
        # Move piece to target position
        piece_id = self.piece_ids[piece]
        target_orientation = p.getQuaternionFromEuler([0, 0, orientation])
        
        p.resetBasePositionAndOrientation(piece_id, 
                                        [world_x, world_y, world_z], 
                                        target_orientation)
        
        # Step physics to check for collisions
        for _ in range(10):
            p.stepSimulation()
        
        # Check if placement is stable (simplified collision detection)
        final_pos, _ = p.getBasePositionAndOrientation(piece_id)
        
        # Consider placement successful if piece is close to target position
        distance = np.linalg.norm([final_pos[0] - world_x, final_pos[1] - world_y, final_pos[2] - world_z])
        
        return distance < 0.02  # 2cm tolerance
    
    def _get_observation(self) -> Dict:
        """Get current environment observation"""
        
        # Pieces state (position + orientation for each piece)
        pieces_state = np.zeros((7, 7))
        for i, piece in enumerate(SOMAPiece):
            if piece in self.piece_ids:
                piece_id = self.piece_ids[piece]
                pos, orn = p.getBasePositionAndOrientation(piece_id)
                euler = p.getEulerFromQuaternion(orn)
                pieces_state[i] = [pos[0], pos[1], pos[2], euler[0], euler[1], euler[2], 
                                 1 if piece in self.remaining_pieces else 0]
        
        # Gripper state
        gripper_pos, _ = p.getBasePositionAndOrientation(self.gripper_id)
        gripper_state = np.array([gripper_pos[0], gripper_pos[1], gripper_pos[2], 
                                0.0, -1.0])  # is_grasping=0, current_piece_id=-1
        
        # Remaining pieces mask
        remaining_mask = np.zeros(7)
        for i, piece in enumerate(SOMAPiece):
            remaining_mask[i] = 1 if piece in self.remaining_pieces else 0
        
        # Assembly progress
        pieces_placed = len(self.placed_pieces)
        total_pieces = 7
        completion_ratio = pieces_placed / total_pieces
        assembly_progress = np.array([pieces_placed, total_pieces, completion_ratio])
        
        # Re-grasp state
        re_grasp_state = np.array([1 if self.enable_re_grasp else 0, 
                                 1 if self.last_grasp_failed else 0])
        
        return {
            'pieces_state': pieces_state.astype(np.float32),
            'occupancy_matrix': self.occupancy_matrix,
            'gripper_state': gripper_state.astype(np.float32),
            'remaining_pieces': remaining_mask.astype(np.int8),
            'assembly_progress': assembly_progress.astype(np.float32),
            're_grasp_state': re_grasp_state.astype(np.int8)
        }
    
    def _get_info(self) -> Dict:
        """Get additional info about environment state"""
        return {
            'pieces_placed': len(self.placed_pieces),
            'pieces_remaining': len(self.remaining_pieces),
            'assembly_complete': self.assembly_complete,
            'current_step': self.current_step,
            'max_steps': self.max_episode_steps,
            'last_grasp_failed': self.last_grasp_failed
        }
    
    def render(self, mode='human'):
        """Render the environment"""
        if mode == 'rgb_array':
            # Get camera image
            width, height = 640, 480
            view_matrix = p.computeViewMatrixFromYawPitchRoll([0, 0, 0], 2, -30, 0, 0, 1)
            proj_matrix = p.computeProjectionMatrixFOV(60, width/height, 0.1, 10)
            
            _, _, rgb_img, _, _ = p.getCameraImage(width, height, view_matrix, proj_matrix)
            return np.array(rgb_img)[:, :, :3]  # Remove alpha channel
        
        # Human rendering is handled by PyBullet GUI
        time.sleep(1/60)  # 60 FPS
    
    def close(self):
        """Close the environment"""
        if self.physics_client is not None:
            p.disconnect(self.physics_client)
            self.physics_client = None

# Test the environment
if __name__ == "__main__":
    env = SOMACubeEnv({'render_mode': 'human', 'max_steps': 100})
    
    obs, info = env.reset()
    print("Environment initialized successfully!")
    print(f"Observation space: {env.observation_space}")
    print(f"Action space: {env.action_space}")
    
    # Run a few test steps
    for i in range(10):
        action = env.action_space.sample()
        obs, reward, terminated, truncated, info = env.step(action)
        
        print(f"Step {i}: Reward={reward:.2f}, Terminated={terminated}")
        
        if terminated or truncated:
            break
        
        env.render()
    
    env.close()
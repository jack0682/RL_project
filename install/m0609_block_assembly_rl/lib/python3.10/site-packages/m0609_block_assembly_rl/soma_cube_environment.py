"""
SOMA Cube Assembly Environment for M0609 Doosan Robot
Properly implements the 7-piece SOMA cube puzzle assembly into a 3x3x3 cube structure.

The SOMA cube consists of 7 unique pieces made from unit cubes:
- V: 4 unit cubes in V shape  
- L: 4 unit cubes in L shape
- T: 4 unit cubes in T shape  
- Z: 4 unit cubes in Z shape
- A: 4 unit cubes in A shape
- B: 4 unit cubes in B shape
- P: 3 unit cubes in P shape

Total: 27 unit cubes forming a 3x3x3 cube
"""

import numpy as np
import gymnasium as gym
from gymnasium import spaces
from typing import Dict, List, Tuple, Optional, Any
from enum import Enum
from dataclasses import dataclass
import time
import json

# Try to import Doosan robot modules
try:
    import DR_init
    from DSR_ROBOT2 import *
    from DR_common2 import *
    DOOSAN_AVAILABLE = True
except ImportError:
    print("Warning: Doosan robot modules not available, running in virtual-only mode")
    DOOSAN_AVAILABLE = False

class SOMAPiece(Enum):
    """
    SOMA Cube pieces with their unit cube compositions
    Each piece is defined by its unit cube positions relative to origin
    """
    V = 0  # V-piece: 4 unit cubes [(0,0,0), (1,0,0), (0,1,0), (0,0,1)]
    L = 1  # L-piece: 4 unit cubes [(0,0,0), (1,0,0), (2,0,0), (0,1,0)]
    T = 2  # T-piece: 4 unit cubes [(0,0,0), (1,0,0), (2,0,0), (1,1,0)]
    Z = 3  # Z-piece: 4 unit cubes [(0,0,0), (1,0,0), (1,1,0), (2,1,0)]
    A = 4  # A-piece: 4 unit cubes [(0,0,0), (1,0,0), (1,1,0), (1,1,1)]
    B = 5  # B-piece: 4 unit cubes [(0,0,0), (0,1,0), (1,1,0), (1,1,1)]
    P = 6  # P-piece: 3 unit cubes [(0,0,0), (1,0,0), (0,1,0)]

@dataclass
class SOMAPieceGeometry:
    """Geometric definition of SOMA pieces in terms of unit cubes"""
    
    # Each piece defined as list of (x,y,z) unit cube positions
    PIECE_DEFINITIONS = {
        SOMAPiece.V: [(0,0,0), (1,0,0), (0,1,0), (0,0,1)],  # 4 cubes - V shape
        SOMAPiece.L: [(0,0,0), (1,0,0), (2,0,0), (0,1,0)],  # 4 cubes - L shape
        SOMAPiece.T: [(0,0,0), (1,0,0), (2,0,0), (1,1,0)],  # 4 cubes - T shape
        SOMAPiece.Z: [(0,0,0), (1,0,0), (1,1,0), (2,1,0)],  # 4 cubes - Z shape
        SOMAPiece.A: [(0,0,0), (1,0,0), (1,1,0), (1,1,1)],  # 4 cubes - A shape
        SOMAPiece.B: [(0,0,0), (0,1,0), (1,1,0), (1,1,1)],  # 4 cubes - B shape  
        SOMAPiece.P: [(0,0,0), (1,0,0), (0,1,0)]             # 3 cubes - P shape
    }
    
    # Colors for visualization
    PIECE_COLORS = {
        SOMAPiece.V: [1.0, 0.0, 0.0],  # Red
        SOMAPiece.L: [0.0, 1.0, 0.0],  # Green
        SOMAPiece.T: [0.0, 0.0, 1.0],  # Blue
        SOMAPiece.Z: [1.0, 1.0, 0.0],  # Yellow
        SOMAPiece.A: [1.0, 0.0, 1.0],  # Magenta
        SOMAPiece.B: [0.0, 1.0, 1.0],  # Cyan
        SOMAPiece.P: [0.5, 0.5, 0.5]   # Gray
    }
    
    @classmethod
    def get_piece_unit_count(cls, piece: SOMAPiece) -> int:
        """Get number of unit cubes in a piece"""
        return len(cls.PIECE_DEFINITIONS[piece])
    
    @classmethod
    def get_total_unit_cubes(cls) -> int:
        """Verify total unit cubes = 27 (3x3x3)"""
        total = sum(cls.get_piece_unit_count(piece) for piece in SOMAPiece)
        assert total == 27, f"SOMA pieces must total 27 unit cubes, got {total}"
        return total
    
    @classmethod
    def get_piece_orientations(cls, piece: SOMAPiece) -> List[np.ndarray]:
        """
        Generate all valid orientations (24 possible) for a piece
        Returns list of 3x3 rotation matrices
        """
        base_shape = np.array(cls.PIECE_DEFINITIONS[piece])
        orientations = []
        
        # Generate all 24 possible orientations (90-degree rotations)
        for rx in range(4):  # Rotations around x-axis
            for ry in range(4):  # Rotations around y-axis  
                for rz in range(4):  # Rotations around z-axis
                    # Create rotation matrix
                    rot_x = cls._rotation_matrix_x(rx * np.pi/2)
                    rot_y = cls._rotation_matrix_y(ry * np.pi/2)
                    rot_z = cls._rotation_matrix_z(rz * np.pi/2)
                    
                    rotation_matrix = rot_z @ rot_y @ rot_x
                    
                    # Apply rotation and normalize
                    rotated = np.round((rotation_matrix @ base_shape.T).T).astype(int)
                    rotated = rotated - rotated.min(axis=0)  # Normalize to origin
                    
                    # Check if this is a unique orientation
                    is_unique = True
                    for existing in orientations:
                        if np.array_equal(np.sort(rotated.view(np.void), axis=0),
                                        np.sort(existing.view(np.void), axis=0)):
                            is_unique = False
                            break
                    
                    if is_unique:
                        orientations.append(rotated)
        
        return orientations
    
    @staticmethod
    def _rotation_matrix_x(angle: float) -> np.ndarray:
        """Rotation matrix around X-axis"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    
    @staticmethod
    def _rotation_matrix_y(angle: float) -> np.ndarray:
        """Rotation matrix around Y-axis"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    
    @staticmethod
    def _rotation_matrix_z(angle: float) -> np.ndarray:
        """Rotation matrix around Z-axis"""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

class SOMACubeState:
    """
    Represents the current state of SOMA cube assembly
    Tracks which pieces are placed and their positions in the 3x3x3 cube
    """
    
    def __init__(self):
        # 3x3x3 cube state (0 = empty, 1-7 = piece ID)
        self.cube_state = np.zeros((3, 3, 3), dtype=int)
        
        # Track placed pieces
        self.placed_pieces = {}  # {piece: (position, orientation)}
        self.remaining_pieces = list(SOMAPiece)
        
        # Assembly progress
        self.assembly_complete = False
        self.total_pieces = len(SOMAPiece)
        self.placed_count = 0
    
    def place_piece(self, piece: SOMAPiece, position: Tuple[int, int, int], 
                   orientation: np.ndarray) -> bool:
        """
        Attempt to place a piece at given position with given orientation
        Returns True if placement is valid and successful
        """
        if piece not in self.remaining_pieces:
            return False
        
        # Get piece geometry with orientation
        piece_cubes = orientation + np.array(position)
        
        # Check bounds and collisions
        for cube_pos in piece_cubes:
            x, y, z = cube_pos
            if (x < 0 or x >= 3 or y < 0 or y >= 3 or z < 0 or z >= 3):
                return False  # Out of bounds
            if self.cube_state[x, y, z] != 0:
                return False  # Position occupied
        
        # Place the piece
        piece_id = piece.value + 1  # 1-indexed for cube state
        for cube_pos in piece_cubes:
            x, y, z = cube_pos
            self.cube_state[x, y, z] = piece_id
        
        # Update tracking
        self.placed_pieces[piece] = (position, orientation)
        self.remaining_pieces.remove(piece)
        self.placed_count += 1
        
        # Check completion
        if self.placed_count == self.total_pieces:
            self.assembly_complete = self._verify_complete_cube()
        
        return True
    
    def remove_piece(self, piece: SOMAPiece) -> bool:
        """Remove a placed piece from the cube"""
        if piece not in self.placed_pieces:
            return False
        
        position, orientation = self.placed_pieces[piece]
        piece_cubes = orientation + np.array(position)
        
        # Clear the piece from cube state
        for cube_pos in piece_cubes:
            x, y, z = cube_pos
            self.cube_state[x, y, z] = 0
        
        # Update tracking
        del self.placed_pieces[piece]
        self.remaining_pieces.append(piece)
        self.placed_count -= 1
        self.assembly_complete = False
        
        return True
    
    def _verify_complete_cube(self) -> bool:
        """Verify that the assembled structure forms a complete 3x3x3 cube"""
        # Check that all 27 positions are filled
        return np.all(self.cube_state != 0)
    
    def get_valid_placements(self, piece: SOMAPiece) -> List[Tuple[Tuple[int, int, int], np.ndarray]]:
        """
        Get all valid placements for a piece in current state
        Returns list of (position, orientation) tuples
        """
        if piece not in self.remaining_pieces:
            return []
        
        valid_placements = []
        orientations = SOMAPieceGeometry.get_piece_orientations(piece)
        
        for orientation in orientations:
            for x in range(3):
                for y in range(3):
                    for z in range(3):
                        position = (x, y, z)
                        
                        # Check if this placement is valid
                        piece_cubes = orientation + np.array(position)
                        
                        valid = True
                        for cube_pos in piece_cubes:
                            cx, cy, cz = cube_pos
                            if (cx < 0 or cx >= 3 or cy < 0 or cy >= 3 or 
                                cz < 0 or cz >= 3):
                                valid = False
                                break
                            if self.cube_state[cx, cy, cz] != 0:
                                valid = False
                                break
                        
                        if valid:
                            valid_placements.append((position, orientation))
        
        return valid_placements

class SOMACubeAssemblyEnv(gym.Env):
    """
    Reinforcement Learning Environment for SOMA Cube Assembly
    
    The goal is to assemble all 7 SOMA pieces into a complete 3x3x3 cube
    using the M0609 Doosan robot.
    """
    
    metadata = {'render_modes': ['human', 'rgb_array']}
    
    def __init__(self, 
                 robot_id: str = "dsr01",
                 robot_model: str = "m0609", 
                 virtual_mode: bool = True,
                 unit_cube_size: float = 30.0,  # mm
                 render_mode: Optional[str] = None):
        """
        Initialize SOMA Cube Assembly Environment
        
        Args:
            robot_id: Doosan robot identifier
            robot_model: Robot model (m0609, etc.)
            virtual_mode: Whether to use virtual robot simulation
            unit_cube_size: Size of each unit cube in mm
            render_mode: Rendering mode
        """
        super().__init__()
        
        self.robot_id = robot_id
        self.robot_model = robot_model
        self.virtual_mode = virtual_mode
        self.unit_cube_size = unit_cube_size
        self.render_mode = render_mode
        
        # Initialize robot connection
        self._init_robot_connection()
        
        # SOMA cube state
        self.soma_state = SOMACubeState()
        self.step_count = 0
        self.max_steps = 50  # Maximum steps per episode
        
        # Verify SOMA cube integrity
        total_cubes = SOMAPieceGeometry.get_total_unit_cubes()
        print(f"✓ SOMA cube verified: {total_cubes} unit cubes from 7 pieces")
        
        # Define action space
        # [piece_id, position_x, position_y, position_z, orientation_id]
        max_orientations = max(len(SOMAPieceGeometry.get_piece_orientations(piece)) 
                             for piece in SOMAPiece)
        
        self.action_space = spaces.Box(
            low=np.array([0, 0, 0, 0, 0]),
            high=np.array([6, 2, 2, 2, max_orientations-1]),
            dtype=np.int32
        )
        
        # Define observation space
        self.observation_space = spaces.Dict({
            # Current 3x3x3 cube state
            'cube_state': spaces.Box(low=0, high=7, shape=(3, 3, 3), dtype=np.int32),
            
            # Remaining pieces (binary mask)
            'remaining_pieces': spaces.Box(low=0, high=1, shape=(7,), dtype=np.int32),
            
            # Assembly progress
            'assembly_progress': spaces.Box(low=0, high=7, shape=(1,), dtype=np.int32),
            
            # Valid actions count (for action masking)
            'valid_actions_count': spaces.Box(low=0, high=1000, shape=(1,), dtype=np.int32),
            
            # Step count
            'step_count': spaces.Box(low=0, high=self.max_steps, shape=(1,), dtype=np.int32)
        })
        
        # Robot workspace for physical assembly
        self.assembly_center = np.array([500.0, 0.0, 200.0])  # mm from robot base
        self.piece_pickup_area = np.array([300.0, -300.0, 100.0])  # mm from robot base
    
    def reset(self, seed: Optional[int] = None, 
              options: Optional[Dict[str, Any]] = None) -> Tuple[Dict, Dict]:
        """
        Reset environment to initial state
        
        Returns:
            observation: Initial observation
            info: Additional information
        """
        super().reset(seed=seed)
        
        # Reset SOMA cube state
        self.soma_state = SOMACubeState()
        self.step_count = 0
        
        # Reset robot to home position
        if not self.virtual_mode and DOOSAN_AVAILABLE:
            self._robot_to_home_position()
        
        # Scatter pieces in pickup area (virtual or move real pieces)
        self._initialize_piece_positions()
        
        observation = self._get_observation()
        info = self._get_info()
        
        return observation, info
    
    def step(self, action: np.ndarray) -> Tuple[Dict, float, bool, bool, Dict]:
        """
        Execute one step in the environment
        
        Args:
            action: [piece_id, pos_x, pos_y, pos_z, orientation_id]
            
        Returns:
            observation: New observation
            reward: Reward for this step
            terminated: Whether episode is finished
            truncated: Whether episode was truncated
            info: Additional information
        """
        self.step_count += 1
        
        # Parse action
        piece_id = int(action[0])
        position = (int(action[1]), int(action[2]), int(action[3]))
        orientation_id = int(action[4])
        
        # Validate action
        if piece_id < 0 or piece_id >= len(SOMAPiece):
            return self._get_observation(), -10.0, False, False, {'error': 'invalid_piece'}
        
        piece = list(SOMAPiece)[piece_id]
        
        # Check if piece is available
        if piece not in self.soma_state.remaining_pieces:
            return self._get_observation(), -5.0, False, False, {'error': 'piece_unavailable'}
        
        # Get piece orientations and validate orientation_id
        orientations = SOMAPieceGeometry.get_piece_orientations(piece)
        if orientation_id >= len(orientations):
            return self._get_observation(), -5.0, False, False, {'error': 'invalid_orientation'}
        
        orientation = orientations[orientation_id]
        
        # Execute physical robot action
        success = self._execute_robot_placement(piece, position, orientation)
        
        if not success:
            reward = -5.0
            info = {'error': 'robot_execution_failed'}
        else:
            # Attempt to place piece in SOMA state
            placement_success = self.soma_state.place_piece(piece, position, orientation)
            
            if placement_success:
                # Calculate reward
                reward = self._calculate_reward(piece, position, orientation)
                info = {'placement': 'success', 'piece': piece.name}
            else:
                reward = -10.0
                info = {'error': 'invalid_placement'}
        
        # Check termination conditions
        terminated = self.soma_state.assembly_complete
        truncated = self.step_count >= self.max_steps
        
        observation = self._get_observation()
        info.update(self._get_info())
        
        return observation, reward, terminated, truncated, info
    
    def _calculate_reward(self, piece: SOMAPiece, position: Tuple[int, int, int], 
                         orientation: np.ndarray) -> float:
        """
        Calculate reward for successful piece placement
        
        Reward components:
        - Base success: +10
        - Assembly progress bonus: +5 per piece placed
        - Completion bonus: +100 for complete cube
        - Efficiency bonus: +2 for each step saved
        """
        reward = 10.0  # Base success reward
        
        # Progress bonus
        reward += 5.0 * self.soma_state.placed_count
        
        # Completion bonus
        if self.soma_state.assembly_complete:
            reward += 100.0
            # Efficiency bonus for completing faster
            steps_saved = self.max_steps - self.step_count
            reward += 2.0 * steps_saved
        
        # Strategic placement bonus (pieces that enable more placements)
        remaining_valid_actions = self._count_total_valid_actions()
        if remaining_valid_actions > 0:
            reward += 1.0  # Keeping options open
        
        return reward
    
    def _execute_robot_placement(self, piece: SOMAPiece, position: Tuple[int, int, int], 
                                orientation: np.ndarray) -> bool:
        """
        Execute physical robot movement to place piece
        
        Returns True if execution successful
        """
        if self.virtual_mode:
            # Virtual mode - simulate with delay
            time.sleep(0.1)
            return np.random.random() > 0.05  # 95% success rate in virtual
        
        if not DOOSAN_AVAILABLE:
            return False
        
        try:
            # Calculate world coordinates
            world_pos = self._grid_to_world_position(position)
            
            # Phase 1: Move to piece pickup position
            pickup_pos = self._get_piece_pickup_position(piece)
            self._move_robot_to_position(pickup_pos, approach_height=50.0)
            
            # Phase 2: Pick up piece
            self._actuate_gripper(close=True)
            time.sleep(0.5)
            
            # Phase 3: Move to assembly position
            self._move_robot_to_position(world_pos, approach_height=50.0)
            
            # Phase 4: Place piece
            self._move_robot_to_position(world_pos)
            self._actuate_gripper(close=False)
            time.sleep(0.2)
            
            # Phase 5: Retract
            retract_pos = world_pos.copy()
            retract_pos[2] += 50.0  # Move up 50mm
            self._move_robot_to_position(retract_pos)
            
            return True
            
        except Exception as e:
            print(f"Robot execution error: {e}")
            return False
    
    def _move_robot_to_position(self, position: np.ndarray, approach_height: float = 0.0):
        """Move robot to specified position"""
        if approach_height > 0:
            # First move to approach height
            approach_pos = position.copy()
            approach_pos[2] += approach_height
            target_pose = posx(approach_pos.tolist() + [0, 180, 0])  # Standard orientation
            movel(target_pose, vel=50, acc=50)
        
        # Move to final position
        target_pose = posx(position.tolist() + [0, 180, 0])
        movel(target_pose, vel=20, acc=20)
    
    def _actuate_gripper(self, close: bool):
        """Actuate gripper - close or open"""
        if close:
            set_digital_output(1, True)   # Close gripper
            set_digital_output(2, False)
        else:
            set_digital_output(1, False)  # Open gripper  
            set_digital_output(2, True)
        
        time.sleep(0.5)  # Wait for gripper action
    
    def _get_piece_pickup_position(self, piece: SOMAPiece) -> np.ndarray:
        """Get pickup position for a specific piece"""
        # Pieces arranged in pickup area
        piece_id = piece.value
        x_offset = (piece_id % 3) * 60.0  # 60mm spacing
        y_offset = (piece_id // 3) * 60.0
        
        return self.piece_pickup_area + np.array([x_offset, y_offset, 0])
    
    def _grid_to_world_position(self, grid_pos: Tuple[int, int, int]) -> np.ndarray:
        """Convert 3x3x3 grid position to world coordinates"""
        x, y, z = grid_pos
        
        world_x = self.assembly_center[0] + (x - 1) * self.unit_cube_size
        world_y = self.assembly_center[1] + (y - 1) * self.unit_cube_size  
        world_z = self.assembly_center[2] + z * self.unit_cube_size
        
        return np.array([world_x, world_y, world_z])
    
    def _count_total_valid_actions(self) -> int:
        """Count total valid actions across all remaining pieces"""
        total = 0
        for piece in self.soma_state.remaining_pieces:
            valid_placements = self.soma_state.get_valid_placements(piece)
            total += len(valid_placements)
        return total
    
    def _get_observation(self) -> Dict:
        """Get current observation"""
        # Remaining pieces mask
        remaining_mask = np.zeros(7, dtype=np.int32)
        for piece in self.soma_state.remaining_pieces:
            remaining_mask[piece.value] = 1
        
        return {
            'cube_state': self.soma_state.cube_state.copy(),
            'remaining_pieces': remaining_mask,
            'assembly_progress': np.array([self.soma_state.placed_count], dtype=np.int32),
            'valid_actions_count': np.array([self._count_total_valid_actions()], dtype=np.int32),
            'step_count': np.array([self.step_count], dtype=np.int32)
        }
    
    def _get_info(self) -> Dict:
        """Get additional information"""
        return {
            'pieces_placed': self.soma_state.placed_count,
            'pieces_remaining': len(self.soma_state.remaining_pieces),
            'assembly_complete': self.soma_state.assembly_complete,
            'step_count': self.step_count,
            'max_steps': self.max_steps,
            'cube_filled_ratio': np.sum(self.soma_state.cube_state > 0) / 27.0
        }
    
    def _init_robot_connection(self):
        """Initialize robot connection"""
        if not self.virtual_mode and DOOSAN_AVAILABLE:
            try:
                # Initialize Doosan robot connection
                DR_init.set_robot_id(self.robot_id)
                DR_init.set_robot_model(self.robot_model)
                
                # Set tool and TCP
                set_tool("Tool_Weight_2.5KG")
                set_tcp("TCP_GRIPPER")
                
                print(f"✓ Connected to {self.robot_model} robot: {self.robot_id}")
            except Exception as e:
                print(f"Robot connection failed: {e}")
                print("Switching to virtual mode")
                self.virtual_mode = True
    
    def _robot_to_home_position(self):
        """Move robot to home position"""
        if DOOSAN_AVAILABLE:
            home_joints = [0, 0, 90, 0, 90, 0]  # Home joint configuration
            movej(home_joints, vel=60, acc=60)
    
    def _initialize_piece_positions(self):
        """Initialize piece positions in pickup area"""
        if self.virtual_mode:
            # Virtual mode - just record positions
            print("✓ Virtual pieces positioned in pickup area")
        else:
            # Real mode - would need to physically arrange pieces
            # This would typically be done manually or with a separate routine
            print("✓ Ensure pieces are arranged in pickup area")
    
    def render(self, mode: str = 'human'):
        """Render the current state"""
        if mode == 'human':
            self._render_text()
        elif mode == 'rgb_array':
            return self._render_image()
    
    def _render_text(self):
        """Text-based rendering of cube state"""
        print(f"\nSOMA Cube Assembly - Step {self.step_count}")
        print(f"Pieces placed: {self.soma_state.placed_count}/7")
        print(f"Assembly complete: {self.soma_state.assembly_complete}")
        
        print("\nCube state (layer by layer, Z=0 to Z=2):")
        for z in range(3):
            print(f"Layer Z={z}:")
            for y in range(2, -1, -1):  # Top to bottom
                row = ""
                for x in range(3):
                    value = self.soma_state.cube_state[x, y, z]
                    if value == 0:
                        row += "  . "
                    else:
                        row += f"  {value} "
                print(f"  {row}")
            print()
        
        remaining_pieces = [piece.name for piece in self.soma_state.remaining_pieces]
        print(f"Remaining pieces: {remaining_pieces}")
    
    def _render_image(self) -> np.ndarray:
        """Image-based rendering (placeholder)"""
        # Would implement 3D visualization here
        return np.zeros((400, 400, 3), dtype=np.uint8)
    
    def close(self):
        """Clean up environment"""
        if not self.virtual_mode and DOOSAN_AVAILABLE:
            # Return robot to safe position
            self._robot_to_home_position()
            print("✓ Robot returned to home position")

def verify_soma_cube_structure():
    """Verify the SOMA cube structure is correct"""
    print("=== SOMA Cube Structure Verification ===")
    
    # Check piece definitions
    total_cubes = 0
    for piece in SOMAPiece:
        cubes = SOMAPieceGeometry.get_piece_unit_count(piece)
        total_cubes += cubes
        print(f"{piece.name}: {cubes} unit cubes")
        
        # Show piece structure
        definition = SOMAPieceGeometry.PIECE_DEFINITIONS[piece]
        print(f"  Structure: {definition}")
    
    print(f"\nTotal unit cubes: {total_cubes}")
    print(f"Target (3x3x3): 27")
    print(f"✓ Structure valid: {total_cubes == 27}")
    
    # Test environment creation
    env = SOMACubeAssemblyEnv(virtual_mode=True)
    obs, info = env.reset()
    print(f"✓ Environment created successfully")
    print(f"  Observation space: {env.observation_space}")
    print(f"  Action space: {env.action_space}")
    
    return total_cubes == 27

if __name__ == "__main__":
    # Verify SOMA cube structure
    is_valid = verify_soma_cube_structure()
    
    if is_valid:
        print("\n🎉 SOMA Cube environment ready for RL training!")
        print("\nUsage:")
        print("from soma_cube_environment import SOMACubeAssemblyEnv")
        print("env = SOMACubeAssemblyEnv(virtual_mode=True)")
        print("obs, info = env.reset()")
    else:
        print("\n❌ SOMA Cube structure verification failed!")
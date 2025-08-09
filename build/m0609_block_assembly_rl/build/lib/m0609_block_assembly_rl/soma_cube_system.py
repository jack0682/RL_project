"""
SOMA Cube System Core Components
Provides the missing core classes referenced throughout the hierarchical RL system

This file was identified as missing during comprehensive package review and is 
required for the hierarchical training system to function properly.
"""

import numpy as np
from typing import Dict, List, Tuple, Optional, Any, Set
from dataclasses import dataclass
from enum import Enum
import copy
import json

# Import from our existing SOMA environment
try:
    from .soma_cube_environment import SOMAPiece, SOMAPieceGeometry
except ImportError:
    try:
        from soma_cube_environment import SOMAPiece, SOMAPieceGeometry
    except ImportError:
        # Define locally if import fails
        from enum import Enum
        
        class SOMAPiece(Enum):
            V = 0
            L = 1
            T = 2
            Z = 3
            A = 4
            B = 5
            P = 6
        
        class SOMAPieceGeometry:
            PIECE_DEFINITIONS = {
                SOMAPiece.V: [(0,0,0), (1,0,0), (0,1,0), (0,0,1)],
                SOMAPiece.L: [(0,0,0), (1,0,0), (2,0,0), (0,1,0)],
                SOMAPiece.T: [(0,0,0), (1,0,0), (2,0,0), (1,1,0)],
                SOMAPiece.Z: [(0,0,0), (1,0,0), (1,1,0), (2,1,0)],
                SOMAPiece.A: [(0,0,0), (1,0,0), (1,1,0), (1,1,1)],
                SOMAPiece.B: [(0,0,0), (0,1,0), (1,1,0), (1,1,1)],
                SOMAPiece.P: [(0,0,0), (1,0,0), (0,1,0)]
            }
            
            PIECE_COLORS = {
                SOMAPiece.V: [1.0, 0.0, 0.0],
                SOMAPiece.L: [0.0, 1.0, 0.0],
                SOMAPiece.T: [0.0, 0.0, 1.0],
                SOMAPiece.Z: [1.0, 1.0, 0.0],
                SOMAPiece.A: [1.0, 0.0, 1.0],
                SOMAPiece.B: [0.0, 1.0, 1.0],
                SOMAPiece.P: [0.5, 0.5, 0.5]
            }

@dataclass
class AssemblyState:
    """
    State representation for SOMA cube assembly
    Used by upper-level planner for sequence planning
    """
    grid: np.ndarray  # 3x3x3 grid showing piece placements
    placed_pieces: List[SOMAPiece]  # Pieces already placed
    remaining_pieces: List[SOMAPiece]  # Pieces still to place
    current_sequence: List['UpperLevelAction']  # Actions taken so far
    
    def __post_init__(self):
        """Initialize grid if not provided"""
        if self.grid is None:
            self.grid = np.zeros((3, 3, 3), dtype=int)
    
    def copy(self) -> 'AssemblyState':
        """Create deep copy of assembly state"""
        return AssemblyState(
            grid=self.grid.copy(),
            placed_pieces=self.placed_pieces.copy(),
            remaining_pieces=self.remaining_pieces.copy(),
            current_sequence=[action.copy() for action in self.current_sequence]
        )
    
    def is_complete(self) -> bool:
        """Check if assembly is complete"""
        return len(self.remaining_pieces) == 0
    
    def get_occupancy_count(self) -> int:
        """Get number of occupied grid cells"""
        return np.count_nonzero(self.grid)

@dataclass
class UpperLevelAction:
    """
    Upper-level action for piece placement
    Specifies which piece to place where and how
    """
    piece: SOMAPiece
    rotation_index: int  # Index into possible rotations (0-23)
    target_position: Tuple[int, int, int]  # Grid position (x,y,z)
    target_pose: Tuple[float, float, float, float, float, float]  # 6DOF robot pose
    
    def copy(self) -> 'UpperLevelAction':
        """Create copy of action"""
        return UpperLevelAction(
            piece=self.piece,
            rotation_index=self.rotation_index,
            target_position=self.target_position,
            target_pose=self.target_pose
        )
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization"""
        return {
            'piece': self.piece.value,
            'rotation_index': self.rotation_index,
            'target_position': self.target_position,
            'target_pose': self.target_pose
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'UpperLevelAction':
        """Create from dictionary"""
        return cls(
            piece=SOMAPiece(data['piece']),
            rotation_index=data['rotation_index'],
            target_position=tuple(data['target_position']),
            target_pose=tuple(data['target_pose'])
        )

@dataclass
class LowerLevelGoal:
    """
    Lower-level goal specification for manipulation controller
    Defines what the robot should achieve physically
    """
    target_pose: List[float]  # 6DOF pose [x,y,z,rx,ry,rz]
    tolerance: float  # Tolerance for goal achievement (mm/degrees)
    block_id: SOMAPiece  # Which piece to manipulate
    task_type: str  # 'grasp', 'place', 'approach', 'retract'
    force_threshold: float = 50.0  # Maximum allowed force (N)
    
    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary"""
        return {
            'target_pose': self.target_pose,
            'tolerance': self.tolerance,
            'block_id': self.block_id.value,
            'task_type': self.task_type,
            'force_threshold': self.force_threshold
        }
    
    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'LowerLevelGoal':
        """Create from dictionary"""
        return cls(
            target_pose=data['target_pose'],
            tolerance=data['tolerance'],
            block_id=SOMAPiece(data['block_id']),
            task_type=data['task_type'],
            force_threshold=data.get('force_threshold', 50.0)
        )

class SOMAPieceManager:
    """
    Manager for SOMA piece operations and transformations
    Handles piece rotations, collision detection, and placement validation
    """
    
    def __init__(self):
        self.piece_geometries = SOMAPieceGeometry.PIECE_DEFINITIONS
        self.piece_colors = SOMAPieceGeometry.PIECE_COLORS
        
        # Pre-compute rotations for efficiency
        self._rotation_cache = {}
        self._generate_rotation_cache()
    
    def _generate_rotation_cache(self):
        """Pre-compute all 24 rotations for each piece"""
        for piece in SOMAPiece:
            self._rotation_cache[piece] = self._compute_piece_rotations(piece)
    
    def _compute_piece_rotations(self, piece: SOMAPiece) -> List[np.ndarray]:
        """
        Compute all 24 possible rotations for a piece
        Returns list of rotated piece geometries
        """
        base_geometry = np.array(self.piece_geometries[piece])
        rotations = []
        
        # Generate 24 distinct rotations (avoiding duplicates)
        rotation_matrices = self._get_24_rotation_matrices()
        
        for rot_matrix in rotation_matrices:
            # Apply rotation to each unit cube position
            rotated_geometry = []
            for cube_pos in base_geometry:
                rotated_pos = rot_matrix @ np.array(cube_pos)
                rotated_pos = np.round(rotated_pos).astype(int)
                rotated_geometry.append(tuple(rotated_pos))
            
            # Normalize position (move to origin)
            rotated_geometry = self._normalize_piece_position(rotated_geometry)
            
            # Check if this rotation is unique
            if not self._is_duplicate_rotation(rotations, rotated_geometry):
                rotations.append(np.array(rotated_geometry))
        
        return rotations
    
    def _get_24_rotation_matrices(self) -> List[np.ndarray]:
        """Generate 24 distinct 3x3 rotation matrices (90-degree rotations)"""
        matrices = []
        
        # Rotation matrices for 90-degree rotations around x, y, z axes
        rot_x_90 = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        rot_y_90 = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
        rot_z_90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        
        identity = np.eye(3)
        
        # Generate all combinations of 0,1,2,3 rotations around each axis
        for i in range(4):
            for j in range(4):
                for k in range(4):
                    # Compose rotations
                    matrix = identity
                    for _ in range(i):
                        matrix = matrix @ rot_x_90
                    for _ in range(j):  
                        matrix = matrix @ rot_y_90
                    for _ in range(k):
                        matrix = matrix @ rot_z_90
                    
                    # Check uniqueness
                    is_unique = True
                    for existing in matrices:
                        if np.allclose(matrix, existing):
                            is_unique = False
                            break
                    
                    if is_unique:
                        matrices.append(matrix)
                        
                    if len(matrices) >= 24:
                        break
                if len(matrices) >= 24:
                    break
            if len(matrices) >= 24:
                break
        
        return matrices[:24]
    
    def _normalize_piece_position(self, geometry: List[Tuple[int, int, int]]) -> List[Tuple[int, int, int]]:
        """Move piece so minimum coordinates are at origin"""
        if not geometry:
            return geometry
        
        positions = np.array(geometry)
        min_coords = np.min(positions, axis=0)
        normalized = positions - min_coords
        
        return [tuple(pos) for pos in normalized]
    
    def _is_duplicate_rotation(self, existing_rotations: List[np.ndarray], 
                             new_rotation: List[Tuple[int, int, int]]) -> bool:
        """Check if rotation already exists"""
        new_set = set(new_rotation)
        
        for existing in existing_rotations:
            existing_set = set(tuple(pos) for pos in existing)
            if new_set == existing_set:
                return True
        
        return False
    
    def get_piece_rotations(self, piece: SOMAPiece) -> List[np.ndarray]:
        """Get all rotations for a piece"""
        return self._rotation_cache[piece]
    
    def check_placement_validity(self, state: AssemblyState, piece: SOMAPiece, 
                               rotation_index: int, position: Tuple[int, int, int]) -> bool:
        """
        Check if piece placement is valid (no collisions, within bounds)
        
        Args:
            state: Current assembly state
            piece: Piece to place
            rotation_index: Which rotation to use
            position: Grid position to place piece
            
        Returns:
            True if placement is valid
        """
        try:
            rotations = self.get_piece_rotations(piece)
            if rotation_index >= len(rotations):
                return False
            
            piece_geometry = rotations[rotation_index]
            px, py, pz = position
            
            # Check each unit cube of the piece
            for cube_offset in piece_geometry:
                # Calculate absolute position
                abs_x = px + cube_offset[0]
                abs_y = py + cube_offset[1]  
                abs_z = pz + cube_offset[2]
                
                # Check bounds
                if (abs_x < 0 or abs_x >= 3 or 
                    abs_y < 0 or abs_y >= 3 or 
                    abs_z < 0 or abs_z >= 3):
                    return False
                
                # Check collision
                if state.grid[abs_x, abs_y, abs_z] != 0:
                    return False
            
            return True
            
        except (IndexError, ValueError):
            return False
    
    def apply_placement(self, state: AssemblyState, piece: SOMAPiece,
                       rotation_index: int, position: Tuple[int, int, int]) -> AssemblyState:
        """
        Apply piece placement to state, returning new state
        
        Args:
            state: Current state
            piece: Piece to place
            rotation_index: Rotation to use
            position: Position to place
            
        Returns:
            New state with piece placed
        """
        # Create new state
        new_state = state.copy()
        
        # Validate placement
        if not self.check_placement_validity(state, piece, rotation_index, position):
            raise ValueError(f"Invalid placement: {piece} at {position} rotation {rotation_index}")
        
        # Get piece geometry
        rotations = self.get_piece_rotations(piece)
        piece_geometry = rotations[rotation_index]
        
        # Place piece in grid
        px, py, pz = position
        piece_id = piece.value + 1  # Use 1-based indexing (0 = empty)
        
        for cube_offset in piece_geometry:
            abs_x = px + cube_offset[0]
            abs_y = py + cube_offset[1]
            abs_z = pz + cube_offset[2]
            new_state.grid[abs_x, abs_y, abs_z] = piece_id
        
        # Update piece lists
        if piece in new_state.remaining_pieces:
            new_state.remaining_pieces.remove(piece)
        if piece not in new_state.placed_pieces:
            new_state.placed_pieces.append(piece)
        
        return new_state
    
    def get_assembly_reward(self, state: AssemblyState) -> float:
        """
        Calculate reward for assembly state
        
        Returns:
            Reward value (0-100)
        """
        if state.is_complete():
            return 100.0  # Complete assembly
        
        # Progress reward based on pieces placed
        progress = len(state.placed_pieces) / len(SOMAPiece)
        
        # Bonus for good structural integrity
        stability_bonus = self._calculate_stability_bonus(state)
        
        return progress * 50.0 + stability_bonus
    
    def _calculate_stability_bonus(self, state: AssemblyState) -> float:
        """Calculate bonus for structural stability"""
        if len(state.placed_pieces) < 2:
            return 0.0
        
        # Count connected components
        connected_count = self._count_connected_components(state.grid)
        
        # Prefer fewer connected components (more integrated structure)
        if connected_count == 1:
            return 20.0  # Single connected structure
        elif connected_count <= 3:
            return 10.0  # Mostly connected
        else:
            return 0.0   # Fragmented
    
    def _count_connected_components(self, grid: np.ndarray) -> int:
        """Count connected components in 3D grid using DFS"""
        visited = np.zeros_like(grid, dtype=bool)
        components = 0
        
        def dfs(x, y, z):
            if (x < 0 or x >= 3 or y < 0 or y >= 3 or z < 0 or z >= 3 or
                visited[x, y, z] or grid[x, y, z] == 0):
                return
            
            visited[x, y, z] = True
            
            # Check 6-connected neighbors
            for dx, dy, dz in [(1,0,0), (-1,0,0), (0,1,0), (0,-1,0), (0,0,1), (0,0,-1)]:
                dfs(x + dx, y + dy, z + dz)
        
        for x in range(3):
            for y in range(3):
                for z in range(3):
                    if grid[x, y, z] != 0 and not visited[x, y, z]:
                        dfs(x, y, z)
                        components += 1
        
        return components
    
    def visualize_state(self, state: AssemblyState) -> str:
        """Create text visualization of assembly state"""
        lines = []
        lines.append("SOMA Cube Assembly State:")
        lines.append(f"Placed: {[p.name for p in state.placed_pieces]}")
        lines.append(f"Remaining: {[p.name for p in state.remaining_pieces]}")
        lines.append(f"Occupancy: {state.get_occupancy_count()}/27")
        
        # Layer-by-layer visualization
        for z in range(3):
            lines.append(f"\nLayer {z}:")
            for y in range(3):
                row = ""
                for x in range(3):
                    val = state.grid[x, y, z]
                    if val == 0:
                        row += "."
                    else:
                        piece = SOMAPiece(val - 1)
                        row += piece.name[0]
                lines.append(f"  {row}")
        
        return "\n".join(lines)
    
    def save_state(self, state: AssemblyState, filepath: str):
        """Save assembly state to JSON file"""
        data = {
            'grid': state.grid.tolist(),
            'placed_pieces': [p.value for p in state.placed_pieces],
            'remaining_pieces': [p.value for p in state.remaining_pieces],
            'current_sequence': [action.to_dict() for action in state.current_sequence]
        }
        
        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)
    
    def load_state(self, filepath: str) -> AssemblyState:
        """Load assembly state from JSON file"""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        return AssemblyState(
            grid=np.array(data['grid']),
            placed_pieces=[SOMAPiece(p) for p in data['placed_pieces']],
            remaining_pieces=[SOMAPiece(p) for p in data['remaining_pieces']],
            current_sequence=[UpperLevelAction.from_dict(a) for a in data['current_sequence']]
        )

# Create convenient alias for backward compatibility
SOMACubeEnvironment = None  # Will be set to actual environment class when needed

def create_initial_assembly_state() -> AssemblyState:
    """Create initial assembly state with all pieces remaining"""
    return AssemblyState(
        grid=np.zeros((3, 3, 3), dtype=int),
        placed_pieces=[],
        remaining_pieces=list(SOMAPiece),
        current_sequence=[]
    )

def create_test_assembly_state() -> AssemblyState:
    """Create test state with some pieces placed for debugging"""
    state = create_initial_assembly_state()
    
    # Place P piece at origin for testing
    manager = SOMAPieceManager()
    try:
        state = manager.apply_placement(state, SOMAPiece.P, 0, (0, 0, 0))
    except ValueError:
        pass  # If placement fails, return initial state
    
    return state

# Module-level convenience functions
def verify_soma_cube_system():
    """Verify the SOMA cube system is working correctly"""
    print("Testing SOMA Cube System...")
    
    # Test piece manager
    manager = SOMAPieceManager()
    
    # Test rotations
    for piece in SOMAPiece:
        rotations = manager.get_piece_rotations(piece)
        expected_unit_cubes = len(SOMAPieceGeometry.PIECE_DEFINITIONS[piece])
        
        print(f"Piece {piece.name}: {len(rotations)} rotations, {expected_unit_cubes} unit cubes each")
        
        # Verify each rotation has correct number of unit cubes
        for i, rotation in enumerate(rotations):
            if len(rotation) != expected_unit_cubes:
                print(f"ERROR: Rotation {i} has {len(rotation)} cubes, expected {expected_unit_cubes}")
                return False
    
    # Test assembly state operations
    initial_state = create_initial_assembly_state()
    print(f"Initial state: {len(initial_state.remaining_pieces)} pieces remaining")
    
    # Test placement
    test_state = create_test_assembly_state()
    print(f"Test state: {len(test_state.placed_pieces)} pieces placed")
    
    print("✅ SOMA Cube System verification completed successfully")
    return True

if __name__ == "__main__":
    verify_soma_cube_system()
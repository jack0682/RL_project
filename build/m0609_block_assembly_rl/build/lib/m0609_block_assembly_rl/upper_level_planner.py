"""
Upper-Level Assembly Sequence Planner for SOMA Cube

Implements GFlowNet combined with Monte Carlo Tree Search (MCTS) for 
optimal assembly sequence planning with large combinatorial action spaces.

Key Features:
- GFlowNet for diverse sequence generation
- MCTS for strategic planning with sparse rewards  
- Handles 24 rotations × 7 pieces × 27 positions = ~4500 actions per step
- Efficient exploration of assembly sequences
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import math
import random
from typing import Dict, List, Tuple, Optional, Any, Set
from dataclasses import dataclass
from collections import defaultdict
import copy

from .soma_cube_system import (
    SOMAPiece, SOMAPieceManager, AssemblyState, UpperLevelAction,
    SOMACubeEnvironment
)

class GFlowNetNode:
    """Node in the GFlowNet graph representing partial assembly states"""
    
    def __init__(self, state: AssemblyState, parent=None, action=None):
        self.state = state
        self.parent = parent
        self.action = action  # Action that led to this state
        self.children = []
        
        # GFlowNet specific
        self.log_flow = 0.0  # Log flow through this node
        self.log_reward = 0.0  # Log reward of this node
        self.visit_count = 0
        
        # MCTS specific
        self.q_value = 0.0
        self.n_visits = 0
        
    def add_child(self, child_node):
        """Add a child node"""
        self.children.append(child_node)
        child_node.parent = self
        
    def is_terminal(self) -> bool:
        """Check if this is a terminal state"""
        return len(self.state.remaining_pieces) == 0
        
    def get_state_key(self) -> str:
        """Get unique key for this state"""
        grid_key = ''.join(map(str, self.state.grid.flatten()))
        pieces_key = ''.join(sorted([p.value for p in self.state.remaining_pieces]))
        return f"{grid_key}_{pieces_key}"

class MCTSNode:
    """MCTS node for strategic planning"""
    
    def __init__(self, state: AssemblyState, parent=None, action=None):
        self.state = state
        self.parent = parent
        self.action = action
        self.children = {}
        
        self.visit_count = 0
        self.value_sum = 0.0
        self.prior = 0.0  # Prior probability from neural network
        
    def is_expanded(self) -> bool:
        """Check if node has been expanded"""
        return len(self.children) > 0
        
    def select_child(self, c_puct: float = 1.0):
        """Select best child using UCB1"""
        best_score = -float('inf')
        best_child = None
        
        for action, child in self.children.items():
            # UCB1 score
            if child.visit_count == 0:
                ucb_score = float('inf')
            else:
                exploitation = child.value_sum / child.visit_count
                exploration = c_puct * child.prior * math.sqrt(self.visit_count) / (1 + child.visit_count)
                ucb_score = exploitation + exploration
                
            if ucb_score > best_score:
                best_score = ucb_score
                best_child = child
                
        return best_child
        
    def backup(self, value: float):
        """Backup value up the tree"""
        self.visit_count += 1
        self.value_sum += value
        if self.parent:
            self.parent.backup(value)

class GFlowNetwork(nn.Module):
    """
    GFlowNet for generating diverse assembly sequences
    
    Architecture optimized for discrete assembly planning with:
    - Forward policy P_F(s' | s) for next state prediction
    - Backward policy P_B(s | s') for trajectory probability
    - Flow estimation for probability mass assignment
    """
    
    def __init__(self, 
                 state_dim: int = 27,  # 3x3x3 grid
                 piece_dim: int = 14,  # 7 pieces × 2 (available/placed)
                 hidden_dim: int = 512,
                 n_layers: int = 4):
        
        super().__init__()
        
        self.state_dim = state_dim
        self.piece_dim = piece_dim
        self.input_dim = state_dim + piece_dim
        self.hidden_dim = hidden_dim
        
        # Shared state encoder
        self.encoder = nn.Sequential(
            nn.Linear(self.input_dim, hidden_dim),
            nn.ReLU(),
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.LayerNorm(hidden_dim)
        )
        
        # Forward policy network P_F(s' | s)
        self.forward_net = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, 1)  # Action logits (will be computed separately)
        )
        
        # Backward policy network P_B(s | s')
        self.backward_net = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(), 
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, 1)
        )
        
        # Flow network F(s)
        self.flow_net = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, 1)
        )
        
        # Action embedding for piece + rotation + position
        self.piece_embedding = nn.Embedding(len(SOMAPiece), 32)
        self.rotation_embedding = nn.Embedding(24, 16)  # Max 24 rotations
        self.position_embedding = nn.Embedding(27, 16)  # 3x3x3 positions
        
        # Action scorer
        self.action_scorer = nn.Sequential(
            nn.Linear(hidden_dim + 64, hidden_dim // 2),  # state + action embeddings
            nn.ReLU(),
            nn.Linear(hidden_dim // 2, 1)
        )
        
    def encode_state(self, state: AssemblyState) -> torch.Tensor:
        """Encode assembly state to vector representation"""
        # Grid encoding
        grid_flat = torch.FloatTensor(state.grid.flatten())
        
        # Piece encoding
        piece_encoding = []
        for piece in SOMAPiece:
            available = 1.0 if piece in state.remaining_pieces else 0.0
            placed = 1.0 if piece in state.placed_pieces else 0.0
            piece_encoding.extend([available, placed])
        piece_vec = torch.FloatTensor(piece_encoding)
        
        # Concatenate and encode
        state_vec = torch.cat([grid_flat, piece_vec]).unsqueeze(0)
        return self.encoder(state_vec)
        
    def encode_action(self, action: UpperLevelAction) -> torch.Tensor:
        """Encode action to vector representation"""
        piece_idx = list(SOMAPiece).index(action.piece)
        position_idx = action.target_position[0] * 9 + action.target_position[1] * 3 + action.target_position[2]
        
        piece_emb = self.piece_embedding(torch.LongTensor([piece_idx]))
        rotation_emb = self.rotation_embedding(torch.LongTensor([action.rotation_index]))
        position_emb = self.position_embedding(torch.LongTensor([position_idx]))
        
        return torch.cat([piece_emb, rotation_emb, position_emb], dim=1)
        
    def forward_policy(self, state: AssemblyState, valid_actions: List[UpperLevelAction]) -> torch.Tensor:
        """Forward policy P_F(s' | s) - probability of next actions"""
        state_encoded = self.encode_state(state)
        
        action_scores = []
        for action in valid_actions:
            action_encoded = self.encode_action(action)
            combined = torch.cat([state_encoded, action_encoded], dim=1)
            score = self.action_scorer(combined)
            action_scores.append(score)
            
        if len(action_scores) == 0:
            return torch.empty(0)
            
        scores = torch.cat(action_scores, dim=0)
        return F.softmax(scores, dim=0)
        
    def backward_policy(self, state: AssemblyState, parent_state: AssemblyState) -> torch.Tensor:
        """Backward policy P_B(s | s') - probability of previous state"""
        state_encoded = self.encode_state(state)
        return torch.sigmoid(self.backward_net(state_encoded))
        
    def flow_function(self, state: AssemblyState) -> torch.Tensor:
        """Flow function F(s) - total flow through state"""
        state_encoded = self.encode_state(state)
        return self.flow_net(state_encoded)

class UpperLevelPlanner:
    """
    Upper-level assembly sequence planner combining GFlowNet and MCTS
    
    Uses GFlowNet for diverse sequence generation and MCTS for strategic
    evaluation with sparse rewards from complete assemblies.
    """
    
    def __init__(self,
                 piece_manager: SOMAPieceManager,
                 environment: SOMACubeEnvironment,
                 device: str = 'cuda',
                 mcts_simulations: int = 100,
                 gflownet_lr: float = 1e-4):
        
        self.piece_manager = piece_manager
        self.environment = environment
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self.mcts_simulations = mcts_simulations
        
        # Initialize GFlowNet
        self.gflownet = GFlowNetwork().to(self.device)
        self.gflow_optimizer = torch.optim.Adam(self.gflownet.parameters(), lr=gflownet_lr)
        
        # Training statistics
        self.training_stats = {
            'trajectories_generated': 0,
            'successful_assemblies': 0,
            'average_sequence_length': 0.0,
            'gflow_loss_history': [],
            'mcts_value_history': []
        }
        
    def get_valid_actions(self, state: AssemblyState) -> List[UpperLevelAction]:
        """Get all valid actions from current state"""
        valid_actions = []
        
        for piece in state.remaining_pieces:
            rotations = self.piece_manager.get_piece_rotations(piece)
            
            for rotation_idx, rotation in enumerate(rotations):
                # Try all possible positions in 3x3x3 grid
                for x in range(3):
                    for y in range(3):
                        for z in range(3):
                            position = (x, y, z)
                            
                            # Check if placement is valid
                            if self.piece_manager.check_placement_validity(
                                state, piece, rotation_idx, position):
                                
                                # Convert grid position to robot pose (placeholder)
                                robot_pose = self._grid_to_robot_pose(position, rotation_idx)
                                
                                action = UpperLevelAction(
                                    piece=piece,
                                    rotation_index=rotation_idx,
                                    target_position=position,
                                    target_pose=robot_pose
                                )
                                valid_actions.append(action)
                                
        return valid_actions
        
    def _grid_to_robot_pose(self, grid_position: Tuple[int, int, int], 
                           rotation_idx: int) -> Tuple[float, float, float, float, float, float]:
        """Convert grid position to 6DOF robot pose"""
        # Convert 3D grid coordinates to robot workspace coordinates
        # Assuming each grid cell is 50mm × 50mm × 50mm
        x = 400 + grid_position[0] * 50  # X position in mm
        y = -100 + grid_position[1] * 50  # Y position in mm  
        z = 100 + grid_position[2] * 50   # Z position in mm
        
        # Convert rotation index to Euler angles (simplified)
        roll = (rotation_idx % 4) * 90    # 0, 90, 180, 270 degrees
        pitch = ((rotation_idx // 4) % 4) * 90
        yaw = ((rotation_idx // 16) % 4) * 90
        
        return (x, y, z, roll, pitch, yaw)
        
    def mcts_search(self, root_state: AssemblyState) -> UpperLevelAction:
        """Perform MCTS search to find best action"""
        root = MCTSNode(root_state)
        
        for _ in range(self.mcts_simulations):
            # Selection
            node = root
            path = [node]
            
            while node.is_expanded() and not self._is_terminal(node.state):
                node = node.select_child()
                path.append(node)
                
            # Expansion  
            if not self._is_terminal(node.state):
                valid_actions = self.get_valid_actions(node.state)
                if valid_actions:
                    # Get action priors from GFlowNet
                    with torch.no_grad():
                        action_probs = self.gflownet.forward_policy(node.state, valid_actions)
                    
                    for i, action in enumerate(valid_actions):
                        new_state = self.piece_manager.apply_placement(
                            node.state, action.piece, action.rotation_index, action.target_position)
                        child = MCTSNode(new_state, parent=node, action=action)
                        child.prior = action_probs[i].item() if len(action_probs) > i else 0.1
                        node.children[action] = child
                    
                    # Select random child for simulation
                    if node.children:
                        node = random.choice(list(node.children.values()))
                        path.append(node)
            
            # Simulation (rollout)
            value = self._simulate_rollout(node.state)
            
            # Backpropagation
            for path_node in path:
                path_node.backup(value)
                
        # Select best action
        if not root.children:
            # No valid actions
            return None
            
        best_action = max(root.children.keys(), 
                         key=lambda a: root.children[a].visit_count)
        return best_action
        
    def _simulate_rollout(self, state: AssemblyState) -> float:
        """Simulate random rollout from state to terminal"""
        current_state = copy.deepcopy(state)
        total_reward = 0.0
        max_steps = 20  # Prevent infinite loops
        
        for step in range(max_steps):
            if self._is_terminal(current_state):
                # Complete assembly
                if len(current_state.remaining_pieces) == 0:
                    total_reward += 1.0  # Sparse terminal reward
                break
                
            valid_actions = self.get_valid_actions(current_state)
            if not valid_actions:
                break
                
            # Random action selection for rollout
            action = random.choice(valid_actions)
            current_state = self.piece_manager.apply_placement(
                current_state, action.piece, action.rotation_index, action.target_position)
            
            # Small intermediate rewards
            total_reward += 0.1  # Reward for placing piece
            
        return total_reward
        
    def _is_terminal(self, state: AssemblyState) -> bool:
        """Check if state is terminal"""
        return len(state.remaining_pieces) == 0
        
    def generate_trajectory_gflownet(self, initial_state: AssemblyState) -> List[Tuple[AssemblyState, UpperLevelAction]]:
        """Generate trajectory using GFlowNet forward policy"""
        trajectory = []
        current_state = copy.deepcopy(initial_state)
        
        max_steps = len(current_state.remaining_pieces)
        
        for step in range(max_steps):
            if self._is_terminal(current_state):
                break
                
            valid_actions = self.get_valid_actions(current_state)
            if not valid_actions:
                break
                
            # Sample action from GFlowNet forward policy
            with torch.no_grad():
                action_probs = self.gflownet.forward_policy(current_state, valid_actions)
                
            if len(action_probs) == 0:
                break
                
            # Sample action
            action_idx = torch.multinomial(action_probs, 1).item()
            action = valid_actions[action_idx]
            
            trajectory.append((current_state, action))
            
            # Apply action
            current_state = self.piece_manager.apply_placement(
                current_state, action.piece, action.rotation_index, action.target_position)
                
        return trajectory
        
    def train_gflownet(self, trajectories: List[List[Tuple[AssemblyState, UpperLevelAction]]]):
        """Train GFlowNet on collected trajectories"""
        if not trajectories:
            return
            
        total_loss = 0.0
        
        for trajectory in trajectories:
            if len(trajectory) == 0:
                continue
                
            # Calculate trajectory reward
            final_state = trajectory[-1][0] if trajectory else None
            if final_state and self._is_terminal(final_state):
                trajectory_reward = 1.0  # Complete assembly
            else:
                trajectory_reward = len(trajectory) * 0.1  # Partial assembly
                
            # Flow matching loss
            loss = self._calculate_flow_matching_loss(trajectory, trajectory_reward)
            total_loss += loss
            
        if total_loss > 0:
            avg_loss = total_loss / len(trajectories)
            self.gflow_optimizer.zero_grad()
            avg_loss.backward()
            torch.nn.utils.clip_grad_norm_(self.gflownet.parameters(), max_norm=1.0)
            self.gflow_optimizer.step()
            
            self.training_stats['gflow_loss_history'].append(avg_loss.item())
            
    def _calculate_flow_matching_loss(self, trajectory: List[Tuple[AssemblyState, UpperLevelAction]], 
                                    reward: float) -> torch.Tensor:
        """Calculate flow matching loss for GFlowNet training"""
        if len(trajectory) == 0:
            return torch.tensor(0.0)
            
        total_loss = torch.tensor(0.0, device=self.device)
        
        # Forward and backward flow consistency
        for i, (state, action) in enumerate(trajectory):
            # Forward flow
            valid_actions = self.get_valid_actions(state)
            if len(valid_actions) == 0:
                continue
                
            forward_probs = self.gflownet.forward_policy(state, valid_actions)
            if len(forward_probs) == 0:
                continue
                
            # Find action index
            action_idx = -1
            for j, va in enumerate(valid_actions):
                if (va.piece == action.piece and 
                    va.rotation_index == action.rotation_index and
                    va.target_position == action.target_position):
                    action_idx = j
                    break
                    
            if action_idx == -1:
                continue
                
            # Flow consistency loss
            log_forward_prob = torch.log(forward_probs[action_idx] + 1e-8)
            flow_loss = -(log_forward_prob * reward)
            total_loss += flow_loss
            
        return total_loss / max(1, len(trajectory))
        
    def plan_assembly_sequence(self, initial_state: AssemblyState) -> List[UpperLevelAction]:
        """Plan complete assembly sequence using combined GFlowNet + MCTS"""
        sequence = []
        current_state = copy.deepcopy(initial_state)
        
        max_steps = len(current_state.remaining_pieces)
        
        for step in range(max_steps):
            if self._is_terminal(current_state):
                break
                
            # Use MCTS to find best action
            best_action = self.mcts_search(current_state)
            if best_action is None:
                break
                
            sequence.append(best_action)
            
            # Apply action to state
            current_state = self.piece_manager.apply_placement(
                current_state, best_action.piece, best_action.rotation_index, 
                best_action.target_position)
                
        return sequence
        
    def train_episode(self, num_trajectories: int = 10) -> Dict[str, float]:
        """Train for one episode by generating trajectories and updating GFlowNet"""
        
        # Generate trajectories using current GFlowNet policy
        trajectories = []
        successful_assemblies = 0
        
        for _ in range(num_trajectories):
            initial_state = AssemblyState(
                grid=np.zeros((3, 3, 3), dtype=int),
                placed_pieces=[],
                remaining_pieces=list(SOMAPiece)[:self.environment.curriculum_level],
                current_sequence=[]
            )
            
            trajectory = self.generate_trajectory_gflownet(initial_state)
            trajectories.append(trajectory)
            
            # Check if assembly was successful
            if trajectory:
                final_state_action = trajectory[-1]
                final_state = self.piece_manager.apply_placement(
                    final_state_action[0], 
                    final_state_action[1].piece,
                    final_state_action[1].rotation_index,
                    final_state_action[1].target_position
                )
                if self._is_terminal(final_state):
                    successful_assemblies += 1
                    
        # Train GFlowNet on collected trajectories
        self.train_gflownet(trajectories)
        
        # Update statistics
        self.training_stats['trajectories_generated'] += len(trajectories)
        self.training_stats['successful_assemblies'] += successful_assemblies
        
        if trajectories:
            avg_length = np.mean([len(traj) for traj in trajectories])
            self.training_stats['average_sequence_length'] = avg_length
        
        return {
            'success_rate': successful_assemblies / max(1, num_trajectories),
            'avg_sequence_length': self.training_stats['average_sequence_length'],
            'total_trajectories': self.training_stats['trajectories_generated'],
            'gflow_loss': self.training_stats['gflow_loss_history'][-1] if self.training_stats['gflow_loss_history'] else 0.0
        }
        
    def save_model(self, filepath: str):
        """Save trained models"""
        torch.save({
            'gflownet_state_dict': self.gflownet.state_dict(),
            'optimizer_state_dict': self.gflow_optimizer.state_dict(),
            'training_stats': self.training_stats
        }, filepath)
        
    def load_model(self, filepath: str):
        """Load trained models"""
        checkpoint = torch.load(filepath, map_location=self.device)
        self.gflownet.load_state_dict(checkpoint['gflownet_state_dict'])
        self.gflow_optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.training_stats = checkpoint['training_stats']
"""
Lower-Level Manipulation Controller for SOMA Cube Assembly

Implements SAC + HER (Hindsight Experience Replay) for continuous control
of precise grasping, insertion, and placement tasks.

Key Features:
- SAC for stable continuous control
- HER for sparse reward environments
- RGB-D vision + proprioceptive fusion
- Force/torque feedback for precise assembly
- High sample efficiency for real hardware deployment
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import cv2
import copy
import random
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from collections import deque
import torchvision.transforms as transforms

from .soma_cube_system import LowerLevelGoal, SOMAPiece

@dataclass
class ManipulationObservation:
    """Observation for manipulation controller"""
    rgb: np.ndarray  # (224, 224, 3) RGB image
    depth: np.ndarray  # (224, 224, 1) depth image  
    proprioception: np.ndarray  # (12,) joint states [angles, velocities]
    force_torque: np.ndarray  # (6,) force/torque sensor
    gripper_state: float  # Gripper opening [0, 1]
    target_pose: np.ndarray  # (6,) target 6DOF pose
    
@dataclass
class ManipulationAction:
    """Action for manipulation controller"""
    delta_pose: np.ndarray  # (6,) change in 6DOF pose
    gripper_command: float  # Gripper command [-1, 1]
    force_mode: bool  # Whether to use force control
    
class VisionEncoder(nn.Module):
    """
    Vision encoder for RGB-D observations
    
    Uses ResNet-style architecture with attention for robust
    feature extraction from visual observations.
    """
    
    def __init__(self, rgb_channels: int = 3, depth_channels: int = 1, 
                 output_dim: int = 256):
        super().__init__()
        
        # RGB encoder branch
        self.rgb_encoder = nn.Sequential(
            # Conv block 1
            nn.Conv2d(rgb_channels, 32, kernel_size=7, stride=2, padding=3),
            nn.BatchNorm2d(32),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(kernel_size=3, stride=2, padding=1),
            
            # Conv block 2  
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(64),
            nn.ReLU(inplace=True),
            
            # Conv block 3
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(128),
            nn.ReLU(inplace=True),
            
            # Conv block 4
            nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1),
            nn.BatchNorm2d(256),
            nn.ReLU(inplace=True),
            
            # Global average pooling
            nn.AdaptiveAvgPool2d((1, 1)),
            nn.Flatten()
        )
        
        # Depth encoder branch (lighter network)
        self.depth_encoder = nn.Sequential(
            nn.Conv2d(depth_channels, 16, kernel_size=5, stride=2, padding=2),
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 32, kernel_size=3, stride=2, padding=1), 
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 64, kernel_size=3, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d((4, 4)),
            nn.Flatten(),
            nn.Linear(64 * 16, 128),
            nn.ReLU(inplace=True)
        )
        
        # Fusion network
        self.fusion_net = nn.Sequential(
            nn.Linear(256 + 128, output_dim),
            nn.ReLU(inplace=True),
            nn.LayerNorm(output_dim),
            nn.Linear(output_dim, output_dim),
            nn.ReLU(inplace=True)
        )
        
        # Spatial attention for RGB features
        self.attention = nn.MultiheadAttention(embed_dim=256, num_heads=8, batch_first=True)
        
    def forward(self, rgb: torch.Tensor, depth: torch.Tensor) -> torch.Tensor:
        """
        Forward pass through vision encoder
        
        Args:
            rgb: (batch_size, 3, 224, 224) RGB images
            depth: (batch_size, 1, 224, 224) depth images
            
        Returns:
            (batch_size, output_dim) visual features
        """
        # Encode RGB
        rgb_features = self.rgb_encoder(rgb)  # (batch_size, 256)
        
        # Encode depth
        depth_features = self.depth_encoder(depth)  # (batch_size, 128)
        
        # Apply self-attention to RGB features (optional enhancement)
        rgb_features_expanded = rgb_features.unsqueeze(1)  # (batch_size, 1, 256)
        attended_rgb, _ = self.attention(rgb_features_expanded, rgb_features_expanded, rgb_features_expanded)
        attended_rgb = attended_rgb.squeeze(1)  # (batch_size, 256)
        
        # Fuse modalities
        combined_features = torch.cat([attended_rgb, depth_features], dim=1)
        fused_features = self.fusion_net(combined_features)
        
        return fused_features

class ProprioceptionEncoder(nn.Module):
    """Encode proprioceptive information (joint states, forces, gripper)"""
    
    def __init__(self, joint_dim: int = 12, force_dim: int = 6, 
                 gripper_dim: int = 1, output_dim: int = 64):
        super().__init__()
        
        input_dim = joint_dim + force_dim + gripper_dim
        
        self.encoder = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(inplace=True),
            nn.LayerNorm(128),
            nn.Linear(128, output_dim),
            nn.ReLU(inplace=True),
            nn.LayerNorm(output_dim)
        )
        
    def forward(self, proprioception: torch.Tensor, force_torque: torch.Tensor, 
               gripper_state: torch.Tensor) -> torch.Tensor:
        """Encode proprioceptive state"""
        combined = torch.cat([proprioception, force_torque, gripper_state], dim=-1)
        return self.encoder(combined)

class GoalEncoder(nn.Module):
    """Encode goal specification for HER"""
    
    def __init__(self, goal_dim: int = 6, output_dim: int = 32):
        super().__init__()
        
        self.encoder = nn.Sequential(
            nn.Linear(goal_dim, 64),
            nn.ReLU(inplace=True),
            nn.Linear(64, output_dim),
            nn.ReLU(inplace=True)
        )
        
    def forward(self, goal: torch.Tensor) -> torch.Tensor:
        """Encode goal pose"""
        return self.encoder(goal)

class SAC_HER_Actor(nn.Module):
    """
    SAC Actor network with HER goal conditioning
    
    Outputs continuous actions for robot manipulation with
    goal-conditioned policy for HER training.
    """
    
    def __init__(self, 
                 vision_dim: int = 256,
                 proprio_dim: int = 64,
                 goal_dim: int = 32,
                 action_dim: int = 7,  # 6DOF + gripper
                 hidden_dim: int = 512):
        super().__init__()
        
        # Feature encoders
        self.vision_encoder = VisionEncoder(output_dim=vision_dim)
        self.proprio_encoder = ProprioceptionEncoder(output_dim=proprio_dim)
        self.goal_encoder = GoalEncoder(output_dim=goal_dim)
        
        # Combined feature dimension
        combined_dim = vision_dim + proprio_dim + goal_dim
        
        # Shared feature network
        self.shared_net = nn.Sequential(
            nn.Linear(combined_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.LayerNorm(hidden_dim)
        )
        
        # Policy head (outputs mean and log_std for Gaussian policy)
        self.mean_head = nn.Linear(hidden_dim, action_dim)
        self.log_std_head = nn.Linear(hidden_dim, action_dim)
        
        # Action bounds
        self.register_buffer('action_scale', torch.ones(action_dim))
        self.register_buffer('action_bias', torch.zeros(action_dim))
        
        # Initialize weights
        self.apply(self._init_weights)
        
    def _init_weights(self, module):
        """Initialize network weights"""
        if isinstance(module, nn.Linear):
            nn.init.orthogonal_(module.weight, gain=np.sqrt(2))
            nn.init.constant_(module.bias, 0.0)
            
    def forward(self, obs: Dict[str, torch.Tensor]) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Forward pass through actor network
        
        Args:
            obs: Dictionary containing 'rgb', 'depth', 'proprioception', 
                 'force_torque', 'gripper_state', 'goal'
                 
        Returns:
            mean: Action means
            log_std: Log standard deviations
        """
        # Encode vision
        vision_features = self.vision_encoder(obs['rgb'], obs['depth'])
        
        # Encode proprioception
        proprio_features = self.proprio_encoder(
            obs['proprioception'], obs['force_torque'], obs['gripper_state']
        )
        
        # Encode goal
        goal_features = self.goal_encoder(obs['goal'])
        
        # Combine features
        combined_features = torch.cat([vision_features, proprio_features, goal_features], dim=-1)
        
        # Shared processing
        shared_out = self.shared_net(combined_features)
        
        # Policy outputs
        mean = self.mean_head(shared_out)
        log_std = self.log_std_head(shared_out)
        
        # Clamp log_std for numerical stability
        log_std = torch.clamp(log_std, min=-20, max=2)
        
        return mean, log_std
        
    def sample(self, obs: Dict[str, torch.Tensor]) -> Tuple[torch.Tensor, torch.Tensor]:
        """Sample action from policy"""
        mean, log_std = self.forward(obs)
        std = log_std.exp()
        
        # Reparameterization trick
        normal = torch.distributions.Normal(mean, std)
        x_t = normal.rsample()
        
        # Apply tanh squashing
        action = torch.tanh(x_t)
        
        # Calculate log probability
        log_prob = normal.log_prob(x_t)
        log_prob -= torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(dim=-1, keepdim=True)
        
        # Scale actions
        action = action * self.action_scale + self.action_bias
        
        return action, log_prob

class SAC_HER_Critic(nn.Module):
    """
    SAC Critic network with HER goal conditioning
    
    Twin Q-networks for reduced overestimation bias in goal-conditioned RL.
    """
    
    def __init__(self,
                 vision_dim: int = 256, 
                 proprio_dim: int = 64,
                 goal_dim: int = 32,
                 action_dim: int = 7,
                 hidden_dim: int = 512):
        super().__init__()
        
        # Feature encoders (shared with actor)
        self.vision_encoder = VisionEncoder(output_dim=vision_dim)
        self.proprio_encoder = ProprioceptionEncoder(output_dim=proprio_dim)
        self.goal_encoder = GoalEncoder(output_dim=goal_dim)
        
        # Combined input dimension
        combined_dim = vision_dim + proprio_dim + goal_dim + action_dim
        
        # Twin Q-networks
        self.q1_net = nn.Sequential(
            nn.Linear(combined_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim // 2, 1)
        )
        
        self.q2_net = nn.Sequential(
            nn.Linear(combined_dim, hidden_dim),
            nn.ReLU(inplace=True), 
            nn.LayerNorm(hidden_dim),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(inplace=True),
            nn.Linear(hidden_dim // 2, 1)
        )
        
        # Initialize weights
        self.apply(self._init_weights)
        
    def _init_weights(self, module):
        """Initialize network weights"""
        if isinstance(module, nn.Linear):
            nn.init.orthogonal_(module.weight, gain=np.sqrt(2))
            nn.init.constant_(module.bias, 0.0)
            
    def forward(self, obs: Dict[str, torch.Tensor], action: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """Forward pass through twin Q-networks"""
        # Encode features
        vision_features = self.vision_encoder(obs['rgb'], obs['depth'])
        proprio_features = self.proprio_encoder(
            obs['proprioception'], obs['force_torque'], obs['gripper_state']
        )
        goal_features = self.goal_encoder(obs['goal'])
        
        # Combine with action
        combined_features = torch.cat([vision_features, proprio_features, goal_features, action], dim=-1)
        
        # Twin Q-values
        q1 = self.q1_net(combined_features)
        q2 = self.q2_net(combined_features)
        
        return q1, q2

class HERBuffer:
    """
    Hindsight Experience Replay Buffer
    
    Stores transitions and generates additional training data by
    relabeling goals with achieved states for sparse reward environments.
    """
    
    def __init__(self, capacity: int = 1_000_000, her_ratio: float = 0.8,
                 reward_fn=None):
        self.capacity = capacity
        self.her_ratio = her_ratio  # Fraction of HER vs normal samples
        self.reward_fn = reward_fn or self._default_reward_function
        
        # Circular buffers for different data types
        self.observations = deque(maxlen=capacity)
        self.actions = deque(maxlen=capacity)
        self.rewards = deque(maxlen=capacity)
        self.next_observations = deque(maxlen=capacity)
        self.dones = deque(maxlen=capacity)
        self.goals = deque(maxlen=capacity)
        self.achieved_goals = deque(maxlen=capacity)  # What was actually achieved
        
        # Episode boundaries for HER relabeling
        self.episode_boundaries = []
        self.current_episode_start = 0
        
    def store_transition(self, obs: Dict, action: np.ndarray, reward: float,
                        next_obs: Dict, done: bool, goal: np.ndarray, 
                        achieved_goal: np.ndarray):
        """Store a single transition"""
        
        self.observations.append(obs)
        self.actions.append(action)
        self.rewards.append(reward)
        self.next_observations.append(next_obs)
        self.dones.append(done)
        self.goals.append(goal)
        self.achieved_goals.append(achieved_goal)
        
        # Track episode boundaries
        if done:
            episode_end = len(self.observations) - 1
            self.episode_boundaries.append((self.current_episode_start, episode_end))
            self.current_episode_start = len(self.observations)
            
            # Keep only recent episode boundaries
            if len(self.episode_boundaries) > 1000:
                self.episode_boundaries.pop(0)
                
    def sample(self, batch_size: int) -> Dict[str, torch.Tensor]:
        """Sample batch with HER relabeling"""
        
        if len(self.observations) < batch_size:
            return None
            
        # Determine how many HER vs normal samples
        n_her_samples = int(batch_size * self.her_ratio)
        n_normal_samples = batch_size - n_her_samples
        
        batch_obs = []
        batch_actions = []
        batch_rewards = []
        batch_next_obs = []
        batch_dones = []
        batch_goals = []
        
        # Sample normal transitions
        for _ in range(n_normal_samples):
            idx = random.randint(0, len(self.observations) - 1)
            
            batch_obs.append(self.observations[idx])
            batch_actions.append(self.actions[idx])
            batch_rewards.append(self.rewards[idx])
            batch_next_obs.append(self.next_observations[idx])
            batch_dones.append(self.dones[idx])
            batch_goals.append(self.goals[idx])
            
        # Sample HER transitions (relabeled goals)
        for _ in range(n_her_samples):
            # Sample random episode
            if not self.episode_boundaries:
                # Fall back to normal sampling if no complete episodes
                idx = random.randint(0, len(self.observations) - 1)
                new_goal = self.goals[idx]
            else:
                episode_start, episode_end = random.choice(self.episode_boundaries)
                
                # Sample transition within episode
                idx = random.randint(episode_start, episode_end)
                
                # Sample future state as new goal (HER strategy: "future")
                future_idx = random.randint(idx, episode_end)
                new_goal = self.achieved_goals[future_idx]
            
            # Create modified observation with new goal
            obs_copy = copy.deepcopy(self.observations[idx])
            next_obs_copy = copy.deepcopy(self.next_observations[idx])
            obs_copy['goal'] = new_goal
            next_obs_copy['goal'] = new_goal
            
            # Recalculate reward with new goal
            new_reward = self.reward_fn(self.achieved_goals[idx], new_goal)
            
            batch_obs.append(obs_copy)
            batch_actions.append(self.actions[idx])
            batch_rewards.append(new_reward)
            batch_next_obs.append(next_obs_copy)
            batch_dones.append(self.dones[idx])
            batch_goals.append(new_goal)
            
        # Convert to tensors
        return self._batch_to_tensors(batch_obs, batch_actions, batch_rewards, 
                                    batch_next_obs, batch_dones, batch_goals)
        
    def _batch_to_tensors(self, batch_obs, batch_actions, batch_rewards,
                         batch_next_obs, batch_dones, batch_goals) -> Dict[str, torch.Tensor]:
        """Convert batch lists to tensor format"""
        
        # Stack observations
        stacked_obs = {}
        stacked_next_obs = {}
        
        for key in batch_obs[0].keys():
            if key in ['rgb', 'depth']:
                # Image data
                stacked_obs[key] = torch.FloatTensor(np.stack([obs[key] for obs in batch_obs]))
                stacked_next_obs[key] = torch.FloatTensor(np.stack([obs[key] for obs in batch_next_obs]))
            else:
                # Vector data
                stacked_obs[key] = torch.FloatTensor(np.array([obs[key] for obs in batch_obs]))
                stacked_next_obs[key] = torch.FloatTensor(np.array([obs[key] for obs in batch_next_obs]))
        
        return {
            'observations': stacked_obs,
            'actions': torch.FloatTensor(np.array(batch_actions)),
            'rewards': torch.FloatTensor(np.array(batch_rewards)).unsqueeze(1),
            'next_observations': stacked_next_obs,
            'dones': torch.BoolTensor(np.array(batch_dones)).unsqueeze(1),
            'goals': torch.FloatTensor(np.array(batch_goals))
        }
        
    def _default_reward_function(self, achieved_goal: np.ndarray, desired_goal: np.ndarray) -> float:
        """Default sparse reward function for goal-conditioned tasks"""
        distance = np.linalg.norm(achieved_goal - desired_goal)
        tolerance = 0.05  # 5cm tolerance for position, 5 degrees for orientation
        
        # Position tolerance (first 3 dimensions)
        pos_distance = np.linalg.norm(achieved_goal[:3] - desired_goal[:3])
        pos_success = pos_distance < tolerance
        
        # Orientation tolerance (last 3 dimensions, assuming Euler angles)
        if len(achieved_goal) > 3:
            ori_distance = np.linalg.norm(achieved_goal[3:] - desired_goal[3:])
            ori_success = ori_distance < np.radians(5)  # 5 degrees
            success = pos_success and ori_success
        else:
            success = pos_success
            
        return 1.0 if success else -0.1  # Sparse reward with small negative for failure
        
    def __len__(self):
        return len(self.observations)

class LowerLevelController:
    """
    Lower-level manipulation controller using SAC + HER
    
    Handles precise grasping, insertion, and placement for SOMA cube assembly
    with RGB-D vision and force feedback integration.
    """
    
    def __init__(self,
                 device: str = 'cuda',
                 lr: float = 3e-4,
                 gamma: float = 0.99,
                 tau: float = 0.005,
                 alpha: float = 0.2,
                 batch_size: int = 256,
                 buffer_capacity: int = 1_000_000,
                 her_ratio: float = 0.8):
        
        self.device = torch.device(device if torch.cuda.is_available() else 'cpu')
        self.gamma = gamma
        self.tau = tau
        self.batch_size = batch_size
        
        # Initialize networks
        self.actor = SAC_HER_Actor().to(self.device)
        self.critic = SAC_HER_Critic().to(self.device)
        self.critic_target = copy.deepcopy(self.critic)
        
        # Freeze target network
        for param in self.critic_target.parameters():
            param.requires_grad = False
            
        # Optimizers
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=lr)
        
        # Automatic entropy tuning
        if alpha == 'auto':
            self.target_entropy = -7  # -action_dim
            self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
            self.alpha_optimizer = torch.optim.Adam([self.log_alpha], lr=lr)
            self.automatic_entropy_tuning = True
        else:
            self.alpha = alpha
            self.automatic_entropy_tuning = False
            
        # HER replay buffer
        self.replay_buffer = HERBuffer(capacity=buffer_capacity, her_ratio=her_ratio)
        
        # Training statistics
        self.training_step = 0
        self.stats = {
            'actor_loss': [],
            'critic_loss': [],
            'alpha_loss': [],
            'alpha_value': [],
            'success_rate': []
        }
        
    def get_action(self, observation: Dict[str, np.ndarray], 
                  deterministic: bool = False) -> np.ndarray:
        """Get action from current policy"""
        
        # Convert observation to tensors
        obs_tensor = self._obs_to_tensor(observation)
        
        with torch.no_grad():
            if deterministic:
                mean, _ = self.actor(obs_tensor)
                action = torch.tanh(mean) * self.actor.action_scale + self.actor.action_bias
            else:
                action, _ = self.actor.sample(obs_tensor)
                
        return action.cpu().numpy()[0]  # Remove batch dimension
        
    def _obs_to_tensor(self, observation: Dict[str, np.ndarray]) -> Dict[str, torch.Tensor]:
        """Convert numpy observation to tensor format"""
        obs_tensor = {}
        
        for key, value in observation.items():
            if key in ['rgb', 'depth']:
                # Image data: add batch dimension and convert to tensor
                if key == 'rgb':
                    # Normalize RGB to [0, 1] and transpose to CHW format
                    img = value.astype(np.float32) / 255.0
                    img = np.transpose(img, (2, 0, 1))  # HWC -> CHW
                else:  # depth
                    # Depth is already single channel
                    img = value.astype(np.float32)
                    if len(img.shape) == 2:
                        img = np.expand_dims(img, 0)  # Add channel dimension
                    else:
                        img = np.transpose(img, (2, 0, 1))
                        
                obs_tensor[key] = torch.FloatTensor(img).unsqueeze(0).to(self.device)
            else:
                # Vector data
                obs_tensor[key] = torch.FloatTensor(value).unsqueeze(0).to(self.device)
                
        return obs_tensor
        
    def store_transition(self, obs: Dict, action: np.ndarray, reward: float,
                        next_obs: Dict, done: bool, goal: np.ndarray):
        """Store transition in HER buffer"""
        
        # Extract achieved goal from observation (current end-effector pose)
        achieved_goal = obs.get('current_pose', np.zeros(6))
        
        self.replay_buffer.store_transition(
            obs, action, reward, next_obs, done, goal, achieved_goal
        )
        
    def update(self, gradient_steps: int = 1):
        """Update networks using SAC + HER"""
        
        if len(self.replay_buffer) < self.batch_size:
            return
            
        for _ in range(gradient_steps):
            self._update_step()
            
    def _update_step(self):
        """Single SAC update step with HER"""
        
        # Sample batch from HER buffer
        batch = self.replay_buffer.sample(self.batch_size)
        if batch is None:
            return
            
        observations = batch['observations']
        actions = batch['actions'].to(self.device)
        rewards = batch['rewards'].to(self.device)
        next_observations = batch['next_observations']
        dones = batch['dones'].to(self.device)
        
        # Move observations to device
        for key in observations:
            observations[key] = observations[key].to(self.device)
        for key in next_observations:
            next_observations[key] = next_observations[key].to(self.device)
            
        # Update critic networks
        with torch.no_grad():
            next_actions, next_log_probs = self.actor.sample(next_observations)
            q1_next, q2_next = self.critic_target(next_observations, next_actions)
            q_next = torch.min(q1_next, q2_next)
            
            if self.automatic_entropy_tuning:
                alpha = self.log_alpha.exp()
            else:
                alpha = self.alpha
                
            q_target = rewards + self.gamma * (1 - dones.float()) * (q_next - alpha * next_log_probs)
            
        # Critic loss
        q1_current, q2_current = self.critic(observations, actions)
        critic_loss = F.mse_loss(q1_current, q_target) + F.mse_loss(q2_current, q_target)
        
        # Update critic
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.critic.parameters(), max_norm=1.0)
        self.critic_optimizer.step()
        
        # Update actor
        new_actions, log_probs = self.actor.sample(observations)
        q1_new, q2_new = self.critic(observations, new_actions)
        q_new = torch.min(q1_new, q2_new)
        
        if self.automatic_entropy_tuning:
            alpha = self.log_alpha.exp()
            actor_loss = (alpha * log_probs - q_new).mean()
            
            # Update alpha
            alpha_loss = -(self.log_alpha * (log_probs + self.target_entropy).detach()).mean()
            self.alpha_optimizer.zero_grad()
            alpha_loss.backward()
            self.alpha_optimizer.step()
            
            self.stats['alpha_loss'].append(alpha_loss.item())
            self.stats['alpha_value'].append(alpha.item())
        else:
            alpha = self.alpha
            actor_loss = (alpha * log_probs - q_new).mean()
            
        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        torch.nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=1.0)
        self.actor_optimizer.step()
        
        # Soft update target networks
        self._soft_update_target()
        
        # Store statistics
        self.stats['actor_loss'].append(actor_loss.item())
        self.stats['critic_loss'].append(critic_loss.item())
        
        self.training_step += 1
        
    def _soft_update_target(self):
        """Soft update target networks"""
        for target_param, param in zip(self.critic_target.parameters(), self.critic.parameters()):
            target_param.data.copy_(self.tau * param.data + (1.0 - self.tau) * target_param.data)
            
    def execute_goal(self, observation: Dict[str, np.ndarray], 
                    goal: LowerLevelGoal, max_steps: int = 100) -> Tuple[bool, Dict]:
        """
        Execute manipulation goal using learned policy
        
        Args:
            observation: Current observation
            goal: Goal specification  
            max_steps: Maximum execution steps
            
        Returns:
            success: Whether goal was achieved
            info: Execution information
        """
        
        success = False
        steps_taken = 0
        total_reward = 0.0
        trajectory = []
        
        # Add goal to observation
        obs = observation.copy()
        obs['goal'] = np.array(goal.target_pose)
        
        for step in range(max_steps):
            # Get action from policy
            action = self.get_action(obs, deterministic=True)
            
            # Execute action (placeholder - would interface with robot)
            next_obs, reward, done = self._execute_action_on_robot(obs, action, goal)
            
            trajectory.append((obs, action, reward, next_obs, done))
            total_reward += reward
            steps_taken += 1
            
            # Check for success
            if self._check_goal_achievement(next_obs, goal):
                success = True
                break
                
            if done:
                break
                
            obs = next_obs
            
        return success, {
            'steps_taken': steps_taken,
            'total_reward': total_reward,
            'trajectory': trajectory,
            'final_distance': self._compute_goal_distance(obs, goal)
        }
        
    def _execute_action_on_robot(self, obs: Dict, action: np.ndarray, 
                               goal: LowerLevelGoal) -> Tuple[Dict, float, bool]:
        """Execute action on robot (placeholder implementation)"""
        # This would interface with actual robot hardware
        # For now, return simulated results
        
        next_obs = obs.copy()
        
        # Simulate robot motion
        current_pose = obs.get('current_pose', np.zeros(6))
        delta_pose = action[:6]
        gripper_cmd = action[6]
        
        # Apply action with some noise
        noise = np.random.normal(0, 0.01, 6)  # Small execution noise
        new_pose = current_pose + delta_pose + noise
        
        next_obs['current_pose'] = new_pose
        next_obs['gripper_state'] = np.array([np.clip(gripper_cmd, -1, 1)])
        
        # Calculate reward
        distance_to_goal = np.linalg.norm(new_pose - np.array(goal.target_pose))
        prev_distance = np.linalg.norm(current_pose - np.array(goal.target_pose))
        
        # Dense reward shaping
        reward = (prev_distance - distance_to_goal) * 10  # Progress reward
        
        # Success bonus
        if distance_to_goal < goal.tolerance:
            reward += 100.0
            
        # Force penalty if too high
        force_magnitude = np.linalg.norm(obs.get('force_torque', np.zeros(6)))
        if force_magnitude > 50:  # High force threshold
            reward -= 10.0
            
        done = distance_to_goal < goal.tolerance or force_magnitude > 100
        
        return next_obs, reward, done
        
    def _check_goal_achievement(self, obs: Dict, goal: LowerLevelGoal) -> bool:
        """Check if goal has been achieved"""
        current_pose = obs.get('current_pose', np.zeros(6))
        target_pose = np.array(goal.target_pose)
        
        distance = np.linalg.norm(current_pose - target_pose)
        return distance < goal.tolerance
        
    def _compute_goal_distance(self, obs: Dict, goal: LowerLevelGoal) -> float:
        """Compute distance to goal"""
        current_pose = obs.get('current_pose', np.zeros(6))
        target_pose = np.array(goal.target_pose)
        return np.linalg.norm(current_pose - target_pose)
        
    def save_model(self, filepath: str):
        """Save trained model"""
        torch.save({
            'actor_state_dict': self.actor.state_dict(),
            'critic_state_dict': self.critic.state_dict(),
            'critic_target_state_dict': self.critic_target.state_dict(),
            'actor_optimizer_state_dict': self.actor_optimizer.state_dict(),
            'critic_optimizer_state_dict': self.critic_optimizer.state_dict(),
            'training_step': self.training_step,
            'stats': self.stats
        }, filepath)
        
    def load_model(self, filepath: str):
        """Load trained model"""
        checkpoint = torch.load(filepath, map_location=self.device)
        
        self.actor.load_state_dict(checkpoint['actor_state_dict'])
        self.critic.load_state_dict(checkpoint['critic_state_dict'])
        self.critic_target.load_state_dict(checkpoint['critic_target_state_dict'])
        self.actor_optimizer.load_state_dict(checkpoint['actor_optimizer_state_dict'])
        self.critic_optimizer.load_state_dict(checkpoint['critic_optimizer_state_dict'])
        
        self.training_step = checkpoint['training_step']
        self.stats = checkpoint['stats']
        
    def get_stats(self) -> Dict:
        """Get training statistics"""
        if not self.stats['actor_loss']:
            return {}
            
        return {
            'training_steps': self.training_step,
            'avg_actor_loss': np.mean(self.stats['actor_loss'][-100:]),
            'avg_critic_loss': np.mean(self.stats['critic_loss'][-100:]),
            'buffer_size': len(self.replay_buffer),
            'success_rate': np.mean(self.stats['success_rate'][-100:]) if self.stats['success_rate'] else 0.0
        }
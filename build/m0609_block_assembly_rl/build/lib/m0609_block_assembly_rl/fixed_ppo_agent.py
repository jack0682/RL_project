"""
Corrected and Optimized PPO Agent for SOMA Cube Assembly
Fixes all import issues, dimension mismatches, and algorithm inefficiencies
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical
import numpy as np
from typing import Dict, List, Tuple, Optional, Any
import gymnasium as gym
from collections import deque
import pickle
import os
from dataclasses import dataclass

@dataclass
class OptimizedPPOConfig:
    """
    Optimized PPO hyperparameters for robotic assembly tasks
    Based on robotics RL best practices and SOMA cube complexity
    """
    # Core PPO parameters - optimized for discrete action spaces
    learning_rate: float = 1e-4  # Lower for stability in robotics
    gamma: float = 0.99  # Higher for long sequences
    lambda_gae: float = 0.95  # Standard GAE parameter
    clip_epsilon: float = 0.2  # Conservative clipping
    entropy_coef: float = 0.02  # Higher for exploration in assembly
    value_loss_coef: float = 0.5
    
    # Training parameters - optimized for sample efficiency
    batch_size: int = 128  # Smaller for better updates
    mini_batch_size: int = 32  # For multiple updates
    update_epochs: int = 4  # Fewer epochs to prevent overfitting
    max_grad_norm: float = 0.5
    
    # Network architecture - right-sized for SOMA cube complexity
    hidden_dims: List[int] = None
    dropout_rate: float = 0.1
    
    # Advanced features
    use_gae: bool = True
    normalize_advantages: bool = True
    normalize_rewards: bool = False  # Don't normalize sparse rewards
    
    def __post_init__(self):
        if self.hidden_dims is None:
            # Optimized architecture for SOMA cube (smaller, more efficient)
            self.hidden_dims = [256, 128, 64]

class PolicyNetwork(nn.Module):
    """
    Unified policy network for SOMA cube assembly
    Handles discrete action space with proper masking
    """
    
    def __init__(self, observation_space: gym.Space, action_space: gym.Space, 
                 config: OptimizedPPOConfig):
        super().__init__()
        
        # Calculate input dimension from observation space
        if isinstance(observation_space, gym.spaces.Dict):
            self.obs_dim = self._calculate_dict_obs_dim(observation_space)
            self.is_dict_obs = True
        else:
            self.obs_dim = observation_space.shape[0]
            self.is_dict_obs = False
        
        # Action space handling
        if isinstance(action_space, gym.spaces.Box):
            self.action_dim = action_space.shape[0]
            self.is_continuous = True
        elif isinstance(action_space, gym.spaces.MultiDiscrete):
            self.action_dims = action_space.nvec
            self.is_continuous = False
        else:
            raise ValueError(f"Unsupported action space: {type(action_space)}")
        
        # Shared feature extractor
        layers = []
        prev_dim = self.obs_dim
        
        for hidden_dim in config.hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.LayerNorm(hidden_dim),  # Better than BatchNorm for RL
                nn.ReLU(),
                nn.Dropout(config.dropout_rate)
            ])
            prev_dim = hidden_dim
        
        self.shared_net = nn.Sequential(*layers)
        
        # Policy heads
        if self.is_continuous:
            # Continuous action space (not typical for SOMA cube)
            self.action_mean = nn.Linear(prev_dim, self.action_dim)
            self.action_logstd = nn.Parameter(torch.zeros(self.action_dim))
        else:
            # Discrete action space - separate head for each action component
            self.action_heads = nn.ModuleList([
                nn.Linear(prev_dim, action_dim) for action_dim in self.action_dims
            ])
        
        # Value head
        self.value_head = nn.Sequential(
            nn.Linear(prev_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 1)
        )
        
        # Initialize weights
        self.apply(self._init_weights)
    
    def _calculate_dict_obs_dim(self, obs_space: gym.spaces.Dict) -> int:
        """Calculate total dimension for dict observation space"""
        total_dim = 0
        for key, space in obs_space.spaces.items():
            if isinstance(space, gym.spaces.Box):
                total_dim += np.prod(space.shape)
            elif isinstance(space, gym.spaces.Discrete):
                total_dim += 1
        return total_dim
    
    def _flatten_dict_obs(self, obs: Dict[str, torch.Tensor]) -> torch.Tensor:
        """Flatten dictionary observation to tensor"""
        flattened = []
        for key in sorted(obs.keys()):  # Consistent ordering
            tensor = obs[key]
            if tensor.dim() > 2:  # Handle multi-dimensional observations
                tensor = tensor.flatten(start_dim=1)
            flattened.append(tensor)
        return torch.cat(flattened, dim=-1)
    
    def _init_weights(self, m):
        """Initialize network weights using orthogonal initialization"""
        if isinstance(m, nn.Linear):
            torch.nn.init.orthogonal_(m.weight, gain=0.5)  # Conservative gain
            torch.nn.init.constant_(m.bias, 0.0)
    
    def forward(self, obs: torch.Tensor, action_mask: Optional[torch.Tensor] = None):
        """
        Forward pass
        
        Args:
            obs: Observations (batch_size, obs_dim) or dict
            action_mask: Optional action masking (batch_size, action_dims)
        
        Returns:
            action_logits: Action logits for each component
            value: State value estimate
        """
        # Handle dict observations
        if self.is_dict_obs and isinstance(obs, dict):
            obs = self._flatten_dict_obs(obs)
        
        # Shared feature extraction
        features = self.shared_net(obs)
        
        # Value prediction
        value = self.value_head(features)
        
        # Action prediction
        if self.is_continuous:
            action_mean = self.action_mean(features)
            action_std = torch.exp(self.action_logstd)
            return action_mean, action_std, value
        else:
            action_logits = []
            for i, head in enumerate(self.action_heads):
                logits = head(features)
                
                # Apply action masking if provided
                if action_mask is not None and action_mask.shape[-1] > i:
                    mask = action_mask[:, i]
                    logits = logits.masked_fill(~mask.bool(), -1e8)
                
                action_logits.append(logits)
            
            return action_logits, value

class M0609PPOAgent:
    """
    Fixed and optimized PPO agent for SOMA cube assembly
    Implements proper sample collection, advantage estimation, and policy updates
    """
    
    def __init__(self, observation_space: gym.Space, action_space: gym.Space,
                 config: OptimizedPPOConfig = None, device: str = None):
        
        self.config = config or OptimizedPPOConfig()
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        
        # Store spaces
        self.observation_space = observation_space
        self.action_space = action_space
        
        # Create network
        self.policy_net = PolicyNetwork(observation_space, action_space, self.config).to(self.device)
        
        # Optimizer
        self.optimizer = optim.Adam(
            self.policy_net.parameters(), 
            lr=self.config.learning_rate,
            eps=1e-5  # Better numerical stability
        )
        
        # LR scheduler for adaptive learning
        self.lr_scheduler = optim.lr_scheduler.CosineAnnealingLR(
            self.optimizer, T_max=10000, eta_min=1e-6
        )
        
        # Experience buffer
        self.buffer = PPOBuffer(self.config, self.device)
        
        # Training metrics
        self.training_stats = {
            'policy_loss': deque(maxlen=100),
            'value_loss': deque(maxlen=100),
            'entropy': deque(maxlen=100),
            'kl_divergence': deque(maxlen=100)
        }
    
    def select_action(self, obs: Any, action_mask: Optional[np.ndarray] = None) -> Tuple[Any, Dict]:
        """
        Select action given observation
        
        Returns:
            action: Selected action
            info: Additional information (log_prob, value, etc.)
        """
        self.policy_net.eval()
        
        with torch.no_grad():
            # Convert observation to tensor
            if isinstance(obs, dict):
                obs_tensor = {k: torch.FloatTensor(v).unsqueeze(0).to(self.device) 
                             for k, v in obs.items()}
            else:
                obs_tensor = torch.FloatTensor(obs).unsqueeze(0).to(self.device)
            
            # Convert action mask if provided
            mask_tensor = None
            if action_mask is not None:
                mask_tensor = torch.BoolTensor(action_mask).unsqueeze(0).to(self.device)
            
            # Forward pass
            if self.policy_net.is_continuous:
                action_mean, action_std, value = self.policy_net(obs_tensor, mask_tensor)
                dist = torch.distributions.Normal(action_mean, action_std)
                action = dist.sample()
                log_prob = dist.log_prob(action).sum(-1)
                action = action.cpu().numpy()[0]
            else:
                action_logits, value = self.policy_net(obs_tensor, mask_tensor)
                
                # Sample from categorical distributions
                action_components = []
                log_probs = []
                
                for logits in action_logits:
                    dist = Categorical(logits=logits)
                    action_comp = dist.sample()
                    log_prob = dist.log_prob(action_comp)
                    
                    action_components.append(action_comp.cpu().numpy()[0])
                    log_probs.append(log_prob)
                
                action = np.array(action_components)
                log_prob = torch.stack(log_probs).sum()
            
            info = {
                'log_prob': log_prob.cpu().numpy(),
                'value': value.cpu().numpy()[0, 0],
                'entropy': self._calculate_entropy(action_logits if not self.policy_net.is_continuous else None)
            }
        
        return action, info
    
    def _calculate_entropy(self, action_logits: Optional[List[torch.Tensor]]) -> float:
        """Calculate policy entropy"""
        if action_logits is None:
            return 0.0
        
        total_entropy = 0.0
        for logits in action_logits:
            probs = F.softmax(logits, dim=-1)
            entropy = -(probs * torch.log(probs + 1e-8)).sum(-1)
            total_entropy += entropy.mean().item()
        
        return total_entropy / len(action_logits)
    
    def store_experience(self, obs: Any, action: Any, reward: float, done: bool, info: Dict):
        """Store experience in buffer"""
        self.buffer.store(obs, action, reward, done, info)
    
    def update(self) -> Dict[str, float]:
        """
        Update policy using collected experiences
        
        Returns:
            training_stats: Dictionary of training statistics
        """
        if not self.buffer.is_ready():
            return {}
        
        # Get batch data
        batch_data = self.buffer.get_batch()
        
        # Compute advantages and returns
        advantages, returns = self._compute_gae(
            batch_data['rewards'], batch_data['values'], batch_data['dones']
        )
        
        # Normalize advantages
        if self.config.normalize_advantages:
            advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # Multiple epochs of updates
        total_stats = {'policy_loss': 0, 'value_loss': 0, 'entropy': 0, 'kl_div': 0}
        
        for epoch in range(self.config.update_epochs):
            # Mini-batch updates
            indices = torch.randperm(len(advantages))
            
            for start in range(0, len(indices), self.config.mini_batch_size):
                end = start + self.config.mini_batch_size
                batch_indices = indices[start:end]
                
                # Get mini-batch
                mb_obs = self._index_batch(batch_data['observations'], batch_indices)
                mb_actions = batch_data['actions'][batch_indices]
                mb_old_log_probs = batch_data['log_probs'][batch_indices]
                mb_advantages = advantages[batch_indices]
                mb_returns = returns[batch_indices]
                
                # Update step
                stats = self._update_minibatch(
                    mb_obs, mb_actions, mb_old_log_probs, mb_advantages, mb_returns
                )
                
                # Accumulate stats
                for key, value in stats.items():
                    total_stats[key] += value
        
        # Average stats
        num_updates = self.config.update_epochs * (len(advantages) // self.config.mini_batch_size)
        if num_updates > 0:
            for key in total_stats:
                total_stats[key] /= num_updates
        
        # Update learning rate
        self.lr_scheduler.step()
        
        # Store training stats
        for key, value in total_stats.items():
            if key in self.training_stats:
                self.training_stats[key].append(value)
        
        # Clear buffer
        self.buffer.clear()
        
        return total_stats
    
    def _compute_gae(self, rewards: torch.Tensor, values: torch.Tensor, 
                     dones: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """Compute Generalized Advantage Estimation"""
        # Ensure rewards, values, and dones have compatible dimensions
        seq_len = len(rewards)
        
        # Append zero value for last step if needed
        if len(values) == seq_len:
            next_values = torch.cat([values[1:], torch.zeros(1, device=values.device)])
        else:
            next_values = values[1:]
            
        advantages = torch.zeros_like(rewards)
        gae = 0
        
        # Convert boolean dones to float for arithmetic
        dones_float = dones.float()
        
        for t in reversed(range(seq_len)):
            if t == seq_len - 1:
                # Last timestep
                next_value = 0
                next_done = 1
            else:
                next_value = next_values[t]
                next_done = dones_float[t]
            
            delta = rewards[t] + self.config.gamma * next_value * (1 - next_done) - values[t]
            gae = delta + self.config.gamma * self.config.lambda_gae * (1 - next_done) * gae
            advantages[t] = gae
        
        returns = advantages + values[:seq_len]  # Ensure same dimension
        return advantages, returns
    
    def _index_batch(self, batch_data: Any, indices: torch.Tensor) -> Any:
        """Index batch data for mini-batch updates"""
        if isinstance(batch_data, dict):
            return {k: v[indices] for k, v in batch_data.items()}
        else:
            return batch_data[indices]
    
    def _update_minibatch(self, obs, actions, old_log_probs, advantages, returns) -> Dict[str, float]:
        """Update policy on mini-batch"""
        self.policy_net.train()
        
        # Forward pass
        if self.policy_net.is_continuous:
            action_mean, action_std, values = self.policy_net(obs)
            dist = torch.distributions.Normal(action_mean, action_std)
            new_log_probs = dist.log_prob(actions).sum(-1)
            entropy = dist.entropy().sum(-1).mean()
        else:
            action_logits, values = self.policy_net(obs)
            
            # Calculate log probabilities and entropy
            new_log_probs = []
            entropies = []
            
            for i, logits in enumerate(action_logits):
                dist = Categorical(logits=logits)
                log_prob = dist.log_prob(actions[:, i])
                entropy = dist.entropy()
                
                new_log_probs.append(log_prob)
                entropies.append(entropy)
            
            new_log_probs = torch.stack(new_log_probs).sum(0)
            entropy = torch.stack(entropies).mean()
        
        values = values.squeeze()
        
        # PPO clipped objective
        ratio = torch.exp(new_log_probs - old_log_probs)
        surr1 = ratio * advantages
        surr2 = torch.clamp(ratio, 1 - self.config.clip_epsilon, 1 + self.config.clip_epsilon) * advantages
        
        policy_loss = -torch.min(surr1, surr2).mean()
        
        # Value loss
        value_loss = F.mse_loss(values, returns)
        
        # Total loss
        total_loss = (policy_loss + 
                     self.config.value_loss_coef * value_loss - 
                     self.config.entropy_coef * entropy)
        
        # Backward pass
        self.optimizer.zero_grad()
        total_loss.backward()
        
        # Gradient clipping
        torch.nn.utils.clip_grad_norm_(self.policy_net.parameters(), self.config.max_grad_norm)
        
        self.optimizer.step()
        
        # Calculate KL divergence for monitoring
        with torch.no_grad():
            kl_div = (old_log_probs - new_log_probs).mean()
        
        return {
            'policy_loss': policy_loss.item(),
            'value_loss': value_loss.item(),
            'entropy': entropy.item(),
            'kl_div': kl_div.item()
        }
    
    def save_model(self, path: str):
        """Save model checkpoint"""
        os.makedirs(os.path.dirname(path), exist_ok=True)
        torch.save({
            'policy_net_state_dict': self.policy_net.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'lr_scheduler_state_dict': self.lr_scheduler.state_dict(),
            'config': self.config,
            'training_stats': dict(self.training_stats)
        }, path)
    
    def load_model(self, path: str):
        """Load model checkpoint"""
        # Fix for PyTorch 2.6+ weights_only default change
        checkpoint = torch.load(path, map_location=self.device, weights_only=False)
        self.policy_net.load_state_dict(checkpoint['policy_net_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        
        if 'lr_scheduler_state_dict' in checkpoint:
            self.lr_scheduler.load_state_dict(checkpoint['lr_scheduler_state_dict'])
        
        if 'training_stats' in checkpoint:
            for key, values in checkpoint['training_stats'].items():
                self.training_stats[key] = deque(values, maxlen=100)

class PPOBuffer:
    """Experience buffer for PPO agent"""
    
    def __init__(self, config: OptimizedPPOConfig, device: str = "cpu"):
        self.config = config
        self.device = device
        self.clear()
    
    def store(self, obs: Any, action: Any, reward: float, done: bool, info: Dict):
        """Store single experience"""
        self.observations.append(obs)
        self.actions.append(action)
        self.rewards.append(reward)
        self.dones.append(done)
        self.log_probs.append(info['log_prob'])
        self.values.append(info['value'])
    
    def is_ready(self) -> bool:
        """Check if buffer has enough samples"""
        return len(self.observations) >= self.config.batch_size
    
    def get_batch(self) -> Dict[str, torch.Tensor]:
        """Get batch data as tensors"""
        # Convert to tensors on correct device
        if isinstance(self.observations[0], dict):
            # Handle dict observations
            obs_dict = {}
            for key in self.observations[0].keys():
                obs_dict[key] = torch.FloatTensor([obs[key] for obs in self.observations]).to(self.device)
            observations = obs_dict
        else:
            observations = torch.FloatTensor(self.observations).to(self.device)
        
        return {
            'observations': observations,
            'actions': torch.FloatTensor(self.actions).to(self.device),
            'rewards': torch.FloatTensor(self.rewards).to(self.device),
            'dones': torch.BoolTensor(self.dones).to(self.device),
            'log_probs': torch.FloatTensor(self.log_probs).to(self.device),
            'values': torch.FloatTensor(self.values).to(self.device)
        }
    
    def clear(self):
        """Clear buffer"""
        self.observations = []
        self.actions = []
        self.rewards = []
        self.dones = []
        self.log_probs = []
        self.values = []

# Utility function for creating agent with environment
def create_ppo_agent(env: gym.Env, config: OptimizedPPOConfig = None, device: str = None) -> M0609PPOAgent:
    """
    Create properly configured PPO agent for given environment
    
    Args:
        env: Gym environment
        config: PPO configuration
        device: Device to use
    
    Returns:
        Configured PPO agent
    """
    return M0609PPOAgent(
        observation_space=env.observation_space,
        action_space=env.action_space,
        config=config,
        device=device
    )
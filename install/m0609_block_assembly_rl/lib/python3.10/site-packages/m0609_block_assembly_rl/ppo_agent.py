"""
PPO Agent for M0609 Block Assembly Optimization

This module implements a Proximal Policy Optimization (PPO) agent with
hierarchical policy networks for Doosan M0609 robot block assembly tasks.
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.distributions import Categorical, Normal
import numpy as np
from typing import Dict, List, Tuple, Optional
import gymnasium as gym
from collections import deque
import pickle
import os
from dataclasses import dataclass

@dataclass
class PPOConfig:
    """PPO hyperparameters configuration"""
    learning_rate: float = 3e-4
    gamma: float = 0.95
    lambda_gae: float = 0.95
    clip_epsilon: float = 0.2
    entropy_coef: float = 0.01
    value_loss_coef: float = 0.5
    batch_size: int = 256
    update_epochs: int = 10
    max_grad_norm: float = 0.5
    
    # Network architecture
    hidden_dims: List[int] = None
    
    def __post_init__(self):
        if self.hidden_dims is None:
            self.hidden_dims = [512, 256, 128]

class StrategyNetwork(nn.Module):
    """
    Strategic policy network for block selection and approach strategy
    
    Input: Environment state (179-dim)
    Output: 
    - Block selection probabilities (7-dim)
    - Approach strategy probabilities (3-dim)
    """
    
    def __init__(self, state_dim: int = 179, num_blocks: int = 7, 
                 num_strategies: int = 3, hidden_dims: List[int] = [512, 256, 128]):
        super().__init__()
        
        # Shared feature extraction
        layers = []
        prev_dim = state_dim
        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(),
                nn.Dropout(0.1)
            ])
            prev_dim = hidden_dim
        
        self.shared_net = nn.Sequential(*layers)
        
        # Block selection head
        self.block_head = nn.Sequential(
            nn.Linear(prev_dim, 64),
            nn.ReLU(),
            nn.Linear(64, num_blocks)
        )
        
        # Approach strategy head
        self.strategy_head = nn.Sequential(
            nn.Linear(prev_dim, 32),
            nn.ReLU(),
            nn.Linear(32, num_strategies)
        )
        
        # Initialize weights
        self.apply(self._init_weights)
    
    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            torch.nn.init.orthogonal_(m.weight, 0.01)
            torch.nn.init.constant_(m.bias, 0.0)
    
    def forward(self, state: torch.Tensor, available_blocks: torch.Tensor = None) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Forward pass
        
        Args:
            state: Environment state tensor (batch_size, 224)
            available_blocks: Available blocks mask (batch_size, 7)
        
        Returns:
            block_logits: Block selection logits (batch_size, 7)
            strategy_logits: Strategy selection logits (batch_size, 3)
        """
        features = self.shared_net(state)
        
        block_logits = self.block_head(features)
        strategy_logits = self.strategy_head(features)
        
        # Mask unavailable blocks
        if available_blocks is not None:
            block_logits = block_logits + (available_blocks - 1) * 1e8
        
        return block_logits, strategy_logits

class GraspNetwork(nn.Module):
    """
    Continuous policy network for grasp optimization
    
    Input: Environment state + selected block (179 + 7 = 186-dim)
    Output: Grasp parameters (position, orientation, force)
    """
    
    def __init__(self, input_dim: int = 186, hidden_dims: List[int] = [256, 128, 64]):
        super().__init__()
        
        # Feature extraction
        layers = []
        prev_dim = input_dim
        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(),
                nn.Dropout(0.1)
            ])
            prev_dim = hidden_dim
        
        self.shared_net = nn.Sequential(*layers)
        
        # Grasp position head (3-dim, range [-1, 1])
        self.position_mean = nn.Sequential(
            nn.Linear(prev_dim, 32),
            nn.ReLU(),
            nn.Linear(32, 3),
            nn.Tanh()
        )
        self.position_log_std = nn.Parameter(torch.zeros(3) - 1.0)
        
        # Grasp orientation head (3-dim, range [-π, π])
        self.orientation_mean = nn.Sequential(
            nn.Linear(prev_dim, 32),
            nn.ReLU(),  
            nn.Linear(32, 3),
            nn.Tanh()
        )
        self.orientation_log_std = nn.Parameter(torch.zeros(3) - 1.0)
        
        # Grasp force head (1-dim, range [0.1, 1.0])
        self.force_mean = nn.Sequential(
            nn.Linear(prev_dim, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
            nn.Sigmoid()
        )
        self.force_log_std = nn.Parameter(torch.zeros(1) - 2.0)
        
        # Fine tuning heads
        # Placement offset (3-dim, range [-0.02, 0.02])
        self.offset_mean = nn.Sequential(
            nn.Linear(prev_dim, 16),
            nn.ReLU(),
            nn.Linear(16, 3),
            nn.Tanh()
        )
        self.offset_log_std = nn.Parameter(torch.zeros(3) - 3.0)
        
        # Rotation adjustment (3-dim, range [-π/12, π/12])
        self.rotation_adj_mean = nn.Sequential(
            nn.Linear(prev_dim, 16),
            nn.ReLU(),
            nn.Linear(16, 3),
            nn.Tanh()
        )
        self.rotation_adj_log_std = nn.Parameter(torch.zeros(3) - 2.0)
        
        # Initialize weights
        self.apply(self._init_weights)
    
    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            torch.nn.init.orthogonal_(m.weight, 0.01)
            torch.nn.init.constant_(m.bias, 0.0)
    
    def forward(self, state_and_block: torch.Tensor) -> Dict[str, Tuple[torch.Tensor, torch.Tensor]]:
        """
        Forward pass
        
        Args:
            state_and_block: Concatenated state and one-hot block selection
        
        Returns:
            Dictionary with mean and log_std for each continuous action component
        """
        features = self.shared_net(state_and_block)
        
        return {
            'grasp_position': (self.position_mean(features), self.position_log_std.expand_as(self.position_mean(features))),
            'grasp_orientation': (self.orientation_mean(features) * np.pi, self.orientation_log_std.expand_as(self.orientation_mean(features))),
            'grasp_force': (self.force_mean(features) * 0.9 + 0.1, self.force_log_std.expand_as(self.force_mean(features))),
            'placement_offset': (self.offset_mean(features) * 0.02, self.offset_log_std.expand_as(self.offset_mean(features))),
            'rotation_adjustment': (self.rotation_adj_mean(features) * np.pi/12, self.rotation_adj_log_std.expand_as(self.rotation_adj_mean(features)))
        }

class ValueNetwork(nn.Module):
    """
    State value network for advantage estimation
    """
    
    def __init__(self, state_dim: int = 179, hidden_dims: List[int] = [512, 256, 128]):
        super().__init__()
        
        layers = []
        prev_dim = state_dim
        for hidden_dim in hidden_dims:
            layers.extend([
                nn.Linear(prev_dim, hidden_dim),
                nn.ReLU(),
                nn.Dropout(0.1)
            ])
            prev_dim = hidden_dim
        
        layers.append(nn.Linear(prev_dim, 1))
        
        self.network = nn.Sequential(*layers)
        
        # Initialize weights
        self.apply(self._init_weights)
    
    def _init_weights(self, m):
        if isinstance(m, nn.Linear):
            torch.nn.init.orthogonal_(m.weight, 1.0)
            torch.nn.init.constant_(m.bias, 0.0)
    
    def forward(self, state: torch.Tensor) -> torch.Tensor:
        return self.network(state).squeeze(-1)

class PPOBuffer:
    """
    Buffer for storing trajectory data for PPO updates
    """
    
    def __init__(self, capacity: int = 10000):
        self.capacity = capacity
        self.buffer = deque(maxlen=capacity)
    
    def store(self, state, action, reward, next_state, done, 
              log_prob, value, advantage=None, return_=None):
        """Store a single transition"""
        self.buffer.append({
            'state': state,
            'action': action,
            'reward': reward,
            'next_state': next_state,
            'done': done,
            'log_prob': log_prob,
            'value': value,
            'advantage': advantage,
            'return': return_
        })
    
    def get_batch(self, batch_size: int) -> Dict:
        """Get a random batch of transitions"""
        if len(self.buffer) < batch_size:
            return None
        
        indices = np.random.choice(len(self.buffer), batch_size, replace=False)
        batch = [self.buffer[i] for i in indices]
        
        # Collate batch
        collated = {}
        for key in batch[0].keys():
            if key == 'action':
                # Handle dictionary actions
                action_batch = {}
                for action_key in batch[0]['action'].keys():
                    tensors = [b['action'][action_key] if isinstance(b['action'][action_key], torch.Tensor) 
                              else torch.tensor(b['action'][action_key], dtype=torch.float32) for b in batch]
                    action_batch[action_key] = torch.stack(tensors)
                collated[key] = action_batch
            else:
                tensors = [b[key] if isinstance(b[key], torch.Tensor) 
                          else torch.tensor(b[key], dtype=torch.float32) for b in batch]
                collated[key] = torch.stack(tensors)
        
        return collated
    
    def compute_gae(self, gamma: float, lambda_gae: float):
        """Compute Generalized Advantage Estimation"""
        advantages = []
        returns = []
        
        gae = 0
        for i in reversed(range(len(self.buffer))):
            delta = (self.buffer[i]['reward'] + 
                    gamma * (0 if self.buffer[i]['done'] else self.buffer[min(i+1, len(self.buffer)-1)]['value']) - 
                    self.buffer[i]['value'])
            gae = delta + gamma * lambda_gae * (0 if self.buffer[i]['done'] else gae)
            advantages.insert(0, gae)
            returns.insert(0, gae + self.buffer[i]['value'])
        
        # Normalize advantages
        advantages = torch.tensor(advantages, dtype=torch.float32)
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1e-8)
        
        # Store in buffer
        for i, (adv, ret) in enumerate(zip(advantages, returns)):
            self.buffer[i]['advantage'] = adv
            self.buffer[i]['return'] = ret
    
    def clear(self):
        """Clear the buffer"""
        self.buffer.clear()
    
    def __len__(self):
        return len(self.buffer)

class M0609PPOAgent:
    """
    PPO Agent for M0609 Block Assembly Task
    
    Implements hierarchical policy with strategic and grasp networks
    """
    
    def __init__(self, config: PPOConfig = None, device: str = 'cpu'):
        self.config = config or PPOConfig()
        self.device = torch.device(device)
        
        # Networks
        self.strategy_net = StrategyNetwork(
            hidden_dims=self.config.hidden_dims
        ).to(self.device)
        
        self.grasp_net = GraspNetwork(
            hidden_dims=[256, 128, 64]
        ).to(self.device)
        
        self.value_net = ValueNetwork(
            hidden_dims=self.config.hidden_dims
        ).to(self.device)
        
        # Optimizers
        self.strategy_optimizer = optim.Adam(
            self.strategy_net.parameters(), 
            lr=self.config.learning_rate
        )
        
        self.grasp_optimizer = optim.Adam(
            self.grasp_net.parameters(), 
            lr=self.config.learning_rate
        )
        
        self.value_optimizer = optim.Adam(
            self.value_net.parameters(), 
            lr=self.config.learning_rate
        )
        
        # Experience buffer
        self.buffer = PPOBuffer()
        
        # Training statistics
        self.training_stats = {
            'total_steps': 0,
            'episodes': 0,
            'policy_loss': [],
            'value_loss': [], 
            'entropy': [],
            'advantages': [],
            'returns': []
        }
    
    def select_action(self, state: np.ndarray, available_blocks: np.ndarray = None, 
                     deterministic: bool = False) -> Tuple[Dict, torch.Tensor, torch.Tensor]:
        """
        Select action given current state
        
        Args:
            state: Environment state
            available_blocks: Available blocks mask
            deterministic: Whether to use deterministic policy
        
        Returns:
            action: Action dictionary
            log_prob: Log probability of selected action
            value: State value estimate
        """
        with torch.no_grad():
            state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            available_tensor = torch.FloatTensor(available_blocks).unsqueeze(0).to(self.device) if available_blocks is not None else None
            
            # Strategic action selection
            block_logits, strategy_logits = self.strategy_net(state_tensor, available_tensor)
            
            if deterministic:
                block_action = torch.argmax(block_logits, dim=-1)
                strategy_action = torch.argmax(strategy_logits, dim=-1)
            else:
                block_dist = Categorical(logits=block_logits)
                strategy_dist = Categorical(logits=strategy_logits)
                
                block_action = block_dist.sample()
                strategy_action = strategy_dist.sample()
            
            # Create one-hot encoding for selected block
            block_onehot = torch.zeros(1, 7).to(self.device)
            block_onehot[0, block_action] = 1.0
            
            # Continuous action selection (grasp parameters)
            state_and_block = torch.cat([state_tensor, block_onehot], dim=-1)
            grasp_outputs = self.grasp_net(state_and_block)
            
            continuous_actions = {}
            continuous_log_probs = []
            
            for key, (mean, log_std) in grasp_outputs.items():
                std = torch.exp(log_std)
                if deterministic:
                    action = mean
                else:
                    dist = Normal(mean, std)
                    action = dist.sample()
                    continuous_log_probs.append(dist.log_prob(action).sum(-1))
                
                continuous_actions[key] = action.squeeze(0).cpu().numpy()
            
            # Combine all actions
            action = {
                'next_block': block_action.item(),
                'approach_strategy': strategy_action.item(),
                **continuous_actions
            }
            
            # Calculate total log probability
            if not deterministic:
                strategic_log_prob = (block_dist.log_prob(block_action) + 
                                     strategy_dist.log_prob(strategy_action))
                total_log_prob = strategic_log_prob + sum(continuous_log_probs)
            else:
                total_log_prob = torch.tensor(0.0, dtype=torch.float32)
            
            # Get value estimate
            value = self.value_net(state_tensor)
            
            return action, total_log_prob, value
    
    def compute_action_log_prob(self, state: torch.Tensor, action: Dict, 
                               available_blocks: torch.Tensor = None) -> torch.Tensor:
        """Compute log probability of given action"""
        # Strategic actions
        block_logits, strategy_logits = self.strategy_net(state, available_blocks)
        
        block_dist = Categorical(logits=block_logits)
        strategy_dist = Categorical(logits=strategy_logits)
        
        strategic_log_prob = (block_dist.log_prob(action['next_block']) + 
                             strategy_dist.log_prob(action['approach_strategy']))
        
        # Continuous actions
        block_onehot = torch.zeros(state.shape[0], 7).to(self.device)
        indices = torch.arange(state.shape[0]).to(self.device)
        block_onehot = block_onehot.scatter(1, action['next_block'].unsqueeze(1).long(), 1.0)
        
        state_and_block = torch.cat([state, block_onehot], dim=-1)
        grasp_outputs = self.grasp_net(state_and_block)
        
        continuous_log_prob = 0
        for key in ['grasp_position', 'grasp_orientation', 'grasp_force', 
                   'placement_offset', 'rotation_adjustment']:
            if key in action:
                mean, log_std = grasp_outputs[key]
                std = torch.exp(log_std)
                dist = Normal(mean, std)
                continuous_log_prob += dist.log_prob(action[key]).sum(-1)
        
        return strategic_log_prob + continuous_log_prob
    
    def update(self):
        """Perform PPO update"""
        if len(self.buffer) < self.config.batch_size:
            return
        
        # Compute GAE
        self.buffer.compute_gae(self.config.gamma, self.config.lambda_gae)
        
        # Multiple epochs of updates
        for _ in range(self.config.update_epochs):
            batch = self.buffer.get_batch(self.config.batch_size)
            if batch is None:
                continue
            
            states = batch['state'].to(self.device)
            actions = {k: v.to(self.device) for k, v in batch['action'].items()}
            old_log_probs = batch['log_prob'].to(self.device)
            advantages = batch['advantage'].to(self.device)
            returns = batch['return'].to(self.device)
            
            # Available blocks mask (simplified)
            available_blocks = torch.ones(states.shape[0], 7).to(self.device)  # All blocks available
            
            # Compute current policy log probabilities
            new_log_probs = self.compute_action_log_prob(states, actions, available_blocks)
            
            # Compute ratio for clipping
            ratio = torch.exp(new_log_probs - old_log_probs)
            
            # Compute policy loss
            surr1 = ratio * advantages
            surr2 = torch.clamp(ratio, 1 - self.config.clip_epsilon, 
                              1 + self.config.clip_epsilon) * advantages
            policy_loss = -torch.min(surr1, surr2).mean()
            
            # Compute value loss
            values = self.value_net(states)
            value_loss = F.mse_loss(values, returns)
            
            # Compute entropy bonus (simplified)
            entropy = -(new_log_probs * torch.exp(new_log_probs)).mean()
            
            # Total loss
            total_loss = (policy_loss + 
                         self.config.value_loss_coef * value_loss - 
                         self.config.entropy_coef * entropy)
            
            # Optimize all networks together to avoid gradient computation issues
            self.strategy_optimizer.zero_grad()
            self.grasp_optimizer.zero_grad()
            self.value_optimizer.zero_grad()
            
            # Compute gradients for the combined loss
            total_loss.backward()
            
            # Clip gradients
            torch.nn.utils.clip_grad_norm_(self.strategy_net.parameters(), 
                                         self.config.max_grad_norm)
            torch.nn.utils.clip_grad_norm_(self.grasp_net.parameters(), 
                                         self.config.max_grad_norm)
            torch.nn.utils.clip_grad_norm_(self.value_net.parameters(), 
                                         self.config.max_grad_norm)
            
            # Update parameters
            self.strategy_optimizer.step()
            self.grasp_optimizer.step()
            self.value_optimizer.step()
            
            # Update statistics
            self.training_stats['policy_loss'].append(policy_loss.item())
            self.training_stats['value_loss'].append(value_loss.item())
            self.training_stats['entropy'].append(entropy.item())
            self.training_stats['advantages'].extend(advantages.detach().cpu().numpy())
            self.training_stats['returns'].extend(returns.detach().cpu().numpy())
        
        # Clear buffer after update
        self.buffer.clear()
        self.training_stats['total_steps'] += 1
    
    def store_experience(self, state, action, reward, next_state, done, log_prob, value):
        """Store experience in buffer"""
        self.buffer.store(state, action, reward, next_state, done, 
                         log_prob.item(), value.item())
    
    def save_model(self, filepath: str):
        """Save trained models"""
        torch.save({
            'strategy_net': self.strategy_net.state_dict(),
            'grasp_net': self.grasp_net.state_dict(),
            'value_net': self.value_net.state_dict(),
            'strategy_optimizer': self.strategy_optimizer.state_dict(),
            'grasp_optimizer': self.grasp_optimizer.state_dict(), 
            'value_optimizer': self.value_optimizer.state_dict(),
            'config': self.config,
            'training_stats': self.training_stats
        }, filepath)
    
    def load_model(self, filepath: str):
        """Load trained models"""
        checkpoint = torch.load(filepath, map_location=self.device)
        
        self.strategy_net.load_state_dict(checkpoint['strategy_net'])
        self.grasp_net.load_state_dict(checkpoint['grasp_net'])
        self.value_net.load_state_dict(checkpoint['value_net'])
        
        if 'strategy_optimizer' in checkpoint:
            self.strategy_optimizer.load_state_dict(checkpoint['strategy_optimizer'])
        if 'grasp_optimizer' in checkpoint:
            self.grasp_optimizer.load_state_dict(checkpoint['grasp_optimizer'])
        if 'value_optimizer' in checkpoint:
            self.value_optimizer.load_state_dict(checkpoint['value_optimizer'])
        
        if 'training_stats' in checkpoint:
            self.training_stats = checkpoint['training_stats']
    
    def get_training_stats(self) -> Dict:
        """Get training statistics"""
        return {
            'total_steps': self.training_stats['total_steps'],
            'episodes': self.training_stats['episodes'],
            'avg_policy_loss': np.mean(self.training_stats['policy_loss'][-100:]) if self.training_stats['policy_loss'] else 0,
            'avg_value_loss': np.mean(self.training_stats['value_loss'][-100:]) if self.training_stats['value_loss'] else 0,
            'avg_entropy': np.mean(self.training_stats['entropy'][-100:]) if self.training_stats['entropy'] else 0,
            'avg_advantage': np.mean(self.training_stats['advantages'][-1000:]) if self.training_stats['advantages'] else 0,
            'avg_return': np.mean(self.training_stats['returns'][-1000:]) if self.training_stats['returns'] else 0
        }

if __name__ == "__main__":
    # Test agent creation
    config = PPOConfig()
    agent = M0609PPOAgent(config)
    
    print("PPO Agent created successfully!")
    print(f"Strategy network parameters: {sum(p.numel() for p in agent.strategy_net.parameters())}")
    print(f"Grasp network parameters: {sum(p.numel() for p in agent.grasp_net.parameters())}")
    print(f"Value network parameters: {sum(p.numel() for p in agent.value_net.parameters())}")
    
    # Test action selection
    dummy_state = np.random.randn(224)
    dummy_available = np.ones(7)
    
    action, log_prob, value = agent.select_action(dummy_state, dummy_available)
    print(f"Sample action: {action}")
    print(f"Log probability: {log_prob}")
    print(f"Value estimate: {value}")
    
    print("Agent test completed successfully!")
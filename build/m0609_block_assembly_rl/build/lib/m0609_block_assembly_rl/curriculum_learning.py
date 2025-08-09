"""
Curriculum Learning System for Hierarchical SOMA Cube Assembly

This module implements a progressive curriculum learning system that:
1. Starts with 2-3 block subproblems
2. Gradually increases complexity to full 7-block assemblies
3. Adapts learning progression based on performance
4. Provides automatic difficulty adjustment and success tracking
"""

import numpy as np
import torch
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass
from collections import deque, defaultdict
from enum import Enum
import random
import logging
import json
import time
from pathlib import Path

from .soma_environment import SOMAEnvironment, SOMABlock, BlockPose

logger = logging.getLogger(__name__)

class CurriculumStage(Enum):
    """Curriculum learning stages"""
    BASIC_PLACEMENT = 0      # 2 blocks, simple placements
    INTERMEDIATE = 1         # 3-4 blocks, moderate complexity
    ADVANCED = 2            # 5-6 blocks, high complexity
    EXPERT = 3              # Full 7-block assembly
    MASTERY = 4             # Variations and robustness

@dataclass
class CurriculumTask:
    """Defines a curriculum learning task"""
    stage: CurriculumStage
    num_blocks: int
    blocks_subset: List[SOMABlock]
    target_success_rate: float
    max_steps: int
    reward_shaping: Dict[str, float]
    complexity_weight: float
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'stage': self.stage.value,
            'num_blocks': self.num_blocks,
            'blocks_subset': [b.value for b in self.blocks_subset],
            'target_success_rate': self.target_success_rate,
            'max_steps': self.max_steps,
            'reward_shaping': self.reward_shaping,
            'complexity_weight': self.complexity_weight
        }

@dataclass 
class PerformanceMetrics:
    """Performance metrics for curriculum progression"""
    success_rate: float
    average_steps: float
    average_reward: float
    convergence_rate: float
    stability_score: float
    episodes_count: int
    
    def is_ready_for_next_stage(self, target_rate: float, min_episodes: int = 100) -> bool:
        """Check if ready to progress to next curriculum stage"""
        return (self.episodes_count >= min_episodes and 
                self.success_rate >= target_rate and 
                self.stability_score > 0.7)

class CurriculumManager:
    """
    Manages curriculum learning progression for hierarchical RL
    
    Features:
    - Automatic difficulty progression based on performance
    - Multiple curriculum stages with increasing complexity
    - Performance tracking and adaptation
    - Task generation and sampling
    - Success criteria evaluation
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        self.config = self._setup_default_config(config)
        
        # Current curriculum state
        self.current_stage = CurriculumStage.BASIC_PLACEMENT
        self.current_task: Optional[CurriculumTask] = None
        
        # Performance tracking
        self.stage_metrics = defaultdict(lambda: defaultdict(list))
        self.episode_history = deque(maxlen=1000)
        self.performance_window = deque(maxlen=100)  # Recent performance
        
        # Stage progression tracking
        self.stage_start_time = {}
        self.stage_episode_counts = defaultdict(int)
        self.progression_history = []
        
        # Initialize curriculum stages
        self.curriculum_stages = self._initialize_curriculum_stages()
        self.current_task = self.curriculum_stages[self.current_stage]
        
        logger.info(f"Curriculum Manager initialized at stage: {self.current_stage.name}")
    
    def _setup_default_config(self, config: Optional[Dict]) -> Dict[str, Any]:
        """Setup default configuration"""
        default = {
            # Progression criteria
            'min_episodes_per_stage': 100,
            'target_success_rates': {
                CurriculumStage.BASIC_PLACEMENT: 0.8,
                CurriculumStage.INTERMEDIATE: 0.7,
                CurriculumStage.ADVANCED: 0.6,
                CurriculumStage.EXPERT: 0.5,
                CurriculumStage.MASTERY: 0.4
            },
            
            # Performance evaluation
            'performance_window_size': 100,
            'stability_threshold': 0.7,
            'convergence_patience': 50,
            
            # Curriculum behavior
            'allow_regression': True,
            'regression_threshold': 0.3,
            'adaptation_rate': 0.1,
            'diversity_bonus': True,
            
            # Reward shaping progression
            'reward_shaping_decay': 0.9,
            'sparse_reward_progression': True
        }
        
        if config:
            default.update(config)
        return default
    
    def _initialize_curriculum_stages(self) -> Dict[CurriculumStage, CurriculumTask]:
        """Initialize all curriculum stages"""
        stages = {}
        
        # Stage 1: Basic Placement (2 simple blocks)
        stages[CurriculumStage.BASIC_PLACEMENT] = CurriculumTask(
            stage=CurriculumStage.BASIC_PLACEMENT,
            num_blocks=2,
            blocks_subset=[SOMABlock.P_BLOCK, SOMABlock.L_BLOCK],  # Simple shapes
            target_success_rate=self.config['target_success_rates'][CurriculumStage.BASIC_PLACEMENT],
            max_steps=20,
            reward_shaping={
                'placement_bonus': 10.0,
                'progress_reward': 1.0,
                'collision_penalty': -2.0,
                'step_penalty': -0.01,
                'completion_bonus': 100.0
            },
            complexity_weight=1.0
        )
        
        # Stage 2: Intermediate (3-4 blocks)
        stages[CurriculumStage.INTERMEDIATE] = CurriculumTask(
            stage=CurriculumStage.INTERMEDIATE,
            num_blocks=3,
            blocks_subset=[SOMABlock.P_BLOCK, SOMABlock.L_BLOCK, SOMABlock.T_BLOCK],
            target_success_rate=self.config['target_success_rates'][CurriculumStage.INTERMEDIATE],
            max_steps=30,
            reward_shaping={
                'placement_bonus': 8.0,
                'progress_reward': 0.8,
                'collision_penalty': -1.5,
                'step_penalty': -0.01,
                'completion_bonus': 100.0
            },
            complexity_weight=1.2
        )
        
        # Stage 3: Advanced (5-6 blocks)
        stages[CurriculumStage.ADVANCED] = CurriculumTask(
            stage=CurriculumStage.ADVANCED,
            num_blocks=5,
            blocks_subset=[SOMABlock.P_BLOCK, SOMABlock.L_BLOCK, SOMABlock.T_BLOCK, 
                          SOMABlock.Z_BLOCK, SOMABlock.A_BLOCK],
            target_success_rate=self.config['target_success_rates'][CurriculumStage.ADVANCED],
            max_steps=40,
            reward_shaping={
                'placement_bonus': 5.0,
                'progress_reward': 0.5,
                'collision_penalty': -1.0,
                'step_penalty': -0.01,
                'completion_bonus': 100.0
            },
            complexity_weight=1.5
        )
        
        # Stage 4: Expert (All 7 blocks)
        stages[CurriculumStage.EXPERT] = CurriculumTask(
            stage=CurriculumStage.EXPERT,
            num_blocks=7,
            blocks_subset=list(SOMABlock),
            target_success_rate=self.config['target_success_rates'][CurriculumStage.EXPERT],
            max_steps=50,
            reward_shaping={
                'placement_bonus': 2.0,
                'progress_reward': 0.2,
                'collision_penalty': -0.5,
                'step_penalty': -0.01,
                'completion_bonus': 100.0
            },
            complexity_weight=2.0
        )
        
        # Stage 5: Mastery (Variations and robustness)
        stages[CurriculumStage.MASTERY] = CurriculumTask(
            stage=CurriculumStage.MASTERY,
            num_blocks=7,
            blocks_subset=list(SOMABlock),
            target_success_rate=self.config['target_success_rates'][CurriculumStage.MASTERY],
            max_steps=60,
            reward_shaping={
                'placement_bonus': 1.0,
                'progress_reward': 0.1,
                'collision_penalty': -0.2,
                'step_penalty': -0.01,
                'completion_bonus': 100.0
            },
            complexity_weight=3.0
        )
        
        return stages
    
    def get_current_task(self) -> CurriculumTask:
        """Get current curriculum task"""
        return self.current_task
    
    def sample_task(self) -> CurriculumTask:
        """Sample a task from current curriculum stage"""
        task = self.current_task
        
        # Add some variation within the stage
        if self.config['diversity_bonus'] and random.random() < 0.2:
            # Sometimes sample from neighboring stages for diversity
            variation_task = self._get_variation_task(task)
            return variation_task
        
        return task
    
    def _get_variation_task(self, base_task: CurriculumTask) -> CurriculumTask:
        """Generate task variation for diversity"""
        # Create variation by adjusting block subset or parameters
        variation = CurriculumTask(
            stage=base_task.stage,
            num_blocks=base_task.num_blocks,
            blocks_subset=base_task.blocks_subset.copy(),
            target_success_rate=base_task.target_success_rate,
            max_steps=base_task.max_steps,
            reward_shaping=base_task.reward_shaping.copy(),
            complexity_weight=base_task.complexity_weight
        )
        
        # Add some randomness
        if len(variation.blocks_subset) < len(SOMABlock):
            # Sometimes add an extra block
            remaining_blocks = [b for b in SOMABlock if b not in variation.blocks_subset]
            if remaining_blocks and random.random() < 0.3:
                extra_block = random.choice(remaining_blocks)
                variation.blocks_subset.append(extra_block)
                variation.num_blocks += 1
        
        # Adjust reward shaping slightly
        for key in variation.reward_shaping:
            if key != 'completion_bonus':
                noise = random.uniform(-0.2, 0.2)
                variation.reward_shaping[key] *= (1.0 + noise)
        
        return variation
    
    def update_performance(self, episode_result: Dict[str, Any]) -> Dict[str, Any]:
        """
        Update performance metrics with episode result
        
        Args:
            episode_result: Dictionary containing:
                - success: bool
                - steps_taken: int
                - total_reward: float
                - convergence_metric: float (optional)
        
        Returns:
            Dictionary with curriculum update information
        """
        
        # Extract metrics from episode result
        success = episode_result.get('success', False)
        steps = episode_result.get('steps_taken', 0)
        reward = episode_result.get('total_reward', 0.0)
        
        # Store episode data
        episode_data = {
            'stage': self.current_stage,
            'success': success,
            'steps': steps,
            'reward': reward,
            'timestamp': time.time()
        }
        
        self.episode_history.append(episode_data)
        self.performance_window.append(episode_data)
        self.stage_episode_counts[self.current_stage] += 1
        
        # Update stage metrics
        self.stage_metrics[self.current_stage]['success_rate'].append(success)
        self.stage_metrics[self.current_stage]['steps'].append(steps)
        self.stage_metrics[self.current_stage]['reward'].append(reward)
        
        # Compute current performance metrics
        current_metrics = self._compute_performance_metrics()
        
        # Check for stage progression
        progression_info = self._check_stage_progression(current_metrics)
        
        return {
            'current_stage': self.current_stage.name,
            'current_metrics': current_metrics,
            'progression_info': progression_info,
            'task_config': self.current_task.to_dict()
        }
    
    def _compute_performance_metrics(self) -> PerformanceMetrics:
        """Compute current performance metrics"""
        if not self.performance_window:
            return PerformanceMetrics(0.0, 0.0, 0.0, 0.0, 0.0, 0)
        
        # Recent performance data
        recent_data = list(self.performance_window)
        
        # Success rate
        successes = [ep['success'] for ep in recent_data]
        success_rate = np.mean(successes)
        
        # Average metrics
        avg_steps = np.mean([ep['steps'] for ep in recent_data])
        avg_reward = np.mean([ep['reward'] for ep in recent_data])
        
        # Convergence rate (improvement over time)
        if len(recent_data) >= 20:
            early_success = np.mean([ep['success'] for ep in recent_data[:20]])
            late_success = np.mean([ep['success'] for ep in recent_data[-20:]])
            convergence_rate = max(0.0, late_success - early_success)
        else:
            convergence_rate = 0.0
        
        # Stability score (consistency of performance)
        if len(successes) >= 10:
            # Rolling average stability
            window_size = min(10, len(successes) // 2)
            rolling_success = []
            for i in range(len(successes) - window_size + 1):
                rolling_success.append(np.mean(successes[i:i + window_size]))
            
            stability_score = 1.0 - np.std(rolling_success)
            stability_score = max(0.0, min(1.0, stability_score))
        else:
            stability_score = 0.0
        
        return PerformanceMetrics(
            success_rate=success_rate,
            average_steps=avg_steps,
            average_reward=avg_reward,
            convergence_rate=convergence_rate,
            stability_score=stability_score,
            episodes_count=len(recent_data)
        )
    
    def _check_stage_progression(self, metrics: PerformanceMetrics) -> Dict[str, Any]:
        """Check if ready to progress to next stage"""
        
        progression_info = {
            'ready_for_next': False,
            'should_regress': False,
            'stage_changed': False,
            'reason': ''
        }
        
        current_target = self.current_task.target_success_rate
        min_episodes = self.config['min_episodes_per_stage']
        
        # Check progression criteria
        ready_for_next = metrics.is_ready_for_next_stage(current_target, min_episodes)
        
        # Check regression criteria (if enabled)
        should_regress = False
        if (self.config['allow_regression'] and 
            metrics.episodes_count >= min_episodes and
            metrics.success_rate < self.config['regression_threshold']):
            should_regress = True
        
        # Execute progression
        if ready_for_next and self.current_stage != CurriculumStage.MASTERY:
            new_stage = self._get_next_stage()
            if new_stage:
                self._progress_to_stage(new_stage)
                progression_info.update({
                    'ready_for_next': True,
                    'stage_changed': True,
                    'reason': f'Advanced to {new_stage.name} (success rate: {metrics.success_rate:.3f})'
                })
                
        elif should_regress and self.current_stage != CurriculumStage.BASIC_PLACEMENT:
            prev_stage = self._get_previous_stage()
            if prev_stage:
                self._progress_to_stage(prev_stage)
                progression_info.update({
                    'should_regress': True,
                    'stage_changed': True,
                    'reason': f'Regressed to {prev_stage.name} (success rate: {metrics.success_rate:.3f})'
                })
        
        return progression_info
    
    def _get_next_stage(self) -> Optional[CurriculumStage]:
        """Get next curriculum stage"""
        stages = list(CurriculumStage)
        current_idx = stages.index(self.current_stage)
        
        if current_idx < len(stages) - 1:
            return stages[current_idx + 1]
        return None
    
    def _get_previous_stage(self) -> Optional[CurriculumStage]:
        """Get previous curriculum stage"""
        stages = list(CurriculumStage)
        current_idx = stages.index(self.current_stage)
        
        if current_idx > 0:
            return stages[current_idx - 1]
        return None
    
    def _progress_to_stage(self, new_stage: CurriculumStage):
        """Progress to a new curriculum stage"""
        old_stage = self.current_stage
        self.current_stage = new_stage
        self.current_task = self.curriculum_stages[new_stage]
        
        # Reset performance window for new stage
        self.performance_window.clear()
        
        # Record progression
        progression_record = {
            'from_stage': old_stage.name,
            'to_stage': new_stage.name,
            'timestamp': time.time(),
            'episode_count': self.stage_episode_counts[old_stage]
        }
        self.progression_history.append(progression_record)
        
        # Apply reward shaping adaptation
        if self.config['sparse_reward_progression']:
            self._adapt_reward_shaping()
        
        logger.info(f"Curriculum progression: {old_stage.name} -> {new_stage.name}")
    
    def _adapt_reward_shaping(self):
        """Adapt reward shaping as curriculum progresses"""
        decay_factor = self.config['reward_shaping_decay']
        
        # Reduce shaped rewards to encourage sparse reward learning
        for key in self.current_task.reward_shaping:
            if key != 'completion_bonus':  # Keep completion bonus constant
                self.current_task.reward_shaping[key] *= decay_factor
    
    def configure_environment(self, env: SOMAEnvironment) -> SOMAEnvironment:
        """Configure environment for current curriculum task"""
        
        # Set curriculum level (number of blocks)
        env.config['curriculum_level'] = self.current_task.num_blocks
        
        # Set max steps
        env.config['max_steps'] = self.current_task.max_steps
        
        # Configure reward shaping
        env.config.update(self.current_task.reward_shaping)
        
        # Set block subset if needed
        env.remaining_blocks = self.current_task.blocks_subset.copy()
        
        # Adjust difficulty based on stage
        if self.current_stage == CurriculumStage.MASTERY:
            # Add extra difficulty for mastery stage
            env.config['domain_randomization'] = True
            env.config['block_tolerance'] = 0.02
        else:
            env.config['domain_randomization'] = False
            env.config['block_tolerance'] = 0.0
        
        return env
    
    def get_curriculum_status(self) -> Dict[str, Any]:
        """Get comprehensive curriculum status"""
        current_metrics = self._compute_performance_metrics()
        
        status = {
            'current_stage': self.current_stage.name,
            'stage_index': self.current_stage.value,
            'total_stages': len(CurriculumStage),
            'current_task': self.current_task.to_dict(),
            'performance_metrics': {
                'success_rate': current_metrics.success_rate,
                'average_steps': current_metrics.average_steps,
                'average_reward': current_metrics.average_reward,
                'stability_score': current_metrics.stability_score,
                'episodes_count': current_metrics.episodes_count
            },
            'progression_history': self.progression_history,
            'stage_episode_counts': dict(self.stage_episode_counts),
            'total_episodes': sum(self.stage_episode_counts.values())
        }
        
        return status
    
    def save_curriculum_state(self, filepath: str):
        """Save curriculum state to file"""
        state = {
            'current_stage': self.current_stage.value,
            'stage_episode_counts': {
                stage.value if hasattr(stage, 'value') else str(stage): count 
                for stage, count in self.stage_episode_counts.items()
            },
            'progression_history': self.progression_history,
            'config': self.config,
            'curriculum_stages': {
                stage.value: task.to_dict() 
                for stage, task in self.curriculum_stages.items()
            }
        }
        
        with open(filepath, 'w') as f:
            json.dump(state, f, indent=2, default=str)
        
        logger.info(f"Curriculum state saved to {filepath}")
    
    def load_curriculum_state(self, filepath: str):
        """Load curriculum state from file"""
        with open(filepath, 'r') as f:
            state = json.load(f)
        
        # Restore state
        self.current_stage = CurriculumStage(state['current_stage'])
        self.stage_episode_counts = defaultdict(int, state['stage_episode_counts'])
        self.progression_history = state['progression_history']
        
        # Update current task
        self.current_task = self.curriculum_stages[self.current_stage]
        
        logger.info(f"Curriculum state loaded from {filepath}")
        logger.info(f"Resumed at stage: {self.current_stage.name}")

class AdaptiveCurriculumManager(CurriculumManager):
    """
    Advanced curriculum manager with adaptive difficulty adjustment
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        super().__init__(config)
        
        # Adaptive parameters
        self.difficulty_adjustment_rate = config.get('difficulty_adjustment_rate', 0.05)
        self.performance_target_window = config.get('performance_target_window', 50)
        self.adaptive_rewards = config.get('adaptive_rewards', True)
        
        # Difficulty history
        self.difficulty_history = deque(maxlen=1000)
        
    def update_performance(self, episode_result: Dict[str, Any]) -> Dict[str, Any]:
        """Update with adaptive difficulty adjustment"""
        
        # Standard curriculum update
        update_info = super().update_performance(episode_result)
        
        # Adaptive difficulty adjustment
        if len(self.performance_window) >= self.performance_target_window:
            self._adapt_difficulty()
        
        return update_info
    
    def _adapt_difficulty(self):
        """Adapt task difficulty based on performance"""
        current_metrics = self._compute_performance_metrics()
        target_success_rate = self.current_task.target_success_rate
        
        # Calculate performance gap
        performance_gap = current_metrics.success_rate - target_success_rate
        
        # Adjust difficulty parameters
        if abs(performance_gap) > 0.1:  # Significant gap
            
            if performance_gap > 0.1:  # Too easy
                # Increase difficulty
                self._increase_difficulty()
                
            elif performance_gap < -0.1:  # Too hard
                # Decrease difficulty
                self._decrease_difficulty()
    
    def _increase_difficulty(self):
        """Increase task difficulty"""
        # Reduce max steps
        self.current_task.max_steps = max(
            self.current_task.max_steps - 2,
            self.current_task.num_blocks * 5  # Minimum reasonable steps
        )
        
        # Reduce reward shaping
        if self.adaptive_rewards:
            for key in self.current_task.reward_shaping:
                if key not in ['completion_bonus', 'step_penalty']:
                    self.current_task.reward_shaping[key] *= 0.95
        
        # Record difficulty change
        self.difficulty_history.append({
            'timestamp': time.time(),
            'stage': self.current_stage.name,
            'adjustment': 'increase',
            'max_steps': self.current_task.max_steps
        })
        
        logger.debug(f"Increased difficulty: max_steps = {self.current_task.max_steps}")
    
    def _decrease_difficulty(self):
        """Decrease task difficulty"""
        # Increase max steps
        self.current_task.max_steps = min(
            self.current_task.max_steps + 2,
            self.current_task.num_blocks * 15  # Maximum reasonable steps
        )
        
        # Increase reward shaping
        if self.adaptive_rewards:
            for key in self.current_task.reward_shaping:
                if key not in ['completion_bonus', 'step_penalty']:
                    self.current_task.reward_shaping[key] *= 1.05
        
        # Record difficulty change
        self.difficulty_history.append({
            'timestamp': time.time(),
            'stage': self.current_stage.name,
            'adjustment': 'decrease',
            'max_steps': self.current_task.max_steps
        })
        
        logger.debug(f"Decreased difficulty: max_steps = {self.current_task.max_steps}")

def create_curriculum_manager(curriculum_type: str = "standard", 
                            config: Dict[str, Any] = None) -> CurriculumManager:
    """
    Factory function to create curriculum manager
    
    Args:
        curriculum_type: "standard" or "adaptive"
        config: Configuration dictionary
        
    Returns:
        CurriculumManager instance
    """
    
    if curriculum_type == "adaptive":
        return AdaptiveCurriculumManager(config)
    else:
        return CurriculumManager(config)
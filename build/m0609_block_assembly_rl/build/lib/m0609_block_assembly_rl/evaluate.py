"""
Evaluation and Monitoring System for M0609 Block Assembly RL

This module provides comprehensive evaluation tools for assessing the performance
of trained RL agents on the Doosan M0609 block assembly task.
"""

import os
import sys
import argparse
import numpy as np
import torch
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from typing import Dict, List, Tuple, Optional
import json
import time
from datetime import datetime
import logging

# Add current directory to path for imports (for standalone execution)
if __name__ == "__main__":
    sys.path.append(os.path.dirname(os.path.abspath(__file__)))
    from environment import M0609BlockAssemblyEnv, BlockType
    from ppo_agent import M0609PPOAgent, PPOConfig
else:
    # Use relative imports for ROS2 package
    from .environment import M0609BlockAssemblyEnv, BlockType
    from .ppo_agent import M0609PPOAgent, PPOConfig

class PerformanceAnalyzer:
    """Comprehensive performance analysis for trained agents"""
    
    def __init__(self, model_path: str, config_path: Optional[str] = None):
        self.model_path = model_path
        self.config_path = config_path
        
        # Load agent
        self.agent = self._load_agent()
        
        # Setup logging
        self.setup_logging()
        
        # Performance metrics storage
        self.evaluation_results = {}
        
    def _load_agent(self) -> M0609PPOAgent:
        """Load trained agent from checkpoint"""
        if not os.path.exists(self.model_path):
            raise FileNotFoundError(f"Model file not found: {self.model_path}")
        
        # Create agent with default config
        ppo_config = PPOConfig()
        agent = M0609PPOAgent(ppo_config)
        
        # Load model weights
        agent.load_model(self.model_path)
        
        # Set to evaluation mode
        agent.strategy_net.eval()
        agent.grasp_net.eval()
        agent.value_net.eval()
        
        return agent
    
    def setup_logging(self):
        """Setup logging for evaluation"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = os.path.join("evaluation_logs", f"eval_{timestamp}")
        os.makedirs(log_dir, exist_ok=True)
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(os.path.join(log_dir, 'evaluation.log')),
                logging.StreamHandler()
            ]
        )
        
        self.log_dir = log_dir
        self.logger = logging.getLogger(__name__)
    
    def comprehensive_evaluation(self, num_episodes: int = 100, 
                               robot_id: str = "dsr01", 
                               robot_model: str = "m0609",
                               virtual_mode: bool = True) -> Dict:
        """Perform comprehensive evaluation of the trained agent"""
        
        self.logger.info(f"Starting comprehensive evaluation with {num_episodes} episodes")
        
        # Initialize environment
        env = M0609BlockAssemblyEnv(
            robot_id=robot_id,
            robot_model=robot_model, 
            virtual_mode=virtual_mode
        )
        
        # Evaluation metrics storage
        episode_data = []
        detailed_metrics = {
            'assembly_times': [],
            'success_rates': [],
            'episode_rewards': [],
            'motion_efficiencies': [],
            'block_sequence_choices': [],
            'grasp_qualities': [],
            'collision_incidents': [],
            'strategy_distributions': {
                'direct': 0, 'detour': 0, 'optimal': 0
            },
            'block_placement_times': {i: [] for i in range(7)},
            'step_by_step_analysis': []
        }
        
        successful_episodes = 0
        total_assembly_time = 0
        
        for episode in range(num_episodes):
            episode_start_time = time.time()
            
            # Run single episode
            episode_result = self._run_evaluation_episode(env, episode)
            
            episode_data.append(episode_result)
            
            # Update detailed metrics
            detailed_metrics['assembly_times'].append(episode_result['total_time'])
            detailed_metrics['success_rates'].append(episode_result['success'])
            detailed_metrics['episode_rewards'].append(episode_result['total_reward'])
            detailed_metrics['motion_efficiencies'].append(episode_result['avg_motion_efficiency'])
            detailed_metrics['block_sequence_choices'].append(episode_result['block_sequence'])
            
            # Update strategy statistics
            for strategy in episode_result['strategies_used']:
                if strategy == 0:
                    detailed_metrics['strategy_distributions']['direct'] += 1
                elif strategy == 1:
                    detailed_metrics['strategy_distributions']['detour'] += 1
                elif strategy == 2:
                    detailed_metrics['strategy_distributions']['optimal'] += 1
            
            # Update block-specific timing
            for block_id, timing in episode_result['block_timings'].items():
                detailed_metrics['block_placement_times'][block_id].append(timing)
            
            detailed_metrics['step_by_step_analysis'].append(episode_result['step_analysis'])
            
            if episode_result['success']:
                successful_episodes += 1
                total_assembly_time += episode_result['total_time']
            
            # Progress logging
            if (episode + 1) % 10 == 0:
                current_success_rate = successful_episodes / (episode + 1)
                avg_time = total_assembly_time / max(1, successful_episodes)
                self.logger.info(f"Episode {episode + 1}/{num_episodes} - "
                               f"Success Rate: {current_success_rate:.3f}, "
                               f"Avg Time: {avg_time:.1f}s")
        
        env.close()
        
        # Calculate summary statistics
        summary_stats = self._calculate_summary_statistics(episode_data, detailed_metrics)
        
        # Store results
        self.evaluation_results = {
            'summary': summary_stats,
            'detailed_metrics': detailed_metrics,
            'episode_data': episode_data,
            'model_path': self.model_path,
            'evaluation_date': datetime.now().isoformat()
        }
        
        # Generate comprehensive report
        self._generate_evaluation_report()
        
        return self.evaluation_results
    
    def _run_evaluation_episode(self, env: M0609BlockAssemblyEnv, episode_id: int) -> Dict:
        """Run a single evaluation episode with detailed tracking"""
        
        obs, info = env.reset()
        
        episode_result = {
            'episode_id': episode_id,
            'total_reward': 0,
            'total_time': 0,
            'success': False,
            'blocks_placed': 0,
            'avg_motion_efficiency': 0,
            'block_sequence': [],
            'strategies_used': [],
            'block_timings': {},
            'step_analysis': [],
            'collision_count': 0,
            'failed_attempts': 0
        }
        
        motion_efficiencies = []
        step_start_time = time.time()
        
        for step in range(7):  # Maximum 7 blocks
            step_start = time.time()
            
            # Get available blocks
            available_blocks = (~env.blocks_placed).astype(np.float32)
            
            if np.sum(available_blocks) == 0:
                break  # All blocks placed
            
            # Select action (deterministic for evaluation)
            action, log_prob, value = self.agent.select_action(
                obs, available_blocks, deterministic=True
            )
            
            # Record strategy choice
            episode_result['strategies_used'].append(action['approach_strategy'])
            episode_result['block_sequence'].append(action['next_block'])
            
            # Execute action
            next_obs, reward, terminated, truncated, step_info = env.step(action)
            
            step_time = time.time() - step_start
            
            # Record step analysis
            step_analysis = {
                'step': step,
                'selected_block': action['next_block'],
                'strategy': action['approach_strategy'],
                'grasp_position': action['grasp_position'].tolist(),
                'grasp_orientation': action['grasp_orientation'].tolist(),
                'reward': reward,
                'step_time': step_time,
                'motion_efficiency': step_info.get('motion_efficiency', 0),
                'success': reward > 0
            }
            
            episode_result['step_analysis'].append(step_analysis)
            episode_result['total_reward'] += reward
            
            # Track motion efficiency
            if 'motion_efficiency' in step_info:
                motion_efficiencies.append(step_info['motion_efficiency'])
            
            # Record block timing
            if reward > 0:  # Successful placement
                episode_result['block_timings'][action['next_block']] = step_time
                episode_result['blocks_placed'] += 1
            else:
                episode_result['failed_attempts'] += 1
            
            obs = next_obs
            
            if terminated or truncated:
                break
        
        # Calculate final metrics
        episode_result['total_time'] = time.time() - step_start_time
        episode_result['success'] = np.sum(env.blocks_placed) == 7
        episode_result['avg_motion_efficiency'] = np.mean(motion_efficiencies) if motion_efficiencies else 0
        
        return episode_result
    
    def _calculate_summary_statistics(self, episode_data: List[Dict], 
                                    detailed_metrics: Dict) -> Dict:
        """Calculate comprehensive summary statistics"""
        
        successful_episodes = [ep for ep in episode_data if ep['success']]
        
        summary = {
            'total_episodes': len(episode_data),
            'successful_episodes': len(successful_episodes),
            'success_rate': len(successful_episodes) / len(episode_data),
            
            # Time performance
            'avg_assembly_time': np.mean([ep['total_time'] for ep in successful_episodes]) if successful_episodes else 0,
            'std_assembly_time': np.std([ep['total_time'] for ep in successful_episodes]) if successful_episodes else 0,
            'min_assembly_time': np.min([ep['total_time'] for ep in successful_episodes]) if successful_episodes else 0,
            'max_assembly_time': np.max([ep['total_time'] for ep in successful_episodes]) if successful_episodes else 0,
            
            # Reward performance
            'avg_episode_reward': np.mean([ep['total_reward'] for ep in episode_data]),
            'std_episode_reward': np.std([ep['total_reward'] for ep in episode_data]),
            
            # Motion efficiency
            'avg_motion_efficiency': np.mean(detailed_metrics['motion_efficiencies']),
            'std_motion_efficiency': np.std(detailed_metrics['motion_efficiencies']),
            
            # Strategy analysis
            'strategy_preferences': detailed_metrics['strategy_distributions'],
            
            # Block-specific performance
            'avg_block_times': {},
            'block_success_rates': {},
            
            # Performance benchmarks
            'target_time_achievement': 0,  # Percentage achieving < 5 min
            'expert_level_achievement': 0,  # Percentage achieving < 4 min
            'baseline_improvement': 0  # Improvement over 10 min baseline
        }
        
        # Calculate block-specific metrics
        for block_id in range(7):
            if detailed_metrics['block_placement_times'][block_id]:
                summary['avg_block_times'][block_id] = np.mean(detailed_metrics['block_placement_times'][block_id])
            
            # Calculate block success rate
            total_attempts = sum(1 for ep in episode_data if block_id in ep['block_sequence'])
            successful_attempts = sum(1 for ep in episode_data if block_id in ep['block_timings'])
            summary['block_success_rates'][block_id] = successful_attempts / max(1, total_attempts)
        
        # Calculate performance benchmarks
        if successful_episodes:
            times = [ep['total_time'] for ep in successful_episodes]
            summary['target_time_achievement'] = sum(1 for t in times if t <= 300) / len(times)
            summary['expert_level_achievement'] = sum(1 for t in times if t <= 240) / len(times)
            
            avg_time = summary['avg_assembly_time']
            baseline_time = 600  # 10 minutes
            if avg_time > 0:
                summary['baseline_improvement'] = (baseline_time - avg_time) / baseline_time * 100
        
        return summary
    
    def _generate_evaluation_report(self):
        """Generate comprehensive evaluation report with visualizations"""
        
        # Create visualizations
        self._create_performance_visualizations()
        
        # Generate text report
        report_path = os.path.join(self.log_dir, 'evaluation_report.txt')
        with open(report_path, 'w') as f:
            f.write("M0609 BLOCK ASSEMBLY RL AGENT EVALUATION REPORT\n")
            f.write("=" * 60 + "\n\n")
            
            # Model information
            f.write(f"Model Path: {self.model_path}\n")
            f.write(f"Evaluation Date: {self.evaluation_results['evaluation_date']}\n\n")
            
            # Summary statistics
            summary = self.evaluation_results['summary']
            f.write("PERFORMANCE SUMMARY\n")
            f.write("-" * 30 + "\n")
            f.write(f"Total Episodes: {summary['total_episodes']}\n")
            f.write(f"Success Rate: {summary['success_rate']:.3f} ({summary['successful_episodes']}/{summary['total_episodes']})\n\n")
            
            if summary['successful_episodes'] > 0:
                f.write("TIMING PERFORMANCE\n")
                f.write("-" * 30 + "\n")
                f.write(f"Average Assembly Time: {summary['avg_assembly_time']:.1f} ± {summary['std_assembly_time']:.1f} seconds\n")
                f.write(f"Best Time: {summary['min_assembly_time']:.1f} seconds\n")
                f.write(f"Worst Time: {summary['max_assembly_time']:.1f} seconds\n")
                f.write(f"Target Achievement (≤5min): {summary['target_time_achievement']:.1%}\n")
                f.write(f"Expert Level (≤4min): {summary['expert_level_achievement']:.1%}\n")
                f.write(f"Improvement over Baseline: {summary['baseline_improvement']:.1f}%\n\n")
            
            # Strategy analysis
            f.write("STRATEGY ANALYSIS\n")
            f.write("-" * 30 + "\n")
            total_strategies = sum(summary['strategy_preferences'].values())
            for strategy, count in summary['strategy_preferences'].items():
                percentage = count / max(1, total_strategies) * 100
                f.write(f"{strategy.capitalize()} Strategy: {percentage:.1f}% ({count} times)\n")
            f.write("\n")
            
            # Motion efficiency
            f.write("MOTION EFFICIENCY\n")
            f.write("-" * 30 + "\n")
            f.write(f"Average Motion Efficiency: {summary['avg_motion_efficiency']:.3f} ± {summary['std_motion_efficiency']:.3f}\n\n")
            
            # Block-specific analysis
            f.write("BLOCK-SPECIFIC PERFORMANCE\n")
            f.write("-" * 30 + "\n")
            for block_id in range(7):
                block_name = list(BlockType.__dict__.keys())[block_id]
                avg_time = summary['avg_block_times'].get(block_id, 0)
                success_rate = summary['block_success_rates'].get(block_id, 0)
                f.write(f"Block {block_id} ({block_name}): ")
                f.write(f"Avg Time: {avg_time:.1f}s, Success Rate: {success_rate:.3f}\n")
            f.write("\n")
            
            # Performance assessment
            f.write("PERFORMANCE ASSESSMENT\n")
            f.write("-" * 30 + "\n")
            if summary['success_rate'] >= 0.95:
                f.write("✓ SUCCESS RATE: Excellent (≥95%)\n")
            elif summary['success_rate'] >= 0.85:
                f.write("✓ SUCCESS RATE: Good (≥85%)\n")
            elif summary['success_rate'] >= 0.70:
                f.write("⚠ SUCCESS RATE: Fair (≥70%)\n")
            else:
                f.write("✗ SUCCESS RATE: Poor (<70%)\n")
            
            if summary['avg_assembly_time'] <= 300:
                f.write("✓ ASSEMBLY TIME: Target Achieved (≤5 min)\n")
            elif summary['avg_assembly_time'] <= 450:
                f.write("⚠ ASSEMBLY TIME: Good Progress (≤7.5 min)\n")
            else:
                f.write("✗ ASSEMBLY TIME: Needs Improvement (>7.5 min)\n")
            
            if summary['baseline_improvement'] >= 50:
                f.write("✓ IMPROVEMENT: Target Achieved (≥50%)\n")
            elif summary['baseline_improvement'] >= 25:
                f.write("⚠ IMPROVEMENT: Good Progress (≥25%)\n")
            else:
                f.write("✗ IMPROVEMENT: Below Expectations (<25%)\n")
        
        # Save detailed results as JSON
        results_path = os.path.join(self.log_dir, 'detailed_results.json')
        with open(results_path, 'w') as f:
            json.dump(self.evaluation_results, f, indent=2, default=str)
        
        self.logger.info(f"Evaluation report saved to {report_path}")
        self.logger.info(f"Detailed results saved to {results_path}")
    
    def _create_performance_visualizations(self):
        """Create comprehensive performance visualizations"""
        
        # Set up matplotlib style
        plt.style.use('seaborn-v0_8')
        
        # 1. Overall Performance Dashboard
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        
        episode_data = self.evaluation_results['episode_data']
        summary = self.evaluation_results['summary']
        
        # Assembly time distribution
        successful_times = [ep['total_time'] for ep in episode_data if ep['success']]
        if successful_times:
            axes[0, 0].hist(successful_times, bins=20, alpha=0.7, color='skyblue', edgecolor='black')
            axes[0, 0].axvline(x=300, color='red', linestyle='--', label='Target (5min)')
            axes[0, 0].axvline(x=np.mean(successful_times), color='green', linestyle='-', label='Average')
            axes[0, 0].set_xlabel('Assembly Time (seconds)')
            axes[0, 0].set_ylabel('Frequency')
            axes[0, 0].set_title('Assembly Time Distribution')
            axes[0, 0].legend()
        
        # Success rate over episodes
        success_rates = []
        window_size = 10
        for i in range(len(episode_data)):
            start_idx = max(0, i - window_size + 1)
            window_successes = [ep['success'] for ep in episode_data[start_idx:i+1]]
            success_rates.append(np.mean(window_successes))
        
        axes[0, 1].plot(success_rates, color='blue', alpha=0.7)
        axes[0, 1].axhline(y=0.95, color='red', linestyle='--', label='Target (95%)')
        axes[0, 1].set_xlabel('Episode')
        axes[0, 1].set_ylabel('Success Rate (Moving Average)')
        axes[0, 1].set_title('Success Rate Progression')
        axes[0, 1].legend()
        
        # Strategy preference pie chart
        strategy_data = summary['strategy_preferences']
        strategy_labels = ['Direct', 'Detour', 'Optimal']
        strategy_values = [strategy_data['direct'], strategy_data['detour'], strategy_data['optimal']]
        
        if sum(strategy_values) > 0:
            axes[0, 2].pie(strategy_values, labels=strategy_labels, autopct='%1.1f%%')
            axes[0, 2].set_title('Strategy Preferences')
        
        # Motion efficiency distribution
        efficiencies = [ep['avg_motion_efficiency'] for ep in episode_data]
        axes[1, 0].hist(efficiencies, bins=20, alpha=0.7, color='lightgreen', edgecolor='black')
        axes[1, 0].set_xlabel('Motion Efficiency')
        axes[1, 0].set_ylabel('Frequency')
        axes[1, 0].set_title('Motion Efficiency Distribution')
        
        # Block-specific performance
        block_names = ['L-Block', 'Rect-Block', 'T-Block', 'Step-Block', 'Cross-Block', 'Pillar-Block', 'Triangle-Block']
        block_times = [summary['avg_block_times'].get(i, 0) for i in range(7)]
        block_success = [summary['block_success_rates'].get(i, 0) for i in range(7)]
        
        x_pos = np.arange(len(block_names))
        axes[1, 1].bar(x_pos, block_times, alpha=0.7, color='coral')
        axes[1, 1].set_xlabel('Block Type')
        axes[1, 1].set_ylabel('Average Placement Time (s)')
        axes[1, 1].set_title('Block-Specific Timing')
        axes[1, 1].set_xticks(x_pos)
        axes[1, 1].set_xticklabels(block_names, rotation=45, ha='right')
        
        # Block success rates
        axes[1, 2].bar(x_pos, block_success, alpha=0.7, color='lightcoral')
        axes[1, 2].set_xlabel('Block Type')
        axes[1, 2].set_ylabel('Success Rate')
        axes[1, 2].set_title('Block-Specific Success Rates')
        axes[1, 2].set_xticks(x_pos)
        axes[1, 2].set_xticklabels(block_names, rotation=45, ha='right')
        axes[1, 2].set_ylim(0, 1.1)
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.log_dir, 'performance_dashboard.png'), dpi=300, bbox_inches='tight')
        plt.close()
        
        # 2. Detailed temporal analysis
        self._create_temporal_analysis_plots(episode_data)
        
        # 3. Strategy analysis plots
        self._create_strategy_analysis_plots(episode_data)
        
        self.logger.info("Performance visualizations created successfully")
    
    def _create_temporal_analysis_plots(self, episode_data: List[Dict]):
        """Create detailed temporal analysis plots"""
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Time per step analysis
        step_times = []
        for ep in episode_data:
            if ep['success']:
                ep_step_times = [step['step_time'] for step in ep['step_analysis']]
                step_times.append(ep_step_times)
        
        if step_times:
            # Convert to DataFrame for easier plotting
            max_steps = max(len(times) for times in step_times)
            step_df = pd.DataFrame(step_times).fillna(0)
            step_means = step_df.mean()
            step_stds = step_df.std()
            
            axes[0, 0].errorbar(range(len(step_means)), step_means, yerr=step_stds, 
                               marker='o', capsize=5, color='blue', alpha=0.7)
            axes[0, 0].set_xlabel('Assembly Step')
            axes[0, 0].set_ylabel('Time (seconds)')
            axes[0, 0].set_title('Time per Assembly Step')
            axes[0, 0].grid(True, alpha=0.3)
        
        # Reward progression
        successful_rewards = []
        for ep in episode_data:
            if ep['success']:
                ep_rewards = []
                cumulative_reward = 0
                for step in ep['step_analysis']:
                    cumulative_reward += step['reward']
                    ep_rewards.append(cumulative_reward)
                successful_rewards.append(ep_rewards)
        
        if successful_rewards:
            reward_df = pd.DataFrame(successful_rewards).fillna(method='ffill', axis=1)
            reward_means = reward_df.mean()
            reward_stds = reward_df.std()
            
            axes[0, 1].errorbar(range(len(reward_means)), reward_means, yerr=reward_stds,
                               marker='s', capsize=5, color='green', alpha=0.7)
            axes[0, 1].set_xlabel('Assembly Step')
            axes[0, 1].set_ylabel('Cumulative Reward')
            axes[0, 1].set_title('Reward Accumulation')
            axes[0, 1].grid(True, alpha=0.3)
        
        # Learning curve (if multiple evaluation runs)
        assembly_times = [ep['total_time'] for ep in episode_data if ep['success']]
        if len(assembly_times) > 10:
            # Moving average
            window = max(5, len(assembly_times) // 20)
            moving_avg = pd.Series(assembly_times).rolling(window=window).mean()
            
            axes[1, 0].plot(assembly_times, alpha=0.3, color='gray', label='Individual Episodes')
            axes[1, 0].plot(moving_avg, color='red', linewidth=2, label=f'Moving Average (window={window})')
            axes[1, 0].axhline(y=300, color='blue', linestyle='--', label='Target (5min)')
            axes[1, 0].set_xlabel('Episode (Successful Only)')
            axes[1, 0].set_ylabel('Assembly Time (seconds)')
            axes[1, 0].set_title('Performance Consistency')
            axes[1, 0].legend()
            axes[1, 0].grid(True, alpha=0.3)
        
        # Block sequence analysis
        sequences = [ep['block_sequence'] for ep in episode_data if ep['success']]
        if sequences:
            # Most common first choices
            first_choices = [seq[0] if seq else -1 for seq in sequences]
            first_choice_counts = pd.Series(first_choices).value_counts()
            
            axes[1, 1].bar(range(len(first_choice_counts)), first_choice_counts.values, 
                          alpha=0.7, color='purple')
            axes[1, 1].set_xlabel('First Block Choice')
            axes[1, 1].set_ylabel('Frequency')
            axes[1, 1].set_title('First Block Selection Frequency')
            axes[1, 1].set_xticks(range(len(first_choice_counts)))
            axes[1, 1].set_xticklabels([f'Block {x}' for x in first_choice_counts.index])
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.log_dir, 'temporal_analysis.png'), dpi=300, bbox_inches='tight')
        plt.close()
    
    def _create_strategy_analysis_plots(self, episode_data: List[Dict]):
        """Create strategy analysis visualizations"""
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        
        # Strategy effectiveness
        strategy_performance = {'direct': [], 'detour': [], 'optimal': []}
        strategy_map = {0: 'direct', 1: 'detour', 2: 'optimal'}
        
        for ep in episode_data:
            for step in ep['step_analysis']:
                strategy_name = strategy_map[step['strategy']]
                strategy_performance[strategy_name].append({
                    'success': step['success'],
                    'efficiency': step['motion_efficiency'],
                    'time': step['step_time']
                })
        
        # Strategy success rates
        strategy_success_rates = {}
        for strategy, performances in strategy_performance.items():
            if performances:
                success_rate = sum(p['success'] for p in performances) / len(performances)
                strategy_success_rates[strategy] = success_rate
        
        if strategy_success_rates:
            axes[0, 0].bar(strategy_success_rates.keys(), strategy_success_rates.values(), 
                          alpha=0.7, color=['blue', 'orange', 'green'])
            axes[0, 0].set_ylabel('Success Rate')
            axes[0, 0].set_title('Strategy Success Rates')
            axes[0, 0].set_ylim(0, 1.1)
        
        # Strategy efficiency comparison
        strategy_efficiencies = {}
        for strategy, performances in strategy_performance.items():
            if performances:
                efficiencies = [p['efficiency'] for p in performances]
                strategy_efficiencies[strategy] = efficiencies
        
        if strategy_efficiencies:
            axes[0, 1].boxplot(strategy_efficiencies.values(), labels=strategy_efficiencies.keys())
            axes[0, 1].set_ylabel('Motion Efficiency')
            axes[0, 1].set_title('Strategy Motion Efficiency Comparison')
        
        # Strategy timing comparison
        strategy_times = {}
        for strategy, performances in strategy_performance.items():
            if performances:
                times = [p['time'] for p in performances if p['success']]
                strategy_times[strategy] = times
        
        if strategy_times:
            axes[1, 0].boxplot(strategy_times.values(), labels=strategy_times.keys())
            axes[1, 0].set_ylabel('Execution Time (seconds)')
            axes[1, 0].set_title('Strategy Execution Time Comparison')
        
        # Block-strategy combination heatmap
        block_strategy_matrix = np.zeros((7, 3))  # 7 blocks × 3 strategies
        
        for ep in episode_data:
            for step in ep['step_analysis']:
                block_id = step['selected_block']
                strategy_id = step['strategy']
                if 0 <= block_id < 7 and 0 <= strategy_id < 3:
                    block_strategy_matrix[block_id, strategy_id] += 1
        
        # Normalize by row (block) to get percentages
        row_sums = block_strategy_matrix.sum(axis=1, keepdims=True)
        normalized_matrix = np.divide(block_strategy_matrix, row_sums, 
                                    out=np.zeros_like(block_strategy_matrix), 
                                    where=row_sums!=0)
        
        im = axes[1, 1].imshow(normalized_matrix, aspect='auto', cmap='YlOrRd')
        axes[1, 1].set_xticks(range(3))
        axes[1, 1].set_xticklabels(['Direct', 'Detour', 'Optimal'])
        axes[1, 1].set_yticks(range(7))
        axes[1, 1].set_yticklabels([f'Block {i}' for i in range(7)])
        axes[1, 1].set_title('Block-Strategy Usage Heatmap')
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=axes[1, 1])
        cbar.set_label('Usage Frequency')
        
        plt.tight_layout()
        plt.savefig(os.path.join(self.log_dir, 'strategy_analysis.png'), dpi=300, bbox_inches='tight')
        plt.close()

def benchmark_comparison(model_path: str, baseline_time: float = 600, 
                        target_time: float = 300, num_episodes: int = 100):
    """Compare agent performance against benchmarks"""
    
    analyzer = PerformanceAnalyzer(model_path)
    results = analyzer.comprehensive_evaluation(num_episodes=num_episodes)
    
    summary = results['summary']
    
    print("\n" + "="*60)
    print("M0609 BLOCK ASSEMBLY BENCHMARK COMPARISON")
    print("="*60)
    
    print(f"\nModel: {os.path.basename(model_path)}")
    print(f"Episodes: {num_episodes}")
    print(f"Success Rate: {summary['success_rate']:.1%}")
    
    if summary['successful_episodes'] > 0:
        avg_time = summary['avg_assembly_time']
        
        print(f"\n{'Metric':<25} {'Value':<15} {'Target':<15} {'Status'}")
        print("-" * 65)
        
        # Time performance
        print(f"{'Average Time':<25} {avg_time:.1f}s{'':<6} {target_time:.1f}s{'':<6} {'✓' if avg_time <= target_time else '✗'}")
        print(f"{'Best Time':<25} {summary['min_assembly_time']:.1f}s{'':<6} {target_time:.1f}s{'':<6} {'✓' if summary['min_assembly_time'] <= target_time else '✗'}")
        
        # Improvement calculation
        improvement = (baseline_time - avg_time) / baseline_time * 100
        print(f"{'Improvement':<25} {improvement:.1f}%{'':<6} 50.0%{'':<9} {'✓' if improvement >= 50 else '✗'}")
        
        # Success rate
        print(f"{'Success Rate':<25} {summary['success_rate']:.1%}{'':<6} 95.0%{'':<9} {'✓' if summary['success_rate'] >= 0.95 else '✗'}")
        
        # Target achievement rates
        print(f"{'Target Achievement':<25} {summary['target_time_achievement']:.1%}{'':<6} 80.0%{'':<9} {'✓' if summary['target_time_achievement'] >= 0.8 else '✗'}")
        
        print(f"\n{'Performance Classification:'}")
        if avg_time <= 240 and summary['success_rate'] >= 0.95:
            print("🏆 EXPERT LEVEL - Exceeds human expert performance")
        elif avg_time <= 300 and summary['success_rate'] >= 0.90:
            print("🎯 TARGET ACHIEVED - Meets project objectives")
        elif avg_time <= 450 and summary['success_rate'] >= 0.75:
            print("📈 GOOD PROGRESS - Significant improvement shown")
        else:
            print("⚠️  NEEDS IMPROVEMENT - Below target performance")
    
    else:
        print("\n❌ No successful episodes - model requires further training")
    
    return results

def main():
    """Main evaluation function"""
    parser = argparse.ArgumentParser(description="Evaluate M0609 Block Assembly RL Agent")
    
    parser.add_argument("--model-path", type=str, required=True,
                       help="Path to trained model file")
    parser.add_argument("--episodes", type=int, default=100,
                       help="Number of evaluation episodes")
    parser.add_argument("--robot-id", type=str, default="dsr01",
                       help="Robot ID")
    parser.add_argument("--robot-model", type=str, default="m0609", 
                       help="Robot model")
    parser.add_argument("--virtual", action="store_true", default=True,
                       help="Use virtual mode (simulation)")
    parser.add_argument("--benchmark", action="store_true",
                       help="Run benchmark comparison")
    
    args = parser.parse_args()
    
    if not os.path.exists(args.model_path):
        print(f"Error: Model file not found: {args.model_path}")
        return
    
    try:
        if args.benchmark:
            # Run benchmark comparison
            benchmark_comparison(args.model_path, num_episodes=args.episodes)
        else:
            # Run comprehensive evaluation
            analyzer = PerformanceAnalyzer(args.model_path)
            results = analyzer.comprehensive_evaluation(
                num_episodes=args.episodes,
                robot_id=args.robot_id,
                robot_model=args.robot_model,
                virtual_mode=args.virtual
            )
            
            print(f"\nEvaluation completed!")
            print(f"Results saved to: {analyzer.log_dir}")
            print(f"Success Rate: {results['summary']['success_rate']:.1%}")
            
            if results['summary']['successful_episodes'] > 0:
                print(f"Average Assembly Time: {results['summary']['avg_assembly_time']:.1f}s")
                print(f"Improvement: {results['summary']['baseline_improvement']:.1f}%")
    
    except Exception as e:
        logging.error(f"Evaluation failed: {e}", exc_info=True)
        raise

if __name__ == "__main__":
    main()
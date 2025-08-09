"""
Visualization Utilities for Hierarchical SOMA Cube Assembly

This module provides comprehensive visualization tools for:
1. Training progress and learning curves
2. Assembly sequences and 3D cube visualization
3. Curriculum learning progression
4. Domain randomization effects
5. Performance analysis and debugging
"""

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import cv2
from typing import Dict, List, Tuple, Optional, Any, Union
from pathlib import Path
import pandas as pd
from collections import defaultdict
import logging

# Set up plotting style
plt.style.use('seaborn-v0_8')
sns.set_palette("husl")

logger = logging.getLogger(__name__)

class TrainingVisualizer:
    """
    Comprehensive visualization for training progress and analysis
    
    Features:
    - Learning curves and performance metrics
    - Curriculum progression visualization
    - Assembly sequence analysis
    - 3D cube state visualization
    - Comparative analysis between runs
    """
    
    def __init__(self, output_dir: str = "visualizations"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Color schemes
        self.colors = {
            'success': '#2ecc71',
            'failure': '#e74c3c', 
            'reward': '#3498db',
            'steps': '#f39c12',
            'curriculum': '#9b59b6',
            'upper_level': '#1abc9c',
            'lower_level': '#e67e22'
        }
        
        logger.info(f"Visualization output directory: {self.output_dir}")
    
    def plot_learning_curves(self, training_data: List[Dict], 
                           eval_data: Optional[List[Dict]] = None,
                           save_path: Optional[str] = None) -> plt.Figure:
        """Plot comprehensive learning curves"""
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('Hierarchical SOMA Assembly - Learning Curves', fontsize=16)
        
        # Convert to DataFrame for easier plotting
        df = pd.DataFrame(training_data)
        
        # Success rate over time
        window_size = 100
        success_rate = df['success'].rolling(window=window_size, min_periods=1).mean()
        axes[0, 0].plot(df['episode'], success_rate, 
                       color=self.colors['success'], linewidth=2)
        axes[0, 0].set_title('Success Rate (Rolling Average)')
        axes[0, 0].set_xlabel('Episode')
        axes[0, 0].set_ylabel('Success Rate')
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].set_ylim(0, 1)
        
        # Reward progression
        reward_ma = df['total_reward'].rolling(window=window_size, min_periods=1).mean()
        axes[0, 1].plot(df['episode'], reward_ma, 
                       color=self.colors['reward'], linewidth=2)
        axes[0, 1].set_title('Average Reward (Rolling Average)')
        axes[0, 1].set_xlabel('Episode')
        axes[0, 1].set_ylabel('Reward')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Steps taken progression
        steps_ma = df['steps_taken'].rolling(window=window_size, min_periods=1).mean()
        axes[1, 0].plot(df['episode'], steps_ma, 
                       color=self.colors['steps'], linewidth=2)
        axes[1, 0].set_title('Average Steps Taken')
        axes[1, 0].set_xlabel('Episode')
        axes[1, 0].set_ylabel('Steps')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Curriculum progression
        if 'curriculum_stage' in df.columns:
            # Map stages to numeric values for plotting
            stage_mapping = {stage: i for i, stage in enumerate(df['curriculum_stage'].unique())}
            df['stage_numeric'] = df['curriculum_stage'].map(stage_mapping)
            
            axes[1, 1].plot(df['episode'], df['stage_numeric'], 
                          color=self.colors['curriculum'], linewidth=2, marker='o', markersize=2)
            axes[1, 1].set_title('Curriculum Progression')
            axes[1, 1].set_xlabel('Episode')
            axes[1, 1].set_ylabel('Curriculum Stage')
            axes[1, 1].set_yticks(list(stage_mapping.values()))
            axes[1, 1].set_yticklabels(list(stage_mapping.keys()), rotation=45)
            axes[1, 1].grid(True, alpha=0.3)
        
        # Add evaluation data if provided
        if eval_data:
            eval_df = pd.DataFrame(eval_data)
            axes[0, 0].scatter(eval_df['episode'], eval_df['success_rate'], 
                             color='red', s=30, alpha=0.7, label='Evaluation', zorder=5)
            axes[0, 1].scatter(eval_df['episode'], eval_df['average_reward'],
                             color='red', s=30, alpha=0.7, label='Evaluation', zorder=5)
            axes[0, 0].legend()
            axes[0, 1].legend()
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_curriculum_analysis(self, training_data: List[Dict], 
                               save_path: Optional[str] = None) -> plt.Figure:
        """Analyze curriculum learning progression"""
        
        df = pd.DataFrame(training_data)
        
        if 'curriculum_stage' not in df.columns:
            logger.warning("No curriculum stage data available")
            return None
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Curriculum Learning Analysis', fontsize=16)
        
        # Performance by curriculum stage
        stage_performance = df.groupby('curriculum_stage').agg({
            'success': 'mean',
            'total_reward': 'mean',
            'steps_taken': 'mean',
            'episode': 'count'
        }).round(3)
        
        # Success rate by stage
        stages = stage_performance.index
        success_rates = stage_performance['success']
        axes[0, 0].bar(stages, success_rates, color=self.colors['success'], alpha=0.7)
        axes[0, 0].set_title('Success Rate by Curriculum Stage')
        axes[0, 0].set_xlabel('Stage')
        axes[0, 0].set_ylabel('Success Rate')
        axes[0, 0].set_ylim(0, 1)
        plt.setp(axes[0, 0].xaxis.get_majorticklabels(), rotation=45)
        
        # Average reward by stage
        avg_rewards = stage_performance['total_reward']
        axes[0, 1].bar(stages, avg_rewards, color=self.colors['reward'], alpha=0.7)
        axes[0, 1].set_title('Average Reward by Stage')
        axes[0, 1].set_xlabel('Stage')
        axes[0, 1].set_ylabel('Reward')
        plt.setp(axes[0, 1].xaxis.get_majorticklabels(), rotation=45)
        
        # Episode count by stage
        episode_counts = stage_performance['episode']
        axes[1, 0].bar(stages, episode_counts, color=self.colors['curriculum'], alpha=0.7)
        axes[1, 0].set_title('Episodes per Stage')
        axes[1, 0].set_xlabel('Stage')
        axes[1, 0].set_ylabel('Episode Count')
        plt.setp(axes[1, 0].xaxis.get_majorticklabels(), rotation=45)
        
        # Learning progression within stages
        for stage in stages:
            stage_data = df[df['curriculum_stage'] == stage]
            if len(stage_data) > 10:  # Only plot if sufficient data
                stage_episodes = stage_data['episode'] - stage_data['episode'].min()
                success_ma = stage_data['success'].rolling(window=20, min_periods=1).mean()
                axes[1, 1].plot(stage_episodes, success_ma, 
                              label=f'{stage}', linewidth=2)
        
        axes[1, 1].set_title('Learning Within Stages')
        axes[1, 1].set_xlabel('Episodes in Stage')
        axes[1, 1].set_ylabel('Success Rate')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def visualize_assembly_sequence(self, sequence: List[Dict], 
                                  save_path: Optional[str] = None) -> go.Figure:
        """Visualize 3D assembly sequence using Plotly"""
        
        fig = go.Figure()
        
        # Colors for different pieces
        piece_colors = px.colors.qualitative.Set3
        
        for step, action in enumerate(sequence):
            piece_id = action.get('piece_id', step)
            position = action.get('target_position', [0, 0, 0])
            
            # Add block as a 3D scatter point (simplified visualization)
            fig.add_trace(go.Scatter3d(
                x=[position[0]],
                y=[position[1]], 
                z=[position[2]],
                mode='markers+text',
                marker=dict(
                    size=20,
                    color=piece_colors[piece_id % len(piece_colors)],
                    opacity=0.8
                ),
                text=[f'Step {step+1}'],
                textposition="top center",
                name=f'Block {piece_id}'
            ))
        
        # Add grid lines for 3x3x3 cube
        grid_lines = []
        for i in range(4):
            for j in range(4):
                # X-direction lines
                grid_lines.extend([
                    go.Scatter3d(x=[i, i], y=[j, j], z=[0, 3], 
                               mode='lines', line=dict(color='gray', width=1),
                               showlegend=False, hoverinfo='skip')
                ])
                # Y-direction lines  
                grid_lines.extend([
                    go.Scatter3d(x=[i, i], y=[0, 3], z=[j, j],
                               mode='lines', line=dict(color='gray', width=1), 
                               showlegend=False, hoverinfo='skip')
                ])
                # Z-direction lines
                grid_lines.extend([
                    go.Scatter3d(x=[0, 3], y=[i, i], z=[j, j],
                               mode='lines', line=dict(color='gray', width=1),
                               showlegend=False, hoverinfo='skip')
                ])
        
        for line in grid_lines:
            fig.add_trace(line)
        
        fig.update_layout(
            title='SOMA Cube Assembly Sequence',
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y', 
                zaxis_title='Z',
                camera=dict(eye=dict(x=1.5, y=1.5, z=1.5)),
                aspectmode='cube'
            ),
            showlegend=True
        )
        
        if save_path:
            fig.write_html(save_path)
        
        return fig
    
    def plot_hierarchical_performance(self, upper_data: List[Dict], lower_data: List[Dict],
                                    save_path: Optional[str] = None) -> plt.Figure:
        """Compare upper-level and lower-level performance"""
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Hierarchical Performance Analysis', fontsize=16)
        
        # Upper-level planning success
        if 'planning_success' in upper_data[0]:
            upper_df = pd.DataFrame(upper_data)
            window = 50
            planning_success = upper_df['planning_success'].rolling(window=window, min_periods=1).mean()
            axes[0, 0].plot(upper_df['episode'], planning_success,
                          color=self.colors['upper_level'], linewidth=2, label='Upper Level')
        
        # Lower-level execution success
        if 'execution_success' in lower_data[0]:
            lower_df = pd.DataFrame(lower_data)
            execution_success = lower_df['execution_success'].rolling(window=window, min_periods=1).mean()
            axes[0, 0].plot(lower_df['episode'], execution_success,
                          color=self.colors['lower_level'], linewidth=2, label='Lower Level')
        
        axes[0, 0].set_title('Success Rate Comparison')
        axes[0, 0].set_xlabel('Episode')
        axes[0, 0].set_ylabel('Success Rate')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Planning vs execution time
        if 'planning_time' in upper_data[0] and 'execution_time' in lower_data[0]:
            planning_times = [d.get('planning_time', 0) for d in upper_data]
            execution_times = [d.get('execution_time', 0) for d in lower_data]
            
            axes[0, 1].hist(planning_times, bins=30, alpha=0.7, 
                          color=self.colors['upper_level'], label='Planning Time')
            axes[0, 1].hist(execution_times, bins=30, alpha=0.7,
                          color=self.colors['lower_level'], label='Execution Time')
            axes[0, 1].set_title('Time Distribution')
            axes[0, 1].set_xlabel('Time (seconds)')
            axes[0, 1].set_ylabel('Frequency')
            axes[0, 1].legend()
        
        # Loss curves comparison
        if 'upper_loss' in upper_data[0]:
            upper_losses = [d.get('upper_loss', 0) for d in upper_data if d.get('upper_loss') is not None]
            if upper_losses:
                episodes = list(range(len(upper_losses)))
                axes[1, 0].plot(episodes, upper_losses, 
                              color=self.colors['upper_level'], linewidth=2, label='Upper Level Loss')
        
        if 'lower_loss' in lower_data[0]:
            lower_losses = [d.get('lower_loss', 0) for d in lower_data if d.get('lower_loss') is not None]
            if lower_losses:
                episodes = list(range(len(lower_losses)))
                axes[1, 0].plot(episodes, lower_losses,
                              color=self.colors['lower_level'], linewidth=2, label='Lower Level Loss')
        
        axes[1, 0].set_title('Training Loss Comparison')
        axes[1, 0].set_xlabel('Training Step')
        axes[1, 0].set_ylabel('Loss')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        # Correlation analysis
        if len(upper_data) == len(lower_data):
            upper_success = [d.get('success', 0) for d in upper_data]
            lower_success = [d.get('success', 0) for d in lower_data]
            
            axes[1, 1].scatter(upper_success, lower_success, alpha=0.5)
            axes[1, 1].set_title('Upper vs Lower Success Correlation')
            axes[1, 1].set_xlabel('Upper Level Success')
            axes[1, 1].set_ylabel('Lower Level Success')
            axes[1, 1].grid(True, alpha=0.3)
            
            # Add correlation coefficient
            correlation = np.corrcoef(upper_success, lower_success)[0, 1]
            axes[1, 1].text(0.05, 0.95, f'Correlation: {correlation:.3f}',
                          transform=axes[1, 1].transAxes, fontsize=12,
                          bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_domain_randomization_effects(self, randomized_data: List[Dict], 
                                        control_data: List[Dict],
                                        save_path: Optional[str] = None) -> plt.Figure:
        """Analyze effects of domain randomization"""
        
        fig, axes = plt.subplots(2, 2, figsize=(15, 10))
        fig.suptitle('Domain Randomization Effects', fontsize=16)
        
        rand_df = pd.DataFrame(randomized_data)
        control_df = pd.DataFrame(control_data)
        
        window = 100
        
        # Success rate comparison
        rand_success = rand_df['success'].rolling(window=window, min_periods=1).mean()
        control_success = control_df['success'].rolling(window=window, min_periods=1).mean()
        
        axes[0, 0].plot(rand_df['episode'], rand_success, 
                       color='red', linewidth=2, label='With DR')
        axes[0, 0].plot(control_df['episode'], control_success,
                       color='blue', linewidth=2, label='Without DR')
        axes[0, 0].set_title('Success Rate: DR vs Control')
        axes[0, 0].set_xlabel('Episode')
        axes[0, 0].set_ylabel('Success Rate')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        
        # Reward comparison
        rand_reward = rand_df['total_reward'].rolling(window=window, min_periods=1).mean()
        control_reward = control_df['total_reward'].rolling(window=window, min_periods=1).mean()
        
        axes[0, 1].plot(rand_df['episode'], rand_reward,
                       color='red', linewidth=2, label='With DR')
        axes[0, 1].plot(control_df['episode'], control_reward,
                       color='blue', linewidth=2, label='Without DR')
        axes[0, 1].set_title('Reward: DR vs Control')
        axes[0, 1].set_xlabel('Episode')
        axes[0, 1].set_ylabel('Reward')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        # Distribution comparison
        axes[1, 0].hist(rand_df['total_reward'], bins=30, alpha=0.7, 
                       color='red', label='With DR', density=True)
        axes[1, 0].hist(control_df['total_reward'], bins=30, alpha=0.7,
                       color='blue', label='Without DR', density=True)
        axes[1, 0].set_title('Reward Distribution')
        axes[1, 0].set_xlabel('Reward')
        axes[1, 0].set_ylabel('Density')
        axes[1, 0].legend()
        
        # Stability analysis (variance over time)
        rand_variance = rand_df['total_reward'].rolling(window=window).std()
        control_variance = control_df['total_reward'].rolling(window=window).std()
        
        axes[1, 1].plot(rand_df['episode'], rand_variance,
                       color='red', linewidth=2, label='With DR')
        axes[1, 1].plot(control_df['episode'], control_variance,
                       color='blue', linewidth=2, label='Without DR')
        axes[1, 1].set_title('Reward Stability (Rolling Std)')
        axes[1, 1].set_xlabel('Episode')
        axes[1, 1].set_ylabel('Standard Deviation')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def create_training_dashboard(self, training_data: List[Dict],
                                eval_data: Optional[List[Dict]] = None,
                                save_path: Optional[str] = None) -> go.Figure:
        """Create interactive training dashboard with Plotly"""
        
        df = pd.DataFrame(training_data)
        
        # Create subplots
        fig = make_subplots(
            rows=2, cols=2,
            subplot_titles=('Success Rate', 'Reward Progress', 
                          'Steps Taken', 'Curriculum Progress'),
            specs=[[{"secondary_y": False}, {"secondary_y": False}],
                   [{"secondary_y": False}, {"secondary_y": False}]]
        )
        
        # Success rate
        window = 100
        success_rate = df['success'].rolling(window=window, min_periods=1).mean()
        fig.add_trace(
            go.Scatter(x=df['episode'], y=success_rate, 
                      name='Success Rate', line=dict(color='green')),
            row=1, col=1
        )
        
        # Reward
        reward_ma = df['total_reward'].rolling(window=window, min_periods=1).mean()
        fig.add_trace(
            go.Scatter(x=df['episode'], y=reward_ma,
                      name='Average Reward', line=dict(color='blue')),
            row=1, col=2
        )
        
        # Steps
        steps_ma = df['steps_taken'].rolling(window=window, min_periods=1).mean()
        fig.add_trace(
            go.Scatter(x=df['episode'], y=steps_ma,
                      name='Average Steps', line=dict(color='orange')),
            row=2, col=1
        )
        
        # Curriculum progression
        if 'curriculum_stage' in df.columns:
            stage_mapping = {stage: i for i, stage in enumerate(df['curriculum_stage'].unique())}
            stage_numeric = df['curriculum_stage'].map(stage_mapping)
            
            fig.add_trace(
                go.Scatter(x=df['episode'], y=stage_numeric,
                          name='Curriculum Stage', line=dict(color='purple'),
                          mode='lines+markers'),
                row=2, col=2
            )
        
        # Add evaluation points if available
        if eval_data:
            eval_df = pd.DataFrame(eval_data)
            fig.add_trace(
                go.Scatter(x=eval_df['episode'], y=eval_df['success_rate'],
                          name='Evaluation', mode='markers', 
                          marker=dict(color='red', size=8)),
                row=1, col=1
            )
        
        fig.update_layout(
            title_text="Hierarchical SOMA Assembly Training Dashboard",
            showlegend=True,
            height=800
        )
        
        if save_path:
            fig.write_html(save_path)
        
        return fig
    
    def visualize_cube_state(self, grid_state: np.ndarray, 
                           save_path: Optional[str] = None) -> go.Figure:
        """Visualize 3D cube state"""
        
        fig = go.Figure()
        
        # Get filled voxel positions
        filled_positions = np.where(grid_state > 0)
        
        if len(filled_positions[0]) > 0:
            # Add voxels as 3D scatter points
            fig.add_trace(go.Scatter3d(
                x=filled_positions[0],
                y=filled_positions[1],
                z=filled_positions[2],
                mode='markers',
                marker=dict(
                    size=15,
                    color=grid_state[filled_positions],
                    colorscale='viridis',
                    opacity=0.8
                ),
                name='SOMA Blocks'
            ))
        
        # Add wireframe for empty cube
        fig.update_layout(
            title='SOMA Cube Assembly State',
            scene=dict(
                xaxis_title='X',
                yaxis_title='Y',
                zaxis_title='Z',
                camera=dict(eye=dict(x=1.5, y=1.5, z=1.5)),
                aspectmode='cube'
            )
        )
        
        if save_path:
            fig.write_html(save_path)
        
        return fig

def create_training_report(training_data: List[Dict], eval_data: List[Dict],
                         config: Dict[str, Any], output_dir: str = "reports"):
    """
    Generate comprehensive training report with visualizations
    
    Args:
        training_data: Training episode data
        eval_data: Evaluation data
        config: Training configuration
        output_dir: Output directory for report
    """
    
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    visualizer = TrainingVisualizer(str(output_path))
    
    # Generate all visualizations
    learning_curves = visualizer.plot_learning_curves(
        training_data, eval_data,
        save_path=str(output_path / "learning_curves.png")
    )
    
    curriculum_analysis = visualizer.plot_curriculum_analysis(
        training_data,
        save_path=str(output_path / "curriculum_analysis.png")
    )
    
    dashboard = visualizer.create_training_dashboard(
        training_data, eval_data,
        save_path=str(output_path / "training_dashboard.html")
    )
    
    # Generate summary statistics
    df = pd.DataFrame(training_data)
    
    summary_stats = {
        'total_episodes': len(df),
        'final_success_rate': df['success'].tail(100).mean(),
        'best_success_rate': df['success'].rolling(100).mean().max(),
        'average_reward': df['total_reward'].mean(),
        'average_steps': df['steps_taken'].mean(),
        'training_time': config.get('total_training_time', 0)
    }
    
    # Save summary
    with open(output_path / "training_summary.json", 'w') as f:
        import json
        json.dump(summary_stats, f, indent=2)
    
    logger.info(f"Training report generated in {output_path}")
    
    return summary_stats
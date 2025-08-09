#!/usr/bin/env python3
"""
ROS2 client interface for SomaCube RL environment
Provides Gym-like interface for external training frameworks
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState

from soma_cube_rl_bridge.srv import RLReset, RLStep, GetSafetyState
from soma_cube_rl_bridge.msg import SafetyState, RLObservation

import threading
import time
from typing import Tuple, Optional, Dict, Any
import json


class SomaCubeRLClient(Node):
    """
    ROS2 client for interacting with SomaCube RL environment
    Provides standard Gym-like interface: reset(), step(), close()
    """
    
    def __init__(self, node_name: str = 'somacube_rl_client', timeout_sec: float = 30.0):
        super().__init__(node_name)
        
        self.timeout_sec = timeout_sec
        self.observation_space_size = 28  # As defined in the RL environment
        self.action_space_size = 6       # 6-DoF joint control
        
        # Service clients
        self.reset_client = self.create_client(RLReset, '/rl/reset')
        self.step_client = self.create_client(RLStep, '/rl/step')
        self.safety_client = self.create_client(GetSafetyState, 'safety/get_state')
        
        # Subscribers for monitoring
        self.safety_sub = self.create_subscription(
            SafetyState,
            'safety/state',
            self.safety_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.observation_sub = self.create_subscription(
            RLObservation,
            '/rl/observation',
            self.observation_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        # State tracking
        self.current_safety_state: Optional[SafetyState] = None
        self.current_observation: Optional[RLObservation] = None
        self.episode_count = 0
        self.step_count = 0
        self.total_reward = 0.0
        
        # Threading for ROS spinning
        self.spin_thread = None
        self.should_spin = True
        
        # Wait for services to be available
        self._wait_for_services()
        
        # Start spinning in background
        self._start_background_spin()
        
        self.get_logger().info("SomaCube RL Client initialized")
    
    def _wait_for_services(self):
        """Wait for all required services to become available"""
        services = [
            (self.reset_client, '/rl/reset'),
            (self.step_client, '/rl/step'),
            (self.safety_client, 'safety/get_state')
        ]
        
        for client, name in services:
            self.get_logger().info(f"Waiting for service: {name}")
            if not client.wait_for_service(timeout_sec=self.timeout_sec):
                raise RuntimeError(f"Service {name} not available after {self.timeout_sec}s")
            self.get_logger().info(f"Service {name} is available")
    
    def _start_background_spin(self):
        """Start ROS2 spinning in background thread"""
        def spin_worker():
            while self.should_spin:
                rclpy.spin_once(self, timeout_sec=0.1)
        
        self.spin_thread = threading.Thread(target=spin_worker)
        self.spin_thread.daemon = True
        self.spin_thread.start()
    
    def safety_callback(self, msg: SafetyState):
        """Update safety state from subscription"""
        self.current_safety_state = msg
        if not msg.safe_to_move:
            self.get_logger().warn(f"Safety violation: {msg.reason}")
    
    def observation_callback(self, msg: RLObservation):
        """Update observation from subscription"""
        self.current_observation = msg
    
    def is_safe(self) -> bool:
        """Check if robot is in safe state for training"""
        if self.current_safety_state is None:
            # Query safety state if not subscribed yet
            request = GetSafetyState.Request()
            future = self.safety_client.call_async(request)
            
            # Wait for response
            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                time.sleep(0.01)
            
            if future.done():
                response = future.result()
                return response.safe_to_move
            else:
                self.get_logger().warn("Failed to get safety state")
                return False
        
        return self.current_safety_state.safe_to_move
    
    def reset(self, seed: Optional[int] = None) -> np.ndarray:
        """
        Reset the environment
        
        Args:
            seed: Random seed for reproducible resets
            
        Returns:
            Initial observation as numpy array
        """
        if not self.is_safe():
            self.get_logger().warn("Attempting reset while robot is not safe")
        
        request = RLReset.Request()
        if seed is not None:
            request.seed = seed
        
        self.get_logger().info(f"Resetting environment (episode {self.episode_count + 1})")
        
        future = self.reset_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        
        if not future.done():
            raise RuntimeError(f"Reset service call timed out after {self.timeout_sec}s")
        
        response = future.result()
        if not response.success:
            raise RuntimeError(f"Reset failed: {response.message}")
        
        # Update episode tracking
        self.episode_count += 1
        self.step_count = 0
        self.total_reward = 0.0
        
        obs_array = np.array(response.initial_obs, dtype=np.float32)
        self.get_logger().info(f"Reset successful, observation shape: {obs_array.shape}")
        
        return obs_array
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, Dict[str, Any]]:
        """
        Execute one step in the environment
        
        Args:
            action: 6-element array of joint positions (degrees)
            
        Returns:
            tuple: (observation, reward, done, info)
        """
        if action.shape != (self.action_space_size,):
            raise ValueError(f"Action must have shape ({self.action_space_size},), got {action.shape}")
        
        request = RLStep.Request()
        request.action = action.tolist()
        
        step_start_time = time.time()
        
        future = self.step_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout_sec)
        
        step_duration = time.time() - step_start_time
        
        if not future.done():
            raise RuntimeError(f"Step service call timed out after {self.timeout_sec}s")
        
        response = future.result()
        if not response.success:
            self.get_logger().error(f"Step failed, terminating episode")
            # Return terminal state
            obs_array = np.array(response.obs, dtype=np.float32) if response.obs else np.zeros(self.observation_space_size)
            return obs_array, -100.0, True, {"error": "Step execution failed"}
        
        # Update tracking
        self.step_count += 1
        self.total_reward += response.reward
        
        # Parse info string (JSON format)
        info_dict = {}
        try:
            info_dict = json.loads(response.info)
        except (json.JSONDecodeError, TypeError):
            info_dict = {"raw_info": response.info}
        
        # Add timing and episode info
        info_dict.update({
            "step_duration_ms": step_duration * 1000,
            "episode_step": self.step_count,
            "episode_reward": self.total_reward,
            "episode_num": self.episode_count
        })
        
        obs_array = np.array(response.obs, dtype=np.float32)
        
        if response.done:
            self.get_logger().info(
                f"Episode {self.episode_count} complete: "
                f"{self.step_count} steps, reward: {self.total_reward:.2f}"
            )
        
        return obs_array, response.reward, response.done, info_dict
    
    def get_action_space_info(self) -> Dict[str, Any]:
        """Get information about the action space"""
        return {
            "shape": (self.action_space_size,),
            "type": "continuous",
            "low": np.array([-170.0, -135.0, -169.0, -90.0, -135.0, -360.0]),
            "high": np.array([170.0, 135.0, 169.0, 90.0, 135.0, 360.0]),
            "description": "6-DoF joint positions in degrees"
        }
    
    def get_observation_space_info(self) -> Dict[str, Any]:
        """Get information about the observation space"""
        return {
            "shape": (self.observation_space_size,),
            "type": "continuous",
            "description": {
                "joint_positions": "elements 0-5: current joint positions (degrees)",
                "joint_velocities": "elements 6-11: current joint velocities (deg/s)", 
                "tcp_position": "elements 12-14: TCP position (mm)",
                "tcp_orientation": "elements 15-18: TCP orientation (quaternion)",
                "tool_force": "elements 19-24: tool force/torque (N, Nm)",
                "distance_to_target": "element 25: distance to target (mm)",
                "current_step": "element 26: current step in episode",
                "remaining_steps": "element 27: steps remaining in episode"
            }
        }
    
    def get_stats(self) -> Dict[str, Any]:
        """Get current training statistics"""
        return {
            "episodes_completed": self.episode_count,
            "current_episode_steps": self.step_count,
            "current_episode_reward": self.total_reward,
            "safety_state": self.current_safety_state.safe_to_move if self.current_safety_state else None,
            "safety_reason": self.current_safety_state.reason if self.current_safety_state else None
        }
    
    def close(self):
        """Clean up resources"""
        self.get_logger().info("Closing SomaCube RL Client")
        self.should_spin = False
        if self.spin_thread and self.spin_thread.is_alive():
            self.spin_thread.join(timeout=1.0)
        
        # Destroy subscribers and clients
        self.destroy_node()


def main():
    """Simple test of the RL client"""
    rclpy.init()
    
    try:
        client = SomaCubeRLClient()
        
        print("🤖 SomaCube RL Client Test")
        print("Action space:", client.get_action_space_info())
        print("Observation space:", client.get_observation_space_info())
        
        # Test reset
        print("\n🔄 Testing reset...")
        obs = client.reset(seed=42)
        print(f"Initial observation shape: {obs.shape}")
        print(f"Initial observation: {obs[:6]}... (showing first 6 elements)")
        
        # Test a few steps
        print("\n👟 Testing steps...")
        for i in range(3):
            # Random action within bounds
            action_info = client.get_action_space_info()
            action = np.random.uniform(
                action_info["low"] * 0.1,  # Small movements for safety
                action_info["high"] * 0.1,
                size=6
            )
            
            obs, reward, done, info = client.step(action)
            print(f"Step {i+1}: reward={reward:.3f}, done={done}, step_time={info.get('step_duration_ms', 0):.1f}ms")
            
            if done:
                break
        
        print("\n📊 Final stats:", client.get_stats())
        
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        client.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
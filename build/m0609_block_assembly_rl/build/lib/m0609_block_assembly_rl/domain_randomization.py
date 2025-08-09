"""
Domain Randomization for Sim-to-Real Transfer

This module implements comprehensive domain randomization techniques for robust
sim-to-real transfer in SOMA cube assembly tasks:

1. Physical parameter randomization (friction, mass, compliance)
2. Sensor noise and calibration errors  
3. Visual domain randomization (lighting, textures, occlusions)
4. Robot dynamics randomization (joint friction, backlash, delays)
5. Progressive randomization scheduling
"""

import numpy as np
import torch
import cv2
import random
from typing import Dict, List, Tuple, Optional, Any, Union
from dataclasses import dataclass
from abc import ABC, abstractmethod
from enum import Enum
import logging
from pathlib import Path
import json

logger = logging.getLogger(__name__)

class RandomizationType(Enum):
    """Types of domain randomization"""
    PHYSICS = "physics"
    VISION = "vision"  
    SENSORS = "sensors"
    DYNAMICS = "dynamics"
    GEOMETRY = "geometry"

@dataclass
class RandomizationParams:
    """Parameters for domain randomization"""
    enabled: bool = True
    randomization_prob: float = 0.8  # Probability of applying randomization
    intensity: float = 1.0  # Randomization intensity (0-1)
    progressive: bool = True  # Whether to use progressive randomization
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'enabled': self.enabled,
            'randomization_prob': self.randomization_prob,
            'intensity': self.intensity,
            'progressive': self.progressive
        }

class DomainRandomizer(ABC):
    """Base class for domain randomizers"""
    
    def __init__(self, params: RandomizationParams):
        self.params = params
        self.randomization_history = []
        
    @abstractmethod
    def randomize(self, data: Any) -> Any:
        """Apply randomization to input data"""
        pass
        
    def should_randomize(self) -> bool:
        """Check if randomization should be applied"""
        return (self.params.enabled and 
                random.random() < self.params.randomization_prob)
    
    def get_intensity(self) -> float:
        """Get current randomization intensity"""
        if self.params.progressive:
            # Progressive randomization can be implemented here
            return self.params.intensity
        return self.params.intensity

class PhysicsRandomizer(DomainRandomizer):
    """
    Randomize physical parameters for sim-to-real transfer
    
    Randomizes:
    - Block mass and inertia
    - Surface friction coefficients
    - Contact compliance and damping
    - Gravitational effects
    - Air resistance
    """
    
    def __init__(self, params: RandomizationParams):
        super().__init__(params)
        
        # Physics parameter ranges
        self.mass_range = (0.8, 1.2)  # ±20% mass variation
        self.friction_range = (0.3, 1.2)  # Friction coefficient range
        self.restitution_range = (0.1, 0.9)  # Bounce/restitution
        self.compliance_range = (0.5, 2.0)  # Contact compliance
        self.damping_range = (0.8, 1.2)  # Contact damping
        
    def randomize(self, physics_params: Dict[str, float]) -> Dict[str, float]:
        """Randomize physics parameters"""
        if not self.should_randomize():
            return physics_params
            
        randomized = physics_params.copy()
        intensity = self.get_intensity()
        
        # Mass randomization
        if 'mass' in randomized:
            mass_factor = np.random.uniform(*self.mass_range)
            randomized['mass'] *= self._apply_intensity(mass_factor, intensity)
            
        # Friction randomization  
        if 'friction' in randomized:
            friction_factor = np.random.uniform(*self.friction_range)
            randomized['friction'] = self._apply_intensity(friction_factor, intensity)
            
        # Restitution randomization
        if 'restitution' in randomized:
            restitution_factor = np.random.uniform(*self.restitution_range)
            randomized['restitution'] = self._apply_intensity(restitution_factor, intensity)
            
        # Contact compliance
        if 'compliance' in randomized:
            compliance_factor = np.random.uniform(*self.compliance_range)
            randomized['compliance'] *= self._apply_intensity(compliance_factor, intensity)
            
        # Damping
        if 'damping' in randomized:
            damping_factor = np.random.uniform(*self.damping_range)
            randomized['damping'] *= self._apply_intensity(damping_factor, intensity)
            
        return randomized
    
    def _apply_intensity(self, factor: float, intensity: float) -> float:
        """Apply intensity scaling to randomization factor"""
        return 1.0 + intensity * (factor - 1.0)

class VisionRandomizer(DomainRandomizer):
    """
    Visual domain randomization for RGB-D images
    
    Randomizes:
    - Lighting conditions (brightness, contrast, shadows)
    - Camera parameters (exposure, white balance, noise)
    - Background textures and colors
    - Object appearance (texture, reflectance)
    - Occlusions and distractors
    """
    
    def __init__(self, params: RandomizationParams):
        super().__init__(params)
        
        # Vision parameter ranges
        self.brightness_range = (-30, 30)
        self.contrast_range = (0.7, 1.3)
        self.saturation_range = (0.5, 1.5)
        self.hue_shift_range = (-15, 15)
        self.noise_std_range = (0, 10)
        self.blur_kernel_range = (0, 3)
        
    def randomize(self, image: np.ndarray) -> np.ndarray:
        """Apply visual randomization to image"""
        if not self.should_randomize():
            return image
            
        randomized_image = image.copy()
        intensity = self.get_intensity()
        
        # Apply randomizations sequentially
        randomized_image = self._randomize_brightness_contrast(randomized_image, intensity)
        randomized_image = self._randomize_color(randomized_image, intensity)
        randomized_image = self._add_noise(randomized_image, intensity)
        randomized_image = self._add_blur(randomized_image, intensity)
        
        return randomized_image
    
    def _randomize_brightness_contrast(self, image: np.ndarray, intensity: float) -> np.ndarray:
        """Randomize brightness and contrast"""
        # Brightness adjustment
        brightness = np.random.uniform(*self.brightness_range) * intensity
        image = np.clip(image.astype(np.float32) + brightness, 0, 255)
        
        # Contrast adjustment
        contrast = np.random.uniform(*self.contrast_range)
        contrast = 1.0 + intensity * (contrast - 1.0)
        image = np.clip(image * contrast, 0, 255)
        
        return image.astype(np.uint8)
    
    def _randomize_color(self, image: np.ndarray, intensity: float) -> np.ndarray:
        """Randomize color properties"""
        # Convert to HSV for easier color manipulation
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
        
        # Hue shift
        hue_shift = np.random.uniform(*self.hue_shift_range) * intensity
        hsv[:, :, 0] = np.clip(hsv[:, :, 0] + hue_shift, 0, 179)
        
        # Saturation adjustment
        saturation_factor = np.random.uniform(*self.saturation_range)
        saturation_factor = 1.0 + intensity * (saturation_factor - 1.0)
        hsv[:, :, 1] = np.clip(hsv[:, :, 1] * saturation_factor, 0, 255)
        
        # Convert back to BGR
        image = cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2BGR)
        return image
    
    def _add_noise(self, image: np.ndarray, intensity: float) -> np.ndarray:
        """Add random noise to image"""
        noise_std = np.random.uniform(*self.noise_std_range) * intensity
        noise = np.random.normal(0, noise_std, image.shape)
        noisy_image = np.clip(image.astype(np.float32) + noise, 0, 255)
        return noisy_image.astype(np.uint8)
    
    def _add_blur(self, image: np.ndarray, intensity: float) -> np.ndarray:
        """Add random blur to image"""
        kernel_size = int(np.random.uniform(*self.blur_kernel_range) * intensity)
        if kernel_size > 0:
            kernel_size = kernel_size * 2 + 1  # Ensure odd kernel size
            image = cv2.GaussianBlur(image, (kernel_size, kernel_size), 0)
        return image
    
    def randomize_depth(self, depth_image: np.ndarray) -> np.ndarray:
        """Apply randomization specific to depth images"""
        if not self.should_randomize():
            return depth_image
            
        intensity = self.get_intensity()
        randomized_depth = depth_image.copy()
        
        # Add depth noise
        depth_noise_std = 0.01 * intensity  # 1cm standard deviation
        noise = np.random.normal(0, depth_noise_std, depth_image.shape)
        randomized_depth = np.clip(randomized_depth + noise, 0, np.inf)
        
        # Simulate missing depth values (holes)
        if random.random() < 0.1 * intensity:
            hole_mask = np.random.random(depth_image.shape) < 0.02
            randomized_depth[hole_mask] = 0
        
        return randomized_depth

class SensorRandomizer(DomainRandomizer):
    """
    Randomize sensor measurements and calibration
    
    Randomizes:
    - Joint encoder noise and bias
    - Force/torque sensor noise and drift
    - Camera calibration parameters
    - IMU measurements
    - Timing delays and jitter
    """
    
    def __init__(self, params: RandomizationParams):
        super().__init__(params)
        
        # Sensor parameter ranges
        self.joint_noise_std = 0.001  # 0.001 radians ~0.06 degrees
        self.joint_bias_range = (-0.005, 0.005)  # ±0.005 radians
        self.force_noise_std = 0.5  # 0.5 N standard deviation
        self.torque_noise_std = 0.05  # 0.05 Nm standard deviation
        self.time_delay_range = (0, 0.02)  # 0-20ms delays
    
    def randomize(self, data: Any) -> Any:
        """Apply randomization to sensor data"""
        return data  # Default implementation
        
    def randomize_joint_state(self, joint_positions: np.ndarray, 
                            joint_velocities: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Randomize joint state measurements"""
        if not self.should_randomize():
            return joint_positions, joint_velocities
            
        intensity = self.get_intensity()
        
        # Position noise and bias
        pos_noise = np.random.normal(0, self.joint_noise_std * intensity, joint_positions.shape)
        pos_bias = np.random.uniform(*self.joint_bias_range, joint_positions.shape) * intensity
        noisy_positions = joint_positions + pos_noise + pos_bias
        
        # Velocity noise
        vel_noise = np.random.normal(0, self.joint_noise_std * intensity, joint_velocities.shape)
        noisy_velocities = joint_velocities + vel_noise
        
        return noisy_positions, noisy_velocities
    
    def randomize_force_torque(self, force_torque: np.ndarray) -> np.ndarray:
        """Randomize force/torque measurements"""
        if not self.should_randomize():
            return force_torque
            
        intensity = self.get_intensity()
        
        # Force noise (first 3 components)
        force_noise = np.random.normal(0, self.force_noise_std * intensity, 3)
        
        # Torque noise (last 3 components)
        torque_noise = np.random.normal(0, self.torque_noise_std * intensity, 3)
        
        noise = np.concatenate([force_noise, torque_noise])
        return force_torque + noise
    
    def randomize_camera_intrinsics(self, intrinsics: Dict[str, float]) -> Dict[str, float]:
        """Randomize camera intrinsic parameters"""
        if not self.should_randomize():
            return intrinsics
            
        randomized = intrinsics.copy()
        intensity = self.get_intensity()
        
        # Focal length variations (±2%)
        if 'fx' in randomized:
            fx_factor = 1.0 + np.random.uniform(-0.02, 0.02) * intensity
            randomized['fx'] *= fx_factor
            
        if 'fy' in randomized:
            fy_factor = 1.0 + np.random.uniform(-0.02, 0.02) * intensity
            randomized['fy'] *= fy_factor
            
        # Principal point variations (±5 pixels)
        if 'cx' in randomized:
            cx_offset = np.random.uniform(-5, 5) * intensity
            randomized['cx'] += cx_offset
            
        if 'cy' in randomized:
            cy_offset = np.random.uniform(-5, 5) * intensity
            randomized['cy'] += cy_offset
            
        return randomized

class DynamicsRandomizer(DomainRandomizer):
    """
    Randomize robot dynamics parameters
    
    Randomizes:
    - Joint friction and backlash
    - Actuator dynamics and delays
    - Link masses and inertias
    - Gear ratios and compliance
    """
    
    def __init__(self, params: RandomizationParams):
        super().__init__(params)
        
        # Dynamics parameter ranges
        self.joint_friction_range = (0.8, 1.2)
        self.backlash_range = (0, 0.01)  # Up to 0.01 radians backlash
        self.mass_variation_range = (0.9, 1.1)  # ±10% mass variation
        self.actuator_delay_range = (0, 0.05)  # Up to 50ms delay
        
    def randomize(self, data: Any) -> Any:
        """Apply randomization to dynamics data"""
        if isinstance(data, dict):
            return self.randomize_dynamics(data)
        return data
        
    def randomize_dynamics(self, dynamics_params: Dict[str, Any]) -> Dict[str, Any]:
        """Randomize robot dynamics parameters"""
        if not self.should_randomize():
            return dynamics_params
            
        randomized = dynamics_params.copy()
        intensity = self.get_intensity()
        
        # Joint friction
        if 'joint_friction' in randomized:
            friction_factors = np.random.uniform(
                *self.joint_friction_range, 
                len(randomized['joint_friction'])
            )
            friction_factors = 1.0 + intensity * (friction_factors - 1.0)
            randomized['joint_friction'] = [
                orig * factor for orig, factor in 
                zip(randomized['joint_friction'], friction_factors)
            ]
            
        # Backlash
        if 'joint_backlash' in randomized:
            backlash = np.random.uniform(*self.backlash_range, 6) * intensity
            randomized['joint_backlash'] = backlash.tolist()
            
        # Link masses
        if 'link_masses' in randomized:
            mass_factors = np.random.uniform(
                *self.mass_variation_range,
                len(randomized['link_masses'])
            )
            mass_factors = 1.0 + intensity * (mass_factors - 1.0)
            randomized['link_masses'] = [
                orig * factor for orig, factor in
                zip(randomized['link_masses'], mass_factors)
            ]
            
        return randomized

class GeometryRandomizer(DomainRandomizer):
    """
    Randomize geometric parameters
    
    Randomizes:
    - Block dimensions and tolerances
    - Robot kinematic parameters
    - Workspace boundaries
    - Tool/gripper geometry
    """
    
    def __init__(self, params: RandomizationParams):
        super().__init__(params)
        
        # Geometry parameter ranges
        self.block_size_range = (0.98, 1.02)  # ±2% size variation
        self.position_tolerance_range = (0, 0.002)  # Up to 2mm tolerance
        self.angle_tolerance_range = (0, np.radians(2))  # Up to 2 degrees
        
    def randomize(self, data: Any) -> Any:
        """Apply randomization to geometry data"""
        if isinstance(data, dict):
            return self.randomize_block_geometry(data)
        return data
        
    def randomize_block_geometry(self, block_params: Dict[str, Any]) -> Dict[str, Any]:
        """Randomize SOMA block geometry"""
        if not self.should_randomize():
            return block_params
            
        randomized = block_params.copy()
        intensity = self.get_intensity()
        
        # Block size variations
        if 'block_size' in randomized:
            size_factor = np.random.uniform(*self.block_size_range)
            size_factor = 1.0 + intensity * (size_factor - 1.0)
            randomized['block_size'] *= size_factor
            
        # Position tolerances
        if 'position_tolerance' in randomized:
            tolerance = np.random.uniform(*self.position_tolerance_range) * intensity
            randomized['position_tolerance'] = tolerance
            
        # Angular tolerances
        if 'angle_tolerance' in randomized:
            angle_tolerance = np.random.uniform(*self.angle_tolerance_range) * intensity
            randomized['angle_tolerance'] = angle_tolerance
            
        return randomized

class DomainRandomizationManager:
    """
    Manages all domain randomization components
    
    Coordinates multiple randomizers and provides unified interface
    for comprehensive domain randomization in SOMA cube assembly.
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        self.config = self._setup_default_config(config)
        
        # Initialize randomizers
        self.randomizers = {}
        
        physics_params = RandomizationParams(**self.config.get('physics', {}))
        self.randomizers[RandomizationType.PHYSICS] = PhysicsRandomizer(physics_params)
        
        vision_params = RandomizationParams(**self.config.get('vision', {}))
        self.randomizers[RandomizationType.VISION] = VisionRandomizer(vision_params)
        
        sensor_params = RandomizationParams(**self.config.get('sensors', {}))
        self.randomizers[RandomizationType.SENSORS] = SensorRandomizer(sensor_params)
        
        dynamics_params = RandomizationParams(**self.config.get('dynamics', {}))
        self.randomizers[RandomizationType.DYNAMICS] = DynamicsRandomizer(dynamics_params)
        
        geometry_params = RandomizationParams(**self.config.get('geometry', {}))
        self.randomizers[RandomizationType.GEOMETRY] = GeometryRandomizer(geometry_params)
        
        # Progressive randomization state
        self.training_step = 0
        self.progressive_schedule = self.config.get('progressive_schedule', {
            'start_step': 1000,
            'full_intensity_step': 10000,
            'intensity_schedule': 'linear'
        })
        
        logger.info("Domain Randomization Manager initialized")
    
    def _setup_default_config(self, config: Optional[Dict]) -> Dict[str, Any]:
        """Setup default configuration"""
        default = {
            'physics': {
                'enabled': True,
                'randomization_prob': 0.8,
                'intensity': 0.8,
                'progressive': True
            },
            'vision': {
                'enabled': True,
                'randomization_prob': 0.9,
                'intensity': 0.7,
                'progressive': True
            },
            'sensors': {
                'enabled': True,
                'randomization_prob': 0.7,
                'intensity': 0.6,
                'progressive': True
            },
            'dynamics': {
                'enabled': True,
                'randomization_prob': 0.6,
                'intensity': 0.5,
                'progressive': True
            },
            'geometry': {
                'enabled': True,
                'randomization_prob': 0.5,
                'intensity': 0.4,
                'progressive': True
            }
        }
        
        if config:
            for key in default:
                if key in config:
                    default[key].update(config[key])
                    
        return default
    
    def update_progressive_intensity(self, training_step: int):
        """Update progressive randomization intensity"""
        self.training_step = training_step
        
        schedule = self.progressive_schedule
        start_step = schedule['start_step']
        full_step = schedule['full_intensity_step']
        
        if training_step < start_step:
            # No randomization initially
            intensity_factor = 0.0
        elif training_step >= full_step:
            # Full intensity
            intensity_factor = 1.0
        else:
            # Progressive ramp-up
            progress = (training_step - start_step) / (full_step - start_step)
            
            if schedule['intensity_schedule'] == 'linear':
                intensity_factor = progress
            elif schedule['intensity_schedule'] == 'quadratic':
                intensity_factor = progress ** 2
            elif schedule['intensity_schedule'] == 'exponential':
                intensity_factor = 1.0 - np.exp(-3 * progress)
            else:
                intensity_factor = progress
        
        # Update all randomizers
        for rand_type, randomizer in self.randomizers.items():
            if randomizer.params.progressive:
                config_key = rand_type.value  # Use the enum value directly
                if config_key in self.config:
                    original_intensity = self.config[config_key]['intensity']
                    randomizer.params.intensity = original_intensity * intensity_factor
    
    def randomize_observation(self, observation: Dict[str, Any]) -> Dict[str, Any]:
        """Apply randomization to observation"""
        randomized_obs = observation.copy()
        
        # Vision randomization
        if 'rgb' in randomized_obs:
            randomized_obs['rgb'] = self.randomizers[RandomizationType.VISION].randomize(
                randomized_obs['rgb']
            )
            
        if 'depth' in randomized_obs:
            randomized_obs['depth'] = self.randomizers[RandomizationType.VISION].randomize_depth(
                randomized_obs['depth']
            )
        
        # Sensor randomization
        if 'joint_positions' in randomized_obs and 'joint_velocities' in randomized_obs:
            pos, vel = self.randomizers[RandomizationType.SENSORS].randomize_joint_state(
                randomized_obs['joint_positions'],
                randomized_obs['joint_velocities']
            )
            randomized_obs['joint_positions'] = pos
            randomized_obs['joint_velocities'] = vel
            
        if 'force_torque' in randomized_obs:
            randomized_obs['force_torque'] = self.randomizers[RandomizationType.SENSORS].randomize_force_torque(
                randomized_obs['force_torque']
            )
        
        return randomized_obs
    
    def randomize_environment_params(self, env_params: Dict[str, Any]) -> Dict[str, Any]:
        """Apply randomization to environment parameters"""
        randomized_params = env_params.copy()
        
        # Physics randomization
        if 'physics' in randomized_params:
            randomized_params['physics'] = self.randomizers[RandomizationType.PHYSICS].randomize(
                randomized_params['physics']
            )
        
        # Dynamics randomization
        if 'dynamics' in randomized_params:
            randomized_params['dynamics'] = self.randomizers[RandomizationType.DYNAMICS].randomize_dynamics(
                randomized_params['dynamics']
            )
            
        # Geometry randomization
        if 'geometry' in randomized_params:
            randomized_params['geometry'] = self.randomizers[RandomizationType.GEOMETRY].randomize_block_geometry(
                randomized_params['geometry']
            )
        
        return randomized_params
    
    def get_randomization_status(self) -> Dict[str, Any]:
        """Get current randomization status"""
        status = {
            'training_step': self.training_step,
            'randomizers': {}
        }
        
        for rand_type, randomizer in self.randomizers.items():
            status['randomizers'][rand_type.value] = {
                'enabled': randomizer.params.enabled,
                'intensity': randomizer.params.intensity,
                'randomization_prob': randomizer.params.randomization_prob
            }
        
        return status
    
    def enable_randomization(self, randomization_type: Optional[RandomizationType] = None):
        """Enable randomization for specific type or all"""
        if randomization_type:
            self.randomizers[randomization_type].params.enabled = True
        else:
            for randomizer in self.randomizers.values():
                randomizer.params.enabled = True
                
        logger.info(f"Enabled randomization: {randomization_type.value if randomization_type else 'all'}")
    
    def disable_randomization(self, randomization_type: Optional[RandomizationType] = None):
        """Disable randomization for specific type or all"""
        if randomization_type:
            self.randomizers[randomization_type].params.enabled = False
        else:
            for randomizer in self.randomizers.values():
                randomizer.params.enabled = False
                
        logger.info(f"Disabled randomization: {randomization_type.value if randomization_type else 'all'}")
    
    def save_config(self, filepath: str):
        """Save randomization configuration"""
        config_to_save = {
            'config': self.config,
            'progressive_schedule': self.progressive_schedule,
            'training_step': self.training_step
        }
        
        with open(filepath, 'w') as f:
            json.dump(config_to_save, f, indent=2)
            
        logger.info(f"Randomization config saved to {filepath}")
    
    def load_config(self, filepath: str):
        """Load randomization configuration"""
        with open(filepath, 'r') as f:
            saved_config = json.load(f)
            
        self.config = saved_config['config']
        self.progressive_schedule = saved_config['progressive_schedule']
        self.training_step = saved_config.get('training_step', 0)
        
        # Reinitialize randomizers with loaded config
        self.__init__(self.config)
        
        logger.info(f"Randomization config loaded from {filepath}")

def create_domain_randomization_manager(config_path: Optional[str] = None) -> DomainRandomizationManager:
    """
    Factory function to create domain randomization manager
    
    Args:
        config_path: Path to configuration file
        
    Returns:
        DomainRandomizationManager instance
    """
    
    config = None
    if config_path and Path(config_path).exists():
        with open(config_path, 'r') as f:
            config = json.load(f)
    
    return DomainRandomizationManager(config)
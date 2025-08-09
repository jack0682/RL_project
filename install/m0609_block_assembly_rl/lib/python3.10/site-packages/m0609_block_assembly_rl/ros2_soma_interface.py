"""
ROS2 Interface for Hierarchical SOMA Cube Assembly

Provides ROS2-compatible interfaces for:
- Real robot execution with M0609 Doosan robot
- RGB-D camera integration  
- Force/torque sensor feedback
- Visualization and monitoring
- Action server for external control
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import numpy as np
import cv2
import time
import threading
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass

# ROS2 message types
from std_msgs.msg import Float64MultiArray, Bool, String
from geometry_msgs.msg import Pose, Twist, WrenchStamped
from sensor_msgs.msg import Image, JointState, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge

# Custom action definitions (would need to be defined in separate package)
from soma_assembly_msgs.action import AssembleSOMACube, ExecuteManipulation
from soma_assembly_msgs.msg import SOMAState, AssemblySequence, PieceInfo
from soma_assembly_msgs.srv import PlanSequence, ResetAssembly

# Doosan robot interface
try:
    import DR_init
    from DSR_ROBOT2 import *
    from DR_common2 import *
    DOOSAN_AVAILABLE = True
except ImportError:
    print("Warning: Doosan robot modules not available")
    DOOSAN_AVAILABLE = False

from .soma_cube_system import SOMAPiece, SOMAPieceManager, AssemblyState, UpperLevelAction, LowerLevelGoal
# from .hierarchical_soma_trainer import HierarchicalSOMATrainer  # Import only when needed
from .upper_level_planner import UpperLevelPlanner
from .lower_level_controller import LowerLevelController

@dataclass
class RobotState:
    """Current robot state information"""
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    end_effector_pose: np.ndarray  # 6DOF pose
    gripper_position: float
    force_torque: np.ndarray  # 6DOF wrench
    is_moving: bool
    error_state: bool

class SOMAVisualizationNode(Node):
    """ROS2 node for SOMA cube assembly visualization"""
    
    def __init__(self):
        super().__init__('soma_visualization')
        
        # Publishers for visualization
        self.marker_pub = self.create_publisher(MarkerArray, 'soma_cube_markers', 10)
        self.sequence_pub = self.create_publisher(AssemblySequence, 'planned_sequence', 10)
        self.state_pub = self.create_publisher(SOMAState, 'assembly_state', 10)
        
        # Timer for regular updates
        self.vis_timer = self.create_timer(0.1, self.publish_visualizations)
        
        # Current state
        self.current_state: Optional[AssemblyState] = None
        self.planned_sequence: List[UpperLevelAction] = []
        
        self.get_logger().info("SOMA Visualization Node initialized")
        
    def update_assembly_state(self, state: AssemblyState):
        """Update current assembly state"""
        self.current_state = state
        
    def update_planned_sequence(self, sequence: List[UpperLevelAction]):
        """Update planned assembly sequence"""
        self.planned_sequence = sequence
        
    def publish_visualizations(self):
        """Publish visualization markers"""
        if self.current_state is None:
            return
            
        # Create marker array for 3D grid visualization
        marker_array = MarkerArray()
        
        # Grid markers
        for x in range(3):
            for y in range(3):
                for z in range(3):
                    marker = Marker()
                    marker.header.frame_id = "base_link"
                    marker.header.stamp = self.get_clock().now().to_msg()
                    marker.ns = "soma_grid"
                    marker.id = x * 9 + y * 3 + z
                    marker.type = Marker.CUBE
                    
                    if self.current_state.grid[x, y, z] == 1:
                        marker.action = Marker.ADD
                        marker.color.r = 0.0
                        marker.color.g = 1.0
                        marker.color.b = 0.0
                        marker.color.a = 0.8
                    else:
                        marker.action = Marker.ADD
                        marker.color.r = 0.5
                        marker.color.g = 0.5
                        marker.color.b = 0.5
                        marker.color.a = 0.1
                        
                    # Position (convert grid to real coordinates)
                    marker.pose.position.x = 0.4 + x * 0.05  # 400mm + x*50mm
                    marker.pose.position.y = -0.1 + y * 0.05  # -100mm + y*50mm
                    marker.pose.position.z = 0.1 + z * 0.05   # 100mm + z*50mm
                    
                    marker.pose.orientation.w = 1.0
                    
                    # Scale
                    marker.scale.x = 0.045  # Slightly smaller than grid cell
                    marker.scale.y = 0.045
                    marker.scale.z = 0.045
                    
                    marker_array.markers.append(marker)
                    
        # Planned sequence markers
        for i, action in enumerate(self.planned_sequence):
            marker = Marker()
            marker.header.frame_id = "base_link" 
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "planned_sequence"
            marker.id = 1000 + i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            
            # Position from target pose
            marker.pose.position.x = action.target_pose[0] / 1000.0  # Convert mm to m
            marker.pose.position.y = action.target_pose[1] / 1000.0
            marker.pose.position.z = action.target_pose[2] / 1000.0 + 0.1  # Above block
            marker.pose.orientation.w = 1.0
            
            marker.text = f"{i+1}: {action.piece.value}"
            marker.scale.z = 0.02  # Text height
            
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            
            marker_array.markers.append(marker)
            
        self.marker_pub.publish(marker_array)
        
        # Publish assembly state
        state_msg = SOMAState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.header.frame_id = "base_link"
        
        # Grid as flat array
        state_msg.grid = self.current_state.grid.flatten().tolist()
        
        # Piece information
        for piece in SOMAPiece:
            piece_info = PieceInfo()
            piece_info.piece_type = piece.value
            piece_info.is_placed = piece in self.current_state.placed_pieces
            piece_info.is_remaining = piece in self.current_state.remaining_pieces
            state_msg.pieces.append(piece_info)
            
        state_msg.completion_ratio = len(self.current_state.placed_pieces) / 7.0
        
        self.state_pub.publish(state_msg)

class RobotInterfaceNode(Node):
    """ROS2 node for interfacing with Doosan M0609 robot"""
    
    def __init__(self, robot_id: str = "dsr01", robot_model: str = "m0609"):
        super().__init__('soma_robot_interface')
        
        self.robot_id = robot_id
        self.robot_model = robot_model
        
        # Robot state
        self.robot_state = RobotState(
            joint_positions=np.zeros(6),
            joint_velocities=np.zeros(6),
            end_effector_pose=np.zeros(6),
            gripper_position=0.0,
            force_torque=np.zeros(6),
            is_moving=False,
            error_state=False
        )
        
        # Publishers
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.ee_pose_pub = self.create_publisher(Pose, 'end_effector_pose', 10)
        self.wrench_pub = self.create_publisher(WrenchStamped, 'force_torque', 10)
        
        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            Float64MultiArray, 'joint_commands', self.joint_command_callback, 10
        )
        self.pose_cmd_sub = self.create_subscription(
            Pose, 'pose_commands', self.pose_command_callback, 10
        )
        
        # Initialize robot connection
        self.robot_connected = False
        if DOOSAN_AVAILABLE:
            self.initialize_robot()
            
        # State update timer
        self.state_timer = self.create_timer(0.01, self.update_robot_state)  # 100Hz
        
        self.get_logger().info(f"Robot Interface Node initialized for {robot_id} ({robot_model})")
        
    def initialize_robot(self):
        """Initialize connection to Doosan robot"""
        try:
            DR_init.__dsr__id = self.robot_id
            DR_init.__dsr__model = self.robot_model
            DR_init.__dsr__node = self
            
            # Set up robot
            set_tool("Tool Weight_2FG")
            set_tcp("2FG_TCP")
            
            # Enable robot
            set_robot_mode(ROBOT_MODE_AUTONOMOUS)
            set_robot_system(ROBOT_SYSTEM_REAL)
            
            self.robot_connected = True
            self.get_logger().info("Successfully connected to Doosan robot")
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to robot: {e}")
            self.robot_connected = False
            
    def update_robot_state(self):
        """Update robot state from hardware"""
        if not self.robot_connected:
            return
            
        try:
            # Get joint states
            joint_pos = get_current_posj()
            if joint_pos:
                self.robot_state.joint_positions = np.array(joint_pos)
                
            # Get end-effector pose
            ee_pose = get_current_posx()
            if ee_pose:
                self.robot_state.end_effector_pose = np.array(ee_pose)
                
            # Get force/torque
            try:
                ft = get_tool_force()
                if ft:
                    self.robot_state.force_torque = np.array(ft)
            except:
                pass  # Force sensor might not be available
                
            # Publish states
            self.publish_joint_states()
            self.publish_ee_pose()
            self.publish_wrench()
            
        except Exception as e:
            self.get_logger().warn(f"Error updating robot state: {e}")
            
    def publish_joint_states(self):
        """Publish joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        
        msg.name = [f"joint_{i+1}" for i in range(6)]
        msg.position = self.robot_state.joint_positions.tolist()
        msg.velocity = self.robot_state.joint_velocities.tolist()
        
        self.joint_state_pub.publish(msg)
        
    def publish_ee_pose(self):
        """Publish end-effector pose"""
        msg = Pose()
        
        # Position
        msg.position.x = self.robot_state.end_effector_pose[0] / 1000.0  # mm to m
        msg.position.y = self.robot_state.end_effector_pose[1] / 1000.0
        msg.position.z = self.robot_state.end_effector_pose[2] / 1000.0
        
        # Orientation (Euler to quaternion conversion)
        roll = np.radians(self.robot_state.end_effector_pose[3])
        pitch = np.radians(self.robot_state.end_effector_pose[4])
        yaw = np.radians(self.robot_state.end_effector_pose[5])
        
        # Simplified quaternion conversion
        msg.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        msg.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        msg.orientation.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        msg.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        
        self.ee_pose_pub.publish(msg)
        
    def publish_wrench(self):
        """Publish force/torque wrench"""
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "tool_frame"
        
        msg.wrench.force.x = self.robot_state.force_torque[0]
        msg.wrench.force.y = self.robot_state.force_torque[1]
        msg.wrench.force.z = self.robot_state.force_torque[2]
        msg.wrench.torque.x = self.robot_state.force_torque[3]
        msg.wrench.torque.y = self.robot_state.force_torque[4]
        msg.wrench.torque.z = self.robot_state.force_torque[5]
        
        self.wrench_pub.publish(msg)
        
    def joint_command_callback(self, msg: Float64MultiArray):
        """Handle joint position commands"""
        if not self.robot_connected or len(msg.data) != 6:
            return
            
        try:
            joint_positions = list(msg.data)
            movej(joint_positions, vel=30, acc=30)
            self.get_logger().debug(f"Executing joint command: {joint_positions}")
        except Exception as e:
            self.get_logger().error(f"Joint command failed: {e}")
            
    def pose_command_callback(self, msg: Pose):
        """Handle Cartesian pose commands"""
        if not self.robot_connected:
            return
            
        try:
            # Convert pose message to robot format
            x = msg.position.x * 1000  # m to mm
            y = msg.position.y * 1000
            z = msg.position.z * 1000
            
            # Quaternion to Euler (simplified)
            # This should use proper quaternion to Euler conversion
            roll = 0.0
            pitch = 0.0  
            yaw = 0.0
            
            target_pose = [x, y, z, roll, pitch, yaw]
            movel(posx(target_pose), vel=50, acc=50)
            
            self.get_logger().debug(f"Executing pose command: {target_pose}")
            
        except Exception as e:
            self.get_logger().error(f"Pose command failed: {e}")
            
    def move_to_pose(self, pose: List[float], velocity: float = 50) -> bool:
        """Move robot to specified 6DOF pose"""
        if not self.robot_connected:
            return False
            
        try:
            movel(posx(pose), vel=velocity, acc=velocity)
            return True
        except Exception as e:
            self.get_logger().error(f"Move to pose failed: {e}")
            return False
            
    def get_current_pose(self) -> Optional[List[float]]:
        """Get current robot pose"""
        if not self.robot_connected:
            return None
            
        try:
            return get_current_posx()
        except Exception as e:
            self.get_logger().error(f"Get current pose failed: {e}")
            return None

class CameraInterfaceNode(Node):
    """ROS2 node for RGB-D camera integration"""
    
    def __init__(self, camera_topic: str = "/camera/color/image_raw", 
                 depth_topic: str = "/camera/depth/image_raw"):
        super().__init__('soma_camera_interface')
        
        self.bridge = CvBridge()
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, camera_topic, self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10
        )
        
        # Publishers (processed images)
        self.processed_rgb_pub = self.create_publisher(Image, 'soma/rgb_processed', 10)
        self.processed_depth_pub = self.create_publisher(Image, 'soma/depth_processed', 10)
        
        # Current images
        self.current_rgb: Optional[np.ndarray] = None
        self.current_depth: Optional[np.ndarray] = None
        self.last_rgb_time = time.time()
        self.last_depth_time = time.time()
        
        self.get_logger().info("Camera Interface Node initialized")
        
    def rgb_callback(self, msg: Image):
        """Handle RGB image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Resize to standard size (224x224) for neural networks
            resized = cv2.resize(cv_image, (224, 224))
            
            self.current_rgb = resized
            self.last_rgb_time = time.time()
            
            # Publish processed image
            processed_msg = self.bridge.cv2_to_imgmsg(resized, "bgr8")
            processed_msg.header = msg.header
            self.processed_rgb_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f"RGB processing failed: {e}")
            
    def depth_callback(self, msg: Image):
        """Handle depth image messages"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            
            # Resize and normalize depth
            resized = cv2.resize(cv_image, (224, 224))
            
            # Convert to meters if needed and normalize
            if resized.dtype == np.uint16:
                resized = resized.astype(np.float32) / 1000.0  # mm to m
                
            self.current_depth = resized
            self.last_depth_time = time.time()
            
            # Publish processed depth
            processed_msg = self.bridge.cv2_to_imgmsg(resized, "passthrough")
            processed_msg.header = msg.header
            self.processed_depth_pub.publish(processed_msg)
            
        except Exception as e:
            self.get_logger().error(f"Depth processing failed: {e}")
            
    def get_current_rgbd(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get current RGB-D images"""
        # Check if images are recent (within 1 second)
        current_time = time.time()
        rgb_fresh = (current_time - self.last_rgb_time) < 1.0
        depth_fresh = (current_time - self.last_depth_time) < 1.0
        
        rgb = self.current_rgb if rgb_fresh else None
        depth = self.current_depth if depth_fresh else None
        
        return rgb, depth

class SOMAAssemblyActionServer(Node):
    """Action server for hierarchical SOMA cube assembly"""
    
    def __init__(self, trainer: HierarchicalSOMATrainer):
        super().__init__('soma_assembly_action_server')
        
        self.trainer = trainer
        
        # Action server
        self._action_server = ActionServer(
            self,
            AssembleSOMACube,
            'assemble_soma_cube',
            self.execute_assembly_callback
        )
        
        # Service for planning only
        self.plan_service = self.create_service(
            PlanSequence, 
            'plan_assembly_sequence',
            self.plan_sequence_callback
        )
        
        # Service for reset
        self.reset_service = self.create_service(
            ResetAssembly,
            'reset_assembly',
            self.reset_assembly_callback
        )
        
        self.get_logger().info("SOMA Assembly Action Server initialized")
        
    def execute_assembly_callback(self, goal_handle):
        """Execute complete SOMA cube assembly"""
        self.get_logger().info('Executing SOMA cube assembly...')
        
        request = goal_handle.request
        feedback_msg = AssembleSOMACube.Feedback()
        result = AssembleSOMACube.Result()
        
        try:
            # Execute hierarchical episode
            episode_result = self.trainer.execute_hierarchical_episode()
            
            # Update feedback during execution (simplified)
            feedback_msg.current_piece = 1
            feedback_msg.total_pieces = request.num_pieces
            feedback_msg.progress_percentage = 0.5
            goal_handle.publish_feedback(feedback_msg)
            
            # Set result
            result.success = episode_result['assembly_successful']
            result.assembly_time = episode_result['assembly_time']
            result.final_reward = episode_result['episode_reward']
            result.pieces_placed = episode_result['pieces_placed']
            
            if result.success:
                goal_handle.succeed()
                self.get_logger().info(f'Assembly succeeded in {result.assembly_time:.1f}s')
            else:
                goal_handle.abort()
                self.get_logger().warn('Assembly failed')
                
            return result
            
        except Exception as e:
            self.get_logger().error(f'Assembly execution failed: {e}')
            goal_handle.abort()
            result.success = False
            result.error_message = str(e)
            return result
            
    def plan_sequence_callback(self, request, response):
        """Plan assembly sequence without execution"""
        try:
            # Create initial state
            initial_state = AssemblyState(
                grid=np.zeros((3, 3, 3), dtype=int),
                placed_pieces=[],
                remaining_pieces=list(SOMAPiece)[:request.num_pieces],
                current_sequence=[]
            )
            
            # Plan sequence
            planned_sequence = self.trainer.upper_planner.plan_assembly_sequence(initial_state)
            
            # Convert to ROS message format
            sequence_msg = AssemblySequence()
            sequence_msg.header.stamp = self.get_clock().now().to_msg()
            
            for action in planned_sequence:
                piece_msg = PieceInfo()
                piece_msg.piece_type = action.piece.value
                piece_msg.target_position = list(action.target_position)
                piece_msg.target_pose = list(action.target_pose)
                piece_msg.rotation_index = action.rotation_index
                sequence_msg.sequence.append(piece_msg)
                
            response.sequence = sequence_msg
            response.success = True
            response.planning_time = 1.0  # Placeholder
            
            self.get_logger().info(f'Planned sequence with {len(planned_sequence)} pieces')
            
        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')
            response.success = False
            response.error_message = str(e)
            
        return response
        
    def reset_assembly_callback(self, request, response):
        """Reset assembly environment"""
        try:
            # Reset trainer environment
            self.trainer.reset_environment()
            
            response.success = True
            self.get_logger().info('Assembly environment reset')
            
        except Exception as e:
            self.get_logger().error(f'Reset failed: {e}')
            response.success = False
            response.error_message = str(e)
            
        return response

class ROSIntegratedSOMASystem:
    """
    Complete ROS2-integrated SOMA cube assembly system
    
    Coordinates all ROS2 nodes for hierarchical RL-based assembly
    """
    
    def __init__(self, 
                 robot_id: str = "dsr01", 
                 robot_model: str = "m0609",
                 use_real_robot: bool = True,
                 use_real_camera: bool = True):
        
        # Initialize ROS2
        rclpy.init()
        
        # Create executor
        self.executor = MultiThreadedExecutor()
        
        # Initialize hierarchical trainer
        self.trainer = HierarchicalSOMATrainer(
            experiment_name="ros2_soma_assembly",
            device='cuda'
        )
        
        # Create ROS2 nodes
        self.nodes = []
        
        # Visualization node (always active)
        self.viz_node = SOMAVisualizationNode()
        self.nodes.append(self.viz_node)
        
        # Robot interface (if using real robot)
        if use_real_robot:
            self.robot_node = RobotInterfaceNode(robot_id, robot_model)
            self.nodes.append(self.robot_node)
        else:
            self.robot_node = None
            
        # Camera interface (if using real camera)
        if use_real_camera:
            self.camera_node = CameraInterfaceNode()
            self.nodes.append(self.camera_node)
        else:
            self.camera_node = None
            
        # Action server
        self.action_server_node = SOMAAssemblyActionServer(self.trainer)
        self.nodes.append(self.action_server_node)
        
        # Add all nodes to executor
        for node in self.nodes:
            self.executor.add_node(node)
            
        # Start executor in separate thread
        self.executor_thread = threading.Thread(target=self.executor.spin)
        self.executor_thread.daemon = True
        self.executor_thread.start()
        
        print("ROS2 SOMA Assembly System initialized")
        print(f"  Robot: {'Real M0609' if use_real_robot else 'Simulation'}")
        print(f"  Camera: {'Real RGB-D' if use_real_camera else 'Simulation'}")
        print("  Action server: /assemble_soma_cube")
        print("  Planning service: /plan_assembly_sequence")
        
    def update_visualization(self, state: AssemblyState, sequence: List[UpperLevelAction] = None):
        """Update visualization with current state"""
        self.viz_node.update_assembly_state(state)
        if sequence:
            self.viz_node.update_planned_sequence(sequence)
            
    def get_robot_state(self) -> Optional[RobotState]:
        """Get current robot state"""
        if self.robot_node:
            return self.robot_node.robot_state
        return None
        
    def get_camera_images(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Get current camera images"""
        if self.camera_node:
            return self.camera_node.get_current_rgbd()
        return None, None
        
    def shutdown(self):
        """Shutdown ROS2 system"""
        self.executor.shutdown()
        self.executor_thread.join(timeout=5.0)
        
        for node in self.nodes:
            node.destroy_node()
            
        rclpy.shutdown()
        print("ROS2 SOMA Assembly System shutdown complete")

def main():
    """Main function to start ROS2 SOMA system"""
    
    import argparse
    parser = argparse.ArgumentParser(description='ROS2 Hierarchical SOMA Cube Assembly')
    parser.add_argument('--robot-id', default='dsr01', help='Robot ID')
    parser.add_argument('--robot-model', default='m0609', help='Robot model') 
    parser.add_argument('--sim-robot', action='store_true', help='Use simulated robot')
    parser.add_argument('--sim-camera', action='store_true', help='Use simulated camera')
    parser.add_argument('--train', action='store_true', help='Run training mode')
    parser.add_argument('--episodes', type=int, default=10000, help='Training episodes')
    
    args = parser.parse_args()
    
    # Create integrated system
    system = ROSIntegratedSOMASystem(
        robot_id=args.robot_id,
        robot_model=args.robot_model,
        use_real_robot=not args.sim_robot,
        use_real_camera=not args.sim_camera
    )
    
    try:
        if args.train:
            print(f"Starting training for {args.episodes} episodes...")
            system.trainer.train(args.episodes)
        else:
            print("System ready for assembly requests...")
            print("Send action goals to /assemble_soma_cube")
            
            # Keep system running
            while rclpy.ok():
                time.sleep(1.0)
                
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        system.shutdown()

if __name__ == '__main__':
    main()
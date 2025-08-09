#include "soma_cube_rl_bridge/motion_proxy_node.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace soma_cube_rl_bridge
{

MotionProxyNode::MotionProxyNode(const rclcpp::NodeOptions & options)
: Node("motion_proxy_node", options),
  is_connected_(false)
{
  declareParameters();
  
  // Initialize service servers (RL-facing interfaces)
  move_joint_server_ = create_service<dsr_msgs2::srv::MoveJoint>(
    "soma_cube_rl_bridge/move_joint",
    std::bind(&MotionProxyNode::moveJointCallback, this, _1, _2));
    
  move_stop_server_ = create_service<dsr_msgs2::srv::MoveStop>(
    "soma_cube_rl_bridge/move_stop", 
    std::bind(&MotionProxyNode::moveStopCallback, this, _1, _2));
  
  // Initialize service clients (Doosan-facing interfaces)
  move_joint_client_ = create_client<dsr_msgs2::srv::MoveJoint>(
    "/motion/move_joint");
  move_stop_client_ = create_client<dsr_msgs2::srv::MoveStop>(
    "/motion/move_stop");
  get_posj_client_ = create_client<dsr_msgs2::srv::GetCurrentPosj>(
    "/aux_control/get_current_posj");
  get_posx_client_ = create_client<dsr_msgs2::srv::GetCurrentPosx>(
    "/aux_control/get_current_posx");
  safety_client_ = create_client<soma_cube_rl_bridge::srv::GetSafetyState>(
    "safety/get_state");
  
  // Initialize publishers (RL-facing state feedback)
  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    "soma_cube_rl_bridge/joint_states", 
    rclcpp::SensorDataQoS());
  tcp_pose_pub_ = create_publisher<geometry_msgs::msg::Pose>(
    "soma_cube_rl_bridge/tcp_pose",
    rclcpp::SensorDataQoS());
  
  // Initialize subscribers (Doosan-facing state feedback) 
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&MotionProxyNode::jointStateCallback, this, _1));
    
  current_posx_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "/msg/current_posx",
    rclcpp::SensorDataQoS(),
    std::bind(&MotionProxyNode::currentPosxCallback, this, _1));
    
  robot_state_sub_ = create_subscription<dsr_msgs2::msg::RobotState>(
    "/msg/robot_state",
    rclcpp::SensorDataQoS(),
    std::bind(&MotionProxyNode::robotStateCallback, this, _1));
    
  robot_error_sub_ = create_subscription<dsr_msgs2::msg::RobotError>(
    "/error",
    rclcpp::QoS(10).reliable().durability_volatile(),
    std::bind(&MotionProxyNode::robotErrorCallback, this, _1));
  
  // Initialize timer for state publishing
  state_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate_hz_),
    std::bind(&MotionProxyNode::stateTimerCallback, this));
  
  // Initialize joint state message
  current_joint_state_.name.resize(6);
  current_joint_state_.position.resize(6);
  current_joint_state_.velocity.resize(6);
  current_joint_state_.effort.resize(6);
  
  for (int i = 0; i < 6; ++i) {
    current_joint_state_.name[i] = "joint_" + std::to_string(i + 1);
    current_joint_state_.position[i] = 0.0;
    current_joint_state_.velocity[i] = 0.0;
    current_joint_state_.effort[i] = 0.0;
  }
  
  // Try to connect to Doosan services
  if (connectToDoosanServices()) {
    is_connected_ = true;
    RCLCPP_INFO(this->get_logger(), "Motion Proxy Node initialized and connected to Doosan services");
  } else {
    RCLCPP_WARN(this->get_logger(), "Motion Proxy Node initialized but could not connect to all Doosan services");
  }
}

void MotionProxyNode::declareParameters()
{
  declare_parameter("publish_rate_hz", 20.0);
  declare_parameter("vel_limit_deg_s", std::vector<double>{50.0, 50.0, 50.0, 50.0, 50.0, 50.0});
  declare_parameter("acc_limit_deg_s2", std::vector<double>{100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
  declare_parameter("joint_min_deg", std::vector<double>{-170.0, -135.0, -169.0, -90.0, -135.0, -360.0});
  declare_parameter("joint_max_deg", std::vector<double>{170.0, 135.0, 169.0, 90.0, 135.0, 360.0});
  declare_parameter("action_timeout_sec", 30.0);
  declare_parameter("service_wait_sec", 5.0);
  
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  vel_limit_deg_s_ = get_parameter("vel_limit_deg_s").as_double_array();
  acc_limit_deg_s2_ = get_parameter("acc_limit_deg_s2").as_double_array();
  joint_min_deg_ = get_parameter("joint_min_deg").as_double_array();
  joint_max_deg_ = get_parameter("joint_max_deg").as_double_array();
  action_timeout_sec_ = get_parameter("action_timeout_sec").as_double();
  service_wait_sec_ = get_parameter("service_wait_sec").as_double();
}

void MotionProxyNode::moveJointCallback(
  const std::shared_ptr<dsr_msgs2::srv::MoveJoint::Request> request,
  std::shared_ptr<dsr_msgs2::srv::MoveJoint::Response> response)
{
  RCLCPP_DEBUG(this->get_logger(), "MoveJoint request received");
  
  // Check safety state first
  if (!checkSafetyState()) {
    response->success = false;
    // Response only includes success field
    RCLCPP_WARN(this->get_logger(), "MoveJoint rejected due to safety state");
    return;
  }
  
  // Validate joint limits
  if (!validateJointLimits(request->pos)) {
    response->success = false;
    // Response only includes success field
    RCLCPP_WARN(this->get_logger(), "MoveJoint rejected due to joint limits");
    return;
  }
  
  // Validate velocity limits
  if (!validateVelocityLimits(request->vel)) {
    response->success = false;
    // Response only includes success field
    RCLCPP_WARN(this->get_logger(), "MoveJoint rejected due to velocity limits");
    return;
  }
  
  // Validate acceleration limits
  if (!validateAccelerationLimits(request->acc)) {
    response->success = false;
    // Response only includes success field
    RCLCPP_WARN(this->get_logger(), "MoveJoint rejected due to acceleration limits");
    return;
  }
  
  // Forward to Doosan service
  if (!move_joint_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
    response->success = false;
    // Response only includes success field
    RCLCPP_ERROR(this->get_logger(), "Doosan move_joint service not available");
    return;
  }
  
  auto future = move_joint_client_->async_send_request(request);
  
  // Wait for response with timeout
  auto timeout = std::chrono::duration<double>(action_timeout_sec_);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) 
      == rclcpp::FutureReturnCode::SUCCESS) {
    auto doosan_response = future.get();
    response->success = doosan_response->success;
    // Copy success field only
    
    RCLCPP_DEBUG(this->get_logger(), "MoveJoint completed: success=%s", 
      response->success ? "true" : "false");
  } else {
    response->success = false;
    // Response only includes success field
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for Doosan move_joint response");
  }
}

void MotionProxyNode::moveStopCallback(
  const std::shared_ptr<dsr_msgs2::srv::MoveStop::Request> request,
  std::shared_ptr<dsr_msgs2::srv::MoveStop::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "MoveStop request received");
  
  // Forward stop request immediately (no safety checks needed for stop)
  if (!move_stop_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
    response->success = false;
    // Response only includes success field
    RCLCPP_ERROR(this->get_logger(), "Doosan move_stop service not available");
    return;
  }
  
  auto future = move_stop_client_->async_send_request(request);
  
  // Wait for response with timeout
  auto timeout = std::chrono::duration<double>(action_timeout_sec_);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) 
      == rclcpp::FutureReturnCode::SUCCESS) {
    auto doosan_response = future.get();
    response->success = doosan_response->success;
    // Copy success field only
    
    RCLCPP_INFO(this->get_logger(), "MoveStop completed: success=%s", 
      response->success ? "true" : "false");
  } else {
    response->success = false;
    // Response only includes success field
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for Doosan move_stop response");
  }
}

void MotionProxyNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  current_joint_state_ = *msg;
  current_joint_state_.header.stamp = this->get_clock()->now();
}

void MotionProxyNode::currentPosxCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 6) {
    current_tcp_pose_.position.x = msg->data[0];
    current_tcp_pose_.position.y = msg->data[1]; 
    current_tcp_pose_.position.z = msg->data[2];
    
    // Convert Euler angles to quaternion (rx, ry, rz in degrees)
    double rx = msg->data[3] * M_PI / 180.0;
    double ry = msg->data[4] * M_PI / 180.0;
    double rz = msg->data[5] * M_PI / 180.0;
    
    // Simplified quaternion conversion (ZYX order)
    double cy = cos(rz * 0.5);
    double sy = sin(rz * 0.5);
    double cp = cos(ry * 0.5);
    double sp = sin(ry * 0.5);
    double cr = cos(rx * 0.5);
    double sr = sin(rx * 0.5);
    
    current_tcp_pose_.orientation.w = cr * cp * cy + sr * sp * sy;
    current_tcp_pose_.orientation.x = sr * cp * cy - cr * sp * sy;
    current_tcp_pose_.orientation.y = cr * sp * cy + sr * cp * sy;
    current_tcp_pose_.orientation.z = cr * cp * sy - sr * sp * cy;
  }
}

void MotionProxyNode::robotStateCallback(const dsr_msgs2::msg::RobotState::SharedPtr msg)
{
  // Update current joint state from robot state if available
  if (msg->current_posj.size() >= 6) {
    for (size_t i = 0; i < 6 && i < msg->current_posj.size(); ++i) {
      current_joint_state_.position[i] = msg->current_posj[i] * M_PI / 180.0; // Convert to radians
    }
  }
  
  if (msg->current_velj.size() >= 6) {
    for (size_t i = 0; i < 6 && i < msg->current_velj.size(); ++i) {
      current_joint_state_.velocity[i] = msg->current_velj[i] * M_PI / 180.0; // Convert to radians/s
    }
  }
}

void MotionProxyNode::robotErrorCallback(const dsr_msgs2::msg::RobotError::SharedPtr msg)
{
  if (msg->level >= 2) { // ERROR level or higher
    RCLCPP_WARN(this->get_logger(), "Robot error received in motion proxy: level=%d", msg->level);
  }
}

void MotionProxyNode::stateTimerCallback()
{
  // Publish current joint state
  current_joint_state_.header.stamp = this->get_clock()->now();
  current_joint_state_.header.frame_id = "base_link";
  joint_state_pub_->publish(current_joint_state_);
  
  // Publish current TCP pose
  tcp_pose_pub_->publish(current_tcp_pose_);
}

bool MotionProxyNode::validateJointLimits(const std::array<double, 6> & joint_positions)
{
  if (joint_positions.size() != joint_min_deg_.size()) {
    return false;
  }
  
  for (size_t i = 0; i < joint_positions.size(); ++i) {
    if (joint_positions[i] < joint_min_deg_[i] || joint_positions[i] > joint_max_deg_[i]) {
      RCLCPP_WARN(this->get_logger(), "Joint %zu position %.2f exceeds limits [%.2f, %.2f]",
        i, joint_positions[i], joint_min_deg_[i], joint_max_deg_[i]);
      return false;
    }
  }
  return true;
}

bool MotionProxyNode::validateVelocityLimits(double velocity)
{
  for (const auto & vel_limit : vel_limit_deg_s_) {
    if (std::abs(velocity) > vel_limit) {
      RCLCPP_WARN(this->get_logger(), "Velocity %.2f exceeds limit %.2f", velocity, vel_limit);
      return false;
    }
  }
  return true;
}

bool MotionProxyNode::validateAccelerationLimits(double acceleration)
{
  for (const auto & acc_limit : acc_limit_deg_s2_) {
    if (std::abs(acceleration) > acc_limit) {
      RCLCPP_WARN(this->get_logger(), "Acceleration %.2f exceeds limit %.2f", acceleration, acc_limit);
      return false;
    }
  }
  return true;
}

bool MotionProxyNode::checkSafetyState()
{
  if (!safety_client_->wait_for_service(std::chrono::duration<double>(1.0))) {
    RCLCPP_WARN(this->get_logger(), "Safety service not available");
    return false;
  }
  
  auto request = std::make_shared<soma_cube_rl_bridge::srv::GetSafetyState::Request>();
  auto future = safety_client_->async_send_request(request);
  
  auto timeout = std::chrono::duration<double>(2.0);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) 
      == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    return response->safe_to_move;
  } else {
    RCLCPP_WARN(this->get_logger(), "Timeout waiting for safety state");
    return false;
  }
}

bool MotionProxyNode::connectToDoosanServices()
{
  bool all_connected = true;
  
  RCLCPP_INFO(this->get_logger(), "Connecting to Doosan services...");
  
  if (!move_joint_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
    RCLCPP_WARN(this->get_logger(), "move_joint service not available");
    all_connected = false;
  }
  
  if (!move_stop_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
    RCLCPP_WARN(this->get_logger(), "move_stop service not available");
    all_connected = false;
  }
  
  if (!get_posj_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
    RCLCPP_WARN(this->get_logger(), "get_current_posj service not available");
    all_connected = false;
  }
  
  if (!get_posx_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
    RCLCPP_WARN(this->get_logger(), "get_current_posx service not available");
    all_connected = false;
  }
  
  return all_connected;
}

} // namespace soma_cube_rl_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<soma_cube_rl_bridge::MotionProxyNode>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Exception in motion proxy: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
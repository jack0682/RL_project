#include "soma_cube_rl_bridge/safety_monitor_node.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace soma_cube_rl_bridge
{

SafetyMonitorNode::SafetyMonitorNode(const rclcpp::NodeOptions & options)
: Node("safety_monitor_node", options),
  safe_to_move_(false),
  safety_reason_("Initializing..."),
  current_robot_mode_(0),
  current_robot_state_(0),
  emergency_stop_(false),
  protective_stop_(false),
  joint_limit_violations_(6, false)
{
  declareParameters();
  
  // Initialize service servers
  get_safety_state_server_ = create_service<soma_cube_rl_bridge::srv::GetSafetyState>(
    "safety/get_state",
    std::bind(&SafetyMonitorNode::getSafetyStateCallback, this, _1, _2));
  
  // Initialize service clients
  get_robot_state_client_ = create_client<dsr_msgs2::srv::GetRobotState>(
    "/system/get_robot_state");
  get_robot_mode_client_ = create_client<dsr_msgs2::srv::GetRobotMode>(
    "/system/get_robot_mode");
  
  // Initialize publishers
  safety_state_pub_ = create_publisher<soma_cube_rl_bridge::msg::SafetyState>(
    "safety/state", rclcpp::QoS(10).reliable().durability_volatile());
  
  // Initialize subscribers
  robot_state_sub_ = create_subscription<dsr_msgs2::msg::RobotState>(
    "/msg/robot_state", 
    rclcpp::SensorDataQoS(),
    std::bind(&SafetyMonitorNode::robotStateCallback, this, _1));
    
  robot_error_sub_ = create_subscription<dsr_msgs2::msg::RobotError>(
    "/error",
    rclcpp::QoS(10).reliable().durability_volatile(),
    std::bind(&SafetyMonitorNode::robotErrorCallback, this, _1));
    
  robot_stop_sub_ = create_subscription<dsr_msgs2::msg::RobotStop>(
    "/msg/robot_stop",
    rclcpp::QoS(10).reliable().durability_volatile(), 
    std::bind(&SafetyMonitorNode::robotStopCallback, this, _1));
  
  // Initialize timer
  monitor_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / monitor_rate_hz_),
    std::bind(&SafetyMonitorNode::monitorTimerCallback, this));
  
  last_event_time_ = getCurrentTime();
  
  RCLCPP_INFO(this->get_logger(), "Safety Monitor Node initialized");
}

void SafetyMonitorNode::declareParameters()
{
  declare_parameter("monitor_rate_hz", 10.0);
  declare_parameter("joint_min_deg", std::vector<double>{-170.0, -135.0, -169.0, -90.0, -135.0, -360.0});
  declare_parameter("joint_max_deg", std::vector<double>{170.0, 135.0, 169.0, 90.0, 135.0, 360.0});
  declare_parameter("tcp_workspace_min", std::vector<double>{-1000.0, -1000.0, 0.0});
  declare_parameter("tcp_workspace_max", std::vector<double>{1000.0, 1000.0, 2000.0});
  
  monitor_rate_hz_ = get_parameter("monitor_rate_hz").as_double();
  joint_min_deg_ = get_parameter("joint_min_deg").as_double_array();
  joint_max_deg_ = get_parameter("joint_max_deg").as_double_array();
  tcp_workspace_min_ = get_parameter("tcp_workspace_min").as_double_array();
  tcp_workspace_max_ = get_parameter("tcp_workspace_max").as_double_array();
}

void SafetyMonitorNode::getSafetyStateCallback(
  const std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState::Request> request,
  std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState::Response> response)
{
  (void)request;
  
  response->safe_to_move = safe_to_move_;
  response->reason = safety_reason_;
  response->last_event_time = last_event_time_;
  
  RCLCPP_DEBUG(this->get_logger(), "Safety state requested: safe=%s, reason=%s", 
    safe_to_move_ ? "true" : "false", safety_reason_.c_str());
}

void SafetyMonitorNode::robotStateCallback(const dsr_msgs2::msg::RobotState::SharedPtr msg)
{
  current_robot_state_ = msg->robot_state;
  current_robot_mode_ = msg->actual_mode;
  
  // Check joint limit violations
  if (msg->current_posj.size() >= joint_limit_violations_.size()) {
    for (size_t i = 0; i < joint_limit_violations_.size(); ++i) {
      double joint_deg = msg->current_posj[i];
      joint_limit_violations_[i] = (joint_deg < joint_min_deg_[i]) || (joint_deg > joint_max_deg_[i]);
    }
  }
  
  updateSafetyState();
}

void SafetyMonitorNode::robotErrorCallback(const dsr_msgs2::msg::RobotError::SharedPtr msg)
{
  if (msg->level >= 2) { // ERROR level or higher
    safe_to_move_ = false;
    safety_reason_ = "Robot error: " + msg->msg1;
    last_event_time_ = getCurrentTime();
    
    RCLCPP_WARN(this->get_logger(), "Robot error received: %s", safety_reason_.c_str());
  }
}

void SafetyMonitorNode::robotStopCallback(const dsr_msgs2::msg::RobotStop::SharedPtr msg)
{
  emergency_stop_ = (msg->stop_mode == 1); // Emergency stop
  protective_stop_ = (msg->stop_mode == 2); // Protective stop
  
  if (emergency_stop_ || protective_stop_) {
    safe_to_move_ = false;
    safety_reason_ = emergency_stop_ ? "Emergency stop active" : "Protective stop active";
    last_event_time_ = getCurrentTime();
    
    RCLCPP_WARN(this->get_logger(), "Stop condition: %s", safety_reason_.c_str());
  }
}

void SafetyMonitorNode::monitorTimerCallback()
{
  updateSafetyState();
  publishSafetyState();
}

void SafetyMonitorNode::updateSafetyState()
{
  bool was_safe = safe_to_move_;
  std::string previous_reason = safety_reason_;
  
  // Reset to safe state, then check all conditions
  safe_to_move_ = true;
  safety_reason_ = "All systems normal";
  
  // Check emergency/protective stops
  if (emergency_stop_) {
    safe_to_move_ = false;
    safety_reason_ = "Emergency stop active";
  } else if (protective_stop_) {
    safe_to_move_ = false;
    safety_reason_ = "Protective stop active";
  }
  
  // Check robot mode (should be AUTONOMOUS for RL operation)
  else if (!checkRobotMode()) {
    safe_to_move_ = false;
    safety_reason_ = "Robot not in AUTONOMOUS mode";
  }
  
  // Check robot state
  else if (!checkRobotState()) {
    safe_to_move_ = false;
    safety_reason_ = "Robot not in valid operational state";
  }
  
  // Check joint limit violations
  else {
    for (size_t i = 0; i < joint_limit_violations_.size(); ++i) {
      if (joint_limit_violations_[i]) {
        safe_to_move_ = false;
        safety_reason_ = "Joint " + std::to_string(i) + " limit violation";
        break;
      }
    }
  }
  
  // Update event time if safety state changed
  if (was_safe != safe_to_move_ || previous_reason != safety_reason_) {
    last_event_time_ = getCurrentTime();
    RCLCPP_INFO(this->get_logger(), "Safety state changed: safe=%s, reason=%s",
      safe_to_move_ ? "true" : "false", safety_reason_.c_str());
  }
}

bool SafetyMonitorNode::checkJointLimits(const std::vector<double> & joint_positions)
{
  if (joint_positions.size() != joint_min_deg_.size()) {
    return false;
  }
  
  for (size_t i = 0; i < joint_positions.size(); ++i) {
    if (joint_positions[i] < joint_min_deg_[i] || joint_positions[i] > joint_max_deg_[i]) {
      return false;
    }
  }
  return true;
}

bool SafetyMonitorNode::checkTcpWorkspace(const std::vector<double> & tcp_position)
{
  if (tcp_position.size() < 3) {
    return false;
  }
  
  for (size_t i = 0; i < 3; ++i) {
    if (tcp_position[i] < tcp_workspace_min_[i] || tcp_position[i] > tcp_workspace_max_[i]) {
      return false;
    }
  }
  return true;
}

bool SafetyMonitorNode::checkRobotMode()
{
  // Mode 1 = MANUAL, Mode 2 = AUTONOMOUS, Mode 3 = MEASURE
  return (current_robot_mode_ == 2);
}

bool SafetyMonitorNode::checkRobotState()
{
  // State 1 = STANDBY, State 2 = MOVING, State 3 = SAFE_STOP, etc.
  // Allow STANDBY and MOVING states for RL operation
  return (current_robot_state_ == 1 || current_robot_state_ == 2);
}

void SafetyMonitorNode::publishSafetyState()
{
  auto msg = std::make_unique<soma_cube_rl_bridge::msg::SafetyState>();
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = "base_link";
  msg->safe_to_move = safe_to_move_;
  msg->reason = safety_reason_;
  msg->last_event_time = last_event_time_;
  msg->robot_mode = current_robot_mode_;
  msg->robot_state = current_robot_state_;
  msg->emergency_stop = emergency_stop_;
  msg->protective_stop = protective_stop_;
  msg->joint_limit_violations = joint_limit_violations_;
  
  safety_state_pub_->publish(std::move(msg));
}

builtin_interfaces::msg::Time SafetyMonitorNode::getCurrentTime()
{
  auto now = this->get_clock()->now();
  return now;
}

} // namespace soma_cube_rl_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<soma_cube_rl_bridge::SafetyMonitorNode>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Exception in safety monitor: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
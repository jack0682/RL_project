#include "soma_cube_rl_bridge/rl_env_node.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <random>
#include <sstream>
#include <iomanip>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace soma_cube_rl_bridge
{

RLEnvironmentNode::RLEnvironmentNode(const rclcpp::NodeOptions & options)
: Node("rl_env_node", options),
  rng_(std::chrono::steady_clock::now().time_since_epoch().count())
{
  declareParameters();
  loadRewardWeights();
  
  // Initialize service servers (RL-facing)
  rl_reset_server_ = create_service<soma_cube_rl_bridge::srv::RLReset>(
    "/rl/reset",
    std::bind(&RLEnvironmentNode::rlResetCallback, this, _1, _2));
    
  rl_step_server_ = create_service<soma_cube_rl_bridge::srv::RLStep>(
    "/rl/step",
    std::bind(&RLEnvironmentNode::rlStepCallback, this, _1, _2));
  
  // Initialize service clients (proxy-facing)
  move_joint_client_ = create_client<dsr_msgs2::srv::MoveJoint>(
    "soma_cube_rl_bridge/move_joint");
  get_posj_client_ = create_client<dsr_msgs2::srv::GetCurrentPosj>(
    "/aux_control/get_current_posj");
  get_posx_client_ = create_client<dsr_msgs2::srv::GetCurrentPosx>(
    "/aux_control/get_current_posx");
  get_tool_force_client_ = create_client<dsr_msgs2::srv::GetToolForce>(
    "/aux_control/get_tool_force");
  safety_client_ = create_client<soma_cube_rl_bridge::srv::GetSafetyState>(
    "safety/get_state");
  
  // Initialize subscribers (state feedback)
  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "soma_cube_rl_bridge/joint_states",
    rclcpp::SensorDataQoS(),
    std::bind(&RLEnvironmentNode::jointStateCallback, this, _1));
    
  tcp_pose_sub_ = create_subscription<geometry_msgs::msg::Pose>(
    "soma_cube_rl_bridge/tcp_pose",
    rclcpp::SensorDataQoS(),
    std::bind(&RLEnvironmentNode::tcpPoseCallback, this, _1));
    
  tool_force_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "/msg/tool_force",
    rclcpp::SensorDataQoS(),
    std::bind(&RLEnvironmentNode::toolForceCallback, this, _1));
  
  // Initialize publishers
  observation_pub_ = create_publisher<soma_cube_rl_bridge::msg::RLObservation>(
    "/rl/observation",
    rclcpp::QoS(10).reliable().durability_volatile());
  
  // Initialize internal state
  current_joint_state_.position.resize(6, 0.0);
  current_joint_state_.velocity.resize(6, 0.0);
  current_tool_force_.resize(6, 0.0);
  
  // Initialize episode state
  episode_state_.step_count = 0;
  episode_state_.total_reward = 0.0;
  episode_state_.is_done = false;
  
  initializeTask();
  
  RCLCPP_INFO(this->get_logger(), "RL Environment Node initialized");
}

void RLEnvironmentNode::declareParameters()
{
  declare_parameter("max_episode_steps", 1000);
  declare_parameter("action_timeout_sec", 30.0);
  declare_parameter("service_wait_sec", 5.0);
  declare_parameter("joint_min_deg", std::vector<double>{-170.0, -135.0, -169.0, -90.0, -135.0, -360.0});
  declare_parameter("joint_max_deg", std::vector<double>{170.0, 135.0, 169.0, 90.0, 135.0, 360.0});
  declare_parameter("vel_limit_deg_s", std::vector<double>{50.0, 50.0, 50.0, 50.0, 50.0, 50.0});
  declare_parameter("acc_limit_deg_s2", std::vector<double>{100.0, 100.0, 100.0, 100.0, 100.0, 100.0});
  
  max_episode_steps_ = get_parameter("max_episode_steps").as_int();
  action_timeout_sec_ = get_parameter("action_timeout_sec").as_double();
  service_wait_sec_ = get_parameter("service_wait_sec").as_double();
  joint_min_deg_ = get_parameter("joint_min_deg").as_double_array();
  joint_max_deg_ = get_parameter("joint_max_deg").as_double_array();
  vel_limit_deg_s_ = get_parameter("vel_limit_deg_s").as_double_array();
  acc_limit_deg_s2_ = get_parameter("acc_limit_deg_s2").as_double_array();
}

void RLEnvironmentNode::loadRewardWeights()
{
  declare_parameter("reward_weights.grasp_success", 100.0);
  declare_parameter("reward_weights.assembly_alignment", 50.0);
  declare_parameter("reward_weights.collision_penalty", -100.0);
  declare_parameter("reward_weights.time_penalty", -0.1);
  declare_parameter("reward_weights.distance_penalty", -1.0);
  declare_parameter("reward_weights.velocity_penalty", -0.01);
  
  reward_weights_.grasp_success = get_parameter("reward_weights.grasp_success").as_double();
  reward_weights_.assembly_alignment = get_parameter("reward_weights.assembly_alignment").as_double();
  reward_weights_.collision_penalty = get_parameter("reward_weights.collision_penalty").as_double();
  reward_weights_.time_penalty = get_parameter("reward_weights.time_penalty").as_double();
  reward_weights_.distance_penalty = get_parameter("reward_weights.distance_penalty").as_double();
  reward_weights_.velocity_penalty = get_parameter("reward_weights.velocity_penalty").as_double();
}

void RLEnvironmentNode::rlResetCallback(
  const std::shared_ptr<soma_cube_rl_bridge::srv::RLReset::Request> request,
  std::shared_ptr<soma_cube_rl_bridge::srv::RLReset::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "RL Reset requested with seed: %d", request->seed);
  
  // Set random seed if provided
  if (request->seed > 0) {
    rng_.seed(static_cast<unsigned int>(request->seed));
  }
  
  // Reset episode state
  episode_state_.step_count = 0;
  episode_state_.total_reward = 0.0;
  episode_state_.is_done = false;
  episode_state_.start_time = std::chrono::steady_clock::now();
  
  // Reset to home position
  if (!resetToHome()) {
    response->success = false;
    response->message = "Failed to reset robot to home position";
    RCLCPP_ERROR(this->get_logger(), "Reset failed: could not move to home position");
    return;
  }
  
  // Wait a moment for the robot to settle
  rclcpp::sleep_for(1s);
  
  // Build initial observation
  response->initial_obs = buildObservation();
  response->success = true;
  response->message = "Environment reset successfully";
  
  RCLCPP_INFO(this->get_logger(), "RL Reset completed successfully");
}

void RLEnvironmentNode::rlStepCallback(
  const std::shared_ptr<soma_cube_rl_bridge::srv::RLStep::Request> request,
  std::shared_ptr<soma_cube_rl_bridge::srv::RLStep::Response> response)
{
  auto step_start = std::chrono::steady_clock::now();
  
  RCLCPP_DEBUG(this->get_logger(), "RL Step %d requested", episode_state_.step_count);
  
  // Check if episode is already done
  if (episode_state_.is_done) {
    response->obs = buildObservation();
    response->reward = 0.0;
    response->done = true;
    response->info = "Episode already completed";
    response->success = true;
    return;
  }
  
  // Check safety state before processing action
  if (!checkSafetyState()) {
    response->obs = buildObservation();
    response->reward = reward_weights_.collision_penalty;
    response->done = true;
    response->info = "Episode terminated due to safety violation";
    response->success = true;
    episode_state_.is_done = true;
    RCLCPP_WARN(this->get_logger(), "RL Step terminated due to safety violation");
    return;
  }
  
  // Validate action dimensions
  if (request->action.size() != 6) {
    response->obs = buildObservation();
    response->reward = 0.0;
    response->done = true;
    response->info = "Invalid action dimension (expected 6, got " + std::to_string(request->action.size()) + ")";
    response->success = false;
    episode_state_.is_done = true;
    return;
  }
  
  // Process action (clamp to limits)
  std::vector<double> processed_action = processAction(request->action);
  
  // Execute action
  bool action_success = executeAction(processed_action);
  
  // Calculate reward
  double reward = calculateReward();
  episode_state_.total_reward += reward;
  
  // Check if done
  bool done = checkDone() || !action_success;
  episode_state_.is_done = done;
  
  // Build response
  response->obs = buildObservation();
  response->reward = reward;
  response->done = done;
  response->info = buildInfo();
  response->success = action_success;
  
  // Update step count
  episode_state_.step_count++;
  
  // Calculate step latency
  auto step_end = std::chrono::steady_clock::now();
  auto step_duration = std::chrono::duration_cast<std::chrono::milliseconds>(step_end - step_start);
  
  RCLCPP_DEBUG(this->get_logger(), "RL Step %d completed in %ld ms: reward=%.3f, done=%s",
    episode_state_.step_count, step_duration.count(), reward, done ? "true" : "false");
}

void RLEnvironmentNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() >= 6) {
    for (size_t i = 0; i < 6; ++i) {
      current_joint_state_.position[i] = msg->position[i];
    }
  }
  
  if (msg->velocity.size() >= 6) {
    for (size_t i = 0; i < 6; ++i) {
      current_joint_state_.velocity[i] = msg->velocity[i];
    }
  }
}

void RLEnvironmentNode::tcpPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
  current_tcp_pose_ = *msg;
}

void RLEnvironmentNode::toolForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() >= 6) {
    for (size_t i = 0; i < 6; ++i) {
      current_tool_force_[i] = msg->data[i];
    }
  }
}

std::vector<double> RLEnvironmentNode::buildObservation()
{
  std::vector<double> observation;
  
  // Joint positions (6 values, convert from radians to degrees for consistency)
  for (size_t i = 0; i < 6; ++i) {
    observation.push_back(current_joint_state_.position[i] * 180.0 / M_PI);
  }
  
  // Joint velocities (6 values, convert from rad/s to deg/s)
  for (size_t i = 0; i < 6; ++i) {
    observation.push_back(current_joint_state_.velocity[i] * 180.0 / M_PI);
  }
  
  // TCP pose (7 values: x, y, z, qx, qy, qz, qw)
  observation.push_back(current_tcp_pose_.position.x);
  observation.push_back(current_tcp_pose_.position.y);
  observation.push_back(current_tcp_pose_.position.z);
  observation.push_back(current_tcp_pose_.orientation.x);
  observation.push_back(current_tcp_pose_.orientation.y);
  observation.push_back(current_tcp_pose_.orientation.z);
  observation.push_back(current_tcp_pose_.orientation.w);
  
  // Tool force/torque (6 values)
  for (size_t i = 0; i < 6; ++i) {
    observation.push_back(current_tool_force_[i]);
  }
  
  // Task-specific observations (distance to target, alignment, etc.)
  double distance_to_target = calculateDistance(current_tcp_pose_, target_tcp_pose_);
  observation.push_back(distance_to_target);
  
  // Episode metadata
  observation.push_back(static_cast<double>(episode_state_.step_count));
  observation.push_back(static_cast<double>(max_episode_steps_ - episode_state_.step_count));
  
  return observation;
}

std::vector<double> RLEnvironmentNode::processAction(const std::vector<double> & action)
{
  std::vector<double> processed_action;
  processed_action.reserve(6);
  
  // Clamp joint positions to valid ranges
  for (size_t i = 0; i < 6; ++i) {
    double clamped_joint = clampValue(action[i], joint_min_deg_[i], joint_max_deg_[i]);
    processed_action.push_back(clamped_joint);
  }
  
  return processed_action;
}

double RLEnvironmentNode::calculateReward()
{
  double total_reward = 0.0;
  
  // Time penalty (encourages faster completion)
  total_reward += reward_weights_.time_penalty;
  
  // Distance reward (encourages moving toward target)
  double distance_to_target = calculateDistance(current_tcp_pose_, target_tcp_pose_);
  double max_distance = 2000.0; // mm
  double normalized_distance = std::min(distance_to_target / max_distance, 1.0);
  total_reward += reward_weights_.distance_penalty * normalized_distance;
  
  // Velocity penalty (encourages smooth motion)
  double velocity_magnitude = 0.0;
  for (const auto & vel : current_joint_state_.velocity) {
    velocity_magnitude += vel * vel;
  }
  velocity_magnitude = std::sqrt(velocity_magnitude);
  total_reward += reward_weights_.velocity_penalty * velocity_magnitude;
  
  // Task-specific rewards
  // Success reward if close to target
  if (distance_to_target < 10.0) { // Within 10mm
    total_reward += reward_weights_.grasp_success;
  }
  
  // Alignment reward if oriented correctly
  // (This is simplified - in a real task, you'd check orientation alignment)
  double orientation_error = 1.0 - std::abs(current_tcp_pose_.orientation.w);
  if (orientation_error < 0.1) {
    total_reward += reward_weights_.assembly_alignment;
  }
  
  return total_reward;
}

bool RLEnvironmentNode::checkDone()
{
  // Check if maximum episode steps reached
  if (episode_state_.step_count >= max_episode_steps_) {
    return true;
  }
  
  // Check if task completed successfully
  double distance_to_target = calculateDistance(current_tcp_pose_, target_tcp_pose_);
  if (distance_to_target < 5.0) { // Within 5mm of target
    RCLCPP_INFO(this->get_logger(), "Task completed successfully!");
    return true;
  }
  
  // Check safety state
  if (!checkSafetyState()) {
    return true;
  }
  
  return false;
}

std::string RLEnvironmentNode::buildInfo()
{
  std::ostringstream info;
  info << std::fixed << std::setprecision(2);
  info << "{";
  info << "\"step\":" << episode_state_.step_count << ",";
  info << "\"total_reward\":" << episode_state_.total_reward << ",";
  info << "\"distance_to_target\":" << calculateDistance(current_tcp_pose_, target_tcp_pose_) << ",";
  info << "\"safety_state\":" << (checkSafetyState() ? "\"safe\"" : "\"unsafe\"") << ",";
  
  // Add action clamping info if needed
  info << "\"action_clamped\":false"; // Would be set during action processing
  
  info << "}";
  return info.str();
}

bool RLEnvironmentNode::resetToHome()
{
  if (!move_joint_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
    RCLCPP_ERROR(this->get_logger(), "Move joint service not available for reset");
    return false;
  }
  
  auto request = std::make_shared<dsr_msgs2::srv::MoveJoint::Request>();
  std::copy(home_joint_positions_.begin(), home_joint_positions_.end(), request->pos.begin());
  request->vel = 30.0; // 30% of max velocity
  request->acc = 50.0; // 50% of max acceleration
  request->time = 5.0; // 5 seconds to reach home
  request->radius = 0.0;
  request->mode = 0; // Absolute positioning
  request->blend_type = 0;
  request->sync_type = 0;
  
  auto future = move_joint_client_->async_send_request(request);
  
  auto timeout = std::chrono::duration<double>(action_timeout_sec_);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) 
      == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    return response->success;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for reset to home");
    return false;
  }
}

bool RLEnvironmentNode::executeAction(const std::vector<double> & processed_action)
{
  if (!move_joint_client_->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
    RCLCPP_ERROR(this->get_logger(), "Move joint service not available");
    return false;
  }
  
  auto request = std::make_shared<dsr_msgs2::srv::MoveJoint::Request>();
  std::copy(processed_action.begin(), processed_action.end(), request->pos.begin());
  request->vel = 20.0; // 20% of max velocity for RL training
  request->acc = 30.0; // 30% of max acceleration
  request->time = 2.0; // 2 seconds per action
  request->radius = 0.0;
  request->mode = 0; // Absolute positioning
  request->blend_type = 0;
  request->sync_type = 0;
  
  auto future = move_joint_client_->async_send_request(request);
  
  auto timeout = std::chrono::duration<double>(action_timeout_sec_);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout) 
      == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    return response->success;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Timeout waiting for action execution");
    return false;
  }
}

double RLEnvironmentNode::clampValue(double value, double min_val, double max_val)
{
  return std::max(min_val, std::min(value, max_val));
}

std::vector<double> RLEnvironmentNode::clampJointPositions(const std::vector<double> & positions)
{
  std::vector<double> clamped_positions;
  clamped_positions.reserve(positions.size());
  
  for (size_t i = 0; i < positions.size() && i < joint_min_deg_.size(); ++i) {
    clamped_positions.push_back(clampValue(positions[i], joint_min_deg_[i], joint_max_deg_[i]));
  }
  
  return clamped_positions;
}

bool RLEnvironmentNode::checkSafetyState()
{
  if (!safety_client_->wait_for_service(std::chrono::duration<double>(1.0))) {
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
    return false;
  }
}

double RLEnvironmentNode::calculateDistance(const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2)
{
  double dx = pose1.position.x - pose2.position.x;
  double dy = pose1.position.y - pose2.position.y;
  double dz = pose1.position.z - pose2.position.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double RLEnvironmentNode::calculateJointDistance(const std::vector<double> & joints1, const std::vector<double> & joints2)
{
  if (joints1.size() != joints2.size()) {
    return std::numeric_limits<double>::max();
  }
  
  double sum = 0.0;
  for (size_t i = 0; i < joints1.size(); ++i) {
    double diff = joints1[i] - joints2[i];
    sum += diff * diff;
  }
  return std::sqrt(sum);
}

void RLEnvironmentNode::initializeTask()
{
  // Initialize home position (all joints at 0 degrees)
  home_joint_positions_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  
  // Initialize target TCP pose for SomaCube assembly task
  target_tcp_pose_.position.x = 400.0; // 400mm in front
  target_tcp_pose_.position.y = 0.0;
  target_tcp_pose_.position.z = 300.0; // 300mm above base
  target_tcp_pose_.orientation.w = 1.0;
  target_tcp_pose_.orientation.x = 0.0;
  target_tcp_pose_.orientation.y = 0.0;
  target_tcp_pose_.orientation.z = 0.0;
  
  RCLCPP_INFO(this->get_logger(), "Task initialized with target pose: [%.1f, %.1f, %.1f]",
    target_tcp_pose_.position.x, target_tcp_pose_.position.y, target_tcp_pose_.position.z);
}

} // namespace soma_cube_rl_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<soma_cube_rl_bridge::RLEnvironmentNode>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Exception in RL environment: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
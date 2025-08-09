#ifndef SOMA_CUBE_RL_BRIDGE__RL_ENV_NODE_HPP_
#define SOMA_CUBE_RL_BRIDGE__RL_ENV_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <random>
#include <array>

#include "dsr_msgs2/srv/move_joint.hpp"
#include "dsr_msgs2/srv/get_current_posj.hpp"
#include "dsr_msgs2/srv/get_current_posx.hpp"
#include "dsr_msgs2/srv/get_tool_force.hpp"

#include "soma_cube_rl_bridge/srv/rl_reset.hpp"
#include "soma_cube_rl_bridge/srv/rl_step.hpp"
#include "soma_cube_rl_bridge/srv/get_safety_state.hpp"
#include "soma_cube_rl_bridge/msg/rl_observation.hpp"

namespace soma_cube_rl_bridge
{

struct RewardWeights
{
  double grasp_success;
  double assembly_alignment;
  double collision_penalty;
  double time_penalty;
  double distance_penalty;
  double velocity_penalty;
};

struct EpisodeState
{
  int step_count;
  double total_reward;
  bool is_done;
  std::chrono::steady_clock::time_point start_time;
};

class RLEnvironmentNode : public rclcpp::Node
{
public:
  explicit RLEnvironmentNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
private:
  // Service servers (RL-facing)
  rclcpp::Service<soma_cube_rl_bridge::srv::RLReset>::SharedPtr rl_reset_server_;
  rclcpp::Service<soma_cube_rl_bridge::srv::RLStep>::SharedPtr rl_step_server_;
  
  // Service clients (proxy-facing)
  rclcpp::Client<dsr_msgs2::srv::MoveJoint>::SharedPtr move_joint_client_;
  rclcpp::Client<dsr_msgs2::srv::GetCurrentPosj>::SharedPtr get_posj_client_;
  rclcpp::Client<dsr_msgs2::srv::GetCurrentPosx>::SharedPtr get_posx_client_;
  rclcpp::Client<dsr_msgs2::srv::GetToolForce>::SharedPtr get_tool_force_client_;
  rclcpp::Client<soma_cube_rl_bridge::srv::GetSafetyState>::SharedPtr safety_client_;
  
  // Subscribers (state feedback)
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tcp_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tool_force_sub_;
  
  // Publishers (observation feedback)
  rclcpp::Publisher<soma_cube_rl_bridge::msg::RLObservation>::SharedPtr observation_pub_;
  
  // Parameters
  int max_episode_steps_;
  double action_timeout_sec_;
  double service_wait_sec_;
  std::vector<double> joint_min_deg_;
  std::vector<double> joint_max_deg_;
  std::vector<double> vel_limit_deg_s_;
  std::vector<double> acc_limit_deg_s2_;
  RewardWeights reward_weights_;
  
  // Internal state
  sensor_msgs::msg::JointState current_joint_state_;
  geometry_msgs::msg::Pose current_tcp_pose_;
  std::vector<double> current_tool_force_;
  EpisodeState episode_state_;
  std::mt19937 rng_;
  
  // Initial/target states for task
  std::array<double, 6> home_joint_positions_;
  geometry_msgs::msg::Pose target_tcp_pose_;
  
  // Callback methods
  void rlResetCallback(
    const std::shared_ptr<soma_cube_rl_bridge::srv::RLReset::Request> request,
    std::shared_ptr<soma_cube_rl_bridge::srv::RLReset::Response> response);
    
  void rlStepCallback(
    const std::shared_ptr<soma_cube_rl_bridge::srv::RLStep::Request> request,
    std::shared_ptr<soma_cube_rl_bridge::srv::RLStep::Response> response);
    
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void tcpPoseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void toolForceCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  
  // Core RL methods
  std::vector<double> buildObservation();
  std::vector<double> processAction(const std::vector<double> & action);
  double calculateReward();
  bool checkDone();
  std::string buildInfo();
  bool resetToHome();
  bool executeAction(const std::vector<double> & processed_action);
  
  // Utility methods
  void declareParameters();
  void loadRewardWeights();
  std::vector<double> clampJointPositions(const std::vector<double> & positions);
  double clampValue(double value, double min_val, double max_val);
  bool checkSafetyState();
  double calculateDistance(const geometry_msgs::msg::Pose & pose1, const geometry_msgs::msg::Pose & pose2);
  double calculateJointDistance(const std::vector<double> & joints1, const std::vector<double> & joints2);
  void initializeTask();
};

} // namespace soma_cube_rl_bridge

#endif // SOMA_CUBE_RL_BRIDGE__RL_ENV_NODE_HPP_
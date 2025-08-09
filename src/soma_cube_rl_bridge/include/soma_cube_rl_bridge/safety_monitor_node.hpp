#ifndef SOMA_CUBE_RL_BRIDGE__SAFETY_MONITOR_NODE_HPP_
#define SOMA_CUBE_RL_BRIDGE__SAFETY_MONITOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "dsr_msgs2/msg/robot_state.hpp"
#include "dsr_msgs2/msg/robot_error.hpp"
#include "dsr_msgs2/msg/robot_stop.hpp"
#include "dsr_msgs2/srv/get_robot_state.hpp"
#include "dsr_msgs2/srv/get_robot_mode.hpp"

#include "soma_cube_rl_bridge/srv/get_safety_state.hpp"
#include "soma_cube_rl_bridge/msg/safety_state.hpp"

namespace soma_cube_rl_bridge
{

class SafetyMonitorNode : public rclcpp::Node
{
public:
  explicit SafetyMonitorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
private:
  // Service servers
  rclcpp::Service<soma_cube_rl_bridge::srv::GetSafetyState>::SharedPtr get_safety_state_server_;
  
  // Service clients
  rclcpp::Client<dsr_msgs2::srv::GetRobotState>::SharedPtr get_robot_state_client_;
  rclcpp::Client<dsr_msgs2::srv::GetRobotMode>::SharedPtr get_robot_mode_client_;
  
  // Publishers
  rclcpp::Publisher<soma_cube_rl_bridge::msg::SafetyState>::SharedPtr safety_state_pub_;
  
  // Subscribers
  rclcpp::Subscription<dsr_msgs2::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<dsr_msgs2::msg::RobotError>::SharedPtr robot_error_sub_;
  rclcpp::Subscription<dsr_msgs2::msg::RobotStop>::SharedPtr robot_stop_sub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  
  // Parameters
  double monitor_rate_hz_;
  std::vector<double> joint_min_deg_;
  std::vector<double> joint_max_deg_;
  std::vector<double> tcp_workspace_min_;
  std::vector<double> tcp_workspace_max_;
  
  // Internal safety state
  bool safe_to_move_;
  std::string safety_reason_;
  builtin_interfaces::msg::Time last_event_time_;
  uint32_t current_robot_mode_;
  uint32_t current_robot_state_;
  bool emergency_stop_;
  bool protective_stop_;
  std::vector<bool> joint_limit_violations_;
  
  // Callback methods
  void getSafetyStateCallback(
    const std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState::Request> request,
    std::shared_ptr<soma_cube_rl_bridge::srv::GetSafetyState::Response> response);
    
  void robotStateCallback(const dsr_msgs2::msg::RobotState::SharedPtr msg);
  void robotErrorCallback(const dsr_msgs2::msg::RobotError::SharedPtr msg);
  void robotStopCallback(const dsr_msgs2::msg::RobotStop::SharedPtr msg);
  
  void monitorTimerCallback();
  
  // Utility methods
  void updateSafetyState();
  bool checkJointLimits(const std::vector<double> & joint_positions);
  bool checkTcpWorkspace(const std::vector<double> & tcp_position);
  bool checkRobotMode();
  bool checkRobotState();
  void publishSafetyState();
  void declareParameters();
  builtin_interfaces::msg::Time getCurrentTime();
};

} // namespace soma_cube_rl_bridge

#endif // SOMA_CUBE_RL_BRIDGE__SAFETY_MONITOR_NODE_HPP_
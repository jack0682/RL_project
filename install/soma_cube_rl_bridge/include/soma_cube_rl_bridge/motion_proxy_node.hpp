#ifndef SOMA_CUBE_RL_BRIDGE__MOTION_PROXY_NODE_HPP_
#define SOMA_CUBE_RL_BRIDGE__MOTION_PROXY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <array>

#include "dsr_msgs2/srv/move_joint.hpp"
#include "dsr_msgs2/srv/move_stop.hpp"
#include "dsr_msgs2/srv/get_current_posj.hpp"
#include "dsr_msgs2/srv/get_current_posx.hpp"
#include "dsr_msgs2/msg/robot_state.hpp"
#include "dsr_msgs2/msg/robot_error.hpp"

#include "soma_cube_rl_bridge/srv/get_safety_state.hpp"

namespace soma_cube_rl_bridge
{

class MotionProxyNode : public rclcpp::Node
{
public:
  explicit MotionProxyNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
private:
  // Service servers (RL-facing)
  rclcpp::Service<dsr_msgs2::srv::MoveJoint>::SharedPtr move_joint_server_;
  rclcpp::Service<dsr_msgs2::srv::MoveStop>::SharedPtr move_stop_server_;
  
  // Service clients (Doosan-facing)  
  rclcpp::Client<dsr_msgs2::srv::MoveJoint>::SharedPtr move_joint_client_;
  rclcpp::Client<dsr_msgs2::srv::MoveStop>::SharedPtr move_stop_client_;
  rclcpp::Client<dsr_msgs2::srv::GetCurrentPosj>::SharedPtr get_posj_client_;
  rclcpp::Client<dsr_msgs2::srv::GetCurrentPosx>::SharedPtr get_posx_client_;
  rclcpp::Client<soma_cube_rl_bridge::srv::GetSafetyState>::SharedPtr safety_client_;
  
  // Publishers (RL-facing state feedback)
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr tcp_pose_pub_;
  
  // Subscribers (Doosan-facing state feedback)
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr current_posx_sub_;
  rclcpp::Subscription<dsr_msgs2::msg::RobotState>::SharedPtr robot_state_sub_;
  rclcpp::Subscription<dsr_msgs2::msg::RobotError>::SharedPtr robot_error_sub_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr state_timer_;
  
  // Parameters
  double publish_rate_hz_;
  std::vector<double> vel_limit_deg_s_;
  std::vector<double> acc_limit_deg_s2_;
  std::vector<double> joint_min_deg_;
  std::vector<double> joint_max_deg_;
  double action_timeout_sec_;
  double service_wait_sec_;
  
  // Internal state
  sensor_msgs::msg::JointState current_joint_state_;
  geometry_msgs::msg::Pose current_tcp_pose_;
  bool is_connected_;
  
  // Callback methods
  void moveJointCallback(
    const std::shared_ptr<dsr_msgs2::srv::MoveJoint::Request> request,
    std::shared_ptr<dsr_msgs2::srv::MoveJoint::Response> response);
    
  void moveStopCallback(
    const std::shared_ptr<dsr_msgs2::srv::MoveStop::Request> request,
    std::shared_ptr<dsr_msgs2::srv::MoveStop::Response> response);
    
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void currentPosxCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void robotStateCallback(const dsr_msgs2::msg::RobotState::SharedPtr msg);
  void robotErrorCallback(const dsr_msgs2::msg::RobotError::SharedPtr msg);
  
  void stateTimerCallback();
  
  // Utility methods
  bool validateJointLimits(const std::array<double, 6> & joint_positions);
  bool validateVelocityLimits(double velocity);
  bool validateAccelerationLimits(double acceleration);
  bool checkSafetyState();
  void declareParameters();
  bool connectToDoosanServices();
};

} // namespace soma_cube_rl_bridge

#endif // SOMA_CUBE_RL_BRIDGE__MOTION_PROXY_NODE_HPP_
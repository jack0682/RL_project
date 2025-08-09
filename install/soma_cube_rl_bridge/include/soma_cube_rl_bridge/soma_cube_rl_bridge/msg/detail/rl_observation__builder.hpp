// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from soma_cube_rl_bridge:msg/RLObservation.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__BUILDER_HPP_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "soma_cube_rl_bridge/msg/detail/rl_observation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace soma_cube_rl_bridge
{

namespace msg
{

namespace builder
{

class Init_RLObservation_contact_detected
{
public:
  explicit Init_RLObservation_contact_detected(::soma_cube_rl_bridge::msg::RLObservation & msg)
  : msg_(msg)
  {}
  ::soma_cube_rl_bridge::msg::RLObservation contact_detected(::soma_cube_rl_bridge::msg::RLObservation::_contact_detected_type arg)
  {
    msg_.contact_detected = std::move(arg);
    return std::move(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::RLObservation msg_;
};

class Init_RLObservation_task_success
{
public:
  explicit Init_RLObservation_task_success(::soma_cube_rl_bridge::msg::RLObservation & msg)
  : msg_(msg)
  {}
  Init_RLObservation_contact_detected task_success(::soma_cube_rl_bridge::msg::RLObservation::_task_success_type arg)
  {
    msg_.task_success = std::move(arg);
    return Init_RLObservation_contact_detected(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::RLObservation msg_;
};

class Init_RLObservation_robot_state
{
public:
  explicit Init_RLObservation_robot_state(::soma_cube_rl_bridge::msg::RLObservation & msg)
  : msg_(msg)
  {}
  Init_RLObservation_task_success robot_state(::soma_cube_rl_bridge::msg::RLObservation::_robot_state_type arg)
  {
    msg_.robot_state = std::move(arg);
    return Init_RLObservation_task_success(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::RLObservation msg_;
};

class Init_RLObservation_gripper_status
{
public:
  explicit Init_RLObservation_gripper_status(::soma_cube_rl_bridge::msg::RLObservation & msg)
  : msg_(msg)
  {}
  Init_RLObservation_robot_state gripper_status(::soma_cube_rl_bridge::msg::RLObservation::_gripper_status_type arg)
  {
    msg_.gripper_status = std::move(arg);
    return Init_RLObservation_robot_state(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::RLObservation msg_;
};

class Init_RLObservation_tool_force
{
public:
  explicit Init_RLObservation_tool_force(::soma_cube_rl_bridge::msg::RLObservation & msg)
  : msg_(msg)
  {}
  Init_RLObservation_gripper_status tool_force(::soma_cube_rl_bridge::msg::RLObservation::_tool_force_type arg)
  {
    msg_.tool_force = std::move(arg);
    return Init_RLObservation_gripper_status(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::RLObservation msg_;
};

class Init_RLObservation_tcp_pose
{
public:
  explicit Init_RLObservation_tcp_pose(::soma_cube_rl_bridge::msg::RLObservation & msg)
  : msg_(msg)
  {}
  Init_RLObservation_tool_force tcp_pose(::soma_cube_rl_bridge::msg::RLObservation::_tcp_pose_type arg)
  {
    msg_.tcp_pose = std::move(arg);
    return Init_RLObservation_tool_force(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::RLObservation msg_;
};

class Init_RLObservation_joint_velocities
{
public:
  explicit Init_RLObservation_joint_velocities(::soma_cube_rl_bridge::msg::RLObservation & msg)
  : msg_(msg)
  {}
  Init_RLObservation_tcp_pose joint_velocities(::soma_cube_rl_bridge::msg::RLObservation::_joint_velocities_type arg)
  {
    msg_.joint_velocities = std::move(arg);
    return Init_RLObservation_tcp_pose(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::RLObservation msg_;
};

class Init_RLObservation_joint_positions
{
public:
  explicit Init_RLObservation_joint_positions(::soma_cube_rl_bridge::msg::RLObservation & msg)
  : msg_(msg)
  {}
  Init_RLObservation_joint_velocities joint_positions(::soma_cube_rl_bridge::msg::RLObservation::_joint_positions_type arg)
  {
    msg_.joint_positions = std::move(arg);
    return Init_RLObservation_joint_velocities(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::RLObservation msg_;
};

class Init_RLObservation_header
{
public:
  Init_RLObservation_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RLObservation_joint_positions header(::soma_cube_rl_bridge::msg::RLObservation::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RLObservation_joint_positions(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::RLObservation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::soma_cube_rl_bridge::msg::RLObservation>()
{
  return soma_cube_rl_bridge::msg::builder::Init_RLObservation_header();
}

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__BUILDER_HPP_

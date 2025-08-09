// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from soma_cube_rl_bridge:msg/SafetyState.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__BUILDER_HPP_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "soma_cube_rl_bridge/msg/detail/safety_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace soma_cube_rl_bridge
{

namespace msg
{

namespace builder
{

class Init_SafetyState_joint_limit_violations
{
public:
  explicit Init_SafetyState_joint_limit_violations(::soma_cube_rl_bridge::msg::SafetyState & msg)
  : msg_(msg)
  {}
  ::soma_cube_rl_bridge::msg::SafetyState joint_limit_violations(::soma_cube_rl_bridge::msg::SafetyState::_joint_limit_violations_type arg)
  {
    msg_.joint_limit_violations = std::move(arg);
    return std::move(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::SafetyState msg_;
};

class Init_SafetyState_protective_stop
{
public:
  explicit Init_SafetyState_protective_stop(::soma_cube_rl_bridge::msg::SafetyState & msg)
  : msg_(msg)
  {}
  Init_SafetyState_joint_limit_violations protective_stop(::soma_cube_rl_bridge::msg::SafetyState::_protective_stop_type arg)
  {
    msg_.protective_stop = std::move(arg);
    return Init_SafetyState_joint_limit_violations(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::SafetyState msg_;
};

class Init_SafetyState_emergency_stop
{
public:
  explicit Init_SafetyState_emergency_stop(::soma_cube_rl_bridge::msg::SafetyState & msg)
  : msg_(msg)
  {}
  Init_SafetyState_protective_stop emergency_stop(::soma_cube_rl_bridge::msg::SafetyState::_emergency_stop_type arg)
  {
    msg_.emergency_stop = std::move(arg);
    return Init_SafetyState_protective_stop(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::SafetyState msg_;
};

class Init_SafetyState_robot_state
{
public:
  explicit Init_SafetyState_robot_state(::soma_cube_rl_bridge::msg::SafetyState & msg)
  : msg_(msg)
  {}
  Init_SafetyState_emergency_stop robot_state(::soma_cube_rl_bridge::msg::SafetyState::_robot_state_type arg)
  {
    msg_.robot_state = std::move(arg);
    return Init_SafetyState_emergency_stop(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::SafetyState msg_;
};

class Init_SafetyState_robot_mode
{
public:
  explicit Init_SafetyState_robot_mode(::soma_cube_rl_bridge::msg::SafetyState & msg)
  : msg_(msg)
  {}
  Init_SafetyState_robot_state robot_mode(::soma_cube_rl_bridge::msg::SafetyState::_robot_mode_type arg)
  {
    msg_.robot_mode = std::move(arg);
    return Init_SafetyState_robot_state(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::SafetyState msg_;
};

class Init_SafetyState_last_event_time
{
public:
  explicit Init_SafetyState_last_event_time(::soma_cube_rl_bridge::msg::SafetyState & msg)
  : msg_(msg)
  {}
  Init_SafetyState_robot_mode last_event_time(::soma_cube_rl_bridge::msg::SafetyState::_last_event_time_type arg)
  {
    msg_.last_event_time = std::move(arg);
    return Init_SafetyState_robot_mode(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::SafetyState msg_;
};

class Init_SafetyState_reason
{
public:
  explicit Init_SafetyState_reason(::soma_cube_rl_bridge::msg::SafetyState & msg)
  : msg_(msg)
  {}
  Init_SafetyState_last_event_time reason(::soma_cube_rl_bridge::msg::SafetyState::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_SafetyState_last_event_time(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::SafetyState msg_;
};

class Init_SafetyState_safe_to_move
{
public:
  explicit Init_SafetyState_safe_to_move(::soma_cube_rl_bridge::msg::SafetyState & msg)
  : msg_(msg)
  {}
  Init_SafetyState_reason safe_to_move(::soma_cube_rl_bridge::msg::SafetyState::_safe_to_move_type arg)
  {
    msg_.safe_to_move = std::move(arg);
    return Init_SafetyState_reason(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::SafetyState msg_;
};

class Init_SafetyState_header
{
public:
  Init_SafetyState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SafetyState_safe_to_move header(::soma_cube_rl_bridge::msg::SafetyState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SafetyState_safe_to_move(msg_);
  }

private:
  ::soma_cube_rl_bridge::msg::SafetyState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::soma_cube_rl_bridge::msg::SafetyState>()
{
  return soma_cube_rl_bridge::msg::builder::Init_SafetyState_header();
}

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__BUILDER_HPP_

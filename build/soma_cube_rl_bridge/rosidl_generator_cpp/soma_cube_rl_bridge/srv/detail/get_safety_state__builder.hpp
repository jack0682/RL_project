// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from soma_cube_rl_bridge:srv/GetSafetyState.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__BUILDER_HPP_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "soma_cube_rl_bridge/srv/detail/get_safety_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace soma_cube_rl_bridge
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::soma_cube_rl_bridge::srv::GetSafetyState_Request>()
{
  return ::soma_cube_rl_bridge::srv::GetSafetyState_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace soma_cube_rl_bridge


namespace soma_cube_rl_bridge
{

namespace srv
{

namespace builder
{

class Init_GetSafetyState_Response_last_event_time
{
public:
  explicit Init_GetSafetyState_Response_last_event_time(::soma_cube_rl_bridge::srv::GetSafetyState_Response & msg)
  : msg_(msg)
  {}
  ::soma_cube_rl_bridge::srv::GetSafetyState_Response last_event_time(::soma_cube_rl_bridge::srv::GetSafetyState_Response::_last_event_time_type arg)
  {
    msg_.last_event_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::GetSafetyState_Response msg_;
};

class Init_GetSafetyState_Response_reason
{
public:
  explicit Init_GetSafetyState_Response_reason(::soma_cube_rl_bridge::srv::GetSafetyState_Response & msg)
  : msg_(msg)
  {}
  Init_GetSafetyState_Response_last_event_time reason(::soma_cube_rl_bridge::srv::GetSafetyState_Response::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_GetSafetyState_Response_last_event_time(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::GetSafetyState_Response msg_;
};

class Init_GetSafetyState_Response_safe_to_move
{
public:
  Init_GetSafetyState_Response_safe_to_move()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetSafetyState_Response_reason safe_to_move(::soma_cube_rl_bridge::srv::GetSafetyState_Response::_safe_to_move_type arg)
  {
    msg_.safe_to_move = std::move(arg);
    return Init_GetSafetyState_Response_reason(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::GetSafetyState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::soma_cube_rl_bridge::srv::GetSafetyState_Response>()
{
  return soma_cube_rl_bridge::srv::builder::Init_GetSafetyState_Response_safe_to_move();
}

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__BUILDER_HPP_

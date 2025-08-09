// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from soma_cube_rl_bridge:srv/RLStep.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__BUILDER_HPP_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "soma_cube_rl_bridge/srv/detail/rl_step__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace soma_cube_rl_bridge
{

namespace srv
{

namespace builder
{

class Init_RLStep_Request_action
{
public:
  Init_RLStep_Request_action()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::soma_cube_rl_bridge::srv::RLStep_Request action(::soma_cube_rl_bridge::srv::RLStep_Request::_action_type arg)
  {
    msg_.action = std::move(arg);
    return std::move(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLStep_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::soma_cube_rl_bridge::srv::RLStep_Request>()
{
  return soma_cube_rl_bridge::srv::builder::Init_RLStep_Request_action();
}

}  // namespace soma_cube_rl_bridge


namespace soma_cube_rl_bridge
{

namespace srv
{

namespace builder
{

class Init_RLStep_Response_success
{
public:
  explicit Init_RLStep_Response_success(::soma_cube_rl_bridge::srv::RLStep_Response & msg)
  : msg_(msg)
  {}
  ::soma_cube_rl_bridge::srv::RLStep_Response success(::soma_cube_rl_bridge::srv::RLStep_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLStep_Response msg_;
};

class Init_RLStep_Response_info
{
public:
  explicit Init_RLStep_Response_info(::soma_cube_rl_bridge::srv::RLStep_Response & msg)
  : msg_(msg)
  {}
  Init_RLStep_Response_success info(::soma_cube_rl_bridge::srv::RLStep_Response::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_RLStep_Response_success(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLStep_Response msg_;
};

class Init_RLStep_Response_done
{
public:
  explicit Init_RLStep_Response_done(::soma_cube_rl_bridge::srv::RLStep_Response & msg)
  : msg_(msg)
  {}
  Init_RLStep_Response_info done(::soma_cube_rl_bridge::srv::RLStep_Response::_done_type arg)
  {
    msg_.done = std::move(arg);
    return Init_RLStep_Response_info(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLStep_Response msg_;
};

class Init_RLStep_Response_reward
{
public:
  explicit Init_RLStep_Response_reward(::soma_cube_rl_bridge::srv::RLStep_Response & msg)
  : msg_(msg)
  {}
  Init_RLStep_Response_done reward(::soma_cube_rl_bridge::srv::RLStep_Response::_reward_type arg)
  {
    msg_.reward = std::move(arg);
    return Init_RLStep_Response_done(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLStep_Response msg_;
};

class Init_RLStep_Response_obs
{
public:
  Init_RLStep_Response_obs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RLStep_Response_reward obs(::soma_cube_rl_bridge::srv::RLStep_Response::_obs_type arg)
  {
    msg_.obs = std::move(arg);
    return Init_RLStep_Response_reward(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLStep_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::soma_cube_rl_bridge::srv::RLStep_Response>()
{
  return soma_cube_rl_bridge::srv::builder::Init_RLStep_Response_obs();
}

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__BUILDER_HPP_

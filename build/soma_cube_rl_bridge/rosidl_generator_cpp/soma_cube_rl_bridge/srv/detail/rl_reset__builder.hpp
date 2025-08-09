// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from soma_cube_rl_bridge:srv/RLReset.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__BUILDER_HPP_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "soma_cube_rl_bridge/srv/detail/rl_reset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace soma_cube_rl_bridge
{

namespace srv
{

namespace builder
{

class Init_RLReset_Request_seed
{
public:
  Init_RLReset_Request_seed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::soma_cube_rl_bridge::srv::RLReset_Request seed(::soma_cube_rl_bridge::srv::RLReset_Request::_seed_type arg)
  {
    msg_.seed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLReset_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::soma_cube_rl_bridge::srv::RLReset_Request>()
{
  return soma_cube_rl_bridge::srv::builder::Init_RLReset_Request_seed();
}

}  // namespace soma_cube_rl_bridge


namespace soma_cube_rl_bridge
{

namespace srv
{

namespace builder
{

class Init_RLReset_Response_message
{
public:
  explicit Init_RLReset_Response_message(::soma_cube_rl_bridge::srv::RLReset_Response & msg)
  : msg_(msg)
  {}
  ::soma_cube_rl_bridge::srv::RLReset_Response message(::soma_cube_rl_bridge::srv::RLReset_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLReset_Response msg_;
};

class Init_RLReset_Response_success
{
public:
  explicit Init_RLReset_Response_success(::soma_cube_rl_bridge::srv::RLReset_Response & msg)
  : msg_(msg)
  {}
  Init_RLReset_Response_message success(::soma_cube_rl_bridge::srv::RLReset_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_RLReset_Response_message(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLReset_Response msg_;
};

class Init_RLReset_Response_initial_obs
{
public:
  Init_RLReset_Response_initial_obs()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RLReset_Response_success initial_obs(::soma_cube_rl_bridge::srv::RLReset_Response::_initial_obs_type arg)
  {
    msg_.initial_obs = std::move(arg);
    return Init_RLReset_Response_success(msg_);
  }

private:
  ::soma_cube_rl_bridge::srv::RLReset_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::soma_cube_rl_bridge::srv::RLReset_Response>()
{
  return soma_cube_rl_bridge::srv::builder::Init_RLReset_Response_initial_obs();
}

}  // namespace soma_cube_rl_bridge

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__BUILDER_HPP_

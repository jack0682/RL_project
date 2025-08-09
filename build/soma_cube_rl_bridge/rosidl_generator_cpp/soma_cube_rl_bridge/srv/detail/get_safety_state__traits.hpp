// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from soma_cube_rl_bridge:srv/GetSafetyState.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__TRAITS_HPP_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "soma_cube_rl_bridge/srv/detail/get_safety_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace soma_cube_rl_bridge
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSafetyState_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSafetyState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSafetyState_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace soma_cube_rl_bridge

namespace rosidl_generator_traits
{

[[deprecated("use soma_cube_rl_bridge::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const soma_cube_rl_bridge::srv::GetSafetyState_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  soma_cube_rl_bridge::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use soma_cube_rl_bridge::srv::to_yaml() instead")]]
inline std::string to_yaml(const soma_cube_rl_bridge::srv::GetSafetyState_Request & msg)
{
  return soma_cube_rl_bridge::srv::to_yaml(msg);
}

template<>
inline const char * data_type<soma_cube_rl_bridge::srv::GetSafetyState_Request>()
{
  return "soma_cube_rl_bridge::srv::GetSafetyState_Request";
}

template<>
inline const char * name<soma_cube_rl_bridge::srv::GetSafetyState_Request>()
{
  return "soma_cube_rl_bridge/srv/GetSafetyState_Request";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::srv::GetSafetyState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<soma_cube_rl_bridge::srv::GetSafetyState_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<soma_cube_rl_bridge::srv::GetSafetyState_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'last_event_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace soma_cube_rl_bridge
{

namespace srv
{

inline void to_flow_style_yaml(
  const GetSafetyState_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: safe_to_move
  {
    out << "safe_to_move: ";
    rosidl_generator_traits::value_to_yaml(msg.safe_to_move, out);
    out << ", ";
  }

  // member: reason
  {
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << ", ";
  }

  // member: last_event_time
  {
    out << "last_event_time: ";
    to_flow_style_yaml(msg.last_event_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GetSafetyState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: safe_to_move
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safe_to_move: ";
    rosidl_generator_traits::value_to_yaml(msg.safe_to_move, out);
    out << "\n";
  }

  // member: reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reason: ";
    rosidl_generator_traits::value_to_yaml(msg.reason, out);
    out << "\n";
  }

  // member: last_event_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_event_time:\n";
    to_block_style_yaml(msg.last_event_time, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GetSafetyState_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace soma_cube_rl_bridge

namespace rosidl_generator_traits
{

[[deprecated("use soma_cube_rl_bridge::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const soma_cube_rl_bridge::srv::GetSafetyState_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  soma_cube_rl_bridge::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use soma_cube_rl_bridge::srv::to_yaml() instead")]]
inline std::string to_yaml(const soma_cube_rl_bridge::srv::GetSafetyState_Response & msg)
{
  return soma_cube_rl_bridge::srv::to_yaml(msg);
}

template<>
inline const char * data_type<soma_cube_rl_bridge::srv::GetSafetyState_Response>()
{
  return "soma_cube_rl_bridge::srv::GetSafetyState_Response";
}

template<>
inline const char * name<soma_cube_rl_bridge::srv::GetSafetyState_Response>()
{
  return "soma_cube_rl_bridge/srv/GetSafetyState_Response";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::srv::GetSafetyState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<soma_cube_rl_bridge::srv::GetSafetyState_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<soma_cube_rl_bridge::srv::GetSafetyState_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<soma_cube_rl_bridge::srv::GetSafetyState>()
{
  return "soma_cube_rl_bridge::srv::GetSafetyState";
}

template<>
inline const char * name<soma_cube_rl_bridge::srv::GetSafetyState>()
{
  return "soma_cube_rl_bridge/srv/GetSafetyState";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::srv::GetSafetyState>
  : std::integral_constant<
    bool,
    has_fixed_size<soma_cube_rl_bridge::srv::GetSafetyState_Request>::value &&
    has_fixed_size<soma_cube_rl_bridge::srv::GetSafetyState_Response>::value
  >
{
};

template<>
struct has_bounded_size<soma_cube_rl_bridge::srv::GetSafetyState>
  : std::integral_constant<
    bool,
    has_bounded_size<soma_cube_rl_bridge::srv::GetSafetyState_Request>::value &&
    has_bounded_size<soma_cube_rl_bridge::srv::GetSafetyState_Response>::value
  >
{
};

template<>
struct is_service<soma_cube_rl_bridge::srv::GetSafetyState>
  : std::true_type
{
};

template<>
struct is_service_request<soma_cube_rl_bridge::srv::GetSafetyState_Request>
  : std::true_type
{
};

template<>
struct is_service_response<soma_cube_rl_bridge::srv::GetSafetyState_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__TRAITS_HPP_

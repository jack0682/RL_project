// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from soma_cube_rl_bridge:msg/SafetyState.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__TRAITS_HPP_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "soma_cube_rl_bridge/msg/detail/safety_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'last_event_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace soma_cube_rl_bridge
{

namespace msg
{

inline void to_flow_style_yaml(
  const SafetyState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

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
    out << ", ";
  }

  // member: robot_mode
  {
    out << "robot_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_mode, out);
    out << ", ";
  }

  // member: robot_state
  {
    out << "robot_state: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_state, out);
    out << ", ";
  }

  // member: emergency_stop
  {
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
    out << ", ";
  }

  // member: protective_stop
  {
    out << "protective_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.protective_stop, out);
    out << ", ";
  }

  // member: joint_limit_violations
  {
    if (msg.joint_limit_violations.size() == 0) {
      out << "joint_limit_violations: []";
    } else {
      out << "joint_limit_violations: [";
      size_t pending_items = msg.joint_limit_violations.size();
      for (auto item : msg.joint_limit_violations) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SafetyState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

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

  // member: robot_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_mode, out);
    out << "\n";
  }

  // member: robot_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_state: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_state, out);
    out << "\n";
  }

  // member: emergency_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emergency_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.emergency_stop, out);
    out << "\n";
  }

  // member: protective_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "protective_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.protective_stop, out);
    out << "\n";
  }

  // member: joint_limit_violations
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_limit_violations.size() == 0) {
      out << "joint_limit_violations: []\n";
    } else {
      out << "joint_limit_violations:\n";
      for (auto item : msg.joint_limit_violations) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SafetyState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace soma_cube_rl_bridge

namespace rosidl_generator_traits
{

[[deprecated("use soma_cube_rl_bridge::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const soma_cube_rl_bridge::msg::SafetyState & msg,
  std::ostream & out, size_t indentation = 0)
{
  soma_cube_rl_bridge::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use soma_cube_rl_bridge::msg::to_yaml() instead")]]
inline std::string to_yaml(const soma_cube_rl_bridge::msg::SafetyState & msg)
{
  return soma_cube_rl_bridge::msg::to_yaml(msg);
}

template<>
inline const char * data_type<soma_cube_rl_bridge::msg::SafetyState>()
{
  return "soma_cube_rl_bridge::msg::SafetyState";
}

template<>
inline const char * name<soma_cube_rl_bridge::msg::SafetyState>()
{
  return "soma_cube_rl_bridge/msg/SafetyState";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::msg::SafetyState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<soma_cube_rl_bridge::msg::SafetyState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<soma_cube_rl_bridge::msg::SafetyState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__TRAITS_HPP_

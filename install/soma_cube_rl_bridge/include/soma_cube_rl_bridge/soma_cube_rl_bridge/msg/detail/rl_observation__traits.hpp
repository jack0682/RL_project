// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from soma_cube_rl_bridge:msg/RLObservation.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__TRAITS_HPP_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "soma_cube_rl_bridge/msg/detail/rl_observation__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'tcp_pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace soma_cube_rl_bridge
{

namespace msg
{

inline void to_flow_style_yaml(
  const RLObservation & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: joint_positions
  {
    if (msg.joint_positions.size() == 0) {
      out << "joint_positions: []";
    } else {
      out << "joint_positions: [";
      size_t pending_items = msg.joint_positions.size();
      for (auto item : msg.joint_positions) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: joint_velocities
  {
    if (msg.joint_velocities.size() == 0) {
      out << "joint_velocities: []";
    } else {
      out << "joint_velocities: [";
      size_t pending_items = msg.joint_velocities.size();
      for (auto item : msg.joint_velocities) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tcp_pose
  {
    out << "tcp_pose: ";
    to_flow_style_yaml(msg.tcp_pose, out);
    out << ", ";
  }

  // member: tool_force
  {
    if (msg.tool_force.size() == 0) {
      out << "tool_force: []";
    } else {
      out << "tool_force: [";
      size_t pending_items = msg.tool_force.size();
      for (auto item : msg.tool_force) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: gripper_status
  {
    out << "gripper_status: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_status, out);
    out << ", ";
  }

  // member: robot_state
  {
    out << "robot_state: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_state, out);
    out << ", ";
  }

  // member: task_success
  {
    out << "task_success: ";
    rosidl_generator_traits::value_to_yaml(msg.task_success, out);
    out << ", ";
  }

  // member: contact_detected
  {
    out << "contact_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_detected, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RLObservation & msg,
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

  // member: joint_positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_positions.size() == 0) {
      out << "joint_positions: []\n";
    } else {
      out << "joint_positions:\n";
      for (auto item : msg.joint_positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: joint_velocities
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_velocities.size() == 0) {
      out << "joint_velocities: []\n";
    } else {
      out << "joint_velocities:\n";
      for (auto item : msg.joint_velocities) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tcp_pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_pose:\n";
    to_block_style_yaml(msg.tcp_pose, out, indentation + 2);
  }

  // member: tool_force
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.tool_force.size() == 0) {
      out << "tool_force: []\n";
    } else {
      out << "tool_force:\n";
      for (auto item : msg.tool_force) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: gripper_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "gripper_status: ";
    rosidl_generator_traits::value_to_yaml(msg.gripper_status, out);
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

  // member: task_success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "task_success: ";
    rosidl_generator_traits::value_to_yaml(msg.task_success, out);
    out << "\n";
  }

  // member: contact_detected
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "contact_detected: ";
    rosidl_generator_traits::value_to_yaml(msg.contact_detected, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RLObservation & msg, bool use_flow_style = false)
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
  const soma_cube_rl_bridge::msg::RLObservation & msg,
  std::ostream & out, size_t indentation = 0)
{
  soma_cube_rl_bridge::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use soma_cube_rl_bridge::msg::to_yaml() instead")]]
inline std::string to_yaml(const soma_cube_rl_bridge::msg::RLObservation & msg)
{
  return soma_cube_rl_bridge::msg::to_yaml(msg);
}

template<>
inline const char * data_type<soma_cube_rl_bridge::msg::RLObservation>()
{
  return "soma_cube_rl_bridge::msg::RLObservation";
}

template<>
inline const char * name<soma_cube_rl_bridge::msg::RLObservation>()
{
  return "soma_cube_rl_bridge/msg/RLObservation";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::msg::RLObservation>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<soma_cube_rl_bridge::msg::RLObservation>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<soma_cube_rl_bridge::msg::RLObservation>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__TRAITS_HPP_

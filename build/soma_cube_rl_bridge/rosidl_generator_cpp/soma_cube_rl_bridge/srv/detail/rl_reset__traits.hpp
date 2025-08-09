// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from soma_cube_rl_bridge:srv/RLReset.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__TRAITS_HPP_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "soma_cube_rl_bridge/srv/detail/rl_reset__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace soma_cube_rl_bridge
{

namespace srv
{

inline void to_flow_style_yaml(
  const RLReset_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: seed
  {
    out << "seed: ";
    rosidl_generator_traits::value_to_yaml(msg.seed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RLReset_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: seed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seed: ";
    rosidl_generator_traits::value_to_yaml(msg.seed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RLReset_Request & msg, bool use_flow_style = false)
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
  const soma_cube_rl_bridge::srv::RLReset_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  soma_cube_rl_bridge::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use soma_cube_rl_bridge::srv::to_yaml() instead")]]
inline std::string to_yaml(const soma_cube_rl_bridge::srv::RLReset_Request & msg)
{
  return soma_cube_rl_bridge::srv::to_yaml(msg);
}

template<>
inline const char * data_type<soma_cube_rl_bridge::srv::RLReset_Request>()
{
  return "soma_cube_rl_bridge::srv::RLReset_Request";
}

template<>
inline const char * name<soma_cube_rl_bridge::srv::RLReset_Request>()
{
  return "soma_cube_rl_bridge/srv/RLReset_Request";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::srv::RLReset_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<soma_cube_rl_bridge::srv::RLReset_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<soma_cube_rl_bridge::srv::RLReset_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace soma_cube_rl_bridge
{

namespace srv
{

inline void to_flow_style_yaml(
  const RLReset_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: initial_obs
  {
    if (msg.initial_obs.size() == 0) {
      out << "initial_obs: []";
    } else {
      out << "initial_obs: [";
      size_t pending_items = msg.initial_obs.size();
      for (auto item : msg.initial_obs) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RLReset_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: initial_obs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.initial_obs.size() == 0) {
      out << "initial_obs: []\n";
    } else {
      out << "initial_obs:\n";
      for (auto item : msg.initial_obs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RLReset_Response & msg, bool use_flow_style = false)
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
  const soma_cube_rl_bridge::srv::RLReset_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  soma_cube_rl_bridge::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use soma_cube_rl_bridge::srv::to_yaml() instead")]]
inline std::string to_yaml(const soma_cube_rl_bridge::srv::RLReset_Response & msg)
{
  return soma_cube_rl_bridge::srv::to_yaml(msg);
}

template<>
inline const char * data_type<soma_cube_rl_bridge::srv::RLReset_Response>()
{
  return "soma_cube_rl_bridge::srv::RLReset_Response";
}

template<>
inline const char * name<soma_cube_rl_bridge::srv::RLReset_Response>()
{
  return "soma_cube_rl_bridge/srv/RLReset_Response";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::srv::RLReset_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<soma_cube_rl_bridge::srv::RLReset_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<soma_cube_rl_bridge::srv::RLReset_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<soma_cube_rl_bridge::srv::RLReset>()
{
  return "soma_cube_rl_bridge::srv::RLReset";
}

template<>
inline const char * name<soma_cube_rl_bridge::srv::RLReset>()
{
  return "soma_cube_rl_bridge/srv/RLReset";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::srv::RLReset>
  : std::integral_constant<
    bool,
    has_fixed_size<soma_cube_rl_bridge::srv::RLReset_Request>::value &&
    has_fixed_size<soma_cube_rl_bridge::srv::RLReset_Response>::value
  >
{
};

template<>
struct has_bounded_size<soma_cube_rl_bridge::srv::RLReset>
  : std::integral_constant<
    bool,
    has_bounded_size<soma_cube_rl_bridge::srv::RLReset_Request>::value &&
    has_bounded_size<soma_cube_rl_bridge::srv::RLReset_Response>::value
  >
{
};

template<>
struct is_service<soma_cube_rl_bridge::srv::RLReset>
  : std::true_type
{
};

template<>
struct is_service_request<soma_cube_rl_bridge::srv::RLReset_Request>
  : std::true_type
{
};

template<>
struct is_service_response<soma_cube_rl_bridge::srv::RLReset_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__TRAITS_HPP_

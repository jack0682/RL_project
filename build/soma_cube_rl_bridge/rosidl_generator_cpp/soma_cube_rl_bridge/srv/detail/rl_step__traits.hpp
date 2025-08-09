// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from soma_cube_rl_bridge:srv/RLStep.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__TRAITS_HPP_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "soma_cube_rl_bridge/srv/detail/rl_step__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace soma_cube_rl_bridge
{

namespace srv
{

inline void to_flow_style_yaml(
  const RLStep_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: action
  {
    if (msg.action.size() == 0) {
      out << "action: []";
    } else {
      out << "action: [";
      size_t pending_items = msg.action.size();
      for (auto item : msg.action) {
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
  const RLStep_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: action
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.action.size() == 0) {
      out << "action: []\n";
    } else {
      out << "action:\n";
      for (auto item : msg.action) {
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

inline std::string to_yaml(const RLStep_Request & msg, bool use_flow_style = false)
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
  const soma_cube_rl_bridge::srv::RLStep_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  soma_cube_rl_bridge::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use soma_cube_rl_bridge::srv::to_yaml() instead")]]
inline std::string to_yaml(const soma_cube_rl_bridge::srv::RLStep_Request & msg)
{
  return soma_cube_rl_bridge::srv::to_yaml(msg);
}

template<>
inline const char * data_type<soma_cube_rl_bridge::srv::RLStep_Request>()
{
  return "soma_cube_rl_bridge::srv::RLStep_Request";
}

template<>
inline const char * name<soma_cube_rl_bridge::srv::RLStep_Request>()
{
  return "soma_cube_rl_bridge/srv/RLStep_Request";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::srv::RLStep_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<soma_cube_rl_bridge::srv::RLStep_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<soma_cube_rl_bridge::srv::RLStep_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace soma_cube_rl_bridge
{

namespace srv
{

inline void to_flow_style_yaml(
  const RLStep_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: obs
  {
    if (msg.obs.size() == 0) {
      out << "obs: []";
    } else {
      out << "obs: [";
      size_t pending_items = msg.obs.size();
      for (auto item : msg.obs) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: reward
  {
    out << "reward: ";
    rosidl_generator_traits::value_to_yaml(msg.reward, out);
    out << ", ";
  }

  // member: done
  {
    out << "done: ";
    rosidl_generator_traits::value_to_yaml(msg.done, out);
    out << ", ";
  }

  // member: info
  {
    out << "info: ";
    rosidl_generator_traits::value_to_yaml(msg.info, out);
    out << ", ";
  }

  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RLStep_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: obs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.obs.size() == 0) {
      out << "obs: []\n";
    } else {
      out << "obs:\n";
      for (auto item : msg.obs) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: reward
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reward: ";
    rosidl_generator_traits::value_to_yaml(msg.reward, out);
    out << "\n";
  }

  // member: done
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "done: ";
    rosidl_generator_traits::value_to_yaml(msg.done, out);
    out << "\n";
  }

  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info: ";
    rosidl_generator_traits::value_to_yaml(msg.info, out);
    out << "\n";
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RLStep_Response & msg, bool use_flow_style = false)
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
  const soma_cube_rl_bridge::srv::RLStep_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  soma_cube_rl_bridge::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use soma_cube_rl_bridge::srv::to_yaml() instead")]]
inline std::string to_yaml(const soma_cube_rl_bridge::srv::RLStep_Response & msg)
{
  return soma_cube_rl_bridge::srv::to_yaml(msg);
}

template<>
inline const char * data_type<soma_cube_rl_bridge::srv::RLStep_Response>()
{
  return "soma_cube_rl_bridge::srv::RLStep_Response";
}

template<>
inline const char * name<soma_cube_rl_bridge::srv::RLStep_Response>()
{
  return "soma_cube_rl_bridge/srv/RLStep_Response";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::srv::RLStep_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<soma_cube_rl_bridge::srv::RLStep_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<soma_cube_rl_bridge::srv::RLStep_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<soma_cube_rl_bridge::srv::RLStep>()
{
  return "soma_cube_rl_bridge::srv::RLStep";
}

template<>
inline const char * name<soma_cube_rl_bridge::srv::RLStep>()
{
  return "soma_cube_rl_bridge/srv/RLStep";
}

template<>
struct has_fixed_size<soma_cube_rl_bridge::srv::RLStep>
  : std::integral_constant<
    bool,
    has_fixed_size<soma_cube_rl_bridge::srv::RLStep_Request>::value &&
    has_fixed_size<soma_cube_rl_bridge::srv::RLStep_Response>::value
  >
{
};

template<>
struct has_bounded_size<soma_cube_rl_bridge::srv::RLStep>
  : std::integral_constant<
    bool,
    has_bounded_size<soma_cube_rl_bridge::srv::RLStep_Request>::value &&
    has_bounded_size<soma_cube_rl_bridge::srv::RLStep_Response>::value
  >
{
};

template<>
struct is_service<soma_cube_rl_bridge::srv::RLStep>
  : std::true_type
{
};

template<>
struct is_service_request<soma_cube_rl_bridge::srv::RLStep_Request>
  : std::true_type
{
};

template<>
struct is_service_response<soma_cube_rl_bridge::srv::RLStep_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__TRAITS_HPP_

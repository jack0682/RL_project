// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dsr_msgs2:srv/JogMulti.idl
// generated code does not contain a copyright notice

#ifndef DSR_MSGS2__SRV__DETAIL__JOG_MULTI__TRAITS_HPP_
#define DSR_MSGS2__SRV__DETAIL__JOG_MULTI__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "dsr_msgs2/srv/detail/jog_multi__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace dsr_msgs2
{

namespace srv
{

inline void to_flow_style_yaml(
  const JogMulti_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: jog_axis
  {
    if (msg.jog_axis.size() == 0) {
      out << "jog_axis: []";
    } else {
      out << "jog_axis: [";
      size_t pending_items = msg.jog_axis.size();
      for (auto item : msg.jog_axis) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: move_reference
  {
    out << "move_reference: ";
    rosidl_generator_traits::value_to_yaml(msg.move_reference, out);
    out << ", ";
  }

  // member: speed
  {
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JogMulti_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: jog_axis
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.jog_axis.size() == 0) {
      out << "jog_axis: []\n";
    } else {
      out << "jog_axis:\n";
      for (auto item : msg.jog_axis) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: move_reference
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "move_reference: ";
    rosidl_generator_traits::value_to_yaml(msg.move_reference, out);
    out << "\n";
  }

  // member: speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed: ";
    rosidl_generator_traits::value_to_yaml(msg.speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JogMulti_Request & msg, bool use_flow_style = false)
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

}  // namespace dsr_msgs2

namespace rosidl_generator_traits
{

[[deprecated("use dsr_msgs2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dsr_msgs2::srv::JogMulti_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  dsr_msgs2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dsr_msgs2::srv::to_yaml() instead")]]
inline std::string to_yaml(const dsr_msgs2::srv::JogMulti_Request & msg)
{
  return dsr_msgs2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dsr_msgs2::srv::JogMulti_Request>()
{
  return "dsr_msgs2::srv::JogMulti_Request";
}

template<>
inline const char * name<dsr_msgs2::srv::JogMulti_Request>()
{
  return "dsr_msgs2/srv/JogMulti_Request";
}

template<>
struct has_fixed_size<dsr_msgs2::srv::JogMulti_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dsr_msgs2::srv::JogMulti_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dsr_msgs2::srv::JogMulti_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace dsr_msgs2
{

namespace srv
{

inline void to_flow_style_yaml(
  const JogMulti_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JogMulti_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
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

inline std::string to_yaml(const JogMulti_Response & msg, bool use_flow_style = false)
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

}  // namespace dsr_msgs2

namespace rosidl_generator_traits
{

[[deprecated("use dsr_msgs2::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const dsr_msgs2::srv::JogMulti_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  dsr_msgs2::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use dsr_msgs2::srv::to_yaml() instead")]]
inline std::string to_yaml(const dsr_msgs2::srv::JogMulti_Response & msg)
{
  return dsr_msgs2::srv::to_yaml(msg);
}

template<>
inline const char * data_type<dsr_msgs2::srv::JogMulti_Response>()
{
  return "dsr_msgs2::srv::JogMulti_Response";
}

template<>
inline const char * name<dsr_msgs2::srv::JogMulti_Response>()
{
  return "dsr_msgs2/srv/JogMulti_Response";
}

template<>
struct has_fixed_size<dsr_msgs2::srv::JogMulti_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dsr_msgs2::srv::JogMulti_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dsr_msgs2::srv::JogMulti_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dsr_msgs2::srv::JogMulti>()
{
  return "dsr_msgs2::srv::JogMulti";
}

template<>
inline const char * name<dsr_msgs2::srv::JogMulti>()
{
  return "dsr_msgs2/srv/JogMulti";
}

template<>
struct has_fixed_size<dsr_msgs2::srv::JogMulti>
  : std::integral_constant<
    bool,
    has_fixed_size<dsr_msgs2::srv::JogMulti_Request>::value &&
    has_fixed_size<dsr_msgs2::srv::JogMulti_Response>::value
  >
{
};

template<>
struct has_bounded_size<dsr_msgs2::srv::JogMulti>
  : std::integral_constant<
    bool,
    has_bounded_size<dsr_msgs2::srv::JogMulti_Request>::value &&
    has_bounded_size<dsr_msgs2::srv::JogMulti_Response>::value
  >
{
};

template<>
struct is_service<dsr_msgs2::srv::JogMulti>
  : std::true_type
{
};

template<>
struct is_service_request<dsr_msgs2::srv::JogMulti_Request>
  : std::true_type
{
};

template<>
struct is_service_response<dsr_msgs2::srv::JogMulti_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // DSR_MSGS2__SRV__DETAIL__JOG_MULTI__TRAITS_HPP_

// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from soma_cube_rl_bridge:msg/RLObservation.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "soma_cube_rl_bridge/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "soma_cube_rl_bridge/msg/detail/rl_observation__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace soma_cube_rl_bridge
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_soma_cube_rl_bridge
cdr_serialize(
  const soma_cube_rl_bridge::msg::RLObservation & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_soma_cube_rl_bridge
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  soma_cube_rl_bridge::msg::RLObservation & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_soma_cube_rl_bridge
get_serialized_size(
  const soma_cube_rl_bridge::msg::RLObservation & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_soma_cube_rl_bridge
max_serialized_size_RLObservation(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace soma_cube_rl_bridge

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_soma_cube_rl_bridge
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, soma_cube_rl_bridge, msg, RLObservation)();

#ifdef __cplusplus
}
#endif

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__RL_OBSERVATION__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from soma_cube_rl_bridge:msg/SafetyState.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__STRUCT_H_
#define SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'reason'
#include "rosidl_runtime_c/string.h"
// Member 'last_event_time'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'joint_limit_violations'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/SafetyState in the package soma_cube_rl_bridge.
typedef struct soma_cube_rl_bridge__msg__SafetyState
{
  std_msgs__msg__Header header;
  bool safe_to_move;
  rosidl_runtime_c__String reason;
  builtin_interfaces__msg__Time last_event_time;
  uint32_t robot_mode;
  uint32_t robot_state;
  bool emergency_stop;
  bool protective_stop;
  rosidl_runtime_c__boolean__Sequence joint_limit_violations;
} soma_cube_rl_bridge__msg__SafetyState;

// Struct for a sequence of soma_cube_rl_bridge__msg__SafetyState.
typedef struct soma_cube_rl_bridge__msg__SafetyState__Sequence
{
  soma_cube_rl_bridge__msg__SafetyState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} soma_cube_rl_bridge__msg__SafetyState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SOMA_CUBE_RL_BRIDGE__MSG__DETAIL__SAFETY_STATE__STRUCT_H_

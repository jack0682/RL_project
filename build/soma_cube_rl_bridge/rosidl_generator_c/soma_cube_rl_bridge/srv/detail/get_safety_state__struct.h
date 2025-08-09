// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from soma_cube_rl_bridge:srv/GetSafetyState.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__STRUCT_H_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/GetSafetyState in the package soma_cube_rl_bridge.
typedef struct soma_cube_rl_bridge__srv__GetSafetyState_Request
{
  uint8_t structure_needs_at_least_one_member;
} soma_cube_rl_bridge__srv__GetSafetyState_Request;

// Struct for a sequence of soma_cube_rl_bridge__srv__GetSafetyState_Request.
typedef struct soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence
{
  soma_cube_rl_bridge__srv__GetSafetyState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} soma_cube_rl_bridge__srv__GetSafetyState_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'reason'
#include "rosidl_runtime_c/string.h"
// Member 'last_event_time'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in srv/GetSafetyState in the package soma_cube_rl_bridge.
typedef struct soma_cube_rl_bridge__srv__GetSafetyState_Response
{
  bool safe_to_move;
  rosidl_runtime_c__String reason;
  builtin_interfaces__msg__Time last_event_time;
} soma_cube_rl_bridge__srv__GetSafetyState_Response;

// Struct for a sequence of soma_cube_rl_bridge__srv__GetSafetyState_Response.
typedef struct soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence
{
  soma_cube_rl_bridge__srv__GetSafetyState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} soma_cube_rl_bridge__srv__GetSafetyState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__GET_SAFETY_STATE__STRUCT_H_

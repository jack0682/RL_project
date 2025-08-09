// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from soma_cube_rl_bridge:srv/RLStep.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__STRUCT_H_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'action'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/RLStep in the package soma_cube_rl_bridge.
typedef struct soma_cube_rl_bridge__srv__RLStep_Request
{
  rosidl_runtime_c__double__Sequence action;
} soma_cube_rl_bridge__srv__RLStep_Request;

// Struct for a sequence of soma_cube_rl_bridge__srv__RLStep_Request.
typedef struct soma_cube_rl_bridge__srv__RLStep_Request__Sequence
{
  soma_cube_rl_bridge__srv__RLStep_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} soma_cube_rl_bridge__srv__RLStep_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'obs'
// already included above
// #include "rosidl_runtime_c/primitives_sequence.h"
// Member 'info'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RLStep in the package soma_cube_rl_bridge.
typedef struct soma_cube_rl_bridge__srv__RLStep_Response
{
  rosidl_runtime_c__double__Sequence obs;
  double reward;
  bool done;
  rosidl_runtime_c__String info;
  bool success;
} soma_cube_rl_bridge__srv__RLStep_Response;

// Struct for a sequence of soma_cube_rl_bridge__srv__RLStep_Response.
typedef struct soma_cube_rl_bridge__srv__RLStep_Response__Sequence
{
  soma_cube_rl_bridge__srv__RLStep_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} soma_cube_rl_bridge__srv__RLStep_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_STEP__STRUCT_H_

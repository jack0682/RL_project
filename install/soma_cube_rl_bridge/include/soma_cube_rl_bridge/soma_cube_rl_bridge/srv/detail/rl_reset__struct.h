// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from soma_cube_rl_bridge:srv/RLReset.idl
// generated code does not contain a copyright notice

#ifndef SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__STRUCT_H_
#define SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/RLReset in the package soma_cube_rl_bridge.
typedef struct soma_cube_rl_bridge__srv__RLReset_Request
{
  int32_t seed;
} soma_cube_rl_bridge__srv__RLReset_Request;

// Struct for a sequence of soma_cube_rl_bridge__srv__RLReset_Request.
typedef struct soma_cube_rl_bridge__srv__RLReset_Request__Sequence
{
  soma_cube_rl_bridge__srv__RLReset_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} soma_cube_rl_bridge__srv__RLReset_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'initial_obs'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/RLReset in the package soma_cube_rl_bridge.
typedef struct soma_cube_rl_bridge__srv__RLReset_Response
{
  rosidl_runtime_c__double__Sequence initial_obs;
  bool success;
  rosidl_runtime_c__String message;
} soma_cube_rl_bridge__srv__RLReset_Response;

// Struct for a sequence of soma_cube_rl_bridge__srv__RLReset_Response.
typedef struct soma_cube_rl_bridge__srv__RLReset_Response__Sequence
{
  soma_cube_rl_bridge__srv__RLReset_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} soma_cube_rl_bridge__srv__RLReset_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SOMA_CUBE_RL_BRIDGE__SRV__DETAIL__RL_RESET__STRUCT_H_
